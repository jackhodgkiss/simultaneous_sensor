#include "simultaneous_sensor.h"

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#include <ti/display/Display.h>

#include <util.h>
#include <icall.h>
#include <icall_ble_api.h>
#include <devinfoservice.h>

#include <ti_drivers_config.h>
#include <ti_ble_config.h>

#include "serial_socket.h"

Display_Handle display_handle = NULL;

#define TASK_PRIORITY                     1

#ifndef TASK_STACK_SIZE
#define TASK_STACK_SIZE                   1024
#endif

#define ADVERTISEMENT_EVENT 1

#define ICALL_EVENT ICALL_MSG_EVENT_ID
#define QUEUE_EVENT UTIL_QUEUE_EVENT_ID

#define ALL_EVENTS (ICALL_EVENT | QUEUE_EVENT)

#define SIMULTANEOUS_SENSOR_ASSERT(expression) if (!(expression)) HAL_ASSERT_SPINLOCK;

static ICall_EntityID self;
static ICall_SyncHandle synchronization_handle;

static Queue_Struct message_queue;
static Queue_Handle message_queue_handle;

Task_Struct task;

uint8_t task_stack[TASK_STACK_SIZE];

typedef struct
{
    uint8_t event;
    void *data;
} ApplicationEvent;

typedef struct
{
    uint32_t event;
    void *buffer;
} AdvertisementEventData;

static uint8 advertisement_handle;

static GAP_Addr_Modes_t address_mode = DEFAULT_ADDRESS_MODE;

static void initialise(void);
static void task_fn(UArg argument_one, UArg argument_two);
static void process_stack_message(ICall_Hdr *message);
static void process_gap_message(gapEventHdr_t *message);
static void process_application_message(ApplicationEvent *message);
static void advertisement_callback(uint32_t event, void *advertisement_buffer, uintptr_t argument);
static bool process_advertisement_event(AdvertisementEventData *event_data);
static status_t enqueue_message(uint8_t event, void *message_data);

void simultaneous_sensor(void)
{
    Task_Params task_parameters;

    Task_Params_init(&task_parameters);
    task_parameters.stack = task_stack;
    task_parameters.stackSize = TASK_STACK_SIZE;
    task_parameters.priority = TASK_PRIORITY;

    Task_construct(&task, task_fn, &task_parameters, NULL);
}

static void initialise(void)
{
    display_handle = Display_open(Display_Type_UART, NULL);
    ICall_registerApp(&self, &synchronization_handle);
    message_queue_handle = Util_constructQueue(&message_queue);
    GAP_RegisterForMsgs(self);
    GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, self, address_mode, &pRandomAddress);
}

static void task_fn(UArg argument_one, UArg argument_two)
{
    initialise();
    for (;;)
    {
        uint32_t tick_start = Clock_getTicks();
        uint32_t events;
        events = Event_pend(synchronization_handle, Event_Id_NONE, ALL_EVENTS, ICALL_TIMEOUT_FOREVER);
        if(events)
        {
            ICall_EntityID destination;
            ICall_ServiceEnum source;
            ICall_HciExtEvt *message = NULL;
            if(ICall_fetchServiceMsg(&source, &destination, (void **)&message) == ICALL_ERRNO_SUCCESS)
            {
                if((source == ICALL_SERVICE_CLASS_BLE) && (destination == self))
                {
                    process_stack_message((ICall_Hdr *)message);
                }
                if(message)
                {
                    ICall_freeMsg(message);
                }
            }
            if(events & QUEUE_EVENT)
            {
                while(!Queue_empty(message_queue_handle))
                {
                    ApplicationEvent *message = (ApplicationEvent *)Util_dequeueMsg(message_queue_handle);
                    if(message)
                    {
                        process_application_message(message);
                        ICall_free(message);
                    }
                }
            }
        }
    }
}

static void process_stack_message(ICall_Hdr *message)
{
    switch(message->event)
    {
    case GAP_MSG_EVENT:
        process_gap_message((gapEventHdr_t*) message);
        break;
    default:
        break;
    }
}

static void process_gap_message(gapEventHdr_t *message)
{
    switch(message->opcode)
    {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
        bStatus_t status = FAILURE;
        gapDeviceInitDoneEvent_t *packet = (gapDeviceInitDoneEvent_t *)message;
        if(packet->hdr.status == SUCCESS)
        {
            uint8_t system_id[DEVINFO_SYSTEM_ID_LEN];
            system_id[0] = packet->devAddr[0];
            system_id[1] = packet->devAddr[1];
            system_id[2] = packet->devAddr[2];

            system_id[4] = 0x00;
            system_id[3] = 0x00;

            system_id[7] = packet->devAddr[5];
            system_id[6] = packet->devAddr[4];
            system_id[5] = packet->devAddr[3];

            DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, system_id);

            status = GapAdv_create(&advertisement_callback, &advParams1, &advertisement_handle);
            SIMULTANEOUS_SENSOR_ASSERT(status == SUCCESS);

            status = GapAdv_loadByHandle(advertisement_handle, GAP_ADV_DATA_TYPE_ADV, sizeof(advData1), advData1);
            SIMULTANEOUS_SENSOR_ASSERT(status == SUCCESS);

            status = GapAdv_loadByHandle(advertisement_handle, GAP_ADV_DATA_TYPE_SCAN_RSP, sizeof(scanResData1), scanResData1);
            SIMULTANEOUS_SENSOR_ASSERT(status == SUCCESS);

            status = GapAdv_setEventMask(advertisement_handle, GAP_ADV_EVT_MASK_START_AFTER_ENABLE | GAP_ADV_EVT_MASK_END_AFTER_DISABLE | GAP_ADV_EVT_MASK_SET_TERMINATED);
            SIMULTANEOUS_SENSOR_ASSERT(status == SUCCESS);

            status = GapAdv_enable(advertisement_handle, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
            SIMULTANEOUS_SENSOR_ASSERT(status == SUCCESS);
        }
        break;
    }
    default:
        break;
    }
}

static void process_application_message(ApplicationEvent *message)
{
    bool safe_to_deallocate = TRUE;
    switch(message->event)
    {
    case ADVERTISEMENT_EVENT:
        safe_to_deallocate = process_advertisement_event((AdvertisementEventData*)(message->data));
        break;
    default:
        break;
    }

    if(safe_to_deallocate)
    {
        if(message->data)
        {
            ICall_free(message->data);
        }
    }
}

static void advertisement_callback(uint32_t event, void *advertisement_buffer, uintptr_t argument)
{
    AdvertisementEventData *data = ICall_malloc(sizeof(AdvertisementEventData));
    if(data)
    {
        data->event = event;
        data->buffer = advertisement_buffer;
        if(enqueue_message(ADVERTISEMENT_EVENT, data) != SUCCESS)
        {
            ICall_free(data);
        }
    }
}

static bool process_advertisement_event(AdvertisementEventData *event_data)
{
    bool safe_to_deallocate = TRUE;
    switch(event_data->event)
    {
    case GAP_EVT_ADV_START_AFTER_ENABLE:
        break;
    case GAP_EVT_INSUFFICIENT_MEMORY:
        safe_to_deallocate = FALSE;
        break;
    default:
        break;
    }
    return(safe_to_deallocate);
}

static status_t enqueue_message(uint8_t event, void *message_data)
{
    uint8_t success;
    ApplicationEvent *message = ICall_malloc(sizeof(ApplicationEvent));
    if(message)
    {
        message->event = event;
        message->data = message_data;
        success = Util_enqueueMsg(message_queue_handle, synchronization_handle, (uint8_t*)message);
        return (success) ? SUCCESS : FAILURE;
    }
    return(bleMemAllocError);
}
