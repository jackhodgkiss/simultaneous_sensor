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
#define CONNECTION_EVENT 2

#define ICALL_EVENT ICALL_MSG_EVENT_ID
#define QUEUE_EVENT UTIL_QUEUE_EVENT_ID

#define ALL_EVENTS (ICALL_EVENT | QUEUE_EVENT)

#define MIN_HEAP_HEADROOM 4000

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

typedef enum
{
    NOT_REGISTER = 0,
    FOR_ATT_RSP = 1,
    FOR_STREAM = 2
} ConnectionEventReason;

static uint8 legacy_advertisement_handle;
static uint8 long_range_advertisement_handle;

static GAP_Addr_Modes_t address_mode = DEFAULT_ADDRESS_MODE;

typedef struct
{
    uint16_t connection_handle;
} ConnectionRecord;

static ConnectionRecord connections[MAX_NUM_BLE_CONNS];

static void initialise(void);
static void task_fn(UArg argument_one, UArg argument_two);
static void process_stack_message(ICall_Hdr *message);
static void process_gap_message(gapEventHdr_t *message);
static void process_application_message(ApplicationEvent *message);
static void advertisement_callback(uint32_t event, void *advertisement_buffer, uintptr_t argument);
static void process_advertisement_event(AdvertisementEventData *event_data);
static bStatus_t enqueue_message(uint8_t event, void *message_data);
static uint8_t add_connection(uint16_t connection_handle);
static uint8_t remove_connection(uint16_t connection_handle);
static bStatus_t clear_connections(uint16_t connection_handle);
static uint8_t get_connection_index(uint16_t connection_handle);
static void characteristic_callback(uint16_t value);
static void incoming_data_callback(uint16_t connection_handle, uint8_t parameter_id, uint16_t length, uint8_t *value);
static bStatus_t register_connection_event(ConnectionEventReason connection_event_reason);

static SerialSocketCallbacks serial_socket_callbacks =
{
 .characteristic_callback = characteristic_callback,
 .incoming_data_callback = incoming_data_callback,
};


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
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);
    GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, GAP_UPDATE_REQ_ACCEPT_ALL);
    GGS_AddService(GATT_ALL_SERVICES);
    GATTServApp_AddService(GATT_ALL_SERVICES);
    DevInfo_AddService();
    serial_socket_add_service(GATT_ALL_SERVICES);
    serial_socket_register_callbacks(&serial_socket_callbacks);
    serial_socket_set_limit(MIN_HEAP_HEADROOM);
    register_connection_event(FOR_STREAM);
    GAP_RegisterForMsgs(self);
    GATT_RegisterForMsgs(self);
    HCI_LE_WriteSuggestedDefaultDataLenCmd(251, 2120);
    GATT_InitClient();
    GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, self, address_mode, &pRandomAddress);
    clear_connections(LINKDB_CONNHANDLE_ALL);
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

            status = GapAdv_create(&advertisement_callback, &advParams1, &legacy_advertisement_handle);
            SIMULTANEOUS_SENSOR_ASSERT(status == SUCCESS);

            status = GapAdv_loadByHandle(legacy_advertisement_handle, GAP_ADV_DATA_TYPE_ADV, sizeof(advData1), advData1);
            SIMULTANEOUS_SENSOR_ASSERT(status == SUCCESS);

            status = GapAdv_loadByHandle(legacy_advertisement_handle, GAP_ADV_DATA_TYPE_SCAN_RSP, sizeof(scanResData1), scanResData1);
            SIMULTANEOUS_SENSOR_ASSERT(status == SUCCESS);

            status = GapAdv_setEventMask(legacy_advertisement_handle, GAP_ADV_EVT_MASK_START_AFTER_ENABLE | GAP_ADV_EVT_MASK_END_AFTER_DISABLE | GAP_ADV_EVT_MASK_SET_TERMINATED);
            SIMULTANEOUS_SENSOR_ASSERT(status == SUCCESS);

            status = GapAdv_enable(legacy_advertisement_handle, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
            SIMULTANEOUS_SENSOR_ASSERT(status == SUCCESS);

            status = GapAdv_create(&advertisement_callback, &advParams2, &long_range_advertisement_handle);
            SIMULTANEOUS_SENSOR_ASSERT(status == SUCCESS);

            status = GapAdv_loadByHandle(long_range_advertisement_handle, GAP_ADV_DATA_TYPE_ADV, sizeof(advData2), advData2);
            SIMULTANEOUS_SENSOR_ASSERT(status == SUCCESS);

            status = GapAdv_setEventMask(long_range_advertisement_handle, GAP_ADV_EVT_MASK_START_AFTER_ENABLE | GAP_ADV_EVT_MASK_END_AFTER_DISABLE | GAP_ADV_EVT_MASK_SET_TERMINATED);
            SIMULTANEOUS_SENSOR_ASSERT(status == SUCCESS);

            status = GapAdv_enable(long_range_advertisement_handle, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
            SIMULTANEOUS_SENSOR_ASSERT(status == SUCCESS);
        }
        break;
    }
    case GAP_LINK_ESTABLISHED_EVENT:
    {
        gapEstLinkReqEvent_t *packet = (gapEstLinkReqEvent_t *)message;
        add_connection(packet->connectionHandle);
        GapAdv_enable(legacy_advertisement_handle, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        GapAdv_enable(long_range_advertisement_handle, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        break;
    }
    case GAP_LINK_TERMINATED_EVENT:
    {
        gapTerminateLinkEvent_t *packet = (gapTerminateLinkEvent_t *)message;
        remove_connection(packet->connectionHandle);
        GapAdv_enable(legacy_advertisement_handle, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        GapAdv_enable(long_range_advertisement_handle, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
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
        process_advertisement_event((AdvertisementEventData*)(message->data));
        break;
    case CONNECTION_EVENT:
        break;
    default:
        break;
    }

    if((safe_to_deallocate == TRUE) && (message->data != NULL))
    {
        ICall_free(message->data);
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

static void process_advertisement_event(AdvertisementEventData *event_data)
{
    switch(event_data->event)
    {
    case GAP_EVT_ADV_START_AFTER_ENABLE:
        Display_printf(display_handle, 0, 0, "Advertisement Set %d Enabled", *(uint8_t *)(event_data->buffer));
        break;
    case GAP_EVT_ADV_END_AFTER_DISABLE:
        Display_printf(display_handle, 0, 0, "Advertisement Set %d Disabled", *(uint8_t *)(event_data->buffer));
    default:
        break;
    }
    if(event_data->event != GAP_EVT_INSUFFICIENT_MEMORY)
    {
        ICall_free(event_data->buffer);
    }
}

static bStatus_t enqueue_message(uint8_t event, void *message_data)
{
    bStatus_t success;
    ApplicationEvent *message = ICall_malloc(sizeof(ApplicationEvent));
    if(message)
    {
        message->event = event;
        message->data = message_data;
        success = Util_enqueueMsg(message_queue_handle, synchronization_handle, (uint8_t*)message);
        return (success) ? SUCCESS : FAILURE;
    }
    return bleMemAllocError;
}

static bStatus_t add_connection(uint16_t connection_handle)
{
    bStatus_t status = bleNoResources;
    for(uint8_t index = 0; index < MAX_NUM_BLE_CONNS; index++)
    {
        if(connections[index].connection_handle == LINKDB_CONNHANDLE_INVALID)
        {
            connections[index].connection_handle = connection_handle;
            break;
        }
    }
    return status;
}

static uint8_t remove_connection(uint16_t connection_handle)
{
    uint8_t connection_index = get_connection_index(connection_handle);

    if(connection_index != MAX_NUM_BLE_CONNS)
    {
        connections[connection_index].connection_handle = LINKDB_CONNHANDLE_INVALID;
    }
    return connection_index;
}

static bStatus_t clear_connections(uint16_t connection_handle)
{
    if(connection_handle == LINKDB_CONNHANDLE_ALL)
    {
        for(int index = 0; index < MAX_NUM_BLE_CONNS; index++)
        {
            connections[index].connection_handle = LINKDB_CONNHANDLE_INVALID;
        }
    }
    return SUCCESS;
}

static uint8_t get_connection_index(uint16_t connection_handle)
{
    for(uint8_t index = 0; index < MAX_NUM_BLE_CONNS; index++)
    {
        if(connections[index].connection_handle == connection_handle)
        {
            return index;
        }
    }
    return MAX_NUM_BLE_CONNS;
}

static void characteristic_callback(uint16_t value)
{

}

static void incoming_data_callback(uint16_t connection_handle, uint8_t parameter_id, uint16_t length, uint8_t *value)
{
    Display_printf(display_handle, 0, 0, "%s", value);
}

static bStatus_t register_connection_event(ConnectionEventReason connection_event_reason)
{
    return SUCCESS;
}
