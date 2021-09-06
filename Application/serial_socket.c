#include "serial_socket.h"

#include <icall.h>
#include <icall_ble_api.h>

#define SERIAL_SOCKET_NOTIFICATION_HEADER_SIZE (ATT_OPCODE_SIZE + 2)

const uint8_t serial_socket_uuid[ATT_UUID_SIZE] =
{
 TI_BASE_UUID_128(SERIAL_SOCKET_SERVICE_UUID)
};

const uint8_t serial_socket_data_in_uuid[ATT_UUID_SIZE] =
{
 TI_BASE_UUID_128(SERIAL_SOCKET_DATA_IN_UUID)
};

const uint8_t serial_socket_data_out_uuid[ATT_UUID_SIZE] =
{
 TI_BASE_UUID_128(SERIAL_SOCKET_DATA_OUT_UUID)
};

static SerialSocketCallbacks *application_callbacks = NULL;

static List_List outgoing_queue;

static uint16_t heap_headroom = 0;

static const gattAttrType_t serial_socket_declaration = { ATT_UUID_SIZE, serial_socket_uuid };

static uint8_t serial_socket_data_in_properties = GATT_PROP_WRITE;

static uint8_t serial_socket_data_in_value[SERIAL_SOCKET_DATA_IN_LEN] = { 0 };

static uint8_t serial_socket_data_out_properties = GATT_PROP_NOTIFY;

static uint8_t serial_socket_data_out_value[SERIAL_SOCKET_DATA_OUT_LEN] = { 0 };

static gattCharCfg_t *serial_socket_data_out_config;

static gattAttribute_t serial_socket_attribute_table[] =
{
 {
  { ATT_BT_UUID_SIZE, primaryServiceUUID },
  GATT_PERMIT_READ,
  0,
  (uint8_t *)&serial_socket_declaration
 },
 {
  { ATT_BT_UUID_SIZE, characterUUID },
  GATT_PERMIT_READ,
  0,
  &serial_socket_data_in_properties
 },
 {
  { ATT_UUID_SIZE, serial_socket_data_in_uuid },
  GATT_PERMIT_WRITE,
  0,
  serial_socket_data_in_value
 },
 {
  { ATT_BT_UUID_SIZE, characterUUID },
  GATT_PERMIT_READ,
  0,
  &serial_socket_data_out_properties
 },
 {
  { ATT_UUID_SIZE, serial_socket_data_out_uuid },
  0,
  0,
  serial_socket_data_out_value
 },
 {
  { ATT_BT_UUID_SIZE, clientCharCfgUUID },
  GATT_PERMIT_READ | GATT_PERMIT_WRITE,
  0,
  (uint8*)&serial_socket_data_out_config
 },
};

static bStatus_t serial_socket_read_attribute_callback(uint16_t connection_handle, gattAttribute_t *attribute,
                                                       uint8_t *value, uint16_t *length, uint16_t offset, uint16_t max_length, uint8_t method);
static bStatus_t serial_socket_write_attribute_callback(uint16_t connection_handle, gattAttribute_t *attribute,
                                                        uint8_t *value, uint16_t length, uint16_t offset, uint8_t method);
static bStatus_t serial_socket_transmit(SerialSocketNode *node);
static bStatus_t serial_socket_queue_data(SerialSocketNode *node);
static void serial_socket_clear_queue();

const gattServiceCBs_t serial_socket_callbacks =
{
 serial_socket_read_attribute_callback,
 serial_socket_write_attribute_callback,
 NULL
};

static bStatus_t serial_socket_read_attribute_callback(uint16_t connection_handle, gattAttribute_t *attribute,
                                                       uint8_t *value, uint16_t *length, uint16_t offset, uint16_t max_length, uint8_t method)
{
    *length = 0;
    return ATT_ERR_ATTR_NOT_FOUND;
}

static bStatus_t serial_socket_write_attribute_callback(uint16_t connection_handle, gattAttribute_t *attribute,
                                                        uint8_t *value, uint16_t length, uint16_t offset, uint8_t method)
{
    bStatus_t status = SUCCESS;
    uint8_t parameter_id = 0xFF;

    if(!memcmp(attribute->type.uuid, clientCharCfgUUID, attribute->type.len))
    {
        status = GATTServApp_ProcessCCCWriteReq(connection_handle, attribute, value, length, offset, GATT_CLIENT_CFG_NOTIFY);
        if(application_callbacks && application_callbacks->characteristic_callback)
        {
            uint16_t val = value[0];
            application_callbacks->characteristic_callback(val);
        }
    }
    else if(!memcmp(attribute->type.uuid, serial_socket_data_in_uuid, attribute->type.len))
    {
        if(length > 0)
        {
            if(application_callbacks && application_callbacks->incoming_data_callback)
            {
                application_callbacks->incoming_data_callback(connection_handle, parameter_id, length, value);
            }
        }
    }
    else
    {
        status = ATT_ERR_ATTR_NOT_FOUND;
    }
    return status;
}

static bStatus_t serial_socket_transmit(SerialSocketNode *node)
{
    bStatus_t status = SUCCESS;
    attHandleValueNoti_t notification_value;
    linkDBInfo_t connection_information;

    status = linkDB_GetInfo(node->connection_handle, &connection_information);

    if((status == SUCCESS) && (node != NULL))
    {
        uint16_t allocation_length = (node->length - node->offset);
        if(allocation_length > (connection_information.MTU - SERIAL_SOCKET_NOTIFICATION_HEADER_SIZE))
        {
            allocation_length = connection_information.MTU - SERIAL_SOCKET_NOTIFICATION_HEADER_SIZE;
        }
        notification_value.len = 0;
        notification_value.pValue = (uint8 *)GATT_bm_alloc(node->connection_handle, ATT_HANDLE_VALUE_NOTI, allocation_length, &notification_value.len);
        if(notification_value.pValue)
        {
            memcpy(notification_value.pValue, (void *)((uint8_t *) node->payload + node->offset), notification_value.len);
            notification_value.handle = serial_socket_attribute_table[4].handle;
            status = GATT_Notification(node->connection_handle, &notification_value, false);
        }
        if(status != SUCCESS)
        {
            GATT_bm_free((gattMsg_t *)&notification_value, ATT_HANDLE_VALUE_NOTI);
        }
        else
        {
            node->offset += notification_value.len;
        }
    }
    else
    {
        status = bleMemAllocError;
    }
    return status;
}

static bStatus_t serial_socket_queue_data(SerialSocketNode *node)
{
    bStatus_t status = SUCCESS;
    gattCharCfg_t *item = NULL;

    if(node != NULL)
    {
        for(int index = 0; index < linkDBNumConns; index++)
        {
            if(serial_socket_data_out_config[index].connHandle == node->connection_handle)
            {
                item = &(serial_socket_data_out_config[index]);
                break;
            }
        }

        if((item != NULL) && (item->connHandle != LINKDB_CONNHANDLE_INVALID)
                && (item->value != GATT_CFG_NO_OPERATION) && (item->value & GATT_CLIENT_CFG_NOTIFY))
        {
            List_put(&outgoing_queue, (List_Elem *) node);
        }
        else
        {
            status = FAILURE;
        }
    }
    else
    {
        status = INVALIDPARAMETER;
    }
    return status;
}

static void serial_socket_clear_queue()
{
    while(!List_empty(&outgoing_queue))
    {
        SerialSocketNode *node = (SerialSocketNode *) List_get(&outgoing_queue);
        ICall_free(node);
    }
}

extern bStatus_t serial_socket_add_service(uint32_t task_id)
{
    uint8_t status;
    serial_socket_data_out_config = (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t)*linkDBNumConns);
    if(serial_socket_data_out_config == NULL)
    {
        return(bleMemAllocError);
    }
    GATTServApp_InitCharCfg(LINKDB_CONNHANDLE_INVALID, serial_socket_data_out_config);
    status = GATTServApp_RegisterService(serial_socket_attribute_table, GATT_NUM_ATTRS(serial_socket_attribute_table), GATT_MAX_ENCRYPT_KEY_SIZE, &serial_socket_callbacks);
    if(status == SUCCESS)
    {
        List_clearList(&outgoing_queue);
    }
    return status;
}

extern bStatus_t serial_socket_register_callbacks(SerialSocketCallbacks *callbacks)
{
    bStatus_t status = SUCCESS;
    if(callbacks)
    {
        application_callbacks = callbacks;
    }
    else
    {
        status = bleAlreadyInRequestedMode;
    }
    return status;
}

extern bStatus_t serial_socket_send_data(uint16_t connection_handle, void *data, uint16_t length)
{
    bStatus_t status = bleMemAllocError;
    SerialSocketNode* node;

    node = (SerialSocketNode*)serial_socket_allocate(sizeof(SerialSocketNode) + length);
    if(node != NULL)
    {
        node->connection_handle = connection_handle;
        node->offset = 0;
        node->length = length;
        memcpy(node->payload, data, length);
        status = serial_socket_queue_data(node);
        if(status == SUCCESS)
        {
            status = serial_socket_process_stream();
        }
        else if(status == FAILURE)
        {
            ICall_free(node);
        }
    }
    return status;
}

extern bStatus_t serial_socket_process_stream()
{
    bStatus_t status = SUCCESS;
    SerialSocketNode *node = (SerialSocketNode*) List_get(&outgoing_queue);

    while((status == SUCCESS) && (node != NULL))
    {
        status = serial_socket_transmit(node);
        if((node->length - node->offset) == 0)
        {
            ICall_free(node);
            node = (SerialSocketNode*)List_get(&outgoing_queue);
        }
        if(status != SUCCESS)
        {
            List_putHead(&outgoing_queue, (List_Elem*)node);
        }
    }
    return status;
}

extern void serial_socket_disconnect()
{
    serial_socket_clear_queue();
}

extern void serial_socket_set_limit(uint16_t heap_limit)
{
    heap_headroom = heap_limit;
}

extern void* serial_socket_allocate(uint16_t allocation_size)
{
    void *allocated_buffer = NULL;
    ICall_heapStats_t heap_statistics;
    ICall_CSState key;

    key = ICall_enterCriticalSection();
    ICall_getHeapStats(&heap_statistics);
    if(((uint16_t) allocation_size) < ((int32_t)(heap_statistics.totalFreeSize - heap_headroom)))
    {
        allocated_buffer = ICall_malloc(allocation_size);
    }
    ICall_leaveCriticalSection(key);
    return allocated_buffer;
}
