#ifndef PROFILES_SERIAL_SOCKET_H_
#define PROFILES_SERIAL_SOCKET_H_

#include <ti/drivers/utils/List.h>
#include <icall_ble_api.h>
#include <att.h>

#define SERIAL_SOCKET_SERVICE_UUID 0xC0C0

#define SERIAL_SOCKET_DATA_IN_ID 0
#define SERIAL_SOCKET_DATA_IN_UUID 0xC0C1
#define SERIAL_SOCKET_DATA_IN_LEN 1
#define SERIAL_SOCKET_DATA_OUT_ID 1
#define SERIAL_SOCKET_DATA_OUT_UUID 0xC0C2
#define SERIAL_SOCKET_DATA_OUT_LEN 1

extern const uint8_t serial_socket_uuid[ATT_UUID_SIZE];
extern const uint8_t serial_socket_data_in_uuid[ATT_UUID_SIZE];
extern const uint8_t serial_socket_data_out_uuid[ATT_UUID_SIZE];

typedef struct
{
    List_Elem element;
    uint16_t connection_handle;
    uint16_t offset;
    uint16_t length;
    uint8_t payload[];
} SerialSocketNode;

typedef void (*SerialSocketCharacteristicCallback)(uint16_t value);

typedef void (*SerialSocketIncomingDataCallback)(uint16_t connection_handle,
                                                 uint8_t parameter_id,
                                                 uint16_t length,
                                                 uint8_t *value);

typedef struct
{
    SerialSocketCharacteristicCallback characteristic_callback;
    SerialSocketIncomingDataCallback incoming_data_callback;
} SerialSocketCallbacks;

extern bStatus_t serial_socket_add_service(uint32_t task_id);
extern bStatus_t serial_socket_register_callbacks(
        SerialSocketCallbacks *callbacks);
extern bStatus_t serial_socket_send_data(uint16_t connection_handle, void *data,
                                         uint16_t length);
extern bStatus_t serial_socket_process_stream();
extern void serial_socket_disconnect();
extern void serial_socket_set_limit(uint16_t heap_limit);
extern void* serial_socket_allocate(uint16_t allocation_size);

#endif
