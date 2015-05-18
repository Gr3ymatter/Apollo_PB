/**
* This file is autogenerated by nRFgo Studio 1.17.0.3211
*/

#ifndef SETUP_MESSAGES_H__
#define SETUP_MESSAGES_H__

#include "hal_platform.h"
#include "aci.h"


#define SETUP_ID 0
#define SETUP_FORMAT 3 /** nRF8001 D */
#define ACI_DYNAMIC_DATA_SIZE 227

/* Service: Gap - Characteristic: Device name - Pipe: SET */
#define PIPE_GAP_DEVICE_NAME_SET          1
#define PIPE_GAP_DEVICE_NAME_SET_MAX_SIZE 6

/* Service: Device Information - Characteristic: Hardware Revision String - Pipe: SET */
#define PIPE_DEVICE_INFORMATION_HARDWARE_REVISION_STRING_SET          2
#define PIPE_DEVICE_INFORMATION_HARDWARE_REVISION_STRING_SET_MAX_SIZE 2

/* Service: Device Information - Characteristic: Firmware Revision String - Pipe: SET */
#define PIPE_DEVICE_INFORMATION_FIRMWARE_REVISION_STRING_SET          3
#define PIPE_DEVICE_INFORMATION_FIRMWARE_REVISION_STRING_SET_MAX_SIZE 4

/* Service: User Data Service - Characteristic: Pill Count - Pipe: TX */
#define PIPE_USER_DATA_SERVICE_PILL_COUNT_TX          4
#define PIPE_USER_DATA_SERVICE_PILL_COUNT_TX_MAX_SIZE 4

/* Service: User Data Service - Characteristic: Pill Count - Pipe: RX */
#define PIPE_USER_DATA_SERVICE_PILL_COUNT_RX          5
#define PIPE_USER_DATA_SERVICE_PILL_COUNT_RX_MAX_SIZE 4

/* Service: User Data Service - Characteristic: Pill Count - Pipe: SET */
#define PIPE_USER_DATA_SERVICE_PILL_COUNT_SET_PILL_COUNT          6
#define PIPE_USER_DATA_SERVICE_PILL_COUNT_SET_PILL_COUNT_MAX_SIZE 4

/* Service: User Data Service - Characteristic: Pill Name - Pipe: RX */
#define PIPE_USER_DATA_SERVICE_PILL_NAME_RX          7
#define PIPE_USER_DATA_SERVICE_PILL_NAME_RX_MAX_SIZE 15

/* Service: User Data Service - Characteristic: Pill Name - Pipe: SET */
#define PIPE_USER_DATA_SERVICE_PILL_NAME_SET_PILL_NAME          8
#define PIPE_USER_DATA_SERVICE_PILL_NAME_SET_PILL_NAME_MAX_SIZE 15

/* Service: User Data Service - Characteristic: Pill Dosage - Pipe: RX */
#define PIPE_USER_DATA_SERVICE_PILL_DOSAGE_RX          9
#define PIPE_USER_DATA_SERVICE_PILL_DOSAGE_RX_MAX_SIZE 4

/* Service: User Data Service - Characteristic: Pill Dosage - Pipe: SET */
#define PIPE_USER_DATA_SERVICE_PILL_DOSAGE_SET_DOSAGE          10
#define PIPE_USER_DATA_SERVICE_PILL_DOSAGE_SET_DOSAGE_MAX_SIZE 4

/* Service: User Data Service - Characteristic: Pill Time - Pipe: TX */
#define PIPE_USER_DATA_SERVICE_PILL_TIME_TX          11
#define PIPE_USER_DATA_SERVICE_PILL_TIME_TX_MAX_SIZE 10

/* Service: User Data Service - Characteristic: Pill Time - Pipe: RX */
#define PIPE_USER_DATA_SERVICE_PILL_TIME_RX          12
#define PIPE_USER_DATA_SERVICE_PILL_TIME_RX_MAX_SIZE 10

/* Service: User Data Service - Characteristic: Pill Time - Pipe: SET */
#define PIPE_USER_DATA_SERVICE_PILL_TIME_SET_TIME          13
#define PIPE_USER_DATA_SERVICE_PILL_TIME_SET_TIME_MAX_SIZE 10


#define NUMBER_OF_PIPES 13

#define SERVICES_PIPE_TYPE_MAPPING_CONTENT {\
  {ACI_STORE_LOCAL, ACI_SET},   \
  {ACI_STORE_LOCAL, ACI_SET},   \
  {ACI_STORE_LOCAL, ACI_SET},   \
  {ACI_STORE_LOCAL, ACI_TX},   \
  {ACI_STORE_LOCAL, ACI_RX},   \
  {ACI_STORE_LOCAL, ACI_SET},   \
  {ACI_STORE_LOCAL, ACI_RX},   \
  {ACI_STORE_LOCAL, ACI_SET},   \
  {ACI_STORE_LOCAL, ACI_RX},   \
  {ACI_STORE_LOCAL, ACI_SET},   \
  {ACI_STORE_LOCAL, ACI_TX},   \
  {ACI_STORE_LOCAL, ACI_RX},   \
  {ACI_STORE_LOCAL, ACI_SET},   \
}

#define GAP_PPCP_MAX_CONN_INT 0x18 /**< Maximum connection interval as a multiple of 1.25 msec , 0xFFFF means no specific value requested */
#define GAP_PPCP_MIN_CONN_INT  0x10 /**< Minimum connection interval as a multiple of 1.25 msec , 0xFFFF means no specific value requested */
#define GAP_PPCP_SLAVE_LATENCY 6
#define GAP_PPCP_CONN_TIMEOUT 0x64 /** Connection Supervision timeout multiplier as a multiple of 10msec, 0xFFFF means no specific value requested */

#define NB_SETUP_MESSAGES 31
#define SETUP_MESSAGES_CONTENT {\
    {0x00,\
        {\
            0x07,0x06,0x00,0x00,0x03,0x02,0x42,0x07,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x10,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x0b,0x00,0x0d,0x01,0x01,0x00,0x00,0x06,0x00,0x01,\
            0x81,0x0a,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x10,0x1c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
            0x00,0x00,0x40,0x12,0x00,0x00,0x00,0x10,0x03,0x90,0x00,0x64,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x10,0x38,0x02,0xff,0x02,0x58,0x0a,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
            0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x05,0x06,0x10,0x54,0x01,0x01,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x00,0x04,0x04,0x02,0x02,0x00,0x01,0x28,0x00,0x01,0x00,0x18,0x04,0x04,0x05,0x05,0x00,\
            0x02,0x28,0x03,0x01,0x0e,0x03,0x00,0x00,0x2a,0x04,0x34,0x06,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x1c,0x06,0x00,0x03,0x2a,0x00,0x01,0x41,0x70,0x6f,0x6c,0x6c,0x6f,0x04,0x04,0x05,0x05,\
            0x00,0x04,0x28,0x03,0x01,0x02,0x05,0x00,0x01,0x2a,0x06,0x04,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x38,0x03,0x02,0x00,0x05,0x2a,0x01,0x01,0xc0,0x00,0x04,0x04,0x05,0x05,0x00,0x06,0x28,\
            0x03,0x01,0x02,0x07,0x00,0x04,0x2a,0x06,0x04,0x09,0x08,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x54,0x07,0x2a,0x04,0x01,0x10,0x00,0x18,0x00,0x06,0x00,0x64,0x00,0x04,0x04,0x02,0x02,\
            0x00,0x08,0x28,0x00,0x01,0x01,0x18,0x04,0x04,0x02,0x02,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x70,0x09,0x28,0x00,0x01,0x0a,0x18,0x04,0x04,0x05,0x05,0x00,0x0a,0x28,0x03,0x01,0x02,\
            0x0b,0x00,0x27,0x2a,0x06,0x0c,0x03,0x02,0x00,0x0b,0x2a,0x27,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x8c,0x01,0x0a,0x00,0x04,0x04,0x05,0x05,0x00,0x0c,0x28,0x03,0x01,0x02,0x0d,0x00,0x26,\
            0x2a,0x06,0x0c,0x05,0x04,0x00,0x0d,0x2a,0x26,0x01,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0xa8,0x00,0x00,0x04,0x04,0x05,0x05,0x00,0x0e,0x28,0x03,0x01,0x02,0x0f,0x00,0x29,0x2a,\
            0x06,0x0c,0x15,0x14,0x00,0x0f,0x2a,0x29,0x01,0x4e,0x6f,0x72,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0xc4,0x64,0x69,0x63,0x53,0x65,0x6d,0x69,0x63,0x6f,0x6e,0x64,0x75,0x63,0x74,0x6f,0x72,\
            0x00,0x06,0x04,0x08,0x07,0x00,0x10,0x29,0x04,0x01,0x19,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0xe0,0x00,0x00,0x01,0x00,0x00,0x04,0x04,0x05,0x05,0x00,0x11,0x28,0x03,0x01,0x02,0x12,\
            0x00,0x4a,0x2a,0x06,0x0c,0x05,0x04,0x00,0x12,0x2a,0x4a,0x01,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0xfc,0x11,0x01,0x00,0x01,0x04,0x04,0x02,0x02,0x00,0x13,0x28,0x00,0x01,0x1c,0x18,0x04,\
            0x04,0x05,0x05,0x00,0x14,0x28,0x03,0x01,0x16,0x15,0x00,0xfc,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0x18,0xfc,0x56,0x3c,0x05,0x04,0x00,0x15,0xfc,0xfc,0x01,0x00,0x00,0x00,0x00,0x46,0x34,\
            0x03,0x02,0x00,0x16,0x29,0x02,0x01,0x00,0x00,0x06,0x0c,0x0b,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0x34,0x0a,0x00,0x17,0x29,0x01,0x01,0x50,0x69,0x6c,0x6c,0x5f,0x43,0x6f,0x75,0x6e,0x74,\
            0x04,0x04,0x05,0x05,0x00,0x18,0x28,0x03,0x01,0x06,0x19,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0x50,0x34,0x12,0x46,0x3c,0x10,0x0f,0x00,0x19,0x12,0x34,0x01,0x00,0x00,0x00,0x00,0x00,\
            0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x0c,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0x6c,0x0a,0x09,0x00,0x1a,0x29,0x01,0x01,0x50,0x69,0x6c,0x6c,0x20,0x4e,0x61,0x6d,0x65,\
            0x04,0x04,0x05,0x05,0x00,0x1b,0x28,0x03,0x01,0x06,0x1c,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0x88,0x35,0x12,0x46,0x3c,0x05,0x04,0x00,0x1c,0x12,0x35,0x01,0x00,0x00,0x00,0x00,0x06,\
            0x0c,0x07,0x06,0x00,0x1d,0x29,0x01,0x01,0x44,0x6f,0x73,0x61,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0xa4,0x67,0x65,0x04,0x04,0x05,0x05,0x00,0x1e,0x28,0x03,0x01,0x16,0x1f,0x00,0x36,0x12,\
            0x56,0x3c,0x0b,0x0a,0x00,0x1f,0x12,0x36,0x01,0x00,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x46,0x34,0x03,0x02,0x00,0x20,0x29,0x02,0x01,\
            0x00,0x00,0x06,0x0c,0x05,0x04,0x00,0x21,0x29,0x01,0x01,0x54,\
        },\
    },\
    {0x00,\
        {\
            0x07,0x06,0x21,0xdc,0x69,0x6d,0x65,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x40,0x00,0x2a,0x00,0x01,0x00,0x80,0x04,0x00,0x03,0x00,0x00,0x2a,0x27,0x01,0x00,0x80,0x04,\
            0x00,0x0b,0x00,0x00,0x2a,0x26,0x01,0x00,0x80,0x04,0x00,0x0d,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x40,0x1c,0x00,0x00,0xfc,0xfc,0x01,0x00,0x0a,0x04,0x00,0x15,0x00,0x16,0x29,0x01,0x01,0x00,\
            0x80,0x05,0x00,0x17,0x00,0x00,0x12,0x34,0x01,0x00,0x08,0x04,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x40,0x38,0x00,0x19,0x00,0x00,0x29,0x01,0x01,0x00,0x80,0x05,0x00,0x1a,0x00,0x00,0x12,0x35,\
            0x01,0x00,0x08,0x04,0x00,0x1c,0x00,0x00,0x29,0x01,0x01,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1d,0x06,0x40,0x54,0x80,0x05,0x00,0x1d,0x00,0x00,0x12,0x36,0x01,0x00,0x0a,0x04,0x00,0x1f,0x00,0x20,\
            0x29,0x01,0x01,0x00,0x80,0x05,0x00,0x21,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
            0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x08,0x06,0x60,0x1c,0x00,0x00,0x00,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x19,0x06,0x70,0x00,0x19,0x02,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
            0x00,0x00,0x00,0x00,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x06,0x06,0xf0,0x00,0x03,0x3c,0x14,\
        },\
    },\
}

#endif
