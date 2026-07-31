#ifndef GW_GRAYSCALE_SENSOR_H
#define GW_GRAYSCALE_SENSOR_H

#define GW_GRAY_ADDR_DEFAULT  0x4CU
#define GW_GRAY_PING          0xAAU
#define GW_GRAY_PING_OK       0x66U
#define GW_GRAY_DIGITAL_MODE  0xDDU

#define GW_GRAY_GET_BIT(value, channel) \
    (((value) >> ((channel) - 1U)) & 0x01U)

#endif
