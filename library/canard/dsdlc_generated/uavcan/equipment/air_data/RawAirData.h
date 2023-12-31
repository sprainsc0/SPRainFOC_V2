/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 * Source file: i:\BlackBox\aircraft_h7\module\canard\dsdl\uavcan\equipment\air_data\1027.RawAirData.uavcan
 */

#ifndef __UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA
#define __UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA

#include <stdint.h>
#include "canard_scalar.h"

#ifdef __cplusplus
extern "C"
{
#endif

/******************************* Source text **********************************
# 1027
# Raw Air Data.
#

# Note: unused vars should be assigned NaN

#
# Heater State
#
uint8 FLAG_HEATER_AVAILABLE      = 1
uint8 FLAG_HEATER_WORKING        = 2
uint8 FLAG_HEATER_OVERCURRENT    = 4
uint8 FLAG_HEATER_OPENCIRCUIT    = 8
uint8 flags

#
# Pressure Data
#
float32 static_pressure                 # Pascal
float32 differential_pressure           # Pascal

#
# Temperature Data
#
float16 static_pressure_sensor_temperature          # Kelvin
float16 differential_pressure_sensor_temperature    # Kelvin

float16 static_air_temperature          # Kelvin
                                        # This field contains the raw temperature reading
                                        # from the externally mounted temperature sensor or,
                                        # in absence of one, the raw temperature of the pressure sensor.

float16 pitot_temperature               # Kelvin

float16[<=16] covariance                # order of diagonal elements :
                                        # static_pressure, differential_pressure,
                                        # static_air_temperature, pitot_temperature
                                        # Pascal^2 for pressure variance and covariance
                                        # Kevin^2 for Temperature variance and covariance
                                        # Pascal*Kelvin for pressure/temperature covariance
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.air_data.RawAirData
saturated uint8 flags
saturated float32 static_pressure
saturated float32 differential_pressure
saturated float16 static_pressure_sensor_temperature
saturated float16 differential_pressure_sensor_temperature
saturated float16 static_air_temperature
saturated float16 pitot_temperature
saturated float16[<=16] covariance
******************************************************************************/

#define UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_ID            1027
#define UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_NAME          "uavcan.equipment.air_data.RawAirData"
#define UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_SIGNATURE     (0xC77DF38BA122F5DAULL)

#define UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_MAX_SIZE      ((397 + 7)/8)

// Constants
#define UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_FLAG_HEATER_AVAILABLE            1 // 1
#define UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_FLAG_HEATER_WORKING              2 // 2
#define UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_FLAG_HEATER_OVERCURRENT          4 // 4
#define UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_FLAG_HEATER_OPENCIRCUIT          8 // 8

#define UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_COVARIANCE_MAX_LENGTH                       16

typedef struct
{
    // FieldTypes
    uint8_t    flags;                         // bit len 8
    float      static_pressure;               // float32 Saturate
    float      differential_pressure;         // float32 Saturate
    float      static_pressure_sensor_temperature; // float16 Saturate
    float      differential_pressure_sensor_temperature; // float16 Saturate
    float      static_air_temperature;        // float16 Saturate
    float      pitot_temperature;             // float16 Saturate
    struct
    {
        uint8_t    len;                       // Dynamic array length
        float*     data;                      // Dynamic Array 16bit[16] max items
    } covariance;

} uavcan_equipment_air_data_RawAirData;

static inline
uint32_t uavcan_equipment_air_data_RawAirData_encode(uavcan_equipment_air_data_RawAirData* source, void* msg_buf);

static inline
int32_t uavcan_equipment_air_data_RawAirData_decode(const CanardTransfer* transfer, uint16_t payload_len, uavcan_equipment_air_data_RawAirData* dest, uint8_t** dyn_arr_buf);

static inline
uint32_t uavcan_equipment_air_data_RawAirData_encode_internal(uavcan_equipment_air_data_RawAirData* source, void* msg_buf, uint32_t offset, uint8_t root_item);

static inline
int32_t uavcan_equipment_air_data_RawAirData_decode_internal(const CanardTransfer* transfer, uint16_t payload_len, uavcan_equipment_air_data_RawAirData* dest, uint8_t** dyn_arr_buf, int32_t offset);

/*
 * UAVCAN data structure definition for libcanard.
 *
 * Autogenerated, do not edit.
 *
 * Source file: i:\BlackBox\aircraft_h7\module\canard\dsdl\uavcan\equipment\air_data\1027.RawAirData.uavcan
 */

#ifndef CANARD_INTERNAL_SATURATE
#define CANARD_INTERNAL_SATURATE(x, max) ( ((x) > max) ? max : ( (-(x) > max) ? (-max) : (x) ) );
#endif

#ifndef CANARD_INTERNAL_SATURATE_UNSIGNED
#define CANARD_INTERNAL_SATURATE_UNSIGNED(x, max) ( ((x) > max) ? max : (x) );
#endif

#if defined(__GNUC__)
# define CANARD_MAYBE_UNUSED(x) x __attribute__((unused))
#else
# define CANARD_MAYBE_UNUSED(x) x
#endif

/**
  * @brief uavcan_equipment_air_data_RawAirData_encode_internal
  * @param source : pointer to source data struct
  * @param msg_buf: pointer to msg storage
  * @param offset: bit offset to msg storage
  * @param root_item: for detecting if TAO should be used
  * @retval returns offset
  */
uint32_t uavcan_equipment_air_data_RawAirData_encode_internal(uavcan_equipment_air_data_RawAirData* source,
  void* msg_buf,
  uint32_t offset,
  uint8_t CANARD_MAYBE_UNUSED(root_item))
{
    uint32_t c = 0;
#ifndef CANARD_USE_FLOAT16_CAST
    uint16_t tmp_float = 0;
#else
    CANARD_USE_FLOAT16_CAST tmp_float = 0;
#endif

    canardEncodeScalar(msg_buf, offset, 8, (void*)&source->flags); // 255
    offset += 8;

    canardEncodeScalar(msg_buf, offset, 32, (void*)&source->static_pressure); // 2147483647
    offset += 32;

    canardEncodeScalar(msg_buf, offset, 32, (void*)&source->differential_pressure); // 2147483647
    offset += 32;

    // float16 special handling
#ifndef CANARD_USE_FLOAT16_CAST
    tmp_float = canardConvertNativeFloatToFloat16(source->static_pressure_sensor_temperature);
#else
    tmp_float = (CANARD_USE_FLOAT16_CAST)source->static_pressure_sensor_temperature;
#endif
    canardEncodeScalar(msg_buf, offset, 16, (void*)&tmp_float); // 32767
    offset += 16;

    // float16 special handling
#ifndef CANARD_USE_FLOAT16_CAST
    tmp_float = canardConvertNativeFloatToFloat16(source->differential_pressure_sensor_temperature);
#else
    tmp_float = (CANARD_USE_FLOAT16_CAST)source->differential_pressure_sensor_temperature;
#endif
    canardEncodeScalar(msg_buf, offset, 16, (void*)&tmp_float); // 32767
    offset += 16;

    // float16 special handling
#ifndef CANARD_USE_FLOAT16_CAST
    tmp_float = canardConvertNativeFloatToFloat16(source->static_air_temperature);
#else
    tmp_float = (CANARD_USE_FLOAT16_CAST)source->static_air_temperature;
#endif
    canardEncodeScalar(msg_buf, offset, 16, (void*)&tmp_float); // 32767
    offset += 16;

    // float16 special handling
#ifndef CANARD_USE_FLOAT16_CAST
    tmp_float = canardConvertNativeFloatToFloat16(source->pitot_temperature);
#else
    tmp_float = (CANARD_USE_FLOAT16_CAST)source->pitot_temperature;
#endif
    canardEncodeScalar(msg_buf, offset, 16, (void*)&tmp_float); // 32767
    offset += 16;

    // Dynamic Array (covariance)
    if (! root_item)
    {
        // - Add array length
        canardEncodeScalar(msg_buf, offset, 5, (void*)&source->covariance.len);
        offset += 5;
    }

    // - Add array items
    for (c = 0; c < source->covariance.len; c++)
    {
#ifndef CANARD_USE_FLOAT16_CAST
        uint16_t tmpe_float = canardConvertNativeFloatToFloat16(source->covariance.data[c]);
#else
        CANARD_USE_FLOAT16_CAST tmpe_float = (CANARD_USE_FLOAT16_CAST)source->covariance.data[c];
#endif
        canardEncodeScalar(msg_buf, offset, 16, (void*)&tmpe_float); // 32767
        offset += 16;
    }

    return offset;
}

/**
  * @brief uavcan_equipment_air_data_RawAirData_encode
  * @param source : Pointer to source data struct
  * @param msg_buf: Pointer to msg storage
  * @retval returns message length as bytes
  */
uint32_t uavcan_equipment_air_data_RawAirData_encode(uavcan_equipment_air_data_RawAirData* source, void* msg_buf)
{
    uint32_t offset = 0;

    offset = uavcan_equipment_air_data_RawAirData_encode_internal(source, msg_buf, offset, 0);

    return (offset + 7 ) / 8;
}

/**
  * @brief uavcan_equipment_air_data_RawAirData_decode_internal
  * @param transfer: Pointer to CanardTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_equipment_air_data_RawAirData dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @param offset: Call with 0, bit offset to msg storage
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_equipment_air_data_RawAirData_decode_internal(
  const CanardTransfer* transfer,
  uint16_t CANARD_MAYBE_UNUSED(payload_len),
  uavcan_equipment_air_data_RawAirData* dest,
  uint8_t** CANARD_MAYBE_UNUSED(dyn_arr_buf),
  int32_t offset)
{
    int32_t ret = 0;
    uint32_t c = 0;
#ifndef CANARD_USE_FLOAT16_CAST
    uint16_t tmp_float = 0;
#else
    CANARD_USE_FLOAT16_CAST tmp_float = 0;
#endif

    ret = canardDecodeScalar(transfer, offset, 8, false, (void*)&dest->flags);
    if (ret != 8)
    {
        goto uavcan_equipment_air_data_RawAirData_error_exit;
    }
    offset += 8;

    ret = canardDecodeScalar(transfer, offset, 32, false, (void*)&dest->static_pressure);
    if (ret != 32)
    {
        goto uavcan_equipment_air_data_RawAirData_error_exit;
    }
    offset += 32;

    ret = canardDecodeScalar(transfer, offset, 32, false, (void*)&dest->differential_pressure);
    if (ret != 32)
    {
        goto uavcan_equipment_air_data_RawAirData_error_exit;
    }
    offset += 32;

    // float16 special handling
    ret = canardDecodeScalar(transfer, offset, 16, false, (void*)&tmp_float);

    if (ret != 16)
    {
        goto uavcan_equipment_air_data_RawAirData_error_exit;
    }
#ifndef CANARD_USE_FLOAT16_CAST
    dest->static_pressure_sensor_temperature = canardConvertFloat16ToNativeFloat(tmp_float);
#else
    dest->static_pressure_sensor_temperature = (float)tmp_float;
#endif
    offset += 16;

    // float16 special handling
    ret = canardDecodeScalar(transfer, offset, 16, false, (void*)&tmp_float);

    if (ret != 16)
    {
        goto uavcan_equipment_air_data_RawAirData_error_exit;
    }
#ifndef CANARD_USE_FLOAT16_CAST
    dest->differential_pressure_sensor_temperature = canardConvertFloat16ToNativeFloat(tmp_float);
#else
    dest->differential_pressure_sensor_temperature = (float)tmp_float;
#endif
    offset += 16;

    // float16 special handling
    ret = canardDecodeScalar(transfer, offset, 16, false, (void*)&tmp_float);

    if (ret != 16)
    {
        goto uavcan_equipment_air_data_RawAirData_error_exit;
    }
#ifndef CANARD_USE_FLOAT16_CAST
    dest->static_air_temperature = canardConvertFloat16ToNativeFloat(tmp_float);
#else
    dest->static_air_temperature = (float)tmp_float;
#endif
    offset += 16;

    // float16 special handling
    ret = canardDecodeScalar(transfer, offset, 16, false, (void*)&tmp_float);

    if (ret != 16)
    {
        goto uavcan_equipment_air_data_RawAirData_error_exit;
    }
#ifndef CANARD_USE_FLOAT16_CAST
    dest->pitot_temperature = canardConvertFloat16ToNativeFloat(tmp_float);
#else
    dest->pitot_temperature = (float)tmp_float;
#endif
    offset += 16;

    // Dynamic Array (covariance)
    //  - Last item in struct & Root item & (Array Size > 8 bit), tail array optimization
    if (payload_len)
    {
        //  - Calculate Array length from MSG length
        dest->covariance.len = ((payload_len * 8) - offset ) / 16; // 16 bit array item size
    }
    else
    {
        // - Array length 5 bits
        ret = canardDecodeScalar(transfer,
                                 offset,
                                 5,
                                 false,
                                 (void*)&dest->covariance.len); // 32767
        if (ret != 5)
        {
            goto uavcan_equipment_air_data_RawAirData_error_exit;
        }
        offset += 5;
    }

    //  - Get Array
    if (dyn_arr_buf)
    {
        dest->covariance.data = (float*)*dyn_arr_buf;
    }

    for (c = 0; c < dest->covariance.len; c++)
    {
        if (dyn_arr_buf)
        {
#ifndef CANARD_USE_FLOAT16_CAST
            uint16_t tmpe_float = 0;
#else
            CANARD_USE_FLOAT16_CAST tmpe_float = 0;
#endif
            ret = canardDecodeScalar(transfer, offset, 16, false, (void*)&tmpe_float);
            if (ret != 16)
            {
                goto uavcan_equipment_air_data_RawAirData_error_exit;
            }

#ifndef CANARD_USE_FLOAT16_CAST
            dest->covariance.data[c] = canardConvertFloat16ToNativeFloat(tmpe_float);
#else
            dest->covariance.data[c] = (float)tmpe_float;
#endif
        }
        offset += 16;
    }
    return offset;

uavcan_equipment_air_data_RawAirData_error_exit:
    if (ret < 0)
    {
        return ret;
    }
    else
    {
        return -CANARD_ERROR_INTERNAL;
    }
}

/**
  * @brief uavcan_equipment_air_data_RawAirData_decode
  * @param transfer: Pointer to CanardTransfer transfer
  * @param payload_len: Payload message length
  * @param dest: Pointer to destination struct
  * @param dyn_arr_buf: NULL or Pointer to memory storage to be used for dynamic arrays
  *                     uavcan_equipment_air_data_RawAirData dyn memory will point to dyn_arr_buf memory.
  *                     NULL will ignore dynamic arrays decoding.
  * @retval offset or ERROR value if < 0
  */
int32_t uavcan_equipment_air_data_RawAirData_decode(const CanardTransfer* transfer,
  uint16_t payload_len,
  uavcan_equipment_air_data_RawAirData* dest,
  uint8_t** dyn_arr_buf)
{
    const int32_t offset = 0;
    int32_t ret = 0;

    // Clear the destination struct
    for (uint32_t c = 0; c < sizeof(uavcan_equipment_air_data_RawAirData); c++)
    {
        ((uint8_t*)dest)[c] = 0x00;
    }

    ret = uavcan_equipment_air_data_RawAirData_decode_internal(transfer, payload_len, dest, dyn_arr_buf, offset);

    return ret;
}

#ifdef __cplusplus
} // extern "C"
#endif
#endif // __UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA