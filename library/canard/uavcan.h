#ifndef UAVCAN_H_INCLUDED
#define UAVCAN_H_INCLUDED

#include "canard.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CANLINK_ID_ALL            0u
#define CANLINK_ID_FCU            1u
#define CANLINK_ID_PMU            2u
#define CANLINK_ID_GPS1           4u
#define CANLINK_ID_GPS2           5u
#define CANLINK_ID_MAG1           8u
#define CANLINK_ID_MAG2           9u
#define CANLINK_ID_ROS            12u
#define CANLINK_ID_OF             16u
#define CANLINK_ID_RNG            32u

#define CANLINK_ID_FOCR           33u
#define CANLINK_ID_FOCP           34u
#define CANLINK_ID_FOCY           35u
#define CANLINK_ID_GCU            40u


CanardInstance *canins(void);

int uavcan_init(void);

uint32_t compare(uint32_t a, uint32_t b);

uint32_t caculate_filters(uint32_t *id, uint8_t len);

uint32_t take_message_id(const CanardPortID subject_id, const CanardNodeID src_node_id, const CanardPriority prio);

uint32_t take_service_id(const CanardPortID service_id,
                                const bool request_not_response,
                                const CanardNodeID src_node_id,
                                const CanardNodeID dst_node_id,
                                const CanardPriority prio);

#ifdef __cplusplus
}
#endif
#endif  // CANARD_DSDL_H_INCLUDED
