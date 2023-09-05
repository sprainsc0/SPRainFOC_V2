#ifndef __CAN_H
#define __CAN_H

#include <canard.h>
#include "datatypes.h"
#include <uavcan/protocol/NodeStatus.h>
#include <uavcan/protocol/param/GetSet.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define CAN_STDID 0
#define CAN_EXTID 1
#define CAN_DTR   0
#define CAN_RTR   1

#define CANARD_CAN_EXT_ID_MASK                      0x1FFFFFFFU
#define CANARD_CAN_STD_ID_MASK                      0x000007FFU

typedef struct
{
    uint32_t hdr;
    uint32_t id;
    uint32_t mask;
} CanardSTM32AcceptanceFilterConfiguration;

struct can_msg
{
    uint64_t ts  : 64;
    uint32_t id  : 29;
    uint32_t ide : 1;
    uint32_t rtr : 1;
    uint32_t rsv : 1;
    uint32_t len : 8;
    uint32_t priv : 8;
    int32_t hdr : 8;
    uint8_t data[64];
};
typedef struct can_msg *can_msg_t;

void can_init(void);
void can_restart(void);
int CAN_Transmit(const void* txframe);
int CAN_Receive(void* frame, uint8_t hdr);
int STM32_CAN_Filters(void *config, uint32_t num_filter_configs);

#ifdef __cplusplus
}
#endif

#endif
