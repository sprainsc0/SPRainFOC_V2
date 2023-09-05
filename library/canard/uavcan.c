#include "uavcan.h"
#include <cmsis_os.h>
#include <assert.h>
#include <string.h>

static CanardInstance canard;

static void* memAllocate(CanardInstance* const ins, const size_t amount)
{
    (void) ins;
    return pvPortMalloc(amount);
}

static void memFree(CanardInstance* const ins, void* const pointer)
{
    (void) ins;
    vPortFree(pointer);
}

CanardInstance *canins(void)
{
    return &canard;
}

uint32_t compare(uint32_t a, uint32_t b)
{
    uint32_t res = 0;
    for (uint32_t i=0; i<32; i++) {
        uint8_t temp = !((a >> i) & 1) ^ ((b >> i) & 1);
        res |= temp << i;
    }
    return res;
}

uint32_t caculate_filters(uint32_t *id, uint8_t len)
{
    uint32_t mask = 0x1FFFFFFF;
    for(uint8_t i=0; i<len; i++) {
        mask &= compare(id[i], id[0]);
    }
    return mask;
}

#define TOPICS_OFFSET_PRIORITY        26U
#define TOPICS_OFFSET_SUBJECT_ID      8U
#define TOPICS_OFFSET_SERVICE_ID      14U
#define TOPICS_OFFSET_DST_NODE_ID     7U

#define SERVICE_NOT_MESSAGE  (UINT32_C(1) << 25U)
#define ANONYMOUS_MESSAGE    (UINT32_C(1) << 24U)
#define REQUEST_NOT_RESPONSE (UINT32_C(1) << 24U)

uint32_t take_message_id(const CanardPortID subject_id, const CanardNodeID src_node_id, const CanardPriority prio)
{
    const uint32_t tmp = subject_id | (CANARD_SUBJECT_ID_MAX + 1) | ((CANARD_SUBJECT_ID_MAX + 1) * 2);
    return src_node_id | (tmp << TOPICS_OFFSET_SUBJECT_ID) | (prio << TOPICS_OFFSET_PRIORITY);
}

uint32_t take_service_id(const CanardPortID service_id,
                                const bool request_not_response,
                                const CanardNodeID src_node_id,
                                const CanardNodeID dst_node_id,
                                const CanardPriority prio)
{
    return src_node_id | (((uint32_t) dst_node_id) << TOPICS_OFFSET_DST_NODE_ID) |
           (((uint32_t) service_id) << TOPICS_OFFSET_SERVICE_ID) |
           (request_not_response ? REQUEST_NOT_RESPONSE : 0U) | SERVICE_NOT_MESSAGE | (prio << TOPICS_OFFSET_PRIORITY);
}

int uavcan_init(void)
{
    canard = canardInit(&memAllocate, &memFree);
#ifdef UAVCAN_NODE_FOCR
    canard.node_id   = CANLINK_ID_FOCR;
#endif
#ifdef UAVCAN_NODE_FOCP
    canard.node_id   = CANLINK_ID_FOCP;
#endif
#ifdef UAVCAN_NODE_FOCY
    canard.node_id   = CANLINK_ID_FOCY;
#endif
    canard.mtu_bytes = CANARD_MTU_CAN_FD;
    
    return 1;
}

