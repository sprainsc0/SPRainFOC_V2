#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <float.h>
#include <ipcpush.h>
#include <hrt_timer.h>
#include <debug.h>

#include "canlink_subscription.h"
#include "canlink.h"

#include <topics/actuator_notify.h>
#include <topics/uavcan_parameter_request.h>
#include <topics/uavcan_parameter_value.h>
#include <topics/uavcan_command_response.h>

#include <uavcan/protocol/NodeStatus.h>
#include <uavcan/protocol/param/GetSet.h>
#include <uavcan/equipment/actuator/Command.h>
#include <uavcan/equipment/notify/Notify.h>

#define TOPICS_OFFSET_PRIORITY        26U
#define TOPICS_OFFSET_SUBJECT_ID      8U
#define TOPICS_OFFSET_SERVICE_ID      14U
#define TOPICS_OFFSET_DST_NODE_ID     7U

#define SERVICE_NOT_MESSAGE  (UINT32_C(1) << 25U)
#define ANONYMOUS_MESSAGE    (UINT32_C(1) << 24U)
#define REQUEST_NOT_RESPONSE (UINT32_C(1) << 24U)

class CanStreamParam : public CanStream
{
public:
	const char *get_name() const
	{
		return CanStreamParam::get_name_static();
	}

	static const char *get_name_static()
	{
		return "PARAM";
	}

	static uint32_t get_id_static()
	{
        uint32_t msg_id[2];
		msg_id[0] = take_message_id(UAVCAN_PROTOCOL_NODESTATUS_ID, CANLINK_ID_GCU, CanardPrioritySlow);
#ifdef UAVCAN_NODE_FOCR
    	msg_id[1] = take_service_id(UAVCAN_PROTOCOL_PARAM_GETSET_ID, true, CANLINK_ID_GCU, CANLINK_ID_FOCR, CanardPrioritySlow);
        return caculate_filters(&msg_id[0], 2);
#elif UAVCAN_NODE_FOCP
		msg_id[1] = take_service_id(UAVCAN_PROTOCOL_PARAM_GETSET_ID, true, CANLINK_ID_GCU, CANLINK_ID_FOCP, CanardPrioritySlow);
        return caculate_filters(&msg_id[0], 2);
#elif UAVCAN_NODE_FOCY
		msg_id[1] = take_service_id(UAVCAN_PROTOCOL_PARAM_GETSET_ID, true, CANLINK_ID_GCU, CANLINK_ID_FOCY, CanardPrioritySlow);
        return caculate_filters(&msg_id[0], 2);
#else
	return 0;
#endif
		
	}

	uint32_t get_id()
	{
		return get_id_static();
	}

	static CanStream *new_instance(CanLink *can)
	{
		return new CanStreamParam(can);
	}

	uint32_t Subscription()
	{
		(void) canardRxSubscribe(canins(),
								CanardTransferKindRequest,
								UAVCAN_PROTOCOL_PARAM_GETSET_ID,
								UAVCAN_PROTOCOL_PARAM_GETSET_REQUEST_MAX_SIZE,
								CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
								&subscription_param);
         
         (void) canardRxSubscribe(canins(),
								CanardTransferKindMessage,
								UAVCAN_PROTOCOL_NODESTATUS_ID,
								UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE,
								CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
								&subscription_info);

		filter_item[0].hdr    = can_hdr_param;
#ifdef UAVCAN_NODE_FOCR
		filter_item[0].id     = take_service_id(UAVCAN_PROTOCOL_PARAM_GETSET_ID, true, CANLINK_ID_GCU, CANLINK_ID_FOCR, CanardPrioritySlow);
#elif UAVCAN_NODE_FOCP
        filter_item[0].id     = take_service_id(UAVCAN_PROTOCOL_PARAM_GETSET_ID, true, CANLINK_ID_GCU, CANLINK_ID_FOCP, CanardPrioritySlow);
#elif UAVCAN_NODE_FOCY
        filter_item[0].id     = take_service_id(UAVCAN_PROTOCOL_PARAM_GETSET_ID, true, CANLINK_ID_GCU, CANLINK_ID_FOCY, CanardPrioritySlow);
#else
        filter_item[0].id     = take_message_id(UAVCAN_PROTOCOL_NODESTATUS_ID, CANLINK_ID_GCU, CanardPrioritySlow);
#endif
        filter_item[0].mask   = get_id_static();

		return STM32_CAN_Filters(&filter_item[0], 1);
	}

private:
	static constexpr uint32_t can_hdr_param = 0;

    uavcan_protocol_param_GetSetRequest can_param_data;
	uavcan_protocol_NodeStatus nodestatus;
	uint8_t data_buf[256];

	CanardRxSubscription subscription_info;
    CanardRxSubscription subscription_param;

	IPCPush param_req;

	CanardSTM32AcceptanceFilterConfiguration   filter_item[1];


	/* do not allow top copying this class */
	CanStreamParam(CanStreamParam &);
	CanStreamParam &operator = (const CanStreamParam &);

protected:
	explicit CanStreamParam(CanLink *can) : CanStream(can),
		param_req(IPC_ID(uavcan_parameter_request))
	{}

	bool link_proc(const uint64_t t, const uint8_t index = 0, const void *data = nullptr)
	{
		CanardTransfer transfer;
        CanardFrame received_frame;

		for(int16_t rx_res=0; (rx_res=CAN_Receive(&received_frame, can_hdr_param))>0;) {
			if(rx_res > 0) {
				const int8_t result = canardRxAccept(canins(),
										&received_frame,            // The CAN frame received from the bus.
										0,  // If the transport is not redundant, use 0.
										&transfer);
				if (result < 0) {
					// An error has occurred: either an argument is invalid or we've ran out of memory.
					// It is possible to statically prove that an out-of-memory will never occur for a given application if
					// the heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
					// Reception of an invalid frame is NOT an error.
					Info_Debug("canard receive error \n");
				} else if (result == 1) {
					process(t, &transfer);  // A transfer has been received, process it.
					canins()->memory_free(canins(), (void*)transfer.payload);  // Deallocate the dynamic memory afterwards.
				}
			}
        }

		return true;
	}

	bool process(const uint64_t t, const void *data)
	{
		CanardTransfer *transfer = (CanardTransfer *)data;
		
		if((transfer->port_id == UAVCAN_PROTOCOL_PARAM_GETSET_ID) &&
			(transfer->remote_node_id == CANLINK_ID_GCU) &&
			(transfer->transfer_kind == CanardTransferKindRequest)) {

			uint8_t *dy_buf = &data_buf[0];

			if(uavcan_protocol_param_GetSetRequest_decode(transfer, 0, &can_param_data, &dy_buf) < 0) {
				return true;
			}
			uavcan_parameter_request_s req;
			
			req.timestamp = t;
			req.param_type = (uint8_t)can_param_data.value.union_tag;
			req.param_index = can_param_data.index;
			req.message_type = can_param_data.operate;
			strncpy(req.param_id, (const char *)can_param_data.name.data,  can_param_data.name.len + 1);
			req.param_id[can_param_data.name.len + 1] = '\0';
			
			if(can_param_data.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE) {
				req.int_value = can_param_data.value.integer_value;
			} else if(can_param_data.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE) {
				req.real_value = can_param_data.value.real_value;
			}

			param_req.push(&req);
		}

		if((transfer->port_id == UAVCAN_PROTOCOL_NODESTATUS_ID) &&
        	(transfer->remote_node_id == CANLINK_ID_GCU) &&
       		(transfer->transfer_kind == CanardTransferKindMessage)) {
			if(uavcan_protocol_NodeStatus_decode(transfer, transfer->payload_size, &nodestatus, nullptr) < 0) {
				return false;
			}
			_can->fcu_vailed = transfer->timestamp_usec;
		}
		
		return true;
	}
};

static const CANStreamSubscriptionItem subscription_list[] = {
	CANStreamSubscriptionItem(&CanStreamParam::new_instance,         &CanStreamParam::get_name_static,         &CanStreamParam::get_id_static),
};

const char *get_subscription_name(const uint32_t link_id)
{
	// search for stream with specified msg id in supported streams list
	for (const auto &link : subscription_list) {
		if (link_id == link.get_id()) {
			return link.get_name();
		}
	}

	return nullptr;
}

CanStream *create_subscription_stream(const char *link_name, CanLink *can)
{
	// search for stream with specified name in supported streams list
	if (link_name != nullptr) {
		for (const auto &link : subscription_list) {
			if (strcmp(link_name, link.get_name()) == 0) {
				return link.new_instance(can);
			}
		}
	}

	return nullptr;
}
