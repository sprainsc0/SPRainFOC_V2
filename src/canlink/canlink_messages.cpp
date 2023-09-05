#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <float.h>
#include <ipcpull.h>

#include "canlink_messages.h"
#include "canlink.h"

#include <topics/uavcan_parameter_request.h>
#include <topics/uavcan_command_request.h>
#include <topics/encoder.h>

#include <uavcan/protocol/NodeStatus.h>
#include <uavcan/protocol/param/GetSet.h>
#include <uavcan/equipment/actuator/Command.h>
#include <uavcan/equipment/foc/Status.h>

class CanStreamStatus : public CanStream
{
public:
	const char *get_name() const
	{
		return CanStreamStatus::get_name_static();
	}

	static const char *get_name_static()
	{
		return "NODE";
	}

	static uint32_t get_id_static()
	{
		return UAVCAN_PROTOCOL_NODESTATUS_ID;
	}

	uint32_t get_id()
	{
		return get_id_static();
	}

	static CanStream *new_instance(CanLink *can)
	{
		return new CanStreamStatus(can);
	}

	uint32_t Subscription()
	{
		return 0;
	}

private:
	uint8_t transfer_id;

	/* do not allow top copying this class */
	CanStreamStatus(CanStreamStatus &);
	CanStreamStatus &operator = (const CanStreamStatus &);


protected:
	explicit CanStreamStatus(CanLink *can) : CanStream(can),
		transfer_id(0)
	{}

	bool link_proc(const uint64_t t, const uint8_t index = 0, const void *data = nullptr)
	{	
        uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
        
        memset(buffer, 0, UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE);

        uavcan_protocol_NodeStatus nodestatus;
        nodestatus.uptime_sec = millis();
        nodestatus.health     = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
        nodestatus.mode       = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
        nodestatus.sub_mode   = 0;
        nodestatus.vendor_specific_status_code = 0;

        const uint32_t total_size = uavcan_protocol_NodeStatus_encode(&nodestatus, buffer);

        const CanardTransfer transfer = {
            /*timestamp_usec */ 0,      // Zero if transmission deadline is not limited.
            /*priority       */ CanardPrioritySlow,
            /*transfer_kind  */ CanardTransferKindMessage,
            /*port_id        */ UAVCAN_PROTOCOL_NODESTATUS_ID, // This is the subject-ID.
            /*remote_node_id */ CANARD_NODE_ID_UNSET,          // Messages cannot be unicast, so use UNSET.
            /*transfer_id    */ transfer_id,
            /*payload_size   */ total_size,
            /*payload        */ buffer,
        };

		++transfer_id;

        const int32_t resp_res = canardTxPush(canins(), &transfer);

        if (resp_res <= 0) {
            return false;
        }

		return true;
	}
};

class CanStreamEncoder : public CanStream
{
public:
	const char *get_name() const
	{
		return CanStreamEncoder::get_name_static();
	}

	static const char *get_name_static()
	{
		return "ENC";
	}

	static uint32_t get_id_static()
	{
		return UAVCAN_EQUIPMENT_FOC_STATUS_ID;
	}

	uint32_t get_id()
	{
		return get_id_static();
	}

	static CanStream *new_instance(CanLink *can)
	{
		return new CanStreamEncoder(can);
	}

	uint32_t Subscription()
	{
		return 0;
	}

private:
	uint8_t transfer_id;

	IPCPull enc_sub;

	/* do not allow top copying this class */
	CanStreamEncoder(CanStreamEncoder &);
	CanStreamEncoder &operator = (const CanStreamEncoder &);


protected:
	explicit CanStreamEncoder(CanLink *can) : CanStream(can),
		enc_sub(IPC_ID(encoder)),
		transfer_id(0)
	{}

	bool link_proc(const uint64_t t, const uint8_t index = 0, const void *data = nullptr)
	{	
		encoder_s enc_data;

		if(enc_sub.update_if_changed(&enc_data)) {
			uint8_t buffer[UAVCAN_EQUIPMENT_FOC_STATUS_MAX_SIZE];
        
			memset(buffer, 0, UAVCAN_EQUIPMENT_FOC_STATUS_MAX_SIZE);

			uavcan_equipment_foc_Status status;

			status.error_code = enc_data.tpye;
			status.angle = enc_data.angle_m;

			const uint32_t total_size = uavcan_equipment_foc_Status_encode(&status, buffer);

			const CanardTransfer transfer = {
				/*timestamp_usec */ 0,      // Zero if transmission deadline is not limited.
				/*priority       */ CanardPriorityHigh,
				/*transfer_kind  */ CanardTransferKindMessage,
				/*port_id        */ UAVCAN_EQUIPMENT_FOC_STATUS_ID, // This is the subject-ID.
				/*remote_node_id */ CANARD_NODE_ID_UNSET,          // Messages cannot be unicast, so use UNSET.
				/*transfer_id    */ transfer_id,
				/*payload_size   */ total_size,
				/*payload        */ buffer,
			};

			++transfer_id;

			const int32_t resp_res = canardTxPush(canins(), &transfer);

			if (resp_res <= 0) {
				return false;
			}
		}

		return true;
	}
};


static const CANStreamListItem message_list[] = {
	CANStreamListItem(&CanStreamStatus::new_instance,         &CanStreamStatus::get_name_static,           &CanStreamStatus::get_id_static),
	CANStreamListItem(&CanStreamEncoder::new_instance,        &CanStreamEncoder::get_name_static,          &CanStreamEncoder::get_id_static),
};

const char *get_message_name(const uint32_t link_id)
{
	// search for stream with specified msg id in supported streams list
	for (const auto &link : message_list) {
		if (link_id == link.get_id()) {
			return link.get_name();
		}
	}

	return nullptr;
}

CanStream *create_message_stream(const char *link_name, CanLink *can)
{
	// search for stream with specified name in supported streams list
	if (link_name != nullptr) {
		for (const auto &link : message_list) {
			if (strcmp(link_name, link.get_name()) == 0) {
				return link.new_instance(can);
			}
		}
	}

	return nullptr;
}
