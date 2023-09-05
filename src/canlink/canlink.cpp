#include "canlink.h"
#include <string>
#include "hrt_timer.h"
#include "foc_function.h"
#include "debug.h"
#include <utlist.h>

#include "canlink_messages.h"
#include "canlink_subscription.h"

const osThreadAttr_t cans_attributes = {
    .name = "cans",
    .priority = (osPriority_t)osPriorityRealtime1,
    .stack_size = 2048};

const osThreadAttr_t canr_attributes = {
    .name = "canr",
    .priority = (osPriority_t)osPriorityHigh,
    .stack_size = 2048};

namespace CAN
{
static CanLink	*gCan;
}

static void cans_func(CanLink *pThis)
{
    pThis->send();
}

static void canr_func(CanLink *pThis)
{
    pThis->recv();
}

CanLink::CanLink(void):
    _led_pub(IPC_ID(actuator_notify)),
    param_send_ts(0),
    connect(false),
    per_connect(false)
{
    
}

bool CanLink::init(void)
{
    can_init();

    _can_param = new CanParam(this);

    pref_can_interval = perf_alloc(PC_INTERVAL, "can_int");
    pref_can_elapsed = perf_alloc(PC_ELAPSED,   "can_ela");

    _send_handle = osThreadNew((osThreadFunc_t)cans_func, this, &cans_attributes);
    _recv_handle = osThreadNew((osThreadFunc_t)canr_func, this, &canr_attributes);

    if ((_send_handle == nullptr) || (_recv_handle == nullptr)) {
        return false;
    }

	return true;
}

int CanLink::interval_from_rate(float rate)
{
	if (rate > 0.000001f) {
		return (int)(1000000.0f / rate);

	} else if (rate < 0.0f) {
		return -1;

	} else {
		return 0;
	}
}

int CanLink::configure_link(const char *stream_name, const float rate)
{
	/* calculate interval in us, -1 means unlimited stream, 0 means disabled */
	int interval = interval_from_rate(rate);

	/* search if stream exists */
	CanStream *stream;
	LL_FOREACH(_can_streams, stream) {
		if (strcmp(stream_name, stream->get_name()) == 0) {
			if (interval != 0) {
				/* set new interval */
				stream->set_interval(interval);

			} else {
				/* delete stream */
				LL_DELETE(_can_streams, stream);
				delete stream;
			}

			return 1;
		}
	}

	if (interval == 0) {
		/* stream was not active and is requested to be disabled, do nothing */
		return 1;
	}

	// search for stream with specified name in supported streams list
	// create new instance if found
	stream = create_message_stream(stream_name, this);

	if (stream != nullptr) {
		stream->set_interval(interval);
		LL_APPEND(_can_streams, stream);

		return 1;
	}

	return 0;
}

uint32_t CanLink::configure_subscription(const char *stream_name, const float rate)
{
    /* calculate interval in us, -1 means unlimited stream, 0 means disabled */
	int interval = interval_from_rate(rate);
    uint32_t sub_id = 0;

	/* search if stream exists */
	CanStream *stream;
	LL_FOREACH(_sub_streams, stream) {
		if (strcmp(stream_name, stream->get_name()) == 0) {
			if (interval != 0) {
				/* set new interval */
				stream->set_interval(interval);
                sub_id = stream->get_id();
                return sub_id;
			} else {
				/* delete stream */
				LL_DELETE(_sub_streams, stream);
				delete stream;
			}

			return 0;
		}
	}

	// search for stream with specified name in supported streams list
	// create new instance if found
	stream = create_subscription_stream(stream_name, this);

	if (stream != nullptr) {
		stream->set_interval(interval);
        sub_id = stream->Subscription();
		LL_APPEND(_sub_streams, stream);

		return sub_id;
	}

	return 0;
}

void CanLink::processReceivedTransfer(uint64_t ts)
{
    /* update streams */
    CanStream *can;
    LL_FOREACH(_sub_streams, can) {
        can->update(ts);
    }
}

void CanLink::send(void)
{
    configure_link("NODE",  1.0f);
    configure_link("ENC",   500.0f);

    while (1)
    {
        const uint64_t ts = micros();

        perf_count(pref_can_interval);

        perf_begin(pref_can_elapsed);

        connect = (ts - fcu_vailed) < 3000000;
        if(per_connect != connect) {
            per_connect = connect;
        }

        _can_param->uavcan_param();
        if((ts - param_send_ts) > 200000) {
            param_send_ts = ts;
            if(_can_param->send_one()) {
                goto can_send;
            }
        }

        CanStream *can;
		LL_FOREACH(_can_streams, can) {
			can->update(ts);
		}

can_send:
        for (const CanardFrame* txf = nullptr; (txf = canardTxPeek(canins())) != nullptr;) {
            if ((txf->timestamp_usec == 0) || (txf->timestamp_usec > micros()))  {
                CAN_Transmit(txf);
            }
            canardTxPop(canins());
            canins()->memory_free(canins(), (CanardFrame*)txf);
        }
        perf_end(pref_can_elapsed);
        
        osDelay(2);
    }
}

void CanLink::recv(void)
{
    configure_subscription("PARAM", 200.0f);

    while (1)
    {
        const uint64_t ts = micros();

        processReceivedTransfer(ts);  // A transfer has been received, process it.

        osDelay(4);
    }
}


int canlink_main(int argc, char *argv[])
{
    if (argc < 1) {
		Info_Debug("input argv error\n");
		return 1;
	}
    for(int i=0; i<argc; i++) {
        if (!strcmp(argv[i], "start")) {

            if (CAN::gCan != nullptr) {
                Info_Debug("already running\n");
                return 0;
            }

            CAN::gCan = new CanLink();
            

            if (CAN::gCan == NULL) {
                Info_Debug("alloc failed\n");
                return 0;
            }
            CAN::gCan->init();
        }
    }
    return 1;
}
