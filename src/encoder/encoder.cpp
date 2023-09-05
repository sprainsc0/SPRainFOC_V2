#include "hrt_timer.h"
#include "debug.h"
#include "encoder.h"
#include "amt222.h"
#include "utils.h"
#include <platform.h>

#define ADC_HALL_A_INDEX       0
#define ADC_HALL_B_INDEX       1
#define ADC_HALL_C_INDEX       2

static uint16_t adc3_raw[3];

const osThreadAttr_t enc_attributes = {
		.name = "enc",
		.priority = (osPriority_t)osPriorityHigh,
		.stack_size = 512};

static void enc_func(Encoder *pThis)
{
    pThis->run(pThis->_param);
}

Encoder::Encoder(void) :
	_enc_ready(false),
	_enc_reset(false),
	_refint(0),
	internal_dt(0.0f),
	_param(NULL)
{

}

Encoder::~Encoder(void)
{

}

bool Encoder::init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	_params_sub     = ipc_subscibe(IPC_ID(parameter_update));
	_encoder_pub    = ipc_active(IPC_ID(encoder), &_enc_data);

	_param_handles.type_handle     = param_find("ENC_TYPE");
	_param_handles.offset_e_handle = param_find("ENC_E_OFFSET");
	_param_handles.offset_m_handle = param_find("ENC_M_OFFSET");
	_param_handles.invert_e_handle = param_find("ENC_E_INVERT");
	_param_handles.invert_m_handle = param_find("ENC_M_INVERT");
	_param_handles.pair_num_handle = param_find("PAIR_NUM");

	_param_handles.u_handle     = param_find("LHALL_U");
    _param_handles.lamda_handle = param_find("LHALL_LAMDA");
    _param_handles.beta_handle  = param_find("LHALL_BETA");

    _param_handles.mida_handle = param_find("LHALL_MIDA");
    _param_handles.midb_handle = param_find("LHALL_MIDB");
    _param_handles.midc_handle = param_find("LHALL_MIDC");

	// Just get and init encoder type in boot startup
	param_get(_param_handles.type_handle,     &_enc_cfg.type);

	switch(_enc_cfg.type) {
	case ENC_ANGLE:
		break;

	case ENC_SWITCH_HALL:
		GPIO_InitStruct.Pin = HALL_B_Pin|HALL_A_Pin|HALL_C_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		break;

	case ENC_ABZ:
		// start up timner
		break;

	case ENC_LINEAR_HALL:
		HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
		HAL_ADC_Start_DMA(&hadc3, (uint32_t *)adc3_raw, 3);
		break;

	default:
		Info_Debug("Encoder Type-%d error \n", _enc_cfg.type);
		break;
	}

	enc_tim_int  = perf_alloc(PC_INTERVAL, "tim_int");
	enc_task_int = perf_alloc(PC_INTERVAL, "enc_int");

	enc_tim_ela  = perf_alloc(PC_ELAPSED, "tim_ela");
	enc_task_ela = perf_alloc(PC_ELAPSED, "enc_ela");

	enc_err_count = perf_alloc(PC_COUNT, "enc_err");

	_handle = osThreadNew((osThreadFunc_t)enc_func, this, &enc_attributes);

    if (_handle != nullptr) {
        return true;
    }

	return false;
}

void Encoder::parameter_update(bool force)
{
	bool updated = force;
    if (!updated) {
		ipc_check(_params_sub, &updated);
	}
    if (updated) {
		parameter_update_s param_update;
		ipc_pull(IPC_ID(parameter_update), _params_sub, &param_update);
        
		param_get(_param_handles.offset_e_handle, &_enc_cfg.offset_e);
		param_get(_param_handles.offset_m_handle, &_enc_cfg.offset_m);
		param_get(_param_handles.invert_e_handle, &_enc_cfg.invert_e);
		param_get(_param_handles.invert_m_handle, &_enc_cfg.invert_m);
		param_get(_param_handles.pair_num_handle, &_enc_cfg.pair_num);

		param_get(_param_handles.u_handle,     &_enc_cfg.lhall_u);
		param_get(_param_handles.lamda_handle, &_enc_cfg.lamda);
		param_get(_param_handles.beta_handle,  &_enc_cfg.beta);
		param_get(_param_handles.mida_handle,  &_enc_cfg.mida);
		param_get(_param_handles.midb_handle,  &_enc_cfg.midb);
		param_get(_param_handles.midc_handle,  &_enc_cfg.midc);
    }
}

void Encoder::run(void *parammeter)
{
	parameter_update(true);
	uint64_t start_timr = micros();

	while(1) {
		const uint64_t ts = micros();

		perf_begin(enc_task_ela);

		perf_count(enc_task_int);

		// push foc status
		ipc_push(IPC_ID(encoder), _encoder_pub, &_enc_data);

		parameter_update(false);
		if(((ts - start_timr) > 350000) && !_enc_reset) {
			hal_amt222_reset();
			start_timr = ts;
			_enc_reset = true;
			_enc_ready = false;
		}

		if(((ts - start_timr) > 350000) && !_enc_ready && _enc_reset) {
			_enc_ready = true;
		}

		perf_end(enc_task_ela);

		osDelay(2);
	}
}

void Encoder::enc_process(void)
{
	uint32_t raw;
	float hall_data[3];
	bool healthy = false;

	if(!_enc_ready) {
		return;
	}

	perf_begin_isr(enc_tim_ela);

	perf_count_isr(enc_tim_int);

	switch(_enc_cfg.type) {
	case ENC_ANGLE:
		if(hal_amt222_read(&raw)) {
			healthy = true;
			// rad
			raw_angle = (float)(raw / 16384.0f) * M_2PI;

			// eletric angle
			if (_enc_cfg.invert_e) {
				elec_angle = (M_2PI - raw_angle) * _enc_cfg.pair_num;
			} else {
				elec_angle = raw_angle * _enc_cfg.pair_num;
			}

			elec_angle -= _enc_cfg.offset_e;
			elec_angle = wrap_PI(elec_angle);

			// position
			mech_angle = raw_angle - _enc_cfg.offset_m;
			if (_enc_cfg.invert_m) {
				mech_angle = M_2PI - mech_angle;
			}
			mech_angle = wrap_PI(mech_angle);
		} else {
			goto fault;
		}
		break;
	case ENC_ABZ:
		break;
	case ENC_SWITCH_HALL:
		break;
	case ENC_LINEAR_HALL:
		if((_refint != 0) && (adc3_raw[ADC_HALL_A_INDEX] != 0) && (adc3_raw[ADC_HALL_B_INDEX] != 0) && (adc3_raw[ADC_HALL_C_INDEX] != 0)) {
			healthy = true;
			// hall_data[ADC_HALL_A_INDEX] = (VREFINT * ((adc3_raw[ADC_HALL_A_INDEX]*(VREF/4096))/(_refint*(VREF/4096))));
			// hall_data[ADC_HALL_B_INDEX] = (VREFINT * ((adc3_raw[ADC_HALL_B_INDEX]*(VREF/4096))/(_refint*(VREF/4096))));
			// hall_data[ADC_HALL_C_INDEX] = (VREFINT * ((adc3_raw[ADC_HALL_C_INDEX]*(VREF/4096))/(_refint*(VREF/4096))));

			hall_data[ADC_HALL_A_INDEX] = (adc3_raw[ADC_HALL_A_INDEX]*(VREF/4096));
			hall_data[ADC_HALL_B_INDEX] = (adc3_raw[ADC_HALL_B_INDEX]*(VREF/4096));
			hall_data[ADC_HALL_C_INDEX] = (adc3_raw[ADC_HALL_C_INDEX]*(VREF/4096));

			hall_data_correct[ADC_HALL_A_INDEX] = (hall_data[ADC_HALL_A_INDEX] - _enc_cfg.mida) / _enc_cfg.lhall_u;
			hall_data_correct[ADC_HALL_B_INDEX] = (hall_data[ADC_HALL_B_INDEX] - _enc_cfg.midb) / ((1.0f + _enc_cfg.lamda)*_enc_cfg.lhall_u);
			hall_data_correct[ADC_HALL_C_INDEX] = (hall_data[ADC_HALL_C_INDEX] - _enc_cfg.midc) / ((1.0f + _enc_cfg.beta)*_enc_cfg.lhall_u);

			utils_truncate_number(&hall_data_correct[ADC_HALL_A_INDEX], -1.0f, 1.0f);
			utils_truncate_number(&hall_data_correct[ADC_HALL_B_INDEX], -1.0f, 1.0f);
			utils_truncate_number(&hall_data_correct[ADC_HALL_C_INDEX], -1.0f, 1.0f);

			const float cos_sita = hall_data_correct[ADC_HALL_A_INDEX];
			const float sin_sita = (hall_data_correct[ADC_HALL_B_INDEX] - hall_data_correct[ADC_HALL_A_INDEX]*sin_alpha) / cos_alpha;
			const float err = sin_sita*cosf(raw_angle) - cos_sita*sinf(raw_angle);
			internal_dt += err*0.0002f;
			raw_angle += 0.6f*err + 20.0f*internal_dt;
			raw_angle = wrap_PI(raw_angle);
			pll_angle = atan2f(sin_sita, cos_sita);

			if (_enc_cfg.invert_e) {
				elec_angle = (M_2PI - raw_angle);
			} else {
				elec_angle = raw_angle;
			}

			elec_angle -= _enc_cfg.offset_e;
			elec_angle = wrap_PI(elec_angle);
		} else {
			goto fault;
		}
		break;
	default:
		goto fault;
		break;
	}

	_enc_data.timestamp = micros();
	_enc_data.tpye      = (enc_sensor_type_t)_enc_cfg.type;
	_enc_data.healthy   = healthy && (perf_get_count(enc_err_count) < 100);
	_enc_data.angle     = raw_angle;
	_enc_data.angle_e   = elec_angle;
	_enc_data.angle_m   = mech_angle;
	_enc_data.linear_hall[ADC_HALL_A_INDEX] = hall_data[ADC_HALL_A_INDEX];
	_enc_data.linear_hall[ADC_HALL_B_INDEX] = hall_data[ADC_HALL_B_INDEX];
	_enc_data.linear_hall[ADC_HALL_C_INDEX] = hall_data[ADC_HALL_C_INDEX];

	perf_end_isr(enc_tim_ela);
	return;

fault:
    perf_count_isr(enc_err_count);
}
