#include "ind_detected.h"
#include "configure.h"
#include "cmsis_os.h"
#include "utils.h"
#include <string>
#include <hrt_timer.h>
#include <debug.h>

IND_Cal::IND_Cal(orb_advert_t &cal_status_pub):
    _mavlink_cal_pub(cal_status_pub),
    foc_mode(0)
{
    _cal_params_handles.motor_ind_handle = param_find("MOTOR_L");
    _cal_params_handles.ind_diff_handle  = param_find("MOTOR_L_DIFF");

    _cal_params_handles.foc_sample_v0_v7_handle = param_find("SAM_MODE");
    _cal_params_handles.sensor_type_handle = param_find("SENSOR_TYPE");
    _cal_params_handles.hfi_sample_handle  = param_find("HFI_SAM_NUM");

    _cal_params_handles.hfi_voltage_start_handle = param_find("HFI_V_START");
	_cal_params_handles.hfi_voltage_run_handle   = param_find("HFI_V_RUN");
    _cal_params_handles.hfi_voltage_max_handle   = param_find("HFI_V_MAX");

    _params_sub     = ipc_subscibe(IPC_ID(parameter_update));
    _foc_status_sub = ipc_subscibe(IPC_ID(foc_status));
    _hfi_status_sub = ipc_subscibe(IPC_ID(hfi_status));
    _foc_ref_pub    = ipc_active(IPC_ID(foc_target), &_foc_ref);
}

IND_Cal::~IND_Cal()
{
    ipc_unsubscibe(_params_sub);
    ipc_inactive(_foc_ref_pub);
}

void IND_Cal::set_ind_parameter(void)
{
    param_set_no_notification(_cal_params_handles.motor_ind_handle,   &_ind_calibration.motor_ind);
    param_set_no_notification(_cal_params_handles.ind_diff_handle,    &_ind_calibration.ind_diff);

    param_notify_changes(PARAM_UPDATE_CALIBRATED_MOTOR);
}

void IND_Cal::set_hfi_parameter(void)
{
    param_set_external(_cal_params_handles.sensor_type_handle, &_ind_calibration.sensor_type, true, false);
    param_set_external(_cal_params_handles.hfi_sample_handle,  &_ind_calibration.hfi_sample,  true, false);
    param_set_external(_cal_params_handles.hfi_voltage_start_handle,  &_ind_calibration.hfi_voltage_start,  true, false);
    param_set_external(_cal_params_handles.hfi_voltage_run_handle,    &_ind_calibration.hfi_voltage_run,    true, false);
    param_set_external(_cal_params_handles.hfi_voltage_max_handle,    &_ind_calibration.hfi_voltage_max,    true, false);
    param_set_external(_cal_params_handles.foc_sample_v0_v7_handle,   &_ind_calibration.foc_sample_v0_v7,   true, false);

    param_notify_changes(PARAM_UPDATE_CALIBRATED_MOTOR);
}

void IND_Cal::get_ind_parameter(void)
{
    param_get(_cal_params_handles.sensor_type_handle,        &_ind_cfg_old.sensor_type);
    param_get(_cal_params_handles.hfi_sample_handle,         &_ind_cfg_old.hfi_sample);
    param_get(_cal_params_handles.hfi_voltage_start_handle,  &_ind_cfg_old.hfi_voltage_start);
    param_get(_cal_params_handles.hfi_voltage_run_handle,    &_ind_cfg_old.hfi_voltage_run);
    param_get(_cal_params_handles.hfi_voltage_max_handle,    &_ind_cfg_old.hfi_voltage_max);
    param_get(_cal_params_handles.foc_sample_v0_v7_handle,   &_ind_cfg_old.foc_sample_v0_v7);
}

void IND_Cal::send_status(uint8_t status)
{
    if(_mavlink_cal_pub == nullptr) {
        _mavlink_cal_pub = ipc_active(IPC_ID(calibrate_status), &_cal_status);
    }
    _cal_status.calibrate_type = IND_CALIBRATING;

    _cal_status.calibrate_statu = status;
    ipc_push(IPC_ID(calibrate_status), _mavlink_cal_pub, &_cal_status);
}

bool IND_Cal::do_calibration(float duty, int samples)
{
    struct hfi_status_s hfi_data;
    struct foc_status_s foc_data;
    
    bool updated = false;
    Info_Debug("[motor]Inductance Detected Waiting FOC Status\n");
    while (1) {
        ipc_check(_foc_status_sub, &updated);
        if(updated) {
            
            ipc_pull(IPC_ID(foc_status), _foc_status_sub, &foc_data);
            foc_mode = foc_data.ctrl_mode;
            break;
        }
    }
    Info_Debug("[motor]Inductance Detected start\n");

    send_status(IND_CALIBRATE_STARTED);
    
    _foc_ref.ctrl_mode = MC_CTRL_IDLE;
    _foc_ref.id_target = 0;
    _foc_ref.iq_target = 0;
    _foc_ref.vd_target = 0;
    _foc_ref.vq_target = 0;
    _foc_ref.phase_override = 0;
    ipc_push(IPC_ID(foc_target), _foc_ref_pub, &_foc_ref);

    get_ind_parameter();

    _ind_calibration.sensor_type = 2;
    _ind_calibration.hfi_sample  = 2;
    _ind_calibration.foc_sample_v0_v7 = 0;
    _ind_calibration.hfi_voltage_start = duty * foc_data.vbus * (2.0f / 3.0f);
    _ind_calibration.hfi_voltage_run   = duty * foc_data.vbus * (2.0f / 3.0f);
    _ind_calibration.hfi_voltage_max   = duty * foc_data.vbus * (2.0f / 3.0f);
    set_hfi_parameter();
    osDelay(1000);

    _foc_ref.ctrl_mode = (MC_CTRL_CURRENT | MC_CTRL_ENABLE | MC_CTRL_DUTY);
    _foc_ref.target_duty = 0.0f;
    ipc_push(IPC_ID(foc_target), _foc_ref_pub, &_foc_ref);
    osDelay(1000);

    int ready_cnt = 0;
	while (1) {
		osDelay(1);
        ipc_pull(IPC_ID(hfi_status), _hfi_status_sub, &hfi_data);
		ready_cnt++;
		if (ready_cnt > 100) {
			break;
		}
        if(hfi_data.ready) {
            break;
        }
	}
        
    float l_sum = 0.0;
	float ld_lq_diff_sum = 0.0;
	float i_sum = 0.0;
	float iterations = 0.0;    
    
    for (int i = 0;i < (samples / 10);) {
		
		_foc_ref.ctrl_mode = (MC_CTRL_CURRENT | MC_CTRL_ENABLE | MC_CTRL_DUTY);
        _foc_ref.target_duty = 0.0f;
        ipc_push(IPC_ID(foc_target), _foc_ref_pub, &_foc_ref);
		osDelay(100);

		float real_bin0, imag_bin0;
		float real_bin2, imag_bin2;
		float real_bin0_i, imag_bin0_i;

        updated = false;
        ipc_check(_hfi_status_sub, &updated);
        if(updated) {
            ipc_pull(IPC_ID(hfi_status), _hfi_status_sub, &hfi_data);

            hfi_data.fft_bin0_func((float*)hfi_data.buffer, &real_bin0, &imag_bin0);
            hfi_data.fft_bin2_func((float*)hfi_data.buffer, &real_bin2, &imag_bin2);
            hfi_data.fft_bin0_func((float*)hfi_data.buffer_current, &real_bin0_i, &imag_bin0_i);

            l_sum += real_bin0;
            ld_lq_diff_sum += 2.0f * sqrtf(SQ(real_bin2) + SQ(imag_bin2));
            i_sum += real_bin0_i;

            iterations++;
            i++;
        }
	}

    _foc_ref.ctrl_mode = MC_CTRL_IDLE;
    ipc_push(IPC_ID(foc_target), _foc_ref_pub, &_foc_ref);

	float curr = i_sum / iterations;
	_ind_calibration.ind_diff  = (ld_lq_diff_sum / iterations) * 1e6 * (2.0f / 3.0f) * 1e-6;
    _ind_calibration.motor_ind =  (l_sum / iterations) * 1e6 * (2.0f / 3.0f) * 1e-6;

    _ind_calibration.sensor_type       = _ind_cfg_old.sensor_type;
    _ind_calibration.hfi_sample        = _ind_cfg_old.hfi_sample;
    _ind_calibration.foc_sample_v0_v7  = _ind_cfg_old.foc_sample_v0_v7;
    _ind_calibration.hfi_voltage_start = _ind_cfg_old.hfi_voltage_start;
    _ind_calibration.hfi_voltage_run   = _ind_cfg_old.hfi_voltage_run;
    _ind_calibration.hfi_voltage_max   = _ind_cfg_old.hfi_voltage_max;
    set_hfi_parameter();
    
    set_ind_parameter();

    send_status(IND_CALIBRATE_SUCCESS);

    Info_Debug("[motor]Inductance Detected Success\n");

    char print[50];
    sprintf(print, "curr-%.6f, diff-%.6f, ind-%.6f \n", curr, _ind_calibration.ind_diff, _ind_calibration.motor_ind);
    Info_Debug("%s \n", print);
    
    return true;
}
