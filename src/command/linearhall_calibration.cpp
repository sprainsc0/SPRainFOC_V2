#include "linearhall_calibration.h"
#include "configure.h"
#include "cmsis_os.h"
#include "utils.h"
#include <string>
#include <hrt_timer.h>
#include <debug.h>

Enc_LHALL::Enc_LHALL(orb_advert_t &cal_status_pub):
    _mavlink_cal_pub(cal_status_pub),
    foc_mode(0)
{
    _cal_params_handles.u_handle     = param_find("LHALL_U");
    _cal_params_handles.lamda_handle = param_find("LHALL_LAMDA");
    _cal_params_handles.beta_handle  = param_find("LHALL_BETA");

    _cal_params_handles.mida_handle = param_find("LHALL_MIDA");
    _cal_params_handles.midb_handle = param_find("LHALL_MIDB");
    _cal_params_handles.midc_handle = param_find("LHALL_MIDC");

    _params_sub     = ipc_subscibe(IPC_ID(parameter_update));
    _enc_sub        = ipc_subscibe(IPC_ID(encoder));
    _foc_status_sub = ipc_subscibe(IPC_ID(foc_status));
    _foc_ref_pub    = ipc_active(IPC_ID(foc_target), &_foc_ref);
}

Enc_LHALL::~Enc_LHALL()
{
    ipc_unsubscibe(_params_sub);
    ipc_unsubscibe(_enc_sub);
    ipc_inactive(_foc_ref_pub);
}

void Enc_LHALL::set_enc_parameter(void)
{
    param_set_no_notification(_cal_params_handles.u_handle,     &_hall_calibration._u_handle);
    param_set_no_notification(_cal_params_handles.lamda_handle, &_hall_calibration._lamda_handle);
    param_set_no_notification(_cal_params_handles.beta_handle,  &_hall_calibration._beta_handle);

    param_set_no_notification(_cal_params_handles.mida_handle, &_hall_calibration._mida_handle);
    param_set_no_notification(_cal_params_handles.midb_handle, &_hall_calibration._midb_handle);
    param_set_no_notification(_cal_params_handles.midc_handle, &_hall_calibration._midc_handle);

    param_notify_changes(PARAM_UPDATE_CALIBRATED_EANG);
}

void Enc_LHALL::send_status(uint8_t status)
{
    if(_mavlink_cal_pub == nullptr) {
        _mavlink_cal_pub = ipc_active(IPC_ID(calibrate_status), &_cal_status);
    }
    _cal_status.calibrate_type = ENCE_CALIBRATING;

    _cal_status.calibrate_statu = status;
    ipc_push(IPC_ID(calibrate_status), _mavlink_cal_pub, &_cal_status);
}

bool Enc_LHALL::do_calibration(float param, uint8_t use_current)
{
    float *d_axis;

    bool updated = false;
    Info_Debug("[lhall]Linear Hall Waiting FOC Status\n");
    while (1) {
        ipc_check(_foc_status_sub, &updated);
        if(updated) {
            struct foc_status_s data;
            ipc_pull(IPC_ID(foc_status), _foc_status_sub, &data);
            foc_mode = data.ctrl_mode;
            break;
        }
    }
    Info_Debug("[lhall]Linear Hall Calibration start\n");
    send_status(ENC_CALIBRATE_STARTED);
    
    if (use_current) {
		_foc_ref.ctrl_mode = (MC_CTRL_OVERRIDE | MC_CTRL_CURRENT | MC_CTRL_ENABLE);
        _foc_ref.id_target = 0;
        _foc_ref.iq_target = 0;
        _foc_ref.vd_target = 0;
        _foc_ref.vq_target = 0;
        _foc_ref.phase_override = 0 - M_PI;
        _foc_ref.openloop_spd = 0;
		d_axis = &_foc_ref.id_target;
	} else {
		_foc_ref.ctrl_mode = (MC_CTRL_OVERRIDE | MC_CTRL_ENABLE);
		_foc_ref.id_target = 0;
        _foc_ref.iq_target = 0;
        _foc_ref.vd_target = 0;
        _foc_ref.vq_target = 0;
        _foc_ref.phase_override = 0 - M_PI;
        _foc_ref.openloop_spd = 0;
		d_axis = &_foc_ref.vd_target;
	}

    Info_Debug("[lhall]Linear Hall wait current stable\n");
    for(int c = 0; *d_axis <= param; c++) {
        *d_axis += 0.01f;
        ipc_push(IPC_ID(foc_target), _foc_ref_pub, &_foc_ref);
        osDelay(10);
    }

    _sample_buffer.sample_count = 0;
    _sample_buffer.max_u[0] = 0.0f;
    _sample_buffer.max_u[1] = 0.0f;
    _sample_buffer.max_u[2] = 0.0f;
    _sample_buffer.min_u[0] = 3.3f;
    _sample_buffer.min_u[1] = 3.3f;
    _sample_buffer.min_u[2] = 3.3f;
    _sample_buffer.mid_u[0] = 0.0f;
    _sample_buffer.mid_u[1] = 0.0f;
    _sample_buffer.mid_u[2] = 0.0f;

    Info_Debug("[lhall]Linear Hall Calibration rotation\n");
    for (; _foc_ref.phase_override <= M_PI; _foc_ref.phase_override += 1.0e-3f) {
        ipc_push(IPC_ID(foc_target), _foc_ref_pub, &_foc_ref);
		osDelay(1);
        collect_sample();
    }
    osDelay(100);
    collect_sample();

    _foc_ref.ctrl_mode = MC_CTRL_IDLE;
    ipc_push(IPC_ID(foc_target), _foc_ref_pub, &_foc_ref);
    
    float ua, ub, uc;
    _sample_buffer.mid_u[0] = (_sample_buffer.max_u[0] - _sample_buffer.min_u[0]) / 2.0f + _sample_buffer.min_u[0];
    _sample_buffer.mid_u[1] = (_sample_buffer.max_u[1] - _sample_buffer.min_u[1]) / 2.0f + _sample_buffer.min_u[1];
    _sample_buffer.mid_u[2] = (_sample_buffer.max_u[2] - _sample_buffer.min_u[2]) / 2.0f + _sample_buffer.min_u[2];

    _hall_calibration._mida_handle = _sample_buffer.mid_u[0];
    _hall_calibration._midb_handle = _sample_buffer.mid_u[1];
    _hall_calibration._midc_handle = _sample_buffer.mid_u[2];

    ua = _sample_buffer.max_u[0] - _sample_buffer.mid_u[0];
    ub = _sample_buffer.max_u[1] - _sample_buffer.mid_u[1];
    uc = _sample_buffer.max_u[2] - _sample_buffer.mid_u[2];
    _hall_calibration._u_handle = ua;
    _hall_calibration._lamda_handle = ub/ua - 1.0f;
    _hall_calibration._beta_handle  = uc/ua - 1.0f;

    set_enc_parameter();

    send_status(ENC_CALIBRATE_SUCCESS);

    Info_Debug("[lhall]Linear Hall Calibration Success\n");

    char print[60];
    sprintf(print, "[lhall]A: max-%.4f,min-%.4f,mid-%.4f", _sample_buffer.max_u[0], _sample_buffer.min_u[0], _sample_buffer.mid_u[0]);
    Info_Debug("%s \n", print);
    sprintf(print, "[lhall]B: max-%.4f,min-%.4f,mid-%.4f", _sample_buffer.max_u[1], _sample_buffer.min_u[1], _sample_buffer.mid_u[1]);
    Info_Debug("%s \n", print);
    sprintf(print, "[lhall]C: max-%.4f,min-%.4f,mid-%.4f", _sample_buffer.max_u[2], _sample_buffer.min_u[2], _sample_buffer.mid_u[2]);
    Info_Debug("%s \n", print);
    
    return true;
}

void Enc_LHALL::collect_sample(void)
{
    struct encoder_s enc_raw;
    
    bool updated = false;
    ipc_check(_enc_sub, &updated);
    if(updated) {
        ipc_pull(IPC_ID(encoder), _enc_sub, &enc_raw);
        if(enc_raw.linear_hall[0] > _sample_buffer.max_u[0]) {
            _sample_buffer.max_u[0] = enc_raw.linear_hall[0];
        }
        if(enc_raw.linear_hall[1] > _sample_buffer.max_u[1]) {
            _sample_buffer.max_u[1] = enc_raw.linear_hall[1];
        }
        if(enc_raw.linear_hall[2] > _sample_buffer.max_u[2]) {
            _sample_buffer.max_u[2] = enc_raw.linear_hall[2];
        }

        if(enc_raw.linear_hall[0] < _sample_buffer.min_u[0]) {
            _sample_buffer.min_u[0] = enc_raw.linear_hall[0];
        }
        if(enc_raw.linear_hall[1] < _sample_buffer.min_u[1]) {
            _sample_buffer.min_u[1] = enc_raw.linear_hall[1];
        }
        if(enc_raw.linear_hall[2] < _sample_buffer.min_u[2]) {
            _sample_buffer.min_u[2] = enc_raw.linear_hall[2];
        }
        _sample_buffer.sample_count++;
    }
}

