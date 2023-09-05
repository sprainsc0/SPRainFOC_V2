#ifndef __IND_CAL_H__
#define __IND_CAL_H__

#include "ipccore.h"
#include <param.h>

#include "topics/parameter_update.h"
#include "topics/foc_target.h"
#include "topics/foc_status.h"
#include "topics/hfi_status.h"
#include "topics/actuator_notify.h"
#include "topics/calibrate_status.h"

class IND_Cal
{
public:
    IND_Cal(orb_advert_t &cal_status_pub);
    ~IND_Cal(void);
    
    bool do_calibration(float duty, int samples);

    void send_status(uint8_t status);
    
private:
    
    // ipc value
    int _params_sub;
    int _foc_status_sub;
    int _hfi_status_sub;

    orb_advert_t _foc_ref_pub;
    struct foc_target_s _foc_ref;

    orb_advert_t &_mavlink_cal_pub;
    struct calibrate_status_s _cal_status;

    uint8_t foc_mode;
    
    struct {
        param_t foc_sample_v0_v7_handle;
        param_t motor_ind_handle;
        param_t ind_diff_handle;
        param_t sensor_type_handle;
        param_t hfi_sample_handle;

        param_t hfi_voltage_start_handle;
        param_t hfi_voltage_run_handle;
        param_t hfi_voltage_max_handle;
    } _cal_params_handles;

    struct encoder_calibration_s {
        int   foc_sample_v0_v7;
		float   motor_ind;
        float   ind_diff;
        int     sensor_type;
        int     hfi_sample;
        float hfi_voltage_start;
        float hfi_voltage_run;
        float hfi_voltage_max;
	} _ind_calibration, _ind_cfg_old;
    
    void set_ind_parameter(void);
    void set_hfi_parameter(void);
    void get_ind_parameter(void);
};

#endif