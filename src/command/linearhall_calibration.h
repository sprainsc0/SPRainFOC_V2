#ifndef __LHALL_CAL_H__
#define __LHALL_CAL_H__

#include "ipccore.h"
#include <param.h>

#include "topics/parameter_update.h"
#include "topics/encoder.h"
#include "topics/foc_target.h"
#include "topics/foc_status.h"
#include "topics/actuator_notify.h"
#include "topics/calibrate_status.h"

class Enc_LHALL
{
public:
    Enc_LHALL(orb_advert_t &cal_status_pub);
    ~Enc_LHALL(void);
    
    bool do_calibration(float param, uint8_t use_current);

    void send_status(uint8_t status);
    
private:

    struct HallSample {
        float max_u[3];
        float min_u[3];
        float mid_u[3];
        uint16_t sample_count;
    };
    
    // ipc value
    int _params_sub;
    int _enc_sub;
    int _foc_status_sub;

    orb_advert_t _foc_ref_pub;
    struct foc_target_s _foc_ref;

    orb_advert_t &_mavlink_cal_pub;
    struct calibrate_status_s _cal_status;

    struct HallSample _sample_buffer;

    uint8_t foc_mode;
    
    struct {
        param_t u_handle;
		param_t lamda_handle;
		param_t beta_handle;

        param_t mida_handle;
        param_t midb_handle;
        param_t midc_handle;
    } _cal_params_handles;

    struct encoder_calibration_s {
		float _u_handle;
		float _lamda_handle;
		float _beta_handle;

        float _mida_handle;
        float _midb_handle;
        float _midc_handle;
	} _hall_calibration;
    
    void set_enc_parameter(void);
    void collect_sample(void);
};

#endif