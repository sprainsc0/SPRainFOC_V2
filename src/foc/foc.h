#ifndef MC_FOC_H
#define MC_FOC_H

#include <datatypes.h>
#include <stdint.h>
#include "cmsis_os.h"
#include <string.h>
#include "uPerf.h"
#include "pid/pid.h"
#include "ipccore.h"
#include "param.h"
#include "td.h"
#include "pll.h"

#include "topics/parameter_update.h"
#include "topics/actuator_notify.h"
#include "topics/encoder.h"
#include "topics/foc_status.h"
#include "topics/foc_target.h"

#define CURRENT_RATE_DT                       0.00008f

class FOC
{
public:
    FOC(void);
    ~FOC(void);

    void *_param;
    void run(void *parameter);

    bool init(void);
    void foc_process(void);

    void parameter_update(bool force);

    void dc_calibration(void);

    bool mosfet_check(void);

protected:
    osThreadId_t _handle;

private:
    static bool _power_state;
    uint8_t     _pre_foc_mode;
    bool        _calibration_ok;

    struct hfi_inject {
        uint32_t pwm_inject[3];
        bool duty_injected;
    } _hfi_inj;
    
    struct motor_config {
        int   foc_sample_v0_v7;
        float current_offset[3];
        float voltage_offset[3];
        float voltage_offset_undriven[3];
        float duty_max;

        float curr_d_p;
        float curr_d_i;
        float curr_q_p;
        float curr_q_i;

        int   sensor_type;
        int   decoupling_type;
        float motor_r;
        float motor_l;
        float flux_linkage;
        int   direct_inv;

        float l_current_max;
    } _mc_cfg;

    struct {
        param_t foc_sample_v0_v7_handle;
        param_t duty_max_handle;

        param_t curr_d_p_handle;
        param_t curr_d_i_handle;
        param_t curr_q_p_handle;
        param_t curr_q_i_handle;

        param_t sensor_type_handle;
        param_t decoupling_type_handle;
        param_t motor_r_handle;
        param_t motor_l_handle;
        param_t flux_linkage_handle;
        param_t direct_inv_handle;

        param_t l_current_max_handle;
    } _param_handles;

    static constexpr float    _current_filter_gain = 0.1f;
    static constexpr float    _foc_dt_us           = 0.59523f; // dead time us
    static constexpr uint32_t _foc_f_sw            = 25000;
    static constexpr float    _low_spd_limited     = 1200.0f; // 
    static constexpr float    _const_tc            = 0.001f; // s
    static constexpr float    _sl_d_current_duty   = 0.0f;
    static constexpr float    _sl_d_current_factor = 0.0f;
    // 参考电压
    uint16_t _refint;
    float    _openloop_rad;
    bool     _drv_fault;

    int _params_sub;

    // int _encoder_sub;
    struct encoder_s _encoder_data;
    bool last_enc_healthy;

    int _foc_target_sub;
    struct foc_target_s _foc_ref;

    orb_advert_t _foc_status_pub;
    struct foc_status_s _foc_m;

    orb_advert_t _led_pub;
    struct actuator_notify_s _led_state;

    PID _id_ctrl;
    PID _iq_ctrl;
    PID _duty_ctrl;

    TD  _td;
    PLL _pll;

    void svm(float alpha, float beta, uint32_t PWMHalfPeriod,
		uint32_t* tAout, uint32_t* tBout, uint32_t* tCout, uint32_t *svm_sector);

    void process_temperature(void);
    
    perf_counter_t foc_adc_int;
    perf_counter_t foc_adc_ela;

    perf_counter_t foc_task_int;
    perf_counter_t foc_task_ela;
};

#endif
