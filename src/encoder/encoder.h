#ifndef MC_ENCODER_H
#define MC_ENCODER_H

#include <string.h>

#include <stdint.h>
#include "cmsis_os.h"
#include "datatypes.h"
#include "uPerf.h"
#include "param.h"

#include "topics/parameter_update.h"
#include "topics/encoder.h"

class Encoder
{
public:
    Encoder(void);
    ~Encoder(void);

    void *_param;
    void run(void *parameter);

    bool init(void);
    void enc_process(void);

    void refint(uint16_t ref) { _refint = ref; }

    void parameter_update(bool force);

    struct encoder_s get_encoder(void) const { return _enc_data; }

protected:
    osThreadId_t _handle;

private:
    static constexpr float sin_alpha = 0.5f;
    static constexpr float cos_alpha = SQRT3_BY_2;

	struct enc_config {
        int   type;
        float offset_e;
        float offset_m;
		int   invert_e;
		int   invert_m;
		int   pair_num;

        float lhall_u;
		float lamda;
		float beta;
        float mida;
        float midb;
        float midc;
    } _enc_cfg;

	struct {
        param_t type_handle;
        param_t offset_e_handle;
        param_t offset_m_handle;
		param_t invert_e_handle;
        param_t invert_m_handle;
		param_t pair_num_handle;

        param_t u_handle;
		param_t lamda_handle;
		param_t beta_handle;
        param_t mida_handle;
        param_t midb_handle;
        param_t midc_handle;
    } _param_handles;

	int _params_sub;

    float elec_angle;
    float raw_angle;
    float pll_angle;
    float mech_angle;
    float internal_dt;

    float hall_data_correct[3];

	orb_advert_t _encoder_pub;
    struct encoder_s _enc_data;

    // 参考电压
    uint16_t _refint;

    bool _enc_ready;
    bool _enc_reset;

	perf_counter_t enc_tim_int;
    perf_counter_t enc_tim_ela;

    perf_counter_t enc_task_int;
    perf_counter_t enc_task_ela;

    perf_counter_t enc_err_count;
};

#endif
