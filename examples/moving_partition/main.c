#include "pogobase.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif

/* ================================================================================================
 * CONSTANTS & TYPEDEFS
 * ============================================================================================== */

/* ---  Motion --- */
static float max_speed_frac        = 0.4f;  /* 0–1: fraction of motorFull while RUNNING         */
static uint32_t run_duration_min   =  800;  /* [ms]                                             */
static uint32_t run_duration_max   = 4000;  /* [ms]                                             */
static uint32_t tumble_duration_min = 200;  /* [ms]                                             */
static uint32_t tumble_duration_max = 600;  /* [ms]                                             */

#define MS_TO_S(ms)  ((float)(ms) * 0.001f)

/* ---  Kuramoto oscillator  --- */
static float natural_freq_mean = 1.0f;      /* [rad s⁻¹]  – average ω₀                          */
static float natural_freq_std  = 0.1f;      /* [rad s⁻¹]  – per-robot Gaussian jitter           */
static float coupling_k        = 1.0f;      /* [rad s⁻¹]  – phase coupling gain                 */

/* θ - locomotion weak-coupling */
static bool  enable_locomotion_coupling = true;
static float osc_to_move_gain          = 0.25f;   /* θ → run/tumble timing (dimensionless) */
static float move_to_osc_gain          = 0.25f;   /* run/tumble → θ  (rad s⁻¹)             */

static float pushsum_gain = 0.5f;

/* ---  Diffusive scalar field for 2-domain partition  --- */
/* field ∈ [-1, 1], slow averaging with neighbours; sign(field) = nodal domain */
static float field_diffusion_alpha = 0.40f;   /* [1/s], local averaging strength    */
static float field_reaction_beta   = 1.0f;   /* [1/s], pushes |field| toward 1     */
static float field_global_gamma    = 2.5f;   /* [1/s], global balancing strength   */



/* ---  LED mode --- */
typedef enum {
    LED_MODE_SIGN,           // Red / blue, sign of partition field
    LED_MODE_RAINBOW_PHASE,  // Rainbow θ (phase)
    LED_MODE_PERIOD_COLOR    // Rainbow period
} led_mode_t;

static led_mode_t led_mode_enum = LED_MODE_SIGN;


/* ---  Messaging --- */
#define BROADCAST_HZ        30U
#define broadcast_period_ms (1000U / BROADCAST_HZ)   /* 33 ms */
#define MAX_NEIGHBORS       20U
#define IR_RANGE_MAX_AGE    200U    /* [ms] keep neighbour this long                 */

/* ================================================================================================
 * ENUM & STRUCTS
 * ============================================================================================== */

typedef enum { STATE_RUN = 0, STATE_TUMBLE = 1 } robot_state_t;

typedef struct {
    uint16_t id;                // Neighbour ID
    float    phase;             // Neighbour θ [rad]
    float    field;             // Neighbour partition field [-1, 1]
    float    ps_s;              // Push-sum s estimate of neighbor
    float    ps_w;              // Push-sum w estimate of neighbor
    uint32_t last_seen_ms;      // Timestamp
} neighbor_t;

/* ==== USERDATA ================================================================================= */

typedef struct {
    /* neighbour list */
    neighbor_t neighbors[MAX_NEIGHBORS];
    uint8_t    nb_neighbors;

    /* oscillator */
    float theta;                /* own phase θ [rad]      */
    float natural_freq;         /* ω₀  [rad s⁻¹]          */

    /* partition field */
    float field;                /* scalar partition field [-1, 1] */

    /* mobility FSM */
    robot_state_t fsm;
    uint32_t   timer_ms;
    bool       tumble_left;

    /* misc */
    uint32_t last_step_ms;
    uint32_t last_broadcast_ms;

    /* cached motor polarity (EEPROM) */
    uint8_t motor_dir_left;
    uint8_t motor_dir_right;

    /* Period computation (oscillator) */
    uint32_t  last_cross_time;   // [ms]
    float     period_est;        // [ms] for visualization

    /* Push-sum values */
    float ps_s;      // for the average of b_i
    float ps_w;      // for the average of 1
    float red_frac;  // local estimate of p = ps_s / ps_w
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

/* ================================================================================================
 * UTILS
 * ============================================================================================== */

static inline float rand_unitf(void) {
    return (float)rand() / (float)(RAND_MAX + 1UL);
}

static inline float random_uniform(float a, float b) {
    return a + (b - a) * rand_unitf();
}

static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

static inline void wrap_phase(float *theta) {
    /* wrap to (-π, π] for numerical stability */
    if (*theta > (float)M_PI) {
        *theta -= 2.0f * (float)M_PI;
    }
    if (*theta <= (-(float)M_PI)) {
        *theta += 2.0f * (float)M_PI;
    }
}

static inline uint8_t period_to_color(float T_ms) {
    /* map period (ms) to 0–255 for rainbow; adjust range as needed */
    const float T_min = 0.0f, T_max = 8000.0f;
    float clamped = fmaxf(T_min, fminf(T_ms, T_max));
    return (uint8_t)(255.0f * (clamped - T_min) / (T_max - T_min));
}

/* ================================================================================================
 * MOTION PRIMITIVES
 * ============================================================================================== */

static inline void motors_forward(float vfrac) {
    pogobot_motor_set(motorL, motorFull * vfrac);
    pogobot_motor_set(motorR, motorFull * vfrac);
}

static inline void motors_stop(void) {
    pogobot_motor_set(motorL, motorStop);
    pogobot_motor_set(motorR, motorStop);
}

static void choose_new_heading(void) {
    /* simple 50-50 chance to flip both polarities → random walk */
    bool flip = (rand() & 1U);
    pogobot_motor_dir_set(motorL, flip ? !mydata->motor_dir_left  : mydata->motor_dir_left);
    pogobot_motor_dir_set(motorR, flip ? !mydata->motor_dir_right : mydata->motor_dir_right);
}

static void tumble_pivot(bool left) {
    if (left) {
        pogobot_motor_set(motorL, motorStop);
        pogobot_motor_set(motorR, motorFull * max_speed_frac);
    } else {
        pogobot_motor_set(motorL, motorFull * max_speed_frac);
        pogobot_motor_set(motorR, motorStop);
    }
}

/* ================================================================================================
 * IR MESSAGING  (phase + partition field sharing)
 * ============================================================================================== */

typedef struct __attribute__((__packed__)) {
    uint16_t sender_id;
    int16_t  phase_q15;   /* q15 encoding of θ  ∈ (-π, π]  → (-32768, 32767] */
    int16_t  field_q15;   /* q15 encoding of field ∈ [-1, 1] */
    int16_t  ps_s_q15;   // encodes ps_s in [0,1]
    int16_t  ps_w_q15;   // encodes ps_w near 1
} phase_msg_t;

#define PHASE_MSG_SIZE  ((uint16_t)sizeof(phase_msg_t))

static int16_t float_phase_to_q15(float theta) {
    /* map (-π, π] → (-32768, 32767] */
    wrap_phase(&theta);
    return (int16_t)lroundf(theta / (float)M_PI * 32767.0f);
}

static float q15_to_float_phase(int16_t q) {
    return ((float)q) * (float)M_PI / 32767.0f;
}

static int16_t float_field_to_q15(float x) {
    float clamped = clampf(x, -1.0f, 1.0f);
    return (int16_t)lroundf(clamped * 32767.0f);
}

static float q15_to_float_field(int16_t q) {
    return ((float)q) / 32767.0f;
}

static int16_t float01_to_q15(float x) {
    float c = clampf(x, 0.0f, 1.0f);
    return (int16_t)lroundf(c * 32767.0f);
}

static float q15_to_float01(int16_t q) {
    return ((float)q) / 32767.0f;
}

static bool send_phase_msg(void) {
    uint32_t now = current_time_milliseconds();
    if (now - mydata->last_broadcast_ms < broadcast_period_ms) {
        return false;
    }

    phase_msg_t m = {
        .sender_id  = pogobot_helper_getid(),
        .phase_q15  = float_phase_to_q15(mydata->theta),
        .field_q15  = float_field_to_q15(mydata->field),
        .ps_s_q15   = float01_to_q15(mydata->ps_s),
        .ps_w_q15   = float01_to_q15(mydata->ps_w)
    };
    pogobot_infrared_sendShortMessage_omni((uint8_t *)&m, PHASE_MSG_SIZE);
    mydata->last_broadcast_ms = now;
    return true;
}

static void process_message(message_t *msg) {
    if (msg->header.payload_length < PHASE_MSG_SIZE) {
        return;
    }

    phase_msg_t const *pm = (phase_msg_t const *)msg->payload;
    uint16_t sender = pm->sender_id;
    if (sender == pogobot_helper_getid()) {
        return;
    }

    /* search / insert neighbour */
    uint8_t idx;
    for (idx = 0; idx < mydata->nb_neighbors; ++idx) {
        if (mydata->neighbors[idx].id == sender) {
            break;
        }
    }
    if (idx == mydata->nb_neighbors) {
        if (mydata->nb_neighbors >= MAX_NEIGHBORS) {
            return;
        }
        mydata->nb_neighbors++;
    }

    mydata->neighbors[idx].id           = sender;
    mydata->neighbors[idx].phase        = q15_to_float_phase(pm->phase_q15);
    mydata->neighbors[idx].field        = q15_to_float_field(pm->field_q15);
    mydata->neighbors[idx].last_seen_ms = current_time_milliseconds();
    mydata->neighbors[idx].ps_s         = q15_to_float01(pm->ps_s_q15);
    mydata->neighbors[idx].ps_w         = q15_to_float01(pm->ps_w_q15);
}

static void purge_old_neighbors(uint32_t now) {
    for (int8_t i = (int8_t)mydata->nb_neighbors - 1; i >= 0; --i) {
        if (now - mydata->neighbors[i].last_seen_ms > IR_RANGE_MAX_AGE) {
            mydata->neighbors[i] = mydata->neighbors[mydata->nb_neighbors - 1];
            mydata->nb_neighbors--;
        }
    }
}

static void update_pushsum(void) {
    uint8_t n = mydata->nb_neighbors;
    if (n == 0) {
        // Re-anchor ps_s, ps_w to local color slowly if isolated
        float b_i = (mydata->field > 0.0f) ? 1.0f : 0.0f;
        mydata->ps_s = 0.99f * mydata->ps_s + 0.01f * b_i;
        mydata->ps_w = 0.99f * mydata->ps_w + 0.01f * 1.0f;
    } else {
        float mean_s = 0.0f, mean_w = 0.0f;
        for (uint8_t i = 0; i < n; ++i) {
            mean_s += mydata->neighbors[i].ps_s;
            mean_w += mydata->neighbors[i].ps_w;
        }
        mean_s /= (float)n;
        mean_w /= (float)n;

        mydata->ps_s += pushsum_gain * (mean_s - mydata->ps_s);
        mydata->ps_w += pushsum_gain * (mean_w - mydata->ps_w);
    }

    float denom = (mydata->ps_w > 1e-3f) ? mydata->ps_w : 1e-3f;
    mydata->red_frac = clampf(mydata->ps_s / denom, 0.0f, 1.0f);
}


/* ================================================================================================
 * LED RENDERING  (partition + θ visualisation)
 * ============================================================================================== */

static void update_leds(void) {
    uint8_t r = 0, g = 0, b = 0;

    if (led_mode_enum == LED_MODE_SIGN) {
        /* sign(field) → red vs blue: 2-domain nodal partition */
        bool positive = (mydata->field >= 0.0f);
        if (positive) {
            r = 255; g = 0;   b = 0;
        } else {
            r = 0;   g = 0;   b = 255;
        }
    } else if (led_mode_enum == LED_MODE_RAINBOW_PHASE) {
        /* rainbow color for oscillator phase */
        float norm = fmodf(mydata->theta + 2.0f * (float)M_PI,
                           2.0f * (float)M_PI) / (2.0f * (float)M_PI);
        uint8_t val = (uint8_t)lroundf(norm * 255.0f);
        rainbow_colormap(val, &r, &g, &b);
    } else { /* LED_MODE_PERIOD_COLOR */
        uint8_t idx = period_to_color(mydata->period_est);
        rainbow_colormap(idx, &r, &g, &b);
    }

    /* centre LED = partition / phase visualization */
    pogobot_led_setColors(r, g, b, 0);

    /* Ring LED 1 shows motion state */
    switch (mydata->fsm) {
        case STATE_RUN:    r = 0;   g = 255; b = 0;   break;   /* green */
        case STATE_TUMBLE: r = 255; g = 160; b = 0;   break;   /* orange */
        default:           r = 255; g = 0;   b = 255; break;
    }
    pogobot_led_setColors(r, g, b, 1);
}

/* ================================================================================================
 * OSCILLATOR UPDATE
 * ============================================================================================== */

static void update_phase(float dt) {
    float prev_phase = mydata->theta;

    /* Kuramoto coupling with neighbours */
    float coupling_sum = 0.0f;
    uint8_t n = mydata->nb_neighbors;
    for (uint8_t i = 0; i < n; ++i) {
        coupling_sum += sinf(mydata->neighbors[i].phase - mydata->theta);
    }
    float coupling_term = (n > 0) ? (coupling_k / (float)n) * coupling_sum : 0.0f;
    mydata->theta += (mydata->natural_freq + coupling_term) * dt;

    if (enable_locomotion_coupling) {
        /* locomotion → oscillator: RUN = +1, TUMBLE = −1 */
        float loco_signal = (mydata->fsm == STATE_RUN) ? 1.0f : -1.0f;
        mydata->theta += move_to_osc_gain * loco_signal * dt;
    }

    wrap_phase(&mydata->theta);

    /* Period estimate: zero-crossing on θ from negative to positive */
    bool zero_crossing = (prev_phase < 0.0f && mydata->theta >= 0.0f);
    if (zero_crossing) {
        uint32_t now = current_time_milliseconds();
        float period_ms = (float)(now - mydata->last_cross_time);
        mydata->period_est = 0.8f * mydata->period_est + 0.2f * period_ms;
        mydata->last_cross_time = now;
    }
}

/* ================================================================================================
 * PARTITION FIELD UPDATE  (simple diffusion / consensus)
 * ============================================================================================== */

static void update_field(float dt) {
    uint8_t n = mydata->nb_neighbors;
    if (n == 0) {
        /* even alone, apply reaction + global term */
        float phi = mydata->field;

        /* double-well: dφ/dt = β(φ - φ³) */
        phi += field_reaction_beta * dt * (phi - phi * phi * phi);

        /* global balancing: dφ/dt = -γ * imbalance */
        float imbalance = 2.0f * mydata->red_frac - 1.0f;   // in [-1, 1]
        phi -= field_global_gamma * dt * imbalance;

        mydata->field = clampf(phi, -1.0f, 1.0f);
        return;
    }

    float self = mydata->field;

    /* 1) local diffusion */
    float mean = 0.0f;
    for (uint8_t i = 0; i < n; ++i) {
        mean += mydata->neighbors[i].field;
    }
    mean /= (float)n;

    float phi = self;
    float alpha_dt = field_diffusion_alpha * dt;
    if (alpha_dt > 1.0f) {
        alpha_dt = 1.0f;
    }
    phi += alpha_dt * (mean - self);

    /* 2) local double-well reaction: φ → ±1, 0 unstable */
    phi += field_reaction_beta * dt * (phi - phi * phi * phi);

    /* 3) global balancing using red_frac */
    float imbalance = 2.0f * mydata->red_frac - 1.0f;   // approximate ⟨sign(φ)⟩
    phi -= field_global_gamma * dt * imbalance;

    /* clamp */
    mydata->field = clampf(phi, -1.0f, 1.0f);

#if 0
    float flip_gain = 0.1f;
    float p_flip = flip_gain * fabsf(imbalance) * dt;  // e.g. flip_gain ~ 0.1–0.5

    if (p_flip > 0.0f) {
        float sign_local = (mydata->field >= 0.0f) ? 1.0f : -1.0f;
        float sign_majority = (imbalance >= 0.0f) ? 1.0f : -1.0f;

        // Only majority robots flip, with small probability
        if (sign_local == sign_majority && random_uniform(0.0f, 1.0f) < p_flip) {
            mydata->field = -mydata->field;  // flip color
        }
    }
#endif
}


/* ================================================================================================
 * RUN/TUMBLE SCHEDULING
 * ============================================================================================== */

static uint32_t draw_run_time_ms(void) {
    return (uint32_t)lroundf(random_uniform(run_duration_min, run_duration_max));
}

static uint32_t draw_tumble_time_ms(void) {
    return (uint32_t)lroundf(random_uniform(tumble_duration_min, tumble_duration_max));
}

/* ================================================================================================
 * INITIALISATION
 * ============================================================================================== */

static void user_init(void) {
    srand(pogobot_helper_getRandSeed());
    pogobot_infrared_set_power(2);

    memset(mydata, 0, sizeof(*mydata));

    /* motor polarity from EEPROM */
    uint8_t dir_mem[3];
    pogobot_motor_dir_mem_get(dir_mem);
    mydata->motor_dir_left  = dir_mem[1];
    mydata->motor_dir_right = dir_mem[0];

    /* oscillator params */
    float jitter = natural_freq_std * (rand_unitf() * 2.0f - 1.0f);
    mydata->natural_freq = natural_freq_mean + jitter;
    mydata->theta        = random_uniform(-(float)M_PI, (float)M_PI);

    /* partition field init: deterministic ±1 based on ID + small noise
       → roughly half robots in each domain, zero-mean global field */
    uint16_t id = pogobot_helper_getid();
    float base = (id & 1U) ? 1.0f : -1.0f;
    float noise = 0.1f * (rand_unitf() * 2.0f - 1.0f);  /* small jitter */
    mydata->field = clampf(base + noise, -1.0f, 1.0f);

    /* initial binary color */
    float b_i = (mydata->field > 0.0f) ? 1.0f : 0.0f;
    /* push-sum / average-consensus state */
    mydata->ps_s = b_i;
    mydata->ps_w = 1.0f;
    mydata->red_frac = b_i;    // initial guess

    /* FSM init */
    mydata->fsm         = STATE_TUMBLE;
    mydata->timer_ms    = 100;      /* tiny initial tumble */
    mydata->tumble_left = true;

    /* simulator / messaging */
    main_loop_hz                  = BROADCAST_HZ;  /* 30 Hz loop */
    msg_rx_fn                     = process_message;
    msg_tx_fn                     = send_phase_msg;
    max_nb_processed_msg_per_tick = 4;
    percent_msgs_sent_per_ticks   = 50;
    error_codes_led_idx           = 3;

    uint32_t now = current_time_milliseconds();
    mydata->last_step_ms      = now;
    mydata->last_broadcast_ms = now;
    mydata->last_cross_time   = now;
    mydata->period_est        = 0.0f;
}

/* ================================================================================================
 * MAIN CONTROL LOOP
 * ============================================================================================== */

static void user_step(void) {
    uint32_t now = current_time_milliseconds();
    uint32_t elapsed_ms = now - mydata->last_step_ms;
    mydata->last_step_ms = now;
    float dt = MS_TO_S(elapsed_ms);

    /* neighbour maintenance */
    purge_old_neighbors(now);

    /* oscillator update */
    update_phase(dt);

    // Update push-sum values
    update_pushsum();

    /* partition field update (simple diffusion) */
    update_field(dt);

    /* run/tumble FSM */
    if (mydata->timer_ms > elapsed_ms) {
        mydata->timer_ms -= elapsed_ms;
    } else {
        mydata->timer_ms = 0;
    }

    switch (mydata->fsm) {
        case STATE_RUN:
            motors_forward(max_speed_frac);
            if (mydata->timer_ms == 0) {
                mydata->tumble_left = (rand() & 1U);
                if (enable_locomotion_coupling) {
                    /* θ → locomotion: spend a bit longer tumbling around θ ≈ +π/2,
                       shorter around θ ≈ −π/2 (weak sine modulation, clamped) */
                    float factor = 1.0f + osc_to_move_gain * sinf(mydata->theta);
                    factor = clampf(factor, 0.5f, 1.5f);
                    mydata->timer_ms = (uint32_t)lroundf(draw_tumble_time_ms() * factor);
                } else {
                    mydata->timer_ms = draw_tumble_time_ms();
                }
                mydata->fsm = STATE_TUMBLE;
            }
            break;

        case STATE_TUMBLE:
        default:
            tumble_pivot(mydata->tumble_left);
            if (mydata->timer_ms == 0) {
                choose_new_heading();
                if (enable_locomotion_coupling) {
                    /* θ → locomotion: run longer when sin θ is negative,
                       shorter when positive (opposite sign to tumble) */
                    float factor = 1.0f - osc_to_move_gain * sinf(mydata->theta);
                    factor = clampf(factor, 0.5f, 1.5f);
                    mydata->timer_ms = (uint32_t)lroundf(draw_run_time_ms() * factor);
                } else {
                    mydata->timer_ms = draw_run_time_ms();
                }
                mydata->fsm = STATE_RUN;
            }
            break;
    }

    /* LED feedback & beacon */
    update_leds();
    send_phase_msg();
}

/* ================================================================================================
 * SIMULATOR CALLBACKS  (YAML config & data export)
 * ============================================================================================== */
#ifdef SIMULATOR
#include <strings.h>

static void global_setup(void) {
    init_from_configuration(max_speed_frac);
    init_from_configuration(run_duration_min);
    init_from_configuration(run_duration_max);
    init_from_configuration(tumble_duration_min);
    init_from_configuration(tumble_duration_max);

    init_from_configuration(natural_freq_mean);
    init_from_configuration(natural_freq_std);
    init_from_configuration(coupling_k);

    init_from_configuration(enable_locomotion_coupling);
    init_from_configuration(osc_to_move_gain);
    init_from_configuration(move_to_osc_gain);

    /* diffusion gain for partition field */
    init_from_configuration(field_diffusion_alpha);

    char led_mode[128] = "sign"; // Initialized with default value
    init_array_from_configuration(led_mode);
    if (strcasecmp(led_mode, "sign") == 0) {
        led_mode_enum = LED_MODE_SIGN;
    } else if (strcasecmp(led_mode, "phase") == 0) {
        led_mode_enum = LED_MODE_RAINBOW_PHASE;
    } else if (strcasecmp(led_mode, "period") == 0) {
        led_mode_enum = LED_MODE_PERIOD_COLOR;
    } else {
        printf("ERROR: unknown led_mode parameter value: '%s', "
               "use either 'sign', 'phase' or 'period'.\n", led_mode);
        exit(1);
    }
}

static void create_data_schema(void) {
    data_add_column_double("theta");
    data_add_column_int8("nb_neighbors");
    data_add_column_double("field");          /* partition scalar */
}

static void export_data(void) {
    data_set_value_double("theta", mydata->theta);
    data_set_value_int8("nb_neighbors", mydata->nb_neighbors);
    data_set_value_double("field", mydata->field);
}
#endif /* SIMULATOR */

/* ================================================================================================
 * ENTRY POINT
 * ============================================================================================== */

int main(void) {
    pogobot_init();

    pogobot_start(user_init, user_step);
#ifdef SIMULATOR
    SET_CALLBACK(callback_global_setup,       global_setup);
    SET_CALLBACK(callback_create_data_schema, create_data_schema);
    SET_CALLBACK(callback_export_data,        export_data);
#endif
    return 0;
}

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
