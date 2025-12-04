/**
 * @file
 *
 * @brief
 * Voter-model opinion dynamics on a swarm of Pogobot robots,
 * with optional physical motion and LED-based visualization.
 *
 * @details
 * This example implements the classical **Voter model** of opinion
 * dynamics on a swarm of Pogobot robots using the same Pogosim API and
 * run-and-tumble motion primitives as the Axelrod example.
 *
 * Each robot carries a single discrete "opinion" variable:
 *
 *   - Opinion of robot *i*:
 *       s_i ∈ {0, ..., S−1},
 *
 * where S = #NUM_OPINIONS is a small integer (e.g. S = 4). Robots repeatedly
 * exchange their current opinion with nearby neighbors using omnidirectional
 * infrared communication. The neighbor set N_i(t) of robot *i* at time *t* is
 * defined by IR visibility within a recent time window.
 *
 * The (asynchronous) Voter-update rule is:
 *
 *   1. For each robot *i*, if N_i(t) ≠ ∅, pick a random neighbor
 *      j ∈ N_i(t).
 *
 *   2. If s_j ≠ s_i, then robot *i* adopts the neighbor’s opinion:
 *
 *         s_i ← s_j.
 *
 *      Otherwise, if s_j = s_i, nothing happens for this update.
 *
 * In the classical (continuous-time) Voter model, updates are asynchronous:
 * at random times, a single site *i* is selected and updates by copying a
 * random neighbor’s state. Here, this is approximated by letting each robot
 * perform one such "copy-from-a-random-neighbor" step at each control-loop
 * iteration, using the latest neighbor information available.
 *
 * Interaction network:
 *
 *   - Static random geometric graph if #moving_robots == false:
 *       Robots remain fixed in space and edges are defined by IR visibility.
 *
 *   - Time-varying random geometric graph if #moving_robots == true:
 *       Robots perform run-and-tumble motion; the neighbor sets N_i(t)
 *       change over time as robots move through the arena.
 *
 * This allows one to compare the same Voter dynamics on static vs.
 * mobile interaction networks.
 *
 * LED visualization:
 *
 *   - Center LED (index 0): encodes the robot’s current opinion s_i.
 *
 *       We map the discrete opinion s_i ∈ {0, …, S−1} to a color index:
 *
 *         idx_i = round( 255 · s_i / (S − 1) ),
 *
 *       and use #rainbow_colormap(idx_i, …) to obtain an RGB color.
 *       Robots sharing the same opinion have identical center-LED colors.
 *
 *   - Ring LED (index 1): indicates the local **consensus** state:
 *
 *       * GREEN  if all currently visible neighbors share the same
 *                 opinion as the robot (local consensus),
 *       * RED    if at least one neighbor holds a different opinion
 *                 (locally mixed neighborhood).
 *
 * Key configuration parameters (typically set via YAML in Pogosim):
 *
 *   - #NUM_OPINIONS      : number of possible opinions S.
 *   - #moving_robots     : if true, robots perform run-and-tumble motion;
 *                          if false, robots remain immobile.
 *   - #max_speed_frac    : fraction of maximum motor speed during RUN.
 *   - #run_duration_min/max_ms
 *   - #tumble_duration_min/max_ms
 *
 * The Voter model is a paradigmatic stochastic spin system for consensus
 * formation and coarsening on graphs. On finite connected graphs, it almost
 * surely reaches an absorbing consensus state (all robots share the same
 * opinion), although the time to reach consensus can be very large
 * depending on the network structure.
 *
 * Scientific references (Voter model):
 *
 *   - Clifford, P. & Sudbury, A. (1973).
 *     "A model for spatial conflict."
 *     Biometrika, 60(3), 581–588.
 *
 *   - Holley, R. A. & Liggett, T. M. (1975).
 *     "Ergodic theorems for weakly interacting infinite systems
 *      and the voter model."
 *     The Annals of Probability, 3(4), 643–663.
 *
 *   - Castellano, C., Fortunato, S., & Loreto, V. (2009).
 *     "Statistical physics of social dynamics."
 *     Reviews of Modern Physics, 81(2), 591–646.
 *
 * This example mirrors the structure of the Axelrod-cultural-dissemination
 * example for Pogobots, but replaces multi-feature cultures by a single
 * multi-valued opinion and uses the Voter copying rule instead of
 * Axelrod’s homophily-based interaction.
 */

#include "pogobase.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

/* ================================================================================================
 * CONSTANTS & TYPEDEFS
 * ============================================================================================== */

/* --- Voter-model parameters --- */
#define NUM_OPINIONS   4       /**< Number of possible opinions S (states) */

/* --- Motion --- */
static float    max_speed_frac        = 0.4f;   /* 0–1: fraction of motorFull while RUNNING */
static uint32_t run_duration_min      =  800;   /* [ms] */
static uint32_t run_duration_max      = 4000;   /* [ms] */
static uint32_t tumble_duration_min   =  200;   /* [ms] */
static uint32_t tumble_duration_max   =  600;   /* [ms] */

/** Whether robots actually move (run-and-tumble) or stay immobile. */
static bool     moving_robots         = true;

#define MS_TO_S(ms)  ((float)(ms) * 0.001f)

/* --- Messaging --- */
#define BROADCAST_HZ        10U
#define broadcast_period_ms (1000U / BROADCAST_HZ)
#define MAX_NEIGHBORS       20U
#define IR_RANGE_MAX_AGE    200U    /* [ms] keep neighbour for this long */

/* ================================================================================================
 * ENUM & STRUCTS
 * ============================================================================================== */

typedef enum { STATE_RUN = 0, STATE_TUMBLE = 1 } robot_state_t;

typedef struct {
    uint16_t id;
    uint8_t  opinion;
    uint32_t last_seen_ms;
} neighbor_t;

/* ==== USERDATA ================================================================================= */

typedef struct {
    /* neighbour list */
    neighbor_t neighbors[MAX_NEIGHBORS];
    uint8_t    nb_neighbors;

    /* own opinion */
    uint8_t opinion;

    /* mobility FSM */
    robot_state_t fsm;
    uint32_t      timer_ms;
    bool          tumble_left;

    /* misc timing */
    uint32_t last_step_ms;
    uint32_t last_broadcast_ms;

    /* cached motor polarity (EEPROM) */
    uint8_t motor_dir_left;
    uint8_t motor_dir_right;

    /* cached metrics */
    bool    locally_consensual;        /**< true if all neighbours share our opinion */
    uint8_t nb_disagreeing_neighbors;  /**< number of neighbours with different opinion */
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

static inline uint32_t random_uint32(uint32_t max_exclusive) {
    if (max_exclusive == 0U) { return 0U; }
    return (uint32_t)(rand_unitf() * (float)max_exclusive);
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

/* Simple random change of direction (flip both polarities with 50% chance). */
static void choose_new_heading(void) {
    bool flip = (rand() & 1U) != 0;
    pogobot_motor_dir_set(motorL, flip ? !mydata->motor_dir_left  : mydata->motor_dir_left);
    pogobot_motor_dir_set(motorR, flip ? !mydata->motor_dir_right : mydata->motor_dir_right);
}

/* Pivot on the spot during a TUMBLE phase. */
static void tumble_pivot(bool left) {
    if (left) {
        pogobot_motor_set(motorL, motorStop);
        pogobot_motor_set(motorR, motorFull * max_speed_frac);
    } else {
        pogobot_motor_set(motorL, motorFull * max_speed_frac);
        pogobot_motor_set(motorR, motorStop);
    }
}

static uint32_t draw_run_time_ms(void) {
    return (uint32_t)lroundf(random_uniform((float)run_duration_min,
                                            (float)run_duration_max));
}

static uint32_t draw_tumble_time_ms(void) {
    return (uint32_t)lroundf(random_uniform((float)tumble_duration_min,
                                            (float)tumble_duration_max));
}

/* ================================================================================================
 * IR MESSAGING  (opinion sharing)
 * ============================================================================================== */

typedef struct __attribute__((__packed__)) {
    uint16_t sender_id;
    uint8_t  opinion;
} voter_msg_t;

#define VOTER_MSG_SIZE  ((uint16_t)sizeof(voter_msg_t))

static bool send_voter_msg(void) {
    uint32_t now = current_time_milliseconds();
    if (now - mydata->last_broadcast_ms < broadcast_period_ms) { return false; }

    voter_msg_t m;
    m.sender_id = pogobot_helper_getid();
    m.opinion   = mydata->opinion;

    pogobot_infrared_sendShortMessage_omni((uint8_t *)&m, VOTER_MSG_SIZE);
    mydata->last_broadcast_ms = now;
    return true;
}

static void process_message(message_t *msg) {
    if (msg->header.payload_length < VOTER_MSG_SIZE) { return; }

    voter_msg_t const *vm = (voter_msg_t const *)msg->payload;
    uint16_t sender = vm->sender_id;
    if (sender == pogobot_helper_getid()) { return; }

    /* search / insert neighbour */
    uint8_t idx;
    for (idx = 0; idx < mydata->nb_neighbors; ++idx) {
        if (mydata->neighbors[idx].id == sender) { break; }
    }
    if (idx == mydata->nb_neighbors) {
        if (mydata->nb_neighbors >= MAX_NEIGHBORS) { return; }
        mydata->nb_neighbors++;
    }

    neighbor_t *n = &mydata->neighbors[idx];
    n->id           = sender;
    n->last_seen_ms = current_time_milliseconds();
    n->opinion      = vm->opinion;
}

static void purge_old_neighbors(uint32_t now) {
    for (int8_t i = (int8_t)mydata->nb_neighbors - 1; i >= 0; --i) {
        if (now - mydata->neighbors[i].last_seen_ms > IR_RANGE_MAX_AGE) {
            mydata->neighbors[i] = mydata->neighbors[mydata->nb_neighbors - 1];
            mydata->nb_neighbors--;
        }
    }
}

/* ================================================================================================
 * VOTER DYNAMICS
 * ============================================================================================== */

/**
 * @brief Perform one asynchronous Voter-model update for this robot.
 *
 * If there is at least one neighbour, we pick one at random and copy its
 * opinion (if different) with probability 1. If there is no neighbour,
 * nothing happens.
 *
 * This implements the rule s_i ← s_j for a uniformly random neighbour j.
 */
static void voter_update_step(void) {
    if (mydata->nb_neighbors == 0U) { return; }

    uint8_t idx = (uint8_t)random_uint32(mydata->nb_neighbors);
    neighbor_t const *n = &mydata->neighbors[idx];

    if (n->opinion != mydata->opinion) {
        mydata->opinion = n->opinion;
    }
}

/**
 * @brief Recompute local consensus statistics.
 *
 * Sets:
 *   - mydata->locally_consensual = true if all neighbours share our opinion.
 *   - mydata->nb_disagreeing_neighbors = number of neighbours with a different opinion.
 */
static void update_local_consensus(void) {
    bool consensual = true;
    uint8_t nb_disagree = 0;

    for (uint8_t i = 0; i < mydata->nb_neighbors; ++i) {
        if (mydata->neighbors[i].opinion != mydata->opinion) {
            consensual = false;
            nb_disagree++;
        }
    }

    mydata->locally_consensual       = consensual;
    mydata->nb_disagreeing_neighbors = nb_disagree;
}

/* ================================================================================================
 * LED RENDERING  (opinion visualisation)
 * ============================================================================================== */

/**
 * @brief Map opinion -> [0,255] for rainbow_colormap.
 */
static uint8_t opinion_to_color_index(uint8_t opinion) {
    if (NUM_OPINIONS <= 1U) { return 0; }

    if (opinion >= NUM_OPINIONS) {
        opinion %= NUM_OPINIONS;
    }

    uint8_t idx = (uint8_t)((255U * (uint32_t)opinion) / (NUM_OPINIONS - 1U));
    return idx;
}

static void update_leds(void) {
    uint8_t r = 0, g = 0, b = 0;

    /* Centre LED: opinion colour */
    uint8_t idx = opinion_to_color_index(mydata->opinion);
    rainbow_colormap(idx, &r, &g, &b);
    pogobot_led_setColors(r, g, b, 0);  /* centre LED */

    /* Ring LED 1: local consensus indicator */
    if (mydata->locally_consensual) {
        /* green = all neighbours share our opinion */
        r = 0; g = 255; b = 0;
    } else {
        /* red = at least one neighbour disagrees */
        r = 255; g = 0; b = 0;
    }
    pogobot_led_setColors(r, g, b, 1);
}

/* ================================================================================================
 * RUN/TUMBLE SCHEDULING
 * ============================================================================================== */

static void update_motion(uint32_t elapsed_ms) {
    if (!moving_robots) {
        motors_stop();
        return;
    }

    if (mydata->timer_ms > elapsed_ms) {
        mydata->timer_ms -= elapsed_ms;
    } else {
        mydata->timer_ms = 0;
    }

    switch (mydata->fsm) {
        case STATE_RUN:
            motors_forward(max_speed_frac);
            if (mydata->timer_ms == 0U) {
                mydata->tumble_left = (rand() & 1U) != 0;
                mydata->timer_ms    = draw_tumble_time_ms();
                mydata->fsm         = STATE_TUMBLE;
            }
            break;

        case STATE_TUMBLE:
        default:
            tumble_pivot(mydata->tumble_left);
            if (mydata->timer_ms == 0U) {
                choose_new_heading();
                mydata->timer_ms = draw_run_time_ms();
                mydata->fsm      = STATE_RUN;
            }
            break;
    }
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

    /* random initial opinion */
    mydata->opinion = (uint8_t)random_uint32(NUM_OPINIONS);

    /* initial metrics */
    mydata->locally_consensual       = false;
    mydata->nb_disagreeing_neighbors = 0;

    /* FSM init */
    mydata->fsm         = STATE_TUMBLE;
    mydata->timer_ms    = 100;
    mydata->tumble_left = true;

    /* simulator / messaging */
    main_loop_hz                  = BROADCAST_HZ;
    msg_rx_fn                     = process_message;
    msg_tx_fn                     = send_voter_msg;
    max_nb_processed_msg_per_tick = 4;
    percent_msgs_sent_per_ticks   = 50;
    error_codes_led_idx           = 3;

    mydata->last_step_ms      = current_time_milliseconds();
    mydata->last_broadcast_ms = current_time_milliseconds();
}

/* ================================================================================================
 * MAIN CONTROL LOOP
 * ============================================================================================== */

static void user_step(void) {
    uint32_t now        = current_time_milliseconds();
    uint32_t elapsed_ms = now - mydata->last_step_ms;
    mydata->last_step_ms = now;

    /* neighbour maintenance */
    purge_old_neighbors(now);

    /* Voter update (opinion dynamics) */
    voter_update_step();

    /* Recompute local consensus metrics */
    update_local_consensus();

    /* Update motion (or keep robots immobile if moving_robots == false) */
    update_motion(elapsed_ms);

    /* LED feedback & opinion beacon */
    update_leds();
    send_voter_msg();
}

/* ================================================================================================
 * SIMULATOR CALLBACKS  (YAML config & data export)
 * ============================================================================================== */
#ifdef SIMULATOR
#include <strings.h>

static void global_setup(void) {
    /* Motion configuration */
    init_from_configuration(max_speed_frac);
    init_from_configuration(run_duration_min);
    init_from_configuration(run_duration_max);
    init_from_configuration(tumble_duration_min);
    init_from_configuration(tumble_duration_max);

    init_from_configuration(moving_robots);
}

static void create_data_schema(void) {
    /* Export basic diagnostics for analysis */
    data_add_column_int8("nb_neighbors");
    data_add_column_int8("locally_consensual");
    data_add_column_int8("nb_disagreeing_neighbors");
    data_add_column_int16("opinion");
}

static void export_data(void) {
    data_set_value_int8("nb_neighbors",             mydata->nb_neighbors);
    data_set_value_int8("locally_consensual",       mydata->locally_consensual ? 1 : 0);
    data_set_value_int8("nb_disagreeing_neighbors", mydata->nb_disagreeing_neighbors);
    data_set_value_int16("opinion",                 (int16_t)mydata->opinion);
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
