/**
 * @file
 *
 * @brief
 * Axelrod cultural dissemination model implemented on a swarm of Pogobot robots,
 * with optional physical motion and on-board LED visualization.
 *
 * @details
 * This example implements a variant of the Axelrod model of cultural emergence
 * on a swarm of Pogobot robots. Each robot carries a discrete "culture" vector
 * of F features, each feature taking one of Q possible traits:
 *
 *   - Culture of robot i:
 *       σ_i = (σ_i^(1), ..., σ_i^(F)),  with  σ_i^(f) ∈ {0, ..., Q-1}.
 *
 * Robots repeatedly exchange their culture vectors with nearby neighbors
 * using omnidirectional infrared communication. The set of neighbors N_i(t) of
 * robot i at time t is defined by the IR communication range and (optionally)
 * by the robot motion: two robots are neighbors if they have exchanged an IR
 * message within a recent time window.
 *
 * The cultural dynamics follows the original Axelrod interaction rule:
 *
 *   1. For each robot i, pick a random neighbor j ∈ N_i(t).
 *   2. Compute the cultural overlap:
 *
 *          overlap_ij = Σ_f I(σ_i^(f) == σ_j^(f)),      where I(...) is 0/1.
 *
 *      This is the number of shared features between i and j (0 ≤ overlap_ij ≤ F).
 *
 *   3. If overlap_ij == 0 or the two culture vectors are identical (no differing
 *      features), no interaction occurs.
 *
 *   4. Otherwise, define the interaction probability:
 *
 *          p_ij = overlap_ij / F.
 *
 *      With probability p_ij, robot i chooses uniformly at random one feature
 *      f* on which i and j differ, and copies that feature from j:
 *
 *          σ_i^(f*) <- σ_j^(f*).
 *
 * This rule implements "homophily" (agents interact only with somewhat similar
 * neighbors) and "social influence" (interaction increases similarity), which
 * are the two key ingredients of the Axelrod model.
 *
 * The interaction network is:
 *
 *   - Static random geometric graph if moving_robots == false:
 *       Robots are fixed in space and edges are defined by IR visibility.
 *
 *   - Time-varying random geometric graph if moving_robots == true:
 *       Robots perform run-and-tumble motion; the neighbor sets N_i(t) evolve
 *       over time as robots move through the arena.
 *
 * This allows one to compare Axelrod dynamics on a fixed vs. mobile interaction
 * network using the same code.
 *
 * LED visualization:
 *
 *   - Center LED (index 0): encodes the full culture vector of the robot.
 *     The F-dimensional culture σ_i is first mapped to an integer "code"
 *     using a base-Q expansion:
 *
 *         code_i = Σ_{f=1..F} σ_i^(f) * Q^(f-1).
 *
 *     This integer is then normalized to [0, 255] and passed to a rainbow
 *     colormap:
 *
 *         idx_i ∈ {0, …, 255}  →  (R_i, G_i, B_i) via rainbow_colormap().
 *
 *     Thus, two robots have exactly the same center-LED color if and only if
 *     they share the same culture vector; different cultures almost always
 *     appear with different colors.
 *
 *   - Ring LED (index 1): indicates whether the robot is still "culturally
 *     active" or locally frozen:
 *
 *       * RED  if there exists at least one neighbor j such that:
 *           0 < overlap_ij < F
 *         i.e., i and j are partially similar but not identical, so further
 *         Axelrod interactions are still possible.
 *
 *       * GREEN if no such neighbor exists, i.e., for all neighbors j either:
 *           overlap_ij == 0   (no similarity, no interaction)
 *         or
 *           overlap_ij == F   (identical culture),
 *         indicating that robot i is culturally frozen with respect to its
 *         current neighborhood.
 *
 * Key configuration parameters (typically set via YAML in Pogosim):
 *
 *   - CULTURE_F          : number of cultural features F per robot.
 *   - CULTURE_Q          : number of traits Q per feature.
 *   - moving_robots      : if true, robots perform run-and-tumble motion;
 *                          if false, robots remain immobile.
 *   - max_speed_frac     : fraction of maximum motor speed during RUN.
 *   - run_duration_*     : min/max duration of RUN phases (ms).
 *   - tumble_duration_*  : min/max duration of TUMBLE phases (ms).
 *
 * Scientific reference:
 *
 *   The model implemented here is based on:
 *
 *     Axelrod, R. (1997).
 *     "The dissemination of culture: A model with local convergence and
 *      global polarization."
 *     Journal of Conflict Resolution, 41(2), 203–226.
 *
 * In this example, we reproduce Axelrod’s local interaction rule on a swarm
 * of physical / simulated robots, and extend it to time-varying interaction
 * networks due to robot motion, with real-time LED-based visualization of
 * cultural groups and local cultural activity.
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

/* --- Axelrod culture parameters --- */
#define CULTURE_F   5       /* number of cultural features per robot   */
#define CULTURE_Q   12      /* number of possible traits per feature   */

/* --- Motion --- */
static float    max_speed_frac        = 0.4f;   /* 0–1: fraction of motorFull while RUNNING */
static uint32_t run_duration_min      =  800;   /* [ms] */
static uint32_t run_duration_max      = 4000;   /* [ms] */
static uint32_t tumble_duration_min   =  200;   /* [ms] */
static uint32_t tumble_duration_max   =  600;   /* [ms] */

/* Whether robots actually move (run-and-tumble) or stay immobile. */
static bool     moving_robots       = true;

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
    uint8_t  features[CULTURE_F];
    uint32_t last_seen_ms;
} neighbor_t;

/* ==== USERDATA ================================================================================= */

typedef struct {
    /* neighbour list */
    neighbor_t neighbors[MAX_NEIGHBORS];
    uint8_t    nb_neighbors;

    /* own culture */
    uint8_t features[CULTURE_F];

    /* mobility FSM */
    robot_state_t fsm;
    uint32_t   timer_ms;
    bool       tumble_left;

    /* misc timing */
    uint32_t last_step_ms;
    uint32_t last_broadcast_ms;

    /* cached motor polarity (EEPROM) */
    uint8_t motor_dir_left;
    uint8_t motor_dir_right;

    /* cached metrics */
    bool     culturally_stable;   /* true iff no “active” links with neighbours */
    uint16_t culture_hash;        /* hashed ID of cultural group */
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

static uint32_t draw_run_time_ms(void) {
    return (uint32_t)lroundf(random_uniform((float)run_duration_min,
                                            (float)run_duration_max));
}

static uint32_t draw_tumble_time_ms(void) {
    return (uint32_t)lroundf(random_uniform((float)tumble_duration_min,
                                            (float)tumble_duration_max));
}

/* ================================================================================================
 * IR MESSAGING  (culture sharing)
 * ============================================================================================== */

typedef struct __attribute__((__packed__)) {
    uint16_t sender_id;
    uint8_t  features[CULTURE_F];
} culture_msg_t;

#define CULTURE_MSG_SIZE  ((uint16_t)sizeof(culture_msg_t))

static bool send_culture_msg(void) {
    uint32_t now = current_time_milliseconds();
    if (now - mydata->last_broadcast_ms < broadcast_period_ms) { return false; }

    culture_msg_t m;
    m.sender_id = pogobot_helper_getid();
    memcpy(m.features, mydata->features, sizeof(m.features));

    pogobot_infrared_sendShortMessage_omni((uint8_t *)&m, CULTURE_MSG_SIZE);
    mydata->last_broadcast_ms = now;
    return true;
}

static void process_message(message_t *msg) {
    if (msg->header.payload_length < CULTURE_MSG_SIZE) { return; }

    culture_msg_t const *cm = (culture_msg_t const *)msg->payload;
    uint16_t sender = cm->sender_id;
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
    memcpy(n->features, cm->features, sizeof(n->features));
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
 * AXELROD DYNAMICS
 * ============================================================================================== */

/* Compute overlap and number of differing features between two cultural vectors. */
static void culture_overlap_and_diff(uint8_t const *a,
                                     uint8_t const *b,
                                     uint8_t *overlap,
                                     uint8_t *nb_diff,
                                     uint8_t diff_idx[CULTURE_F]) {
    uint8_t ov = 0;
    uint8_t nd = 0;
    for (uint8_t k = 0; k < CULTURE_F; ++k) {
        if (a[k] == b[k]) {
            ov++;
        } else {
            if (nd < CULTURE_F) {
                diff_idx[nd] = k;
            }
            nd++;
        }
    }
    *overlap = ov;
    *nb_diff = nd;
}

/* One asynchronous Axelrod update with a random neighbour (if any). */
static void axelrod_update_step(void) {
    if (mydata->nb_neighbors == 0U) { return; }

    /* pick a random neighbour */
    uint8_t idx = (uint8_t)random_uint32(mydata->nb_neighbors);
    neighbor_t *n = &mydata->neighbors[idx];

    uint8_t overlap = 0;
    uint8_t nb_diff = 0;
    uint8_t diff_idx[CULTURE_F];

    culture_overlap_and_diff(mydata->features, n->features,
                             &overlap, &nb_diff, diff_idx);

    if (overlap == 0U || nb_diff == 0U) {
        /* either no similarity → no interaction,
         * or identical culture → nothing to copy */
        return;
    }

    /* Interaction probability proportional to similarity (Axelrod rule). */
    float p = (float)overlap / (float)CULTURE_F;
    if (rand_unitf() <= p) {
        /* copy one differing feature from neighbour */
        uint8_t which = diff_idx[random_uint32(nb_diff)];
        mydata->features[which] = n->features[which];
    }
}

/* Recompute whether this robot is culturally stable (no active Axelrod links). */
static void update_cultural_stability(void) {
    bool stable = true;

    for (uint8_t i = 0; i < mydata->nb_neighbors; ++i) {
        uint8_t overlap = 0;
        uint8_t nb_diff = 0;
        uint8_t diff_idx[CULTURE_F];
        (void)diff_idx; /* unused, but keeps signature consistent */

        culture_overlap_and_diff(mydata->features,
                                 mydata->neighbors[i].features,
                                 &overlap, &nb_diff, diff_idx);

        /* Active link if some overlap but not identical. */
        if (overlap > 0U && nb_diff > 0U) {
            stable = false;
            break;
        }
    }

    mydata->culturally_stable = stable;
}

/* ================================================================================================
 * LED RENDERING  (culture visualisation)
 * ============================================================================================== */

/* Map cultural vector -> [0,255] for rainbow_colormap. */
static uint8_t culture_to_color_index(uint8_t const *features) {
    uint32_t code = 0;
    uint32_t base = 1;

    for (uint8_t k = 0; k < CULTURE_F; ++k) {
        code += (uint32_t)features[k] * base;
        base *= (uint32_t)CULTURE_Q;  /* after loop: base = Q^F */
    }

    uint32_t space = (base == 0U) ? 1U : base;  /* Q^F */
    if (space <= 1U) {
        return 0; /* degenerate case */
    }

    if (code >= space) {
        code %= space;
    }

    uint8_t idx = (uint8_t)((code * 255U) / (space - 1U));
    return idx;
}



static void update_leds(void) {
    uint8_t r = 0, g = 0, b = 0;

    /* Centre LED: culture group colour */
    uint8_t idx = culture_to_color_index(mydata->features);
    rainbow_colormap(idx, &r, &g, &b);
    pogobot_led_setColors(r, g, b, 0);  /* centre LED */

    /* Ring LED 1: cultural stability indicator */
    if (mydata->culturally_stable) {
        /* green = culturally frozen (no active links) */
        r = 0; g = 255; b = 0;
    } else {
        /* red = still culturally active */
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
                mydata->tumble_left = (rand() & 1U);
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

void user_init(void) {
    srand(pogobot_helper_getRandSeed());
    pogobot_infrared_set_power(2);

    memset(mydata, 0, sizeof(*mydata));

    /* motor polarity from EEPROM */
    uint8_t dir_mem[3];
    pogobot_motor_dir_mem_get(dir_mem);
    mydata->motor_dir_left  = dir_mem[1];
    mydata->motor_dir_right = dir_mem[0];

    /* random initial culture */
    for (uint8_t k = 0; k < CULTURE_F; ++k) {
        mydata->features[k] = (uint8_t)random_uint32(CULTURE_Q);
    }

    /* initial culture hash & stability */
    mydata->culture_hash      = 0;
    mydata->culturally_stable = false;

    /* FSM init */
    mydata->fsm        = STATE_TUMBLE;
    mydata->timer_ms   = 100;
    mydata->tumble_left = true;

    /* simulator / messaging */
    main_loop_hz                  = BROADCAST_HZ;  /* 30 Hz loop */
    msg_rx_fn                     = process_message;
    msg_tx_fn                     = send_culture_msg;
    max_nb_processed_msg_per_tick = 4;
    percent_msgs_sent_per_ticks   = 50;
    error_codes_led_idx           = 3;

    mydata->last_step_ms      = current_time_milliseconds();
    mydata->last_broadcast_ms = current_time_milliseconds();
}

/* ================================================================================================
 * MAIN CONTROL LOOP
 * ============================================================================================== */

void user_step(void) {
    uint32_t now        = current_time_milliseconds();
    uint32_t elapsed_ms = now - mydata->last_step_ms;
    mydata->last_step_ms = now;

    /* neighbour maintenance */
    purge_old_neighbors(now);

    /* Axelrod update (cultural dynamics) */
    axelrod_update_step();

    /* Recompute whether this robot is culturally “frozen” */
    update_cultural_stability();

    /* Update motion (or keep robots immobile if moving_robots == false) */
    update_motion(elapsed_ms);

    /* LED feedback & culture beacon */
    update_leds();
    send_culture_msg();
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
    data_add_column_int8("culturally_stable");
    data_add_column_int32("culture_hash");
    for (uint8_t k = 0; k < CULTURE_F; ++k) {
        char name[32];
        snprintf(name, sizeof(name), "feature_%u", (unsigned)k);
        data_add_column_int16(name);
    }
}

static uint16_t compute_culture_hash(uint8_t const *features) {
    uint32_t code = 0;
    uint32_t base = 1;

    for (uint8_t k = 0; k < CULTURE_F; ++k) {
        code += (uint32_t)features[k] * base;
        base *= (uint32_t)CULTURE_Q;  /* base = Q^F */
    }

    uint32_t space = (base == 0U) ? 1U : base;
    if (code >= space) {
        code %= space;
    }
    return (uint16_t)(code & 0xFFFFu);
}


static void export_data(void) {
    mydata->culture_hash = compute_culture_hash(mydata->features);

    data_set_value_int8("nb_neighbors",      mydata->nb_neighbors);
    data_set_value_int8("culturally_stable", mydata->culturally_stable ? 1 : 0);
    data_set_value_int32("culture_hash",     mydata->culture_hash);

    for (uint8_t k = 0; k < CULTURE_F; ++k) {
        char name[32];
        snprintf(name, sizeof(name), "feature_%u", (unsigned)k);
        data_set_value_int16(name, mydata->features[k]);
    }
}
#endif /* SIMULATOR */


/* ================================================================================================
 * ENTRY POINT
 * ============================================================================================== */

int main(void) {
    pogobot_init();
    pogobot_start(user_init, user_step);

    SET_CALLBACK(callback_global_setup,       global_setup);
    SET_CALLBACK(callback_create_data_schema, create_data_schema);
    SET_CALLBACK(callback_export_data,        export_data);
    return 0;
}

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker

