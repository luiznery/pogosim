/**
 * @file
 *
 * @brief
 * Local Naming Game on a swarm of Pogobot robots, with configurable LED
 * visualisation and optional robot motion.
 *
 * @details
 * This example implements a minimalist variant of the *Naming Game* on a
 * swarm of Pogobot robots. Each robot tries to negotiate a shared “name”
 * (a discrete label) for a single abstract object by exchanging short IR
 * messages with neighbours.
 *
 * ### State of each robot
 *
 * Each robot maintains:
 *
 *   - A finite **inventory** of word labels:
 *
 *       \f[
 *           \mathcal{I}_i(t) = \{ w^{(1)}_i, \dots, w^{(M_i(t))}_i \},
 *       \f]
 *
 *     where each word is an integer \f$w \in \{0,\dots,W_{\max}-1\}\f$.
 *
 *   - A **current uttered word** \f$u_i(t)\f$ (chosen at random from the
 *     inventory when the robot acts as “speaker”).
 *
 *   - Counters of **successful** and **failed** interactions in which the
 *     robot acted as a listener.
 *
 * In the original Naming Game, agents often start with an empty inventory and
 * invent new words when they first act as speakers. We keep this behaviour:
 * when a robot with an empty inventory is about to speak, it **invents** a
 * new random word label and adds it to its inventory.
 *
 * ### Interaction rule (listener-only Naming Game)
 *
 * Time is discretised by the main control loop. At regular intervals each
 * robot broadcasts one IR message containing its current word. When robot
 * \f$i\f$ receives a message from neighbour \f$j\f$ with word \f$u_j\f$,
 * \f$i\f$ behaves as a **listener** and applies a local Naming Game update:
 *
 *   1. If \f$u_j \in \mathcal{I}_i\f$ (listener already knows the word):
 *
 *        - The interaction is a **success**.
 *        - The listener *aligns* on that word by pruning its inventory:
 *
 *            \f[
 *              \mathcal{I}_i(t^+) = \{ u_j \}, \quad
 *              u_i(t^+) = u_j.
 *            \f]
 *
 *   2. Otherwise (\f$u_j \notin \mathcal{I}_i\f$):
 *
 *        - The interaction is a **failure**.
 *        - The listener simply **adds** the word to its inventory:
 *
 *            \f[
 *              \mathcal{I}_i(t^+) = \mathcal{I}_i(t) \cup \{ u_j \},
 *              \quad u_i(t^+) = u_j.
 *            \f]
 *
 * This “listener-only” variant is a standard simplification of the original
 * pairwise Naming Game that remains in the same universality class while
 * being easier to implement on asynchronous, broadcast-based platforms such
 * as robot swarms.
 *
 * Over time, the local successes prune inventories and the population
 * typically converges towards a **global consensus** on a single shared word.
 *
 * ### Relation to the Voter model
 *
 * The classical **Voter model** is an opinion dynamics process in which each
 * agent holds exactly one opinion and, when updated, simply copies the state
 * of a random neighbour. In the present implementation:
 *
 *   - If we constrained each robot to a **single-word inventory** and removed
 *     invention, the dynamics would reduce to a Voter-like process.
 *   - The Naming Game adds:
 *       - A *memory* of multiple candidate words per agent.
 *       - An **invention** mechanism introducing new opinions (words).
 *       - Asymmetric speaker/listener roles with success/failure asymmetry.
 *
 * Hence the Naming Game can be seen as a minimal extension of the Voter model
 * that can account for the emergence of **shared lexical conventions** rather
 * than simple opinion copying.
 *
 * ### Motion and interaction network
 *
 * The same code can be run with:
 *
 *   - `moving_robots == false`: robots remain fixed, yielding a static random
 *     geometric communication graph.
 *   - `moving_robots == true`: robots perform run-and-tumble motion, so the
 *     interaction graph becomes time varying.
 *
 * This allows one to study how motion affects convergence of the Naming Game
 * on dynamic networks using the same controller.
 *
 * ### LED visualisation and configuration
 *
 * Each Pogobot has a centre RGB LED (index 0) and a ring LED (index 1).
 * We expose a configuration parameter:
 *
 *   - `led_mode` (uint8, set via YAML / command line):
 *
 *       - `0` (**WORD + SUCCESS**, default):
 *           - Centre LED hue encodes the current word \f$u_i\f$ using a
 *             rainbow colormap.
 *           - Brightness is higher when the inventory is small (robot has
 *             “settled” on a single word).
 *           - Ring LED is **green** on the last successful interaction,
 *             **red** otherwise.
 *
 *       - `1` (**LOCAL CONSENSUS**):
 *           - Centre LED hue encodes the current word.
 *           - The brightness is scaled by the fraction of neighbours sharing
 *             the same current word, so bright patches reveal emergent
 *             consensus domains.
 *           - Ring LED still encodes last success/failure.
 *
 *       - `2` (**INVENTORY SIZE**):
 *           - Centre LED colour encodes the current word; ring LED encodes the
 *             inventory size (blue-ish when the robot stores many competing
 *             words, green when inventory collapses to a single word).
 *
 * This makes the demo both **scientifically meaningful** (one can visually
 * track consensus formation) and **aesthetically pleasing** for interactive
 * presentations.
 *
 * ### Key configuration parameters
 *
 * These are typically set via Pogosim YAML configuration files:
 *
 *   - `moving_robots`       : enable/disable physical motion.
 *   - `max_speed_frac`      : fraction of maximum motor speed during RUN.
 *   - `run_duration_min/max`: min/max run times [ms].
 *   - `tumble_duration_min/max`: min/max tumble times [ms].
 *   - `led_mode`            : LED visualisation mode as described above.
 *
 * ### Scientific background (non-exhaustive)
 *
 *   - L. Steels, “A Self-Organizing Spatial Vocabulary”, Artificial Life 2,
 *     1995 — early formulation of Naming Games for lexical emergence.
 *   - A. Baronchelli, M. Felici, V. Loreto, E. Caglioti, L. Steels,
 *     “Sharp transition towards shared vocabularies in multi-agent systems”,
 *     Journal of Statistical Mechanics, 2006 — finite-size scaling and
 *     consensus properties of the Naming Game.
 *   - C. Castellano, S. Fortunato, V. Loreto, “Statistical physics of social
 *     dynamics”, Rev. Mod. Phys. 81, 2009 — review including the Voter model
 *     and related opinion dynamics processes.
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

/* --- Naming Game word / inventory parameters --- */
#define MAX_INVENTORY_SIZE  16U    /**< Maximum number of words stored per robot. */
#define MAX_WORD_ID         255U   /**< Words are uint8_t ∈ [0, MAX_WORD_ID].    */

/* --- Motion (same scaffold as Axelrod example) --- */
static float    max_speed_frac        = 0.4f;   /* 0–1: fraction of motorFull while RUNNING */
static uint32_t run_duration_min      =  800;   /* [ms] */
static uint32_t run_duration_max      = 4000;   /* [ms] */
static uint32_t tumble_duration_min   =  200;   /* [ms] */
static uint32_t tumble_duration_max   =  600;   /* [ms] */

/* Whether robots actually move (run-and-tumble) or stay immobile. */
static bool     moving_robots         = true;

/* --- LED visualisation mode (configurable) --- */
static uint8_t  led_mode              = 0;      /* 0: word+success, 1: consensus, 2: inventory size */

/* --- IR messaging --- */
#define BROADCAST_HZ        10U
#define broadcast_period_ms (1000U / BROADCAST_HZ)

#define MAX_NEIGHBORS       20U
#define IR_RANGE_MAX_AGE    200U    /* [ms] keep neighbour entry this long */

/* ================================================================================================
 * ENUMS & STRUCTS
 * ============================================================================================== */

typedef enum {
    STATE_RUN    = 0,
    STATE_TUMBLE = 1
} robot_state_t;

typedef struct {
    uint16_t id;
    uint8_t  word;              /* neighbour's current word */
    uint8_t  inventory_size;    /* neighbour inventory size (for diagnostics) */
    uint8_t  last_success;      /* 1 if neighbour's last interaction was success */
    uint32_t last_seen_ms;
} neighbor_t;

/* ==== USERDATA ================================================================================= */

typedef struct {
    /* Naming Game state */
    uint8_t  inventory[MAX_INVENTORY_SIZE];
    uint8_t  inventory_size;
    uint8_t  current_word;
    bool     last_heard_success;
    uint32_t nb_success;
    uint32_t nb_failure;

    /* neighbour list for consensus estimation */
    neighbor_t neighbors[MAX_NEIGHBORS];
    uint8_t    nb_neighbors;

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
 * NAMING GAME INVENTORY HELPERS
 * ============================================================================================== */

static bool inventory_contains(uint8_t word) {
    for (uint8_t i = 0; i < mydata->inventory_size; ++i) {
        if (mydata->inventory[i] == word) {
            return true;
        }
    }
    return false;
}

static void inventory_clear_except(uint8_t word) {
    mydata->inventory[0]  = word;
    mydata->inventory_size = 1U;
}

/* Add word to inventory if not already there; overwrite random slot if full. */
static void inventory_add_word(uint8_t word) {
    if (inventory_contains(word)) {
        return;
    }

    if (mydata->inventory_size < MAX_INVENTORY_SIZE) {
        mydata->inventory[mydata->inventory_size++] = word;
    } else {
        /* Overwrite a random entry when memory is saturated. */
        uint8_t idx = (uint8_t)random_uint32(MAX_INVENTORY_SIZE);
        mydata->inventory[idx] = word;
    }
}

/* Invent a new word not currently in inventory (best effort). */
static uint8_t invent_new_word(void) {
    for (uint16_t tries = 0; tries < 512U; ++tries) {
        uint8_t candidate = (uint8_t)random_uint32(MAX_WORD_ID + 1U);
        if (!inventory_contains(candidate)) {
            return candidate;
        }
    }
    /* Fallback: random word; may already be in inventory. */
    return (uint8_t)random_uint32(MAX_WORD_ID + 1U);
}

/* Ensure current_word is valid and chosen from inventory. */
static void choose_current_word(void) {
    if (mydata->inventory_size == 0U) {
        uint8_t w = invent_new_word();
        mydata->inventory[0]  = w;
        mydata->inventory_size = 1U;
        mydata->current_word   = w;
    } else {
        uint8_t idx = (uint8_t)random_uint32(mydata->inventory_size);
        mydata->current_word = mydata->inventory[idx];
    }
}

/* ================================================================================================
 * IR MESSAGING (Naming Game broadcast)
 * ============================================================================================== */

typedef struct __attribute__((__packed__)) {
    uint16_t sender_id;
    uint8_t  word;
    uint8_t  inventory_size;
    uint8_t  last_success;  /* 0 or 1 from sender's perspective */
} naming_msg_t;

#define NAMING_MSG_SIZE  ((uint16_t)sizeof(naming_msg_t))

static bool send_naming_msg(void) {
    uint32_t now = current_time_milliseconds();
    if (now - mydata->last_broadcast_ms < broadcast_period_ms) {
        return false;
    }

    choose_current_word();

    naming_msg_t m;
    m.sender_id      = pogobot_helper_getid();
    m.word           = mydata->current_word;
    m.inventory_size = mydata->inventory_size;
    m.last_success   = mydata->last_heard_success ? 1U : 0U;

    pogobot_infrared_sendShortMessage_omni((uint8_t *)&m, NAMING_MSG_SIZE);
    mydata->last_broadcast_ms = now;
    return true;
}

/* Update / insert neighbour entry. */
static neighbor_t *update_neighbor_from_msg(naming_msg_t const *nm, uint32_t now) {
    uint8_t idx;
    for (idx = 0; idx < mydata->nb_neighbors; ++idx) {
        if (mydata->neighbors[idx].id == nm->sender_id) {
            break;
        }
    }

    if (idx == mydata->nb_neighbors) {
        if (mydata->nb_neighbors >= MAX_NEIGHBORS) {
            return NULL;
        }
        mydata->nb_neighbors++;
    }

    neighbor_t *n = &mydata->neighbors[idx];
    n->id            = nm->sender_id;
    n->word          = nm->word;
    n->inventory_size = nm->inventory_size;
    n->last_success   = nm->last_success;
    n->last_seen_ms   = now;

    return n;
}

static void purge_old_neighbors(uint32_t now) {
    for (int8_t i = (int8_t)mydata->nb_neighbors - 1; i >= 0; --i) {
        if (now - mydata->neighbors[i].last_seen_ms > IR_RANGE_MAX_AGE) {
            mydata->neighbors[i] = mydata->neighbors[mydata->nb_neighbors - 1];
            mydata->nb_neighbors--;
        }
    }
}

/* Listener-only Naming Game update on message reception. */
static void naming_game_listener_update(uint8_t heard_word) {
    if (inventory_contains(heard_word)) {
        /* SUCCESS: prune inventory to the heard word. */
        inventory_clear_except(heard_word);
        mydata->last_heard_success = true;
        mydata->nb_success++;
    } else {
        /* FAILURE: add the new word to inventory. */
        inventory_add_word(heard_word);
        mydata->last_heard_success = false;
        mydata->nb_failure++;
    }
    mydata->current_word = heard_word;
}

static void process_message(message_t *msg) {
    if (msg->header.payload_length < NAMING_MSG_SIZE) {
        return;
    }

    naming_msg_t const *nm = (naming_msg_t const *)msg->payload;
    if (nm->sender_id == pogobot_helper_getid()) {
        return;
    }

    uint32_t now = current_time_milliseconds();
    (void)update_neighbor_from_msg(nm, now);

    /* Apply Naming Game listener update. */
    naming_game_listener_update(nm->word);
}

/* ================================================================================================
 * LED RENDERING (word / consensus visualisation)
 * ============================================================================================== */

/* Simple mapping word -> [0,255] index for rainbow_colormap. */
static uint8_t word_to_color_index(uint8_t word) {
    return word;  /* direct mapping, words are already in [0,255] */
}

/* Apply brightness scaling (0–255) to RGB components. */
static void apply_brightness(uint8_t *r, uint8_t *g, uint8_t *b, uint8_t brightness) {
    uint16_t br = brightness;
    *r = (uint8_t)(((*r) * br) / 255U);
    *g = (uint8_t)(((*g) * br) / 255U);
    *b = (uint8_t)(((*b) * br) / 255U);
}

/* Compute local consensus: fraction of neighbours sharing current_word. */
static uint8_t local_consensus_pct(void) {
    if (mydata->nb_neighbors == 0U) {
        return 100U;
    }

    uint8_t same = 0U;
    for (uint8_t i = 0; i < mydata->nb_neighbors; ++i) {
        if (mydata->neighbors[i].word == mydata->current_word) {
            same++;
        }
    }
    return (uint8_t)((100U * same) / mydata->nb_neighbors);
}

/* Compute a rough encoding of inventory size into [0,255] brightness. */
static uint8_t inventory_brightness(void) {
    if (mydata->inventory_size == 0U) {
        return 32U;  /* should not really happen, but avoid darkness */
    }
    /* More words → lower brightness; keep >= 32 for visibility. */
    uint8_t b = (uint8_t)(255U / mydata->inventory_size);
    if (b < 32U) {
        b = 32U;
    }
    if (b > 255U) {
        b = 255U;
    }
    return b;
}

/* Map inventory size to a simple ring colour (mode 2). */
static void ring_color_inventory(uint8_t *r, uint8_t *g, uint8_t *b) {
    if (mydata->inventory_size <= 1U) {
        /* single word → green (converged) */
        *r = 0U; *g = 255U; *b = 0U;
    } else if (mydata->inventory_size < MAX_INVENTORY_SIZE / 2U) {
        /* small inventory → yellowish */
        *r = 255U; *g = 255U; *b = 0U;
    } else {
        /* large inventory → bluish */
        *r = 0U; *g = 128U; *b = 255U;
    }
}

static void update_leds(void) {
    uint8_t r = 0U, g = 0U, b = 0U;

    /* Ensure we have a valid current word for visualisation. */
    if (mydata->inventory_size == 0U) {
        choose_current_word();
    }

    uint8_t idx = word_to_color_index(mydata->current_word);
    rainbow_colormap(idx, &r, &g, &b);

    uint8_t brightness = 255U;

    switch (led_mode) {
        default:
        case 0: {
            /* WORD + SUCCESS: brightness governed by inventory size. */
            brightness = inventory_brightness();
            apply_brightness(&r, &g, &b, brightness);

            /* Ring LED = success (green) / failure (red). */
            uint8_t rr = mydata->last_heard_success ? 0U   : 255U;
            uint8_t gg = mydata->last_heard_success ? 255U : 0U;
            uint8_t bb = 0U;
            pogobot_led_setColors(rr, gg, bb, 1);
            break;
        }

        case 1: {
            /* LOCAL CONSENSUS: brightness from fraction of neighbours sharing word. */
            uint8_t consensus = local_consensus_pct();          /* 0–100 */
            brightness = (uint8_t)((consensus * 255U) / 100U);  /* 0–255 */
            if (brightness < 32U) {
                brightness = 32U;
            }
            apply_brightness(&r, &g, &b, brightness);

            /* Ring LED still encodes last success/failure. */
            uint8_t rr = mydata->last_heard_success ? 0U   : 255U;
            uint8_t gg = mydata->last_heard_success ? 255U : 0U;
            uint8_t bb = 0U;
            pogobot_led_setColors(rr, gg, bb, 1);
            break;
        }

        case 2: {
            /* INVENTORY SIZE: word hue at full brightness; ring encodes inventory size. */
            uint8_t rr, gg, bb;
            ring_color_inventory(&rr, &gg, &bb);
            pogobot_led_setColors(rr, gg, bb, 1);
            break;
        }
    }

    /* Centre LED always shows the word colour (possibly dimmed). */
    pogobot_led_setColors(r, g, b, 0);
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
        mydata->timer_ms = 0U;
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

static void user_init(void) {
    srand(pogobot_helper_getRandSeed());
    pogobot_infrared_set_power(2);

    memset(mydata, 0, sizeof(*mydata));

    /* motor polarity from EEPROM */
    uint8_t dir_mem[3];
    pogobot_motor_dir_mem_get(dir_mem);
    mydata->motor_dir_left  = dir_mem[1];
    mydata->motor_dir_right = dir_mem[0];

    /* Initial inventory is empty; first time a robot speaks, it will invent a word. */
    mydata->inventory_size    = 0U;
    mydata->current_word      = 0U;
    mydata->last_heard_success = false;
    mydata->nb_success        = 0U;
    mydata->nb_failure        = 0U;

    /* FSM init */
    mydata->fsm         = STATE_TUMBLE;
    mydata->timer_ms    = 100U;
    mydata->tumble_left = true;

    /* simulator / messaging hooks */
    main_loop_hz                  = BROADCAST_HZ;
    msg_rx_fn                     = process_message;
    msg_tx_fn                     = send_naming_msg;
    max_nb_processed_msg_per_tick = 4;
    percent_msgs_sent_per_ticks   = 50;
    error_codes_led_idx           = 3;

    uint32_t now          = current_time_milliseconds();
    mydata->last_step_ms  = now;
    mydata->last_broadcast_ms = now;
}

/* ================================================================================================
 * MAIN CONTROL LOOP
 * ============================================================================================== */

static void user_step(void) {
    uint32_t now        = current_time_milliseconds();
    uint32_t elapsed_ms = now - mydata->last_step_ms;
    mydata->last_step_ms = now;

    /* Maintain neighbour list (for consensus statistics and LED mode 1). */
    purge_old_neighbors(now);

    /* Update motion (or keep robots immobile). */
    update_motion(elapsed_ms);

    /* LED feedback & culture beacon. */
    update_leds();

    /* Broadcast our current word periodically. */
    send_naming_msg();
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

    /* LED visualisation mode */
    init_from_configuration(led_mode);
}

static void create_data_schema(void) {
    data_add_column_int8("nb_neighbors");
    data_add_column_int8("inventory_size");
    data_add_column_int16("current_word");
    data_add_column_int8("last_heard_success");
    data_add_column_int32("nb_success");
    data_add_column_int32("nb_failure");
    data_add_column_int8("local_consensus_pct");
}

static void export_data(void) {
    data_set_value_int8("nb_neighbors",        mydata->nb_neighbors);
    data_set_value_int8("inventory_size",      mydata->inventory_size);
    data_set_value_int16("current_word",       mydata->current_word);
    data_set_value_int8("last_heard_success",  mydata->last_heard_success ? 1 : 0);
    data_set_value_int32("nb_success",         (int32_t)mydata->nb_success);
    data_set_value_int32("nb_failure",         (int32_t)mydata->nb_failure);
    data_set_value_int8("local_consensus_pct", (int8_t)local_consensus_pct());
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
