// To compile as a simulation:
//  ./build.sb
//  ./examples/probes/probes -c conf/probes.yaml
//
// To compile on real robots:
//  cd examples/probes
//  make clean bin                           # To compile the "robots" category
//     OR:
//  ROBOT_CATEGORY=probes make clean bin     # To compile the "probes" category

// Main include for pogobots, both for real robots and for simulations
#include "pogobase.h"

/*
 * =====================
 * Configuration & docs
 * =====================
 *
 * There are 3 independent “limits” in this file:
 *
 *  1) ROBOT_LOG_BUFFER_SIZE
 *     Per-robot FIFO of *raw* log bytes, recorded locally on the robot.
 *
 *  2) PROBE_CACHE_BUFFER_SIZE
 *     Per-probe FIFO of *decoded* log entries (rid, t_ms, phase).
 *
 *  3) PROBE_PARTIAL_SLOTS
 *     Per-probe number of distinct sender IDs for which we can keep an
 *     in-progress (partial) 5-byte log entry when messages split entries.
 *
 * Note: these are memory budgets: if they are exceeded, the oldest data is dropped.
 */

#define PROBE_CACHE_BUFFER_SIZE   (5u * 1024u)   // 5 KB total cache on probes

// One entry in the probe's cache: robot id, timestamp (uint32_t in ms), and phase.
typedef struct __attribute__((__packed__)) {
    uint16_t rid;       // robot id
    uint32_t t_ms;      // time in milliseconds, stored as uint32_t (no encoding)
    uint8_t  phase;     // state
} ProbeCacheEntry;

// Size of one entry and maximum number of entries that fit in PROBE_CACHE_BUFFER_SIZE
#define PROBE_ENTRY_BYTES  ((uint16_t)sizeof(ProbeCacheEntry))
#define PROBE_MAX_ENTRIES  (PROBE_CACHE_BUFFER_SIZE / PROBE_ENTRY_BYTES)

// Raw log entry as stored on robots: 4-byte timestamp + 1-byte phase.
typedef struct __attribute__((__packed__)) {
    uint32_t t_ms;
    uint8_t  phase;
} RobotLogEntryRaw;

#define ROBOT_LOG_ENTRY_BYTES   ((uint8_t)sizeof(RobotLogEntryRaw))

// Robot-side settings
// 32 KB ring buffer so that, at 1 Hz, we can store far more than >= 3600 entries (1 hour) with margin.
#define ROBOT_LOG_BUFFER_SIZE      (32u * 1024u) // 32 KB per robot
#define ROBOT_LOG_PERIOD_MS        1000u         // 1 Hz logging

// Robots record logs during the "experiment" and only broadcast afterwards.
// Example: record for 60 s, wait 10 s, then start cyclic rebroadcasting.
// (These can be overridden from YAML in simulation via global_setup.)
uint32_t log_record_duration_s = 60u;
uint32_t log_post_wait_s       = 10u;

/*
 * Wire format (long IR message payload)
 *
 * We keep the payload size FIXED so that sender/receiver agree and we never
 * read past mr->payload.
 *
 * Layout:
 *   byte 0..1: sender_id (uint16_t)
 *   byte 2:    chunk_len (0..LOG_CHUNK_DATA_BYTES)
 *   byte 3:    msg_type  (MSG_TYPE_LOG_CHUNK)
 *   byte 4..:  chunk_data
 */
// Fixed payload size for pogobot_infrared_sendLongMessage_omniGen(...)
#define LOG_MESSAGE_BYTES   32u

// Header is 4 bytes: sender_id (2) + chunk_len (1) + msg_type (1)
#define LOG_HEADER_BYTES    4u

// Number of complete raw entries we try to pack in one message.
// (Each entry is ROBOT_LOG_ENTRY_BYTES bytes, currently 5 bytes.)
#define LOG_ENTRIES_PER_MSG  5u

// Data bytes (multiple of ROBOT_LOG_ENTRY_BYTES), plus computed padding up to 32 bytes.
#define LOG_CHUNK_DATA_BYTES  (LOG_ENTRIES_PER_MSG * (uint32_t)ROBOT_LOG_ENTRY_BYTES)
#define LOG_PAD_BYTES         (LOG_MESSAGE_BYTES - LOG_HEADER_BYTES - LOG_CHUNK_DATA_BYTES)

// Compile-time sanity checks
_Static_assert((LOG_CHUNK_DATA_BYTES % ROBOT_LOG_ENTRY_BYTES) == 0u,
               "LOG_CHUNK_DATA_BYTES must be a multiple of ROBOT_LOG_ENTRY_BYTES");
_Static_assert((LOG_HEADER_BYTES + LOG_CHUNK_DATA_BYTES) <= LOG_MESSAGE_BYTES,
               "Log message header+data exceed LOG_MESSAGE_BYTES");

// Probe-side: track partial (incomplete) entries without assuming robot IDs are dense.
#define PROBE_PARTIAL_SLOTS  256u
#define PROBE_RID_UNUSED     0xFFFFu

typedef struct __attribute__((__packed__)) {
    uint16_t rid;                           // sender id tracked in this slot
    uint8_t  len;                           // 0..ROBOT_LOG_ENTRY_BYTES
    uint8_t  bytes[ROBOT_LOG_ENTRY_BYTES];  // partial entry bytes
    uint32_t last_seen_ms;                  // for simple Least-Recently-Used (LRU) eviction
} ProbePartialSlot;

_Static_assert(sizeof(ProbePartialSlot) == (2u + 1u + ROBOT_LOG_ENTRY_BYTES + 4u),
               "ProbePartialSlot must stay packed");


// "Global" variables set by the YAML configuration file (in simulation) by the function global_setup, or with a fixed values (in experiments). These values should be seen as constants shared by all robots. On real robots, they will be optimized away as const values.
uint32_t run_duration_min     = 200;
uint32_t run_duration_max     = 1200;
uint32_t tumble_duration_min  = 100;
uint32_t tumble_duration_max  = 1100;

typedef enum {
    MSG_TYPE_LOG_CHUNK = 1,
} msg_type_t;

// NOTE: We keep this struct *exactly* LOG_MESSAGE_BYTES bytes.
typedef struct __attribute__((__packed__, __aligned__(4))) {
    uint16_t sender_id;                         // bytes 0..1
    uint8_t  chunk_len;                         // byte 2 (0..LOG_CHUNK_DATA_BYTES)
    uint8_t  msg_type;                          // byte 3
    uint8_t  chunk_data[LOG_CHUNK_DATA_BYTES];  // bytes 4..
    /*
     * Padding up to LOG_MESSAGE_BYTES.
     *
     * IMPORTANT: LOG_PAD_BYTES depends on sizeof(...), so it cannot be used in a
     * preprocessor #if (the preprocessor cannot evaluate sizeof). We therefore
     * always declare the member.
     *
     * When LOG_PAD_BYTES == 0, GCC/Clang accept a zero-length array as a common extension. 
     */
#if defined(__GNUC__) || defined(__clang__)
    uint8_t  pad[LOG_PAD_BYTES];
#else
    uint8_t  pad[(LOG_PAD_BYTES > 0u) ? LOG_PAD_BYTES : 1u];
#endif
} LogMessageRaw;

_Static_assert(sizeof(LogMessageRaw) == LOG_MESSAGE_BYTES,
               "LogMessageRaw must be exactly LOG_MESSAGE_BYTES bytes");

#if !defined(__GNUC__) && !defined(__clang__)
_Static_assert(LOG_PAD_BYTES > 0u,
               "Compiler lacks zero-length arrays; choose LOG_ENTRIES_PER_MSG so LOG_PAD_BYTES>0");
#endif

typedef union __attribute__((aligned(4))) {
    uint8_t       payload[LOG_MESSAGE_BYTES];
    LogMessageRaw fields;
} message;


// Normal "Global" variables should be inserted within the USERDATA struct.
// /!\  In simulation, don't declare non-const global variables outside this struct, elsewise they will be shared among all agents (and this is not realistic).

typedef enum {
    PHASE_RUN = 0,
    PHASE_TUMBLE,
} state_t;

typedef struct {
    // Run & tumble state
    state_t   phase;
    uint32_t  phase_start_time;
    uint32_t  phase_duration;
    uint8_t   tumble_direction; // 0 = left, 1 = right

    // === ROBOT logging + broadcasting (5 KB FIFO queue) ===
    uint8_t   robot_log_buffer[ROBOT_LOG_BUFFER_SIZE];
    uint16_t  robot_log_head;      // next write index
    uint16_t  robot_log_tail;      // next byte to send
    uint16_t  robot_log_count;     // number of bytes currently stored
    uint32_t  last_log_time_ms;

    // Experiment timing / deferred broadcasting
    uint32_t  sim_start_time_ms;
    bool   log_broadcast_enabled;

    // Cyclic rebroadcast cursor (non-destructive): we keep sending the same ring
    // buffer content in order, restarting from robot_log_tail when done.
    uint16_t  robot_bcast_cursor;      // next byte to transmit (ring index)
    uint16_t  robot_bcast_remaining;   // bytes left in current cycle (0 => restart)

    // === PROBE: global cache of decoded log entries (5 KB FIFO) ===
    ProbeCacheEntry probe_cache[PROBE_MAX_ENTRIES];
    uint16_t probe_cache_head;   // next write index (entry index)
    uint16_t probe_cache_tail;   // next read index (entry index)
    uint16_t probe_cache_count;  // number of entries currently stored

    // Probe-side partial-reassembly state for up to PROBE_PARTIAL_SLOTS senders.
    ProbePartialSlot probe_partials[PROBE_PARTIAL_SLOTS];
} USERDATA;

// Call this macro in the same file (.h or .c) as the declaration of USERDATA
DECLARE_USERDATA(USERDATA);

// Don't forget to call this macro in the main .c file of your project (only once!)
REGISTER_USERDATA(USERDATA);
// Now, members of the USERDATA struct can be accessed through mydata->MEMBER. E.g. mydata->data_foo
//  On real robots, the compiler will automatically optimize the code to access member variables as if they were true globals.


// Prototypes of functions that may be discarded depending on the robot category
void probes_process_message(message_t* mr);
void probes_user_init(void);
void probes_user_step(void);

/**
 * @brief Generates a random duration for the run phase.
 *
 * The run phase duration is randomly chosen
 *
 * @return uint32_t Random run duration in milliseconds.
 */
static uint32_t get_run_duration(void) {
    return run_duration_min + (rand() % (run_duration_max - run_duration_min + 1));
}

/**
 * @brief Generates a random duration for the tumble phase.
 *
 * The tumble phase duration is randomly chosen
 *
 * @return uint32_t Random tumble duration in milliseconds.
 */
static uint32_t get_tumble_duration(void) {
    return tumble_duration_min + (rand() % (tumble_duration_max - tumble_duration_min + 1));
}

static void robot_log_state(uint32_t now_ms) {
    // Ensure room for one entry: if full, drop oldest entry(ies)
    while ((uint32_t)mydata->robot_log_count + ROBOT_LOG_ENTRY_BYTES > ROBOT_LOG_BUFFER_SIZE) {
        mydata->robot_log_tail = (mydata->robot_log_tail + ROBOT_LOG_ENTRY_BYTES) % ROBOT_LOG_BUFFER_SIZE;
        mydata->robot_log_count -= ROBOT_LOG_ENTRY_BYTES;
    }

    uint16_t idx = mydata->robot_log_head;

    mydata->robot_log_buffer[idx] = (uint8_t)(now_ms & 0xFF); idx = (idx + 1) % ROBOT_LOG_BUFFER_SIZE;
    mydata->robot_log_buffer[idx] = (uint8_t)((now_ms >> 8) & 0xFF); idx = (idx + 1) % ROBOT_LOG_BUFFER_SIZE;
    mydata->robot_log_buffer[idx] = (uint8_t)((now_ms >> 16) & 0xFF); idx = (idx + 1) % ROBOT_LOG_BUFFER_SIZE;
    mydata->robot_log_buffer[idx] = (uint8_t)((now_ms >> 24) & 0xFF); idx = (idx + 1) % ROBOT_LOG_BUFFER_SIZE;
    mydata->robot_log_buffer[idx] = (uint8_t)mydata->phase; idx = (idx + 1) % ROBOT_LOG_BUFFER_SIZE;

    mydata->robot_log_head  = idx;
    mydata->robot_log_count += ROBOT_LOG_ENTRY_BYTES;
}


// Cyclic, non-destructive rebroadcast:
// - during the main "experiment" we return false (no TX)
// - after the record+wait window, we repeatedly stream the whole ring buffer
//   (oldest -> newest) in fixed-size chunks, restarting from the oldest byte
//   once a full pass has been completed.
static bool robots_send_log_chunk(void) {
    if (!mydata->log_broadcast_enabled) return false;
    if (mydata->robot_log_count == 0u) return false;

    // Start / restart a cycle if needed
    if (mydata->robot_bcast_remaining == 0u) {
        mydata->robot_bcast_cursor    = mydata->robot_log_tail;
        mydata->robot_bcast_remaining = mydata->robot_log_count;
    }

    message msg;
    // Ensure deterministic payload (also clears computed padding bytes)
    for (uint16_t i = 0; i < LOG_MESSAGE_BYTES; i++) {
        msg.payload[i] = 0;
    }
    msg.fields.sender_id = (uint16_t)pogobot_helper_getid();
    msg.fields.msg_type  = MSG_TYPE_LOG_CHUNK;

    uint16_t available = mydata->robot_bcast_remaining;
    uint8_t to_copy = (available > LOG_CHUNK_DATA_BYTES)
                      ? LOG_CHUNK_DATA_BYTES
                      : (uint8_t)available;

    msg.fields.chunk_len = to_copy;

    for (uint8_t i = 0; i < to_copy; i++) {
        uint16_t idx = (mydata->robot_bcast_cursor + i) % ROBOT_LOG_BUFFER_SIZE;
        msg.fields.chunk_data[i] = mydata->robot_log_buffer[idx];
    }
    for (uint8_t i = to_copy; i < LOG_CHUNK_DATA_BYTES; i++) {
        msg.fields.chunk_data[i] = 0;
    }

    // Return as status code (rc == 0 -> success)
    int rc = pogobot_infrared_sendLongMessage_omniGen(msg.payload, LOG_MESSAGE_BYTES);
    if (rc == 0) {
        mydata->robot_bcast_cursor    = (mydata->robot_bcast_cursor + to_copy) % ROBOT_LOG_BUFFER_SIZE;
        mydata->robot_bcast_remaining = (uint16_t)(mydata->robot_bcast_remaining - to_copy);
        return true;
    }
    return false;
}


void user_init(void) {
#ifndef SIMULATOR
    printf("robots: setup ok\n");
#endif
    srand(pogobot_helper_getRandSeed());

    // 20 Hz main loop
    main_loop_hz = 20;

    // Robots do not process incoming messages (just broadcast)
    max_nb_processed_msg_per_tick = 0;
    msg_rx_fn = NULL;
    msg_tx_fn = robots_send_log_chunk;
    error_codes_led_idx = 3;

    pogobot_infrared_set_power(2);

    // Initial run&tumble state
    mydata->phase = PHASE_TUMBLE;
    mydata->phase_start_time = current_time_milliseconds();
    mydata->phase_duration   = get_tumble_duration();
    mydata->tumble_direction = rand() % 2;

    // Logging / broadcasting state
    uint32_t now = current_time_milliseconds();
    mydata->sim_start_time_ms       = now;
    mydata->log_broadcast_enabled   = false;
    mydata->robot_log_head          = 0;
    mydata->robot_log_tail          = 0;
    mydata->robot_log_count         = 0;
    mydata->last_log_time_ms        = now;

    mydata->robot_bcast_cursor      = 0u;
    mydata->robot_bcast_remaining   = 0u;
}

void user_step(void) {
    uint32_t now = current_time_milliseconds();

    // === Run & tumble phase switching ===
    if (now - mydata->phase_start_time >= mydata->phase_duration) {
        // Switch phase
        if (mydata->phase == PHASE_RUN) {
            mydata->phase = PHASE_TUMBLE;
            mydata->phase_duration = get_tumble_duration();
            mydata->tumble_direction = rand() % 2;
            pogobot_led_setColor(255, 0, 0);   // red = tumble
        } else {
            mydata->phase = PHASE_RUN;
            mydata->phase_duration = get_run_duration();
            pogobot_led_setColor(0, 255, 0);   // green = run
        }
        mydata->phase_start_time = now;
    }

    // === Motors ===
    if (mydata->phase == PHASE_RUN) {
        pogobot_motor_set(motorL, motorFull);
        pogobot_motor_set(motorR, motorFull);
    } else { // PHASE_TUMBLE
        if (mydata->tumble_direction == 0) {
            pogobot_motor_set(motorL, motorStop);
            pogobot_motor_set(motorR, motorFull);
        } else {
            pogobot_motor_set(motorL, motorFull);
            pogobot_motor_set(motorR, motorStop);
        }
    }

    // === Logging during "experiment", then deferred cyclic rebroadcast ===
    uint32_t elapsed_ms = (uint32_t)(now - mydata->sim_start_time_ms);
    uint32_t record_ms  = (uint32_t)(log_record_duration_s * 1000u);
    uint32_t wait_ms    = (uint32_t)(log_post_wait_s * 1000u);

    // Record at 1 Hz for the first log_record_duration_s seconds.
    if (elapsed_ms < record_ms) {
        if (now - mydata->last_log_time_ms >= ROBOT_LOG_PERIOD_MS) {
            robot_log_state(now);
            // Reduce drift by stepping in fixed increments
            mydata->last_log_time_ms += ROBOT_LOG_PERIOD_MS;
        }
    } else {
        // After record_ms: stop recording.
        // After additional wait_ms: enable cyclic rebroadcast.
        if ((!mydata->log_broadcast_enabled) && (elapsed_ms >= (record_ms + wait_ms))) {
            mydata->log_broadcast_enabled = true;
            // Force cycle restart on first TX attempt
            mydata->robot_bcast_remaining = 0u;
        }
    }
}

static void probe_cache_push(uint16_t rid, uint32_t t_ms, uint8_t phase) {
    // If full, drop oldest entries to make room (keep most recent data)
    if (mydata->probe_cache_count >= PROBE_MAX_ENTRIES) {
        // Drop one oldest entry
        mydata->probe_cache_tail = (mydata->probe_cache_tail + 1) % PROBE_MAX_ENTRIES;
        mydata->probe_cache_count--;
    }

    uint16_t idx = mydata->probe_cache_head;

    mydata->probe_cache[idx].rid   = rid;
    mydata->probe_cache[idx].t_ms  = t_ms;
    mydata->probe_cache[idx].phase = phase;

    mydata->probe_cache_head  = (idx + 1) % PROBE_MAX_ENTRIES;
    mydata->probe_cache_count++;
}

// Find or allocate a per-sender partial slot -- simple Least-Recently-Used (LRU) eviction when full.
static ProbePartialSlot* probe_partial_get(uint16_t rid, uint32_t now_ms) {
    ProbePartialSlot *free_slot = NULL;
    ProbePartialSlot *lru_slot = &mydata->probe_partials[0];

    for (uint32_t i = 0; i < PROBE_PARTIAL_SLOTS; i++) {
        ProbePartialSlot *s = &mydata->probe_partials[i];
        if (s->rid == rid) {
            s->last_seen_ms = now_ms;
            return s;
        }
        if ((s->rid == PROBE_RID_UNUSED) && (free_slot == NULL)) {
            free_slot = s;
        }

        // Prefer evicting slots that are not in the middle of reassembly.
        bool s_is_better_victim = (s->len == 0) && (s->last_seen_ms <= lru_slot->last_seen_ms);
        bool lru_is_mid_entry = (lru_slot->len != 0);
        if (lru_is_mid_entry && (s->len == 0)) {
            lru_slot = s;
        } else if (!lru_is_mid_entry && s_is_better_victim) {
            lru_slot = s;
        } else if (lru_is_mid_entry && (s->last_seen_ms < lru_slot->last_seen_ms)) {
            // If we have to evict a mid-entry slot, use strict LRU.
            lru_slot = s;
        }
    }

    ProbePartialSlot *s = (free_slot != NULL) ? free_slot : lru_slot;
    s->rid = rid;
    s->len = 0;
    s->last_seen_ms = now_ms;
    return s;
}


void probes_process_message(message_t* mr) {
    message msg;

    // Copy the fixed-size payload (wire format is LOG_MESSAGE_BYTES bytes)
    for (uint16_t i = 0; i < LOG_MESSAGE_BYTES; i++) {
        msg.payload[i] = mr->payload[i];
    }

    if (msg.fields.msg_type != MSG_TYPE_LOG_CHUNK) {
        return;
    }

    uint32_t now_ms = current_time_milliseconds();
    uint16_t rid = msg.fields.sender_id;

    uint8_t chunk_len = msg.fields.chunk_len;
    if (chunk_len == 0) return;
    if (chunk_len > LOG_CHUNK_DATA_BYTES) {
        // Defensive clamp (should never happen if sender is correct)
        chunk_len = LOG_CHUNK_DATA_BYTES;
    }

    // Reconstruct 5-byte entries using per-sender partial buffer
    ProbePartialSlot *slot = probe_partial_get(rid, now_ms);
    uint8_t *pbuf = slot->bytes;
    uint8_t *plen = &slot->len;

    for (uint8_t i = 0; i < chunk_len; i++) {
        // Append next byte to partial entry
        if (*plen < ROBOT_LOG_ENTRY_BYTES) {
            pbuf[*plen] = msg.fields.chunk_data[i];
            (*plen)++;
        }

        // If we now have a full 5-byte entry, decode and push to cache
        if (*plen == ROBOT_LOG_ENTRY_BYTES) {
            uint32_t t =
                ((uint32_t)pbuf[0])       |
                ((uint32_t)pbuf[1] << 8 ) |
                ((uint32_t)pbuf[2] << 16) |
                ((uint32_t)pbuf[3] << 24);
            uint8_t phase = pbuf[4];

            probe_cache_push(rid, t, phase);

            // Reset for next entry
            *plen = 0;
        }
    }
}


void probes_user_init(void) {
#ifndef SIMULATOR
    printf("probes: setup ok\n");
#endif

    srand(pogobot_helper_getRandSeed());

    // Main loop is in 10 Hz
    main_loop_hz = 10;
    max_nb_processed_msg_per_tick = 10;
    msg_rx_fn = probes_process_message;
    msg_tx_fn = NULL;
    error_codes_led_idx = 2;

    mydata->probe_cache_head  = 0;
    mydata->probe_cache_tail  = 0;
    mydata->probe_cache_count = 0;

    for (uint32_t i = 0; i < PROBE_PARTIAL_SLOTS; i++) {
        mydata->probe_partials[i].rid = PROBE_RID_UNUSED;
        mydata->probe_partials[i].len = 0;
        mydata->probe_partials[i].last_seen_ms = 0;
    }
}

static const char* phase_to_str(uint8_t p) {
    return (p == PHASE_RUN) ? "RUN" : "TUMBLE";
}


void probes_user_step(void) {
    // Flush the cache: print all entries currently stored, in arrival order
    while (mydata->probe_cache_count > 0) {
        uint16_t idx = mydata->probe_cache_tail;
        ProbeCacheEntry *e = &mydata->probe_cache[idx];

        printf("PROBE %u: robot %u t=%lu ms state=%s (%u)\n",
               pogobot_helper_getid(),
               (unsigned int)e->rid,
               (unsigned long)e->t_ms,   // t_ms is a uint32_t in ms
               phase_to_str(e->phase),
               (unsigned int)e->phase);

        mydata->probe_cache_tail  = (idx + 1) % PROBE_MAX_ENTRIES;
        mydata->probe_cache_count--;
    }
}


#ifdef SIMULATOR
void global_setup() {
    init_from_configuration(run_duration_min);
    init_from_configuration(run_duration_max);
    init_from_configuration(tumble_duration_min);
    init_from_configuration(tumble_duration_max);
    init_from_configuration(log_record_duration_s);
    init_from_configuration(log_post_wait_s);
}
#endif


/**
 * @brief Program entry point.
 *
 * This function initializes the robot system and starts the main execution loop by
 * passing the user initialization and control functions to the platform's startup routine.
 *
 * @return int Returns 0 upon successful completion.
 */
int main(void) {
    // Initialization routine for the robots
    pogobot_init();
#ifndef SIMULATOR
    printf("init ok\n");
#endif

    // Start the robot's main loop with the defined user_init and user_step functions. Ignored by the probes.
    pogobot_start(user_init, user_step);
    // Use robots category "robots" by default. Same behavior as: pogobot_start(user_init, user_step, "robots");
    //   --> Make sure that the category "robots" is used to declare the pogobots in your configuration file. Cf conf/probes.yaml

    // Init and main loop functions for the "probes" category. Ignored by the "robots" category.
    pogobot_start(probes_user_init, probes_user_step, "probes");

    // Specify the callback functions. Only called by the simulator.
    SET_CALLBACK(callback_global_setup, global_setup);              // Called once to initialize global values (e.g. configuration-specified constants)
    return 0;
}

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
