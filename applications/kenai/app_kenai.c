/*
    Kenai Custom FW - Trolling Motor Steering State Machine using DC motor with hall A/B encoder and gearbox
    UART control via COMM_CUSTOM_APP_DATA (ID=36)

    Copyright 2016 - 2019 Benjamin Vedder    benjamin@vedder.se
    Modified by Kenai 20260703

    This file is part of the VESC firmware.
    GPL v3 — see <http://www.gnu.org/licenses/>.
*/

#include "app.h"
#include "ch.h"
#include "hal.h"

#include "mc_interface.h"
#include "utils.h"
#include "terminal.h"
#include "commands.h"
#include "timeout.h"
#include "buffer.h"
#include "conf_general.h"  
#include "utils_math.h"  
#include "utils_sys.h"  
#include "datatypes.h"  // for eeprom_var  
  
// Custom EEPROM slot (0..127 valid, see EEPROM_VARS_CUSTOM) used to persist the  
// last FAILSAFE reason across resets/power cycles for post-mortem diagnosis.  
#define KENAI_EEPROM_ADDR_FAILSAFE 0

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

// Helper: call utils_norm_angle on a volatile float without discarding qualifier
#define NORM_ANGLE(v)  do { float _norm_tmp = (v); utils_norm_angle(&_norm_tmp); (v) = _norm_tmp; } while(0)

// get_rpm() is output-shaft (post-gearbox); l_max_erpm is motor-shaft. Divide by ratio to match units.
#define KENAI_GEARBOX_RATIO 154.4f

// Safety margin: clamped_target stays this many deg short of a hard stop so PID
// error can reach zero before contact — else output holds near-max current forever.
#define STOP_APPROACH_MARGIN_DEG 0.1f

// Homing result below this span => invalid (e.g. hall disconnected/frozen encoder).
#define MIN_VALID_SPAN_DEG 10.0f

// ACTIVE-only no-hall runaway guard: high current + large persistent error + frozen encoder.  
#define NO_HALL_CURRENT_THRESH_A 3.0f  
#define NO_HALL_ERROR_THRESH_DEG 3.0f  
#define NO_HALL_POS_EPS_DEG      0.5f  
#define NO_HALL_TIME_S           0.35f  
  
// FAILSAFE reason codes — reported in MSG_GET_STATE so Pixhawk can differentiate a  
// recoverable comms timeout from a latched HW/encoder fault requiring kenai_stop.  
#define FAILSAFE_REASON_NONE         0  
#define FAILSAFE_REASON_UART_TIMEOUT 1  
#define FAILSAFE_REASON_NO_HALL      2  
  
// Only write to EEPROM if value actually changed, to avoid unnecessary flash wear.  
static void kenai_store_failsafe_reason_if_changed(uint8_t reason) {  
    eeprom_var v_old;  
    bool have_old = conf_general_read_eeprom_var_custom(&v_old, KENAI_EEPROM_ADDR_FAILSAFE);  
    if (!have_old || v_old.as_u32 != (uint32_t)reason) {  
        eeprom_var v_new; v_new.as_u32 = (uint32_t)reason;  
        conf_general_store_eeprom_var_custom(&v_new, KENAI_EEPROM_ADDR_FAILSAFE);  
    }  
}

// ============================================================
// ANGLE COORDINATE SPACES
// ============================================================
// ENC  — Encoder Absolute, 0°–360°. Source: mc_interface_get_pid_pos_now(), DIR_MULT applied.
//        p_pid_offset unused; trim = center_trim_deg (from p_pid_gain_dec_angle), +toward stop2
//        (field is >=0 clamped in VESC Tool — stop2-ward trim only). REL steering ref only.
//        p_pid_ang_div = HOMING_TIMEOUT_S (s), default 30.0. NEVER set 0.0 — bricks VESC if FOC set!
//
// REL  — Relative to Center, ±deg. 0=center, +toward stop1, -toward stop2. Pixhawk sends this
//        (MSG_SET_ANGLE). Vars: target_angle_deg, error, stow_err
//
// SPAN — Mechanical Span, 0°–360°, stop-to-stop, normalized. Direction-aware (encoder_inverted).
//        Vars: mech_span, range_limit
// ============================================================

// ============================================================
// Protocol message IDs  (COMM_CUSTOM_APP_DATA payload byte 0)
// ============================================================
#define MSG_DEPLOY      0x01    // Pixhawk → VESC: start homing
#define MSG_STOW        0x02    // Pixhawk → VESC: go to storage position
#define MSG_SET_ANGLE   0x03    // Pixhawk → VESC: set target angle (float32, deg from center)
#define MSG_GET_STATE   0x04    // Pixhawk → VESC: query state → reply

// ============================================================
// State machine types
// ============================================================
typedef enum {
    SERVO_STATE_IDLE     = 0,
    SERVO_STATE_HOMING   = 1,
    SERVO_STATE_ACTIVE   = 2,
    SERVO_STATE_STOWED   = 3,
    SERVO_STATE_FAILSAFE = 4,
} servo_state_t;

typedef enum {
    HOMING_PHASE_FIND_STOP1       = 0,
    HOMING_PHASE_FIND_STOP2       = 1,
    // 2 removed — was HOMING_PHASE_DRIVE_TO_CENTER, no longer used
    HOMING_PHASE_RETURN_TO_START  = 3,
} homing_phase_t;

// ============================================================
// Thread
// ============================================================
static THD_FUNCTION(control_thread, arg);
static THD_WORKING_AREA(control_thread_wa, 2048);  // was 1024, bumped after brick 2026-07-31

// ============================================================
// Private variables
// ============================================================
static volatile bool stop_now = true;
static volatile bool control_is_running = false;

static volatile servo_state_t  servo_state  = SERVO_STATE_IDLE;
static volatile homing_phase_t homing_phase = HOMING_PHASE_FIND_STOP1;

static volatile float stop1_angle      = 0.0f;  // [ENC] positive hard stop (higher encoder value)
static volatile float stop2_angle      = 0.0f;  // [ENC] negative hard stop (lower encoder value)
static volatile float center_angle     = 0.0f;  // [ENC] midpoint of mechanical arc
static volatile float storage_angle    = 0.0f;  // [ENC] stow position = center + storage_offset_deg

// Custom position PID state — ACTIVE state only
static float active_i_term    = 0.0f;
static float active_prev_error = 0.0f;
static float active_d_filter   = 0.0f;
static float active_prev_pos   = 0.0f;  // [REL] previous pos_now_rel for D-on-measurement

static volatile float target_angle_deg = 0.0f;  // [REL] commanded angle from Pixhawk, ±degrees from center

// [REL] symmetric clamp/scale limit, recomputed each tick. Default avoids /0.0f before first tick.
static volatile float sym_limit_deg = (270.0f / 2.0f) - STOP_APPROACH_MARGIN_DEG;

static volatile bool deploy_requested  = false;
static volatile bool stow_requested    = false;
static volatile bool stowed_reached = false;
static volatile bool in_deadband = false;
static volatile bool homing_completed  = false;
static volatile bool no_hall_fault_latched = false;  // latched FAILSAFE cause — cleared only by kenai_stop  
static volatile uint8_t failsafe_reason = FAILSAFE_REASON_NONE;  // last FAILSAFE cause, reported via MSG_GET_STATE  
static float no_hall_timer = 0.0f;  // ACTIVE-only accumulator for no-hall runaway guard

static volatile float homing_timer = 0.0f;
static volatile float stall_timer  = 0.0f;

static volatile systime_t last_cmd_time     = 0;
static volatile bool cmd_received_ever = false;

// Encoder direction detection — set during homing, used in all PID states
// true  = positive mc_interface_set_current() causes get_pid_pos_now() to DECREASE
//        (A/B wires physically swapped relative to motor winding direction)
// false = positive current causes position to INCREASE (normal wiring)
static volatile bool  encoder_inverted     = false;  // true if A/B wires physically swapped
static volatile bool  enc_dir_detected     = false;  // set true once direction confirmed in FIND_STOP1
static volatile float pos_at_homing_start    = 0.0f;  // [ENC] snapshot before FIND_STOP1
static volatile float mech_span              = 270.0f; // [SPAN] computed from homing, stop-to-stop degrees
static volatile float homing_traveled_deg    = 0.0f;  // accumulated travel in current homing phase
static volatile float homing_phase_prev_pos  = 0.0f;  // pos_now from previous tick for accumulation
static volatile float homing_return_distance_deg = 0.0f; // [SPAN] distance to travel back, latched at abort
static volatile float homing_return_current_sign  = 1.0f; // sign of current for RETURN_TO_START — opposite of aborted phase's direction

// ============================================================
// Forward declarations
// ============================================================
static void process_custom_app_data(unsigned char *data, unsigned int len);
static void terminal_kenai_state(int argc, const char **argv);
static void terminal_kenai_deploy(int argc, const char **argv);
static void terminal_kenai_stow(int argc, const char **argv);
static void terminal_kenai_angle(int argc, const char **argv);
static void terminal_kenai_stop(int argc, const char **argv) {
    (void)argc; (void)argv;
    deploy_requested = false;  
    stow_requested   = false;  
    no_hall_fault_latched = false;  // manual override — required to clear the no-hall latch  
    failsafe_reason  = FAILSAFE_REASON_NONE;  
    servo_state      = SERVO_STATE_IDLE;  
    mc_interface_release_motor();  
    commands_printf("Kenai: STOP → IDLE, motor released.\n");
}

static void terminal_kenai_set_stops(int argc, const char **argv) {
    if (argc == 3) {
        float stop1 = 0.0f, stop2 = 0.0f;
        sscanf(argv[1], "%f", &stop1);
        sscanf(argv[2], "%f", &stop2);
        const volatile mc_configuration *mcconf = mc_interface_get_configuration();
        float storage_offset_deg = mcconf->s_pid_min_erpm;
        stop1_angle = stop1;
        stop2_angle = stop2;
        // Direction-aware center — uses encoder_inverted from last homing (default false if never homed)
        float span;
        float center_angle_new;
        if (!encoder_inverted) {
            span             = stop1_angle - stop2_angle; // [SPAN] stop2→stop1 in positive direction
            if (span < 0.0f) span += 360.0f;
            center_angle_new = stop2_angle + span / 2.0f; // [ENC] midpoint of arc
        } else {
            span             = stop2_angle - stop1_angle; // [SPAN] stop1→stop2 in positive direction
            if (span < 0.0f) span += 360.0f;
            center_angle_new = stop1_angle + span / 2.0f; // [ENC] midpoint of arc
        }
        // Normalize before publishing so locked pair is always valid. encoder_inverted
        // is written only by this thread — no lock needed for it; center_angle+mech_span
        // is the pair needing atomicity vs terminal_kenai_set_stops() and control_thread's
        // own per-tick snapshot below — terminal_kenai_state() is a lock-free reader, not
        // part of this pairing.
        NORM_ANGLE(center_angle_new); // keep in [0°, 360°)
        if (center_angle_new >= 360.0f) center_angle_new = 0.0f;  // guard exact-360 edge case
        utils_sys_lock_cnt();
        center_angle = center_angle_new;
        mech_span    = span;  // persist for ACTIVE clamping and UART scaling
        utils_sys_unlock_cnt();
        if (!encoder_inverted) {
            storage_angle = center_angle + storage_offset_deg;
        } else {
            storage_angle = center_angle - storage_offset_deg;
        }
        NORM_ANGLE(storage_angle);                        // [ENC] stow position
        if (storage_angle >= 360.0f) storage_angle = 0.0f;  // guard exact-360 edge case
        homing_completed  = true;
        in_deadband       = false;
        active_i_term     = 0.0f;
        active_prev_error = 0.0f;
        active_d_filter   = 0.0f;
        {
            // Mirror DRIVE_TO_CENTER->ACTIVE: ACTIVE's PID reads pos_now_rel vs steering_center_angle
            // (trimmed, not plain center_angle). Seed active_prev_pos live — shaft may be anywhere.
            float _trim_ss = mcconf->p_pid_gain_dec_angle;   // +toward stop2 (see control_thread)
            float _steer_center_ss = center_angle;
            if (!encoder_inverted) {
                _steer_center_ss = center_angle - _trim_ss;
            } else {
                _steer_center_ss = center_angle + _trim_ss;
            }
            NORM_ANGLE(_steer_center_ss);
            if (_steer_center_ss >= 360.0f) _steer_center_ss = 0.0f;

            float _pos_ss = mc_interface_get_pid_pos_now();
            float _rel_ss = utils_angle_difference(_pos_ss, _steer_center_ss);
            if (encoder_inverted) _rel_ss = -_rel_ss;
            active_prev_pos = _rel_ss;
        }
        servo_state       = SERVO_STATE_ACTIVE;
        commands_printf("Kenai: stops set stop1=%.1f stop2=%.1f center=%.1f storage=%.1f → ACTIVE\n",
            (double)stop1, (double)stop2, (double)center_angle, (double)storage_angle);
    } else {
        commands_printf("Usage: kenai_set_stops [stop1_deg] [stop2_deg]\n");
    }
}

// ============================================================
// app_custom_start
// ============================================================
void app_custom_start(void) {
    app_uartcomm_start(UART_PORT_COMM_HEADER);   // START UART
	commands_set_app_data_handler(process_custom_app_data);

    terminal_register_command_callback(
            "kenai_state",
            "Print current servo state and angles.",
            0,
            terminal_kenai_state);

    terminal_register_command_callback(
            "kenai_deploy",
            "Start homing procedure.",
            0,
            terminal_kenai_deploy);

    terminal_register_command_callback(
            "kenai_stow",
            "Drive to storage position.",
            0,
            terminal_kenai_stow);

     terminal_register_command_callback(
            "kenai_angle",
            "Set target angle (degrees). Only works in ACTIVE state.",
            "[angle]",
            terminal_kenai_angle);

    terminal_register_command_callback(
            "kenai_stop",
            "Stop motor and return to IDLE state.",
            "",
            terminal_kenai_stop);

    terminal_register_command_callback(
            "kenai_set_stops",
            "Manually set hard stop angles, skips homing (for debugging).",
            "kenai_set_stops [stop1_deg] [stop2_deg] for example(45 315) ",
            terminal_kenai_set_stops);

    // No EEPROM restore — always boot to SERVO_STATE_IDLE (its default init value).
    // Encoder is incremental (hall A/B), so nothing meaningful survives a power cycle
    // anyway; a fresh deploy/homing is always required after boot regardless.
    last_cmd_time = chVTGetSystemTimeX();
    cmd_received_ever = false;

    stop_now = false;  
    control_is_running = true;   // set BEFORE thread starts to avoid race in app_custom_stop  
    timeout_configure_app_monitor(true);  // require this app's thread to check in, or IWDG resets MCU  
  
    {  
        eeprom_var v;  
        if (conf_general_read_eeprom_var_custom(&v, KENAI_EEPROM_ADDR_FAILSAFE)) {  
            commands_printf("Kenai: last saved failsafe_reason before this boot = %d", (int)v.as_u32);  
        }  
    }  
  
    chThdCreateStatic(control_thread_wa, sizeof(control_thread_wa),  
            NORMALPRIO, control_thread, NULL);  
	}

// ============================================================
// app_custom_stop
// ============================================================
void app_custom_stop(void) {
    commands_set_app_data_handler(0);
    terminal_unregister_callback(terminal_kenai_state);
    terminal_unregister_callback(terminal_kenai_deploy);
    terminal_unregister_callback(terminal_kenai_stow);
    terminal_unregister_callback(terminal_kenai_angle);
    terminal_unregister_callback(terminal_kenai_stop);
    terminal_unregister_callback(terminal_kenai_set_stops);

    stop_now = true;  
    while (control_is_running) {  
        chThdSleepMilliseconds(1);  
    }  
    timeout_configure_app_monitor(false);  // stop requiring THREAD_APP check-ins once stopped  
}

// ============================================================
// app_custom_configure
// ============================================================
void app_custom_configure(app_configuration *conf) {
    (void)conf;
}

// ============================================================
// UART packet handler
// ============================================================
static void process_custom_app_data(unsigned char *data, unsigned int len) {
    if (len < 1) return;

    // Every received packet resets the FAILSAFE timeout
    last_cmd_time = chVTGetSystemTimeX();
    cmd_received_ever = true;

    int32_t ind = 0;
    uint8_t msg = data[ind++];

    switch (msg) {


    case MSG_DEPLOY: {
        // Only trigger homing from IDLE or STOWED — ignore if already active/homing
        if (servo_state == SERVO_STATE_IDLE || servo_state == SERVO_STATE_STOWED) {
            deploy_requested = true;
            stow_requested   = false;
        }
        uint8_t tx[2]; int32_t ti = 0;
        tx[ti++] = (uint8_t)servo_state;
        tx[ti++] = msg;
        commands_send_app_data(tx, ti);
    } break;

    case MSG_STOW: {
        if (homing_completed) {
            stow_requested   = true;
            deploy_requested = false;
        }
        uint8_t tx[2]; int32_t ti = 0;
        tx[ti++] = (uint8_t)servo_state;
        tx[ti++] = msg;
        commands_send_app_data(tx, ti);
    } break;

    case MSG_SET_ANGLE: {
        if (len >= 3) {
            // sym_limit_deg is computed once per control-thread tick (single source of truth,
            // see control_thread) and simply read here — guarantees this scaling always matches
            // ACTIVE's clamp exactly, no duplicated formula to drift out of sync.
            target_angle_deg = (float)buffer_get_int16(data, &ind) * sym_limit_deg / 1000.0f;
        }
        // No ack — high-frequency command
    } break;

    case MSG_GET_STATE: {
        float pos_now = mc_interface_get_pid_pos_now();
        float current = mc_interface_get_tot_current_filtered();
        uint8_t tx[32]; int32_t ti = 0;
        tx[ti++] = msg;
        tx[ti++] = (uint8_t)servo_state;
        buffer_append_float32_auto(tx, pos_now,          &ti);
        buffer_append_float32_auto(tx, center_angle,     &ti);
        buffer_append_float32_auto(tx, storage_angle,    &ti);
        buffer_append_float32_auto(tx, target_angle_deg, &ti);
        buffer_append_float32_auto(tx, current,          &ti);
        commands_send_app_data(tx, ti);
    } break;

    default:
        break;
    }
}

// ============================================================
// Control thread — 200 Hz
// ============================================================
static THD_FUNCTION(control_thread, arg) {
    (void)arg;
    chRegSetThreadName("Kenai Servo");
    control_is_running = true;

    systime_t time_last = chVTGetSystemTimeX();

    for (;;) {  
        if (stop_now) {  
            control_is_running = false;  
            return;  
        }  
  
        timeout_feed_WDT(THREAD_APP);  // confirm this loop iteration is alive to the IWDG watchdog  
  
        float dt = (float)ST2MS(chVTTimeElapsedSinceX(time_last)) / 1000.0f;
        time_last = chVTGetSystemTimeX();

        // Read custom parameters from repurposed Speed PID fields
        const volatile mc_configuration *mcconf = mc_interface_get_configuration();
        float homing_current     = mcconf->s_pid_kp;             // Speed Kp field — SERVO_HOMING_CURRENT (A)
        float stall_rpm_thr      = mcconf->s_pid_ki;           	 // Speed Ki   → STALL_RPM_THR (ERPM)
		float active_deadband_deg = mcconf->s_pid_kd;            // Speed Kd   → ACTIVE_DEADBAND_DEG (deg)
        float storage_offset_deg = mcconf->s_pid_min_erpm;       // Min ERPM   → STORAGE_OFFSET_DEG (0.0)
        float storage_tol_deg    = mcconf->s_pid_ramp_erpms_s;   // Ramp       → STORAGE_TOL_DEG    (2.0)
        float homing_max_travel_deg = 300.0f;                        // hardcoded HOMING_MAX_TRAVEL_DEG
        float homing_timeout_s      = mcconf->p_pid_ang_div;         // Ang Div → HOMING_TIMEOUT_S (s), default 30.0. NEVER 0.0 — bricks VESC w/ FOC!
		float center_trim_deg       = mcconf->p_pid_gain_dec_angle;  // Gain Dec Angle → CENTER_TRIM_DEG [REL deg], +toward stop2, default 0.0
        float homing_max_rpm        = mcconf->s_pid_kd_filter * (mcconf->l_max_erpm / KENAI_GEARBOX_RATIO); // fraction of OUTPUT-shaft max RPM (e.g. 0.2 = ~20% of ~39 RPM)

        // Snapshot center_angle+mech_span together — the real cross-thread pair (both written
        // atomically by terminal_kenai_set_stops()/FIND_STOP2). encoder_inverted has no concurrent
        // writer besides this thread, so read it directly — no lock needed.
        float center_angle_snap;
        float range_limit;
        bool  encoder_inverted_snap = encoder_inverted;
        utils_sys_lock_cnt();
        center_angle_snap = center_angle;
        range_limit       = mech_span;
        utils_sys_unlock_cnt();

        // Recalculate storage_angle every loop (parameters may change via VESC Tool)
        // Direction-aware: positive REL = toward stop1 = positive current direction.
        // encoder_inverted=true: positive current decreases ENC, so subtract in ENC space.
     if (homing_completed) {
            if (!encoder_inverted_snap) {
                storage_angle = center_angle_snap + storage_offset_deg;
            } else {
                storage_angle = center_angle_snap - storage_offset_deg;
            }
            NORM_ANGLE(storage_angle);
				if (storage_angle >= 360.0f) storage_angle = 0.0f;  // guard exact-360 edge case
        }

        // Steering center = true midpoint + live trim. Used only for REL "straight ahead" ref
        // in ACTIVE — never for stop1/stop2/center_angle/range_limit, so hard-stop clamp and
        // re-homing keep the true geometric midpoint. center_trim_deg: +toward stop2
        // (field is >=0 clamped in VESC Tool — stop2-ward trim only).
        float steering_center_angle = center_angle_snap;
        if (homing_completed) {
            // !encoder_inverted: stop1 = higher ENC → SUBTRACT trim (toward stop2). encoder_inverted: ADD.
            if (!encoder_inverted_snap) {
                steering_center_angle = center_angle_snap - center_trim_deg;
            } else {
                steering_center_angle = center_angle_snap + center_trim_deg;
            }
            NORM_ANGLE(steering_center_angle);
            if (steering_center_angle >= 360.0f) steering_center_angle = 0.0f;
        }

        // Symmetric clamp/scale limit — SINGLE SOURCE OF TRUTH, computed once per tick here.
        // Consumed by ACTIVE's clamp below AND by MSG_SET_ANGLE's UART scaling in
        // process_custom_app_data() (a different thread) via the shared volatile sym_limit_deg,
        // so the two can never disagree/drift even if this formula changes later.
        {  
            float half_range_calc = range_limit / 2.0f;  
            float sym_limit_calc  = half_range_calc - fabsf(center_trim_deg) - STOP_APPROACH_MARGIN_DEG;  
            if (sym_limit_calc < 0.0f) sym_limit_calc = 0.0f; // guard: trim >= half_range (degenerate config)  
            if (sym_limit_calc > 93.0f) sym_limit_calc = 93.0f; // hard cap — never command more than ±93 deg from trimmed center, regardless of mechanical span  
            sym_limit_deg = sym_limit_calc;  
        }

        // --------------------------------------------------------
        // FAILSAFE check
        // Only fires in ACTIVE state; IDLE/STOWED/HOMING call timeout_reset() themselves
        // --------------------------------------------------------
        float cmd_age_s = (float)ST2MS(chVTTimeElapsedSinceX(last_cmd_time)) / 1000.0f;
        bool no_signal  = cmd_received_ever && (cmd_age_s > 2.0f);

                if (no_signal && servo_state == SERVO_STATE_ACTIVE) {  
                        servo_state       = SERVO_STATE_FAILSAFE;  
            failsafe_reason   = FAILSAFE_REASON_UART_TIMEOUT;  
            in_deadband       = false;  
            active_i_term     = 0.0f;  
            active_prev_error = 0.0f;  
            active_d_filter   = 0.0f;  
            active_prev_pos   = 0.0f;  
            mc_interface_release_motor();  
            kenai_store_failsafe_reason_if_changed(failsafe_reason);  
        }

        // --------------------------------------------------------
        // State machine
        // --------------------------------------------------------
        switch (servo_state) {

        // ---- IDLE -----------------------------------------------
        case SERVO_STATE_IDLE:
            timeout_reset();
            mc_interface_release_motor();
            if (deploy_requested) {
                deploy_requested  = false;
                stow_requested    = false;  // clear stale stow — prevents immediate stow after homing
                homing_phase      = HOMING_PHASE_FIND_STOP1;
                homing_timer      = 0.0f;
                stall_timer       = 0.0f;
                homing_completed  = false;
                pos_at_homing_start       = mc_interface_get_pid_pos_now();
                enc_dir_detected          = false;  // reset: re-detect direction each homing
                homing_traveled_deg       = 0.0f;
                homing_phase_prev_pos     = pos_at_homing_start;
                servo_state               = SERVO_STATE_HOMING;
                break;
            }
            break;

        // ---- HOMING ---------------------------------------------
        case SERVO_STATE_HOMING:
            homing_timer += dt;

            if (homing_timer > homing_timeout_s) {
                if (homing_phase == HOMING_PHASE_RETURN_TO_START) {
                    commands_printf("Kenai: RETURN TIMEOUT — IDLE");
                    mc_interface_release_motor();
                    servo_state = SERVO_STATE_IDLE;
                } else {
                    commands_printf("Kenai: HOMING TIMEOUT — returning to start");
                    mc_interface_set_current(0.0f);
                    // Latch direction/distance BEFORE overwriting homing_phase — FIND_STOP1 drove
                    // +homing_current, FIND_STOP2 drove -homing_current, so return is the opposite sign.
                    homing_return_distance_deg = homing_traveled_deg;
                    homing_return_current_sign = (homing_phase == HOMING_PHASE_FIND_STOP1) ? -1.0f : 1.0f;
                    homing_phase        = HOMING_PHASE_RETURN_TO_START;
                    homing_traveled_deg = 0.0f;
                    homing_timer        = 0.0f;
                }
                break;
            }

            {
                float pos_now = mc_interface_get_pid_pos_now();

                // Accumulate absolute traveled distance for abort check
                homing_traveled_deg += fabsf(utils_angle_difference(pos_now, homing_phase_prev_pos));
                homing_phase_prev_pos = pos_now;

                if (homing_traveled_deg > homing_max_travel_deg && homing_phase != HOMING_PHASE_RETURN_TO_START) {
                    commands_printf("Kenai: HOMING ABORT — traveled %.1f deg, returning to start",
                                    (double)homing_traveled_deg);
                    mc_interface_set_current(0.0f);
                    // Latch direction/distance BEFORE overwriting homing_phase — see timeout branch above.
                    homing_return_distance_deg = homing_traveled_deg;
                    homing_return_current_sign = (homing_phase == HOMING_PHASE_FIND_STOP1) ? -1.0f : 1.0f;
                    homing_phase        = HOMING_PHASE_RETURN_TO_START;
                    homing_traveled_deg = 0.0f;
                    homing_timer        = 0.0f;
                    break;
                }

                switch (homing_phase) {

                case HOMING_PHASE_FIND_STOP1:
                    {
                        float _hcur1 = homing_current;
                        if (homing_max_rpm > 0.0f) {
                            // Taper only in top 20% of speed range (like stock l_erpm_start)
                            float _rpm1 = fabsf(mc_interface_get_rpm());
                            float _s1 = utils_map(_rpm1, homing_max_rpm * 0.8f, homing_max_rpm, 1.0f, 0.1f);
                            utils_truncate_number(&_s1, 0.1f, 1.0f);
                            _hcur1 *= _s1;
                        }
                        mc_interface_set_current(_hcur1);
                    }
					timeout_reset();

                    // Early wrap-safe direction sample (mirrors FIND_STOP2) — avoids relying on
                    // full end-of-travel angle, which can exceed 180 deg and flip sign on wrap.
                    if (!enc_dir_detected && homing_traveled_deg >= 5.0f) {
                        encoder_inverted = (utils_angle_difference(pos_now, pos_at_homing_start) < 0.0f);
                        enc_dir_detected = true;
                        commands_printf("Kenai: enc_inverted=%d (detected during stop1 travel)", (int)encoder_inverted);
                    }
                  //  if (current > stall_thr) { stall_timer += dt; }  //not in use curently switch for rpm
				  if (homing_timer > 0.5f && fabsf(mc_interface_get_rpm()) < stall_rpm_thr) { stall_timer += dt; }
                    else                      { stall_timer  = 0.0f; }
                     if (stall_timer > 0.3f) {
                        stop1_angle = pos_now;
                        stall_timer = 0.0f;
                        // Direction detection from FIND_STOP1 movement.
                        // Only reliable if motor moved >=5 deg before stalling.
                        // If motor started at stop1, enc_dir_detected stays false
                        // and detection is deferred to end of FIND_STOP2.
                        if (!enc_dir_detected) {
                            float movement = fabsf(utils_angle_difference(stop1_angle, pos_at_homing_start));
                            if (movement >= 5.0f) {
                                encoder_inverted = (utils_angle_difference(stop1_angle, pos_at_homing_start) < 0.0f);
                                enc_dir_detected = true;
                                commands_printf("Kenai: Stop1=%.1f enc_inverted=%d (from stop1 move %.1f deg)",
                                        (double)stop1_angle, (int)encoder_inverted, (double)movement);
                            } else {
                                commands_printf("Kenai: Stop1=%.1f moved only %.1f deg — deferring direction detect to stop2",
                                        (double)stop1_angle, (double)movement);
                            }
                        }
                        homing_timer          = 0.0f;  // reset timer for phase 2
                        homing_traveled_deg   = 0.0f;  // reset travel counter for phase 2
                        homing_phase_prev_pos = pos_now;
                        homing_phase          = HOMING_PHASE_FIND_STOP2;
                    }
                    break;

                case HOMING_PHASE_FIND_STOP2:
                    // mc_interface_set_pid_speed(-homing_erpm);  //not in use curently switch for current
                    {
                        float _hcur2 = homing_current;
                        if (homing_max_rpm > 0.0f) {
                            float _rpm2 = fabsf(mc_interface_get_rpm());
                            float _s2 = utils_map(_rpm2, homing_max_rpm * 0.8f, homing_max_rpm, 1.0f, 0.1f);
                            utils_truncate_number(&_s2, 0.1f, 1.0f);
                            _hcur2 *= _s2;
                        }
                        mc_interface_set_current(-_hcur2);
                    }
					timeout_reset();

                    if (!enc_dir_detected && homing_traveled_deg >= 5.0f) {
                        encoder_inverted = (utils_angle_difference(pos_now, stop1_angle) > 0.0f);
                        enc_dir_detected = true;
                        commands_printf("Kenai: enc_inverted=%d (detected during stop2 travel)", (int)encoder_inverted);
                    }
                   // if (current > stall_thr) { stall_timer += dt; }  //not in use curently switch for rpm
                   if (homing_timer > 0.5f && fabsf(mc_interface_get_rpm()) < stall_rpm_thr) { stall_timer += dt; }
				   else                      { stall_timer  = 0.0f; }

                     if (stall_timer > 0.3f) {
                        stop2_angle = pos_now;
                        stall_timer = 0.0f;

                        // Fallback direction detection from FIND_STOP2 movement.
                        // Negative current drove motor from stop1 to stop2.
                        // utils_angle_difference(stop2, stop1) > 0: negative current increased ENC = inverted.
                        // utils_angle_difference(stop2, stop1) < 0: negative current decreased ENC = normal.
                        // Only reached if both 5-deg early checks never fired (near-zero span,
                        // so angle stays <<180 deg) — NOT a general full-span check, since
                        // utils_angle_difference() flips sign on spans > 180 deg.
                        if (!enc_dir_detected) {
                            encoder_inverted = (utils_angle_difference(stop2_angle, stop1_angle) > 0.0f);
                            enc_dir_detected = true;
                            commands_printf("Kenai: enc_inverted=%d (from stop2 move — motor started at stop1)",
                                    (int)encoder_inverted);
                        }

                        // Compute center and storage
                        // Correct midpoint: travel from stop2 to stop1 in positive direction,
                        // then go half that distance. Handles 0/360 boundary correctly.
                        // Direction-aware center: arc goes from negative-stop to positive-stop.
                        // encoder_inverted=false: stop1 > stop2 → arc = stop1-stop2, center = stop2+span/2
                        // encoder_inverted=true:  stop2 > stop1 → arc = stop2-stop1, center = stop1+span/2
                        float span;
                        float center_angle_new;
                        if (!encoder_inverted) {
                            span             = stop1_angle - stop2_angle;
                            if (span < 0.0f) span += 360.0f;
                            center_angle_new = stop2_angle + span / 2.0f; // [ENC]
                        } else {
                            span             = stop2_angle - stop1_angle;
                            if (span < 0.0f) span += 360.0f;
                            center_angle_new = stop1_angle + span / 2.0f; // [ENC]
                        }

                        // Invalid span (hall disconnected/frozen encoder => stop1≈stop2≈start) —
                        // latch FAILSAFE instead of publishing a bogus center/entering ACTIVE. A live
                        // UART signal alone (MSG_GET_STATE/MSG_SET_ANGLE) must NOT clear this — the
                        // fault is the encoder, not comms; only kenai_stop clears no_hall_fault_latched.
                        if (span < MIN_VALID_SPAN_DEG) {
                            commands_printf("Kenai: HOMING INVALID — span %.1f deg (<%.1f) — no encoder motion — FAILSAFE",
                                    (double)span, (double)MIN_VALID_SPAN_DEG);
                            mc_interface_release_motor();
                            no_hall_fault_latched = true;  
                            failsafe_reason = FAILSAFE_REASON_NO_HALL;  
                            kenai_store_failsafe_reason_if_changed(failsafe_reason);
                            servo_state = SERVO_STATE_FAILSAFE;
                            break;
                        }

                        // Normalize before publishing so locked pair is always valid. encoder_inverted
                        // is written only by this thread — no lock needed for it; center_angle+mech_span
                        // is the pair needing atomicity vs terminal_kenai_set_stops() and control_thread's
                        // own per-tick snapshot above — terminal_kenai_state() is a lock-free reader, not
                        // part of this pairing.
                        NORM_ANGLE(center_angle_new); // keep in [0°, 360°)
                        if (center_angle_new >= 360.0f) center_angle_new = 0.0f;  // guard exact-360 edge case
                        utils_sys_lock_cnt();
                        center_angle = center_angle_new;
                        mech_span    = span;  // persist for ACTIVE clamping and UART scaling
                        utils_sys_unlock_cnt();
                        // storage_offset_deg is motor-space REL; convert to ENC-space
                        if (!encoder_inverted) {
                            storage_angle = center_angle + storage_offset_deg;
                        } else {
                            storage_angle = center_angle - storage_offset_deg;
                        }
                        NORM_ANGLE(storage_angle);                   // [ENC] stow position
                        if (storage_angle >= 360.0f) storage_angle = 0.0f;  // guard exact-360 edge case
                        homing_completed  = true;

                        commands_printf("Kenai: Stop2=%.1f  Center=%.1f  Storage=%.1f — ACTIVE",
                                (double)stop2_angle, (double)center_angle, (double)storage_angle);

                        in_deadband       = false;
                        active_i_term     = 0.0f;
                        active_prev_error = 0.0f;
                        active_d_filter   = 0.0f;
                        {
                            // steering_center_angle (computed at top of loop) is STALE here —
                            // it was built from this tick's *old* homing_completed/center_angle,
                            // before we just set them above. Recompute locally, same pattern as
                            // terminal_kenai_set_stops()'s _steer_center_ss, to avoid seeding
                            // active_prev_pos with a bogus value (spurious D-kick on 1st ACTIVE tick).
                            float _steer_center_now = center_angle;
                            if (!encoder_inverted) {
                                _steer_center_now = center_angle - center_trim_deg;
                            } else {
                                _steer_center_now = center_angle + center_trim_deg;
                            }
                            NORM_ANGLE(_steer_center_now);
                            if (_steer_center_now >= 360.0f) _steer_center_now = 0.0f;

                            float _rel_active0 = utils_angle_difference(pos_now, _steer_center_now);
                            if (encoder_inverted) _rel_active0 = -_rel_active0;
                            active_prev_pos = _rel_active0;
                        }
                        time_last   = chVTGetSystemTimeX(); // reset dt so ACTIVE's 1st tick isn't stretched by homing-completion work above
                        servo_state = SERVO_STATE_ACTIVE;
                    }
                    break;

                case HOMING_PHASE_RETURN_TO_START: {
                    timeout_reset();
                    // Option 1: reverse known direction for the same distance traveled — no absolute-angle
                    // compare. homing_traveled_deg reuses the safe <=180deg/tick delta sum (runs every
                    // phase, see top of block), so it's correct past 180deg/multiple wraps, unlike
                    // utils_angle_difference() on far-apart absolute angles (shortest-path only, <=180deg).
                    if (homing_traveled_deg >= homing_return_distance_deg) {
                        commands_printf("Kenai: Returned to start (%.1f/%.1f deg) — IDLE",
                                (double)homing_traveled_deg, (double)homing_return_distance_deg);
                        mc_interface_release_motor();
                        deploy_requested = false;
                        servo_state = SERVO_STATE_IDLE;
                    } else {
                        // Fixed current, known-opposite polarity — sidesteps wraparound entirely.
                        mc_interface_set_current(homing_return_current_sign * homing_current);
                    }
                    break;
                }
                }
            }
            break;

                // ---- ACTIVE ---------------------------------------------
        case SERVO_STATE_ACTIVE:
    if (stow_requested) {
                stow_requested  = false;
                stowed_reached  = false;
                in_deadband     = false;
                active_i_term     = 0.0f;
                active_prev_error = 0.0f;
                active_d_filter   = 0.0f;
                {
                    // STOWED's own PID below computes pos_now_rel_s against center_angle_snap
                    // (untrimmed, this tick's snapshot) — seed active_prev_pos in the SAME frame/
                    // same snapshot, or the first STOWED tick sees a spurious jump (bug4 recurrence)
                    // and/or a torn read vs. STOWED's own use of center_angle_snap below.
                    float _pos_for_stow = mc_interface_get_pid_pos_now();
                    float _rel_for_stow = utils_angle_difference(_pos_for_stow, center_angle_snap);
                    if (encoder_inverted_snap) _rel_for_stow = -_rel_for_stow;
                    active_prev_pos = _rel_for_stow;
                }
                servo_state       = SERVO_STATE_STOWED;
                break;
            }

            {
                    // sym_limit_deg computed once per tick above (single source of truth,
                    // shared with MSG_SET_ANGLE's UART scaling) — no duplicated formula here.
                    float clamped_target = target_angle_deg; // [REL] motor-space
                    utils_truncate_number(&clamped_target, -sym_limit_deg, sym_limit_deg);

                float pos_now_act = mc_interface_get_pid_pos_now();          // [ENC] current position

                // ENC→motor-space REL: utils_angle_difference gives +=ENC-increasing.
                // inverted=true: +current decreases ENC, so flip sign so += toward stop1
                float pos_now_rel = utils_angle_difference(pos_now_act, steering_center_angle); // [REL] ENC-space, trimmed center
                if (encoder_inverted_snap) pos_now_rel = -pos_now_rel;      // flip to motor-space REL

                float error = clamped_target - pos_now_rel;

                // Deadband with hysteresis
                if (fabsf(error) < active_deadband_deg) {
                    if (!in_deadband) {
                        active_d_filter = 0.0f;  // flush stale D on deadband entry
                    }
                    in_deadband = true;
                } else if (fabsf(error) > active_deadband_deg * 2.0f) {
                    in_deadband = false;
                }

				if (in_deadband) {
					mc_interface_set_brake_current(mcconf->lo_current_max * 0.05f);  // ~5% braking
				} else {
                    // Custom position PID driving current directly.
                    // Gains reuse p_pid_kp / p_pid_ki / p_pid_kd from mcconf.
                    float p_term = error * mcconf->p_pid_kp;

                    {  
                        float i_candidate = active_i_term + error * mcconf->p_pid_ki * dt;  
  
                        // Only accept the new integration step if it doesn't push total output  
                        // deeper into saturation; otherwise freeze (keep old active_i_term).  
                        float provisional_output = p_term + i_candidate + active_d_filter;  
                        if (fabsf(provisional_output) < 1.0f || (provisional_output * error) < 0.0f) {  
                            active_i_term = i_candidate;  
                        }  
                        utils_truncate_number(&active_i_term, -1.0f, 1.0f); // still keep a hard ceiling as a safety backstop  
                    }

                    float d_raw = 0.0f;
                    if (dt > 1e-6f) {
                        float pos_change = utils_angle_difference(pos_now_rel, active_prev_pos);
                        d_raw = -pos_change * mcconf->p_pid_kd / dt;
                    }
                    UTILS_LP_FAST(active_d_filter, d_raw, mcconf->p_pid_kd_filter);

                    float output = p_term + active_i_term + active_d_filter;  
                    utils_truncate_number(&output, -1.0f, 1.0f);  
                    float _out_lim = (output >= 0.0f) ? mcconf->lo_current_max : fabsf(mcconf->lo_current_min);  
                    mc_interface_set_current(output * _out_lim);
                }

                // No-hall runaway guard: high current + large persistent error + frozen encoder
                // (<NO_HALL_POS_EPS_DEG movement) held for NO_HALL_TIME_S => hall disconnected/failed.
                // Excludes correct high-current holding/braking (error~0) and homing (own guard above).
                if (fabsf(mc_interface_get_tot_current_filtered()) > NO_HALL_CURRENT_THRESH_A &&
                    fabsf(error) > NO_HALL_ERROR_THRESH_DEG &&
                    fabsf(utils_angle_difference(pos_now_rel, active_prev_pos)) < NO_HALL_POS_EPS_DEG) {
                    no_hall_timer += dt;
                } else {
                    no_hall_timer = 0.0f;
                }
                if (no_hall_timer > NO_HALL_TIME_S) {  
                    commands_printf("Kenai: NO-HALL GUARD — high current, frozen encoder, large error — FAILSAFE");  
                    no_hall_fault_latched = true;  
                    failsafe_reason   = FAILSAFE_REASON_NO_HALL;  
                    servo_state       = SERVO_STATE_FAILSAFE;  
                    in_deadband       = false;  
                    active_i_term     = 0.0f;  
                    active_prev_error = 0.0f;  
                    active_d_filter   = 0.0f;  
                    active_prev_pos   = 0.0f;  
                    no_hall_timer     = 0.0f;  
                    mc_interface_release_motor();  
                    kenai_store_failsafe_reason_if_changed(failsafe_reason);  
                    break;  
                }

                active_prev_error = error;
                active_prev_pos   = pos_now_rel;
                timeout_reset();
            }
            break;

        // ---- STOWED ---------------------------------------------
        case SERVO_STATE_STOWED:
            timeout_reset();

            if (deploy_requested) {
                deploy_requested = false;
                stow_requested   = false;  // clear stale stow — prevents immediate stow after homing
                homing_phase     = HOMING_PHASE_FIND_STOP1;
                homing_timer     = 0.0f;
                stall_timer      = 0.0f;
                stowed_reached   = false;
                homing_completed = false;
                pos_at_homing_start   = mc_interface_get_pid_pos_now(); // [ENC] snapshot for direction detect
                enc_dir_detected      = false;  // reset: re-detect direction each homing
                homing_traveled_deg   = 0.0f;
                homing_phase_prev_pos = pos_at_homing_start;
				servo_state           = SERVO_STATE_HOMING;
                break;
            }
            if (!stowed_reached) {
                float pos_now   = mc_interface_get_pid_pos_now();  // [ENC] current position

                // Compute error in motor-space REL (same convention as ACTIVE state).
                float pos_now_rel_s = utils_angle_difference(pos_now, center_angle_snap);     // [REL] ENC-space
                if (encoder_inverted_snap) pos_now_rel_s = -pos_now_rel_s;                   // flip to motor-space
                float storage_rel   = utils_angle_difference(storage_angle, center_angle_snap); // [REL] ENC-space
                if (encoder_inverted_snap) storage_rel = -storage_rel;                        // flip to motor-space
                float stow_err = storage_rel - pos_now_rel_s;

                if (fabsf(stow_err) < storage_tol_deg) {
                    stowed_reached    = true;
                    active_i_term     = 0.0f;
                    active_prev_error = 0.0f;
                    active_d_filter   = 0.0f;
                    mc_interface_release_motor();  // storage reached — release motor, rely on mechanical lock
                } else {
                    // Same custom PID as ACTIVE — reuses active_* PID state variables  
                    float p_term = stow_err * mcconf->p_pid_kp;  
                    {  
                        float i_candidate = active_i_term + stow_err * mcconf->p_pid_ki * dt;  
  
                        // Only accept the new integration step if it doesn't push total output  
                        // deeper into saturation; otherwise freeze (keep old active_i_term).  
                        float provisional_output = p_term + i_candidate + active_d_filter;  
                        if (fabsf(provisional_output) < 1.0f || (provisional_output * stow_err) < 0.0f) {  
                            active_i_term = i_candidate;  
                        }  
                        utils_truncate_number(&active_i_term, -1.0f, 1.0f); // still keep a hard ceiling as a safety backstop  
                    }

                        float d_raw = 0.0f;
                        if (dt > 1e-6f) {
                            float pos_change_s = utils_angle_difference(pos_now_rel_s, active_prev_pos);
                            d_raw = -pos_change_s * mcconf->p_pid_kd / dt;
                        }
                    UTILS_LP_FAST(active_d_filter, d_raw, mcconf->p_pid_kd_filter);
                    float output = p_term + active_i_term + active_d_filter;  
                    utils_truncate_number(&output, -1.0f, 1.0f);  
                    float _out_lim = (output >= 0.0f) ? mcconf->lo_current_max : fabsf(mcconf->lo_current_min);  
                    mc_interface_set_current(output * _out_lim);  
                    active_prev_error = stow_err;
                    active_prev_pos   = pos_now_rel_s;
                    timeout_reset();
                }
            } else {
                mc_interface_release_motor();  // already stowed — keep motor released every tick
            }
            break;

        // ---- FAILSAFE -------------------------------------------
        case SERVO_STATE_FAILSAFE:
            timeout_reset();
			mc_interface_release_motor();

            // Signal restored → back to IDLE — but NOT if latched by the no-hall guard/homing-span  
            // check, since that fault is the encoder, not comms; only kenai_stop clears the latch.  
            if (!no_signal && !no_hall_fault_latched) {  
                commands_printf("Kenai: Signal restored — IDLE");  
                failsafe_reason = FAILSAFE_REASON_NONE;  
                servo_state = SERVO_STATE_IDLE;  
            }  
            break;

        default:
            servo_state = SERVO_STATE_IDLE;
            mc_interface_release_motor();
            break;
        }

        chThdSleepMilliseconds(5); // 200 Hz
    }
}

// ============================================================
// Terminal commands
// ============================================================
static void terminal_kenai_state(int argc, const char **argv) {
    (void)argc; (void)argv;
    const char *names[] = {"IDLE", "HOMING", "ACTIVE", "STOWED", "FAILSAFE"};
    int s = (int)servo_state;
    commands_printf("  Kenai state : %d (%s)", s, (s >= 0 && s <= 4) ? names[s] : "?");
    commands_printf("  pos_now     : %.2f deg [ENC]", (double)mc_interface_get_pid_pos_now());
    commands_printf("  center      : %.2f deg [ENC]", (double)center_angle);
    commands_printf("  trim        : %.2f deg [REL, +toward stop2] (from p_pid_gain_dec_angle)", (double)mc_interface_get_configuration()->p_pid_gain_dec_angle);
    commands_printf("  storage     : %.2f deg [ENC]", (double)storage_angle);
    commands_printf("  target      : %.2f deg [REL from center]", (double)target_angle_deg);
    commands_printf("  current     : %.2f A",   (double)mc_interface_get_tot_current_filtered());
    commands_printf("  homing_done : %d",        homing_completed);
    commands_printf("  stowed_done : %d",        stowed_reached);
    commands_printf("  enc_inverted: %d",        (int)encoder_inverted);  
    commands_printf("  no_hall_flt : %d",        (int)no_hall_fault_latched);  
    // fs_reason legend: 0=NONE (no failsafe), 1=UART_TIMEOUT (comms lost, auto-recovers on signal),  
    //                    2=NO_HALL (latched HW/encoder fault — requires kenai_stop to clear)  
    commands_printf("  fs_reason   : %d",        (int)failsafe_reason); 
    {  
        eeprom_var v;  
        if (conf_general_read_eeprom_var_custom(&v, KENAI_EEPROM_ADDR_FAILSAFE)) {  
            commands_printf("  fs_reason(0=NONE, 1=UART_TIMEOUT, 2=NO_HALL): %d", (int)v.as_u32);  
        }  
    }  
}

static void terminal_kenai_deploy(int argc, const char **argv) {
    (void)argc; (void)argv;
    deploy_requested = true;
    stow_requested   = false;
    commands_printf("Kenai: DEPLOY requested");
}

static void terminal_kenai_stow(int argc, const char **argv) {
    (void)argc; (void)argv;
    if (!homing_completed) {
        commands_printf("Kenai: STOW rejected — homing not done (run kenai_deploy first or kenai_set_stops)");
        return;
    }
    stow_requested   = true;
    deploy_requested = false;
    commands_printf("Kenai: STOW requested");
}

static void terminal_kenai_angle(int argc, const char **argv) {
    if (argc == 2) {
        float angle = (float)atof(argv[1]);
        target_angle_deg = angle;
        commands_printf("Kenai: target angle = %.2f deg", (double)angle);
    } else {
        commands_printf("Usage: kenai_angle <degrees>");
    }
}
