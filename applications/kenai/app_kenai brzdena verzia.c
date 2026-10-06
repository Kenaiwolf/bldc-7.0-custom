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
  
#include <math.h>  
#include <string.h>  
#include <stdio.h>  
#include <stdint.h>  
#include <stdbool.h>  
#include <stdlib.h>  
  
// Helper: call utils_norm_angle on a volatile float without discarding qualifier  
#define NORM_ANGLE(v)  do { float _norm_tmp = (v); utils_norm_angle(&_norm_tmp); (v) = _norm_tmp; } while(0)
  
// ============================================================  
// ANGLE COORDINATE SPACES  
// ============================================================  
// ENC  — Encoder Absolute, 0°–360°  
//        Source: mc_interface_get_pid_pos_now()  
//        DIR_MULT and p_pid_offset already applied by VESC API.  
//        Positive direction = direction of positive mc_interface_set_current().  
//        stop1 = positive-current hard stop; stop2 = negative-current hard stop.    
//        encoder_inverted=false: stop1 > stop2 in ENC (normal A/B wiring).    
//        encoder_inverted=true:  stop2 > stop1 in ENC (A/B wires physically swapped).  
//        Variables: stop1_angle, stop2_angle, center_angle, storage_angle,  
//                   pos_now, abs_target  
//  
// REL  — Relative to Center, ±degrees  
//        0° = center, positive = toward stop1, negative = toward stop2.  
//        This is what the Pixhawk sends (MSG_SET_ANGLE payload).  
//        Variables: target_angle_deg, error, stow_err  
//  
// SPAN — Mechanical Span, 0°–360°  
//        Total arc of the mechanical range (stop-to-stop), normalized to [0°, 360°).    
//        Formula is direction-aware: see encoder_inverted flag.
//        Variables: mech_span, range_limit  
// ============================================================  
  
// ============================================================  
// Protocol message IDs  (COMM_CUSTOM_APP_DATA payload byte 0)  
// ============================================================
#define MSG_DEPLOY      0x01    // Pixhawk → VESC: start homing  
#define MSG_STOW        0x02    // Pixhawk → VESC: go to storage position  
#define MSG_SET_ANGLE   0x03    // Pixhawk → VESC: set target angle (float32, deg from center)  
#define MSG_GET_STATE   0x04    // Pixhawk → VESC: query state → reply  
  
// ============================================================  
// EEPROM addresses (custom, 0-31)  
// ============================================================  
#define EEPROM_ADDR_STATE        0   // uint32: servo_state_t (STOWED=3 or IDLE=0)    
#define EEPROM_ADDR_CENTER       1   // float:  center_angle (absolute encoder degrees)    
#define EEPROM_ADDR_ENCODER_DIR  2   // uint32: encoder_inverted (0=normal, 1=A/B swapped)  
  
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
    HOMING_PHASE_FIND_STOP1       = 0,  // Drive positive until stall    
    HOMING_PHASE_FIND_STOP2       = 1,  // Drive negative until stall    
    HOMING_PHASE_DRIVE_TO_CENTER  = 2,  // Drive to computed center    
    HOMING_PHASE_RETURN_TO_START  = 3,  // Return to pre-homing position on abort/timeout  
} homing_phase_t;  
  
// ============================================================  
// Thread  
// ============================================================  
static THD_FUNCTION(control_thread, arg);  
static THD_WORKING_AREA(control_thread_wa, 1024);  
  
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
  
static volatile bool deploy_requested  = false;  
static volatile bool stow_requested    = false;  
static volatile bool stowed_reached = false;  
static volatile bool in_deadband = false;  
static volatile bool homing_completed  = false;  
  
static volatile float homing_timer = 0.0f;  
static volatile float stall_timer  = 0.0f;  
  
static volatile systime_t last_cmd_time     = 0;  
static volatile bool cmd_received_ever = false;  
  
// Deferred EEPROM write flag — center save moved out of hard-stop moment  
static volatile bool pending_save_center = false;  
  
// Encoder direction detection — set during homing, used in all PID states  
// true  = positive mc_interface_set_current() causes get_pid_pos_now() to DECREASE  
//         (A/B wires physically swapped relative to motor winding direction)  
// false = positive current causes position to INCREASE (normal wiring)  
static volatile bool  encoder_inverted     = false;  // true if A/B wires physically swapped  
static volatile bool  enc_dir_detected     = false;  // set true once direction confirmed in FIND_STOP1  
static volatile float pos_at_homing_start    = 0.0f;  // [ENC] snapshot before FIND_STOP1   
static volatile float mech_span              = 270.0f; // [SPAN] computed from homing, stop-to-stop degrees  
static volatile float homing_traveled_deg    = 0.0f;  // accumulated travel in current homing phase    
static volatile float homing_phase_prev_pos  = 0.0f;  // pos_now from previous tick for accumulation   
static volatile homing_phase_t homing_abort_phase = HOMING_PHASE_FIND_STOP1; // phase that caused abort  
static volatile bool homing_braking = false; // true while actively brake-decelerating in FIND_STOP1/2
  
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
    servo_state      = SERVO_STATE_IDLE;      
    mc_interface_set_current(0.0f);      
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
        if (!encoder_inverted) {    
            span         = stop1_angle - stop2_angle; // [SPAN] stop2→stop1 in positive direction    
            if (span < 0.0f) span += 360.0f;    
            center_angle = stop2_angle + span / 2.0f; // [ENC] midpoint of arc    
        } else {    
            span         = stop2_angle - stop1_angle; // [SPAN] stop1→stop2 in positive direction    
            if (span < 0.0f) span += 360.0f;    
            center_angle = stop1_angle + span / 2.0f; // [ENC] midpoint of arc    
        }  
        mech_span = span;  // persist for ACTIVE clamping and UART scaling  
        NORM_ANGLE(center_angle); // keep in [0°, 360°)  
        if (center_angle >= 360.0f) center_angle = 0.0f;  // guard exact-360 edge case  
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
        active_prev_pos   = 0.0f;      
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
  
    // Restore state from EEPROM  
    eeprom_var v;  
    if (conf_general_read_eeprom_var_custom(&v, EEPROM_ADDR_CENTER)) {  
        center_angle = v.as_float;  
        homing_completed = true;  
        // storage_angle will be recalculated on first control loop iteration  
        storage_angle = center_angle;  
    }  
    if (conf_general_read_eeprom_var_custom(&v, EEPROM_ADDR_ENCODER_DIR)) {    
        encoder_inverted = (bool)v.as_u32;    
    }    
    if (conf_general_read_eeprom_var_custom(&v, EEPROM_ADDR_STATE)) {    
        if ((servo_state_t)v.as_u32 == SERVO_STATE_STOWED) { 
            servo_state = SERVO_STATE_STOWED;  
            stowed_reached = false; // Will drive to storage_angle on first loop  
        }  
    }  
  
    last_cmd_time = chVTGetSystemTimeX();  
    cmd_received_ever = false;  
  
    stop_now = false;  
    control_is_running = true;   // set BEFORE thread starts to avoid race in app_custom_stop  
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
        stow_requested   = true;  
        deploy_requested = false;  
        uint8_t tx[2]; int32_t ti = 0;  
        tx[ti++] = (uint8_t)servo_state;  
        tx[ti++] = msg;  
        commands_send_app_data(tx, ti);  
    } break;  
  
    case MSG_SET_ANGLE: {   
        if (len >= 3) { 
            float half_range_uart = mech_span / 2.0f;  
            target_angle_deg = (float)buffer_get_int16(data, &ind) * half_range_uart / 1000.0f;  
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
  
        float dt = (float)ST2MS(chVTTimeElapsedSinceX(time_last)) / 1000.0f;  
        time_last = chVTGetSystemTimeX();  
  
        // Read custom parameters from repurposed Speed PID fields  
        const volatile mc_configuration *mcconf = mc_interface_get_configuration();  
        float homing_current     = mcconf->s_pid_kp;             // Speed Kp field — SERVO_HOMING_CURRENT (A) 
        //float stall_thr          = mcconf->s_pid_ki;           // Speed Ki   → STALL_CURRENT_THR  (3.0 A)  //not in use currently swithched to rpm
        float stall_rpm_thr      = mcconf->s_pid_ki;           	 // Speed Ki   → STALL_RPM_THR (ERPM)  
		float active_deadband_deg = mcconf->s_pid_kd;            // Speed Kd   → ACTIVE_DEADBAND_DEG (deg)
		// float homing_backoff_deg = mcconf->s_pid_kd;             // Speed Kd   → HOMING_BACKOFF_DEG (5.0)  
        float storage_offset_deg = mcconf->s_pid_min_erpm;       // Min ERPM   → STORAGE_OFFSET_DEG (0.0)  
        float storage_tol_deg    = mcconf->s_pid_ramp_erpms_s;   // Ramp       → STORAGE_TOL_DEG    (2.0)  
        float range_limit           = mech_span;                     // [SPAN] auto-computed from homing  
        float homing_max_travel_deg = mcconf->p_pid_ang_div;         // Ang Div → HOMING_MAX_TRAVEL_DEG (300.0)  
        float homing_timeout_s      = mcconf->p_pid_gain_dec_angle;  // Gain Dec → HOMING_TIMEOUT_S   (30.0)  
        // rpm_now/mc_interface_get_rpm() is output-shaft-referenced (post-gearbox), while  
        // l_max_erpm is motor-shaft (pre-gearbox/electrical). Must divide by gearbox ratio  
        // to compare homing_max_rpm against get_rpm() in matching units.  
        #define KENAI_GEARBOX_RATIO 154.4f  
        float homing_max_rpm        = mcconf->s_pid_kd_filter * (mcconf->l_max_erpm / KENAI_GEARBOX_RATIO); // fraction of OUTPUT-shaft max RPM (e.g. 0.2 = 20% of ~39 output RPM)
  
        // Recalculate storage_angle every loop (parameters may change via VESC Tool)    
        // Direction-aware: positive REL = toward stop1 = positive current direction.    
        // encoder_inverted=true: positive current decreases ENC, so subtract in ENC space.    
        if (homing_completed) {    
            if (!encoder_inverted) {    
                storage_angle = center_angle + storage_offset_deg;    
            } else {    
                storage_angle = center_angle - storage_offset_deg;    
            }    
            NORM_ANGLE(storage_angle);  
				if (storage_angle >= 360.0f) storage_angle = 0.0f;  // guard exact-360 edge case  
        }  
  
        // --------------------------------------------------------  
        // FAILSAFE check  
        // Only fires in ACTIVE state; IDLE/STOWED/HOMING call timeout_reset() themselves  
        // --------------------------------------------------------  
        float cmd_age_s = (float)ST2MS(chVTTimeElapsedSinceX(last_cmd_time)) / 1000.0f;  
        bool no_signal  = cmd_received_ever && (cmd_age_s > 2.0f);  
  
                if (no_signal && servo_state == SERVO_STATE_ACTIVE) {  
                        servo_state       = SERVO_STATE_FAILSAFE;    
            in_deadband       = false;    
            active_i_term     = 0.0f;    
            active_prev_error = 0.0f;    
            active_d_filter   = 0.0f;  
            active_prev_pos   = 0.0f;    
            mc_interface_set_current(0.0f); 
        }  
  
        // --------------------------------------------------------  
        // State machine  
        // --------------------------------------------------------  
        switch (servo_state) {  
  
        // ---- IDLE -----------------------------------------------  
        case SERVO_STATE_IDLE:  
            timeout_reset();  
            mc_interface_set_current(0.0f);  
            if (deploy_requested) {    
                deploy_requested  = false;    
                stow_requested    = false;  // clear stale stow — prevents immediate stow after homing  
                homing_phase      = HOMING_PHASE_FIND_STOP1;      
                homing_timer      = 0.0f;      
                stall_timer       = 0.0f;      
                homing_completed  = false;      
                homing_braking    = false;    
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
                    mc_interface_set_current(0.0f);  
                    servo_state = SERVO_STATE_IDLE;  
                } else {  
                    commands_printf("Kenai: HOMING TIMEOUT — returning to start");    
                    mc_interface_set_current(0.0f);    
                    homing_abort_phase  = homing_phase;  
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
                    homing_abort_phase  = homing_phase;  
                    homing_phase        = HOMING_PHASE_RETURN_TO_START;  
                    homing_traveled_deg = 0.0f;  
                    homing_timer        = 0.0f;  
                    break;  
                }  
    
                switch (homing_phase) {  
  
                case HOMING_PHASE_FIND_STOP1:      
                   // mc_interface_set_pid_speed(homing_erpm);  //not in use curently switch for current    
                    // High inertia + low friction means reducing drive current does not decelerate  
                    // the motor — it coasts. Actively brake once above homing_max_rpm, resume  
                    // driving once below the hysteresis band (50% of homing_max_rpm).  
                    {    
                        float _rpm1 = mc_interface_get_rpm(); // signed; positive expected in this phase  
                        if (homing_max_rpm > 0.0f) {  
                            if (!homing_braking && _rpm1 > homing_max_rpm) {  
                                homing_braking = true;  
                            } else if (homing_braking && _rpm1 < homing_max_rpm * 0.5f) {  
                                homing_braking = false;  
                            }  
                        } else {  
                            homing_braking = false;  
                        }  
                        if (homing_braking) {  
                            mc_interface_set_brake_current(homing_current);  
                        } else {  
                            mc_interface_set_current(homing_current);  
                        }  
                    }    
					timeout_reset();
  
                  //  if (current > stall_thr) { stall_timer += dt; }  //not in use curently switch for rpm  
                  // Suspend the stall timer while actively braking — gearbox backlash can make the  
                  // shaft momentarily stop or reverse as the brake pulse takes up the free play,  
                  // which would otherwise look like a real hard-stop stall.  
				  if (!homing_braking && homing_timer > 0.5f && fabsf(mc_interface_get_rpm()) < stall_rpm_thr) { stall_timer += dt; }   
                    else                      { stall_timer  = 0.0f; }
                     if (stall_timer > 0.3f) {      
                        stop1_angle = pos_now;      
                        stall_timer = 0.0f;      
                        // Direction detection from FIND_STOP1 movement.    
                        // Only reliable if motor moved >=5 deg before stalling.    
                        // If motor started at stop1, enc_dir_detected stays false    
                        // and detection is deferred to end of FIND_STOP2.    
                        {    
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
                        homing_braking        = false;  // reset speed-limiter state for phase 2    
                        homing_phase          = HOMING_PHASE_FIND_STOP2;
                    } 
                    break;  
  
                case HOMING_PHASE_FIND_STOP2:      
                    // mc_interface_set_pid_speed(-homing_erpm);  //not in use curently switch for current    
                    // Mirror of FIND_STOP1 bang-bang brake logic for the negative direction.  
                    {    
                        float _rpm2 = mc_interface_get_rpm(); // signed; negative expected in this phase  
                        if (homing_max_rpm > 0.0f) {  
                            if (!homing_braking && _rpm2 < -homing_max_rpm) {  
                                homing_braking = true;  
                            } else if (homing_braking && _rpm2 > -homing_max_rpm * 0.5f) {  
                                homing_braking = false;  
                            }  
                        } else {  
                            homing_braking = false;  
                        }  
                        if (homing_braking) {  
                            mc_interface_set_brake_current(homing_current);  
                        } else {  
                            mc_interface_set_current(-homing_current);  
                        }  
                    }    
					timeout_reset();
  
                   // if (current > stall_thr) { stall_timer += dt; }  //not in use curently switch for rpm  
                   // Suspend the stall timer while actively braking — same backlash reasoning as FIND_STOP1.  
                   if (!homing_braking && homing_timer > 0.5f && fabsf(mc_interface_get_rpm()) < stall_rpm_thr) { stall_timer += dt; }   
				   else                      { stall_timer  = 0.0f; } 
  
                     if (stall_timer > 0.3f) {    
                        stop2_angle = pos_now;    
                        stall_timer = 0.0f;    
    
                        // Fallback direction detection from FIND_STOP2 movement.    
                        // Negative current drove motor from stop1 to stop2.    
                        // utils_angle_difference(stop2, stop1) > 0: negative current increased ENC = inverted.    
                        // utils_angle_difference(stop2, stop1) < 0: negative current decreased ENC = normal.    
                        // This is always reliable — motor always travels the full mechanical range.    
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
                        if (!encoder_inverted) {    
                            span         = stop1_angle - stop2_angle;    
                            if (span < 0.0f) span += 360.0f;    
                            center_angle = stop2_angle + span / 2.0f; // [ENC]    
                        } else {    
                            span         = stop2_angle - stop1_angle;    
                            if (span < 0.0f) span += 360.0f;    
                            center_angle = stop1_angle + span / 2.0f; // [ENC]    
                        }  
                        mech_span = span;  // persist for ACTIVE clamping and UART scaling  
                        NORM_ANGLE(center_angle); // keep in [0°, 360°)  
                        if (center_angle >= 360.0f) center_angle = 0.0f;  // guard exact-360 edge case  
                        // storage_offset_deg is motor-space REL; convert to ENC-space    
                        if (!encoder_inverted) {    
                            storage_angle = center_angle + storage_offset_deg;    
                        } else {    
                            storage_angle = center_angle - storage_offset_deg;    
                        }    
                        NORM_ANGLE(storage_angle);                   // [ENC] stow position    
                        if (storage_angle >= 360.0f) storage_angle = 0.0f;  // guard exact-360 edge case  
                        homing_completed  = true;  
  
                        commands_printf("Kenai: Stop2=%.1f  Center=%.1f  Storage=%.1f",  
                                (double)stop2_angle, (double)center_angle, (double)storage_angle);  
  
                        // Defer center save — motor is against hard stop, do not release it now  
                        pending_save_center = true;  
  
                        homing_timer = 0.0f;  
						homing_traveled_deg = 0.0f;   // ADD THIS — reset for DRIVE_TO_CENTER phase  
                        homing_phase = HOMING_PHASE_DRIVE_TO_CENTER;  
                    }  
                    break;  
  
                case HOMING_PHASE_DRIVE_TO_CENTER:  
                    mc_interface_set_pid_pos(center_angle);  
                    timeout_reset();  
  
						if (fabsf(utils_angle_difference(pos_now, center_angle)) < storage_tol_deg) { 
                        commands_printf("Kenai: Homing complete — ACTIVE");  
                        
						// Motor is at center — safe to write flash now  
                        if (pending_save_center) {    
                            pending_save_center = false;    
                            eeprom_var vc;    
                            vc.as_float = center_angle;    
                            conf_general_store_eeprom_var_custom(&vc, EEPROM_ADDR_CENTER);    
                            eeprom_var vd;    
                            vd.as_u32 = (uint32_t)encoder_inverted;    
                            conf_general_store_eeprom_var_custom(&vd, EEPROM_ADDR_ENCODER_DIR);    
                        }  
                        eeprom_var vs;  
                        vs.as_u32 = (uint32_t)SERVO_STATE_IDLE;  
                        conf_general_store_eeprom_var_custom(&vs, EEPROM_ADDR_STATE);  
                                                in_deadband       = false;    
                        active_i_term     = 0.0f;    
                        active_prev_error = 0.0f;    
                        active_d_filter   = 0.0f;  
                        active_prev_pos   = 0.0f;    
                        time_last         = chVTGetSystemTimeX(); // reset dt after long EEPROM writes    
                        servo_state       = SERVO_STATE_ACTIVE;  
						}    
                    break;  
  
                case HOMING_PHASE_RETURN_TO_START: {  
                    timeout_reset();  
                    float dist_to_start = fabsf(utils_angle_difference(pos_now, pos_at_homing_start));  
                    if (dist_to_start < storage_tol_deg) {  
                        commands_printf("Kenai: Returned to start — IDLE");  
                        mc_interface_set_current(0.0f);  
                        deploy_requested = false;  
                        servo_state = SERVO_STATE_IDLE;
                    } else {  
                        // Drive opposite to the phase that caused the abort.  
                        // FIND_STOP1 drove positive current → return drives negative.  
                        // FIND_STOP2 drove negative current → return drives positive.  
                        float return_dir = (homing_abort_phase == HOMING_PHASE_FIND_STOP1) ? -1.0f : 1.0f;  
                        mc_interface_set_current(return_dir * homing_current);  
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
                active_prev_pos   = 0.0f;  
                servo_state       = SERVO_STATE_STOWED; 
                break;  
            }  
  
            {  
                // Clamp command in motor-space REL (positive = toward stop1 = positive current direction)  
                float half_range = range_limit / 2.0f;                      // [SPAN/2] max REL excursion  
                float clamped_target = target_angle_deg;                     // [REL] motor-space  
                utils_truncate_number(&clamped_target, -half_range, half_range);  
  
                float pos_now_act = mc_interface_get_pid_pos_now();          // [ENC] current position  
  
                // Convert ENC position to motor-space REL.  
                // utils_angle_difference gives ENC-space REL (positive = increasing ENC).  
                // For encoder_inverted=true, positive current decreases ENC, so flip the sign  
                // to get motor-space REL (positive = toward stop1 = positive current direction).  
                float pos_now_rel = utils_angle_difference(pos_now_act, center_angle); // [REL] ENC-space  
                if (encoder_inverted) pos_now_rel = -pos_now_rel;           // flip to motor-space REL  
  
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
  
                    active_i_term += error * mcconf->p_pid_ki * dt;    
                    {    
                        float p_tmp = p_term;    
                        utils_truncate_number_abs(&p_tmp, 1.0f);    
                        utils_truncate_number_abs(&active_i_term, 1.0f - fabsf(p_tmp));    
                    }  
  
                    float d_raw = 0.0f;    
                    if (dt > 1e-6f) {    
                        float pos_change = utils_angle_difference(pos_now_rel, active_prev_pos);    
                        d_raw = -pos_change * mcconf->p_pid_kd / dt;    
                    }    
                    UTILS_LP_FAST(active_d_filter, d_raw, mcconf->p_pid_kd_filter);  
  
                    float output = p_term + active_i_term + active_d_filter;  
                    utils_truncate_number(&output, -1.0f, 1.0f);  
                    // Stop proximity guard: within 5° of either hard stop, limit to 50% current.    
                    // Protects gearbox from sustained full-torque against the mechanical stop.    
                    // Only active after homing — stop angles are 0.0f before first homing.    
                    float stop_scale = 1.0f;    
                    if (homing_completed) {    
                        float dist_s1 = fabsf(utils_angle_difference(pos_now_act, stop1_angle));    
                        float dist_s2 = fabsf(utils_angle_difference(pos_now_act, stop2_angle));    
                        if ((dist_s1 < 5.0f) || (dist_s2 < 5.0f)) stop_scale = 1.0f;    // changed to be ignored for 50% it should be stop_scale = 0.5f;
                    }    
                    mc_interface_set_current(output * mcconf->lo_current_max * stop_scale);  
                }  
  
                active_prev_error = error;    
                active_prev_pos   = pos_now_rel;    
                timeout_reset();    
            }    
            break;
  
        // ---- STOWED ---------------------------------------------  
        case SERVO_STATE_STOWED:    
            timeout_reset();    
  
            if (!homing_completed) {  
                servo_state = SERVO_STATE_IDLE;  
                mc_interface_set_current(0.0f);  
                break;  
            }  
  
            if (deploy_requested) {    
                deploy_requested = false;    
                stow_requested   = false;  // clear stale stow — prevents immediate stow after homing  
                homing_phase     = HOMING_PHASE_FIND_STOP1;      
                homing_timer     = 0.0f;      
                stall_timer      = 0.0f;      
                stowed_reached   = false;      
                homing_completed = false;      
                homing_braking   = false;  
                // Clear STOWED in EEPROM    
                eeprom_var v;    
                v.as_u32 = (uint32_t)SERVO_STATE_IDLE;    
                conf_general_store_eeprom_var_custom(&v, EEPROM_ADDR_STATE);    
                time_last   = chVTGetSystemTimeX(); // reset dt after EEPROM write    
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
                float pos_now_rel_s = utils_angle_difference(pos_now, center_angle);     // [REL] ENC-space  
                if (encoder_inverted) pos_now_rel_s = -pos_now_rel_s;                   // flip to motor-space  
                float storage_rel   = utils_angle_difference(storage_angle, center_angle); // [REL] ENC-space  
                if (encoder_inverted) storage_rel = -storage_rel;                        // flip to motor-space  
                float stow_err = storage_rel - pos_now_rel_s;  
  
                if (fabsf(stow_err) < storage_tol_deg) {  
                    stowed_reached    = true;  
                    active_i_term     = 0.0f;  
                    active_prev_error = 0.0f;  
                    active_d_filter   = 0.0f;  
                    mc_interface_set_current(0.0f);  
                    // Save STOWED to EEPROM  
                    eeprom_var v;  
                    v.as_u32 = (uint32_t)SERVO_STATE_STOWED;  
                    conf_general_store_eeprom_var_custom(&v, EEPROM_ADDR_STATE);  
                } else {  
                    // Same custom PID as ACTIVE — reuses active_* PID state variables  
                    float p_term = stow_err * mcconf->p_pid_kp;    
                    active_i_term += stow_err * mcconf->p_pid_ki * dt;    
                    {    
                        float p_tmp = p_term;    
                        utils_truncate_number_abs(&p_tmp, 1.0f);    
                        utils_truncate_number_abs(&active_i_term, 1.0f - fabsf(p_tmp));    
                    }  
                        float d_raw = 0.0f;  
                        if (dt > 1e-6f) {  
                            float pos_change_s = utils_angle_difference(pos_now_rel_s, active_prev_pos);  
                            d_raw = -pos_change_s * mcconf->p_pid_kd / dt;  
                        }  
                    UTILS_LP_FAST(active_d_filter, d_raw, mcconf->p_pid_kd_filter);  
                    float output = p_term + active_i_term + active_d_filter;  
                    utils_truncate_number(&output, -1.0f, 1.0f);  
                    mc_interface_set_current(output * mcconf->lo_current_max);  
                    active_prev_error = stow_err;  
                    active_prev_pos   = pos_now_rel_s; 
                    timeout_reset();  
                }  
            } else {  
                mc_interface_set_current(0.0f);  
            }  
            break;  
  
        // ---- FAILSAFE -------------------------------------------  
        case SERVO_STATE_FAILSAFE:  
            timeout_reset();
			mc_interface_set_current(0.0f);  
  
            // Signal restored → back to IDLE  
            if (!no_signal) {  
                commands_printf("Kenai: Signal restored — IDLE");  
                servo_state = SERVO_STATE_IDLE;  
            }  
            break;  
  
        default:  
            servo_state = SERVO_STATE_IDLE;  
            mc_interface_set_current(0.0f);  
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
    commands_printf("  storage     : %.2f deg [ENC]", (double)storage_angle);
    commands_printf("  target      : %.2f deg [REL from center]", (double)target_angle_deg);  
    commands_printf("  current     : %.2f A",   (double)mc_interface_get_tot_current_filtered());  
    commands_printf("  homing_done : %d",        homing_completed);    
    commands_printf("  stowed_done : %d",        stowed_reached);    
    commands_printf("  enc_inverted: %d",        (int)encoder_inverted);  
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
