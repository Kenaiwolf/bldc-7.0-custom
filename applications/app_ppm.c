/*
	Modified by Kenai Custom FW for trolling motor steering
		
	Copyright 2016 - 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#pragma GCC optimize ("Os")

#include "app.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "servo_dec.h"
#include "mc_interface.h"
#include "timeout.h"
#include "utils_math.h"
#include "utils_sys.h"
#include "comm_can.h"
#include <math.h>

// Settings  
#define MAX_CAN_AGE						0.1  
#define MIN_PULSES_WITHOUT_POWER		50  
  
// ============================================================  Kenai 20260703
// TROLLING MOTOR SERVO — CUSTOM PARAMETERS  
// All tunable via VESC Tool using SPEED PID tab fields (labels are wrong — see table below).  
//  
// VESC Tool field          | Our parameter              | Default  
// -------------------------|----------------------------|--------  
// Speed Kp                 | HOMING_ERPM                | 200.0  
// Speed Ki                 | STALL_CURRENT_THR (A)      | 3.0  
// Speed Kd                 | HOMING_BACKOFF_DEG         | 5.0  
// Speed Kd Filter          | TRIGGER_PULSE_MS threshold | 0.9  
// Speed Min ERPM           | STORAGE_OFFSET_DEG		 | 0.0  
// Speed Ramp ERPM/s        | STORAGE_TOLERANCE_DEG      | 2.0  
// Pos Angle Division       | RANGE_LIMIT_DEG (safety)   | 270.0  
// Pos Gain Decrease Angle  | HOMING_TIMEOUT_S           | 30.0  
// ============================================================  
// To change: open VESC Tool -> Motor Config -> PID Controller tab,  
// edit the Speed PID fields. Values are saved to VESC flash.  
// ============================================================  
  
// State machine states  
typedef enum {  
	SERVO_STATE_IDLE    = 0,  // Power on, waiting for 800us->valid transition  
	SERVO_STATE_HOMING  = 1,  // Driving to hard stop for calibration  
	SERVO_STATE_ACTIVE  = 2,  // Normal PPM position control  
	SERVO_STATE_STOWED  = 3,  // Driving to storage angle, then 0A  
	SERVO_STATE_FAILSAFE = 4, // No PWM signal — 0A brake  
} servo_state_t;

// Threads
static THD_FUNCTION(ppm_thread, arg);
static THD_WORKING_AREA(ppm_thread_wa, 1024); //changed from __attribute__((section(".ram4"))) static THD_WORKING_AREA(ppm_thread_wa, 515); Kenai 20260704 
static thread_t *ppm_tp;
static volatile bool ppm_rx = false;

// Private functions
static void servodec_func(void);

// Private variables  
static volatile bool is_running = false;  
static volatile bool stop_now = true;  
  
// Trolling motor state machine  Kenai 20260703
static volatile servo_state_t servo_state = SERVO_STATE_IDLE;  
static volatile bool last_pulse_was_trigger = false; // true when last pulse was 800us  
static volatile float hard_stop_angle = 0.0;         // Absolute angle of detected hard stop  
static volatile float storage_angle = 0.0;           // Computed storage angle (hard_stop + offset)  
static volatile float homing_timer = 0.0;            // Seconds spent in HOMING state  
static volatile bool homing_completed = false;        // true after first successful hard stop detection
static volatile bool stowed_reached = false;          // true when storage angle reached — prevents oscillation
static volatile ppm_config config;
static volatile int pulses_without_power = 0;
static float input_val = 0.0;
static volatile float direction_hyst = 0;
static volatile bool ppm_detached = false;
static volatile float ppm_override = 0.0;

// Private functions

void app_ppm_configure(ppm_config *conf) {
	config = *conf;
	pulses_without_power = 0;

	if (is_running) {
		servodec_set_pulse_options(config.pulse_start, config.pulse_end, config.median_filter);
	}

	direction_hyst = config.max_erpm_for_dir * 0.20;
}

void app_ppm_start(void) {
	stop_now = false;
	chThdCreateStatic(ppm_thread_wa, sizeof(ppm_thread_wa), NORMALPRIO, ppm_thread, NULL);
}

void app_ppm_stop(void) {
	stop_now = true;

	if (is_running) {
		chEvtSignalI(ppm_tp, (eventmask_t) 1);
		servodec_stop();
	}

	while(is_running) {
		chThdSleepMilliseconds(1);
	}
}

float app_ppm_get_decoded_level(void) {
	return input_val;
}

void app_ppm_detach(bool detach) {
	ppm_detached = detach;
}

void app_ppm_override(float val) {
	ppm_override = val;
}

static void servodec_func(void) {
	ppm_rx = true;
	chSysLockFromISR();
	chEvtSignalI(ppm_tp, (eventmask_t) 1);
	chSysUnlockFromISR();
}

static THD_FUNCTION(ppm_thread, arg) {
	(void)arg;

	chRegSetThreadName("APP_PPM");
	ppm_tp = chThdGetSelfX();

	servodec_set_pulse_options(config.pulse_start, config.pulse_end, config.median_filter);
	servodec_init(servodec_func);
	is_running = true;

	for(;;) {
		chEvtWaitAnyTimeout((eventmask_t)1, MS2ST(2));

		if (stop_now) {
			is_running = false;
			return;
		}

		if (ppm_rx) {
			ppm_rx = false;
			timeout_reset();
		}

		const volatile mc_configuration *mcconf = mc_interface_get_configuration();
		const float rpm_now = mc_interface_get_rpm();
		float servo_val = servodec_get_servo(0);

		if (ppm_detached) {
			servo_val = ppm_override;
		}

		float servo_ms = utils_map(servo_val, -1.0, 1.0, config.pulse_start, config.pulse_end);

		static bool servoError = false;

		switch (config.ctrl_type) {
		case PPM_CTRL_TYPE_CURRENT_NOREV:
		case PPM_CTRL_TYPE_DUTY_NOREV:
		case PPM_CTRL_TYPE_PID_NOREV:
		case PPM_CTRL_TYPE_PID_POSITION_360:
			input_val = servo_val;
			servo_val += 1.0;
			servo_val /= 2.0;
			break;

		default:
			// Mapping with respect to center pulsewidth
			if (servo_ms < config.pulse_center) {
				servo_val = utils_map(servo_ms, config.pulse_start,
						config.pulse_center, -1.0, 0.0);
			} else {
				servo_val = utils_map(servo_ms, config.pulse_center,
						config.pulse_end, 0.0, 1.0);
			}
			input_val = servo_val;
			break;
		}
		// All pins and buttons are still decoded for debugging, even
		// when output is disabled.
		if (app_is_output_disabled()) {
			continue;
		}

		if (timeout_has_timeout() || servodec_get_time_since_update() > timeout_get_timeout_msec()) {
			pulses_without_power = 0;
			servoError = true;
			float timeoutCurrent = timeout_get_brake_current();
			mc_interface_set_brake_current(timeoutCurrent);
			if(config.multi_esc){
				for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
					can_status_msg *msg = comm_can_get_status_msg_index(i);

					if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
						comm_can_set_current_brake(msg->id, timeoutCurrent);
					}
				}
			}
			continue;
		} else if (mc_interface_get_fault() != FAULT_CODE_NONE && config.safe_start != SAFE_START_NO_FAULT){
			pulses_without_power = 0;
		}

		// Apply deadband
		utils_deadband(&servo_val, config.hyst, 1.0);

		// Apply throttle curve
		servo_val = utils_throttle_curve(servo_val, config.throttle_exp, config.throttle_exp_brake, config.throttle_exp_mode);

		// Apply ramping
		static systime_t last_time = 0;
		static float servo_val_ramp = 0.0;
		float ramp_time = fabsf(servo_val) > fabsf(servo_val_ramp) ? config.ramp_time_pos : config.ramp_time_neg;

		// TODO: Remember what this was about?
//		if (fabsf(servo_val) > 0.001) {
//			ramp_time = fminf(config.ramp_time_pos, config.ramp_time_neg);
//		}

		const float dt = (float)ST2MS(chVTTimeElapsedSinceX(last_time)) / 1000.0;
		last_time = chVTGetSystemTimeX();

		if (ramp_time > 0.01) {
			const float ramp_step = dt / ramp_time;
			utils_step_towards(&servo_val_ramp, servo_val, ramp_step);
			servo_val = servo_val_ramp;
		}

		float current = 0;
		bool current_mode = false;
		bool current_mode_brake = false;
		bool send_current = false;
		bool send_duty = false;
		static bool force_brake = true;
		static int8_t did_idle_once = 0; //0 = haven't idle ;1 = idle once ; 2 = idle twice
		float rpm_local = mc_interface_get_rpm();
		float rpm_lowest = rpm_local;
		float rpm_highest = rpm_local;

		switch (config.ctrl_type) {
		case PPM_CTRL_TYPE_CURRENT_BRAKE_REV_HYST:
			current_mode = true;

			// Hysteresis 20 % of actual RPM
			if (force_brake) {
				if (rpm_local < config.max_erpm_for_dir - direction_hyst) { // for 2500 it's 2000
					force_brake = false;
					did_idle_once = 0;
				}
			} else {
				if (rpm_local > config.max_erpm_for_dir + direction_hyst) { // for 2500 it's 3000
					force_brake = true;
					did_idle_once = 0;
				}
			}

			if (servo_val >= 0.0) {
				if (servo_val == 0.0) {
					// if there was a idle in between then allow going backwards
					if (did_idle_once == 1 && !force_brake) {
						did_idle_once = 2;
					}
				} else{
					// accelerated forward or fast enough at least
					if (rpm_local > -config.max_erpm_for_dir){ // for 2500 it's -2500
						did_idle_once = 0;
					}
				}

				if (rpm_now >= 0.0) { //Accelerate
					current = servo_val * mcconf->lo_current_max;
				} else { //Brake
					current = servo_val * fabsf(mcconf->lo_current_min);
				}

			} else {
				// too fast
				if (force_brake){
					current_mode_brake = true;
				} else {
					// not too fast backwards
					if (rpm_local > -config.max_erpm_for_dir) { // for 2500 it's -2500
						// first time that we brake and we are not too fast
						if (did_idle_once != 2) {
							did_idle_once = 1;
							current_mode_brake = true;
						}
					// too fast backwards
					} else {
						// if brake was active already
						if (did_idle_once == 1) {
							current_mode_brake = true;
						} else {
							// it's ok to go backwards now braking would be strange now
							did_idle_once = 2;
						}
					}
				}

				if (current_mode_brake) {
					// braking
					current = fabsf(servo_val * mcconf->lo_current_min);
				} else {
					// reverse acceleration
					current = servo_val * fabsf(mcconf->lo_current_min);
				}
			}

			if (fabsf(servo_val) < 0.001) {
				pulses_without_power++;
			}

			break;
		case PPM_CTRL_TYPE_CURRENT:
		case PPM_CTRL_TYPE_CURRENT_NOREV:
			current_mode = true;
			if ((servo_val >= 0.0 && rpm_now >= 0.0) || (servo_val < 0.0 && rpm_now <= 0.0)) { //Accelerate
				current = servo_val * mcconf->lo_current_max;
			} else { //Brake
				current = servo_val * fabsf(mcconf->lo_current_min);
			}

			if (fabsf(servo_val) < 0.001) {
				pulses_without_power++;
			}
			break;

		case PPM_CTRL_TYPE_CURRENT_NOREV_BRAKE:
		case PPM_CTRL_TYPE_CURRENT_SMART_REV:
			current_mode = true;
			current_mode_brake = servo_val < 0.0;

			if (servo_val >= 0.0 && rpm_now > 0.0) { //Positive input AND going forward = accelerating
				current = servo_val * mcconf->lo_current_max;
			} else { //Negative input OR going backwards = brake (no reverse allowed in those control types)
				current = fabsf(servo_val * mcconf->lo_current_min);
			}

			if (fabsf(servo_val) < 0.001) {
				pulses_without_power++;
			}
			break;

		case PPM_CTRL_TYPE_DUTY:
		case PPM_CTRL_TYPE_DUTY_NOREV:
			if (fabsf(servo_val) < 0.001) {
				pulses_without_power++;
			}

			if (!(pulses_without_power < MIN_PULSES_WITHOUT_POWER && config.safe_start)) {
				mc_interface_set_duty(utils_map(servo_val, -1.0, 1.0, -mcconf->l_max_duty, mcconf->l_max_duty));
				send_duty = true;
			}
			break;

		case PPM_CTRL_TYPE_PID:
		case PPM_CTRL_TYPE_PID_NOREV:
			if (fabsf(servo_val) < 0.001) {
				pulses_without_power++;
			}

			if (!(pulses_without_power < MIN_PULSES_WITHOUT_POWER && config.safe_start)) {
				mc_interface_set_pid_speed(servo_val * config.pid_max_erpm);
				send_current = true;
			}
			break;

		case PPM_CTRL_TYPE_PID_POSITION_180: // Used for trolling motor servo state machine  Kenai 20260703
		case PPM_CTRL_TYPE_PID_POSITION_360: // (360 mode falls through to same logic)  
		{  
			// ============================================================  
			// Read custom parameters from Speed PID tab (labels are wrong in VESC Tool)  
			// ============================================================  
			float homing_erpm        = mcconf->s_pid_kp;           // Speed Kp field  
			float stall_thr          = mcconf->s_pid_ki;           // Speed Ki field  
			float homing_backoff_deg = mcconf->s_pid_kd;           // Speed Kd field  
			float storage_offset_deg = mcconf->s_pid_min_erpm;     // Speed Min ERPM field
			float trigger_pulse_ms   = mcconf->s_pid_kd_filter;    // Speed Kd Filter field 
			float storage_tol_deg    = mcconf->s_pid_ramp_erpms_s; // Speed Ramp field  
			float homing_timeout_s   = mcconf->p_pid_gain_dec_angle; // Pos Gain Dec Angle field
			float range_limit        = mcconf->p_pid_ang_div;        // Pos Angle Division field — total servo range in degrees
  
			// ============================================================  
			// Detect signal type from raw pulse length  
			// servodec_get_last_pulse_len() always returns the last received  
			// pulse length in ms, even if it was rejected as out-of-range.  
			// ============================================================  
			float raw_pulse_ms = servodec_get_last_pulse_len(0);  
			bool no_signal     = (servodec_get_time_since_update() > 2000); // True timeout  
			bool trigger_pulse = (!no_signal) && (raw_pulse_ms < trigger_pulse_ms); // 800us  
			bool valid_pulse   = (!no_signal) && (!trigger_pulse);  
  
			// ============================================================  
			// FAILSAFE: overrides all states — no PWM cable disconnected  
			// ============================================================  
			if (no_signal) {  
				servo_state = SERVO_STATE_FAILSAFE;  
				mc_interface_set_brake_current(timeout_get_brake_current());  
				last_pulse_was_trigger = false;  
				break;  
			}  
  
			// ============================================================  
			// State machine transitions  
			// ============================================================  
			switch (servo_state) {  
  
			case SERVO_STATE_IDLE:  
				// Stay in IDLE with 0A until we see 800us followed by valid PWM.  
				// This handles both fresh power-on and VESC power-cycle while  
				// Pixhawk is already running (Pixhawk boots with ~1500us, then  
				// immediately sends 800us, then valid PWM — the 800us->valid  
				// transition is the calibration trigger).  
				mc_interface_set_current(0.0);  
				if (trigger_pulse) {  
					last_pulse_was_trigger = true;  
				} else if (valid_pulse && last_pulse_was_trigger) {  
					// Transition: 800us was seen, now valid PWM arrived -> start homing  
					servo_state = SERVO_STATE_HOMING;  
					homing_timer = 0.0;  
					// Drive toward hard stop at slow speed (negative = toward stop 1)  
					// homing_erpm is stored in Speed Kp field (label wrong in VESC Tool)  
					mc_interface_set_pid_speed(-homing_erpm);  
				}  
				break;  
  
			case SERVO_STATE_HOMING:  
				// Drive slowly toward hard stop. Detect stall by current threshold.  
				// stall_thr is stored in Speed Ki field (label wrong in VESC Tool).  
				homing_timer += dt; // dt is loop period in seconds (approx 0.001s at 1kHz)  
				if (trigger_pulse) {   
					// 800us received -> go to STOWED
					stowed_reached = false;    
					last_pulse_was_trigger = false;    
					servo_state = SERVO_STATE_STOWED;    
					break;    
				}    
				if (homing_timer > homing_timeout_s) {  
					// Homing took too long — encoder or mechanical problem  
					// Go back to IDLE and wait for next trigger  
					servo_state = SERVO_STATE_IDLE;  
					mc_interface_set_current(0.0);  
					last_pulse_was_trigger = false;  
					break;  
				}  
				if (fabsf(mc_interface_get_tot_current_directional_filtered()) > stall_thr) {  
					// Stall detected — we are at the hard stop  
					hard_stop_angle = mc_interface_get_pid_pos_now();    
					homing_completed = true;  
					// Back off slightly to release mechanical pressure  
					// homing_backoff_deg stored in Speed Kd field  
					float backoff_target = hard_stop_angle + homing_backoff_deg;  
					mc_interface_set_pid_pos(backoff_target);  
					// Set position offset so hard stop = 0 in our signed coordinate system  
					// storage_offset_deg stored in Speed Kd Filter field  
					// storage_angle is the absolute angle to go to when stowing  
					storage_angle = hard_stop_angle + (range_limit / 2.0) + storage_offset_deg;  
					servo_state = SERVO_STATE_ACTIVE;  
				} else {  
					mc_interface_set_pid_speed(-homing_erpm);  
				}  
				break;  
  
			case SERVO_STATE_ACTIVE:  
				// Normal PPM position control.  
				// servo_val is -1 to +1 (180 mode: center stick = 0).  
				// Map to absolute angle: center stick = hard_stop + range/2.  
				// Bug 2 fix is in run_pid_control_pos (simple subtraction).   
				if (trigger_pulse) {   
					// 800us during homing = abort, go to STOWED
					stowed_reached = false;    
					last_pulse_was_trigger = false;    
					servo_state = SERVO_STATE_STOWED;    
					break;    
				}    
				{    
					// Map servo_val (-1..+1) to angle relative to hard stop.  
					// Full left = hard_stop_angle, full right = hard_stop_angle + range.  
					// range is detected during homing (not hardcoded).  
					// For now use p_pid_ang_div as range limit safety cap.  
					float half_range = range_limit / 2.0;  
					// Center of range = hard_stop + half_range  
					float center_angle = hard_stop_angle + half_range;  
					// Map: servo_val=0 -> center, servo_val=-1 -> hard_stop, servo_val=+1 -> hard_stop+range  
					float angle = center_angle + servo_val * half_range;  
					// Clamp to valid range (prevents commanding past hard stops)  
					utils_truncate_number(&angle, hard_stop_angle, hard_stop_angle + range_limit);  
					mc_interface_set_pid_pos(angle);  
				}  
				break;  
  
			case SERVO_STATE_STOWED:  
				// Drive to storage angle, then release (0A).  
				// storage_tol_deg stored in Speed Ramp field.  
				// Track trigger pulse so next valid PWM can trigger homing.  
				if (trigger_pulse) {  
					last_pulse_was_trigger = true;  
				}  
				if (valid_pulse && last_pulse_was_trigger) {  
					// 800us -> valid PWM sequence detected — start homing  
					servo_state = SERVO_STATE_HOMING;  
					homing_timer = 0.0;  
					last_pulse_was_trigger = false;  
					stowed_reached = false;  
					break;  
				}  
				if (!homing_completed) {    
					// Homing never finished — storage_angle unknown, just release motor    
					mc_interface_set_current(0.0);    
				} else if (!stowed_reached) {    
					float pos_now = mc_interface_get_pid_pos_now();    
					if (fabsf(pos_now - storage_angle) < storage_tol_deg) {    
						// Reached storage position — release motor (no holding torque)    
						stowed_reached = true;    
						mc_interface_set_current(0.0);    
					} else {    
						mc_interface_set_pid_pos(storage_angle);    
					}    
				} else {    
					// Already at storage — keep motor released    
					mc_interface_set_current(0.0);    
				}  
			break;
  
			case SERVO_STATE_FAILSAFE:  
				// Signal returned — go back to IDLE and wait for 800us trigger  
				servo_state = SERVO_STATE_IDLE;  
				mc_interface_set_current(0.0);  
				last_pulse_was_trigger = false;  
				break;  
  
			default:  
				servo_state = SERVO_STATE_IDLE;  
				mc_interface_set_current(0.0);  
				break;  
			}  
		}  
		break;

		default:
			continue;
		}
		//Safe start : If startup, servo timeout or fault, check if idle has been verified for some pulses before driving the motor
		if (pulses_without_power < MIN_PULSES_WITHOUT_POWER && config.safe_start) {
			static int pulses_without_power_before = 0;
			if (pulses_without_power == pulses_without_power_before) {
				pulses_without_power = 0;
			}
			pulses_without_power_before = pulses_without_power;

			if (servoError){
				continue;
			}
			if (current_mode) {
				current = 0.0;
			}
		} else {
			servoError = false;
		}

		const float duty_now = mc_interface_get_duty_cycle_now();
		float current_highest_abs = fabsf(mc_interface_get_tot_current_directional_filtered());
		float duty_highest_abs = fabsf(duty_now);

		//If multiple VESCs over CAN, store highest/lowest running values of the whole setup
		if (config.multi_esc) {
			for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
				can_status_msg *msg = comm_can_get_status_msg_index(i);

				if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
					if (fabsf(msg->rpm) < fabsf(rpm_lowest)) {
						rpm_lowest = msg->rpm;
					}

					if (fabsf(msg->rpm) > fabsf(rpm_highest)) {
						rpm_highest = msg->rpm;
					}

					if (fabsf(msg->current) > current_highest_abs) {
						current_highest_abs = fabsf(msg->current);
					}

					if (fabsf(msg->duty) > duty_highest_abs) {
						duty_highest_abs = fabsf(msg->duty);
					}
				}
			}
		}

		if (config.ctrl_type == PPM_CTRL_TYPE_CURRENT_SMART_REV) {
			bool duty_control = false;
			static bool was_duty_control = false;
			static float duty_rev = 0.0;

			if (servo_val < -0.92 && duty_highest_abs < (mcconf->l_min_duty * 1.5) &&
					current_highest_abs < (mcconf->l_current_max * mcconf->l_current_max_scale * 0.7)) {
				duty_control = true;
			}

			if (duty_control || (was_duty_control && servo_val < -0.1)) {
				was_duty_control = true;

				float goal = config.smart_rev_max_duty * -servo_val;
				utils_step_towards(&duty_rev, -goal,
						config.smart_rev_max_duty * dt / config.smart_rev_ramp_time);

				mc_interface_set_duty(duty_rev);

				// Send the same duty cycle to the other controllers
				if (config.multi_esc) {
					for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
						can_status_msg *msg = comm_can_get_status_msg_index(i);

						if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
							comm_can_set_duty(msg->id, duty_rev);
						}
					}
				}

				current_mode = false;
			} else {
				duty_rev = duty_now;
				was_duty_control = false;
			}
		}
		//CTRL TYPE PID_NOREV & DUTY_NOREV : Acting as master, send motor control command to all slave VESCs detected on the CANbus
		if ((send_current || send_duty) && config.multi_esc) {
			float current_filtered = mc_interface_get_tot_current_directional_filtered();
			float duty = mc_interface_get_duty_cycle_now();

			for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
				can_status_msg *msg = comm_can_get_status_msg_index(i);

				if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
					if (send_current) {
						comm_can_set_current(msg->id, current_filtered);
					} else if (send_duty) {
						comm_can_set_duty(msg->id, duty);
					}
				}
			}
		}
//CTRL TYPE CURRENT
		if (current_mode) {
			if (current_mode_brake) { //If braking applied
				mc_interface_set_brake_current(fabsf(current));

				// Send brake command to all ESCs seen recently on the CAN bus
				if (config.multi_esc) {
					for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
						can_status_msg *msg = comm_can_get_status_msg_index(i);

						if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
							comm_can_set_current_brake_rel(msg->id, fabsf(servo_val));
						}
					}
				}
			} else {
				float current_out = current;
				bool is_reverse = false;
				static bool autoTCdisengaged = false;
				if (current_out < 0.0) { // Not braking AND negative current = reverse engaged
					is_reverse = true;
					current_out = -current_out;
					current = -current;
					rpm_local = -rpm_local;
					rpm_lowest = -rpm_lowest;
					rpm_highest = -rpm_highest;
					servo_val = -servo_val;
				}

				// Send acceleration command to all ESCs seen recently on the CAN bus
				if (config.multi_esc) {
					if (config.tc) {
						if(mc_interface_get_fault() != FAULT_CODE_NONE) {
							autoTCdisengaged = true;
						} else if (autoTCdisengaged && rpm_highest < rpm_local + config.tc_max_diff && rpm_lowest > rpm_local - config.tc_max_diff) { //No Fault anymore and no traction control action needed = re-enable traction control
							autoTCdisengaged = false;
						}
					}
					for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
						can_status_msg *msg = comm_can_get_status_msg_index(i);

						if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
							//Traction Control - Applied to slaves except if a fault has occured on the local VESC (undriven wheel may generate fake RPM)
							if (config.tc && config.tc_max_diff > 1.0 && !autoTCdisengaged) {
								float rpm_tmp = msg->rpm;
								if (is_reverse) {
									rpm_tmp = -rpm_tmp;
								}

								float diff = rpm_tmp - rpm_lowest;
								servo_val = utils_map(diff, 0.0, config.tc_max_diff, servo_val, 0.0);
							}
							//Send motor drive command to slaves
							if (is_reverse) {
								comm_can_set_current_rel(msg->id, -servo_val);
							} else {
								comm_can_set_current_rel(msg->id, servo_val);
							}
						}
					}
					//Traction Control - Applying locally
					if (config.tc && config.tc_max_diff > 1.0) {
						float diff = rpm_local - rpm_lowest;
						current_out = utils_map(diff, 0.0, config.tc_max_diff, current, 0.0);
						if (current_out < mcconf->cc_min_current) {
							current_out = 0.0;
						}
					}
				}
				//Drive local motor
				if (is_reverse) {
					mc_interface_set_current(-current_out);
				} else {
					mc_interface_set_current(current_out);
				}
			}
		}

	}
}

