/*
 Copyright 2016 Benjamin Vedder	benjamin@vedder.se

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

#include "app.h"
#include "ch.h"
#include "hal.h"
#include "hw.h"
#include "mc_interface.h"
#include "commands.h"
#include "utils.h"
#include "timeout.h"
#include <string.h>
#include <math.h>
#include <float.h>
#include "led_external.h"
#include "datatypes.h"
#include "comm_can.h"
#include "terminal.h"
#include "queue.h"
#include "queue_avg.h"
#include "quick_select.h"

// Registers
#define MPU_ADDR		0x68
//#define MPU_ADDR		0x72
#define WHO_AM_I		0x75

#define PWR_MGMT_1		0x6B	/* Default value 0x40 */
#define PWR_MGMT_2		0x6C

#define ACCEL_XOUT_H	0x3B	/* Read Only Register */
#define ACCEL_XOUT_L	0X3C	/* Read Only Register */
#define ACCEL_YOUT_H	0x3D	/* Read Only Register */
#define ACCEL_YOUT_L	0X3E	/* Read Only Register */
#define ACCEL_ZOUT_H	0x3F	/* Read Only Register */
#define ACCEL_ZOUT_L	0X40	/* Read Only Register */
#define TEMP_OUT_H		0x41	/* Read Only Register */
#define TEMP_OUT_L		0x42	/* Read Only Register */

#define ACCEL_CONFIG	0x1C
#define GYRO_CONFIG	 	0x1B
#define DLPF_CONFIG		0x1A

// Settings
#define OUTPUT_ITERATION_TIME_MS		20
#define OUTPUT_QUEUE_FACTOR				30
#define ACC_QUEUE_FACTOR				25
#define STDDEV_QUEUE_FACTOR				10
#define ACC_AVG_QUEUE_SIZE				1
#define GYRO_ITERATION_TIME_MS			2
#define LOW_PASS_SMOOTHING				350
#define STDDEV_G_TRES					0.2
#define BACK_TO_ZERO_FACTOR				1.1
#define STDDEV_COEFF					0.5
#define MAX_CAN_AGE						0.1
#define RPM_FILTER_SAMPLES				8
#define LOCAL_TIMEOUT					2000

#define RESTRICT_PITCH                	1 // restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#define RAD_TO_DEG						(180 / M_PI)
#define BALDWIN_STREET					19.3
#define RESET_ANGLE						45
#define MIN_BRAKE_ANGLE					-5
#define STAT_DRIFT_INIT_ITERATIONS		100 // to find out gyro stationary drift
#define STAT_DRIFT_MAX_STDDEV			0.1
#define COMPLEMENTARY_RATIO_GYRO		0.995 // depends on gyro sensitivity config
#define COMPLEMENTARY_RATIO_ACC			0.8

// Threads
static THD_FUNCTION(gyro_thread, arg);
static THD_WORKING_AREA(gyro_thread_wa, 1024);
static THD_FUNCTION(output_thread, arg);
static THD_WORKING_AREA(output_thread_wa, 1024);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile int gyro_error = 0;
static volatile chuk_config config;
static volatile bool output_running = false;
static volatile systime_t last_update_time;
static volatile int whoAmItook = 0;
static volatile int initPowerMgmTook = 0;
static volatile int noDataReads = 0;
static volatile i2cflags_t errors = 0;

/* IMU Data */
static volatile int16_t accX, accY, accZ;
static volatile int16_t gyroX, gyroY, gyroZ;
static volatile int16_t tempRaw;


static volatile float gyroXangle, gyroYangle; // Angle calculate using the gyro only
static volatile float compAngleX, compAngleY; // Calculated angle using a complementary filter

static volatile uint32_t timer;

static volatile float pitchOut = 0;
static volatile float rollOut = 0;

static const int outputQueueSize = GYRO_ITERATION_TIME_MS * OUTPUT_QUEUE_FACTOR;
static const int accQueueSize = GYRO_ITERATION_TIME_MS * ACC_QUEUE_FACTOR;
static const int stddevQueueSize = GYRO_ITERATION_TIME_MS * STDDEV_QUEUE_FACTOR;
static volatile Queue* outputPitchQ;
static volatile Queue* outputRollQ;

static volatile Queue* stddevPitchQ;
static volatile Queue* stddevRollQ;

static volatile Queue* accPitchQ;
static volatile Queue* accRollQ;

static volatile Queue* stddevGewichskraftQ;

static volatile QueueAvg* accXAvgQ;
static volatile QueueAvg* accYAvgQ;
static volatile QueueAvg* accZAvgQ;

static volatile float* pitchBuffer;
static volatile float* rollBuffer;

static volatile float* accPitchBuffer;
static volatile float* accRollBuffer;

static volatile int outputCount = 0;

static volatile float statDriftPitch = 0;
static volatile float statDriftRoll = 0;

static volatile int statDriftIter = 0;
static volatile float statDriftTotalTime = 0;

static volatile float pitchSmoothed = 0;
static volatile float rollSmoothed = 0;

static volatile float pitchStddev = 0;
static volatile float rollStddev = 0;

static volatile float complRatioPitch = COMPLEMENTARY_RATIO_GYRO;
static volatile float complRatioRoll = COMPLEMENTARY_RATIO_GYRO;

// Private functions
static void terminal_cmd_gyro_status(int argc, const char **argv);

void app_gyro_configure(chuk_config *conf) {
	config = *conf;

	terminal_register_command_callback("gyro_status",
			"Print the status of the gyro app", 0, terminal_cmd_gyro_status);
}

void app_gyro_start(void) {
	stop_now = false;
	hw_start_i2c();

	outputPitchQ = newQueue(outputQueueSize);
	outputRollQ = newQueue(outputQueueSize);

	accPitchQ = newQueue(accQueueSize);
	accRollQ = newQueue(accQueueSize);

	stddevPitchQ = newQueue(stddevQueueSize);
	stddevRollQ = newQueue(stddevQueueSize);

	stddevGewichskraftQ = newQueue(stddevQueueSize);

	accXAvgQ = newQueueAvg(ACC_AVG_QUEUE_SIZE);
	accYAvgQ = newQueueAvg(ACC_AVG_QUEUE_SIZE);;
	accZAvgQ = newQueueAvg(ACC_AVG_QUEUE_SIZE);;

	pitchBuffer = (float *) malloc(outputQueueSize * sizeof(float));
	rollBuffer = (float *) malloc(outputQueueSize * sizeof(float));

	accPitchBuffer = (float *) malloc(accQueueSize * sizeof(float));
	accRollBuffer = (float *) malloc(accQueueSize * sizeof(float));

	chThdCreateStatic(gyro_thread_wa, sizeof(gyro_thread_wa), NORMALPRIO, gyro_thread, NULL);
}

void app_gyro_stop(void) {
	stop_now = true;

	if (is_running) {
		hw_stop_i2c();
	}

	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

float app_gyro_get_filtered(void) {
	// other cases
	if (fabsf(pitchOut) > BALDWIN_STREET || fabsf(rollOut) > BALDWIN_STREET) {
		return 0;
	}

	// brake case
	if (pitchOut < MIN_BRAKE_ANGLE) {
		return pitchOut;
	}

	return fmaxf(pitchOut, fabsf(rollOut));;
}

void app_gyro_update_output() {
//    commands_printf("update output");
	if (!output_running) {
		last_update_time = 0;
		output_running = true;
		chThdCreateStatic(output_thread_wa, sizeof(output_thread_wa), NORMALPRIO, output_thread, NULL);
	}

	last_update_time = chVTGetSystemTime();
	timeout_reset();
}

/**
 * Converts data from 2complemented representation to signed integer
 */
int16_t complement2signed(uint8_t msb, uint8_t lsb) {
	uint16_t word = 0;
	word = (msb << 8) + lsb;
	if (msb > 0x7F) {
		return -1 * ((int16_t) ((~word) + 1));
	}
	return (int16_t) word;
}

static THD_FUNCTION(gyro_thread, arg) {
	(void) arg;

	chRegSetThreadName("Gyro i2c");
	is_running = true;

	uint8_t rxbuf[14];
	uint8_t txbuf[2];
	msg_t status = MSG_OK;
	systime_t tmo = MS2ST(5);
	i2caddr_t gyro_addr = MPU_ADDR;
	gyro_data gyro_d_tmp;

	bool is_ok = true;

	volatile bool initPowerMgm = true;

	for (;;) {
		if (stop_now) {
			is_running = false;
			gyro_error = 0;
			return;
		}

		if (initPowerMgm) {
			// accelerometer config
			txbuf[0] = ACCEL_CONFIG;
			txbuf[1] = 0x18; // 16
//			txbuf[1] = 0x00; // 0
			i2cAcquireBus(&HW_I2C_DEV);
			status = i2cMasterTransmitTimeout(&HW_I2C_DEV, gyro_addr, txbuf, 2, rxbuf, 0, tmo);
			i2cReleaseBus(&HW_I2C_DEV);
			is_ok = status == MSG_OK;

			if (!is_ok) {
				errors = i2cGetErrors(&HW_I2C_DEV);
				// print errors
				commands_printf("ACCEL_CONFIG_ERR: %d", errors);
				gyro_error = 2;
				hw_try_restore_i2c();
				initPowerMgm = true;
				chThdSleepMilliseconds(100);
				continue;
			}

			// gyroscope config
			txbuf[0] = GYRO_CONFIG;
			txbuf[1] = 0x18; // 2000 grad/s
//			txbuf[1] = 0x10; // 1000 grad/s
//			txbuf[1] = 0x00; // 250 grad/s
			i2cAcquireBus(&HW_I2C_DEV);
			status = i2cMasterTransmitTimeout(&HW_I2C_DEV, gyro_addr, txbuf, 2, rxbuf, 0, tmo);
			i2cReleaseBus(&HW_I2C_DEV);
			is_ok = status == MSG_OK;

			if (!is_ok) {
				errors = i2cGetErrors(&HW_I2C_DEV);
				// print errors
				commands_printf("GYRO_CONFIG_ERR: %d", errors);
				gyro_error = 2;
				hw_try_restore_i2c();
				initPowerMgm = true;
				chThdSleepMilliseconds(100);
				continue;
			}

			// digital low pass filter config
			txbuf[0] = DLPF_CONFIG;
//			txbuf[1] = 0x00; // 260Hz, 0ms
//			txbuf[1] = 0x05; // 10Hz, 13.8ms
			txbuf[1] = 0x06; // 5Hz, 19ms
			i2cAcquireBus(&HW_I2C_DEV);
			status = i2cMasterTransmitTimeout(&HW_I2C_DEV, gyro_addr, txbuf, 2, rxbuf, 0, tmo);
			i2cReleaseBus(&HW_I2C_DEV);
			is_ok = status == MSG_OK;

			if (!is_ok) {
				errors = i2cGetErrors(&HW_I2C_DEV);
				// print errors
				commands_printf("DLPF_CONFIG_ERR: %d", errors);
				gyro_error = 2;
				hw_try_restore_i2c();
				initPowerMgm = true;
				chThdSleepMilliseconds(100);
				continue;
			}

			// init power management
			txbuf[0] = PWR_MGMT_1;
			txbuf[1] = 0x01;
//			txbuf[1] = 0x00;
			i2cAcquireBus(&HW_I2C_DEV);
			status = i2cMasterTransmitTimeout(&HW_I2C_DEV, gyro_addr, txbuf, 2, rxbuf, 0, tmo);
			i2cReleaseBus(&HW_I2C_DEV);
			is_ok = status == MSG_OK;

			if (!is_ok) {
				errors = i2cGetErrors(&HW_I2C_DEV);
				// print errors
				commands_printf("PWR_MGMT_ERR: %d", errors);
				gyro_error = 2;
				hw_try_restore_i2c();
				initPowerMgm = true;
				chThdSleepMilliseconds(100);
				continue;
			}

			initPowerMgm = false;

			// Wait for sensor to stabilize
			chThdSleepMilliseconds(100);

	        /* Set gyro starting angle */
	        txbuf[0] = ACCEL_XOUT_H;
	        i2cAcquireBus(&HW_I2C_DEV);
	        status = i2cMasterTransmitTimeout(&HW_I2C_DEV, gyro_addr, txbuf, 1, rxbuf, 6, tmo);
	        i2cReleaseBus(&HW_I2C_DEV);
	        is_ok = status == MSG_OK;

	        if (!is_ok) {
	            errors = i2cGetErrors(&HW_I2C_DEV);
	            // print errors
	            commands_printf("INITIAL_ANGLE_ERR: %d", errors);
	            gyro_error = 2;
	            //hw_try_restore_i2c();
	            initPowerMgm = true;
	            chThdSleepMilliseconds(100);
	            continue;
	        }

	        accX = complement2signed(rxbuf[0], rxbuf[1]);
	        accY = complement2signed(rxbuf[2], rxbuf[3]);
	        accZ = complement2signed(rxbuf[4], rxbuf[5]);

	        float pitch = 0;
	        float roll = 0;

	        if (RESTRICT_PITCH) { // Eq. 25 and 26
	            roll = atan2f(accY, accZ) * RAD_TO_DEG;
	            pitch = atanf(-accX / sqrtf(accY * accY + accZ * accZ)) * RAD_TO_DEG; //atan
	        } else { // Eq. 28 and 29
	            roll = atanf(accY / sqrtf(accX * accX + accZ * accZ)) * RAD_TO_DEG; //atan
	            pitch = atan2f(-accX, accZ) * RAD_TO_DEG;
	        }

	        gyroXangle = roll;
	        gyroYangle = pitch;
	        compAngleX = roll;
	        compAngleY = pitch;

	        timer = ST2MS(chVTGetSystemTime());
		}

		// read data
		txbuf[0] = ACCEL_XOUT_H;
		i2cAcquireBus(&HW_I2C_DEV);
		status = i2cMasterTransmitTimeout(&HW_I2C_DEV, gyro_addr, txbuf, 1, rxbuf, 14, tmo);
		i2cReleaseBus(&HW_I2C_DEV);
		is_ok = status == MSG_OK;

		if (!is_ok) {
			errors = i2cGetErrors(&HW_I2C_DEV);
			// print errors
			//commands_printf("DATA_ERR: %d", errors);
			noDataReads++;
			gyro_error = 2;
			hw_try_restore_i2c();
			initPowerMgm = false;
			chThdSleepMilliseconds(10);
			continue;
		}

		static uint8_t last_buffer[14];
		int same = 1;

		for (int i = 0; i < 14; i++) {
			if (last_buffer[i] != rxbuf[i]) {
				same = 0;
			}
		}

		memcpy(last_buffer, rxbuf, 14);

		if (!same) {
			gyro_error = 0;
			accX = complement2signed(rxbuf[0], rxbuf[1]);
			accY = complement2signed(rxbuf[2], rxbuf[3]);
			accZ = complement2signed(rxbuf[4], rxbuf[5]);
			tempRaw = complement2signed(rxbuf[6], rxbuf[7]);
            gyroX = complement2signed(rxbuf[8], rxbuf[9]);
            gyroY = complement2signed(rxbuf[10], rxbuf[11]);
            gyroZ = complement2signed(rxbuf[12], rxbuf[13]);

//            enqueueAvg(accXAvgQ, accX);
//            enqueueAvg(accYAvgQ, accY);
//            enqueueAvg(accZAvgQ, accZ);
//
//            accX = getAvg(accXAvgQ);
//            accY = getAvg(accYAvgQ);
//            accZ = getAvg(accZAvgQ);


            float gyroXrate = gyroX / 16.4; // Convert to deg/s
            float gyroYrate = gyroY / 16.4; // Convert to deg/s

            float g = sqrtf(accX * accX + accY * accY + accZ * accZ) / 2048.;
            enqueue(stddevGewichskraftQ, g);

            float stddevG = FLT_MAX;
            if (size(stddevGewichskraftQ) == stddevQueueSize) {
            	stddevG = utils_stddev(stddevGewichskraftQ->items, stddevQueueSize);
            }

//            commands_printf("%.3f %.3f %.3f", g, gyroYrate, gyroXrate);

            uint32_t now = ST2MS(chVTGetSystemTime());
            float dt = (float) (now - timer) / 1000.; // Calculate delta time
            timer = now;

			if (statDriftIter == STAT_DRIFT_INIT_ITERATIONS) {
				gyroYrate += dt / -statDriftTotalTime * statDriftPitch;
				gyroXrate += dt / -statDriftTotalTime * statDriftRoll;
			}

	        float pitch = 0;
	        float roll = 0;

	        static float pitchStatic = 0;
	        static float rollStatic = 0;


			if (RESTRICT_PITCH) { // Eq. 25 and 26
				roll = atan2f(accY, accZ) * RAD_TO_DEG;
				pitch = atanf(-accX / sqrtf(accY * accY + accZ * accZ)) * RAD_TO_DEG; //atan
			} else { // Eq. 28 and 29
				roll = atanf(accY / sqrtf(accX * accX + accZ * accZ)) * RAD_TO_DEG; //atan
				pitch = atan2f(-accX, accZ) * RAD_TO_DEG;
			}


			// stddev of accelerometer data
			enqueue(stddevPitchQ, pitch);
			enqueue(stddevRollQ, roll);
			if (size(stddevPitchQ) == stddevQueueSize) {
				pitchStddev = utils_stddev(stddevPitchQ->items, stddevQueueSize);
			}
			if (size(stddevRollQ) == stddevQueueSize) {
				rollStddev = utils_stddev(stddevRollQ->items, stddevQueueSize);
			}

			// median filter for acc pitch and roll
//    		enqueue(accPitchQ, pitch);
//    		enqueue(accRollQ, roll);
//    		if (size(accPitchQ) == accQueueSize) {
//    			memcpy(accPitchBuffer, accPitchQ->items, accQueueSize * sizeof(float));
//    			pitch = sel(accPitchBuffer, 0, accQueueSize - 1, accQueueSize / 2);
//    		}
//    		if (size(accRollQ) == accQueueSize) {
//    			memcpy(accRollBuffer, accRollQ->items, accQueueSize * sizeof(float));
//    			roll = sel(accRollBuffer, 0, accQueueSize - 1, accQueueSize / 2);
//    		}


			if (stddevG < STDDEV_G_TRES) {
				pitchStatic = pitch;
				rollStatic = roll;
			} else {
				pitchStatic = 0;
				rollStatic = 0;
			}

			pitch = pitchStatic;
			roll = rollStatic;

    		// low pass filter
            pitchSmoothed += (pitch - pitchSmoothed) / LOW_PASS_SMOOTHING;
            rollSmoothed += (roll - rollSmoothed) / LOW_PASS_SMOOTHING;

            pitch = pitchSmoothed;
            roll = rollSmoothed;

            // check for NaN
			if (pitch != pitch) {
				commands_printf("pitch is NaN accX %d accY %d accZ %d", accX, accY, accZ);
				pitch = 0;
			}
			if (roll != roll) {
				commands_printf("roll is NaN accX %d accY %d accZ %d", accX, accY, accZ);
				roll = 0;
			}

    		// flip complementary filter ratio if object is stationary
    		complRatioPitch = pitchStddev < STDDEV_COEFF ? COMPLEMENTARY_RATIO_ACC : COMPLEMENTARY_RATIO_GYRO;
    		complRatioRoll = rollStddev < STDDEV_COEFF ? COMPLEMENTARY_RATIO_ACC : COMPLEMENTARY_RATIO_GYRO;

			// wait until gyro stationary drift is calculated
			if (statDriftIter < STAT_DRIFT_INIT_ITERATIONS) {
				if (size(stddevPitchQ) == stddevQueueSize && size(stddevRollQ) == stddevQueueSize
						&& pitchStddev < STAT_DRIFT_MAX_STDDEV && rollStddev < STAT_DRIFT_MAX_STDDEV) {
					statDriftIter++;
					statDriftTotalTime += dt;
					statDriftRoll += gyroXrate;
					statDriftPitch += gyroYrate;
				}
				compAngleY = 0;
				compAngleX = 0;
			} else {
				// compensate stationary drift
				compAngleY = complRatioPitch * (compAngleY + gyroYrate * dt) + (1 - complRatioPitch) * pitch;
				compAngleX = complRatioRoll * (compAngleX + gyroXrate * dt) + (1 - complRatioRoll) * roll;
			}

            float temperature = (float) tempRaw / 340.0 + 36.53;

			pitchOut = compAngleY;
			rollOut = compAngleX;


			// median filter for pitch and roll
    		enqueue(outputPitchQ, pitchOut);
    		enqueue(outputRollQ, rollOut);
    		if (size(outputPitchQ) == outputQueueSize) {
    			memcpy(pitchBuffer, outputPitchQ->items, outputQueueSize * sizeof(float));
    			pitchOut = sel(pitchBuffer, 0, outputQueueSize - 1, outputQueueSize / 2);
    		}
    		if (size(outputRollQ) == outputQueueSize) {
    			memcpy(rollBuffer, outputRollQ->items, outputQueueSize * sizeof(float));
    			rollOut = sel(rollBuffer, 0, outputQueueSize - 1, outputQueueSize / 2);
    		}


//	        commands_printf("x%d y%d z%d x%d y%d z%d t%d", accX, accY, accZ, gyroX, gyroY, gyroZ, tempRaw);
//	        commands_printf("roll %.1f gyroX %.1f compX %.1f temp %.1f", roll, gyroXangle, compAngleX, temperature);
//	        commands_printf("pitch %.1f gyroY %.1f compY %.1f temp %.1f", pitch, gyroYangle, compAngleY, temperature);

//			commands_printf("roll %.1f pitch %.1f temp %.1f", roll, pitch, temperature);
			app_gyro_update_output();
		}

		if (timeout_has_timeout()) {
			gyro_error = 1;
		}

		chThdSleepMilliseconds(GYRO_ITERATION_TIME_MS);
	}
}

static THD_FUNCTION(output_thread, arg) {
	(void) arg;

	chRegSetThreadName("Gyro output");

	for (;;) {
		chThdSleepMilliseconds(OUTPUT_ITERATION_TIME_MS);

		if (timeout_has_timeout() || gyro_error != 0) {
			continue;
		}

		// Local timeout to prevent this thread from causing problems after not
		// being used for a while.
		if (chVTTimeElapsedSinceX(last_update_time) > MS2ST(LOCAL_TIMEOUT)) {
			continue;
		}

		const volatile mc_configuration *mcconf = mc_interface_get_configuration();

		outputCount++;

		float out_val = app_gyro_get_filtered();

		// Apply deadband
		//TODO
		// Apply throttle curve
		//TODO

		float current = 0;
		bool current_mode_brake = false;

		if (out_val >= 0.0) {
			out_val = utils_map(out_val, 0.0, BALDWIN_STREET, 0.0, 1.0);
			current = out_val * mcconf->lo_current_motor_max_now;
			current_mode_brake = false;
		} else {
			out_val = utils_map(out_val, 0.0, -BALDWIN_STREET, 0.0, -1.0);
			current = fabsf(out_val * mcconf->lo_current_motor_min_now);
			current_mode_brake = true;
		}

//		if (outputCount == 10) {
			commands_printf("%.1f | %.1f | %.3f", pitchOut, rollOut, current);
			outputCount = 0;
//		}

		// Find lowest RPM
		float rpm_local = mc_interface_get_rpm();
		float rpm_lowest = rpm_local;
		if (config.multi_esc) {
			for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
				can_status_msg *msg = comm_can_get_status_msg_index(i);

				if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
					float rpm_tmp = msg->rpm;

					if (fabsf(rpm_tmp) < fabsf(rpm_lowest)) {
						rpm_lowest = rpm_tmp;
					}
				}
			}
		}

		if (current_mode_brake) {
			mc_interface_set_brake_current(current);

			// Send brake command to all ESCs seen recently on the CAN bus
			for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
				can_status_msg *msg = comm_can_get_status_msg_index(i);

				if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
					comm_can_set_current_brake(msg->id, current);
				}
			}
		} else {
			float current_out = current;

			// Traction control
			if (config.multi_esc) {
				for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
					can_status_msg *msg = comm_can_get_status_msg_index(i);

					if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
						comm_can_set_current(msg->id, current_out);
					}
				}
			}
			mc_interface_set_current(current_out);
		}
	}
}

static void terminal_cmd_gyro_status(int argc, const char **argv) {
	(void) argc;
	(void) argv;

	commands_printf("Gyro Status");
	commands_printf("Output: %s (error: %d)", output_running ? "On" : "Off", gyro_error);
	commands_printf("Who am I took: %d", whoAmItook);
	commands_printf("Init power mgm took: %d", initPowerMgmTook);
	commands_printf("No data reads: %d", noDataReads);
	commands_printf(" ");
}
