/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "FailureControl.hpp"
#include <matrix/math.hpp>
#include <cmath>

FailureControl::FailureControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

FailureControl::~FailureControl()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool FailureControl::init()
{
	// execute Run() on every sensor_accel publication
	if (!_vehicle_odometry_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// alternatively, Run on fixed interval
	// ScheduleOnInterval(5000_us); // 2000 us interval, 200 Hz rate

	return true;
}

void quaternionToEuler(const matrix::Quatf &q, float &roll, float &pitch)
{
    // Ensure the quaternion is normalized
    matrix::Quatf normalized_q = q.unit();

    // Extract components
    float q0 = normalized_q(0);
    float q1 = normalized_q(1);
    float q2 = normalized_q(2);
    float q3 = normalized_q(3);

    // Compute pitch
    pitch = asinf(-2.0f * (q1 * q3 - q0 * q2));

    // Compute roll
    roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
}

void FailureControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (_motor_failure_detection_sub.updated()) {
		motor_failure_detection_s motor_failure_detection;

		if (_motor_failure_detection_sub.copy(&motor_failure_detection)) {
			_failed_motor = motor_failure_detection.failed_motor;
			PX4_INFO("Failure control enabled for motor %d\n", _failed_motor);
		}
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}


	// Example
	//  update vehicle_status to check arming state
	if (_vehicle_odometry_sub.updated()) {
		vehicle_odometry_s vehicle_odometry;

		if (_vehicle_odometry_sub.copy(&vehicle_odometry)) {
			if (_failed_motor == 0) {
				perf_end(_loop_perf);
				return;
			}
			float roll = 0.0;
			float pitch = 0.0;
			matrix::Quatf q(vehicle_odometry.q);
			quaternionToEuler(q, roll, pitch);

			if (_failed_motor == 1) {
				roll = roll;
				pitch = pitch;
			} else if (_failed_motor == 2) {
				roll = -roll;
				pitch = -pitch;
			} else if (_failed_motor == 3) {
				roll = pitch;
				pitch = -roll;
			} else {
				// 4
				roll = -pitch;
				pitch = roll;
			}

			float roll1 = (roll + pitch) / sqrtf(2);
			float pitch1 = (roll - pitch) / sqrtf(2);

			float r = _pid1.getOutput(roll1, 0.0);
			float p = _pid2.getOutput(pitch1, 0.0);

			float vz = vehicle_odometry.velocity[2];
			vz = _pid3.getOutput(vz, 0.0);

			actuator_motors_s actuator_motors;
			actuator_motors.timestamp = hrt_absolute_time();
			actuator_motors.timestamp_sample = actuator_motors.timestamp;
			float control[12];
			for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) {
				control[i] = NAN;
			}
			control[0] = 0.0;
			control[3 - 1] = vz + r;
			control[2 - 1] = vz + p;
			control[4 - 1] = vz - r;
			for (int i = 4; i < actuator_motors_s::NUM_CONTROLS; i++) {
				actuator_motors.control[i] = control[i];
			}
			if (_failed_motor == 1) {
				actuator_motors.control[0] = control[0];
				actuator_motors.control[1] = control[1];
				actuator_motors.control[2] = control[2];
				actuator_motors.control[3] = control[3];
			} else if (_failed_motor == 2) {
				actuator_motors.control[0] = control[1];
				actuator_motors.control[1] = control[0];
				actuator_motors.control[2] = control[3];
				actuator_motors.control[3] = control[2];
			} else if (_failed_motor == 3) {
				actuator_motors.control[0] = control[2];
				actuator_motors.control[2] = control[1];
				actuator_motors.control[1] = control[3];
				actuator_motors.control[3] = control[2];
			}

			_actuator_motors_pub.publish(actuator_motors);
		}
	}

	perf_end(_loop_perf);
}

int FailureControl::task_spawn(int argc, char *argv[])
{
	FailureControl *instance = new FailureControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int FailureControl::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int FailureControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FailureControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Controlled failure in case of a single motor failure.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("failure_control", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int failure_control_main(int argc, char *argv[])
{
	return FailureControl::main(argc, argv);
}
