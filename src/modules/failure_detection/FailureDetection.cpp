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

#include "FailureDetection.hpp"

FailureDetection::FailureDetection() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

FailureDetection::~FailureDetection()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool FailureDetection::init()
{
	// execute Run() on every sensor_accel publication
	if (!_sensor_combined_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// alternatively, Run on fixed interval
	// ScheduleOnInterval(5000_us); // 2000 us interval, 200 Hz rate

	return true;
}

void FailureDetection::Run()
{
	if (should_exit()) {
		// if (death_count < 10) {
		// 	death_count++;
		// 	return;
		// }
		_sensor_combined_sub.unregisterCallback();
		ScheduleClear();
		exit_and_cleanup();
		return;
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

	if (_motor_failure_emulation_sub.updated()) {
		motor_failure_emulation_s motor_failure_emulation;

		if (_motor_failure_emulation_sub.copy(&motor_failure_emulation)) {
			int failed = motor_failure_emulation.motor_to_fail;
			if (failed) {
				PX4_INFO("Failure emulation detected\n");
				// request_stop();
				// for testing failure control, we can trigger the detector here.

				// otherwise, detector runs as usual

				if (_param_ov_fail_detect.get()) {
					// detector.handle(motor_failure_emulation);
					// _failed_motor = failed;
					// publish_motor_failure_detection();
					// request_stop();
				}
			}
		}
	}

	if (_sensor_combined_sub.updated()) {
		sensor_combined_s combined;

		if (_sensor_combined_sub.copy(&combined)) {
			float signals[6] = {
				combined.gyro_rad[0],
				combined.gyro_rad[1],
				combined.gyro_rad[2],
				combined.accelerometer_m_s2[0],
				combined.accelerometer_m_s2[1],
				combined.accelerometer_m_s2[2],
			};
			// PX4_INFO("%f %f %f %f %f %f\n", (double) signals[0], (double) signals[1], (double) signals[2], (double) signals[3], (double) signals[4], (double) signals[5]);
			_failed_motor = _detector.check_failure_now(signals, combined.timestamp);
		}

		if (_thousand_samples_count == -1) {
			_thousand_samples_time = combined.timestamp;
			_thousand_samples_count = 0;
		}

		_thousand_samples_count++;
		if (_thousand_samples_count == 1000) {
			_thousand_samples_count = -1;
			PX4_INFO("1000 samples took %f s\n", (double) ((combined.timestamp - _thousand_samples_time) / 1e6f));
		}
	}

	if (_failed_motor) {
		PX4_INFO("Failure detected in motor %d\n", _failed_motor);
		motor_failure_detection_s det = {
			.timestamp = hrt_absolute_time(),
			.failed_motor = _failed_motor
		};
		_motor_failure_detection_pub.publish(det);
		request_stop();
	}


	perf_end(_loop_perf);
}

void FailureDetection::publish_motor_failure_detection()
{
	if (published_failure) {
		return;
	}
	published_failure = true;
	motor_failure_detection_s det = {
		.timestamp = hrt_absolute_time(),
		.failed_motor = _failed_motor
	};
	_motor_failure_detection_pub.publish(det);
}

int FailureDetection::task_spawn(int argc, char *argv[])
{
	FailureDetection *instance = new FailureDetection();

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

int FailureDetection::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int FailureDetection::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FailureDetection::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Monitor sensors to detect a sudden motor failure.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("failure_detection", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int failure_detection_main(int argc, char *argv[])
{
	return FailureDetection::main(argc, argv);
}
