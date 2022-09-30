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

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <stdlib.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/orb_test.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_status.h>

#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/distance_sensor.h>

#include <matrix/matrix/math.hpp>
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

#include <uORB/topics/smc_control.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_status.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/landing_gear.h>

using namespace time_literals;

class SMCAttControl : public ModuleBase<SMCAttControl>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	SMCAttControl();
	~SMCAttControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	// Publications
	uORB::Publication<orb_test_s> _orb_test_pub{ORB_ID(orb_test)};
	uORB::Publication<smc_control_s> _smc_control_pub{ORB_ID(smc_control)};
	uORB::Publication<actuator_controls_s>		_actuators_0_pub{ORB_ID(actuator_controls)};
	uORB::Publication<actuator_controls_status_s>	_actuator_controls_status_0_pub{ORB_ID(actuator_controls_status_0)};

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)};        // subscription that schedules WorkItemExample when updated
	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates
	uORB::Subscription                 _vehicle_status_sub{ORB_ID(vehicle_status)};          // regular subscription for additional data

	uORB::Subscription	_smc_control_sub{ORB_ID(smc_control)};
	//获取当前角度（按四元数记录，需要转换
	uORB::Subscription	_vehicle_attitude_sub{ORB_ID(vehicle_attitude)};	//暂时以固定频率运行作为测试，因此不用SubscriptionCallbackWorkItem
	//获取当前角速度 (参照mc_att_control)
	// uORB::Subscription	_vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};	//需要用callback
	//获取期望角度（存储的时候有做了转换成欧拉角
	uORB::Subscription	_vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	//期望油门推力
	uORB::Subscription _v_rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _landing_gear_sub{ORB_ID(landing_gear)};

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
	)

	//控制需要的一些参数，后期可以改成param
	const float _ixx = 0.03, _iyy = 0.03, _izz = 0.06;	//参考仿真无人机的参数
	const float _mass = 1.5;
	const float _d = 0.66;	//电机离质心的距离
	// uORB::SubscriptionMultiArray<distance_sensor_s> _distance_sensor_subs{ORB_ID::distance_sensor};
	// uORB::Subscription  _distance_sensor_subs{ORB_ID(distance_sensor), 1};
	uORB::Subscription  _distance_sensor_subs{ORB_ID(distance_sensor)};

	int sign(float val);
	float quat_to_roll(const matrix::Quatf &q);
	void updateActuatorControlsStatus(const actuator_controls_s &actuators, float dt);

	matrix::Vector3f _rates_sp;			/**< angular rates setpoint */
	float _energy_integration_time{0.0f};
	float _control_energy[4] {};
	hrt_abstime _last_run{0};
	int8_t _landing_gear{landing_gear_s::GEAR_DOWN};

	float _down{0.0f};
	float		_thrust_sp{0.0f};		/**< thrust setpoint */

	bool _armed{false};

	smc_control_s _control{};

};
