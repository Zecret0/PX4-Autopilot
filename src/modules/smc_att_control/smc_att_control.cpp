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

#include "smc_att_control.hpp"
#include <px4_platform_common/defines.h>
#include <float.h>
#include <mathlib/mathlib.h>

using namespace matrix;
SMCAttControl::SMCAttControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

SMCAttControl::~SMCAttControl()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool SMCAttControl::init()
{
	// execute Run() on every angular_velocity publication
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// alternatively, Run on fixed interval
	// ScheduleOnInterval(10000_us); // 2000 us interval, 200 Hz rate

	return true;
}

void SMCAttControl::Run()
{

	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// reschedule backup

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	const hrt_abstime now = hrt_absolute_time();
	_control.timestamp = hrt_absolute_time();
	const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
	_last_run = now;

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}

	//对控制器，以滚转通道为例
	//输入：期望滚转角、期望滚转角速率、当前的俯仰角速率、偏航角速率、当前滚转角、当前滚转角速率
	//输出：滚转角对应的控制输入u1
	//直接以欧拉角做运算

	//处理输入量
	vehicle_attitude_s v_att;
	_vehicle_attitude_sub.update(&v_att);
	const Quatf q{v_att.q};
	const Eulerf euler_att{Dcmf(q)};
	vehicle_attitude_setpoint_s vsp_att;
	_vehicle_attitude_setpoint_sub.update(&vsp_att);
	vehicle_angular_velocity_s v_rates;
	_vehicle_angular_velocity_sub.update(&v_rates);

	quat_to_roll(q);
	//获取输入量
	_control.att[0] = euler_att.phi();	//角度
	// _control.att[0] = quat_to_roll(q);
	_control.att[1] = euler_att.theta();
	_control.att[2] = euler_att.psi();
	// _control.attd[0] = vsp_att.roll_body;	//期望角度
	// _control.attd[1] = vsp_att.pitch_body;
	// _control.attd[2] = vsp_att.yaw_body;
	_control.attd[0] = vsp_att.roll_body < M_PI_F? vsp_att.roll_body : 0.0f;
	_control.attd[1] = vsp_att.pitch_body < M_PI_F? vsp_att.pitch_body : 0.0f;
	_control.attd[2] = vsp_att.yaw_body < M_PI_F? vsp_att.yaw_body : 0.0f;
	// _control.attd[0] = 0.f;	//期望角度
	// _control.attd[1] = 0.f;
	// _control.attd[2] = 0.f;
	_control.datt[0] = v_rates.xyz[0];	//角速度
	_control.datt[1] = v_rates.xyz[1];
	_control.datt[2] = v_rates.xyz[2];
	_control.dattd[0] = 0;	_control.dattd[1] = 0;	_control.dattd[2] = 0;	//设定期望角速度 = 0
	float phi = _control.att[0];	float theta = _control.att[1];	float psi = _control.att[2];
	float dphi = _control.datt[0];	float dtheta = _control.datt[1];	float dpsi = _control.datt[2];
	float phid = _control.attd[0];	float thetad = _control.attd[1];	float psid = _control.attd[2];
	float dphid = _control.dattd[0];	float dthetad = _control.dattd[1];	float dpsid = _control.dattd[2];

	//以最基础的滑膜控制为例（SMC_ESO_Ctrl）
	//参数计算
	float a1 = (_iyy - _izz)/_ixx;	float a2 = (_izz - _ixx)/_iyy;	float a3 = (_ixx - _iyy)/_izz;
	float b1 = _d/_ixx;	float b2 = _d/_iyy;	float b3 = _d/_izz;
	//滑膜面计算
	float z1 = phid - phi;	float z2 = dphi - dphid - z1;
	float z3 = thetad - theta; float z4 = dtheta - dthetad - z3;
	float z5 = psid - psi;	float z6 = dpsi - dpsid - z5;
	_control.z[0] = z1;	_control.z[1] = z2;	_control.z[2] = z3;
	_control.z[3] = z4;	_control.z[4] = z5;	_control.z[5] = z6;
	//控制量计算
	_control.u1 = 1/b1*(-sign(z2) - z2 - a1*dtheta*dpsi + dphid - dphi);
	_control.u2 = 1/b2*(-sign(z4) - z4 - a2*dphi*dpsi + dthetad - dtheta);
	_control.u3 = 1/b3*(-sign(z6) - z6 - a3*dphi*dtheta + dpsid - dpsi);

	//发布给执行机构
	// use rates setpoint topic
	vehicle_rates_setpoint_s v_rates_sp;

	if (_v_rates_sp_sub.update(&v_rates_sp)) {
		_rates_sp(0) = v_rates_sp.roll;
		_rates_sp(1) = v_rates_sp.pitch;
		_rates_sp(2) = v_rates_sp.yaw;
		_thrust_sp = -v_rates_sp.thrust_body[2];
	}
	// publish actuator controls
	actuator_controls_s actuators{};
	actuators.control[actuator_controls_s::INDEX_ROLL] = PX4_ISFINITE(_control.u1) ? _control.u1 : 0.0f;
	actuators.control[actuator_controls_s::INDEX_PITCH] = PX4_ISFINITE(_control.u2) ? _control.u2 : 0.0f;
	actuators.control[actuator_controls_s::INDEX_YAW] = PX4_ISFINITE(_control.u3) ? _control.u3 : 0.0f;
	 actuators.control[actuator_controls_s::INDEX_THROTTLE] = PX4_ISFINITE(_thrust_sp) ? _thrust_sp : 0.0f;
	// actuators.control[actuator_controls_s::INDEX_ROLL] =  _control.u1;
	// actuators.control[actuator_controls_s::INDEX_PITCH] = _control.u2;
	// actuators.control[actuator_controls_s::INDEX_YAW] =  _control.u3;
	// actuators.control[actuator_controls_s::INDEX_THROTTLE] = _thrust_sp;

	//记录
	_control.outputs[0] = actuators.control[actuator_controls_s::INDEX_ROLL];
	_control.outputs[1] = actuators.control[actuator_controls_s::INDEX_PITCH];
	_control.outputs[2] = actuators.control[actuator_controls_s::INDEX_YAW];
	_control.outputs[3] = actuators.control[actuator_controls_s::INDEX_THROTTLE];
	actuators.control[actuator_controls_s::INDEX_LANDING_GEAR] = _landing_gear;

	actuators.timestamp = hrt_absolute_time();
	// _actuators_0_pub.publish(actuators);
	_smc_control_pub.publish(_control);

	updateActuatorControlsStatus(actuators, dt);
	perf_end(_loop_perf);
}

int SMCAttControl::task_spawn(int argc, char *argv[])
{
	SMCAttControl *instance = new SMCAttControl();

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

int SMCAttControl::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int SMCAttControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SMCAttControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("safe_detector", "safe");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int smc_att_control_main(int argc, char *argv[])
{
	return SMCAttControl::main(argc, argv);
}


/**
 * @brief sign function
 *
 *  @param number to take the sign from
 *  @return -1 if val < 0, 0 if val == 0, 1 if val > 0
 */
int SMCAttControl::sign(float val)
{
	return (0.0f < val ) - (val < 0.0f);
}

float SMCAttControl::quat_to_roll(const matrix::Quatf &q)
{
	float w = q(0);
	float x = q(1);
	float y = q(2);
	float z = q(3);

	float dcm21 = 2*(w*x + y*z);
	float dcm22 = w*w - x*x - y*y + z*z;

	float roll = atan2(dcm21,dcm22);

	return roll;
}

void SMCAttControl::updateActuatorControlsStatus(const actuator_controls_s &actuators, float dt)
{
	for (int i = 0; i < 4; i++) {
		_control_energy[i] += actuators.control[i] * actuators.control[i] * dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {

		actuator_controls_status_s status;
		status.timestamp = actuators.timestamp;

		for (int i = 0; i < 4; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		_actuator_controls_status_0_pub.publish(status);
		_energy_integration_time = 0.f;
	}
}
