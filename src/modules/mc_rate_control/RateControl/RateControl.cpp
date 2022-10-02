/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

/**
 * @file RateControl.cpp
 */

#include <RateControl.hpp>
#include <px4_platform_common/defines.h>

using namespace matrix;

void RateControl::setGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_p = P;
	_gain_i = I;
	_gain_d = D;
}

void RateControl::setSaturationStatus(const Vector<bool, 3> &saturation_positive,
				      const Vector<bool, 3> &saturation_negative)
{
	_control_allocator_saturation_positive = saturation_positive;
	_control_allocator_saturation_negative = saturation_negative;
}

Vector3f RateControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const float dt, const bool landed)
{
	// angular rates error
	Vector3f rate_error = rate_sp - rate;

	// PID control with feed forward
	const Vector3f torque = _gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);

	// update integral only if we are not landed
	if (!landed) {
		updateIntegral(rate_error, dt);
	}

	return torque;
}

void RateControl::updateIntegral(Vector3f &rate_error, const float dt)
{
	for (int i = 0; i < 3; i++) {
		// prevent further positive control saturation
		if (_control_allocator_saturation_positive(i)) {
			rate_error(i) = math::min(rate_error(i), 0.f);
		}

		// prevent further negative control saturation
		if (_control_allocator_saturation_negative(i)) {
			rate_error(i) = math::max(rate_error(i), 0.f);
		}

		// I term factor: reduce the I gain with increasing rate error.
		// This counteracts a non-linear effect where the integral builds up quickly upon a large setpoint
		// change (noticeable in a bounce-back effect after a flip).
		// The formula leads to a gradual decrease w/o steps, while only affecting the cases where it should:
		// with the parameter set to 400 degrees, up to 100 deg rate error, i_factor is almost 1 (having no effect),
		// and up to 200 deg error leads to <25% reduction of I.
		float i_factor = rate_error(i) / math::radians(400.f);
		i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

		// Perform the integration using a first order method
		float rate_i = _rate_int(i) + i_factor * _gain_i(i) * rate_error(i) * dt;

		// do not propagate the result if out of range or invalid
		if (PX4_ISFINITE(rate_i)) {
			_rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
		}
	}
}

void RateControl::getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status)
{
	rate_ctrl_status.rollspeed_integ = _rate_int(0);
	rate_ctrl_status.pitchspeed_integ = _rate_int(1);
	rate_ctrl_status.yawspeed_integ = _rate_int(2);
}

//增加smc的控制器
Vector3f RateControl::smcControl(const Vector3f &att, const Vector3f &att_sp, const Vector3f &rate, Vector3f &rate_sp,
			    hrt_abstime now)
{
	//smc_control 信息主要用作日志的记录量，方便分析
	smc_control_s control;
	control.timestamp = now;

	//预先处理？四元数->欧拉角

	//假定期望的角速率为0
	rate_sp.setZero();

	//记录期望信息
	control.attd[0] = att_sp(0);	control.attd[1] = att_sp(1);	control.attd[2] = att_sp(2);	//角度
	control.dattd[0] = rate_sp(0);	control.dattd[1] = rate_sp(1);	control.dattd[2] = rate_sp(2);	//角速度
	//记录状态信息
	control.att[0] = att(0);	control.att[1] = att(1);	control.att[2] = att(2);
	control.datt[0] = rate(0);	control.datt[1] = rate(1);	control.datt[2] = rate(2);
	//为了看着方便
	float phi = control.att[0];	float theta = control.att[1];	float psi = control.att[2];
	float dphi = control.datt[0];	float dtheta = control.datt[1];	float dpsi = control.datt[2];
	float phid = control.attd[0];	float thetad = control.attd[1];	float psid = control.attd[2];
	float dphid = control.dattd[0];	float dthetad = control.dattd[1];	float dpsid = control.dattd[2];

	//控制参数计算
	float a1 = (_iyy - _izz)/_ixx;	float a2 = (_izz - _ixx)/_iyy;	float a3 = (_ixx - _iyy)/_izz;
	float b1 = _d/_ixx;	float b2 = _d/_iyy;	float b3 = _d/_izz;

	//滑膜面计算
	float z1 = phid - phi;	float z2 = dphi - dphid - z1;
	float z3 = thetad - theta; float z4 = dtheta - dthetad - z3;
	float z5 = psid - psi;	float z6 = dpsi - dpsid - z5;
	//记录滑膜面信息
	control.z[0] = z1;	control.z[1] = z2;	control.z[2] = z3;
	control.z[3] = z4;	control.z[4] = z5;	control.z[5] = z6;

	//控制量计算
	// _control.u1 = 1/b1*(-sign(z2) - z2 - a1*dtheta*dpsi + dphid - dphi);
	// _control.u2 = 1/b2*(-sign(z4) - z4 - a2*dphi*dpsi + dthetad - dtheta);
	// _control.u3 = 1/b3*(-sign(z6) - z6 - a3*dphi*dtheta + dpsid - dpsi);
	control.u1 = 1/b1*(-z2 - z2 - a1*dtheta*dpsi + dphid - dphi);
	control.u2 = 1/b2*(-z4 - z4 - a2*dphi*dpsi + dthetad - dtheta);
	control.u3 = 1/b3*(-z6 - z6 - a3*dphi*dtheta + dpsid - dpsi);

	Vector3f torque;
	torque(0) = control.u1;
	torque(1) = control.u2;
	torque(2) = control.u3;

	_smc_control_pub.publish(control);

	return torque;
}
