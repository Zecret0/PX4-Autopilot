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
 * @file ASMCControl.cpp
 */

#include <ASMCControl.hpp>
#include <px4_platform_common/defines.h>

using namespace matrix;

void ASMCControl::setasmcParam(const Vector3f &a1, const Vector3f &r0, const Vector3f &alpha, const Vector3f d0,
					const Vector3f &e, const Vector3f &n, const Vector3f &gamma, const Vector3f &tau)
{
	_asmc_a1 = a1;		_asmc_r0 = r0;		_asmc_alpha = alpha;		_asmc_d0 = d0;
	_asmc_e = e;		_asmc_n = n;		_asmc_gamma = gamma;		_asmc_tau = tau;

	setasmcInit();	//设定初始值
}
void ASMCControl::setSaturation(const float &sat)
{
	_saturation = sat;
}

//设定控制状态量初始值
void ASMCControl::setasmcInit()
{
	_asmc_r = _asmc_r0;
}

//获取控制器的参数用作日志记录
void ASMCControl::getStates(asmc_control_s &asmccontrol)
{
	for (int i = 0; i < 3; i++)
	{
		asmccontrol.ueq[i] = _asmc_ueq(i);
		asmccontrol.k[i] = _asmc_k(i);
		asmccontrol.r[i] = _asmc_r(i);

		asmccontrol.rho[i] = _asmc_rho(i);
	}

}

float ASMCControl::saturation(const float &sigma)
{
	if (sigma >= _saturation)
	{
		return 1.f;
	} else if (sigma <= -_saturation)
	{
		return -1.f;
	} else
	{
		return (sigma/_saturation);
	}
}

//限幅平均滤波
Vector3f ASMCControl::filter(const Vector3f &torque)
{
	Vector3f sum(0, 0, 0);
	// _filter_torque = torque;

	for (int i = 0; i < 12; i++)
	{
		sum += torque;
	}

	return (sum/12);

}

float ASMCControl::filterf(const float &value)
{
	float sum = 0;
	for (int i = 0; i < 12; i++)
	{
		sum+=value;
	}
	return (sum / 12.f);

}

//增加asmc的控制器
Vector3f ASMCControl::asmcControl(const Vector3f &att, const Vector3f &att_sp, const Vector3f &rate, Vector3f &rate_sp,
			    hrt_abstime now, const float dt)
{
	//for log
	// asmc_control_s asmccontrol;
	_asmccontrol.timestamp = now;

	rate_sp.setZero();	//期望角速度 = 0；
	const Vector3f derr = rate - rate_sp;
	const Vector3f sigma = filter(att) - filter(att_sp) + 0.1f*derr;
	// const Vector3f sigma = att - att_sp + 0.1f*derr;
	// const Vector3f sigma = att - att_sp + derr;

	_asmc_rho = _asmc_r0 + _asmc_r;

	// Vector3f ut = -(_asmc_k + _asmc_n).emult(sign(sigma));
	Vector3f ut;
	Vector3f delta;
	for (int i = 0; i < 3; i++)	//有1/Vectro3f 的操作，不知道怎么处理，于是分开写
	{
		ut(i) = -(_asmc_k(i) + _asmc_n(i))*saturation(sigma(i));
		// ut(i) = -(_asmc_k(i) + _asmc_n(i))*tanh(sigma(i));
		// ut(i) = -(_asmc_k(i) + _asmc_n(i))*sign(sigma(i));
		delta(i) = _asmc_k(i) - 1/_asmc_alpha(i)*abs(_asmc_ueq(i)) - _asmc_e(i);
		_asmc_ueq(i) += 1/_asmc_tau(i)*(ut(i) - _asmc_ueq(i))*dt;
		// _asmc_ueq(i) = (1 - 1/_asmc_tau(i))*_asmc_ueq(i) + 1/_asmc_tau(i)*ut(i);
		_asmc_k(i) += -_asmc_rho(i)*saturation(delta(i))*dt;
		// _asmc_k(i) += -_asmc_rho(i)*tanh(delta(i))*dt;
		// _asmc_k(i) += -_asmc_rho(i)*sign(delta(i))*dt;

		if (abs(delta(i)) > _asmc_d0(i))
		{
			_asmc_r(i) += _asmc_gamma(i)*abs(delta(i)) * dt;
		} else{
			_asmc_r(i) += 0;
		}

	}
	// _asmc_k += -_asmc_rho.emult((float)sign(delta));

	//计算输出控制量
	Vector3f torque;
	//做一个限幅
	_asmc_k(0) = math::constrain(_asmc_k(0), -1.f, 2.f);
	// torque(0) = -(_asmc_k(0) + _asmc_n(0))*(sigma(0)) - rate(1)*rate(2)*(_iyy - _izz)/_ixx - derr(0);
	// torque(1) = -(_asmc_k(1) + _asmc_n(1))*(sigma(1)) - rate(0)*rate(2)*(_izz - _ixx)/_iyy - derr(1);
	// torque(2) = -(_asmc_k(2) + _asmc_n(2))*(sigma(2)) - rate(0)*rate(1)*(_ixx - _iyy)/_izz - derr(2);
	//测试定值增益
	// torque(0) = -(_asmc_k(0) + _asmc_n(0))*tanh(sigma(0));
	//测试sat函数代替sign函数
	torque(0) = -(1.5f)*saturation(sigma(0)) - rate(1)*rate(2)*(_iyy - _izz)/_ixx - 0.1f*derr(0);
	// torque(0) = -(0.7f)*tanh(sigma(0)) - rate(1)*rate(2)*(_iyy - _izz)/_ixx - 0.1f*derr(0);
	// torque(0) = -(_asmc_k(0) + _asmc_n(0))*saturation(sigma(0)) - rate(1)*rate(2)*(_iyy - _izz)/_ixx - derr(0);
	torque(1) = -(_asmc_k(1) + _asmc_n(1))*sign(sigma(1)) - rate(0)*rate(2)*(_izz - _ixx)/_iyy - derr(1);
	torque(2) = -(_asmc_k(2) + _asmc_n(2))*sign(sigma(2)) - rate(0)*rate(1)*(_ixx - _iyy)/_izz - derr(2);

	torque(0) = math::constrain(torque(0), -1.0f, 1.0f);
	torque(1) = math::constrain(torque(1), -1.0f, 1.0f);
	torque(2) = math::constrain(torque(2), -1.0f, 1.0f);

	//for log
	for (int i = 0; i < 3; i++)
	{
		_asmccontrol.att[i] = filterf(att(i));
		// _asmccontrol.att[i] = att(i);
		_asmccontrol.att_sp[i] = filterf(att_sp(i));
		// _asmccontrol.att_sp[i] = att_sp(i);

		_asmccontrol.ueq[i] = _asmc_ueq(i);
		_asmccontrol.k[i] = _asmc_k(i);
		_asmccontrol.r[i] = _asmc_r(i);

		_asmccontrol.rho[i] = _asmc_rho(i);

		_asmccontrol.sigma[i] = sigma(i);
		_asmccontrol.derr[i] = derr(i);
		_asmccontrol.ut[i] = ut(i);
		_asmccontrol.delta[i] = delta(i);
	}

	_asmc_control_pub.publish(_asmccontrol);
	return torque;

}
