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

//设定控制状态量初始值
void ASMCControl::setasmcInit()
{
	_asmc_r = _asmc_r0;
}

//增加asmc的控制器
Vector3f ASMCControl::asmcControl(const Vector3f &att, const Vector3f &att_sp, const Vector3f &rate, Vector3f &rate_sp,
			    hrt_abstime now)
{
	const Vector3f derr = rate - rate_sp;
	const Vector3f sigma = att - att_sp + derr;

	_asmc_rho = _asmc_r0 + _asmc_r;

	Vector3f ut = -(_asmc_k + _asmc_n).emult(sigma);
	Vector3f delta;
	for (int i = 0; i < 3; i++)	//有1/Vectro3f 的操作，不知道怎么处理，于是分开写
	{
		delta(i) = _asmc_k(i) - 1/_asmc_alpha(i)*abs(_asmc_ueq(i)) - _asmc_e(i);
		_asmc_ueq(i) = (1 - 1/_asmc_tau(i))*_asmc_ueq(i) + 1/_asmc_tau(i)*ut(i);
		_asmc_k(i) += _asmc_rho(i)*sign(delta(i));

		if (abs(delta(i)) > _asmc_d0(i))
		{
			_asmc_r(i) += _asmc_gamma(i)*abs(delta(i));
		} else{
			_asmc_r(i) += 0;
		}

	}
	// _asmc_k += -_asmc_rho.emult((float)sign(delta));

	//计算输出控制量
	Vector3f torque;
	torque(0) = -(_asmc_k(0) + _asmc_n(0))*(sigma(0)) - rate(1)*rate(2)*(_iyy - _izz)/_ixx - derr(0);
	torque(1) = -(_asmc_k(1) + _asmc_n(1))*(sigma(1)) - rate(0)*rate(2)*(_izz - _ixx)/_iyy - derr(1);
	torque(2) = -(_asmc_k(2) + _asmc_n(2))*(sigma(2)) - rate(0)*rate(1)*(_ixx - _iyy)/_izz - derr(2);
	// torque(0) = -(_asmc_k(0) + _asmc_n(0))*sign(sigma(0)) - rate(1)*rate(2)*(_iyy - _izz)/_ixx - derr(0);
	// torque(1) = -(_asmc_k(1) + _asmc_n(1))*sign(sigma(1)) - rate(0)*rate(2)*(_izz - _ixx)/_iyy - derr(1);
	// torque(2) = -(_asmc_k(2) + _asmc_n(2))*sign(sigma(2)) - rate(0)*rate(1)*(_ixx - _iyy)/_izz - derr(2);

	return torque;

}
