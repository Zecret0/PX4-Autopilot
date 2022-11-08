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
 * @file ADRC.cpp
 */

#include <ADRC.hpp>
#include <px4_platform_common/defines.h>

using namespace matrix;

void ADRC::setParam(const float b01, const float b02, const float b03)
{
	_b01 = b01;
	_b02 = b02;
	_b03 = b03;
}


float ADRC::fal(const float e, const float alpha, const float delta)
{
	float s = (sign(e + delta) - sign(e - delta)) / 2;
	// float y = e/(delta^(1 - alpha)) * s + abs(e)^alpha*sign(e)*(1 - s);
	float y = e/(powf(delta, 1 - alpha)) * s + (powf(abs(e), alpha))*sign(e)*(1 - s);

	return y;
}

float ADRC::ESO(const float b0u, const float y, hrt_abstime now,  const float dt)
{
	float e = _eso_z1 - y;
	float fe = fal(e, 0.5,  0.004);
	float fe1 = fal(e, 0.25, 0.004);

	// _eso_z1 += (_eso_z2 - _b01*e)*0.1f;
	// _eso_z2 += (b0u - fe*_b02 + _eso_z3)*0.1f;
	// _eso_z3 += (fe1 * _b03)*0.1f;
	_eso_z1 += (_eso_z2 - _b01*e)*dt;
	_eso_z2 += (b0u - fe*_b02 + _eso_z3)*dt;
	_eso_z3 += (fe1 * _b03)*dt ;

	//记录日志并发布
	_adrcmsg.timestamp = now;
	_adrcmsg.dt = dt;

	_adrcmsg.b01 = _b01;
	_adrcmsg.b02 = _b02;
	_adrcmsg.b03 = _b03;

	_adrcmsg.b0u = b0u;
	_adrcmsg.y = y;

	_adrcmsg.e = e;
	_adrcmsg.fe = fe;
	_adrcmsg.fe1 = fe1;

	_adrcmsg.eso_z1 = _eso_z1;
	_adrcmsg.eso_z2 = _eso_z2;
	_adrcmsg.eso_z3 = _eso_z3;

	_adrc_pub.publish(_adrcmsg);

	return _eso_z3;
}
