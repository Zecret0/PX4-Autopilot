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
 * @file ASMCControl.hpp
 *
 * Adaptive Sliding Mode Controller
 */

#pragma once

#include <matrix/matrix/math.hpp>

#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <matrix/matrix/math.hpp>
#include <uORB/Subscription.hpp>



class ADRC
{
public:
	ADRC() = default;
	~ADRC() = default;

	/**
	 * @brief Set the Param object
	 *
	 * @param alpha
	 * @param delta
	 */
	void setParam(const float b01, const float b02, const float b03);

	/**
	 * @brief fal函数
	 *
	 * @param e
	 * @return float
	 */
	float fal(const float e, const float alpha, const float delta);
private:



	//Subscription

	// Publish

	//ESO状态量
	float _eso_z1{0.f};
	float _eso_z2{0.f};
	float _eso_z3{0.f};

	//adrc参数
	float _b01;
	float _b02;
	float _b03;


};
