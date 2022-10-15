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

#include <lib/mixer/MultirotorMixer/MultirotorMixer.hpp>
#include <uORB/topics/rate_ctrl_status.h>

#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <matrix/matrix/math.hpp>
#include <uORB/Subscription.hpp>

#include <uORB/topics/smc_control.h>
#include <uORB/topics/asmc_control.h>
#include <uORB/topics/vehicle_local_position.h>

class ASMCControl
{
public:
	ASMCControl() = default;
	~ASMCControl() = default;

	/**
	 * @brief 设定控制器参数
	 *
	 * @param a1
	 * @param r0
	 * @param alpha
	 * @param d0
	 * @param e
	 * @param n
	 * @param gamma
	 * @param tau
	 */
	void setasmcParam(const matrix::Vector3f &a1, const matrix::Vector3f &r0, const matrix::Vector3f &alpha, const matrix::Vector3f d0,
					const matrix::Vector3f &e, const matrix::Vector3f &n, const matrix::Vector3f &gamma, const matrix::Vector3f &tau);

	/**
	 * @brief 设定阈值
	 *
	 * @param sat
	 */
	void setSaturation(const float &sat);

	/**
	 * @brief 设定控制状态量的初始值
	 *
	 */
	void setasmcInit();

	/**
	 * @brief Adaptive Sliding Mode Controller
	 *
	 * @param att
	 * @param att_sp
	 * @param rate
	 * @param rate_sp
	 * @param now
	 * @return Vector3f
	 */
	matrix::Vector3f asmcControl(const matrix::Vector3f &att, const matrix::Vector3f &att_sp, const matrix::Vector3f &rate, matrix::Vector3f &rate_sp,
			    hrt_abstime now, const float dt);

	/**
	 * @brief Get the States object
	 *
	 * @param asmccontrol
	 */
	void getStates(asmc_control_s &asmccontrol);
private:

	/**
	 * @brief 对sign(x)函数做阈值处理
	 *
	 * @param sigma
	 * @param sat
	 * @return 1 if sigma > sat
	 * 		    -1 if sigma < -sat
	 * 		   sigma/sat	if -sat < sigma < sat
	 */
	float saturation(const float &sigma, const float saturation);

	/**
	 * @brief 对输出控制量做限幅平均滤波
	 *
	 * @param torque
	 * @return Vector3f	滤波后的控制输入
	 */
	matrix::Vector3f filter(const matrix::Vector3f &torque);
	float filterf(const float &torque);

	//Subscription
	uORB::Subscription _vehicle_local_pos_sub{ORB_ID(vehicle_local_position)};

	// Publish
	uORB::Publication<smc_control_s>	_smc_control_pub{ORB_ID(smc_control)};
	uORB::Publication<asmc_control_s>	_asmc_control_pub{ORB_ID(asmc_control)};	//挪到McRateControl.run中
																								//在外面记录信息不准确影响分析，还是在里面记录

	// States
	// matrix::Vector3f _rate_int; ///< integral term of the rate controller

	// Feedback from control allocation
	// matrix::Vector<bool, 3> _control_allocator_saturation_negative;
	// matrix::Vector<bool, 3> _control_allocator_saturation_positive;

	//无人机位置信息
	vehicle_local_position_s _local_pos;
	float _t{0};

	//asmc控制信息
	asmc_control_s _asmccontrol;

	//模型参数
	// const float _ixx = 0.003,	_iyy = 0.003,	_izz = 0.006;	//实机
	const float _ixx = 0.03,	_iyy = 0.03,	_izz = 0.06;	//仿真参数
	const float _mass = 1.5;	//仿真参数
	// const float _mass = 0.553;
	const float _d = 0.66;	//电机离质心的距离

	//sat函数阈值
	float _saturation{0.8f};

	//滤波器参数
	// matrix::Vector3f _filter_torque(0, 0, 0);


	//asmc控制参数
	matrix::Vector3f _asmc_a1;
	matrix::Vector3f _asmc_r0;
	matrix::Vector3f _asmc_alpha;
	matrix::Vector3f _asmc_d0;
	matrix::Vector3f _asmc_e;
	matrix::Vector3f _asmc_n;
	matrix::Vector3f _asmc_gamma;
	matrix::Vector3f _asmc_tau;

	//asmc控制器过程参数
	matrix::Vector3f _asmc_rho;

	//asmc控制器状态量
	matrix::Vector3f _asmc_ueq{};
	matrix::Vector3f _asmc_k{};
	matrix::Vector3f _asmc_r{};
};
