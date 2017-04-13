/****************************************************************************
 *
 *   Copyright (c) 2013 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file ecl_lift_controller.h
 * Definition of a simple lift PID controller.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @contributor Yoann Lapijover <yoann.lapijover@epfl.ch>
 *
 * Acknowledgements:
 *
 *   The control design is based on a design
 *   by Paul Riseborough and Andrew Tridgell, 2013,
 *   which in turn is based on initial work of
 *   Jonathan Challinger, 2012.
 */

#ifndef ECL_LIFT_CONTROLLER_H
#define ECL_LIFT_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include "ecl_controller.h"

class __EXPORT ECL_LiftController :
	public ECL_Controller
{
public:
	ECL_LiftController();

	~ECL_LiftController();

	float control_attitude(const struct ECL_ControlData &ctl_data);
	float control_bodyrate(const struct ECL_ControlData &ctl_data);

	/* Additional Setters */
	void set_max_rate_pos(float max_rate_pos)
	{
		_max_rate = max_rate_pos;
	}

	void set_max_rate_neg(float max_rate_neg)
	{
		_max_rate_neg = max_rate_neg;
	}

	void set_lift_ff(float lift_ff)
	{
		_lift_ff = lift_ff;
	}

	void set_lift_kp_n(float kp_n)
	{
		_k_p_n = kp_n;
	}

	void set_lift_ki_n(float ki_n)
	{
		_k_i_n = ki_n;
	}

	void set_palp_length(float value) {
		_palp_length = value;
	}

	void set_palp_min_speed(float value) {
		_palp_min_speed = value;
	}

	void set_palp_dft_hgt(float value) {
		_palp_dft_hgt = value;
	}


protected:
	float _max_rate_neg;
	float _lift_ff;
	float _k_p_n;
	float _k_i_n;
	float _palp_length;
	float _palp_min_speed;
	float _palp_dft_hgt;

};

#endif // ECL_LIFT_CONTROLLER_H
