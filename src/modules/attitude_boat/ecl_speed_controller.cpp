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
 * @file ecl_pitch_controller.cpp
 * Implementation of a simple speed PID controller.
 *
 * Authors and acknowledgements in header.
 */

#include "ecl_speed_controller.h"
#include <math.h>
#include <stdint.h>
#include <float.h>
#include <geo/geo.h>
#include <mathlib/mathlib.h>
#include <systemlib/err.h>

ECL_SpeedController::ECL_SpeedController() :
	ECL_Controller("speed"),
	_max_rate_neg(0.0f),
	_speed_ff(0.0f)
{
}

ECL_SpeedController::~ECL_SpeedController()
{
}

float ECL_SpeedController::control_attitude(const struct ECL_ControlData &ctl_data)
{

	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.speed_setpoint) &&
	      PX4_ISFINITE(ctl_data.roll) &&
	      PX4_ISFINITE(ctl_data.pitch) &&
	      PX4_ISFINITE(ctl_data.groundspeed))) {
		warnx("not controlling speed");
		return _rate_setpoint;
	}

	/* Calculate the error */
	float speed_error = ctl_data.speed_setpoint - ctl_data.groundspeed;

	/*  Apply P controller: rate setpoint from current error and time constant */
	_rate_setpoint =  speed_error / _tc;

	/* limit the rate */
	if (_max_rate > 0.01f && _max_rate_neg > 0.01f) {
		if (_rate_setpoint > 0.0f) {
			_rate_setpoint = (_rate_setpoint > _max_rate) ? _max_rate : _rate_setpoint;

		} else {
			_rate_setpoint = (_rate_setpoint < -_max_rate_neg) ? -_max_rate_neg : _rate_setpoint;
		}

	}

	return _rate_setpoint;
}

float ECL_SpeedController::control_bodyrate(const struct ECL_ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.pitch) &&
	      PX4_ISFINITE(ctl_data.roll_rate) &&
	      PX4_ISFINITE(ctl_data.yaw_rate) &&
	      PX4_ISFINITE(ctl_data.yaw_rate_setpoint) &&
	      PX4_ISFINITE(ctl_data.airspeed_min) &&
	      PX4_ISFINITE(ctl_data.airspeed_max) &&
	      PX4_ISFINITE(ctl_data.scaler))) {
		return math::constrain(_last_output, -1.0f, 1.0f);
	}

	/* get the usual dt estimate */
	uint64_t dt_micros = hrt_elapsed_time(&_last_run);
	_last_run = hrt_absolute_time();
	float dt = (float)dt_micros * 1e-6f;

	/* lock integral for long intervals */
	bool lock_integrator = ctl_data.lock_integrator;

	if (dt_micros > 500000) {
		lock_integrator = true;
	}

	_rate_error = _rate_setpoint;

	if (!lock_integrator && _k_i > 0.0f) {

		float id = _rate_error * dt;

		/*
		* anti-windup: do not allow integrator to increase if actuator is at limit
		*/
		if (_last_output < -1.0f) {
			/* only allow motion to center: increase value */
			id = math::max(id, 0.0f);

		} else if (_last_output > 1.0f) {
			/* only allow motion to center: decrease value */
			id = math::min(id, 0.0f);
		}

		_integrator += id * _k_i;
	}

	/* integrator limit */
	float integrator_constrained = math::constrain(_integrator, -_integrator_max, _integrator_max);

	/* Apply PI rate controller and store non-limited output */
	_last_output = _rate_error + integrator_constrained + _k_ff * ctl_data.speed_setpoint / ctl_data.airspeed_max;

	/* We are not allowing the motor to turn in reverse direction */
	return math::constrain(_last_output, 0.0f, 1.0f);
}