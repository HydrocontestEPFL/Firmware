/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file fw_att_control_main.c
 * Implementation of a generic attitude controller based on classic orthogonal PIDs.
 *
 * @author Lorenz Meier 	<lm@inf.ethz.ch>
 * @author Thomas Gubler 	<thomasgubler@gmail.com>
 * @author Roman Bapst		<bapstr@ethz.ch>
 *
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/fw_virtual_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/fw_virtual_rates_setpoint.h>
#include <uORB/topics/mc_virtual_rates_setpoint.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/distance_sensor.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>

#include <attitude_boat/ecl_speed_controller.h>
#include <attitude_boat/ecl_lift_controller.h>
#include <attitude_boat/ecl_roll_controller.h>
#include <attitude_boat/ecl_yaw_controller.h>
#include <attitude_boat/ecl_pitch_controller.h>
#include <platforms/px4_defines.h>

/**
 * Fixedwing attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int fw_att_control_main(int argc, char *argv[]);

class FixedwingAttitudeControl
{
public:
	/**
	 * Constructor
	 */
	FixedwingAttitudeControl();

	/**
	 * Destructor, also kills the main task.
	 */
	~FixedwingAttitudeControl();

	/**
	 * Start the main task.
	 *
	 * @return	OK on success.
	 */
	int		start();

	/**
	 * Task status
	 *
	 * @return	true if the mainloop is running
	 */
	bool		task_running() { return _task_running; }

private:

	bool		_task_should_exit;		/**< if true, attitude control task should exit */
	bool		_task_running;			/**< if true, task is running in its mainloop */
	int		_control_task;			/**< task handle */

	int		_ctrl_state_sub;	/**< control state subscription */
	int		_accel_sub;			/**< accelerometer subscription */
	int		_att_sp_sub;			/**< vehicle attitude setpoint */
	int		_attitude_sub;			/**< raw rc channels data subscription */
	int		_vcontrol_mode_sub;		/**< vehicle status subscription */
	int 		_params_sub;			/**< notification of parameter updates */
	int 		_manual_sub;			/**< notification of manual control updates */
	int		_global_pos_sub;		/**< global position subscription */
	int 	_local_pos_sub;
	int		_vehicle_status_sub;		/**< vehicle status subscription */
	int		_vehicle_land_detected_sub;	/**< vehicle land detected subscription */
	int 	_distance_sensor_sub;

	orb_advert_t	_rate_sp_pub;			/**< rate setpoint publication */
	orb_advert_t	_attitude_sp_pub;		/**< attitude setpoint point */
	orb_advert_t	_actuators_0_pub;		/**< actuator control group 0 setpoint */
	orb_advert_t	_actuators_1_pub;		/**< actuator control group 1 setpoint (Airframe) */
	orb_advert_t	_actuators_2_pub;		/**< actuator control group 2 setpoint (Airframe) */

	orb_id_t _rates_sp_id;	// pointer to correct rates setpoint uORB metadata structure
	orb_id_t _actuators_id;	// pointer to correct actuator controls0 uORB metadata structure
	orb_id_t _attitude_setpoint_id;

	struct control_state_s				_ctrl_state;	/**< control state */
	struct accel_report					_accel;			/**< body frame accelerations */
	struct vehicle_attitude_setpoint_s	_att_sp;		/**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s		_rates_sp;	/* attitude rates setpoint */
	struct manual_control_setpoint_s	_manual;		/**< r/c channel data */
	struct vehicle_control_mode_s		_vcontrol_mode;		/**< vehicle control mode */
	struct actuator_controls_s			_actuators;		/**< actuator control inputs */
	struct actuator_controls_s			_actuators_airframe;	/**< actuator control inputs */
	struct actuator_controls_s			_actuators_alternate; /**< actuator control inputs alternate for back flap */
	struct vehicle_global_position_s	_global_pos;		/**< global position */
	struct vehicle_local_position_s		_local_pos;		/**< global position */
	struct vehicle_status_s				_vehicle_status;	/**< vehicle status */
	struct vehicle_land_detected_s		_vehicle_land_detected;	/**< vehicle land detected */
	struct distance_sensor_s			_distance_sensor;

	hrt_abstime _last_update_distance;

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_nonfinite_input_perf;		/**< performance counter for non finite input */
	perf_counter_t	_nonfinite_output_perf;		/**< performance counter for non finite output */

	bool		_setpoint_valid;		/**< flag if the position control setpoint is valid */
	bool		_debug;				/**< if set to true, print debug output */

	float _flaps_cmd_last;
	float _flaperons_cmd_last;


	struct {
		float p_tc;
		float p_p;
		float p_i;
		float p_ff;
		float p_rmax_pos;
		float p_rmax_neg;
		float p_integrator_max;

		float l_tc;
		float l_p;
		float l_p_n;
		float l_i;
		float l_i_n;
		float l_ff;
		float l_rmax_pos;
		float l_rmax_neg;
		float l_integrator_max;

		float palp_length;
		float palp_min_speed;
		float palp_dft_hgt;

		float s_tc;
		float s_p;
		float s_i;
		float s_ff;
		float s_rmax_pos;
		float s_rmax_neg;
		float s_integrator_max;

		float r_tc;
		float r_p;
		float r_i;
		float r_ff;
		float r_integrator_max;
		float r_rmax;
		float y_p;
		float y_i;
		float y_d;
		float y_ff;
		float y_integrator_max;
		float y_coordinated_min_speed;
		int32_t y_coordinated_method;
		float y_rmax;
		float w_p;
		float w_i;
		float w_ff;
		float w_integrator_max;
		float w_rmax;

		float airspeed_min;
		float airspeed_trim;
		float airspeed_max;

		float trim_roll;
		float trim_pitch;
		float trim_yaw;
		float rollsp_offset_deg;		/**< Roll Setpoint Offset in deg */
		float pitchsp_offset_deg;		/**< Pitch Setpoint Offset in deg */
		float rollsp_offset_rad;		/**< Roll Setpoint Offset in rad */
		float pitchsp_offset_rad;		/**< Pitch Setpoint Offset in rad */
		float man_roll_max;				/**< Max Roll in rad */
		float man_pitch_max;			/**< Max Pitch in rad */
		float man_yaw_rate_max;
		float roll_yaw_rate_scaling;

		float flaps_scale;				/**< Scale factor for flaps */
		float flaperon_scale;			/**< Scale factor for flaperons */

		int vtol_type;					/**< VTOL type: 0 = tailsitter, 1 = tiltrotor */

	}		_parameters;			/**< local copies of interesting parameters */

	struct {

		param_t p_tc;
		param_t p_p;
		param_t p_i;
		param_t p_ff;
		param_t p_rmax_pos;
		param_t p_rmax_neg;
		param_t p_integrator_max;

		param_t s_tc;
		param_t s_p;
		param_t s_i;
		param_t s_ff;
		param_t s_rmax_pos;
		param_t s_rmax_neg;
		param_t s_integrator_max;

		param_t l_tc;
		param_t l_p;
		param_t l_p_n;
		param_t l_i;
		param_t l_i_n;
		param_t l_ff;
		param_t l_rmax_pos;
		param_t l_rmax_neg;
		param_t l_integrator_max;

		param_t palp_length;
		param_t palp_min_speed;
		param_t palp_dft_hgt;

		param_t r_tc;
		param_t r_p;
		param_t r_i;
		param_t r_ff;
		param_t r_integrator_max;
		param_t r_rmax;
		param_t y_p;
		param_t y_i;
		param_t y_d;
		param_t y_ff;
		param_t y_integrator_max;
		param_t y_coordinated_min_speed;
		param_t y_coordinated_method;
		param_t y_rmax;
		param_t w_p;
		param_t w_i;
		param_t w_ff;
		param_t w_integrator_max;
		param_t w_rmax;

		param_t airspeed_min;
		param_t airspeed_trim;
		param_t airspeed_max;

		param_t trim_roll;
		param_t trim_pitch;
		param_t trim_yaw;
		param_t rollsp_offset_deg;
		param_t pitchsp_offset_deg;
		param_t man_roll_max;
		param_t man_pitch_max;
		param_t man_yaw_rate_max;

		param_t roll_yaw_rate_scaling;

		param_t flaps_scale;
		param_t flaperon_scale;

		param_t vtol_type;

	}		_parameter_handles;		/**< handles for interesting parameters */

	// Rotation matrix and euler angles to extract from control state
	math::Matrix<3, 3> _R;
	float _roll;
	float _pitch;
	float _yaw;

	ECL_RollController				_roll_ctrl;
	ECL_PitchController				_pitch_ctrl;
	ECL_YawController				_yaw_ctrl;
	ECL_LiftController 				_lift_ctrl;
	ECL_SpeedController 			_speed_ctrl;


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Update control outputs
	 *
	 */
	void		control_update();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	void		vehicle_manual_poll();


	/**
	* Check for changes in the distance sensor
	*/
	void 		distance_sensor_poll();

	/**
	 * Check for accel updates.
	 */
	void		vehicle_accel_poll();

	/**
	 * Check for set triplet updates.
	 */
	void		vehicle_setpoint_poll();

	/**
	 * Check for global position updates.
	 */
	void		global_pos_poll();

	/**
	 * Check for local position updates.
	 */
	void		local_pos_poll();


	/**
	 * Check for vehicle status updates.
	 */
	void		vehicle_status_poll();

	/**
	 * Check for vehicle land detected updates.
	 */
	void		vehicle_land_detected_poll();

	/**
	 * Calcul the correct roll for a given speed and yaw_rate
	 */
	float 		rollFromYawRate(float yaw_rate_ref, float boat_speed);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude controller collection task.
	 */
	void		task_main();

};

namespace att_control
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

FixedwingAttitudeControl	*g_control = nullptr;
}

FixedwingAttitudeControl::FixedwingAttitudeControl() :

	_task_should_exit(false),
	_task_running(false),
	_control_task(-1),

	/* subscriptions */
	_ctrl_state_sub(-1),
	_accel_sub(-1),
	_vcontrol_mode_sub(-1),
	_params_sub(-1),
	_manual_sub(-1),
	_global_pos_sub(-1),
	_local_pos_sub(-1),
	_vehicle_status_sub(-1),
	_vehicle_land_detected_sub(-1),
	_distance_sensor_sub(-1),

	/* publications */
	_rate_sp_pub(nullptr),
	_attitude_sp_pub(nullptr),
	_actuators_0_pub(nullptr),
	_actuators_1_pub(nullptr),
	_actuators_2_pub(nullptr),

	_rates_sp_id(0),
	_actuators_id(0),
	_attitude_setpoint_id(0),

	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "fwa_dt")),
#if 0
	_nonfinite_input_perf(perf_alloc(PC_COUNT, "fwa_nani")),
	_nonfinite_output_perf(perf_alloc(PC_COUNT, "fwa_nano")),
#else
	_nonfinite_input_perf(nullptr),
	_nonfinite_output_perf(nullptr),
#endif
	/* states */
	_setpoint_valid(false),
	_debug(false),
	_flaps_cmd_last(0),
	_flaperons_cmd_last(0)
{
	/* safely initialize structs */
	_ctrl_state = {};
	_accel = {};
	_att_sp = {};
	_rates_sp = {};
	_manual = {};
	_vcontrol_mode = {};
	_actuators = {};
	_actuators_airframe = {};
	_actuators_alternate = {};
	_global_pos = {};
	_local_pos = {};
	_vehicle_status = {};
	_vehicle_land_detected = {};
	_distance_sensor = {};


	_parameter_handles.p_tc = param_find("FW_P_TC");
	_parameter_handles.p_p = param_find("FW_PR_P");
	_parameter_handles.p_i = param_find("FW_PR_I");
	_parameter_handles.p_ff = param_find("FW_PR_FF");
	_parameter_handles.p_rmax_pos = param_find("FW_P_RMAX_POS");
	_parameter_handles.p_rmax_neg = param_find("FW_P_RMAX_NEG");
	_parameter_handles.p_integrator_max = param_find("FW_PR_IMAX");

	_parameter_handles.s_tc = param_find("BF_S_TC");
	_parameter_handles.s_p = param_find("BF_SR_P");
	_parameter_handles.s_i = param_find("BF_SR_I");
	_parameter_handles.s_ff = param_find("BF_SR_FF");
	_parameter_handles.s_rmax_pos = param_find("BF_S_RMAX_POS");
	_parameter_handles.s_rmax_neg = param_find("BF_S_RMAX_NEG");
	_parameter_handles.s_integrator_max = param_find("BF_SR_IMAX");

	_parameter_handles.l_tc = param_find("BF_L_TC");
	_parameter_handles.l_p = param_find("BF_LR_P");
	_parameter_handles.l_p_n = param_find("BF_LR_P_N");
	_parameter_handles.l_i = param_find("BF_LR_I");
	_parameter_handles.l_i_n = param_find("BF_LR_I_N");
	_parameter_handles.l_ff = param_find("BF_LR_FF");
	_parameter_handles.l_rmax_pos = param_find("BF_L_RMAX_POS");
	_parameter_handles.l_rmax_neg = param_find("BF_L_RMAX_NEG");
	_parameter_handles.l_integrator_max = param_find("BF_LR_IMAX");

	_parameter_handles.palp_length = param_find("BF_PALP_LENGTH");
	_parameter_handles.palp_min_speed = param_find("BF_PALP_SPD_MIN");
	_parameter_handles.palp_dft_hgt = param_find("BF_PALP_DFT_HGT");

	_parameter_handles.r_tc = param_find("FW_R_TC");
	_parameter_handles.r_p = param_find("FW_RR_P");
	_parameter_handles.r_i = param_find("FW_RR_I");
	_parameter_handles.r_ff = param_find("FW_RR_FF");
	_parameter_handles.r_integrator_max = param_find("FW_RR_IMAX");
	_parameter_handles.r_rmax = param_find("FW_R_RMAX");

	_parameter_handles.y_p = param_find("FW_YR_P");
	_parameter_handles.y_i = param_find("FW_YR_I");
	_parameter_handles.y_ff = param_find("FW_YR_FF");
	_parameter_handles.y_integrator_max = param_find("FW_YR_IMAX");
	_parameter_handles.y_rmax = param_find("FW_Y_RMAX");

	_parameter_handles.w_p = param_find("FW_WR_P");
	_parameter_handles.w_i = param_find("FW_WR_I");
	_parameter_handles.w_ff = param_find("FW_WR_FF");
	_parameter_handles.w_integrator_max = param_find("FW_WR_IMAX");
	_parameter_handles.w_rmax = param_find("FW_W_RMAX");

	_parameter_handles.airspeed_min = param_find("FW_AIRSPD_MIN");
	_parameter_handles.airspeed_trim = param_find("FW_AIRSPD_TRIM");
	_parameter_handles.airspeed_max = param_find("FW_AIRSPD_MAX");

	_parameter_handles.y_coordinated_min_speed = param_find("FW_YCO_VMIN");
	_parameter_handles.y_coordinated_method = param_find("FW_YCO_METHOD");

	_parameter_handles.trim_roll = param_find("TRIM_ROLL");
	_parameter_handles.trim_pitch = param_find("TRIM_PITCH");
	_parameter_handles.trim_yaw = param_find("TRIM_YAW");
	_parameter_handles.rollsp_offset_deg = param_find("FW_RSP_OFF");
	_parameter_handles.pitchsp_offset_deg = param_find("FW_PSP_OFF");

	_parameter_handles.man_roll_max = param_find("FW_MAN_R_MAX");
	_parameter_handles.man_pitch_max = param_find("FW_MAN_P_MAX");
	_parameter_handles.man_yaw_rate_max = param_find("BF_MAN_YR_MAX");

	_parameter_handles.roll_yaw_rate_scaling = param_find("BF_R_YR_SCAL");


	_parameter_handles.flaps_scale = param_find("FW_FLAPS_SCL");
	_parameter_handles.flaperon_scale = param_find("FW_FLAPERON_SCL");

	_parameter_handles.vtol_type = param_find("VT_TYPE");

	/* fetch initial parameter values */
	parameters_update();
}

FixedwingAttitudeControl::~FixedwingAttitudeControl()
{
	if (_control_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	perf_free(_loop_perf);
	perf_free(_nonfinite_input_perf);
	perf_free(_nonfinite_output_perf);

	att_control::g_control = nullptr;
}

int
FixedwingAttitudeControl::parameters_update()
{

	param_get(_parameter_handles.p_tc, &(_parameters.p_tc));
	param_get(_parameter_handles.p_p, &(_parameters.p_p));
	param_get(_parameter_handles.p_i, &(_parameters.p_i));
	param_get(_parameter_handles.p_ff, &(_parameters.p_ff));
	param_get(_parameter_handles.p_rmax_pos, &(_parameters.p_rmax_pos));
	param_get(_parameter_handles.p_rmax_neg, &(_parameters.p_rmax_neg));
	param_get(_parameter_handles.p_integrator_max, &(_parameters.p_integrator_max));

	param_get(_parameter_handles.l_tc, &(_parameters.l_tc));
	param_get(_parameter_handles.l_p, &(_parameters.l_p));
	param_get(_parameter_handles.l_p_n, &(_parameters.l_p_n));
	param_get(_parameter_handles.l_i, &(_parameters.l_i));
	param_get(_parameter_handles.l_i_n, &(_parameters.l_i_n));
	param_get(_parameter_handles.l_ff, &(_parameters.l_ff));
	param_get(_parameter_handles.l_rmax_pos, &(_parameters.l_rmax_pos));
	param_get(_parameter_handles.l_rmax_neg, &(_parameters.l_rmax_neg));
	param_get(_parameter_handles.l_integrator_max, &(_parameters.l_integrator_max));

	param_get(_parameter_handles.s_tc, &(_parameters.s_tc));
	param_get(_parameter_handles.s_p, &(_parameters.s_p));
	param_get(_parameter_handles.s_i, &(_parameters.s_i));
	param_get(_parameter_handles.s_ff, &(_parameters.s_ff));
	param_get(_parameter_handles.s_rmax_pos, &(_parameters.s_rmax_pos));
	param_get(_parameter_handles.s_rmax_neg, &(_parameters.s_rmax_neg));
	param_get(_parameter_handles.s_integrator_max, &(_parameters.s_integrator_max));

	param_get(_parameter_handles.palp_length, &(_parameters.palp_length));
	param_get(_parameter_handles.palp_min_speed, &(_parameters.palp_min_speed));
	param_get(_parameter_handles.palp_dft_hgt, &(_parameters.palp_dft_hgt));


	param_get(_parameter_handles.r_tc, &(_parameters.r_tc));
	param_get(_parameter_handles.r_p, &(_parameters.r_p));
	param_get(_parameter_handles.r_i, &(_parameters.r_i));
	param_get(_parameter_handles.r_ff, &(_parameters.r_ff));

	param_get(_parameter_handles.r_integrator_max, &(_parameters.r_integrator_max));
	param_get(_parameter_handles.r_rmax, &(_parameters.r_rmax));

	param_get(_parameter_handles.y_p, &(_parameters.y_p));
	param_get(_parameter_handles.y_i, &(_parameters.y_i));
	param_get(_parameter_handles.y_ff, &(_parameters.y_ff));
	param_get(_parameter_handles.y_integrator_max, &(_parameters.y_integrator_max));
	param_get(_parameter_handles.y_coordinated_min_speed, &(_parameters.y_coordinated_min_speed));
	param_get(_parameter_handles.y_coordinated_method, &(_parameters.y_coordinated_method));
	param_get(_parameter_handles.y_rmax, &(_parameters.y_rmax));

	param_get(_parameter_handles.w_p, &(_parameters.w_p));
	param_get(_parameter_handles.w_i, &(_parameters.w_i));
	param_get(_parameter_handles.w_ff, &(_parameters.w_ff));
	param_get(_parameter_handles.w_integrator_max, &(_parameters.w_integrator_max));
	param_get(_parameter_handles.w_rmax, &(_parameters.w_rmax));

	param_get(_parameter_handles.airspeed_min, &(_parameters.airspeed_min));
	param_get(_parameter_handles.airspeed_trim, &(_parameters.airspeed_trim));
	param_get(_parameter_handles.airspeed_max, &(_parameters.airspeed_max));

	param_get(_parameter_handles.trim_roll, &(_parameters.trim_roll));
	param_get(_parameter_handles.trim_pitch, &(_parameters.trim_pitch));
	param_get(_parameter_handles.trim_yaw, &(_parameters.trim_yaw));
	param_get(_parameter_handles.rollsp_offset_deg, &(_parameters.rollsp_offset_deg));
	param_get(_parameter_handles.pitchsp_offset_deg, &(_parameters.pitchsp_offset_deg));
	_parameters.rollsp_offset_rad = math::radians(_parameters.rollsp_offset_deg);
	_parameters.pitchsp_offset_rad = math::radians(_parameters.pitchsp_offset_deg);
	param_get(_parameter_handles.man_roll_max, &(_parameters.man_roll_max));
	param_get(_parameter_handles.man_pitch_max, &(_parameters.man_pitch_max));
	param_get(_parameter_handles.man_yaw_rate_max, &(_parameters.man_yaw_rate_max));
	_parameters.man_roll_max = math::radians(_parameters.man_roll_max);
	_parameters.man_pitch_max = math::radians(_parameters.man_pitch_max);
	_parameters.man_yaw_rate_max = math::radians(_parameters.man_yaw_rate_max);

	param_get(_parameter_handles.roll_yaw_rate_scaling, &(_parameters.roll_yaw_rate_scaling));

	param_get(_parameter_handles.flaps_scale, &_parameters.flaps_scale);
	param_get(_parameter_handles.flaperon_scale, &_parameters.flaperon_scale);

	param_get(_parameter_handles.vtol_type, &_parameters.vtol_type);

	/* pitch control parameters */
	_pitch_ctrl.set_time_constant(_parameters.p_tc);
	_pitch_ctrl.set_k_p(_parameters.p_p);
	_pitch_ctrl.set_k_i(_parameters.p_i);
	_pitch_ctrl.set_k_ff(_parameters.p_ff);
	_pitch_ctrl.set_integrator_max(_parameters.p_integrator_max);
	_pitch_ctrl.set_max_rate_pos(math::radians(_parameters.p_rmax_pos));
	_pitch_ctrl.set_max_rate_neg(math::radians(_parameters.p_rmax_neg));

	/* speed control parameter */
	_speed_ctrl.set_time_constant(_parameters.s_tc);
	_speed_ctrl.set_k_p(_parameters.s_p);
	_speed_ctrl.set_k_i(_parameters.s_i);
	_speed_ctrl.set_k_ff(_parameters.s_ff);
	_speed_ctrl.set_integrator_max(_parameters.s_integrator_max);
	_speed_ctrl.set_max_rate_pos(math::radians(_parameters.s_rmax_pos));
	_speed_ctrl.set_max_rate_neg(math::radians(_parameters.s_rmax_neg));

	/* lift control parameter */
	_lift_ctrl.set_time_constant(_parameters.l_tc);
	_lift_ctrl.set_k_p(_parameters.l_p);
	_lift_ctrl.set_lift_kp_n(_parameters.l_p_n);
	_lift_ctrl.set_k_i(_parameters.l_i);
	_lift_ctrl.set_lift_ki_n(_parameters.l_i_n);
	_lift_ctrl.set_k_ff(_parameters.l_ff);
	_lift_ctrl.set_integrator_max(_parameters.l_integrator_max);
	_lift_ctrl.set_max_rate_pos(math::radians(_parameters.l_rmax_pos));
	_lift_ctrl.set_max_rate_neg(math::radians(_parameters.l_rmax_neg));
	_lift_ctrl.set_palp_length(_parameters.palp_length);
	_lift_ctrl.set_palp_min_speed(_parameters.palp_min_speed);
	_lift_ctrl.set_palp_dft_hgt(_parameters.palp_dft_hgt);

	/* roll control parameters */
	_roll_ctrl.set_time_constant(_parameters.r_tc);
	_roll_ctrl.set_k_p(_parameters.r_p);
	_roll_ctrl.set_k_i(_parameters.r_i);
	_roll_ctrl.set_k_ff(_parameters.r_ff);
	_roll_ctrl.set_integrator_max(_parameters.r_integrator_max);
	_roll_ctrl.set_max_rate(math::radians(_parameters.r_rmax));

	/* yaw control parameters */
	_yaw_ctrl.set_k_p(_parameters.y_p);
	_yaw_ctrl.set_k_i(_parameters.y_i);
	_yaw_ctrl.set_k_ff(_parameters.y_ff);
	_yaw_ctrl.set_integrator_max(_parameters.y_integrator_max);
	_yaw_ctrl.set_coordinated_min_speed(_parameters.y_coordinated_min_speed);
	_yaw_ctrl.set_max_rate(math::radians(_parameters.y_rmax));

	return OK;
}

void
FixedwingAttitudeControl::vehicle_control_mode_poll()
{
	bool vcontrol_mode_updated;

	/* Check if vehicle control mode has changed */
	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);

	if (vcontrol_mode_updated) {

		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &_vcontrol_mode);
	}
}

void
FixedwingAttitudeControl::vehicle_manual_poll()
{
	bool manual_updated;

	/* get pilots inputs */
	orb_check(_manual_sub, &manual_updated);

	if (manual_updated) {

		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}
}

void
FixedwingAttitudeControl::distance_sensor_poll()
{
	bool distance_updated;

	/* get pilots inputs */
	orb_check(_distance_sensor_sub, &distance_updated);

	if (distance_updated) {

		orb_copy(ORB_ID(distance_sensor), _distance_sensor_sub, &_distance_sensor);
		_last_update_distance = hrt_absolute_time();
	}
}

void
FixedwingAttitudeControl::vehicle_accel_poll()
{
	/* check if there is a new position */
	bool accel_updated;
	orb_check(_accel_sub, &accel_updated);

	if (accel_updated) {
		orb_copy(ORB_ID(sensor_accel), _accel_sub, &_accel);
	}
}

void
FixedwingAttitudeControl::vehicle_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool att_sp_updated;
	orb_check(_att_sp_sub, &att_sp_updated);

	if (att_sp_updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
		_setpoint_valid = true;
	}
}

void
FixedwingAttitudeControl::global_pos_poll()
{
	/* check if there is a new global position */
	bool global_pos_updated;
	orb_check(_global_pos_sub, &global_pos_updated);

	if (global_pos_updated) {
		orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
	}
}

void
FixedwingAttitudeControl::local_pos_poll()
{
	/* check if there is a new global position */
	bool local_pos_updated;
	orb_check(_local_pos_sub, &local_pos_updated);

	if (local_pos_updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
	}
}

void
FixedwingAttitudeControl::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_rates_sp_id) {
			if (_vehicle_status.is_vtol) {
				_rates_sp_id = ORB_ID(fw_virtual_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_virtual_fw);
				_attitude_setpoint_id = ORB_ID(fw_virtual_attitude_setpoint);

			} else {
				_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_0);
				_attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
			}
		}
	}
}

void
FixedwingAttitudeControl::vehicle_land_detected_poll()
{
	/* check if there is new status information */
	bool vehicle_land_detected_updated;
	orb_check(_vehicle_land_detected_sub, &vehicle_land_detected_updated);

	if (vehicle_land_detected_updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
	}
}

void
FixedwingAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
	att_control::g_control->task_main();
}

void
FixedwingAttitudeControl::task_main()
{
	/*
	 * do subscriptions
	 */
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_accel_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 0);
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_distance_sensor_sub = orb_subscribe(ORB_ID(distance_sensor));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));

	parameters_update();

	/* get an initial update for all sensor and status data */
	vehicle_setpoint_poll();
	vehicle_accel_poll();
	vehicle_control_mode_poll();
	vehicle_manual_poll();
	distance_sensor_poll();
	vehicle_status_poll();
	vehicle_land_detected_poll();

	/* wakeup source */
	px4_pollfd_struct_t fds[2];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _ctrl_state_sub;
	fds[1].events = POLLIN;

	_task_running = true;

	while (!_task_should_exit) {
		static int loop_counter = 0;

		/* wait for up to 500ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		perf_begin(_loop_perf);

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* only run controller if attitude changed */
		if (fds[1].revents & POLLIN) {
			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too large deltaT's */
			if (deltaT > 1.0f) {
				deltaT = 0.01f;
			}

			/* load local copies */
			orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);


			/* get current rotation matrix and euler angles from control state quaternions */
			math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
			_R = q_att.to_dcm();

			math::Vector<3> euler_angles;
			euler_angles = _R.to_euler();
			_roll    = euler_angles(0);
			_pitch   = euler_angles(1);
			_yaw     = euler_angles(2);

			if (_vehicle_status.is_vtol && _parameters.vtol_type == 0) {
				/* vehicle is a tailsitter, we need to modify the estimated attitude for fw mode
				 *
				 * Since the VTOL airframe is initialized as a multicopter we need to
				 * modify the estimated attitude for the fixed wing operation.
				 * Since the neutral position of the vehicle in fixed wing mode is -90 degrees rotated around
				 * the pitch axis compared to the neutral position of the vehicle in multicopter mode
				 * we need to swap the roll and the yaw axis (1st and 3rd column) in the rotation matrix.
				 * Additionally, in order to get the correct sign of the pitch, we need to multiply
				 * the new x axis of the rotation matrix with -1
				 *
				 * original:			modified:
				 *
				 * Rxx  Ryx  Rzx		-Rzx  Ryx  Rxx
				 * Rxy	Ryy  Rzy		-Rzy  Ryy  Rxy
				 * Rxz	Ryz  Rzz		-Rzz  Ryz  Rxz
				 * */
				math::Matrix<3, 3> R_adapted = _R;		//modified rotation matrix

				/* move z to x */
				R_adapted(0, 0) = _R(0, 2);
				R_adapted(1, 0) = _R(1, 2);
				R_adapted(2, 0) = _R(2, 2);

				/* move x to z */
				R_adapted(0, 2) = _R(0, 0);
				R_adapted(1, 2) = _R(1, 0);
				R_adapted(2, 2) = _R(2, 0);

				/* change direction of pitch (convert to right handed system) */
				R_adapted(0, 0) = -R_adapted(0, 0);
				R_adapted(1, 0) = -R_adapted(1, 0);
				R_adapted(2, 0) = -R_adapted(2, 0);
				euler_angles = R_adapted.to_euler();  //adapted euler angles for fixed wing operation

				/* fill in new attitude data */
				_R = R_adapted;
				_roll    = euler_angles(0);
				_pitch   = euler_angles(1);
				_yaw     = euler_angles(2);

				/* lastly, roll- and yawspeed have to be swaped */
				float helper = _ctrl_state.roll_rate;
				_ctrl_state.roll_rate = -_ctrl_state.yaw_rate;
				_ctrl_state.yaw_rate = helper;
			}

			vehicle_setpoint_poll();

			vehicle_accel_poll();

			vehicle_control_mode_poll();

			vehicle_manual_poll();

			distance_sensor_poll();

			global_pos_poll();

			local_pos_poll();

			vehicle_status_poll();

			vehicle_land_detected_poll();

			// the position controller will not emit attitude setpoints in some modes
			// we need to make sure that this flag is reset
			_att_sp.fw_control_yaw = _att_sp.fw_control_yaw && _vcontrol_mode.flag_control_auto_enabled;

			/* lock integrator until control is started */
			bool lock_integrator;

			if (_vcontrol_mode.flag_control_attitude_enabled && !_vehicle_status.is_rotary_wing) {
				lock_integrator = false;

			} else {
				lock_integrator = true;
			}


			/* decide if in stabilized or full manual control */
			if (_vcontrol_mode.flag_control_attitude_enabled) {

				/* Prepare speed_body_u and speed_body_w */
				float speed_body_u = _R(0, 0) * _global_pos.vel_n + _R(1, 0) * _global_pos.vel_e + _R(2, 0) * _global_pos.vel_d;
				float speed_body_v = _R(0, 1) * _global_pos.vel_n + _R(1, 1) * _global_pos.vel_e + _R(2, 1) * _global_pos.vel_d;
				float speed_body_w = _R(0, 2) * _global_pos.vel_n + _R(1, 2) * _global_pos.vel_e + _R(2, 2) * _global_pos.vel_d;

				/* scale around tuning airspeed */
				float airspeed;

				/* if airspeed is not updating, we assume the normal average speed */
				if (bool nonfinite = !PX4_ISFINITE(_ctrl_state.airspeed) || !_ctrl_state.airspeed_valid) {
					airspeed = _parameters.airspeed_trim;

					//We assume that the airspeed is the waterspeed = body longitudinal velocity of the boat
					if(PX4_ISFINITE(speed_body_u))
					{
						airspeed = speed_body_u;
					}

					if (nonfinite) {
						perf_count(_nonfinite_input_perf);
					}

				} else {
					/* prevent numerical drama by requiring 0.5 m/s minimal speed */
					airspeed = math::max(0.5f, _ctrl_state.airspeed);
				}

				/*
				 * For scaling our actuators using anything less than the min (close to stall)
				 * speed doesn't make any sense - its the strongest reasonable deflection we
				 * want to do in flight and its the baseline a human pilot would choose.
				 *
				 * Forcing the scaling to this value allows reasonable handheld tests.
				 */
				float airspeed_scaling = _parameters.airspeed_trim / ((airspeed < _parameters.airspeed_min) ? _parameters.airspeed_min :
							 airspeed);

				/* Use min airspeed to calculate ground speed scaling region.
				 * Don't scale below gspd_scaling_trim
				 */
				float groundspeed = sqrtf(_global_pos.vel_n * _global_pos.vel_n +
							  _global_pos.vel_e * _global_pos.vel_e);
				float gspd_scaling_trim = (_parameters.airspeed_min * 0.6f);
				float groundspeed_scaler = gspd_scaling_trim / ((groundspeed < gspd_scaling_trim) ? gspd_scaling_trim : groundspeed);

				bool  roll_control_enable = false;
				float roll_sp = _parameters.rollsp_offset_rad;
				float roll_rate_sp = 0.0f;

				bool  pitch_control_enable = false;
				float pitch_sp = _parameters.pitchsp_offset_rad;
				float pitch_rate_sp = 0.0f;

				bool  yaw_control_enable = false;
				float yaw_sp = 0.0f;
				float yaw_rate_sp = 0.0f;

				bool  throttle_control_enable = false;
				float throttle_sp = 0.0f;

				bool  lift_control_enable = false;
				float lift_sp = 0.0f;

				/* Read attitude setpoint from uorb if
				 * - velocity control or position control is enabled (pos controller is running)
				 * - manual control is disabled (another app may send the setpoint, but it should
				 *   for sure not be set from the remote control values)
				 */


				if ((_vcontrol_mode.flag_control_auto_enabled ||
				    !_vcontrol_mode.flag_control_manual_enabled) && false) {
					/* read in attitude setpoint from attitude setpoint uorb topic */
					/* roll_sp = (_manual.y * _parameters.man_roll_max) + _parameters.rollsp_offset_rad;
					pitch_sp = _att_sp.pitch_body + _parameters.pitchsp_offset_rad;
					yaw_sp = _att_sp.yaw_body;
					throttle_sp = _att_sp.thrust; */

					/* Flag corresponding to the "TURNS" flight mode*/

					float bendRadius_sp = 200.0f; //initially turn radius equal to Rmax

					if (_manual.y>0) {
						bendRadius_sp = (15.0f + (200.0f-15.0f)*expf(-_manual.y*_manual.y/(0.4f*0.4f)));
					}

					if (_manual.y<0) {
						bendRadius_sp = -(15.0f + (200.0f-15.0f)*expf(-_manual.y*_manual.y/(0.4f*0.4f)));
					}

					roll_control_enable = true;
					roll_sp += atanf(bendRadius_sp*speed_body_u/9.81f);
					roll_rate_sp = 0.0f;

					pitch_control_enable = true; // pitch is controlled very smoothly by the backfoil (we do not want to control the pitch with flaps)
					pitch_sp += 0.0f;

					yaw_control_enable = true;
					yaw_rate_sp = speed_body_u/bendRadius_sp;



					/* reset integrals where needed */
					if (_att_sp.roll_reset_integral) {
						_roll_ctrl.reset_integrator();
					}

					if (_att_sp.pitch_reset_integral) {
						_pitch_ctrl.reset_integrator();
						_lift_ctrl.reset_integrator();
						_speed_ctrl.reset_integrator();
					}

					if (_att_sp.yaw_reset_integral) {
						_yaw_ctrl.reset_integrator();
					}

				} else if (_vcontrol_mode.flag_control_velocity_enabled) {

					throttle_control_enable = true;
					throttle_sp = _manual.z * _parameters.airspeed_max;

					/* reset integrals where needed */
					if (_att_sp.roll_reset_integral) {
						_roll_ctrl.reset_integrator();
					}

					if (_att_sp.pitch_reset_integral) {
						_pitch_ctrl.reset_integrator();
						_lift_ctrl.reset_integrator();
						_speed_ctrl.reset_integrator();
					}

					if (_att_sp.yaw_reset_integral) {
						_yaw_ctrl.reset_integrator();
					}

				} else if (_vcontrol_mode.flag_control_altitude_enabled) {
					/*
					 * Velocity should be controlled and manual is enabled.
					*/
					 /*
					throttle_control_enable = true;
					throttle_sp = _manual.z * _parameters.airspeed_max;
					*/

					roll_control_enable = true;
					roll_sp = rollFromYawRate(_manual.y * _parameters.man_yaw_rate_max, speed_body_u);

					yaw_control_enable = true;
					yaw_rate_sp = _manual.y * _parameters.man_yaw_rate_max;

					/* We use the pitch as an altitude sp*/
					pitch_control_enable = true;
					/* The maximum altitude is 30 cm*/
					pitch_sp = _manual.x * 0.30f;

					/* reset integrals where needed */
					if (_att_sp.roll_reset_integral) {
						_roll_ctrl.reset_integrator();
					}

					if (_att_sp.pitch_reset_integral) {
						_pitch_ctrl.reset_integrator();
						_lift_ctrl.reset_integrator();
						_speed_ctrl.reset_integrator();
					}

					if (_att_sp.yaw_reset_integral) {
						_yaw_ctrl.reset_integrator();
					}

				} else {

					/* Flag corresponding to the "STABILIZED" flight mode*/

					// Only activate the roll control, the boat is fully
					// manual, we just try to keep the roll to a specific angle.
					roll_control_enable = true;
					roll_sp = _manual.y * _parameters.man_roll_max;


                    if (_manual.aux3 > 0.5f) {
					    lift_control_enable = true;
                    } else {
					    lift_control_enable = false;
                    }

					lift_sp = (_manual.x + 1) * 0.25f;

					// roll_control_enable = true;
					// roll_sp = rollFromYawRate(_manual.y * _parameters.man_yaw_rate_max, speed_body_u);

					yaw_control_enable = true;
					yaw_rate_sp = _manual.y * _parameters.man_yaw_rate_max;

				}

				/* If the aircraft is on ground reset the integrators */
				// ADD : Add a case when the aircraft is stop -> condition on the speed or on the trottle
				if (_vehicle_land_detected.landed || _vehicle_status.is_rotary_wing) {
					_roll_ctrl.reset_integrator();
					_pitch_ctrl.reset_integrator();
					_lift_ctrl.reset_integrator();
					_speed_ctrl.reset_integrator();
					_yaw_ctrl.reset_integrator();
				}

				/* Prepare data for attitude controllers */
				struct ECL_ControlData control_input = {};
				control_input.roll = _roll;
				control_input.pitch = _pitch;
				control_input.yaw = _yaw;
				control_input.roll_rate = _ctrl_state.roll_rate;
				control_input.pitch_rate = _ctrl_state.pitch_rate;
				control_input.yaw_rate = _ctrl_state.yaw_rate;
				control_input.body_x = _local_pos.x;
				control_input.body_y = _local_pos.y;
				control_input.body_z = _local_pos.z;
				control_input.speed_body_u = speed_body_u;
				control_input.speed_body_v = speed_body_v;
				control_input.speed_body_w = speed_body_w;
				control_input.acc_body_x = _accel.x;
				control_input.acc_body_y = _accel.y;
				control_input.acc_body_z = _accel.z;
				control_input.roll_setpoint = roll_sp;
				control_input.pitch_setpoint = pitch_sp;
				control_input.yaw_setpoint = yaw_sp;
				control_input.lift_setpoint = lift_sp;
				control_input.speed_setpoint = throttle_sp;
				control_input.roll_rate_setpoint = roll_rate_sp;
				control_input.pitch_rate_setpoint = pitch_rate_sp;
				control_input.yaw_rate_setpoint = yaw_rate_sp;
				control_input.airspeed_min = _parameters.airspeed_min;
				control_input.airspeed_max = _parameters.airspeed_max;
				control_input.airspeed = airspeed;
				control_input.scaler = airspeed_scaling;
				//control_input.fly_waterspeed_min = ;
				//control_input..fly_waterspeed_max = ;
				control_input.lock_integrator = lock_integrator;
				control_input.groundspeed = groundspeed;
				control_input.groundspeed_scaler = groundspeed_scaler;
				//control_input.boat_state = ;
				control_input.palpeur_distance = _distance_sensor.current_distance;
				control_input.palpeur_valid = (hrt_elapsed_time(&_last_update_distance) < 10000);


				/* Run attitude controllers */
				if ( true ) {
					float pitch_u = 0.0f;
					float yaw_u = 0.0f;
					float roll_u = 0.0f;
					float throttle_u = 0.0f;
					float lift_u = 0.0f;

					/*
					*	Lift control
					*	This controller is used for the altitude control
					*/
					if(lift_control_enable)
					{
						_lift_ctrl.control_attitude(control_input);
						control_input.lift_rate_setpoint = _lift_ctrl.get_desired_rate();
						lift_u = _lift_ctrl.control_bodyrate(control_input);
						lift_u = (PX4_ISFINITE(lift_u)) ? lift_u + _parameters.pitchsp_offset_deg : 0.0f;
					}
					else
					{
						lift_u = -_manual.x + _parameters.pitchsp_offset_rad;
						_lift_ctrl.reset_integrator();
					}


					/*
					*	Roll control
					*	This controller is used for the normal stabilisation -> Really important !
					*/
					if(roll_control_enable)
					{
						_roll_ctrl.control_attitude(control_input);
						control_input.roll_rate_setpoint = _roll_ctrl.get_desired_rate();
						roll_u = _roll_ctrl.control_bodyrate(control_input);
						roll_u = (PX4_ISFINITE(roll_u)) ? roll_u : 0.0f;
					}
					else
					{
						roll_u = (_manual.y * _parameters.man_roll_max) + _parameters.rollsp_offset_rad;
						_roll_ctrl.reset_integrator();
					}


					/*
					*	Yaw control
					*	TODO :
					*	Add case for different bifoiler state mode
					*/
					if(yaw_control_enable)
					{
						/*
						// yaw controller only used for TURNS flight mode
						// we want to use the yaw_rate_sp defined above and not the one calculated by the yaw controller
						roll_u += _yaw_ctrl.control_bodyrate(control_input);
						roll_u = (PX4_ISFINITE(roll_u)) ? roll_u : 0.0f;
						// yaw input must be sent here to the ailerons so to the actuator group roll (that is why we sum roll_u to the previous one)
						*/

						yaw_u = _yaw_ctrl.control_bodyrate(control_input);
						yaw_u += _manual.r;
					}
					else
					{
						yaw_u = _manual.r;
						_yaw_ctrl.reset_integrator();
					}


					/*
					*	Speed control
					*	This is the controller for the main propulsion motor.
					*/
					if(throttle_control_enable)
					{
						_speed_ctrl.control_attitude(control_input);
						control_input.speed_rate_setpoint = _speed_ctrl.get_desired_rate();
						throttle_u = _speed_ctrl.control_bodyrate(control_input);
						throttle_u = (PX4_ISFINITE(throttle_u)) ? throttle_u : 0.0f;
					}
					else
					{
						_speed_ctrl.reset_integrator();
						if(_manual.aux4 >= 0.0f){
							if( _manual.aux2 >= 0.0f )
								throttle_u = _manual.z;
							else
								throttle_u = -_manual.z;
						}
						else{
							throttle_u = 0;
						}
					}


					/*
					*	Pitch Control
					*	This controler regul the pitch with the back foil
					*/
					if(pitch_control_enable)
					{
						// pitch is controlled very smoothly by the backfoil (we do not want to control the pitch with flaps)
						_pitch_ctrl.control_attitude(control_input);
						control_input.pitch_rate_setpoint = _pitch_ctrl.get_desired_rate();
						pitch_u = _pitch_ctrl.control_bodyrate(control_input);
						pitch_u = (PX4_ISFINITE(pitch_u)) ? pitch_u : 0.0f;

					}
					else
					{
						_pitch_ctrl.reset_integrator();
						pitch_u = _manual.aux1;
					}

					/* Updating the control inputs on the actuator groups */
					_actuators.control[actuator_controls_s::INDEX_ROLL] = roll_u;
					_actuators.control[actuator_controls_s::INDEX_PITCH] = lift_u;
					_actuators.control[actuator_controls_s::INDEX_YAW] = yaw_u;
					_actuators.control[3] = (PX4_ISFINITE(throttle_u) &&
								 !(_vehicle_status.engine_failure ||
								   _vehicle_status.engine_failure_cmd)) ?
								throttle_u : 0.0f;

					_actuators_alternate.control[4] = pitch_u;


					if (!PX4_ISFINITE(yaw_u)) {
						_yaw_ctrl.reset_integrator();
						perf_count(_nonfinite_output_perf);

						if (_debug && loop_counter % 10 == 0) {
							warnx("yaw_u %.4f", (double)yaw_u);
						}
					}

				} else {
					perf_count(_nonfinite_input_perf);

					if (_debug && loop_counter % 10 == 0) {
						warnx("Non-finite setpoint roll_sp: %.4f, pitch_sp %.4f", (double)roll_sp, (double)pitch_sp);
					}
				}

				/*
				 * Lazily publish the rate setpoint (for analysis, the actuators are published below)
				 * only once available
				 */
				_rates_sp.roll = _roll_ctrl.get_desired_rate();
				_rates_sp.pitch = _pitch_ctrl.get_desired_rate();
				_rates_sp.yaw = _lift_ctrl.get_desired_rate();

				_rates_sp.timestamp = hrt_absolute_time();

				if (_rate_sp_pub != nullptr) {
					/* publish the attitude rates setpoint */
					orb_publish(_rates_sp_id, _rate_sp_pub, &_rates_sp);

				} else if (_rates_sp_id) {
					/* advertise the attitude rates setpoint */
					_rate_sp_pub = orb_advertise(_rates_sp_id, &_rates_sp);
				}

			} else {
				/* manual/direct control */
				_actuators.control[actuator_controls_s::INDEX_ROLL] = (_manual.y + _parameters.trim_roll);
				_actuators.control[actuator_controls_s::INDEX_PITCH] = -_manual.x + _parameters.trim_pitch;
				_actuators.control[actuator_controls_s::INDEX_YAW] = _manual.r + _parameters.trim_yaw;
				/* Check if user want to go backward (AUX 2 switch) */
				if(_manual.aux4 >= 0.0f){
					if( _manual.aux2 >= 0.0f )
						_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _manual.z;
					else
						_actuators.control[actuator_controls_s::INDEX_THROTTLE] = -_manual.z;
				}
				else{
					_actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0;
				}


				//printf("Actuator = %f \n", (double) _actuators.control[actuator_controls_s::INDEX_THROTTLE] * 100.);
				_actuators_alternate.control[4] = _manual.aux1;

				_roll_ctrl.reset_integrator();
			}

			/* lazily publish the setpoint only once available */
			_actuators.timestamp = hrt_absolute_time();
			_actuators.timestamp_sample = _ctrl_state.timestamp;
			_actuators_airframe.timestamp = hrt_absolute_time();
			_actuators_airframe.timestamp_sample = _ctrl_state.timestamp;

			/* Only publish if any of the proper modes are enabled */
			if (_vcontrol_mode.flag_control_rates_enabled ||
			    _vcontrol_mode.flag_control_attitude_enabled ||
			    _vcontrol_mode.flag_control_manual_enabled) {
				/* publish the actuator controls */
				if (_actuators_0_pub != nullptr) {
					orb_publish(_actuators_id, _actuators_0_pub, &_actuators);

				} else if (_actuators_id) {
					_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
				}

				if (_actuators_2_pub != nullptr) {
					/* publish the actuator controls*/
					orb_publish(ORB_ID(actuator_controls_2), _actuators_2_pub, &_actuators_airframe);

				} else {
					/* advertise and publish */
					_actuators_2_pub = orb_advertise(ORB_ID(actuator_controls_2), &_actuators_airframe);
				}


				if (_actuators_1_pub != nullptr) {
					/* publish the actuator controls*/
					orb_publish(ORB_ID(actuator_controls_1), _actuators_1_pub, &_actuators_alternate);

				} else {
					/* advertise and publish */
					_actuators_1_pub = orb_advertise(ORB_ID(actuator_controls_1), &_actuators_alternate);
				}
			}
		}

		loop_counter++;
		perf_end(_loop_perf);
	}

	warnx("exiting.\n");

	_control_task = -1;
	_task_running = false;
}

float FixedwingAttitudeControl::rollFromYawRate(float yaw_rate_ref, float boat_speed)
{
	float value = 0.0f;
	value += yaw_rate_ref * _parameters.roll_yaw_rate_scaling;
	return math::constrain(value, -_parameters.man_roll_max, _parameters.man_roll_max);
}

int
FixedwingAttitudeControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("fw_att_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1300,
					   (px4_main_t)&FixedwingAttitudeControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int fw_att_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: fw_att_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (att_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		att_control::g_control = new FixedwingAttitudeControl;

		if (att_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != att_control::g_control->start()) {
			delete att_control::g_control;
			att_control::g_control = nullptr;
			warn("start failed");
			return 1;
		}

		/* check if the waiting is necessary at all */
		if (att_control::g_control == nullptr || !att_control::g_control->task_running()) {

			/* avoid memory fragmentation by not exiting start handler until the task has fully started */
			while (att_control::g_control == nullptr || !att_control::g_control->task_running()) {
				usleep(50000);
				printf(".");
				fflush(stdout);
			}

			printf("\n");
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (att_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete att_control::g_control;
		att_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (att_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
