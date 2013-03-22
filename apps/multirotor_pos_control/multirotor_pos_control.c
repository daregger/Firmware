/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @file multirotor_pos_control.c
 *
 * Skeleton for multirotor position controller
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <float.h>
#include <errno.h>
#include <debug.h>
#include <termios.h>
#include <time.h>
#include <math.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_global_position_setpoint.h>
#include <uORB/topics/vehicle_vicon_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/debug_key_value.h>
#include <systemlib/geo/geo.h>

#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>
#include <poll.h>

#include "multirotor_pos_control_params.h"
#include <mavlink/mavlink_log.h>

static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */

__EXPORT int multirotor_pos_control_main(int argc, char *argv[]);

/**
 * Mainloop of position controller.
 */
static int multirotor_pos_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: deamon {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 * 
 * The actual stack size should be set in the call
 * to task_spawn().
 */
int multirotor_pos_control_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("multirotor pos control already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn("multirotor pos control",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 30,
					 4096,
					 multirotor_pos_control_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tmultirotor pos control app is running\n");
		} else {
			printf("\tmultirotor pos control app not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

static int
multirotor_pos_control_thread_main(int argc, char *argv[]){
	/* welcome user */
	printf("[multirotor pos control] Control started, taking over position control\n");
	int mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	mavlink_log_info(mavlink_fd, "[multirotor pos control] Control started, taking over position control\n");

	/* structures */
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_global_position_setpoint_s global_pos_sp;
	memset(&global_pos_sp, 0, sizeof(global_pos_sp));
	struct vehicle_local_position_setpoint_s local_pos_sp;
	memset(&local_pos_sp, 0, sizeof(local_pos_sp));
	struct vehicle_local_position_s local_pos_est;
	memset(&local_pos_est, 0, sizeof(local_pos_est));
	struct vehicle_vicon_position_s vicon_pos;
	memset(&vicon_pos, 0, sizeof(vicon_pos));
	struct manual_control_setpoint_s manual;
	memset(&manual, 0, sizeof(manual));
	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));
	struct vehicle_status_s vehicle_status;
	memset(&vehicle_status, 0, sizeof(vehicle_status));
	struct sensor_combined_s sensors;
	memset(&sensors, 0, sizeof(sensors));
	struct vehicle_gps_position_s gps;
	memset(&gps, 0, sizeof(gps));

	/* subscribe */
	int sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	int sub_params = orb_subscribe(ORB_ID(parameter_update));
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int vicon_pos_sub = orb_subscribe(ORB_ID(vehicle_vicon_position));
	int global_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_global_position_setpoint));
	int local_pos_est_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	int vehicle_gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	//int local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));

	/* publish attitude setpoint */
	orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);
	orb_advert_t local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &local_pos_sp);
	//orb_advert_t global_pos_sp_pub = orb_advertise(ORB_ID(vehicle_global_position_setpoint), &global_pos_sp);

	static float z[3] = {0, 0, 0}; /* output variables from tangent plane mapping */
	static float rotMatrix[4] = {1.0f,  0.0f, 0.0f,  1.0f};
	static float pos_ctrl_gain_p = 0.8f;
	static float pos_ctrl_gain_d = 0.8f;
	static float pos_ctrl_gain_i = 0.0f;
	static float pos_ctrl_integral_x = 0.0f;
	static float pos_ctrl_integral_y = 0.0f;
	static float pos_ctrl_antiwindup = 0.05f;
	static float z_ctrl_gain_p = 0.8f;
	static float z_ctrl_gain_d = 0.6f;
	static float z_ctrl_gain_i = 0.0f;
	static float z_ctrl_integral = 0.0f;
	static float z_ctrl_antiwindup = 0.05f;
	static float z_ctrl_thrust_feedforward = 0.65f; /* approx. hoovering thrust for 1800mAh Akku*/
	const float pitch_limit = 0.28f;
	const float roll_limit = 0.28f;
	const float thrust_limit_upper = 0.5f;
	const float thrust_limit_lower = 0.3f;
	static float local_pos_sp_x = 0.0f;
	static float local_pos_sp_y = 0.0f;
	static float local_pos_sp_z = -0.8f;
	static float local_pos_sp_x_old = 0.0f;
	static float local_pos_sp_y_old = 0.0f;
	static float global_pos_sp_lat = 47.375711f; /* degree lat */
	static float global_pos_sp_lon = 8.550134; /* degree lon */
	static float global_pos_sp_alt = 520.0f; /* m AMSL */
	static float y_pos_err_earth = 0.0f;
	static float x_pos_err_earth = 0.0f;
	static const float y_vel_setpoint = 0.0f;
	static const float x_vel_setpoint = 0.0f;
	static float y_vel_err_earth = 0.0f;
	static float x_vel_err_earth = 0.0f;
	static bool local_flag_vel_limit_enable;
	static float vel_limit_gain_xy = 0.0f;
	static float vel_limit_gain_xy_threshold = 0.0f;
	static float vel_limit_gain_z = 0.0f;
	static float vel_limit_gain_z_threshold = 0.0f;
	static float local_pos_sp_y_target = 0.0f;
	static float local_pos_sp_x_target = 0.0f;
	static float local_pos_sp_z_target = 0.0f;
	static bool local_flag_useGPS = false;
	static bool local_flag_Global_setpoint_received_once = false;

	static double lat_current = 0.0d; //[°]] --> 47.0
	static double lon_current = 0.0d; //[°]] -->8.5
	static double alt_current = 0.0d; //[m] above MSL

	static int printcounter = 0;

	struct multirotor_position_control_params pos_params;
	struct multirotor_position_control_param_handles handle_pos_params;
	parameters_init(&handle_pos_params);
	parameters_update(&handle_pos_params, &pos_params);
	/* First parameter read at start up */
	pos_ctrl_gain_d = pos_params.pos_d;
	pos_ctrl_gain_p = pos_params.pos_p;
	pos_ctrl_gain_i = pos_params.pos_i;
	pos_ctrl_antiwindup = pos_params.pos_i_antiwindup;
	z_ctrl_gain_p = pos_params.height_p;
	z_ctrl_gain_d = pos_params.height_d;
	z_ctrl_gain_i = pos_params.height_i;
	z_ctrl_antiwindup = pos_params.height_i_antiwindup;
	z_ctrl_thrust_feedforward = pos_params.height_ff;
	local_pos_sp_x_target = pos_params.loc_sp_x;
	local_pos_sp_y_target = pos_params.loc_sp_y;
	local_pos_sp_z_target = pos_params.loc_sp_z;
	local_pos_sp_x_old = pos_params.loc_sp_x;
	local_pos_sp_y_old = pos_params.loc_sp_y;
	global_pos_sp_lat = pos_params.glo_sp_lat;
	global_pos_sp_lon = pos_params.glo_sp_lon;
	global_pos_sp_alt = pos_params.glo_sp_alt;
	local_flag_vel_limit_enable = ((pos_params.vel_limit_enabled >= 0.9f) && (pos_params.vel_limit_enabled <= 1.1f));
	vel_limit_gain_xy = pos_params.vel_limit_gain_xy;
	vel_limit_gain_xy_threshold = pos_params.vel_limit_gain_xy_threshold;
	vel_limit_gain_z = pos_params.vel_limit_gain_z;
	vel_limit_gain_z_threshold = pos_params.vel_limit_gain_z_threshold;
	local_flag_useGPS = ((pos_params.useGPS >= 0.9f) && (pos_params.useGPS <= 1.1f));
	/* END First parameter read at start up */
	/* only publish local_sp, not for controller use */
	local_pos_sp.x = local_pos_sp_x_target;
	local_pos_sp.y = local_pos_sp_y_target;
	local_pos_sp.z = local_pos_sp_z;
	local_pos_sp.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(vehicle_local_position_setpoint), local_pos_sp_pub, &local_pos_sp);
	/* only publish global_sp, not for controller use
	global_pos_sp.lat = (int32_t)(global_pos_sp_lat * 1E7);
	global_pos_sp.lon = (int32_t)(global_pos_sp_lon * 1E7);
	global_pos_sp.altitude = global_pos_sp_alt;
	global_pos_sp.timestamp = hrt_absolute_time();
	orb_publish(ORB_ID(vehicle_global_position_setpoint), global_pos_sp_pub, &global_pos_sp);*/

	if(local_flag_useGPS){
		mavlink_log_info(mavlink_fd, "[posCTRL] I'm using GPS");
		/* wait until gps signal turns valid, only then can we initialize the projection */
		while (gps.fix_type < 3) {
			struct pollfd fds1[2] = {
					{ .fd = vehicle_gps_sub, .events = POLLIN },
					{ .fd = sub_params,   .events = POLLIN },
			};

			/* wait for GPS updates, BUT READ VEHICLE STATUS (!)
			 * this choice is critical, since the vehicle status might not
			 * actually change, if this app is started after GPS lock was
			 * aquired.
			 */
			if (poll(fds1, 2, 5000)) {
				if (fds1[0].revents & POLLIN){
					/* Wait for the GPS update to propagate (we have some time) */
					usleep(5000);
					/* Read wether the vehicle status changed */
					orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_sub, &gps);
				}
				if (fds1[1].revents & POLLIN){
					/* Read out parameters to check for an update there, e.g. useGPS variable */
					/* read from param to clear updated flag */
					struct parameter_update_s update;
					orb_copy(ORB_ID(parameter_update), sub_params, &update);
					/* update parameters */
					parameters_update(&handle_pos_params, &pos_params);
					if(!((pos_params.useGPS >= 0.9f) && (pos_params.useGPS <= 1.1f))){
						local_flag_useGPS = false;
						mavlink_log_info(mavlink_fd, "[posCTRL] Revert GPS - Goingt to VICON");
						break; /* leave gps fix type 3 while loop */
					}
				}
			}
			static int printcounter = 0;
			if (printcounter == 100) {
				printcounter = 0;
				printf("[posCTRL] wait for GPS fix type 3\n");
			}
			printcounter++;
		}

		/* check again if useGPS was not aborted and only if not set up tangent plane map initialization*/
		if(local_flag_useGPS){
			/* get gps value for first initialization */
			orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_sub, &gps);
			lat_current = ((double)(gps.lat)) * 1e-7;
			lon_current = ((double)(gps.lon)) * 1e-7;
			alt_current = gps.alt * 1e-3;
			/* initialize coordinates */
			map_projection_init(lat_current, lon_current);
			/* publish global position messages only after first GPS message */
			printf("[posCTRL] initialized projection with: lat: %.10f,  lon:%.10f\n", lat_current, lon_current);
		}
	}else{
		mavlink_log_info(mavlink_fd, "[posCTRL] I'm NOT using GPS - I use VICON");
		/* onboard calculated position estimations */
	}

	perf_counter_t interval_perf = perf_alloc(PC_INTERVAL, "multirotor_pos_control_interval");
	perf_counter_t mc_err_perf = perf_alloc(PC_COUNT, "multirotor_pos_control_err");
	struct pollfd fds2[3] = {
					{ .fd = sensor_sub, .events = POLLIN }, //ca. 130 Hz
					{ .fd = sub_params,   .events = POLLIN },
					{ .fd = global_pos_sp_sub,   .events = POLLIN },
				};

	thread_running = true;
	uint64_t last_time = 0;

	while (!thread_should_exit) {
		/* wait for a sensor update update, check for exit condition every 2000 ms */
		int ret = poll(fds2, 3, 2000);
		if (ret < 0) {
			/* poll error, count it in perf */
			perf_count(mc_err_perf);
		} else if (ret == 0) {
			/* no return value, ignore */
		} else {
			if (fds2[1].revents & POLLIN){
				/* new parameter */
				/* read from param to clear updated flag */
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), sub_params, &update);
				/* update parameters */
				parameters_update(&handle_pos_params, &pos_params);
				pos_ctrl_gain_d = pos_params.pos_d;
				pos_ctrl_gain_p = pos_params.pos_p;
				pos_ctrl_gain_i = pos_params.pos_i;
				pos_ctrl_antiwindup = pos_params.pos_i_antiwindup;
				z_ctrl_gain_p = pos_params.height_p;
				z_ctrl_gain_d = pos_params.height_d;
				z_ctrl_gain_i = pos_params.height_i;
				z_ctrl_antiwindup = pos_params.height_i_antiwindup;
				z_ctrl_thrust_feedforward = pos_params.height_ff;
				/* write local_pos_sp and limit them
				 * only overwrite the values that changed from the qgc parameter setting,
				 * not the ones from the rc setpoint movement */
				float manual_setpoint_limit = 200;
				if(pos_params.loc_sp_x != local_pos_sp_x_old){
					if((pos_params.loc_sp_x < manual_setpoint_limit) && (pos_params.loc_sp_x > -manual_setpoint_limit)){
						local_pos_sp_x_target = pos_params.loc_sp_x;
						local_pos_sp_x_old = pos_params.loc_sp_x;
					}
				}
				if(pos_params.loc_sp_y != local_pos_sp_y_old){
					if((pos_params.loc_sp_y < manual_setpoint_limit) && (pos_params.loc_sp_y > -manual_setpoint_limit)){
						local_pos_sp_y_target = pos_params.loc_sp_y;
						local_pos_sp_y_old = pos_params.loc_sp_y;
					}
				}
				/* z can be always written new, it cannot be altered by any RC channel */
				if((pos_params.loc_sp_z < 0.0f) && (pos_params.loc_sp_z > -manual_setpoint_limit)){
					local_pos_sp_z_target = pos_params.loc_sp_z;
				}
				local_flag_vel_limit_enable = ((pos_params.vel_limit_enabled >= 0.9f) && (pos_params.vel_limit_enabled <= 1.1f));
				vel_limit_gain_xy = pos_params.vel_limit_gain_xy;
				vel_limit_gain_xy_threshold = pos_params.vel_limit_gain_xy_threshold;
				vel_limit_gain_z = pos_params.vel_limit_gain_z;
				vel_limit_gain_z_threshold = pos_params.vel_limit_gain_z_threshold;
				global_pos_sp_lat = pos_params.glo_sp_lat;
				global_pos_sp_lon = pos_params.glo_sp_lon;
				global_pos_sp_alt = pos_params.glo_sp_alt;
				//printf("[posCTRL] vel_limit_local: %8.4f\n", (double)(vel_limit_local));
				local_flag_useGPS = ((pos_params.useGPS >= 0.9f) && (pos_params.useGPS <= 1.1f));
				//printf("[posCTRL] local_flag_useGPS %s", local_flag_useGPS ? "true" : "false");
			}
			if (fds2[2].revents & POLLIN) {
				local_flag_Global_setpoint_received_once = true;
				/* new global position setpoint, comming from waypoint handler */
				orb_copy(ORB_ID(vehicle_global_position_setpoint), global_pos_sp_sub, &global_pos_sp);
				/* map waypoint into local tangent plane */
				map_projection_project(((double)(global_pos_sp.lat)) * 1e-7f, ((double)(global_pos_sp.lon)) * 1e-7f, &(z[0]), &(z[1]));
				/* set the new targets */
				local_pos_sp_x_target = z[0];
				local_pos_sp_y_target = z[1];
			}
			if (fds2[0].revents & POLLIN) {
				/* new sensor value */
				//float dT = (hrt_absolute_time() - last_time) / 1000000.0f;
				//last_time = hrt_absolute_time();
				/*if (printcounter >= 400) {
					printcounter = 0;
					if(local_flag_Global_setpoint_received_once){
						printf("[posCTRL] received_once\n");
					}else{
						printf("[posCTRL] NEVER received\n");
					}

				}
				printcounter++;
				*/

				orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &manual);
				orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
				orb_copy(ORB_ID(vehicle_local_position), local_pos_est_sub, &local_pos_est);
				orb_copy(ORB_ID(vehicle_vicon_position), vicon_pos_sub, &vicon_pos);
				orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vehicle_status);
				orb_copy(ORB_ID(sensor_combined), sensor_sub, &sensors);

				/* if speed limit enabled then move around local setpoint with speed defined by vel_limit_gain_xy
				* as long as the setpoint is not in the target zone (size defined by vel_limit_gain_xy_threshold) */
				if(local_flag_vel_limit_enable){
					if(fabs(local_pos_sp_y_target-local_pos_sp_y)>vel_limit_gain_xy_threshold){
						if(local_pos_sp_y_target>local_pos_sp_y){
							local_pos_sp_y += ((pos_params.vel_limit_gain_xy)/260.0f);
						}else{
							local_pos_sp_y -= ((pos_params.vel_limit_gain_xy)/260.0f);
						}
					}
					if(fabs(local_pos_sp_x_target-local_pos_sp_x)>vel_limit_gain_xy_threshold){
						if(local_pos_sp_x_target>local_pos_sp_x){
							local_pos_sp_x += ((pos_params.vel_limit_gain_xy)/260.0f);
						}else{
							local_pos_sp_x -= ((pos_params.vel_limit_gain_xy)/260.0f);
						}
					}
					if(fabs(local_pos_sp_z_target-local_pos_sp_z)>vel_limit_gain_z_threshold){
						if(local_pos_sp_z_target>local_pos_sp_z){
							local_pos_sp_z += ((pos_params.vel_limit_gain_z)/260.0f);
						}else{
							local_pos_sp_z -= ((pos_params.vel_limit_gain_z)/260.0f);
						}
					}
				}else{
					local_pos_sp_y = local_pos_sp_y_target;
					local_pos_sp_x = local_pos_sp_x_target;
					local_pos_sp_z = local_pos_sp_z_target;
				}

				if (vehicle_status.state_machine == SYSTEM_STATE_AUTO) {
					/* move around local position setpoint with RC roll/pitch stick when in AUTO State */
					if(fabs(manual.pitch) > pos_params.sp_gain_xy_threshold){
						local_pos_sp_x_target += -manual.pitch*pos_params.sp_gain_xy;
					}
					if(fabs(manual.roll) > pos_params.sp_gain_xy_threshold){
						local_pos_sp_y_target += manual.roll*pos_params.sp_gain_xy;
					}

					/* ROLL & PITCH CONTROLLER */
					y_pos_err_earth = -(local_pos_est.y - local_pos_sp_y);
					x_pos_err_earth = (local_pos_est.x - local_pos_sp_x);
					y_vel_err_earth = -(local_pos_est.vy - y_vel_setpoint);
					x_vel_err_earth = (local_pos_est.vx - x_vel_setpoint);

					/* limit xy intergrator */
					if(fabs(pos_ctrl_integral_x)<pos_ctrl_antiwindup){
						pos_ctrl_integral_x += x_pos_err_earth;
					}
					if(fabs(pos_ctrl_integral_y)<pos_ctrl_antiwindup){
						pos_ctrl_integral_y += y_pos_err_earth;
					}

					/* rotMatrix is from body to earth */
					if(local_flag_useGPS){
						rotMatrix[0] = cos(att.yaw);
						rotMatrix[1] = -sin(att.yaw);
						rotMatrix[2] = sin(att.yaw);
						rotMatrix[3] = cos(att.yaw);
					}else{
						rotMatrix[0] = cos(vicon_pos.yaw);
						rotMatrix[1] = -sin(vicon_pos.yaw);
						rotMatrix[2] = sin(vicon_pos.yaw);
						rotMatrix[3] = cos(vicon_pos.yaw);
					}
					/* PID controller in earth frame, different sign because of Transformation from earth to body frame */
					float rollpos = (rotMatrix[0]*y_pos_err_earth-rotMatrix[1]*x_pos_err_earth)*pos_ctrl_gain_p;
					float rollvel = (rotMatrix[0]*y_vel_err_earth-rotMatrix[1]*x_vel_err_earth)*pos_ctrl_gain_d;
					float rollint = (rotMatrix[0]*pos_ctrl_integral_y-rotMatrix[1]*pos_ctrl_integral_x)*pos_ctrl_gain_i;
					float pitchpos = (-rotMatrix[2]*y_pos_err_earth+rotMatrix[3]*x_pos_err_earth)*pos_ctrl_gain_p;
					float pitchvel = (-rotMatrix[2]*y_vel_err_earth+rotMatrix[3]*x_vel_err_earth)*pos_ctrl_gain_d;
					float pitchint = (-rotMatrix[2]*pos_ctrl_integral_y+rotMatrix[3]*pos_ctrl_integral_x)*pos_ctrl_gain_i;;
					float rolltot = rollpos + rollvel + rollint;
					float pitchtot = pitchpos + pitchvel + pitchint;

					/* limit setpoints to maximal the values of the manual flight */
					if((rolltot <= roll_limit) && (rolltot >= -roll_limit)){
						att_sp.roll_body = rolltot;
					}else{
						if(rolltot > roll_limit){
							att_sp.roll_body = roll_limit;
						}
						if(rolltot < -roll_limit){
							att_sp.roll_body = -roll_limit;
						}
					}
					if((pitchtot <= pitch_limit) && (pitchtot >= -pitch_limit)){
						att_sp.pitch_body = pitchtot;
					}else{
						if(pitchtot > pitch_limit){
							att_sp.pitch_body = pitch_limit;
						}
						if(pitchtot < -pitch_limit){
							att_sp.pitch_body = -pitch_limit;
						}
					}

					/* YAW REGLER */
					/*if ((manual.yaw < -0.01f || 0.01f < manual.yaw) && manual.throttle > 0.3f) {
						att_sp.yaw_body = att_sp.yaw_body + manual.yaw * 0.0025f;
						} else if (manual.throttle <= 0.3f) {
						att_sp.yaw_body = att.yaw;
					}*/
					att_sp.yaw_body = 0.0f;

					/* Z CONTROLLER, PID with Feedforward */
					float z_vel_setpoint = 0.0f;
					float z_pos_err_earth = (local_pos_est.z - local_pos_sp_z);
					if(fabs(z_ctrl_integral)<z_ctrl_antiwindup){
						z_ctrl_integral += z_pos_err_earth;
					}
					float z_vel_err_earth = (local_pos_est.vz - z_vel_setpoint);
					float z_ctrl_thrust_err = z_pos_err_earth*z_ctrl_gain_p + z_vel_err_earth*z_ctrl_gain_d + z_ctrl_integral*z_ctrl_gain_i;
					float z_ctrl_thrust = z_ctrl_thrust_feedforward + z_ctrl_thrust_err;
					/* the throttle stick on the rc control limits the maximum thrust */
					float thrust_limit_upper = manual.throttle;
					if (z_ctrl_thrust >= thrust_limit_upper){
						z_ctrl_thrust = thrust_limit_upper;
					/* never go too low with the thrust, quadrotor may become uncontrollable */
					}else if(z_ctrl_thrust < thrust_limit_lower){
						z_ctrl_thrust = thrust_limit_lower;
					}
					/* check for finite thrust from the controller, otherwise apply hoovering thrust */
					if(isfinite(z_ctrl_thrust)){
						att_sp.thrust = z_ctrl_thrust;
					}else{
						att_sp.thrust = z_ctrl_thrust_feedforward;
					}
					att_sp.timestamp = hrt_absolute_time();

					/* publish global position setpoint */
					/*global_pos_sp.lat = (int32_t)(global_pos_sp_lat * 1E7);
					global_pos_sp.lon = (int32_t)(global_pos_sp_lon * 1E7);
					global_pos_sp.altitude = global_pos_sp_alt;
					global_pos_sp.timestamp = hrt_absolute_time();
					if((isfinite(global_pos_sp_lat)) && (isfinite(global_pos_sp_lon)) && (isfinite(global_pos_sp_alt))){
						orb_publish(ORB_ID(vehicle_global_position_setpoint), global_pos_sp_pub, &global_pos_sp);
					}*/


					//OVERRIDE CONTROLLER if desired
					//att_sp.roll_body = manual.roll;
					//att_sp.pitch_body = manual.pitch;
					//att_sp.thrust =  manual.throttle;
					//END OVERRIDE CONTROLLER

					/* publish new attitude setpoint */
					orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);
					/* measure in what intervals the controller runs */
					perf_count(interval_perf);
				} else {
					/* manual control */
				}
				/* publish local position setpoint */
				local_pos_sp.x = local_pos_sp_x;
				local_pos_sp.y = local_pos_sp_y;
				local_pos_sp.z = local_pos_sp_z;
				local_pos_sp.timestamp = hrt_absolute_time();
				if((isfinite(local_pos_sp.x)) && (isfinite(local_pos_sp.y)) && (isfinite(local_pos_sp.z))){
					orb_publish(ORB_ID(vehicle_local_position_setpoint), local_pos_sp_pub, &local_pos_sp);
				}
			} /* end of poll call for sensor updates */
		} /* end of poll return value check */
	}
	//mavlink_log_info(mavlink_fd, "[multirotor pos control] ending now...\n");
	thread_running = false;
	fflush(stdout);
	return 0;
}

