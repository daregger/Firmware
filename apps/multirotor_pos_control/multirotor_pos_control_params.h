/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Tobias Naegeli <naegelit@student.ethz.ch>
 *           Lorenz Meier <lm@inf.ethz.ch>
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

/*
 * @file multirotor_position_control_params.h
 * 
 * Parameters for position controller
 */

#include <systemlib/param/param.h>

struct multirotor_position_control_params {
	float pos_p;
	float pos_d;
	float height_p;
	float height_d;
	float height_i;
	float height_ff;
	float loc_sp_x;
	float loc_sp_y;
	float loc_sp_z;
	float glo_sp_lat;
	float glo_sp_lon;
	float glo_sp_alt;
	float sp_gain_xy;
	float sp_gain_xy_threshold;
	float vel_limit_enabled;
	float vel_limit_gain_xy;
	float vel_limit_gain_xy_threshold;
	float vel_limit_gain_z;
	float vel_limit_gain_z_threshold;
	float useGPS;
};

struct multirotor_position_control_param_handles {
	param_t pos_p;
	param_t pos_d;
	param_t height_p;
	param_t height_d;
	param_t height_i_param_handle;
	param_t height_ff;
	param_t loc_sp_x_param_handle;
	param_t loc_sp_y_param_handle;
	param_t loc_sp_z_param_handle;
	param_t glo_sp_lat_param_handle;
	param_t glo_sp_lon_param_handle;
	param_t glo_sp_alt_param_handle;
	param_t sp_gain_xy_param_handle;
	param_t sp_gain_xy_threshold_param_handle;
	param_t vel_limit_enabled_param_handle;
	param_t vel_limit_gain_xy_param_handle;
	param_t vel_limit_gain_xy_threshold_param_handle;
	param_t vel_limit_gain_z_param_handle;
	param_t vel_limit_gain_z_threshold_param_handle;
	param_t useGPS_param_handle;
};

/**
 * Initialize all parameter handles and values
 *
 */
int parameters_init(struct multirotor_position_control_param_handles *h);

/**
 * Update all parameters
 *
 */
int parameters_update(const struct multirotor_position_control_param_handles *h, struct multirotor_position_control_params *p);
