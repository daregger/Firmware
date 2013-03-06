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
 * @file multirotor_position_control_params.c
 * 
 * Parameters for EKF filter
 */

#include "multirotor_pos_control_params.h"

/* Extended Kalman Filter covariances */

/* controller parameters */
PARAM_DEFINE_FLOAT(POS_XY_P, 0.8f);
PARAM_DEFINE_FLOAT(POS_XY_D, 0.8f);
PARAM_DEFINE_FLOAT(POS_Z_P, 0.8f);
PARAM_DEFINE_FLOAT(POS_Z_D, 0.8f);
PARAM_DEFINE_FLOAT(POS_Z_I, 0.0f);
PARAM_DEFINE_FLOAT(POS_Z_ff, 0.65f);
PARAM_DEFINE_FLOAT(POS_loc_sp_x, 0.0f);
PARAM_DEFINE_FLOAT(POS_loc_sp_y, 0.0f);
PARAM_DEFINE_FLOAT(POS_loc_sp_z, -0.8f);
PARAM_DEFINE_FLOAT(POS_glo_sp_lat, 47.375711f);
PARAM_DEFINE_FLOAT(POS_glo_sp_lon, 8.550134f);
PARAM_DEFINE_FLOAT(POS_glo_sp_alt, 520.0f);
PARAM_DEFINE_FLOAT(POS_sp_gain_xy, 0.0f);
PARAM_DEFINE_FLOAT(POS_sp_gain_xy_t, 0.1f);
PARAM_DEFINE_FLOAT(POS_vel_lim_en, 1.0f);
PARAM_DEFINE_FLOAT(POS_vel_lim_xyG, 1.0f);
PARAM_DEFINE_FLOAT(POS_vel_lim_xyT, 1.0f);
PARAM_DEFINE_FLOAT(POS_vel_lim_zG, 1.0f);
PARAM_DEFINE_FLOAT(POS_vel_lim_zT, 1.0f);
PARAM_DEFINE_FLOAT(POS_useGPS, 1.0f);

int parameters_init(struct multirotor_position_control_param_handles *h)
{
	/* PID parameters */
	h->pos_p 	=	param_find("POS_XY_P");
	h->pos_d 	=	param_find("POS_XY_D");
	h->height_p =	param_find("POS_Z_P");
	h->height_d =	param_find("POS_Z_D");
	h->height_i_param_handle =	param_find("POS_Z_I");
	h->height_ff =	param_find("POS_Z_ff");
	h->loc_sp_x_param_handle = param_find("POS_loc_sp_x");
	h->loc_sp_y_param_handle = param_find("POS_loc_sp_y");
	h->loc_sp_z_param_handle = param_find("POS_loc_sp_z");
	h->glo_sp_lat_param_handle = param_find("POS_glo_sp_lat");
	h->glo_sp_lon_param_handle = param_find("POS_glo_sp_lon");
	h->glo_sp_alt_param_handle = param_find("POS_glo_sp_alt");
	h->sp_gain_xy_param_handle = param_find("POS_sp_gain_xy");
	h->sp_gain_xy_threshold_param_handle = param_find("POS_sp_gain_xy_t");
	h->vel_limit_enabled_param_handle = param_find("POS_vel_lim_en");
	h->vel_limit_gain_xy_param_handle = param_find("POS_vel_lim_xyG");
	h->vel_limit_gain_xy_threshold_param_handle = param_find("POS_vel_lim_xyT");
	h->vel_limit_gain_z_param_handle = param_find("POS_vel_lim_zG");
	h->vel_limit_gain_z_threshold_param_handle = param_find("POS_vel_lim_zT");
	h->useGPS_param_handle = param_find("POS_useGPS");
	return OK;
}

int parameters_update(const struct multirotor_position_control_param_handles *h, struct multirotor_position_control_params *p)
{
	param_get(h->pos_p, &(p->pos_p));
	param_get(h->pos_d, &(p->pos_d));
	param_get(h->height_p, &(p->height_p));
	param_get(h->height_d, &(p->height_d));
	param_get(h->height_i_param_handle, &(p->height_i));
	param_get(h->height_ff, &(p->height_ff));
	param_get(h->loc_sp_x_param_handle, &(p->loc_sp_x));
	param_get(h->loc_sp_y_param_handle, &(p->loc_sp_y));
	param_get(h->loc_sp_z_param_handle, &(p->loc_sp_z));
	param_get(h->glo_sp_lat_param_handle, &(p->glo_sp_lat));
	param_get(h->glo_sp_lon_param_handle, &(p->glo_sp_lon));
	param_get(h->glo_sp_alt_param_handle, &(p->glo_sp_alt));
	param_get(h->sp_gain_xy_param_handle, &(p->sp_gain_xy));
	param_get(h->sp_gain_xy_threshold_param_handle, &(p->sp_gain_xy_threshold));
	param_get(h->vel_limit_enabled_param_handle, &(p->vel_limit_enabled));
	param_get(h->vel_limit_gain_xy_param_handle, &(p->vel_limit_gain_xy));
	param_get(h->vel_limit_gain_xy_threshold_param_handle, &(p->vel_limit_gain_xy_threshold));
	param_get(h->vel_limit_gain_z_param_handle, &(p->vel_limit_gain_z));
	param_get(h->vel_limit_gain_z_threshold_param_handle, &(p->vel_limit_gain_z_threshold));
	param_get(h->useGPS_param_handle, &(p->useGPS));
	return OK;
}
