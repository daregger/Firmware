/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file GPS driver interface.
 */

#ifndef _DRV_GPS_H
#define _DRV_GPS_H

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_sensor.h"
#include "drv_orb_dev.h"

#define GPS_DEVICE_PATH	"/dev/gps"
#define GPS_BAUDRATES_TO_TRY {9600, 38400, 57600, 115200}

typedef enum {
	GPS_DRIVER_MODE_UBX = 0,
	GPS_DRIVER_MODE_MTK19,
	GPS_DRIVER_MODE_MTK16,
	GPS_DRIVER_MODE_NMEA,
} gps_driver_mode_t;


/*
 * ObjDev tag for GPS data.
 */
ORB_DECLARE(gps);

/*
 * ioctl() definitions
 */
#define _GPSIOCBASE			(0x2800)            //TODO: arbitrary choice...
#define _GPSIOC(_n)		(_IOC(_GPSIOCBASE, _n))

/** configure ubx mode */
#define GPS_CONFIGURE_UBX	_GPSIOC(0)

/** configure mtk mode */
#define GPS_CONFIGURE_MTK19	_GPSIOC(1)
#define GPS_CONFIGURE_MTK16	_GPSIOC(2)

/** configure mtk mode */
#define GPS_CONFIGURE_NMEA	_GPSIOC(3)

#endif /* _DRV_GPS_H */