/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file ocpoc_xadc_batt_monitor.cpp
 *
 * @author Lianying Ji <ji@aerotenna.com>
 * @author Dave Royer <dave@aerotenna.com>
 */
 
#include "ocpoc_xadc_batt_monitor.h"
#include "px4_adc.h"



void OCPOC_XADC_BATT_MONITOR::init(void)
{
}

int  OCPOC_XADC_BATT_MONITOR::read(struct adc_msg_s (*buf)[12], unsigned int len)
{
	uint32_t buff[1];

	xadc_fd = fopen("/sys/bus/iio/devices/iio:device0/in_voltage8_raw", "r");

	if (xadc_fd != NULL) {
		fscanf(xadc_fd, "%d", buff);
		fclose(xadc_fd);
	}else{
		return -1;
	}

	(*buf)[0].am_data = buff[0];
	(*buf)[0].am_channel = ADC_BATTERY_VOLTAGE_CHANNEL;

	return sizeof((*buf)[0]);
}

