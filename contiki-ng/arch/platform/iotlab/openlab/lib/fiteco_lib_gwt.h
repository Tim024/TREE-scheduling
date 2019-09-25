/*
 * This file is part of HiKoB Openlab.
 *
 * HiKoB Openlab is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, version 3.
 *
 * HiKoB Openlab is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with HiKoB Openlab. If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright (C) 2012 HiKoB.
 */

/*
 * fiteco_lib_gwt.h
 *
 *  Created on: Nov 30, 2012
 *      Author: burindes
 */

#ifndef FITECO_LIB_GWT_H_
#define FITECO_LIB_GWT_H_

#include "handler.h"
#include "ina226.h"

/**
 * Stop sampling the current.
 */
void fiteco_lib_gwt_current_monitor_stop();

/**
 * Configure the current monitor sampling parameters.
 *
 * Sampling should be stopped before configuring.
 *
 * \param period the sampling period
 * \param average the averaging factor
 */
void fiteco_lib_gwt_current_monitor_configure(ina226_sampling_period_t period,
        ina226_averaging_factor_t average);

typedef enum
{
    FITECO_GWT_CURRENT_MONITOR__OFF = 0,
    FITECO_GWT_CURRENT_MONITOR__OPEN_3V = 1,
    FITECO_GWT_CURRENT_MONITOR__OPEN_5V = 2,
    FITECO_GWT_CURRENT_MONITOR__BATTERY = 3,
} fiteco_lib_gwt_current_monitor_selection_t;

/** Handler for current monitor sample */
typedef void (*fiteco_lib_gwt_current_monitor_handler_t)(handler_arg_t arg,
        float voltage, float current, float power, uint32_t timestamp);

/**
 * Select the input for the Current Monitor circuit and start sampling
 *
 * \param selection the power input to measure
 * \param handler the handler function to call on each new measure
 * \param arg optional argument to provide to the handler
 */
void fiteco_lib_gwt_current_monitor_select(
        fiteco_lib_gwt_current_monitor_selection_t selection,
        fiteco_lib_gwt_current_monitor_handler_t handler, handler_arg_t arg);

typedef enum
{
    FITECO_GWT_OPENNODE_POWER__OFF = 0,
    FITECO_GWT_OPENNODE_POWER__MAIN = 1,
    FITECO_GWT_OPENNODE_POWER__BATTERY = 2,
} fiteco_lib_gwt_opennode_power_selection_t;

/** Select the OpenNode power mode */
void fiteco_lib_gwt_opennode_power_select(
        fiteco_lib_gwt_opennode_power_selection_t selection);

/** Enable OpenNode battery charging */
void fiteco_lib_gwt_battery_charge_enable();

/** Disable OpenNode battery charging */
void fiteco_lib_gwt_battery_charge_disable();

#endif /* FITECO_LIB_GWT_H_ */
