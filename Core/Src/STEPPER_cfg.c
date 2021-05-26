/*
 * File: STEPPER_cfg.c
 * Driver Name: [[ STEPPER Motor ]]
 * SW Layer:   ECUAL
 * Created on: Jun 28, 2020
 * Author:     Khaled Magdy
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.DeepBlueMbedded.com
 *
 */

#include "STEPPER.h"

const STEPPER_CfgType STEPPER_CfgParam[STEPPER_UNITS] =
{
	// Stepper Motor 1 Configurations
    {
	    {GPIOD, GPIOD, GPIOD, GPIOD},
		{GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5},
		2058,
		STEPPER_UNIPOLAR,
		FULL_STEP_DRIVE
//		HALF_STEP_DRIVE
//		WAVE_DRIVE
	}
};
