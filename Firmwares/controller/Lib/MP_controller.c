/*
 * MP_controller.c
 *
 *  Created on: Aug 2, 2018
 *      Author: miftakur
 */

/* Includes ------------------------------------------------------------------*/
#include "MP_controller.h"

/* External variables --------------------------------------------------------*/
extern float gainImu[2];
extern float gainCom[2];

/* Internal variables --------------------------------------------------------*/
float servo_gain[2] = { 0.5f, 0.5f };
int servo_slew[2] = { 5, 5 };
int rollAngle = 0;
int pitchAngle = 0;
float tYprev = 0.0f, tZprev = 0.0f;

int SV_applySlew(int input_now, int delta, int rate_limit)
{
	if (rate_limit != 0)
	{
		if (delta > rate_limit)
			delta = rate_limit;
		if (delta < -rate_limit)
			delta = -rate_limit;
	}
	return input_now + delta;
}

void SV_setParameter(float *gain, int *slew)
{
	servo_gain[0] = *gain;
	servo_gain[1] = *(gain + 1);

	servo_slew[0] = *slew;
	servo_slew[1] = *(slew + 1);

	tYprev = 0.0f;
	tZprev = 0.0f;
}

void SV_calculation(int rollError, int pitchError)
{
	//roll angle calculation
	rollError *= servo_gain[0];
	rollAngle = SV_applySlew(rollAngle, rollError, servo_slew[0]);
	rollAngle = constrain(rollAngle, ROLL_ANGLE_MIN, ROLL_ANGLE_MAX);

	// pitch angle calculation
	pitchError *= servo_gain[1];
	pitchAngle = SV_applySlew(pitchAngle, pitchError, servo_slew[1]);
	pitchAngle = constrain(pitchAngle, PITCH_ANGLE_MIN, PITCH_ANGLE_MAX);
}

void SV_getAngle(int * rAngle, int * pAngle)
{
	*rAngle = rollAngle;
	*pAngle = pitchAngle;
}

void MP_Conversion(float *sigYZ, float *roll, float *AyzCom, float *rollCom)
{
	float tY, tZ;
	float deltaY, deltaZ;
	float AyCom, AzCom;

	tY = *sigYZ * cos(degrees_to_radians(*roll));
	tZ = *sigYZ * sin(degrees_to_radians(*roll));

	deltaY = tY - tYprev;
	deltaZ = tZ - tZprev;

	tYprev = tY;
	tZprev = tZ;

	//  AyCom = tY * gainAyCom;
	//  AzCom = tZ * gainAzCom;

	AzCom = deltaZ * gainCom[0];
	AyCom = deltaY * gainCom[1];

	*AyzCom = sqrt((AyCom * AyCom) + (AzCom * AzCom));
	*rollCom = radians_to_degrees(atan2(AzCom, AyCom));  //arc tangen y/x

//	*AyzCom = AzCom;
//	*rollCom= AyCom;
}

float degrees_to_radians(float deg)
{
	return deg * M_PI / 180.0;
}

float radians_to_degrees(float rad)
{
	return rad * 180.0 / M_PI;
}
