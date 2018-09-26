/*
 * MP_controller.h
 *
 *  Created on: Aug 2, 2018
 *      Author: miftakur
 */

#ifndef MP_CONTROLLER_H_
#define MP_CONTROLLER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "main.h"
#include <math.h>
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define ROLL_ANGLE_MIN		0
#define ROLL_ANGLE_MAX		180

//1427
#define PITCH_ANGLE_MIN		60
//1627
#define PITCH_ANGLE_MAX		120
//#define PITCH_ANGLE_MIN		45
//#define PITCH_ANGLE_MAX		135
/* Exported macro ------------------------------------------------------------*/
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
/* Exported functions ------------------------------------------------------- */

// SERVO
int SV_applySlew(int input_now, int delta, int rate_limit);
void SV_setParameter(float *gain, int *slew);
void SV_calculation(int rollError, int pitchError);
void SV_getAngle(int * rAngle, int * pAngle);

//FIN
void MP_Conversion(float *sigYZ, float *roll, float *AyzCom, float *rollCom);

float degrees_to_radians(float deg);
float radians_to_degrees(float rad);


#endif /* MP_CONTROLLER_H_ */
