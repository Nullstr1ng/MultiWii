#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "IMU.h"
#include "EEPROM.h"
#include "MultiWii.h"
#include "PIDControllers.h"

int32_t prop = 0;
uint8_t axis, i;
int16_t errorAngle;
int16_t delta;
int16_t PTerm = 0, ITerm = 0, DTerm, PTermACC, ITermACC;

static int16_t lastGyro[2] = { 0, 0 };

//#if PID_CONTROLLER == 1
//static int16_t delta1[2], delta2[2];

//#elif PID_CONTROLLER == 2
//static int16_t delta1[3], delta2[3];

void PIDControllersClass::init()
{
	
}

void PIDControllersClass::One(int16_t errorAngleI[2], int16_t errorGyroI[3], int32_t errorGyroI_YAW, uint8_t dynP8[2], uint8_t dynD8[2]) {
	static int16_t delta1[2], delta2[2];
	int16_t error;
	int16_t rc;

	if (f.HORIZON_MODE) prop = min(max(abs(rcCommand[PITCH]), abs(rcCommand[ROLL])), 512);

	// PITCH & ROLL
	for (axis = 0; axis < 2; axis++) {
		rc = rcCommand[axis] << 1;
		error = rc - imu.gyroData[axis];
		errorGyroI[axis] = constrain(errorGyroI[axis] + error, -16000, +16000);   // WindUp   16 bits is ok here
		if (abs(imu.gyroData[axis]) > 640) errorGyroI[axis] = 0;

		ITerm = (errorGyroI[axis] >> 7) * conf.pid[axis].I8 >> 6;                  // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000

		PTerm = mul(rc, conf.pid[axis].P8) >> 6;

		if (f.ANGLE_MODE || f.HORIZON_MODE) { // axis relying on ACC
			// 50 degrees max inclination
			errorAngle = constrain(rc + GPS_angle[axis], -500, +500) - att.angle[axis] + conf.angleTrim[axis]; //16 bits is ok here

			// todo
			// autotune
			//

			errorAngleI[axis] = constrain(errorAngleI[axis] + errorAngle, -10000, +10000);                                            // WindUp     //16 bits is ok here

			PTermACC = mul(errorAngle, conf.pid[PIDLEVEL].P8) >> 7; // 32 bits is needed for calculation: errorAngle*P8 could exceed 32768   16 bits is ok for result

			int16_t limit = conf.pid[PIDLEVEL].D8 * 5;
			PTermACC = constrain(PTermACC, -limit, +limit);

			ITermACC = mul(errorAngleI[axis], conf.pid[PIDLEVEL].I8) >> 12; // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result

			ITerm = ITermACC + ((ITerm - ITermACC) * prop >> 9);
			PTerm = PTermACC + ((PTerm - PTermACC) * prop >> 9);
		}

		PTerm -= mul(imu.gyroData[axis], dynP8[axis]) >> 6; // 32 bits is needed for calculation

		delta = imu.gyroData[axis] - lastGyro[axis];  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
		lastGyro[axis] = imu.gyroData[axis];
		DTerm = delta1[axis] + delta2[axis] + delta;
		delta2[axis] = delta1[axis];
		delta1[axis] = delta;

		DTerm = mul(DTerm, dynD8[axis]) >> 5;     // 32 bits is needed for calculation

		axisPID[axis] = PTerm + ITerm - DTerm;
	}

	//YAW
#define GYRO_P_MAX 300
#define GYRO_I_MAX 250

	rc = mul(rcCommand[YAW], (2 * conf.yawRate + 30)) >> 5;

	error = rc - imu.gyroData[YAW];
	errorGyroI_YAW += mul(error, conf.pid[YAW].I8);
	errorGyroI_YAW = constrain(errorGyroI_YAW, 2 - ((int32_t) 1 << 28), -2 + ((int32_t) 1 << 28));
	if (abs(rc) > 50) errorGyroI_YAW = 0;

	PTerm = mul(error, conf.pid[YAW].P8) >> 6;
#ifndef COPTER_WITH_SERVO
	int16_t limit = GYRO_P_MAX - conf.pid[YAW].D8;
	PTerm = constrain(PTerm, -limit, +limit);
#endif

	ITerm = constrain((int16_t) (errorGyroI_YAW >> 13), -GYRO_I_MAX, +GYRO_I_MAX);

	axisPID[YAW] = PTerm + ITerm;
}

// pid controller 2
void PIDControllersClass::AlexK(int32_t errorGyroI[3]) {
	static int16_t delta1[3], delta2[2];
	static int16_t lastError[3] = { 0, 0, 0 };
	int16_t deltaSum;
	int16_t AngleRateTmp, RateError;

	#define GYRO_I_MAX 256
	#define ACC_I_MAX 256
	prop = min(max(abs(rcCommand[PITCH]), abs(rcCommand[ROLL])), 500); // range [0;500]
	
	//----------PID controller----------
	for (axis = 0; axis<3; axis++) {
		//-----Get the desired angle rate depending on flight mode
		if ((f.ANGLE_MODE || f.HORIZON_MODE) && axis<2) { // MODE relying on ACC
			// calculate error and limit the angle to 50 degrees max inclination
			errorAngle = constrain((rcCommand[axis] << 1) + GPS_angle[axis], -500, +500) - att.angle[axis] + conf.angleTrim[axis]; //16 bits is ok here
		}
		if (axis == 2) {//YAW is always gyro-controlled (MAG correction is applied to rcCommand)
			AngleRateTmp = (((int32_t) (conf.yawRate + 27) * rcCommand[2]) >> 5);
		}
		else {
			if (!f.ANGLE_MODE) {//control is GYRO based (ACRO and HORIZON - direct sticks control is applied to rate PID
				AngleRateTmp = ((int32_t) (conf.rollPitchRate + 27) * rcCommand[axis]) >> 4;
				if (f.HORIZON_MODE) {
					//mix up angle error to desired AngleRateTmp to add a little auto-level feel
					AngleRateTmp += ((int32_t) errorAngle * conf.pid[PIDLEVEL].I8) >> 8;
				}
			} else {//it's the ANGLE mode - control is angle based, so control loop is needed
				AngleRateTmp = ((int32_t) errorAngle * conf.pid[PIDLEVEL].P8)>>4;
			}
		}
	
		//--------low-level gyro-based PID. ----------
		//Used in stand-alone mode for ACRO, controlled by higher level regulators in other modes
		//-----calculate scaled error.AngleRates
		//multiplication of rcCommand corresponds to changing the sticks scaling here
		RateError = AngleRateTmp - imu.gyroData[axis];
	
		//-----calculate P component
		PTerm = ((int32_t) RateError * conf.pid[axis].P8) >> 7;
	
		//-----calculate I component
		//there should be no division before accumulating the error to integrator, because the precision would be reduced.
		//Precision is critical, as I prevents from long-time drift. Thus, 32 bits integrator is used.
		//Time correction (to avoid different I scaling for different builds based on average cycle time)
		//is normalized to cycle time = 2048.
		errorGyroI[axis] += (((int32_t) RateError * cycleTime) >> 11) * conf.pid[axis].I8;
		//limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
		//I coefficient (I8) moved before integration to make limiting independent from PID settings
		errorGyroI[axis] = constrain(errorGyroI[axis], (int32_t) -GYRO_I_MAX << 13, (int32_t) +GYRO_I_MAX << 13);
		ITerm = errorGyroI[axis] >> 13;
	
		//-----calculate D-term
		delta = RateError - lastError[axis];  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
		lastError[axis] = RateError;
	
		//Correct difference by cycle time. Cycle time is jittery (can be different 2 times), so calculated difference
		// would be scaled by different dt each time. Division by dT fixes that.
		delta = ((int32_t) delta * ((uint16_t) 0xFFFF / (cycleTime >> 4))) >> 6;
		//add moving average here to reduce noise
		deltaSum = delta1[axis] + delta2[axis] + delta;
		delta2[axis] = delta1[axis];
		delta1[axis] = delta;
	
		//DTerm = (deltaSum*conf.pid[axis].D8)>>8;
		//Solve overflow in calculation above...
		DTerm = ((int32_t) deltaSum*conf.pid[axis].D8) >> 8;
		//-----calculate total PID output
		axisPID[axis] = PTerm + ITerm + DTerm;
	}
}

// pid controller 2
void PIDControllersClass::ReWrite(int32_t errorGyroI[3]) {
	static int16_t delta1[3], delta2[2];
	static int16_t lastError[3] = { 0, 0, 0 };
	int16_t deltaSum;
	int16_t AngleRateTmp, RateError;

#define GYRO_I_MAX 256
#define ACC_I_MAX 256

	prop = min(max(abs(rcCommand[PITCH]), abs(rcCommand[ROLL])), 500); // range [0;500]

	//----------PID controller----------
	for (axis = 0; axis<3; axis++) {
		uint8_t rate = 0;

		if (axis == 2) {
			rate = conf.yawRate;
		}
		else if (axis == 1) {
			rate = conf.rollPitchRate;
		}
		else if (axis == 0) {
			rate = conf.rollPitchRate;
		}

		if (axis == 2) {//YAW is always gyro-controlled (MAG correction is applied to rcCommand)
			AngleRateTmp = (((int32_t) (rate + 27) * rcCommand[2]) >> 5);
		}
		else {
#ifdef GPS
			errorAngle = constrain(2 * rcCommand[axis] + GPS_angle[axis], -500, +500) - att.angle[axis] + conf.angleTrim[axis];
#else
			errorAngle = constrain(2 * rcCommand[axis], -500, +500) - att.angle[axis] + conf.angleTrim[axis];
#endif

			if (!f.ANGLE_MODE) {//control is GYRO based (ACRO and HORIZON - direct sticks control is applied to rate PID
				AngleRateTmp = ((int32_t) (rate + 27) * rcCommand[axis]) >> 4;
				if (f.HORIZON_MODE) {
					//mix up angle error to desired AngleRateTmp to add a little auto-level feel
					AngleRateTmp += ((int32_t) errorAngle * conf.pid[PIDLEVEL].I8) >> 8;
				}
			}
			else {//it's the ANGLE mode - control is angle based, so control loop is needed
				AngleRateTmp = ((int32_t) errorAngle * conf.pid[PIDLEVEL].P8) >> 4;
			}
		}

		//--------low-level gyro-based PID. ----------
		//Used in stand-alone mode for ACRO, controlled by higher level regulators in other modes
		//-----calculate scaled error.AngleRates
		//multiplication of rcCommand corresponds to changing the sticks scaling here
		RateError = AngleRateTmp - (imu.gyroData[axis] / 4);

		//-----calculate P component
		PTerm = ((int32_t) RateError * conf.pid[axis].P8) >> 7;

		//-----calculate I component
		//there should be no division before accumulating the error to integrator, because the precision would be reduced.
		//Precision is critical, as I prevents from long-time drift. Thus, 32 bits integrator is used.
		//Time correction (to avoid different I scaling for different builds based on average cycle time)
		//is normalized to cycle time = 2048.
		errorGyroI[axis] += (((int32_t) RateError * cycleTime) >> 11) * conf.pid[axis].I8;
		//limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
		//I coefficient (I8) moved before integration to make limiting independent from PID settings
		errorGyroI[axis] = constrain(errorGyroI[axis], (int32_t) -GYRO_I_MAX << 13, (int32_t) +GYRO_I_MAX << 13);
		ITerm = errorGyroI[axis] >> 13;

		//-----calculate D-term
		delta = RateError - lastError[axis];  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
		lastError[axis] = RateError;

		//Correct difference by cycle time. Cycle time is jittery (can be different 2 times), so calculated difference
		// would be scaled by different dt each time. Division by dT fixes that.
		delta = ((int32_t) delta * ((uint16_t) 0xFFFF / (cycleTime >> 4))) >> 6;
		//add moving average here to reduce noise
		deltaSum = delta1[axis] + delta2[axis] + delta;
		delta2[axis] = delta1[axis];
		delta1[axis] = delta;

		//DTerm = (deltaSum*conf.pid[axis].D8)>>8;
		//Solve overflow in calculation above...
		DTerm = ((int32_t) deltaSum*conf.pid[axis].D8) >> 8;
		//-----calculate total PID output
		axisPID[axis] = PTerm + ITerm + DTerm;
	}
}

void PIDControllersClass::LuxFloat(int32_t errorGyroI[3]) {
	
}

PIDControllersClass PIDControllers;

