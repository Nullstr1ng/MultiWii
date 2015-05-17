// PIDControllers.h

#ifndef _PIDCONTROLLERS_h
#define _PIDCONTROLLERS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class PIDControllersClass
{
 protected:


 public:
	void init();
	void One(int16_t errorAngleI[2], int16_t errorGyroI[3], int32_t errorGyroI_YAW, uint8_t dynP8[2], uint8_t dynD8[2]);
	void AlexK(int32_t errorGyroI[3]);
	void ReWrite(int32_t errorGyroI[3]);
	void LuxFloat(int32_t errorGyroI[3]);
};

extern PIDControllersClass PIDControllers;

#endif

