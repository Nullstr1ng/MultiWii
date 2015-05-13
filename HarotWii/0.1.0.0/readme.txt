HarotWii is a 100% MultiWii 2.4 version with some additional features such as:
1. Can connect to Baseflight and Cleanflight configurators.
	This version though is concentrating on Baseflight since Cleanflight is using a different flight modes and other cool features that Baseflight doesn't have.

2. Added new board called Gizduino and RCTimer CRIUS v2 AIOP. 
	RCTimer's CRIUS is using a different barometer board "MS561101BA". The default one that was using "BMP085"
	Gizduino - http://www.e-gizmo.com/KIT/categ/pmaster.php?dpage=/KIT/categ/kitcontrol.htm

3. Supports generic sonar module such as HC-SR04, SRF04, DYP-ME007
	Check config.h SECTION 9 for PIN assignments
	
4. Inflight PID tuning.
	This requires AUX2(3 way switch) and AUX3(Potentio). 
		AUX2(3 way switch) - Is used to adjust from P, I, or D. 
		- P is the LO switch, I CEnter, and D is the HIghest switch
		AUX3(Potentio) - Is used to adjust the values of P, I, and D.
	Check config.h SECTION 9

5. Support for MultiWii Mobile Control for Windows Phone.
	This has new MSP called MSP_SUPRESS_DATA_FROM_RX = 150

6. Can skip GYRO calibration at startup
	Make sure to calibrate the GYRO after you update the firmware.
	Configurable via config.h
	Check config.h SECTION 9
	
7. PID Controller
	PID controller codes are now in separate .cpp file called "PIDControllers.cpp"
	
*Baseflight Configurator*
You can connect HarotWii to Baseflight configurator but there are some limitations such as in these tabs:
Configuration - Mixer, Serial Receiver, Battery Voltage, Board Alignment, GPS, Current Sensor
PID Tuning - 0.64 release now supports individual axis rates. You can still set Pitch and Roll rates by changing the Roll rate. That will automatically update the Pitch rate as well.
Servos tab - not working
CLI