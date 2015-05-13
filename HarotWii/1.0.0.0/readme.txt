HarotWii is a 100% MultiWii 2.4 version with some additional features such as:
1. Can connect to Baseflight and Cleanflight configurators.
This version though is 

2. Added new board called Gizduino and RCTimer CRIUS v2 AIOP. 
RCTimer's CRIUS is using a different barometer board "MS561101BA" unlike the default one that was using "BMP085"
Gizduino 

3. Supports generic sonar module such as HC-SR04, SRF04, DYP-ME007

4. Inflight PID tuning.
This requires AUX2(3 way switch) and AUX3(Potentio). 
	AUX2(3 way switch) - Is used to adjust from P, I, or D. 
	- P is the LO switch, I CEnter, and D is the HIghest switch
	AUX3(Potentio) - Is used to adjust the values of P, I, and D.
Check config.h SECTION 9

5. Support for MultiWii Mobile Control for Windows Phone.
This has new MSP called MSP_SUPRESS_DATA_FROM_RX = 150

6. Can skip GYRO calibration at startup
Configurable via config.h
Check config.h SECTION 9