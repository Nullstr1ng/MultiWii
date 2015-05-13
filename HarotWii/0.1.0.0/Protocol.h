#ifndef PROTOCOL_H_
#define PROTOCOL_H_

//#ifndef (CLEANFLIGHT)
typedef enum {
	FEATURE_RX_PPM = 1 << 0,
	FEATURE_VBAT = 1 << 1,
	FEATURE_INFLIGHT_ACC_CAL = 1 << 2,
	FEATURE_RX_SERIAL = 1 << 3,
	FEATURE_MOTOR_STOP = 1 << 4,
	FEATURE_SERVO_TILT = 1 << 5,
	FEATURE_SOFTSERIAL = 1 << 6,
	FEATURE_GPS = 1 << 7,
	FEATURE_FAILSAFE = 1 << 8,
	FEATURE_SONAR = 1 << 9,
	FEATURE_TELEMETRY = 1 << 10,
	FEATURE_CURRENT_METER = 1 << 11,
	FEATURE_3D = 1 << 12,
	FEATURE_RX_PARALLEL_PWM = 1 << 13,
	FEATURE_RX_MSP = 1 << 14,
	FEATURE_RSSI_ADC = 1 << 15,
	FEATURE_LED_STRIP = 1 << 16,
	FEATURE_DISPLAY = 1 << 17,
	FEATURE_ONESHOT125 = 1 << 18,
	FEATURE_BLACKBOX = 1 << 19
} features_e;
//#endif

void serialCom();
void debugmsg_append_str(const char *str);
void featureSet(uint32_t mask);

#endif /* PROTOCOL_H_ */
