#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "EEPROM.h"
#include "LCD.h"
#include "Output.h"
#include "GPS.h"
#include "MultiWii.h"
#include "Serial.h"
#include "Protocol.h"
#include "RX.h"

#if defined(CLEANFLIGHT)
#include "target.h"
#include "version.h"
#include "platform.h"
#endif

/************************************** MultiWii Serial Protocol *******************************************************/
// Multiwii Serial Protocol 0 
#define MSP_VERSION              0

//to multiwii developpers/committers : do not add new MSP messages without a proper argumentation/agreement on the forum
//range id [50-99] won't be assigned and can therefore be used for any custom multiwii fork without further MSP id conflict

#define MSP_PRIVATE              1     //in+out message      to be used for a generic framework : MSP + function code (LIST/GET/SET) + data. no code yet

#pragma region CLEANFLIGHT MSP
#if defined(CLEANFLIGHT)
/* stripped off from cleanflight*/
/* https://github.com/cleanflight/cleanflight/blob/master/src/main/io/serial_msp.c */
#define MSP_PROTOCOL_VERSION 0
#define API_VERSION_MAJOR                   1 // increment when major changes are made
#define API_VERSION_MINOR                   5 // increment when any change is made, reset to zero when major changes are released after changing API_VERSION_MAJOR

#define API_VERSION_LENGTH                  2

#define MULTIWII_IDENTIFIER "MWII";
#define CLEANFLIGHT_IDENTIFIER "CLFL"
#define BASEFLIGHT_IDENTIFIER "BAFL";

#define FLIGHT_CONTROLLER_IDENTIFIER_LENGTH 4
static const char *flightControllerIdentifier = CLEANFLIGHT_IDENTIFIER; // 4 UPPER CASE alpha numeric characters that identify the flight controller.

#define FLIGHT_CONTROLLER_VERSION_LENGTH    3
#define FLIGHT_CONTROLLER_VERSION_MASK      0xFFF

const char *boardIdentifier = TARGET_BOARD_IDENTIFIER;
#define BOARD_IDENTIFIER_LENGTH             4 // 4 UPPER CASE alpha numeric characters that identify the board being used.
#define BOARD_HARDWARE_REVISION_LENGTH      2

#define MSP_API_VERSION                 1    //out message
#define MSP_FC_VARIANT                  2    //out message
#define MSP_FC_VERSION                  3    //out message
#define MSP_BOARD_INFO                  4    //out message
#define MSP_BUILD_INFO                  5    //out message

//
// MSP commands for Cleanflight original features
//
#define MSP_CHANNEL_FORWARDING          32    //out message         Returns channel forwarding settings
#define MSP_SET_CHANNEL_FORWARDING      33    //in message          Channel forwarding settings

#define MSP_MODE_RANGES                 34    //out message         Returns all mode ranges
#define MSP_SET_MODE_RANGE              35    //in message          Sets a single mode range

#define MSP_FEATURE                     36
#define MSP_SET_FEATURE                 37

#define MSP_BOARD_ALIGNMENT             38
#define MSP_SET_BOARD_ALIGNMENT         39

#define MSP_CURRENT_METER_CONFIG        40
#define MSP_SET_CURRENT_METER_CONFIG    41

#define MSP_MIXER                       42
#define MSP_SET_MIXER                   43

#define MSP_RX_CONFIG                   44
#define MSP_SET_RX_CONFIG               45

#define MSP_LED_COLORS                  46
#define MSP_SET_LED_COLORS              47

#define MSP_LED_STRIP_CONFIG            48
#define MSP_SET_LED_STRIP_CONFIG        49

#define MSP_RSSI_CONFIG                 50
#define MSP_SET_RSSI_CONFIG             51

#define MSP_ADJUSTMENT_RANGES           52
#define MSP_SET_ADJUSTMENT_RANGE        53

// private - only to be used by the configurator, the commands are likely to change
#define MSP_CF_SERIAL_CONFIG            54
#define MSP_SET_CF_SERIAL_CONFIG        55

#define MSP_VOLTAGE_METER_CONFIG        56
#define MSP_SET_VOLTAGE_METER_CONFIG    57

#define MSP_SONAR_ALTITUDE              58 //out message get sonar altitude [cm]

#define MSP_PID_CONTROLLER              59
#define MSP_SET_PID_CONTROLLER          60
//
// Baseflight MSP commands (if enabled they exist in Cleanflight)
//
#define MSP_RX_MAP                      64 //out message get channel map (also returns number of channels total)
#define MSP_SET_RX_MAP                  65 //in message set rx map, numchannels to set comes from MSP_RX_MAP

// FIXME - Provided for backwards compatibility with configurator code until configurator is updated.
// DEPRECATED - DO NOT USE "MSP_BF_CONFIG" and MSP_SET_BF_CONFIG.  In Cleanflight, isolated commands already exist and should be used instead.
#define MSP_BF_CONFIG                      66 //out message baseflight-specific settings that aren't covered elsewhere
#define MSP_SET_BF_CONFIG                  67 //in message baseflight-specific settings save

#define MSP_REBOOT                      68 //in message reboot settings

// BASEFLIGHT
#define MSP_BF_BUILD_INFO               69 //out message build date as well as some space for future expansion

#define MSP_DATAFLASH_SUMMARY           70 //out message - get description of dataflash chip
#define MSP_DATAFLASH_READ              71 //out message - get content of dataflash chip
#define MSP_DATAFLASH_ERASE             72 //in message - erase dataflash chip

// Additional commands that are not compatible with MultiWii
#define MSP_UID                  160    //out message         Unique device ID
#define MSP_ACC_TRIM             240    //out message         get acc angle trim values
#define MSP_SET_ACC_TRIM         239    //in message          set acc angle trim values
#define MSP_GPSSVINFO            164    //out message         get Signal Strength (only U-Blox)
#define MSP_GPSDEBUGINFO         166    //out message         get GPS debugging data (only U-Blox)
#endif

#pragma endregion

#pragma region ORIGINAL MultiWii MSP OUT COMMANDS
#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114   //out message         powermeter trig
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes
#define MSP_SERVO_CONF           120   //out message         Servo settings

#define MSP_NAV_STATUS           121   //out message         Returns navigation status
#define MSP_NAV_CONFIG           122   //out message         Returns navigation parameters

#define MSP_CELLS                130   //out message         FRSKY Battery Cell Voltages

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211   //in message          define a new heading hold direction
#define MSP_SET_SERVO_CONF       212   //in message          Servo settings
#define MSP_SET_MOTOR            214   //in message          PropBalance function
#define MSP_SET_NAV_CONFIG       215   //in message          Sets nav config parameters - write to the eeprom  

#define MSP_SET_ACC_TRIM         239   //in message          set acc angle trim values
#define MSP_ACC_TRIM             240   //out message         get acc angle trim values
#define MSP_BIND                 241   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4

#define MSP_SUPRESS_DATA_FROM_RX 150
#define MSP_GYRO_CALIBRATION     151
#pragma endregion

#ifdef DEBUGMSG
#define DEBUG_MSG_BUFFER_SIZE 128
static char debug_buf[DEBUG_MSG_BUFFER_SIZE];
static uint8_t head_debug;
static uint8_t tail_debug;
static uint8_t debugmsg_available();
static void debugmsg_serialize(uint8_t l);
#endif

static uint32_t enabledFeatures = 0;

static uint8_t CURRENTPORT = 0;
static uint8_t USE_CLEANFLIGHT_REPLIES = 0;

#define INBUF_SIZE 64
static uint8_t inBuf[INBUF_SIZE][UART_NUMBER];
static uint8_t checksum[UART_NUMBER];
static uint8_t indRX[UART_NUMBER];
static uint8_t cmdMSP[UART_NUMBER];

void evaluateOtherData(uint8_t sr);
void evaluateCommand(uint8_t c);

static uint8_t read8()  {
	return inBuf[indRX[CURRENTPORT]++][CURRENTPORT] & 0xff;
}
static uint16_t read16() {
	uint16_t t = read8();
	t += (uint16_t) read8() << 8;
	return t;
}
static uint32_t read32() {
	uint32_t t = read16();
	t += (uint32_t) read16() << 16;
	return t;
}

static void serialize8(uint8_t a) {
	SerialSerialize(CURRENTPORT, a);
	checksum[CURRENTPORT] ^= a;
}
static void serialize16(int16_t a) {
	serialize8((a) & 0xFF);
	serialize8((a >> 8) & 0xFF);
}
static void serialize32(uint32_t a) {
	serialize8((a) & 0xFF);
	serialize8((a >> 8) & 0xFF);
	serialize8((a >> 16) & 0xFF);
	serialize8((a >> 24) & 0xFF);
}

static void headSerialResponse(uint8_t err, uint8_t s) {
	serialize8('$');
	serialize8('M');
	serialize8(err ? '!' : '>');
	checksum[CURRENTPORT] = 0; // start calculating a new checksum
	serialize8(s);
	serialize8(cmdMSP[CURRENTPORT]);
}

static void headSerialReply(uint8_t s) {
	headSerialResponse(0, s);
}

static void headSerialError() {
	headSerialResponse(1, 0);
}

static void tailSerialReply() {
	serialize8(checksum[CURRENTPORT]); UartSendData(CURRENTPORT);
}

static void serializeNames(PGM_P s) {
	headSerialReply(strlen_P(s));
	for (PGM_P c = s; pgm_read_byte(c); c++)
		serialize8(pgm_read_byte(c));
	tailSerialReply();
}

static void __attribute__((noinline)) s_struct_w(uint8_t *cb, uint8_t siz) {
	while (siz--) *cb++ = read8();
}

static void s_struct_partial(uint8_t *cb, uint8_t siz) {
	while (siz--) serialize8(*cb++);
}

static void s_struct(uint8_t *cb, uint8_t siz) {
	headSerialReply(siz);
	s_struct_partial(cb, siz);
	tailSerialReply();
}

static void mspAck() {
	headSerialReply(0); tailSerialReply();
}

enum MSP_protocol_bytes {
	IDLE,
	HEADER_START,
	HEADER_M,
	HEADER_ARROW,
	HEADER_SIZE,
	HEADER_CMD
};

void featureSet(uint32_t mask) {
	enabledFeatures |= mask;
}

void serialCom() {
	uint8_t c, cc, port, state, bytesTXBuff;
	static uint8_t offset[UART_NUMBER];
	static uint8_t dataSize[UART_NUMBER];
	static uint8_t c_state[UART_NUMBER];
	uint32_t timeMax; // limit max time in this function in case of GPS

	timeMax = micros();
	for (port = 0; port<UART_NUMBER; port++) {
		CURRENTPORT = port;
#define RX_COND
#if defined(SERIAL_RX) && (UART_NUMBER > 1)
#define RX_COND && (RX_SERIAL_PORT != port)
#endif
		cc = SerialAvailable(port);
		while (cc-- RX_COND) {
			bytesTXBuff = SerialUsedTXBuff(port); // indicates the number of occupied bytes in TX buffer
			if (bytesTXBuff > TX_BUFFER_SIZE - 50) return; // ensure there is enough free TX buffer to go further (50 bytes margin)
			c = SerialRead(port);
#ifdef SUPPRESS_ALL_SERIAL_MSP
			evaluateOtherData(c); // no MSP handling, so go directly
#else //SUPPRESS_ALL_SERIAL_MSP
			state = c_state[port];
			// regular data handling to detect and handle MSP and other data
			if (state == IDLE) {
				if (c == '$') state = HEADER_START;
				else evaluateOtherData(c); // evaluate all other incoming serial data
			}
			else if (state == HEADER_START) {
				state = (c == 'M') ? HEADER_M : IDLE;
			}
			else if (state == HEADER_M) {
				state = (c == '<') ? HEADER_ARROW : IDLE;
			}
			else if (state == HEADER_ARROW) {
				if (c > INBUF_SIZE) {  // now we are expecting the payload size
					state = IDLE;
					continue;
				}
				dataSize[port] = c;
				checksum[port] = c;
				offset[port] = 0;
				indRX[port] = 0;
				state = HEADER_SIZE;  // the command is to follow
			}
			else if (state == HEADER_SIZE) {
				cmdMSP[port] = c;
				checksum[port] ^= c;
				state = HEADER_CMD;
			}
			else if (state == HEADER_CMD) {
				if (offset[port] < dataSize[port]) {
					checksum[port] ^= c;
					inBuf[offset[port]++][port] = c;
				}
				else {
					if (checksum[port] == c) { // compare calculated and transferred checksum
						evaluateCommand(cmdMSP[port]); // we got a valid packet, evaluate it
					}
					state = IDLE;
					cc = 0; // no more than one MSP per port and per cycle
				}
			}
			c_state[port] = state;

			// SERIAL: try to detect a new nav frame based on the current received buffer
#if defined(GPS_SERIAL)
			if (GPS_SERIAL == port) {
				static uint32_t GPS_last_frame_seen; //Last gps frame seen at this time, used to detect stalled gps communication
				if (GPS_newFrame(c)) {
					//We had a valid GPS data frame, so signal task scheduler to switch to compute
					if (GPS_update == 1) GPS_update = 0; else GPS_update = 1; //Blink GPS update
					GPS_last_frame_seen = timeMax;
					GPS_Frame = 1;
				}

				// Check for stalled GPS, if no frames seen for 1.2sec then consider it LOST
				if ((timeMax - GPS_last_frame_seen) > 1200000) {
					//No update since 1200ms clear fix...
					f.GPS_FIX = 0;
					GPS_numSat = 0;
				}
			}
			if (micros() - timeMax > 250) return;  // Limit the maximum execution time of serial decoding to avoid time spike
#endif
#endif // SUPPRESS_ALL_SERIAL_MSP
		} // while
	} // for
}

void evaluateCommand(uint8_t c) {
	uint32_t i, tmp = 0, junk;
	uint8_t zczxczxczxc = 0;
	const char *build = __DATE__;

	switch (c) {
		// adding this message as a comment will return an error status for MSP_PRIVATE (end of switch), allowing third party tools to distinguish the implementation of this message
		//case MSP_PRIVATE:
		//  headSerialError();tailSerialReply(); // we don't have any custom msp currently, so tell the gui we do not use that
		//  break;
#if defined(CLEANFLIGHT)
	case MSP_API_VERSION:
		headSerialReply(1 + API_VERSION_LENGTH);
		serialize8(MSP_PROTOCOL_VERSION);
		serialize8(API_VERSION_MAJOR);
		serialize8(API_VERSION_MINOR);
		tailSerialReply();

		USE_CLEANFLIGHT_REPLIES = 1;

		break;
	case MSP_FC_VARIANT:
		headSerialReply(FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);

		for (i = 0; i < FLIGHT_CONTROLLER_IDENTIFIER_LENGTH; i++) {
			serialize8(flightControllerIdentifier[i]);
		}
		tailSerialReply();

		break;
	case MSP_FC_VERSION:
		headSerialReply(FLIGHT_CONTROLLER_VERSION_LENGTH);

		serialize8(FC_VERSION_MAJOR);
		serialize8(FC_VERSION_MINOR);
		serialize8(FC_VERSION_PATCH_LEVEL);
		tailSerialReply();

		break;

	case MSP_BOARD_INFO:
		headSerialReply(
			BOARD_IDENTIFIER_LENGTH +
			BOARD_HARDWARE_REVISION_LENGTH
			);
		for (i = 0; i < BOARD_IDENTIFIER_LENGTH; i++) {
			serialize8(boardIdentifier[i]);
		}

//#ifdef NAZE
//		serialize16(hardwareRevision);
//#else
		serialize16(0); // No other build targets currently have hardware revision detection.
//#endif

		tailSerialReply();

		break;

	case MSP_BF_BUILD_INFO: // BASEFLIGHT
		headSerialReply(11 + 4 + 4);
		for (i = 0; i < 11; i++)
			serialize8(build[i]); // MMM DD YYYY as ascii, MMM = Jan/Feb... etc
		serialize32(0); // future exp
		serialize32(0); // future exp
		tailSerialReply();

	case MSP_BUILD_INFO: // CLEANFLIGHT
		headSerialReply(11 + 4 + 4);
		for (i = 0; i < 11; i++)
			serialize8(build[i]); // MMM DD YYYY as ascii, MMM = Jan/Feb... etc
		serialize32(0); // future exp
		serialize32(0); // future exp

		/*headSerialReply(
			BUILD_DATE_LENGTH +
			BUILD_TIME_LENGTH +
			GIT_SHORT_REVISION_LENGTH
			);

		for (i = 0; i < BUILD_DATE_LENGTH; i++) {
			serialize8(buildDate[i]);
		}
		for (i = 0; i < BUILD_TIME_LENGTH; i++) {
			serialize8(buildTime[i]);
		}

		for (i = 0; i < GIT_SHORT_REVISION_LENGTH; i++) {
			serialize8(shortGitRevision[i]);
		}*/

		tailSerialReply();
		break;
#endif
	case MSP_SUPRESS_DATA_FROM_RX:
		supress_data_from_rx = read8();

		headSerialReply(1);
		serialize8((uint8_t)supress_data_from_rx);
		tailSerialReply();

		break;
	case MSP_SET_RAW_RC:
		s_struct_w((uint8_t*) &rcSerial, 16);
		rcSerialCount = 50; // 1s transition 
		break;
	case MSP_SET_PID:
		mspAck();
		s_struct_w((uint8_t*) &conf.pid[0].P8, 3 * PIDITEMS);
		break;
	case MSP_SET_BOX:
		mspAck();
#if EXTAUX
		s_struct_w((uint8_t*)&conf.activate[0],CHECKBOXITEMS*4);
#else
		s_struct_w((uint8_t*) &conf.activate[0], CHECKBOXITEMS * 2);
#endif
		break;
	case MSP_SET_RC_TUNING:
		mspAck();
		s_struct_w((uint8_t*) &conf.rcRate8, 7);
		break;
#if !defined(DISABLE_SETTINGS_TAB)
	case MSP_SET_MISC:
		struct {
			uint16_t a, b, c, d, e, f;
			uint32_t g;
			uint16_t h;
			uint8_t  i, j, k, l;
		} set_misc;
		mspAck();
		s_struct_w((uint8_t*) &set_misc, 22);
#if defined(POWERMETER)
		conf.powerTrigger1 = set_misc.a / PLEVELSCALE;
#endif
		conf.minthrottle = set_misc.b;
#ifdef FAILSAFE 
		conf.failsafe_throttle = set_misc.e;
#endif  
#if MAG
		conf.mag_declination = set_misc.h;
#endif
#if defined(VBAT)
		conf.vbatscale        = set_misc.i;
		conf.vbatlevel_warn1  = set_misc.j;
		conf.vbatlevel_warn2  = set_misc.k;
		conf.vbatlevel_crit   = set_misc.l;
#endif
		break;
	case MSP_MISC:
		struct {
#if defined(CLEANFLIGHT)
			uint16_t a, b, c, d, e;
			uint8_t f;
			uint8_t g;
			uint8_t h;

			uint8_t i;
			uint8_t j;
			uint8_t k;

			uint16_t l;

			uint8_t  m, n, o, p;
#else
			uint16_t a, b, c, d, e, f;
			uint32_t g;
			uint16_t h;
			uint8_t  i, j, k, l;
#endif
		} misc;
		misc.a = intPowerTrigger1;
		misc.b = conf.minthrottle;
		misc.c = MAXTHROTTLE;
		misc.d = MINCOMMAND;
#ifdef FAILSAFE 
		misc.e = conf.failsafe_throttle;
#else  
		misc.e = 0;
#endif

#if defined(CLEANFLIGHT)
#if GPS
		//serialize8(masterConfig.gpsConfig.provider); // gps_type
		//serialize8(0); // TODO gps_baudrate (an index, cleanflight uses a uint32_t
		//serialize8(masterConfig.gpsConfig.sbasMode); // gps_ubx_sbas
#else
		//f serialize8(0); // gps_type
		//g serialize8(0); // TODO gps_baudrate (an index, cleanflight uses a uint32_t
		//h serialize8(0); // gps_ubx_sbas
		misc.f = 0;
		misc.g = 1;
		misc.h = 0;
#endif
		// i serialize8(masterConfig.batteryConfig.multiwiiCurrentMeterOutput);
		// j serialize8(masterConfig.rxConfig.rssi_channel);
		// k serialize8(0);

		misc.i = 0;
		misc.j = 0;
		misc.k = 0;

	#if MAG
		misc.l = conf.mag_declination;
	#else
		misc.l = 0;
	#endif
	#ifdef VBAT
		misc.m = conf.vbatscale;
		misc.n = conf.vbatlevel_warn1;
		misc.o = conf.vbatlevel_warn2;
		misc.p = conf.vbatlevel_crit;
	#else
		misc.m = 0; misc.n = 0; misc.o = 0; misc.p = 0;
	#endif

		s_struct((uint8_t*) &misc, 32);
#else
	#ifdef LOG_PERMANENT
		misc.f = plog.arm;
		misc.g = plog.lifetime + (plog.armed_time / 1000000); // <- computation must be moved outside from serial
	#else
		misc.f = 0; misc.g = 0;
	#endif

	#if MAG
		misc.h = conf.mag_declination;
	#else
		misc.h = 0;
	#endif
	#ifdef VBAT
		misc.i = conf.vbatscale;
		misc.j = conf.vbatlevel_warn1;
		misc.k = conf.vbatlevel_warn2;
		misc.l = conf.vbatlevel_crit;
	#else
		misc.i = 0; misc.j = 0; misc.k = 0; misc.l = 0;
	#endif

		s_struct((uint8_t*) &misc, 22);
#endif
		
		break;
#endif
//#if defined (DYNBALANCE)
	case MSP_SET_MOTOR:
		mspAck();
		s_struct_w((uint8_t*)&motor_disarmed,16);

		break;
//#endif
#ifdef MULTIPLE_CONFIGURATION_PROFILES
	case MSP_SELECT_SETTING:
		if (!f.ARMED) {
			global_conf.currentSet = read8();
			if (global_conf.currentSet > 2) global_conf.currentSet = 0;
			writeGlobalSet(0);
			readEEPROM();
		}
		mspAck();
		break;
#endif
	case MSP_SET_HEAD:
		mspAck();
		s_struct_w((uint8_t*) &magHold, 2);
		break;
	case MSP_IDENT:
		struct {
			uint8_t v, t, msp_v;
			uint32_t cap;
		} id;
		id.v = VERSION;
		id.t = MULTITYPE;
		id.msp_v = MSP_VERSION;
		id.cap = (0 + BIND_CAPABLE) | DYNBAL << 2 | FLAP << 3 | NAVCAP << 4 | EXTAUX << 5 | ((uint32_t) NAVI_VERSION << 28); //Navi version is stored in the upper four bits; 
		s_struct((uint8_t*) &id, 7);
		break;
	case MSP_STATUS:
		struct {
			uint16_t cycleTime, i2c_errors_count, sensor;
			uint32_t flag;
			uint8_t set;
		} st;

		st.cycleTime = cycleTime;
		st.i2c_errors_count = i2c_errors_count;
		st.sensor = ACC | BARO << 1 | MAG << 2 | GPS << 3 | SONAR << 4;

#if ACC
		if (f.ANGLE_MODE)   tmp |= 1 << BOXANGLE;
		if (f.HORIZON_MODE) tmp |= 1 << BOXHORIZON;
#endif
#if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
		if (f.BARO_MODE) tmp |= 1 << BOXBARO;
#endif
		if (f.MAG_MODE) tmp |= 1 << BOXMAG;
#if !defined(FIXEDWING)
#if defined(HEADFREE)
		if(f.HEADFREE_MODE)       tmp |= 1<<BOXHEADFREE;
		if(rcOptions[BOXHEADADJ]) tmp |= 1<<BOXHEADADJ;
#endif
#endif
#if defined(SERVO_TILT) || defined(GIMBAL)|| defined(SERVO_MIX_TILT)
		if(rcOptions[BOXCAMSTAB]) tmp |= 1<<BOXCAMSTAB;
#endif
#if defined(CAMTRIG)
		if(rcOptions[BOXCAMTRIG]) tmp |= 1<<BOXCAMTRIG;
#endif
#if GPS
		switch (f.GPS_mode) {
		case GPS_MODE_HOLD:
			tmp |= 1 << BOXGPSHOLD;
			break;
		case GPS_MODE_RTH:
			tmp |= 1 << BOXGPSHOME;
			break;
		case GPS_MODE_NAV:
			tmp |= 1 << BOXGPSNAV;
			break;
		}
#endif
#if defined(FIXEDWING) || defined(HELICOPTER)
		if(f.PASSTHRU_MODE) tmp |= 1<<BOXPASSTHRU;
#endif
#if defined(BUZZER)
		if(rcOptions[BOXBEEPERON]) tmp |= 1<<BOXBEEPERON;
#endif
#if defined(LED_FLASHER)
		if(rcOptions[BOXLEDMAX]) tmp |= 1<<BOXLEDMAX;
		if(rcOptions[BOXLEDLOW]) tmp |= 1<<BOXLEDLOW;
#endif
#if defined(LANDING_LIGHTS_DDR)
		if(rcOptions[BOXLLIGHTS]) tmp |= 1<<BOXLLIGHTS;
#endif
#if defined(VARIOMETER)
		if(rcOptions[BOXVARIO]) tmp |= 1<<BOXVARIO;
#endif
#if defined(INFLIGHT_ACC_CALIBRATION)
		if (rcOptions[BOXCALIB]) tmp |= 1 << BOXCALIB;
#endif
#if defined(GOVERNOR_P)
		if(rcOptions[BOXGOV]) tmp |= 1<<BOXGOV;
#endif
#if defined(OSD_SWITCH)
		if(rcOptions[BOXOSD]) tmp |= 1<<BOXOSD;
#endif
#if defined(INFLIGHT_PID_TUNING)
		if (f.PIDTUNE_MODE) tmp |= 1 << BOXPIDTUNE;
#endif
#if SONAR
		if (f.SONAR_MODE) tmp |= 1 << BOXSONAR;
#endif
		if (f.ARMED) tmp |= 1 << BOXARM;
		st.flag = tmp;
		st.set = global_conf.currentSet;
		s_struct((uint8_t*) &st, 11);
		break;
	case MSP_RAW_IMU:
//#if defined(DYNBALANCE)
		for(uint8_t axis=0;axis<3;axis++) {imu.gyroData[axis]=imu.gyroADC[axis];imu.accSmooth[axis]= imu.accADC[axis];} // Send the unfiltered Gyro & Acc values to gui.
//#endif 
		s_struct((uint8_t*) &imu, 18);
		break;
	case MSP_SERVO:
		s_struct((uint8_t*) &servo, 16);
		break;
	case MSP_SERVO_CONF:
		s_struct((uint8_t*) &conf.servoConf[0].min, 56); // struct servo_conf_ is 7 bytes length: min:2 / max:2 / middle:2 / rate:1    ----     8 servo =>  8x7 = 56
		break;
	case MSP_SET_SERVO_CONF:
		mspAck();
		s_struct_w((uint8_t*) &conf.servoConf[0].min, 56);
		break;
	case MSP_MOTOR:
		s_struct((uint8_t*) &motor, 16);
		break;
	case MSP_ACC_TRIM:
		headSerialReply(4);
		s_struct_partial((uint8_t*) &conf.angleTrim[PITCH], 2);
		s_struct_partial((uint8_t*) &conf.angleTrim[ROLL], 2);
		tailSerialReply();
		break;
	case MSP_SET_ACC_TRIM:
		mspAck();
		s_struct_w((uint8_t*) &conf.angleTrim[PITCH], 2);
		s_struct_w((uint8_t*) &conf.angleTrim[ROLL], 2);
		break;
	case MSP_RC:
		s_struct((uint8_t*) &rcData, RC_CHANS * 2);
		break;
#if GPS
	case MSP_SET_RAW_GPS:
		struct {
			uint8_t a, b;
			int32_t c, d;
			int16_t e;
			uint16_t f;
		} set_set_raw_gps;
		mspAck();
		s_struct_w((uint8_t*) &set_set_raw_gps, 14);
		f.GPS_FIX = set_set_raw_gps.a;
		GPS_numSat = set_set_raw_gps.b;
		GPS_coord[LAT] = set_set_raw_gps.c;
		GPS_coord[LON] = set_set_raw_gps.d;
		GPS_altitude = set_set_raw_gps.e;
		GPS_speed = set_set_raw_gps.f;
		GPS_update |= 2;              // New data signalisation to GPS functions
		break;
	case MSP_RAW_GPS:
		struct {
			uint8_t a, b;
			int32_t c, d;
			int16_t e;
			uint16_t f, g;
		} msp_raw_gps;
		msp_raw_gps.a = f.GPS_FIX;
		msp_raw_gps.b = GPS_numSat;
		msp_raw_gps.c = GPS_coord[LAT];
		msp_raw_gps.d = GPS_coord[LON];
		msp_raw_gps.e = GPS_altitude;
		msp_raw_gps.f = GPS_speed;
		msp_raw_gps.g = GPS_ground_course;

		s_struct((uint8_t*) &msp_raw_gps, 16);
		break;
	case MSP_GPSSVINFO:
		headSerialReply(1 + (GPS_numCh * 4));
		
		serialize8(GPS_numCh);
		for (i = 0; i < GPS_numCh; i++){
			serialize8(GPS_svinfo_chn[i]);
			serialize8(GPS_svinfo_svid[i]);
			serialize8(GPS_svinfo_quality[i]);
			serialize8(GPS_svinfo_cno[i]);
		}
		tailSerialReply();

		break;
	case MSP_COMP_GPS:
		struct {
			uint16_t a;
			int16_t b;
			uint8_t c;
		} msp_comp_gps;
		msp_comp_gps.a = GPS_distanceToHome;
		msp_comp_gps.b = GPS_directionToHome;
		msp_comp_gps.c = GPS_update & 1;
		s_struct((uint8_t*) &msp_comp_gps, 5);
		break;
#if defined(USE_MSP_WP)
	case MSP_SET_NAV_CONFIG:
		mspAck();
		s_struct_w((uint8_t*)&GPS_conf,sizeof(GPS_conf));
		break;
	case MSP_NAV_CONFIG:
		s_struct((uint8_t*)&GPS_conf,sizeof(GPS_conf));
		break;
	case MSP_NAV_STATUS: // to move to struct transmission
		headSerialReply(7);
		serialize8(f.GPS_mode);
		serialize8(NAV_state);
		serialize8(mission_step.action);
		serialize8(mission_step.number);
		serialize8(NAV_error);
		serialize16( (int16_t)(target_bearing/100));
		//serialize16(magHold);
		tailSerialReply();
		break;
	case MSP_WP: // to move to struct transmission
	{
						uint8_t wp_no;
						uint8_t flag;
						bool    success;

						wp_no = read8(); //get the wp number  
						headSerialReply(21);
						if (wp_no == 0) { //Get HOME coordinates
							serialize8(wp_no);
							serialize8(mission_step.action);
							serialize32(GPS_home[LAT]);
							serialize32(GPS_home[LON]);
							flag = MISSION_FLAG_HOME;
						}
						if (wp_no == 255) { //Get poshold coordinates
							serialize8(wp_no);
							serialize8(mission_step.action);
							serialize32(GPS_hold[LAT]);
							serialize32(GPS_hold[LON]);
							flag = MISSION_FLAG_HOLD;
						}
						if ((wp_no>0) && (wp_no<255)) {
							if (NAV_state == NAV_STATE_NONE) {
								success = recallWP(wp_no);
								serialize8(wp_no);
								serialize8(mission_step.action);
								serialize32(mission_step.pos[LAT]);
								serialize32(mission_step.pos[LON]);
								if (success == true) flag = mission_step.flag;
								else flag = MISSION_FLAG_CRC_ERROR; //CRC error
							} else {
								serialize8(wp_no);
								serialize8(0);
								serialize32(GPS_home[LAT]);
								serialize32(GPS_home[LON]);
								flag = MISSION_FLAG_NAV_IN_PROG;
							}
						}
						serialize32(mission_step.altitude);
						serialize16(mission_step.parameter1);
						serialize16(mission_step.parameter2);
						serialize16(mission_step.parameter3);
						serialize8(flag);
						tailSerialReply();
	}
		break;
	case MSP_SET_WP: // to move to struct transmission
	{
							uint8_t wp_no = read8(); //Get the step number

							if (NAV_state == NAV_STATE_HOLD_INFINIT && wp_no == 255) { //Special case - during stable poshold we allow change the hold position
								mission_step.number = wp_no;
								mission_step.action = MISSION_HOLD_UNLIM; 
								uint8_t temp = read8();
								mission_step.pos[LAT] =  read32();
								mission_step.pos[LON] =  read32();
								mission_step.altitude =  read32();
								mission_step.parameter1 = read16();
								mission_step.parameter2 = read16();
								mission_step.parameter3 = read16();
								mission_step.flag     =  read8();
								if (mission_step.altitude != 0) set_new_altitude(mission_step.altitude);
								GPS_set_next_wp(&mission_step.pos[LAT], &mission_step.pos[LON], &GPS_coord[LAT], &GPS_coord[LON]);
								if ((wp_distance/100) >= GPS_conf.safe_wp_distance) NAV_state = NAV_STATE_NONE;
								else NAV_state = NAV_STATE_WP_ENROUTE;
								break;
							}
							if (NAV_state == NAV_STATE_NONE) { // The Nav state is not zero, so navigation is in progress, silently ignore SET_WP command)
								mission_step.number     =  wp_no;
								mission_step.action     =  read8();
								mission_step.pos[LAT]   =  read32();
								mission_step.pos[LON]   =  read32();
								mission_step.altitude   =  read32();
								mission_step.parameter1 = read16();
								mission_step.parameter2 = read16();
								mission_step.parameter3 = read16();
								mission_step.flag       =  read8();
								//It's not sure, that we want to do poshold change via mission planner so perhaps the next if is deletable
								/*
								if (mission_step.number == 255) //Set up new hold position via mission planner, It must set the action to MISSION_HOLD_INFINIT 
								{
								if (mission_step.altitude !=0) set_new_altitude(mission_step.altitude); //Set the altitude
								GPS_set_next_wp(&mission_step.pos[LAT], &mission_step.pos[LON], &GPS_coord[LAT], &GPS_coord[LON]);
								NAV_state = NAV_STATE_WP_ENROUTE; //Go to that position, then it will switch to poshold unlimited when reached
								}
								*/
								if (mission_step.number == 0) { //Set new Home position
									GPS_home[LAT] = mission_step.pos[LAT];
									GPS_home[LON] = mission_step.pos[LON];
								}
								if (mission_step.number >0 && mission_step.number<255)                      //Not home and not poshold, we are free to store it in the eprom
								if (mission_step.number <= getMaxWPNumber())                              // Do not thrash the EEPROM with invalid wp number
									storeWP();
								mspAck();
							}
	}
		break;
#endif //USE_MSP_WP
#endif //GPS
	case MSP_ATTITUDE:
		s_struct((uint8_t*) &att, 6);
		break;
	case MSP_ALTITUDE:
		s_struct((uint8_t*) &alt, 6);
		break;
	case MSP_ANALOG:
		s_struct((uint8_t*) &analog, 7);
		break;
	case MSP_RC_TUNING:
		s_struct((uint8_t*) &conf.rcRate8, 7);

		break;
	case MSP_PID:
		s_struct((uint8_t*) &conf.pid[0].P8, 3 * PIDITEMS);

		break;
	case MSP_PIDNAMES:
		serializeNames(pidnames);

		break;
#if defined(CLEANFLIGHT)
	case MSP_PID_CONTROLLER:
		headSerialReply(1);
		//s_struct((uint8_t *) PID_CONTROLLER, 7);
		serialize8(PID_CONTROLLER - 1);
		tailSerialReply();

		break;
#endif
	case MSP_BOX:
#if EXTAUX
		s_struct((uint8_t*)&conf.activate[0],4*CHECKBOXITEMS);
#else
		s_struct((uint8_t*) &conf.activate[0], 2 * CHECKBOXITEMS);
#endif
		break;
	case MSP_BOXNAMES:
		serializeNames(boxnames);
		break;
	case MSP_BOXIDS:
		headSerialReply(CHECKBOXITEMS);
		for (uint8_t i = 0; i < CHECKBOXITEMS; i++)
			serialize8(pgm_read_byte(&(boxids[i])));
		tailSerialReply();
		break;
	case MSP_MOTOR_PINS:
		s_struct((uint8_t*) &PWM_PIN, 8);
		break;
	case MSP_RESET_CONF:
		if (!f.ARMED) LoadDefaults();
		mspAck();
		break;
	case MSP_ACC_CALIBRATION:
		if (!f.ARMED) calibratingA = 512;
		mspAck();
		break;
#if GYRO
	case MSP_GYRO_CALIBRATION:
		if (!f.ARMED) calibratingG = 512;
		mspAck();
		break;
#endif
#if MAG
	case MSP_MAG_CALIBRATION:
		if (!f.ARMED) f.CALIBRATE_MAG = 1;
		mspAck();
		break;
#endif
#if defined(SPEK_BIND)
	case MSP_BIND:
		spekBind();
		mspAck();
		break;
#endif
	case MSP_EEPROM_WRITE:
		writeParams(0);
		mspAck();
		break;
	case MSP_DEBUG:
		s_struct((uint8_t*) &debug, 8);
		break;
#if defined(CLEANFLIGHT)
	case MSP_BF_CONFIG:
		// baseflight
		//headSerialReply(1 + 4 + 1 + 2 + 2 + 2 + 2 + 2 + 2 + 2);
		headSerialReply(1 + 4 + 1 + 2 + 2 + 2 + 2 + 2 + 2 + 2);
		serialize8((uint8_t) MULTITYPE); // QUADX
		
		// features
		serialize32(1 << 4 | 1 << 9 | 1 << 2); // MOTOR_STOP, FEATURE_SONAR, FEATURE_INFLIGHT_ACC_CAL
		//serialize32((uint32_t)0); // MOTOR_STOP, FEATURE_SONAR

		// rx provider
		serialize8((uint8_t) 0);

		// board alignment
		serialize16((uint16_t) 0);
		serialize16((uint16_t) 0);
		serialize16((uint16_t) 0);

		// battery
		serialize16((uint16_t) 0);
		serialize16((uint16_t) 0);

		// motor_pwn_rate
		serialize16((uint16_t) 0);

		// pitch and roll rate
		// serialize8((uint8_t) s_struct((uint8_t*) &, 7););
		serialize8((uint8_t) conf.rollPitchRate);
		serialize8((uint8_t) conf.rollPitchRate);

		/* ? baseflight
		serialize16(mcfg.currentscale);
		serialize16(mcfg.currentoffset);
		serialize16(mcfg.motor_pwm_rate);
		serialize8(cfg.rollPitchRate[0]);
		serialize8(cfg.rollPitchRate[1]);
		*/

		// battery
		/*serialize16((uint16_t) 0);
		serialize16((uint16_t) 0);*/

		tailSerialReply();

		break;
	case MSP_CF_SERIAL_CONFIG:
		headSerialReply(
			((sizeof(uint8_t) +sizeof(uint16_t) +(sizeof(uint8_t) * 4)) * 4)
			);
		for (i = 0; i < 4; i++) {
			serialize8(0);
			serialize16(0);
			serialize8(0);
			serialize8(0);
			serialize8(0);
			serialize8(0);
		}
		tailSerialReply();
		break;
	case MSP_UID:
		headSerialReply(12);
		serialize32(U_ID_0);
		serialize32(U_ID_1);
		serialize32(U_ID_2);
		tailSerialReply();
		break;
#endif
#ifdef DEBUGMSG
	case MSP_DEBUGMSG:
	{
							uint8_t size = debugmsg_available();
							if (size > 16) size = 16;
							headSerialReply(size);
							debugmsg_serialize(size);
							tailSerialReply();
	}
		break;
#endif
	default:  // we do not know how to handle the (valid) message, indicate error MSP $M!
		headSerialError(); tailSerialReply();
		break;
	}
}

// evaluate all other incoming serial data
void evaluateOtherData(uint8_t sr) {
#ifndef SUPPRESS_OTHER_SERIAL_COMMANDS
#if GPS
#if !defined(I2C_GPS)
	// on the GPS port, we must avoid interpreting incoming values for other commands because there is no
	// protocol protection as is with MSP commands
	// doing so with single chars would be prone to error.
	if (CURRENTPORT == GPS_SERIAL) return;
#endif
#endif
	switch (sr) {
		// Note: we may receive weird characters here which could trigger unwanted features during flight.
		//       this could lead to a crash easily.
		//       Please use if (!f.ARMED) where neccessary
#ifdef LCD_CONF
	case 's':
	case 'S':
		if (!f.ARMED) configurationLoop();
		break;
#endif
#ifdef LOG_PERMANENT_SHOW_AT_L
	case 'L':
		if (!f.ARMED) dumpPLog(1);
		break;
#endif
#if defined(LCD_TELEMETRY) && defined(LCD_TEXTSTAR)
	case 'A': // button A press
		toggle_telemetry(1);
		break;
	case 'B': // button B press
		toggle_telemetry(2);
		break;
	case 'C': // button C press
		toggle_telemetry(3);
		break;
	case 'D': // button D press
		toggle_telemetry(4);
		break;
	case 'a': // button A release
	case 'b': // button B release
	case 'c': // button C release
	case 'd': // button D release
		break;
#endif
#ifdef LCD_TELEMETRY
	case '0':
	case '1':
	case '2':
	case '3':
	case '4':
	case '5':
	case '6':
	case '7':
	case '8':
	case '9':
#ifndef SUPPRESS_TELEMETRY_PAGE_R
	case 'R':
#endif
#if defined(DEBUG) || defined(DEBUG_FREE)
	case 'F':
#endif
		toggle_telemetry(sr);
		break;
#endif // LCD_TELEMETRY
	}
#endif // SUPPRESS_OTHER_SERIAL_COMMANDS
}

void SerialWrite16(uint8_t port, int16_t val)
{
	CURRENTPORT = port;
	serialize16(val); UartSendData(port);
}


#ifdef DEBUGMSG
void debugmsg_append_str(const char *str) {
	while(*str) {
		debug_buf[head_debug++] = *str++;
		if (head_debug == DEBUG_MSG_BUFFER_SIZE) {
			head_debug = 0;
		}
	}
}

static uint8_t debugmsg_available() {
	if (head_debug >= tail_debug) {
		return head_debug-tail_debug;
	} else {
		return head_debug + (DEBUG_MSG_BUFFER_SIZE-tail_debug);
	}
}

static void debugmsg_serialize(uint8_t l) {
	for (uint8_t i=0; i<l; i++) {
		if (head_debug != tail_debug) {
			serialize8(debug_buf[tail_debug++]);
			if (tail_debug == DEBUG_MSG_BUFFER_SIZE) {
				tail_debug = 0;
			}
		} else {
			serialize8('\0');
		}
	}
}
#else
void debugmsg_append_str(const char *str) {};
#endif