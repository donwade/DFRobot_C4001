/*!
 * @file  mRangeVelocity.ino
 * @brief  radar measurement demo
 * @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author ZhixinLiu(zhixin.liu@dfrobot.com)
 * @version V1.0
 * @date 2024-02-02
 * @url https://github.com/dfrobot/DFRobot_C4001
 */
#ifdef ARDUINO_M5STACK_CORE2
#include <M5Unified.h>
#endif

#include "DFRobot_C4001.h"

//#define I2C_COMMUNICATION  //use I2C for communication, but use the serial port for communication if the line of codes were masked

#ifdef  I2C_COMMUNICATION
/*
 * DEVICE_ADDR_0 = 0x2A     default iic_address
 * DEVICE_ADDR_1 = 0x2B
 */
DFRobot_C4001_I2C radar(&Wire, DEVICE_ADDR_0);
#else
/* ---------------------------------------------------------------------------------------------------------------------
 *    board   |             MCU                | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |   m0  |
 *     VCC    |            3.3V/5V             |        VCC           |    VCC    |   VCC   |  VCC  |     X      |  vcc  |
 *     GND    |              GND               |        GND           |    GND    |   GND   |  GND  |     X      |  gnd  |
 *     RX     |              TX                |     Serial1 TX1      |     5     |   5/D6  |  D2   |     X      |  tx1  |
 *     TX     |              RX                |     Serial1 RX1      |     4     |   4/D7  |  D3   |     X      |  rx1  |
 * ----------------------------------------------------------------------------------------------------------------------*/
/* Baud rate cannot be changed */
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
SoftwareSerial mySerial(4, 5);
DFRobot_C4001_UART radar(&mySerial, 9600);
#elif defined(ESP32)
DFRobot_C4001_UART radar(&Serial1, 9600, /*rx*/ 13, /*tx*/ 14);
#else
DFRobot_C4001_UART radar(&Serial1, 9600);
#endif
#endif

void ShowConfig(const char *message)
{
	Serial.printf("\n--------------------------------\n%s -----\n", message);
    // show current  params

    sSensorStatus_t data;
    data = radar.getStatus();
    //  0 stop  1 start
    Serial.printf("work status  = %s\n", data.workStatus ? "RUNNING":"STOPPED");

    //  0 is exist   1 speed
    Serial.printf("work mode  = %s\n", data.workMode ? "DETECT":"SPEED");

    //  0 no init    1 init success
    Serial.printf("init status = %s\n", data.initStatus ? "DONE":"FAILED");

    Serial.printf("min range = %d\n", radar.getTMinRange());
    Serial.printf("max range = %d\n", radar.getTMaxRange());
    Serial.printf("threshold range = %d\n", radar.getThresRange());
    Serial.printf("fretting detection = %d\n\n", radar.getFrettingDetection());
}

void setup()
{

#ifdef ARDUINO_M5STACK_CORE2
    M5.begin();
#endif

    Serial.begin(115200);

    while (!Serial);

    while (!radar.begin())
    {
        Serial.println("NO 4001 found!");
        delay(1000);
    }

    Serial.println("4001 connected!");

	ShowConfig("BEFORE config");

    // speed Mode
    radar.setSensorMode(eSpeedMode);

    /*
     * min Detection range Minimum distance, unit cm, range 0.3~20m (30~2000),
     * not exceeding max, otherwise the function is abnormal.
     *
     * max Detection range Maximum distance, unit cm, range 2.4~20m (240~2000)

     * thres Target detection threshold, dimensionless unit 0.1, range 0~6553.5 (0~65535)
     */

#define MIN_DETECTION_RANGE 30
#define MAX_DETECTION_RANGE 2000
#define THRESHOLD_VALUE 10

// used for detection, not speed
#define TRIGGER_RANGE (MAX_DETECTION_RANGE -1)
     
    if (radar.setDetectThres(/*min*/ MIN_DETECTION_RANGE,
    						 /*max*/ MAX_DETECTION_RANGE, 
    						 /*thres*/ THRESHOLD_VALUE))
        Serial.println("set detect threshold successfully");

    // set Fretting Detection (aka micro gestures)
    radar.setFrettingDetection(eOFF);

	ShowConfig("AFTER config");
	delay(5000);

}

typedef struct {
	float lastEnergyDb;
	uint32_t onTime;
} TARGET_DATA;

#define MAX_TARGETS 20
TARGET_DATA targetDb[MAX_TARGETS];

void loop()
{
	static float maxDbFound = -100;
	static float minDbFound = 0;
	static uint32_t maxAbsFound = 0;
	static uint32_t minAbsFound = 0xFFFFFFFF;

	uint8_t targetNumber = radar.getTargetNumber();
	
	assert ( targetNumber < MAX_TARGETS);

	uint32_t energyNowAbs = radar.getTargetEnergy();
    //Serial.printf("target energy    = %d\n\n", energyNowAbs);

	if (energyNowAbs)
	{
	    float dbNow = radar.getTargetEnergyDb();
	    
	    if (dbNow > maxDbFound) maxDbFound = dbNow;
	    if (dbNow < minDbFound) minDbFound = dbNow;

	    if (energyNowAbs > maxAbsFound) maxAbsFound = energyNowAbs;
	    if (energyNowAbs < minAbsFound) minAbsFound = energyNowAbs;


	    if (dbNow == targetDb[targetNumber].lastEnergyDb) return;

		Serial.printf("target number = %d\n", targetNumber); // must exist

	    targetDb[targetNumber].lastEnergyDb = dbNow;
	    
		Serial.printf("target Speed  = %7.4f m/s \n", radar.getTargetSpeed());
		Serial.printf("target range  = %5.3f m\n", radar.getTargetRange());
	    Serial.printf("%5.3f db < %5.3f db < %5.3f db\n", minDbFound, dbNow, maxDbFound);
	    Serial.printf("%d < %d  < %d (0x%X)\n", minAbsFound, energyNowAbs, maxAbsFound, maxAbsFound);
		Serial.println();
		if (!targetDb[targetNumber].onTime) targetDb[targetNumber].onTime = millis();
			    
	}
	else
	{
		// target reports no energy. report and close it.
		//Serial.printf("%d = %d\n", targetNumber, energyNowAbs);
		if (targetDb[targetNumber].onTime) 
		{
			uint32_t deltaT = millis() - targetDb[targetNumber].onTime;
			targetDb[targetNumber].onTime = 0;
			Serial.printf("target %d ran for %d mS\n\n", targetNumber, deltaT);
		}
	    targetDb[targetNumber].lastEnergyDb = 0.0;
	}
	
    delay(100);
}
