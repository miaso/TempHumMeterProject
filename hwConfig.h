/* 
 * File:   readerConfig.h
 * Author: Dan
 *
 * Created on 01 mai 2015, 09:48
 */

#ifndef HWCONFIG_H
#define	HWCONFIG_H

#ifdef	__cplusplus
extern "C" {
#endif
/******************************************************************************/
// defines
/******************************************************************************/
#define KEEPAWAKE_INTERVAL_MIN              2           // 2 sec
#define KEEPAWAKE_INTERVAL_MAX              300         // 5 min
#define KEEPAWAKE_INTERVAL_DEFAULT          3           // 3 sec

#define DATA_STORE_INTERVAL_MIN             15          // 15 sec
#define DATA_STORE_INTERVAL_MAX             86400       // seconds = 1 day
#define DATA_STORE_INTERVAL_DEFAULT         900         // 15 min

#define REALTIME_INTERVAL_MIN               60          // 1  min
#define REALTIME_INTERVAL_MAX               3600        // 1 hour
#define REALTIME_INTERVAL_DEFAULT           900         // 15 min

#define WAKEUP_INTERVAL_MIN                 60          // 1  min
#define WAKEUP_INTERVAL_MAX                 86400       // seconds = 1 day
#define WAKEUP_INTERVAL_DEFAULT             900         // 15 min

#define METER_REPORT_INTERVAL_MIN           10          // 10 sec
#define METER_REPORT_INTERVAL_DEFAULT       30          // 30 sec
#define METER_REPORT_INTERVAL_MAX           300         // 2 min
    
#define PULSE_FACTOR_DEFAULT                100
/******************************************************************************/
// variables
/******************************************************************************/

/******************************************************************************/
// defines
/******************************************************************************/
void hwInit(void);
void enableRTCC(void);
void swInit(void);
void disableHw(void);

void loadDefaultConfig(void);

void clearSensorConfig(void);
void clearHistoricalData(void);
void clearDetectionParams(void);
void clearzWaveNetworkSettings(void);

void saveConfig(void);
void saveRegister(uint16_t reg);

void sensorConfigNone(void);
void sensorConfigPulse(void);
void sensorConfigAnalog(void);
void sensorConfigLed(void);
void changeSensorConfig(void);

void syncCounters(void);
void configureIntervals(void);
void configureTariffs(void);

uint16_t getConfigRegister(uint16_t reg, uint16_t *idx, uint8_t *buffer);

/******************************************************************************/
#ifdef	__cplusplus
}
#endif

#endif	/* READERCONFIG_H */

