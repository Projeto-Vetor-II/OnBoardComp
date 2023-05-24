#pragma once
#include "database.h"

#ifndef LIBVECTOR_H_INCLUDED
#define LIBVECTOR_H_INCLUDED

#define BMP_RATE_DIV 10     // optionally sample bmp less frequently than mpu
#define FS 50               // hz
#define TEMPO_ACIONAMENTO 2 // em segundos
/*
class CoeffDataManager
{
private:
    std::ifstream coeffDataFile(COEFF_PATH);
    int[3] coeffs;

public:
    CoeffDataManager();
    int *getCoeffs();
}
*/

void initValuesKf();
void initSensors();
void logSensorsThread();
void console();
void initGpios();
void turnOnLedGreen();
void turnOffLedGreen();
void turnOnLedRed();
void turnOffLedRed();
void createPath();
void checkBarometerThread(AltitudeFlag *);
void checkAccelThread(AccelFlag *);
void checkParachute(ParachuteFlag *);
void setState(AltitudeFlag, AccelFlag, ParachuteFlag, StabilityFlag, FlightState *);
void headerbb();
void headerll();
void headerLogging();
void pauseButton();
void ledState();

void on_pause_press();
void on_pause_release();
void __signal_handler(__attribute__((unused)) int dummy);
void __dmp_handler(void);
char *my_itoa(int, char *);

#endif
