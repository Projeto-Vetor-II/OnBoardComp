#include "inc/libvector.h"

using namespace std;

ofstream logs;

// char path[50];
//Criar thread de timer: usar tempo previsto de apogeu para acionar o paraquedas de qualquer maneira

int main()
{
	cout << setprecision(4) << fixed;
	logs << setprecision(4) << fixed;

	FlightState flightState = STATE_STABILIZATION;
	AccelFlag accelFlag = ACCEL_NEAR_ZERO;
	AltitudeFlag altitudeFlag = ALTITUDE_STATIONARY;
	ParachuteFlag parachuteFlag = PARACHUTE_DEACTIVATED;
	StabilityFlag stabilityFlag = STABILITY_UNSTABLE;

	initGpios();
	initValuesKf();
	initSensors();

	createPath();
	logs.open(path, ios::app);
	headerLogging();

	std::thread logSensorsT(logSensorsThread);

	
	std::thread checkBarometerT([&]() {
        checkBarometerThread(&altitudeFlag);
    });

	std::thread checkAccelT([&]() {
		checkAccelThread(&accelFlag);
	});
	

	cout << "Running \n";
	turnOnLedGreen();
	while (rc_get_state() != EXITING){}

	logSensorsT.join();
	checkBarometerT.join();
	checkAccelT.join();

	turnOffLedGreen();
	rc_mpu_power_off();
	rc_bmp_power_off();
	logs.close();

	return 0;
}

