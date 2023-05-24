
#include "../inc/libvector.h"

using namespace std;

rc_mpu_data_t mpu_data;
rc_bmp_data_t bmp_data;
rc_kalman_t kf = RC_KALMAN_INITIALIZER;
rc_vector_t u = RC_VECTOR_INITIALIZER;
rc_vector_t y = RC_VECTOR_INITIALIZER;
rc_filter_t acc_lp = RC_FILTER_INITIALIZER;
rc_mpu_config_t mpu_conf;
rc_matrix_t F = RC_MATRIX_INITIALIZER;
rc_matrix_t G = RC_MATRIX_INITIALIZER;
rc_matrix_t H = RC_MATRIX_INITIALIZER;
rc_matrix_t Q = RC_MATRIX_INITIALIZER;
rc_matrix_t R = RC_MATRIX_INITIALIZER;
rc_matrix_t Pi = RC_MATRIX_INITIALIZER;

char path[50];

int counter_samples_fall = 0;
int counter_samples_rise = 0;
double oldData, newData;

int counter_ignitor = 0;
int ignitionSignal = 0;

extern ofstream logs;

std::mutex logMutex;
std::mutex accelDataMutex;
std::mutex barometerDataMutex;

std::condition_variable logCondition;
std::condition_variable accelDataCondition;
std::condition_variable barometerDataCondition;

bool LOGGING_FLAG = false;
bool NEW_BAROMETER_DATA_FLAG = false;
bool NEW_ACCEL_DATA_FLAG = false;

void console()
{      
        cout << '\r';
        cout << rc_nanos_since_epoch() << ",";
        cout << kf.x_est.d[0] << ",";
        cout << kf.x_est.d[1] << ",";
        cout << kf.x_est.d[2] << ",";
        cout << bmp_data.alt_m << ",";
        cout << mpu_data.accel[0] << ",";
        cout << mpu_data.accel[1] << ",";
        cout << mpu_data.accel[2] << ",";
        cout << acc_lp.newest_output << ",";
        cout << mpu_data.gyro[0] << ",";
        cout << mpu_data.gyro[1] << ",";
        cout << mpu_data.gyro[2];
}

void headerLogging()
{
        logs << "time,altitude,velocity,accel_bias,alt (bmp),accx,accy,accz,vert_accel,gyro[0],gyro[1],gyro[2]\n";
}

void createPath()
{
        int mytime = (int)time(NULL);
        char time_str[16];
        my_itoa(mytime, time_str);

        strcpy(path, PATH);
        strcat(path, time_str);
        strcat(path, ".csv");
}

void logSensorsThread() {
        while(rc_get_state() != EXITING)
        {
                std::unique_lock<std::mutex> lockLog(logMutex);
                logCondition.wait(lockLog, [] { return LOGGING_FLAG; });

                logs << rc_nanos_since_epoch() << ",";
                logs << kf.x_est.d[0] << ",";
                logs << kf.x_est.d[1] << ",";
                logs << kf.x_est.d[2] << ",";
                logs << bmp_data.alt_m << ",";
                logs << mpu_data.accel[0] << ",";
                logs << mpu_data.accel[1] << ",";
                logs << mpu_data.accel[2] << ",";
                logs << acc_lp.newest_output << ",";
                logs << mpu_data.gyro[0] << ",";
                logs << mpu_data.gyro[1] << ",";
                logs << mpu_data.gyro[2];
                logs << "\n";

                LOGGING_FLAG = false;
                logMutex.unlock();
        }
}

void checkBarometerThread(AltitudeFlag* altitudeFlag) {
        while(rc_get_state() != EXITING)
        {
                std::unique_lock<std::mutex> lockBarometerData(barometerDataMutex);
                barometerDataCondition.wait(lockBarometerData, [] { return NEW_BAROMETER_DATA_FLAG; });

                newData = bmp_data.alt_m;
                if (newData <= oldData - DIFF_ALTITUDE_JUMP)
                {
                        counter_samples_fall += 3;
                }
                else if (newData < oldData)
                {
                        counter_samples_fall++;
                }
                else
                {
                        counter_samples_fall = 0;
                }
                if (newData >= oldData + DIFF_ALTITUDE_JUMP)
                {
                        counter_samples_rise += 3;
                }
                else if (newData > oldData)
                {
                        counter_samples_rise++;
                }
                else
                {
                        counter_samples_rise = 0;
                }
                if (counter_samples_fall >= SAMPLES_LIMIT)
                {
                        *altitudeFlag = ALTITUDE_FALLING;
                        counter_samples_fall = 0;
                }
                else if (counter_samples_rise >= SAMPLES_LIMIT)
                {
                        *altitudeFlag = ALTITUDE_RISING;
                        counter_samples_rise = 0;
                }
                else
                {
                        *altitudeFlag = ALTITUDE_STATIONARY;
                }
                //cout << "Valor novo: " << newData << "----- Valor antigo: " << oldData << "----- altitudeFlag: " << unsigned(*altitudeFlag) << "---- CS_fall: " << counter_samples_fall << "--- CS_rise  " << counter_samples_rise << "\n";
                oldData = newData;

                NEW_BAROMETER_DATA_FLAG = false;
                lockBarometerData.unlock();
        
        }
}

void checkAccelThread(AccelFlag *accelFlag) {
        while(rc_get_state() != EXITING)
        {
                std::unique_lock<std::mutex> lockAccelData(accelDataMutex);
                accelDataCondition.wait(lockAccelData, [] { return NEW_ACCEL_DATA_FLAG; });

                if (acc_lp.newest_output <= 1 && acc_lp.newest_output >= -1)
                {
                        *accelFlag = ACCEL_NEAR_ZERO;
                }
                else if (acc_lp.newest_output <= -GRAVITY + 1 && acc_lp.newest_output >= -GRAVITY - 1)
                {
                        *accelFlag = ACCEL_NEAR_G;
                }
                else if (acc_lp.newest_output > 1)
                {
                        *accelFlag = ACCEL_HIGH_POSITIVE;
                }
                else if (acc_lp.newest_output <= -1 && acc_lp.newest_output >= -GRAVITY + 1)
                {
                        *accelFlag = ACCEL_LOW_NEGATIVE;
                }
                else
                {
                        *accelFlag = ACCEL_HIGH_NEGATIVE;
                }

                NEW_ACCEL_DATA_FLAG = false;
                lockAccelData.unlock();
        
        }
}

void checkParachute(ParachuteFlag *parachuteFlag)
{
        if (rc_gpio_get_value(3, 1) == 1)
        {
                *parachuteFlag = PARACHUTE_ACTIVATED;
        }
        else
        {
                *parachuteFlag = PARACHUTE_DEACTIVATED;
        }
        // GPIO retorna -1 em caso de erro.
        // Criar modo de emergência caso a velocidade esteja muito alta e em queda -> paraquedas não foi acionado devido
        // a um erro de GPIO
}


void setState(AltitudeFlag altitudeFlag, AccelFlag accelFlag, ParachuteFlag parachuteFlag, StabilityFlag stabilityFlag, FlightState *flightState)
{
        if (parachuteFlag == PARACHUTE_DEACTIVATED && altitudeFlag == ALTITUDE_STATIONARY && accelFlag == ACCEL_NEAR_ZERO && stabilityFlag == STABILITY_UNSTABLE)
        {
                *flightState = STATE_STABILIZATION; 
        }
        else if (parachuteFlag == PARACHUTE_DEACTIVATED && altitudeFlag == ALTITUDE_STATIONARY && accelFlag == ACCEL_NEAR_ZERO && stabilityFlag == STABILITY_ESTABLE)
        {
                *flightState = STATE_PREPARED_4_FLIGHT; 
        }
        else if (parachuteFlag == PARACHUTE_DEACTIVATED && stabilityFlag == STABILITY_ESTABLE && altitudeFlag == ALTITUDE_RISING && accelFlag == ACCEL_HIGH_POSITIVE)
        {
                *flightState = STATE_ACCELERATED_FLIGHT; 
        }
        else if (parachuteFlag == PARACHUTE_DEACTIVATED && stabilityFlag == STABILITY_ESTABLE && altitudeFlag == ALTITUDE_RISING && (accelFlag == ACCEL_NEAR_G || accelFlag == ACCEL_LOW_NEGATIVE))
        {
                *flightState = STATE_RETARDED_FLIGHT; 
        }
        else if (parachuteFlag == PARACHUTE_DEACTIVATED && stabilityFlag == STABILITY_ESTABLE && altitudeFlag == ALTITUDE_FALLING && accelFlag == ACCEL_NEAR_G)
        {
                *flightState = STATE_FALL_NO_PARACHUTE; 
        }
        else if (parachuteFlag == PARACHUTE_ACTIVATED && stabilityFlag == STABILITY_ESTABLE && altitudeFlag == ALTITUDE_FALLING && accelFlag == ACCEL_LOW_NEGATIVE)
        {
                *flightState = STATE_FALL_PARACHUTE_DECELERATE; 
        }
        else if (parachuteFlag == PARACHUTE_ACTIVATED && stabilityFlag == STABILITY_ESTABLE && altitudeFlag == ALTITUDE_FALLING && accelFlag == ACCEL_NEAR_ZERO)
        {
                *flightState = STATE_FALL_PARACHUTE_TERMINAL_VELOCITY; 
        }
        else if (parachuteFlag == PARACHUTE_ACTIVATED && stabilityFlag == STABILITY_ESTABLE && altitudeFlag == ALTITUDE_STATIONARY && accelFlag == ACCEL_NEAR_ZERO)
        {
                *flightState = STATE_LANDED; 
        }
        else
        {
                *flightState = STATE_UNKNOWN; 
        }
}

/*
void parachute_triggering()
{
        if (falling == true && parachuteOpen == false)
        {
                if (counter_ignitor < FS * TEMPO_ACIONAMENTO)
                {
                        ignitionSignal = 1;
                        turnon_ledred();
                        counter_ignitor++;
                }
                else
                {
                        // rc_gpio_set_value(3, 2, 0);
                        // rc_gpio_set_value(2, 3, 0);

                        turnoff_ledred();
                        counter_ignitor = 0;
                        ignitionSignal = 0;
                        parachuteOpen = true;
                }
        }
}
*/


// PAUSE SECTION //
void ledState()
{
        if (rc_get_state() != EXITING)
        {
                turnOnLedGreen();
        }
}

char *my_itoa(int num, char *str)
{
        if (str == NULL)
        {
                return NULL;
        }
        sprintf(str, "%d", num);
        return str;
}
