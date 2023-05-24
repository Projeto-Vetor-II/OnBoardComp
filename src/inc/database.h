#pragma once

#include <stdio.h>
#include <iostream>
#include <robotcontrol.h>
#include <time.h>
#include <signal.h>
#include <fstream>
#include <iomanip>
#include <string>
#include <thread>
#include <condition_variable>
#include <mutex>

#define PATH "/OnBoardComp/data/"
#define COEFF_PATH "/OnBoardComp/src/teste.csv"

#ifndef DATABASE_H_INCLUDED
#define DATABASE_H_INCLUDED

#define Nx 3
#define Ny 1
#define Nu 1
#define SAMPLE_RATE 200 // hz
#define DT (1.0 / SAMPLE_RATE)
#define ACCEL_LP_TC 20 * DT // fast LP filter for accel
#define PRINT_HZ 10
#define BMP_RATE_DIV 10 // optionally sample bmp less frequently than mpu

#define FS 50 // hz

#define SAMPLES_LIMIT 10      // number of samples
#define DIFF_ALTITUDE_JUMP 25 // jump

// CONSTANTS //
#define GRAVITY 9.80665


typedef enum {
    STATE_STABILIZATION,
    STATE_PREPARED_4_FLIGHT,
    STATE_ACCELERATED_FLIGHT,
    STATE_RETARDED_FLIGHT,
    STATE_FALL_NO_PARACHUTE,
    STATE_FALL_PARACHUTE_DECELERATE,
    STATE_FALL_PARACHUTE_TERMINAL_VELOCITY,
    STATE_LANDED,
    STATE_UNKNOWN
} FlightState;

typedef enum {
    ACCEL_NEAR_ZERO,
    ACCEL_NEAR_G,
    ACCEL_HIGH_POSITIVE,
    ACCEL_LOW_NEGATIVE,
    ACCEL_HIGH_NEGATIVE
} AccelFlag;

typedef enum {
    ALTITUDE_STATIONARY,
    ALTITUDE_RISING,
    ALTITUDE_FALLING
} AltitudeFlag;

typedef enum {
    PARACHUTE_DEACTIVATED,
    PARACHUTE_ACTIVATED
} ParachuteFlag;

typedef enum {
    STABILITY_ESTABLE,
    STABILITY_UNSTABLE
} StabilityFlag;

// CAM FLAGS //

#define CAM_ON 0
#define CAM_CAPTURE 1
#define CAM_ERROR 0xFF
#define CAM_OFF 3

// VARIABLES //

extern rc_mpu_data_t mpu_data;
extern rc_bmp_data_t bmp_data;
extern rc_kalman_t kf;
extern rc_vector_t u;
extern rc_vector_t y;
extern rc_filter_t acc_lp;
extern rc_mpu_config_t mpu_conf;
extern rc_matrix_t F;
extern rc_matrix_t G;
extern rc_matrix_t H;
extern rc_matrix_t Q;
extern rc_matrix_t R;
extern rc_matrix_t Pi;

extern char path[50];

extern int counter_samples_fall;
extern int counter_samples_rise;
extern double oldData, newData;

extern int counter_ignitor;
extern int ignitionSignal;

extern bool LOGGING_FLAG;
extern bool NEW_BAROMETER_DATA_FLAG;
extern bool NEW_ACCEL_DATA_FLAG;

extern std::mutex logMutex;
extern std::mutex accelDataMutex;
extern std::mutex barometerDataMutex;

extern std::condition_variable logCondition;
extern std::condition_variable accelDataCondition;
extern std::condition_variable barometerDataCondition;


#endif
