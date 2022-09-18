
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <math.h> // for M_PI
#include <rc/math/kalman.h>
#include <rc/math/filter.h>
#include <rc/math/quaternion.h>
#include <rc/time.h>
#include <rc/bmp.h>
#include <rc/mpu.h>
#include <rc/gpio.h>
#include <time.h>


#ifndef LIBVECTOR_H_INCLUDED
#define LIBVECTOR_H_INCLUDED

//RC

void initvaluekf(rc_matrix_t *,rc_matrix_t *,rc_matrix_t *,rc_matrix_t *,rc_matrix_t *,rc_matrix_t *, rc_vector_t *, rc_vector_t *);
int init_sensors(rc_kalman_t *,rc_matrix_t *, rc_matrix_t *,rc_matrix_t *,rc_matrix_t *,rc_matrix_t *,rc_matrix_t *,rc_bmp_data_t *, rc_mpu_data_t *, rc_mpu_config_t *, rc_filter_t *);



//SIMPLIFICACOES


//FORMATACAO DE TEXTO


void headerbb(void);
void headerll(FILE *);

void logging(FILE *, rc_kalman_t *, rc_bmp_data_t *, rc_filter_t *,long long unsigned int);
void console (rc_kalman_t *, rc_bmp_data_t *, rc_filter_t *,long long unsigned int);

//Init sensors//
//static void __dmp_handler(rc_filter_t *, rc_mpu_config_t *, rc_bmp_data_t *, rc_kalman_t *,rc_vector_t *, rc_vector_t *);


char *my_itoa(int num, char *str);


#endif
