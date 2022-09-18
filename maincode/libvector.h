
#include <stdio.h>
#include <stdlib.h>
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
//#include </qvectoronboard/Robotics_Cape_Installer/libraries/rc_usefulincludes.h>
//#include </qvectoronboard/Robotics_Cape_Installer/libraries/roboticscape.h>


#ifndef LIBVECTOR_H_INCLUDED
#define LIBVECTOR_H_INCLUDED

//RC


//void get_initial_data(rc_kalman_t * kf, rc_filter_t *acc_lp)


void initvaluekf(rc_matrix_t *,rc_matrix_t *,rc_matrix_t *,rc_matrix_t *,rc_matrix_t *,rc_matrix_t *, rc_vector_t *, rc_vector_t *);
int init_sensors(rc_kalman_t *,rc_matrix_t *, rc_matrix_t *,rc_matrix_t *,rc_matrix_t *,rc_matrix_t *,rc_matrix_t *,rc_bmp_data_t *, rc_mpu_data_t *, rc_mpu_config_t *, rc_filter_t *);



//SIMPLIFICACOES


//FORMATACAO DE TEXTO

void headerbb(void);
void headerll(FILE **);

void logging(rc_kalman_t *, rc_bmp_data_t *, rc_filter_t *, long long unsigned int, char *, char* ,char*, char*, int, unsigned int, FILE **);
void console(rc_kalman_t *, rc_bmp_data_t *, rc_filter_t *, long long unsigned int);
//int ConcatenaNovaString(char* , char* , char*);
char *my_itoa(int num, char *str);


#endif
