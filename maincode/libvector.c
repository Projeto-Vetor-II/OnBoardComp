
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

#include </OnBoardComp/maincode/libvector.h>

#define Nx 3
#define Ny 1
#define Nu 1
#define SAMPLE_RATE 200 // hz
#define DT (1.0 / SAMPLE_RATE)
#define ACCEL_LP_TC 20 * DT // fast LP filter for accel
#define PRINT_HZ 10
#define BMP_RATE_DIV 10 // optionally sample bmp less frequently than mpu

//#define FS              50     //hz

//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
// SIMPLIFICACOES

// static void __signal_handler(__attribute__ ((unused)) int dummy);

/*void get_initial_data(rc_kalman_t * kf, rc_filter_t *acc_lp){
        double pressure, altitude, acc, vel;

        pressure = rc_bmp_get_pressure_pa();
        altitude = rc_bmp_get_altitude_m();
        acc = acc_lp->newest_output;
        vel = kf->x_est.d[1];
        printf("\r");
        printf(" Altitude = %8.4fm|", altitude);
        printf(" Velocidade = %7.4fm/s|", vel);
        printf(" Pressão = %7.4fm/s^2|", pressure);
        printf(" Aceleração = %7.4fm/s^2|", acc);


}*/

void initvaluekf(rc_matrix_t *F, rc_matrix_t *G, rc_matrix_t *H, rc_matrix_t *Q, rc_matrix_t *R, rc_matrix_t *Pi, rc_vector_t *u, rc_vector_t *y)
{

        rc_matrix_zeros(F, Nx, Nx);
        rc_matrix_zeros(G, Nx, Nu);
        rc_matrix_zeros(H, Ny, Nx);
        rc_matrix_zeros(Q, Nx, Nx);
        rc_matrix_zeros(R, Ny, Ny);
        rc_matrix_zeros(Pi, Nx, Nx);
        rc_vector_zeros(u, Nu);
        rc_vector_zeros(y, Ny);

        F->d[0][0] = 1.0;
        F->d[0][1] = DT;
        F->d[0][2] = 0.0;
        F->d[1][0] = 0.0;
        F->d[1][1] = 1.0;
        F->d[1][2] = -DT; // subtract accel bias
        F->d[2][0] = 0.0;
        F->d[2][1] = 0.0;
        F->d[2][2] = 1.0; // accel bias state
        G->d[0][0] = 0.5 * DT * DT;
        G->d[0][1] = DT;
        G->d[0][2] = 0.0;
        H->d[0][0] = 1.0;
        H->d[0][1] = 0.0;
        H->d[0][2] = 0.0;
        // covariance matrices
        Q->d[0][0] = 0.000000001;
        Q->d[1][1] = 0.000000001;
        Q->d[2][2] = 0.0001; // don't want bias to change too quickly
        R->d[0][0] = 1000000.0;
        // initial P, cloned from converged P while running
        Pi->d[0][0] = 1258.69;
        Pi->d[0][1] = 158.6114;
        Pi->d[0][2] = -9.9937;
        Pi->d[1][0] = 158.6114;
        Pi->d[1][1] = 29.9870;
        Pi->d[1][2] = -2.5191;
        Pi->d[2][0] = -9.9937;
        Pi->d[2][1] = -2.5191;
        Pi->d[2][2] = 0.3174;
}

//--------------------------------------------------------------------------------------
// FUNCOES DE FORMATACAO DE TEXTO

void headerbb(void)
{

        printf("\r\n");
        printf("time|");
        printf(" altitude|");
        printf(" velocity|");
        printf(" accel_bias|");
        printf(" alt (bmp)|");
        printf(" vert_accel|");
        printf("\n");
}
void headerll(FILE **fp)
{

        // fflush(stdout);
        fprintf(*fp, "time,");
        fprintf(*fp, "altitude,");
        fprintf(*fp, "velocity,");
        fprintf(*fp, "accel_bias,");
        fprintf(*fp, "alt (bmp),");
        fprintf(*fp, "vert_accel,");
        fprintf(*fp, "\n");
}

int checkIgnitor(void)
{
        return !rc_gpio_get_value(3, 1);
}

void logging(rc_kalman_t *kf, rc_bmp_data_t *bmp_data, rc_filter_t *acc_lp, long long unsigned int counter, char *path, char *pathNew, char *sufix, char *commLinux, int FS, unsigned int n_iterations, FILE **fp)
{

        if (n_iterations % (FS) == 0)
        {
                my_itoa(n_iterations, sufix);
                strcat(sufix, ".csv");

                strcpy(pathNew, path);
                strcat(pathNew, sufix);

                if (n_iterations == 0)
                {
                        *fp = fopen(pathNew, "w");
                        headerll(fp);
                }
                else
                {
                        fclose(*fp);
                        char reserva[50];

                        my_itoa(n_iterations - FS, sufix);
                        strcat(sufix, ".csv");

                        strcpy(reserva, path);
                        strcat(reserva, sufix);

                        strcpy(commLinux, "sync -d ");
                        strcat(commLinux, reserva);
                        // printf("%s \n", commLinux);
                        //  char comteste[] = "pwd";
                        //  system(comteste);

                        // system("watch -n 1 grep -e Dirty: /proc/meminfo");

                        *fp = fopen(pathNew, "w");
                }
        }

        fprintf(*fp, "%6.9lf,", (double)counter / 1000000000);
        fprintf(*fp, "%8.4f,", kf->x_est.d[0]);
        fprintf(*fp, "%7.4f,", kf->x_est.d[1]);
        fprintf(*fp, "%7.4f,", kf->x_est.d[2]);
        fprintf(*fp, "%9.4f,", bmp_data->alt_m);
        fprintf(*fp, "%7.4f", acc_lp->newest_output);
        fprintf(*fp, "\n");
}

void console(rc_kalman_t *kf, rc_bmp_data_t *bmp_data, rc_filter_t *acc_lp, long long unsigned int counter)
{

        printf("\r");
        printf("%6.9lfs|", (double)counter / 1000000000);
        printf(" %8.4fm|", kf->x_est.d[0]);
        printf(" %7.4fm/s|", kf->x_est.d[1]);
        printf(" %7.4fm/s^2|", kf->x_est.d[2]);
        printf(" %9.4fm|", bmp_data->alt_m);
        printf(" %7.4fm/s^2|", acc_lp->newest_output);
}

int init_sensors(rc_kalman_t *kf, rc_matrix_t *F, rc_matrix_t *G, rc_matrix_t *H, rc_matrix_t *Q, rc_matrix_t *R, rc_matrix_t *Pi, rc_bmp_data_t *bmp_data, rc_mpu_data_t *mpu_data, rc_mpu_config_t *mpu_conf, rc_filter_t *acc_lp)
{

        if (rc_kalman_alloc_lin(kf, *F, *G, *H, *Q, *R, *Pi) == -1)
                return -1;
        // initialize the little LP filter to take out accel noise
        if (rc_filter_first_order_lowpass(acc_lp, DT, ACCEL_LP_TC))
                return -1;
        // set signal handler so the loop can exit cleanly
        // signal(SIGINT, __signal_handler);
        //*running = 1;
        // init barometer and read in first data
        printf("initializing barometer\n");
        if (rc_bmp_init(BMP_OVERSAMPLE_16, BMP_FILTER_16))
                return -1;
        if (rc_bmp_read(bmp_data))
                return -1;
        // init DMP
        printf("initializing DMP\n");
        *mpu_conf = rc_mpu_default_config();
        mpu_conf->dmp_sample_rate = SAMPLE_RATE;
        mpu_conf->dmp_fetch_accel_gyro = 1;
        if (rc_mpu_initialize_dmp(mpu_data, *mpu_conf))
                return -1;
        // wait for dmp to settle then start filter callback
        printf("waiting for sensors to settle");
        fflush(stdout);
        rc_usleep(500000);
        return 1;
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
