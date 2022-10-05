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
#include </OnBoardComp/maincode/libvector.h>
#include <sys/stat.h>
#include <sys/types.h>

#define Nx 3
#define Ny 1
#define Nu 1
#define SAMPLE_RATE 200 // hz
#define DT (1.0 / SAMPLE_RATE)
#define ACCEL_LP_TC 20 * DT // fast LP filter for accel
#define PRINT_HZ 10
#define BMP_RATE_DIV 10 // optionally sample bmp less frequently than mpu

#define FS 50 // hz
#define new_bmp_rate_div 2

//#define DURACAO_JANELA  1
//#define TAM_JANELA      DURACAO_JANELA*BMP_RATE_DIV

#define TEMPO_DETECCAO 1 // TEM QUE SER MULTIPLO INTEIRO DA DURACAO_JANELA
#define COUNTER_MAX TEMPO_DETECCAO *BMP_RATE_DIV
#define TEMPO_ACIONAMENTO 2 // em segundos

static int running = 0;
// int *prtRunning = &running;
static rc_mpu_data_t mpu_data;
static rc_bmp_data_t bmp_data;
static rc_kalman_t kf = RC_KALMAN_INITIALIZER;
static rc_vector_t u = RC_VECTOR_INITIALIZER;
static rc_vector_t y = RC_VECTOR_INITIALIZER;
static rc_filter_t acc_lp = RC_FILTER_INITIALIZER;

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__((unused)) int dummy);
static void __dmp_handler(void);
int init_sensors(rc_kalman_t *, rc_matrix_t *, rc_matrix_t *, rc_matrix_t *, rc_matrix_t *, rc_matrix_t *, rc_matrix_t *, rc_bmp_data_t *, rc_mpu_data_t *, rc_mpu_config_t *, rc_filter_t *);

int main(void)
{
        // Inicializar BMP com pressao atmosferica a nivel do mar
        // rc_bmp_set_sea_level_pressure_pa(101900); //Mudar depois para receber essa informacao no lancamento

        rc_gpio_init(2, 3, GPIOHANDLE_REQUEST_OUTPUT);
        rc_gpio_set_value(2, 3, 0);

        rc_gpio_init(3, 1, GPIOHANDLE_REQUEST_INPUT);
        rc_gpio_init(3, 2, GPIOHANDLE_REQUEST_OUTPUT);
        rc_gpio_set_value(3, 2, 1);

        char commLinux[50];

        // Graba o timestamp de inicio do codigo e cria o nome do arquivo de logging
        int mytime = (int)time(NULL);
        char time_str[16];
        my_itoa(mytime, time_str);

        char path[50];
        strcpy(path, "//OnBoardComp//logging//");
        strcat(path, time_str);
        mkdir(path, S_IRWXU);
        strcat(path, "//");

        // Inicializando matriz de Kalman
        rc_mpu_config_t mpu_conf;
        rc_matrix_t F = RC_MATRIX_INITIALIZER;
        rc_matrix_t G = RC_MATRIX_INITIALIZER;
        rc_matrix_t H = RC_MATRIX_INITIALIZER;
        rc_matrix_t Q = RC_MATRIX_INITIALIZER;
        rc_matrix_t R = RC_MATRIX_INITIALIZER;
        rc_matrix_t Pi = RC_MATRIX_INITIALIZER;

        initvaluekf(&F, &G, &H, &Q, &R, &Pi, &u, &y);
        rc_bmp_set_sea_level_pressure_pa(101900); // Mudar depois para receber essa informacao no lancamento
        signal(SIGINT, __signal_handler);

        running = init_sensors(&kf, &F, &G, &H, &Q, &R, &Pi, &bmp_data, &mpu_data, &mpu_conf, &acc_lp);

        rc_mpu_set_dmp_callback(__dmp_handler);

        // Imprime cabecalhos
        headerbb(); // imprime cabecalho no console

        // Contador e tempo inicial
        long long unsigned int counter = 0;
        long long unsigned int initial_time;
        unsigned int n_iterations = 0;

        char pathNew[50];

        char sufix[20];

        // Abre arquivo de logging
        FILE *fp;

        int counter_samples = 0;
        double leitura_anterior, leitura_nova;

        int caindo = 0;
        int paraquedas_acionado = 0;
        int counter_ignitor = 0;
        int sinal_acionamento = 0;

        while (running)
        {

                if (n_iterations == 0)
                {
                        initial_time = rc_nanos_since_boot();
                        leitura_nova = bmp_data.alt_m;
                }

                if (n_iterations % (BMP_RATE_DIV) == 0)
                {
                        leitura_nova = bmp_data.alt_m;
                        if (leitura_nova < leitura_anterior)
                        {
                                counter_samples++;
                        }
                        else
                        {
                                counter_samples = 0;
                        }
                        if (counter_samples >= 7)
                        {
                                caindo = 1;
                                counter_samples = 0;
                        }

                        printf("Valor novo: %9.4f ---- Valor antigo: %9.4f ----- Caindo: %i ----- Counter Samples: %d\n", leitura_nova, leitura_anterior, caindo, counter_samples);

                        leitura_anterior = leitura_nova;
                }

                // paraquedas_acionado = checkIgnitor();

                if (caindo == 1 && paraquedas_acionado == 0)
                {
                        if (counter_ignitor < FS * TEMPO_ACIONAMENTO)
                        {
                                sinal_acionamento = 1;
                                rc_gpio_set_value(2, 3, 1);
                                counter_ignitor++;
                        }
                        else
                        {
                                // rc_gpio_set_value(3, 2, 0);
                                rc_gpio_set_value(2, 3, 0);
                                counter_ignitor = 0;
                                sinal_acionamento = 0;
                                paraquedas_acionado = 1;
                        }
                }

                printf("Caindo: %d ---- Paraquedas_acionado: %d ----- SINAL_TESTE: %d ------ counter_ignitor: %d\n", caindo, paraquedas_acionado, sinal_acionamento, counter_ignitor);
                logging(&kf, &bmp_data, &acc_lp, counter, path, pathNew, sufix, commLinux, FS, n_iterations, &fp);
                // console(&kf, &bmp_data, &acc_lp, counter);
                fflush(stdout);

                rc_nanosleep(1000000000 / FS - (rc_nanos_since_boot() - initial_time + 1185 - counter));
                counter = rc_nanos_since_boot() - initial_time + 1185;

                n_iterations++;
        }

        rc_mpu_power_off();
        rc_bmp_power_off();
        return 0;
}

static void __signal_handler(__attribute__((unused)) int dummy)
{
        running = 0;
        return;
}
static void __dmp_handler(void)
{
        int i;
        double accel_vec[3];
        static int bmp_sample_counter = 0;
        // make copy of acceleration reading before rotating
        for (i = 0; i < 3; i++)
                accel_vec[i] = mpu_data.accel[i];
        // rotate accel vector
        rc_quaternion_rotate_vector_array(accel_vec, mpu_data.dmp_quat);
        // do first-run filter setup
        if (kf.step == 0)
        {
                kf.x_est.d[0] = bmp_data.alt_m;
                rc_filter_prefill_inputs(&acc_lp, accel_vec[2] - 9.80665);
                rc_filter_prefill_outputs(&acc_lp, accel_vec[2] - 9.80665);
        }
        // calculate acceleration and smooth it just a tad
        rc_filter_march(&acc_lp, accel_vec[2] - 9.80665);
        u.d[0] = acc_lp.newest_output;
        // don't bother filtering Barometer, kalman will deal with that
        y.d[0] = bmp_data.alt_m;
        if (rc_kalman_update_lin(&kf, u, y))
                running = 0;
        // now check if we need to sample BMP this loop
        bmp_sample_counter++;
        if (bmp_sample_counter >= BMP_RATE_DIV)
        {
                // perform the i2c reads to the sensor, on bad read just try later
                if (rc_bmp_read(&bmp_data))
                        return;
                bmp_sample_counter = 0;
        }
        return;
}
