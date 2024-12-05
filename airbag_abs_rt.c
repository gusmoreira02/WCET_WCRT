// airbag_abs_rt.c
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sched.h>
#include <time.h>
#include <errno.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/mman.h>

// Estados dos sensores
volatile int airbag_sensor = 0; // Desativado por padrão
volatile int abs_sensor = 0;    // Desativado por padrão

// Mutex para sincronização
pthread_mutex_t mutex;

// Variáveis para métricas do Airbag
long long airbag_exec_time = 0;
long long airbag_wcet = 0;
long long airbag_response_time = 0;
long long airbag_wcrt = 0;
long long airbag_hwm = 0; // HWM rastreia o maior WCRT observado
int airbag_deadline_miss = 0;

// Variáveis para métricas do ABS
long long abs_exec_time = 0;
long long abs_wcet = 0;
long long abs_response_time = 0;
long long abs_wcrt = 0;
long long abs_hwm = 0; // HWM rastreia o maior WCRT observado
int abs_deadline_miss = 0;

// Função de temporização precisa
void sleep_until(struct timespec *ts, int period_ms) {
    ts->tv_sec += (ts->tv_nsec + period_ms * 1000000) / 1000000000;
    ts->tv_nsec = (ts->tv_nsec + period_ms * 1000000) % 1000000000;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ts, NULL);
}

// Função para obter o tempo atual em microssegundos
long long get_time_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)(ts.tv_sec) * 1000000LL + ts.tv_nsec / 1000LL;
}

// Função da thread do Airbag
void *airbag_thread(void *data) {
    struct timespec next_activation;
    struct sched_param param;

    // Inicializa os atributos da thread
    pthread_attr_t attr;
    pthread_attr_init(&attr);

    // Define o tamanho da pilha
    pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);

    // Define a política de escalonamento e prioridade
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    param.sched_priority = 90; // Prioridade alta
    pthread_attr_setschedparam(&attr, &param);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

    // Bloqueia a memória
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall falhou");
        exit(EXIT_FAILURE);
    }

    clock_gettime(CLOCK_MONOTONIC, &next_activation);
    const int period_ms = 100; // Período de 100 ms
    const long long deadline_us = 100000; // Deadline de 100 ms em microssegundos

    // Variáveis para (m,k)-firm
    int activation_count = 0;
    int deadline_miss_window = 0;
    const int K_TOTAL = 20; // k do (m,k)-firm
    const int M_FIRM = 10;   // m do (m,k)-firm

    // Variáveis para coleta de amostras
    int sample_count = 0;
    long long wcrt_samples[20]; // Armazena os WCRT das 20 amostras
    int wcrt_sample_index = 0;

    while (1) {
        long long release_time = get_time_us(); // Captura o tempo de liberação

        // Executa a tarefa
        pthread_mutex_lock(&mutex);
        int sensor_value = airbag_sensor;
        pthread_mutex_unlock(&mutex);

        if (sensor_value) {
            long long start_time = get_time_us(); // Captura o tempo de início da execução

            // Simula carga de trabalho
            volatile int i;
            for (i = 0; i < 1000000; ++i);

            long long end_time = get_time_us();

            // Calcula métricas
            long long exec_time = end_time - start_time;
            long long response_time = end_time - release_time;

            pthread_mutex_lock(&mutex);
            airbag_exec_time = exec_time;
            airbag_response_time = response_time;

            if (exec_time > airbag_wcet)
                airbag_wcet = exec_time;

            if (response_time > airbag_wcrt)
                airbag_wcrt = response_time;

            // Coleta de amostras para o HWM
            if (sample_count < 20) {
                wcrt_samples[wcrt_sample_index++] = response_time;
                sample_count++;

                if (sample_count == 20) {
                    // Encontra o maior WCRT das 20 amostras
                    long long max_wcrt_sample = 0;
                    for (int j = 0; j < 20; j++) {
                        if (wcrt_samples[j] > max_wcrt_sample)
                            max_wcrt_sample = wcrt_samples[j];
                    }

                    // Atualiza o HWM apenas se o novo máximo for maior que o HWM atual
                    if (max_wcrt_sample > airbag_hwm) {
                        airbag_hwm = max_wcrt_sample;
                        printf("Airbag HWM atualizado: %lld us\n", airbag_hwm);
                    }

                    wcrt_sample_index = 0;
                    sample_count = 0; // Reinicia a coleta
                }
            }

            // Verifica se houve perda de deadline
            if (response_time > deadline_us) {
                airbag_deadline_miss++;
                deadline_miss_window++;
            }
            pthread_mutex_unlock(&mutex);

            // printf("Airbag ativado.\n"); // Comentado para reduzir saída
        } else {
            pthread_mutex_lock(&mutex);
            airbag_exec_time = 0;
            airbag_response_time = 0;
            pthread_mutex_unlock(&mutex);

            // printf("Airbag inativo.\n"); // Comentado para reduzir saída
        }

        activation_count++;

        // Verifica se a janela (k ativações) foi concluída
        if (activation_count >= K_TOTAL) {
            pthread_mutex_lock(&mutex);
            printf("Airbag - Fator Skip: %d\n", deadline_miss_window);
            // Verifica se a tarefa atendeu ao (m,k)-firm
            if (deadline_miss_window <= (K_TOTAL - M_FIRM)) {
                printf("Airbag atendeu ao (m,k)-firm nesta janela.\n");
            } else {
                printf("Airbag NÃO atendeu ao (m,k)-firm nesta janela.\n");
            }
            activation_count = 0;
            deadline_miss_window = 0;
            pthread_mutex_unlock(&mutex);
        }

        // Aguarda até o próximo período
        sleep_until(&next_activation, period_ms);
    }

    pthread_exit(NULL);
}

// Função da thread do ABS
void *abs_thread(void *data) {
    struct timespec next_activation;
    struct sched_param param;

    // Inicializa os atributos da thread
    pthread_attr_t attr;
    pthread_attr_init(&attr);

    // Define o tamanho da pilha
    pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);

    // Define a política de escalonamento e prioridade
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    param.sched_priority = 80; // Prioridade alta
    pthread_attr_setschedparam(&attr, &param);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

    // Bloqueia a memória
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall falhou");
        exit(EXIT_FAILURE);
    }

    clock_gettime(CLOCK_MONOTONIC, &next_activation);
    const int period_ms = 100; // Período de 100 ms
    const long long deadline_us = 100000; // Deadline de 100 ms em microssegundos

    // Variáveis para (m,k)-firm
    int activation_count = 0;
    int deadline_miss_window = 0;
    const int K_TOTAL = 10; // k do (m,k)-firm
    const int M_FIRM = 8;   // m do (m,k)-firm

    // Variáveis para coleta de amostras
    int sample_count = 0;
    long long wcrt_samples[20]; // Armazena os WCRT das 20 amostras
    int wcrt_sample_index = 0;

    while (1) {
        long long release_time = get_time_us(); // Captura o tempo de liberação

        // Executa a tarefa
        pthread_mutex_lock(&mutex);
        int sensor_value = abs_sensor;
        pthread_mutex_unlock(&mutex);

        if (sensor_value) {
            long long start_time = get_time_us(); // Captura o tempo de início da execução

            // Simula carga de trabalho
            volatile int i;
            for (i = 0; i < 1000000; ++i);

            long long end_time = get_time_us();

            // Calcula métricas
            long long exec_time = end_time - start_time;
            long long response_time = end_time - release_time;

            pthread_mutex_lock(&mutex);
            abs_exec_time = exec_time;
            abs_response_time = response_time;

            if (exec_time > abs_wcet)
                abs_wcet = exec_time;

            if (response_time > abs_wcrt)
                abs_wcrt = response_time;

            // Coleta de amostras para o HWM
            if (sample_count < 20) {
                wcrt_samples[wcrt_sample_index++] = response_time;
                sample_count++;

                if (sample_count == 20) {
                    // Encontra o maior WCRT das 20 amostras
                    long long max_wcrt_sample = 0;
                    for (int j = 0; j < 20; j++) {
                        if (wcrt_samples[j] > max_wcrt_sample)
                            max_wcrt_sample = wcrt_samples[j];
                    }

                    // Atualiza o HWM apenas se o novo máximo for maior que o HWM atual
                    if (max_wcrt_sample > abs_hwm) {
                        abs_hwm = max_wcrt_sample;
                        printf("ABS HWM atualizado: %lld us\n", abs_hwm);
                    }

                    wcrt_sample_index = 0;
                    sample_count = 0; // Reinicia a coleta
                }
            }

            // Verifica se houve perda de deadline
            if (response_time > deadline_us) {
                abs_deadline_miss++;
                deadline_miss_window++;
            }
            pthread_mutex_unlock(&mutex);

            // printf("ABS ativado.\n"); // Comentado para reduzir saída
        } else {
            pthread_mutex_lock(&mutex);
            abs_exec_time = 0;
            abs_response_time = 0;
            pthread_mutex_unlock(&mutex);

            // printf("ABS inativo.\n"); // Comentado para reduzir saída
        }

        activation_count++;

        // Verifica se a janela (k ativações) foi concluída
        if (activation_count >= K_TOTAL) {
            pthread_mutex_lock(&mutex);
            printf("ABS - Fator Skip: %d\n", deadline_miss_window);
            // Verifica se a tarefa atendeu ao (m,k)-firm
            if (deadline_miss_window <= (K_TOTAL - M_FIRM)) {
                printf("ABS atendeu ao (m,k)-firm nesta janela.\n");
            } else {
                printf("ABS NÃO atendeu ao (m,k)-firm nesta janela.\n");
            }
            activation_count = 0;
            deadline_miss_window = 0;
            pthread_mutex_unlock(&mutex);
        }

        // Aguarda até o próximo período
        sleep_until(&next_activation, period_ms);
    }

    pthread_exit(NULL);
}

// Função para entrada do teclado
void *keyboard_input_thread(void *arg) {
    struct termios oldt, newt;
    int oldf;
    char c;

    // Configura o terminal para entrada não bloqueante
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // Desativa modo canônico e eco
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    printf("Pressione 'a' para alternar o sensor Airbag, 'b' para ABS, 'q' para sair.\n");

    while (1) {
        c = getchar();

        if (c != EOF) {
            if (c == 'a' || c == 'A') {
                pthread_mutex_lock(&mutex);
                airbag_sensor = !airbag_sensor;
                pthread_mutex_unlock(&mutex);
                printf("Sensor Airbag alternado para %d\n", airbag_sensor);
            } else if (c == 'b' || c == 'B') {
                pthread_mutex_lock(&mutex);
                abs_sensor = !abs_sensor;
                pthread_mutex_unlock(&mutex);
                printf("Sensor ABS alternado para %d\n", abs_sensor);
            } else if (c == 'q' || c == 'Q') {
                printf("Saindo do programa.\n");
                // Restaura as configurações do terminal
                tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
                fcntl(STDIN_FILENO, F_SETFL, oldf);
                exit(0);
            }
        }
        usleep(100000); // Dorme por 100 ms
    }
    return NULL;
}

// Função da thread de display
void *display_thread(void *arg) {
    while (1) {
        pthread_mutex_lock(&mutex);
        printf("\n----- Atualização do Display -----\n");
        printf("Airbag:\n");
        printf("  Estado: %s\n", airbag_sensor ? "Ativado" : "Desativado");
        printf("  Tempo de Execução: %lld us\n", airbag_exec_time);
        printf("  WCRT: %lld us\n", airbag_wcrt);
        printf("  Deadline: 100000 us\n");
        printf("  WCET: %lld us\n", airbag_wcet);
        printf("  HWM (WCRT Máximo): %lld us\n", airbag_hwm);
        printf("  Deadlines Perdidos: %d\n", airbag_deadline_miss);

        printf("ABS:\n");
        printf("  Estado: %s\n", abs_sensor ? "Ativado" : "Desativado");
        printf("  Tempo de Execução: %lld us\n", abs_exec_time);
        printf("  WCRT: %lld us\n", abs_wcrt);
        printf("  Deadline: 100000 us\n");
        printf("  WCET: %lld us\n", abs_wcet);
        printf("  HWM (WCRT Máximo): %lld us\n", abs_hwm);
        printf("  Deadlines Perdidos: %d\n", abs_deadline_miss);
        printf("-----------------------------------\n");
        pthread_mutex_unlock(&mutex);

        sleep(2); // Atualiza a cada 2 segundos
    }
    return NULL;
}

// Função da thread de alta prioridade (interferência)
void *high_priority_thread(void *data) {
    struct timespec next_activation;
    struct sched_param param;

    // Inicializa os atributos da thread
    pthread_attr_t attr;
    pthread_attr_init(&attr);

    // Define o tamanho da pilha
    pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);

    // Define a política de escalonamento e prioridade
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    param.sched_priority = 95; // Prioridade mais alta
    pthread_attr_setschedparam(&attr, &param);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

    clock_gettime(CLOCK_MONOTONIC, &next_activation);
    const int period_ms = 50; // Período de 50 ms

    while (1) {
        long long start_time = get_time_us();

        // Simula carga de trabalho
        volatile int i;
        for (i = 0; i < 5000000; ++i);

        long long end_time = get_time_us();

        // Aguarda até o próximo período
        sleep_until(&next_activation, period_ms);
    }

    pthread_exit(NULL);
}

// Função da thread de baixa prioridade (carga de trabalho)
void *low_priority_thread(void *data) {
    // Esta thread executa com prioridade padrão e consome CPU continuamente
    while (1) {
        // Simula carga de trabalho contínua
        volatile int i;
        for (i = 0; i < 10000000; ++i);
    }

    pthread_exit(NULL);
}

int main(int argc, char *argv[]) {
    pthread_t airbag_tid, abs_tid, keyboard_tid, display_tid;
    pthread_t high_priority_tid, low_priority_tid;
    int ret;

    // Inicializa o mutex
    pthread_mutex_init(&mutex, NULL);

    // Cria a thread de entrada do teclado
    ret = pthread_create(&keyboard_tid, NULL, keyboard_input_thread, NULL);
    if (ret) {
        fprintf(stderr, "Erro ao criar thread do teclado: %s\n", strerror(ret));
        exit(EXIT_FAILURE);
    }

    // Cria a thread de alta prioridade (interferência)
    ret = pthread_create(&high_priority_tid, NULL, high_priority_thread, NULL);
    if (ret) {
        fprintf(stderr, "Erro ao criar thread de alta prioridade: %s\n", strerror(ret));
        exit(EXIT_FAILURE);
    }

    // Cria a thread de baixa prioridade (carga de trabalho)
    ret = pthread_create(&low_priority_tid, NULL, low_priority_thread, NULL);
    if (ret) {
        fprintf(stderr, "Erro ao criar thread de baixa prioridade: %s\n", strerror(ret));
        exit(EXIT_FAILURE);
    }

    // Cria a thread do Airbag
    ret = pthread_create(&airbag_tid, NULL, airbag_thread, NULL);
    if (ret) {
        fprintf(stderr, "Erro ao criar thread do Airbag: %s\n", strerror(ret));
        exit(EXIT_FAILURE);
    }

    // Cria a thread do ABS
    ret = pthread_create(&abs_tid, NULL, abs_thread, NULL);
    if (ret) {
        fprintf(stderr, "Erro ao criar thread do ABS: %s\n", strerror(ret));
        exit(EXIT_FAILURE);
    }

    // Cria a thread do Display
    ret = pthread_create(&display_tid, NULL, display_thread, NULL);
    if (ret) {
        fprintf(stderr, "Erro ao criar thread do Display: %s\n", strerror(ret));
        exit(EXIT_FAILURE);
    }

    // Aguarda as threads (o programa continuará em execução)
    pthread_join(keyboard_tid, NULL);
    pthread_join(high_priority_tid, NULL);
    pthread_join(low_priority_tid, NULL);
    pthread_join(airbag_tid, NULL);
    pthread_join(abs_tid, NULL);
    pthread_join(display_tid, NULL);

    // Destroi o mutex
    pthread_mutex_destroy(&mutex);

    return 0;
}
