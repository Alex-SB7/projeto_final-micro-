#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include <stdlib.h>
#include "inc/ssd1306.h"
#include "inc/font.h"
#include <stdio.h>
#include <math.h>
#include "hardware/adc.h"

//flags de interrupção
int p_flag_1 = 0;
int p_flag_2 = 0;

// Variáveis para debouncing
static volatile uint a = 1;
static volatile uint ab = 1;
static volatile uint32_t last_time = 0; // Armazena o tempo do último evento (em microssegundos)

//Variáveis do joystick
int t_max = 2000.0, t_out = 0.0, 
    t_min = 0.0, js_max = 4096.0, 
    jsy_in = 0.0, jsx_in = 0.0, js_min = 0.0,
    v_max = 2.0, v_out = 0.0, v_min = 50.0;

char var_v_string[20], var_t_string[20];

//Variáveis da comunicação I2C
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define ENDERECO 0x3C

//Variáveis da comunicação UART
#define UART_ID uart0
#define UART_TX_PIN 17
#define UART_RX_PIN 16

//Variáveis do PWM
#define pin_buzzer 21
double var_div_clock = 35.0;
#define CLOCK_PWM_HZ 125000000

//Variáveis da simulação de FFT
#define FFT_BINS 44
#define TEMPOS 128
#define NUM_BARRAS 4
const int posicoes_x[NUM_BARRAS] = {0, 35, 70, 105};
const int largura_barra = 21;
uint8_t intensidades[NUM_BARRAS] = {0};

// Definição dos wraps das notas - base A4 = 440 Hz, usando div_clock = 35.0
#define nota_REST 0

// Oitava 2
#define nota_C2   54601
#define nota_CS2  51536
#define nota_D2   48644
#define nota_DS2  45917
#define nota_E2   43337
#define nota_F2   40905
#define nota_FS2  38610
#define nota_G2   36443
#define nota_GS2  34397
#define nota_A2   32468
#define nota_AS2  30646
#define nota_B2   28925

// Oitava 3
#define nota_C3   27302
#define nota_CS3  25770
#define nota_D3   24324
#define nota_DS3  22959
#define nota_E3   21670
#define nota_F3   20454
#define nota_FS3  19305
#define nota_G3   18222
#define nota_GS3  17199
#define nota_A3   16234
#define nota_AS3  15323
#define nota_B3   14463

// Oitava 4
#define nota_C4   13651
#define nota_CS4  12885
#define nota_D4   12162
#define nota_DS4  11479
#define nota_E4   10835
#define nota_F4   10227
#define nota_FS4  9653
#define nota_G4   9111
#define nota_GS4  8600
#define nota_A4   8117
#define nota_AS4  7661
#define nota_B4   7231

// Oitava 5
#define nota_C5   6825
#define nota_CS5  6442
#define nota_D5   6081
#define nota_DS5  5740
#define nota_E5   5417
#define nota_F5   5113
#define nota_FS5  4826
#define nota_G5   4555
#define nota_GS5  4300
#define nota_A5   4058
#define nota_AS5  3831 
#define nota_B5   3614 

// Oitava 6
#define nota_C6   3410 
#define nota_CS6  3219 
#define nota_D6   3039 
#define nota_DS6  2868 
#define nota_E6   2707 
#define nota_F6   2556 
#define nota_FS6  2412 
#define nota_G6   2277 
#define nota_GS6  2149 
#define nota_A6   2028 
#define nota_AS6  1914 
#define nota_B6   1806 

// Oitava 7
#define nota_C7   1705

//Musicas
//Musicas
int undertale_notas[] = {
    // Parte 1
nota_F4,nota_REST,nota_A4,nota_REST,nota_G4,
nota_E4,nota_REST,nota_G4,nota_REST,nota_F4,
nota_D4,nota_REST,nota_F4,nota_REST,nota_E4,
nota_C4,nota_REST,nota_D4,nota_REST,nota_A3,
nota_F4,nota_REST,nota_A4,nota_REST,nota_G4,
nota_C5,nota_REST,nota_G4,nota_REST,nota_A4,
nota_REST,nota_A4,nota_REST,nota_G4,nota_REST,
nota_A4,nota_REST,nota_C5,nota_REST,nota_D5,
nota_A4,nota_REST,nota_E5,nota_F5,nota_E5,
nota_D5,nota_C5,nota_REST,nota_E5,nota_REST,
nota_D5,nota_REST,nota_F5,nota_REST,nota_A5,
nota_REST,nota_G5,nota_A5,nota_D5,nota_E5,
nota_F5,nota_A5,nota_REST,

// Parte 2
nota_F5,nota_REST,nota_A5,nota_REST,nota_G5,
nota_A5,nota_D5,nota_E5,nota_C6,nota_A5,
nota_REST,nota_A5,nota_REST,nota_A5,nota_REST,
nota_AS5,nota_REST, nota_AS5,nota_REST,nota_C6,
nota_D6,nota_D5, nota_F5,nota_G5,nota_A5,nota_C6,
nota_D6, nota_F6,nota_E6,nota_C6,nota_A5,nota_AS5,
nota_A5,nota_G5,nota_REST,nota_F5,nota_REST,
nota_A5,nota_REST,nota_G5,nota_A5,nota_D5,
nota_E5,nota_F5,nota_A5,nota_REST,
nota_F5,nota_REST,nota_A5,nota_REST,nota_G5,nota_A5,
nota_D5,nota_E5,nota_C6,nota_A5,nota_REST,
nota_A5,nota_REST,nota_A5,nota_REST,nota_AS5,
nota_REST,nota_AS5,nota_REST,nota_C6,nota_D6,
nota_D5,nota_F5,nota_G5,nota_A5,nota_C6,
nota_D6,nota_F6,nota_E6,nota_C6,nota_A5,
nota_AS5,nota_A5,nota_G5,nota_REST,

// Parte 3
nota_F6,nota_REST,
nota_F6,nota_REST,nota_E6,nota_REST,
nota_E6,nota_D6,nota_F6,nota_E6,nota_F6,
nota_D6,nota_REST,nota_D6,nota_E6,nota_F6,
nota_C7,nota_A6,nota_REST,nota_D6,nota_F6,
nota_E6,nota_F6,nota_D6,nota_REST,nota_D6,
nota_E6,nota_F6,nota_C7,nota_A6,nota_REST,
nota_D6,nota_F6,nota_E6,nota_F6,nota_D6,
nota_REST,nota_D6,nota_E6,nota_F6,nota_C7,
nota_A6,nota_REST,nota_C6,nota_REST,nota_G6,
nota_F6,nota_E6,nota_D6,nota_REST,nota_C6,
nota_E6,nota_REST,nota_D6,nota_F6,nota_E6,
nota_F6,nota_D6,nota_REST,nota_D6,nota_E6,
nota_F6,nota_C7,nota_A6,nota_REST,nota_D6,
nota_F6,nota_E6,nota_F6,nota_D6,nota_REST,
nota_D6,nota_E6,nota_F6,nota_C7,nota_A6,
nota_REST,nota_D6,nota_F6,nota_E6,nota_F6,
nota_D6,nota_REST,nota_D6,nota_E6,nota_F6,
nota_C7,nota_A6,nota_REST,nota_C6,nota_REST,
nota_G6,nota_F6,nota_E6,nota_D6,nota_REST,
nota_C6,nota_E6

};

int undertale_tempos[] = {
  // Parte 1
250,125,250,125,800,
250,125,250,125,800,
250,125,250,125,800,
250,125,250,125,800,
250,125,250,125,800,
250,125,250,125,800,
50,125,50,125,50,
125,50,125,50,400,
300,50,80,70,80,
120,80,125,125,125,
500,1000,

// Parte 2
300,125,300,
125,800,200,350,350,
200,800,125,300,125,
300,125,800,200,350,
350,200,900,10,200,
10,200,10,200,10,
200,10,700,100,250,
350,350,250,800,200,
300,300,250,900,100,
100,700,150,300,125,
300,125,800,200,350,
350,200,800,125,300,
125,300,125,800,200,
350,350,200,900,10,
200,10,200,10,200,

// Parte 3
10,200,10,700,100,
250,350,350,250,800,
200,300,300,250,900,
100,100,700,200,100,
75,100,75,100,75,
100,100,100,100,100,
165,150,100,100,100,
100,300,100,100,100,
100,100,165,150,100,
100,100,100,300,100,
100,100,100,100,165,
150,100,100,100,100,
300,250,100,200,200,
100,90,90,250,200,
300,100,100,100,100,
100,165,150,100,100,
100,100,300,100,100,
100,100,100,165,150,
100,100,100,100,300,
100,100,100,100,100,
165,150,100,100,100,
100,300,250,100,200,
200,100,90,90,250,
200, 200, 200
};


int eleanor_notas[] = {nota_E5, nota_FS5, nota_G5, nota_A5, nota_G5, nota_FS5, nota_E5, nota_B4, nota_G4, nota_REST,
                          nota_E5, nota_FS5, nota_G5, nota_A5, nota_G5, nota_FS5, nota_E5, nota_B4, nota_G4, nota_REST,

                          nota_A4, nota_REST, nota_A4, nota_B4, nota_G4, nota_E4,
                          nota_G4, nota_A4, nota_B4, 
                          nota_D5, nota_CS5, nota_B4,
                          nota_CS5, nota_B4, nota_A4,
                          nota_B4, nota_A4, nota_G4,
                          nota_A4,
                          nota_G4, nota_A4, nota_B4, nota_C5, nota_B4,
                          
                          nota_A4, nota_REST, nota_A4, nota_B4, nota_G4, nota_E4,
                          nota_G4, nota_A4, nota_B4, 
                          nota_D5, nota_CS5, nota_B4,
                          nota_CS5, nota_B4, nota_A4,
                          nota_B4, nota_A4, nota_G4,
                          nota_A4,
                          nota_G4, nota_A4, nota_B4, nota_C5, nota_B4,
                        
                          nota_A4, nota_G4, nota_A4, nota_B4, nota_G4, nota_E4, nota_REST,
                          nota_E4, nota_E5, nota_B4, nota_A4, nota_G4, nota_E4,
                        
                          nota_A4, nota_G4, nota_A4, nota_B4, nota_G4, nota_E4, nota_REST,
                          nota_E4, nota_G4, nota_B3, nota_A3, nota_G3,
                          nota_E5, nota_B4, nota_A4, nota_G4, nota_E4};

int eleanor_tempos[] = {750, 200, 200, 350, 350, 350, 350, 350, 500, 1500,
                           750, 200, 200, 350, 350, 350, 350, 350, 500, 1500,

                          150, 5, 150, 150, 350, 500,
                          150, 150, 150,
                          300, 150, 150,
                          300, 150, 150,
                          300, 150, 150, 700,
                          150, 150, 150, 350, 500,
                          
                          150, 5, 150, 150, 350, 500,
                          150, 150, 150,
                          300, 150, 150,
                          300, 150, 150,
                          300, 150, 150, 700,
                          150, 150, 150, 350, 500,
                        
                          350, 200, 500, 200, 350, 750, 5,
                          200, 500, 200, 350, 350, 1000,
                          
                          350, 200, 500, 200, 350, 750, 5,
                          200, 500, 200, 350, 350,
                          500, 200, 300, 750};

/*int daytripper_notas[] = {nota_E4, nota_G4, nota_GS4, nota_B4, nota_E5, 
                          nota_D5, nota_B4, nota_FS5, nota_B4, nota_D5, 
                          nota_E5, nota_E4, nota_G4, nota_GS4, nota_B4, 
                          nota_E5, nota_D5, nota_B4, nota_FS5, nota_B4, 
                          nota_D5, nota_E5, nota_B4, nota_D5, nota_DS5, 
                          nota_FS5, nota_B5, nota_A5, nota_FS5, nota_CS6, 
                          nota_FS5, nota_A5, nota_B5, nota_E4, nota_G4, 
                          nota_GS4, nota_B4, nota_E5, nota_D5, nota_B4, 
                          nota_FS5, nota_B4, nota_D5, nota_E5,
                            // encerramento
                        nota_E5, nota_E5, nota_DS5, nota_D5, nota_CS5, nota_C5, nota_B4};

int daytripper_tempos[] = {700, 200, 200, 200, 200, 700, 212, 400, 200, 200, 275,
                           700, 200, 200, 200, 200, 700, 212, 400, 200, 200, 275,
                           700, 200, 200, 200, 200, 700, 212, 400, 200, 200, 275,
                           700, 200, 200, 200, 200, 700, 212, 400, 200, 200, 275,
                           // tempos do encerramento
                           200, 200, 200, 200, 200, 200, 400};*/




int NUM_NOTAS_UNDERTALE  = sizeof(undertale_notas) / sizeof(undertale_notas[0]);
int NUM_NOTAS_ELEANOR = sizeof(eleanor_notas) / sizeof(eleanor_notas [0]);


float calcula_freq_pwm(uint32_t wrap, float clkdiv) {
    if (wrap == 0) return 0;
    else return CLOCK_PWM_HZ / (var_div_clock * wrap);
}

void toca_nota(int nota, int duracao, uint slice_fun_1, int fac_tempo_1, int div_vol)
{
    pwm_set_wrap(slice_fun_1, nota);
    int var_aux = nota/div_vol;
    pwm_set_gpio_level(pin_buzzer, var_aux);
    sleep_us(fac_tempo_1 * duracao);
}

void toca_pausa(int duracao, uint slice_fun_3, int fac_tempo_2)
{
    pwm_set_wrap(slice_fun_3, 2000);
    pwm_set_gpio_level(pin_buzzer, 0);
    sleep_us(fac_tempo_2 * duracao);
}

// Atualiza a intensidade de cada barra com base na frequência atual
void atualiza_banda_por_freq(float freq) {
    // Organiza os centros, no caso, as faixas centrais de frequencia (em Hz)
    float centros[NUM_BARRAS] = {63, 160, 400, 1200};
    // for para aplicar a intensidade em cada barra
    for (int i = 0; i < NUM_BARRAS; i++) {
        float fc = centros[i];

        // Define os limites à esquerda e direita da banda
        // no caso a esquerda vai ser o 63 e o da direita 1.2k e é feito o ajuste para decair caso a intensidade for menor
        // que 63 e maior que 1.2k
        float left = (i == 0) ? 0.0f : centros[i - 1];
        float right = (i == NUM_BARRAS - 1) ? fc + (fc - centros[i - 1]) : centros[i + 1];

        float ganho = 0.0f;
        
        //  Calcula o ganho da barra com base na posição da frequência / ajuste para suavizar as bandas e melhor distribuicao
        if (freq >= left && freq <= fc) {
            ganho = (freq - left) / (fc - left); // Crescendo até o centro -> ganho
        } else if (freq > fc && freq <= right) {
            ganho = (right - freq) / (right - fc); // Caindo depois do centro -> decaimento
        }

        // Limita o ganho entre 0 e 1 , alem disso se houver ganho, aplica e atualiza a intensidade da barra
        if (ganho > 0.0f) {
            // Ajuste das barras para nao ultrapassar o display
            int novo_valor = (int)(ganho * 44.0f);
            // Caso o valor ultrapasse 44, ele vai ser 44, evitando problema no display
            if (novo_valor > 44) novo_valor = 44;  
            if (novo_valor > intensidades[i]) {
                intensidades[i] = novo_valor;
            }
        }
    }
}

void desenha_spectrum(ssd1306_t* ssd, uint8_t intensidades[]) {
    ssd1306_fill(ssd, false);
    // Desenha o gráfico de barras (spectrum) no display OLED
    // Linha base do gráfico
    ssd1306_line(ssd, 0, 50, 125, 50, true);
    // Linhas horizontais para dar referência visual das bandas
    ssd1306_line(ssd, 0, 45, 20, 45, true);
    ssd1306_line(ssd, 35, 45, 55, 45, true);
    ssd1306_line(ssd, 70, 45, 90, 45, true);
    ssd1306_line(ssd, 105, 45, 125, 45, true);

    // Desenha cada barra vertical de acordo com a intensidade
    for (int i = 0; i < NUM_BARRAS; i++) {
        int altura = intensidades[i];
        int base_y = 44;
        for (int y = 0; y < altura; y++) {
            for (int x = 0; x < largura_barra; x++) {
                ssd1306_pixel(ssd, posicoes_x[i] + x, base_y - y, true);
            }
        }
    }
    // Legendas das frequências centrais abaixo das barras
    ssd1306_draw_string(ssd, "63", 2, 55);
    ssd1306_draw_string(ssd, "160", 31, 55);
    ssd1306_draw_string(ssd, "400", 66, 55);
    ssd1306_draw_string(ssd, "12K", 101, 55);
    ssd1306_pixel(ssd, 107, 61, true);
    // Atualiza o conteúdo no display
    ssd1306_send_data(ssd);
}

// Animação do decaimento suave das barras
void decaimento_suave() {
    for (int i = 0; i < NUM_BARRAS; i++) {
        intensidades[i] = (int)(intensidades[i] * 0.8f); // Decaimento exponencial de 20%

        if (intensidades[i] < 1) {
            intensidades[i] = 0; // Elimina resíduos visuais pequenos
        }
    }
}

void toca_musica(int vetor_notas[], int vetor_tempos[], int num_notas, int slice_fun_2, ssd1306_t tela)
{
    v_out = 26; // Valor padrão de volume
    t_out = 1000; // Fator de tempo padrão
    char temp_c, buffer[5];
    int  temp_i;
    for (int i = 0; i < num_notas; i++)
    {

        printf("Foi\n");
        if (uart_is_readable(UART_ID))
        {
            temp_i = 0;
            do {
                temp_c = uart_getc(UART_ID);
                temp_c == ' ' ? (buffer[temp_i] = '\0') : (buffer[temp_i] = temp_c);
                temp_i ++; 
                printf("%c\n", temp_c);
            } while (temp_c != ' ');
            v_out = atoi(buffer);
            printf("%s\n", buffer);

            temp_i = 0;
            do {
                temp_c = uart_getc(UART_ID);
                temp_c == ' ' ? (buffer[temp_i] = '\0') : (buffer[temp_i] = temp_c);
                temp_i ++;
                printf("%c\n", temp_c);
            } while (temp_c != ' ');
            t_out = atoi(buffer);

            while (uart_is_readable(uart0)) {
                temp_c = uart_getc(uart0);
            }

        }
        printf("%s %d %d", buffer, v_out, t_out);

        if (vetor_notas[i] == nota_REST)
        {
            toca_pausa(vetor_tempos[i], slice_fun_2, t_out);
        }
        else
        {
            // calculo para quando tocar a nota, enviar o valor de frequencia convertido para 
            // o display, e por fim as funcoes necessarias pra tal feito
            float freq_real = calcula_freq_pwm(vetor_notas[i], var_div_clock);
            toca_nota(vetor_notas[i], vetor_tempos[i], slice_fun_2, t_out, v_out);
            desenha_spectrum(&tela, intensidades);
            atualiza_banda_por_freq(freq_real);
            decaimento_suave();
            sleep_ms(50);
            printf("Freq: %.2f\n", freq_real);
        }
        printf("Fator Tempo: %d", t_out);
    }

}

int main()
{
    stdio_init_all();
    sleep_ms(200);

    // Configuração pino do buzzer
    gpio_init(pin_buzzer);
    gpio_set_dir(pin_buzzer, GPIO_OUT);
    //Configuração da leitura do canal AD
    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);

    //Configuração do módulo PWM
    gpio_set_function(pin_buzzer, GPIO_FUNC_PWM); 
    uint slice = pwm_gpio_to_slice_num(pin_buzzer); 
    pwm_set_clkdiv(slice, var_div_clock); 
    pwm_set_enabled(slice, true); 
    
    //Configuração da comunicação UART
    uart_init(UART_ID, 115200);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    sleep_ms(500); // Atraso para estabilizar a UART

    //Configuração da comunicação I2C com Display OLED
    i2c_init(I2C_PORT, 400 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
    gpio_pull_up(I2C_SDA); // Pull up the data line
    gpio_pull_up(I2C_SCL); // Pull up the clock line
    ssd1306_t ssd; // Inicializa a estrutura do display
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, ENDERECO, I2C_PORT); // Inicializa o display
    ssd1306_config(&ssd); // Configura o display
    ssd1306_send_data(&ssd); // Envia os dados para o display

    // Limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    while (true)
    {
        printf ("Aguardando leitura\n");
        if(uart_is_readable(UART_ID))
        {
            uint8_t caracter = uart_getc(UART_ID);
            printf("Caractere recebido: %d\n", caracter);

            while (uart_is_readable(uart0)) 
            {
                char temp = uart_getc(uart0);
            }
            
            if (caracter == 1)
            {
                toca_musica(undertale_notas, undertale_tempos, NUM_NOTAS_UNDERTALE, slice, ssd);
            }
            else if (caracter == 2)
            {
                toca_musica(eleanor_notas, eleanor_tempos, NUM_NOTAS_ELEANOR, slice, ssd);
            }
        } 
        
        pwm_set_gpio_level(pin_buzzer, 0);
        bool ainda_tem_intensidade = true;
        while (ainda_tem_intensidade) {
        desenha_spectrum(&ssd, intensidades); // Redesenha com novas intensidades
        decaimento_suave();
        sleep_ms(50);

        // Verifica se ainda há alguma barra com valor > 0
        ainda_tem_intensidade = false;
        for (int i = 0; i < NUM_BARRAS; i++) {
            if (intensidades[i] > 0) {
                ainda_tem_intensidade = true;
                break;
            }
        }
    }
}
}