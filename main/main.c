#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"

#include "tft_lcd_ili9341/gfx/gfx_ili9341.h"
#include "tft_lcd_ili9341/ili9341/ili9341.h"

const uint PIN_LED_VERMELHO = 2;
const uint PIN_LED_AMARELO  = 3;
const uint PIN_LED_VERDE    = 21;
const uint PIN_LED_AZUL     = 20;

const uint PIN_BTN_VERMELHO = 6;
const uint PIN_BTN_AMARELO  = 10;
const uint PIN_BTN_VERDE    = 8;
const uint PIN_BTN_AZUL     = 9;

const uint PIN_BUZZER = 14;

#define MAX_SEQUENCIA       20
#define DURACAO_NOTA_MS     500
#define PAUSA_ENTRE_MS      100
#define DURACAO_INPUT_MS    200
#define TIMEOUT_BTN_US      7000000ULL

#define MSG_ESTADO      0x01000000U
#define MSG_PONTUACAO   0x02000000U
#define MSG_HIGHLIGHT   0x03000000U

#define ESTADO_IDLE       0U
#define ESTADO_MOSTRANDO  1U
#define ESTADO_AGUARDANDO 2U
#define ESTADO_ACERTO     3U
#define ESTADO_ERRO       4U

#define COR_VERMELHO  0U
#define COR_VERDE     1U
#define COR_AZUL      2U
#define COR_AMARELO   3U
#define COR_NENHUMA   0xFFU

#define FREQ_VERMELHO  262U
#define FREQ_VERDE     330U
#define FREQ_AZUL      392U
#define FREQ_AMARELO   523U
#define FREQ_ERRO      150U
#define FREQ_VITORIA   784U

#define SCREEN_ROTATION  1

#define BOX_W   145
#define BOX_H   85
#define BOX_GAP 10
#define BOX_Y0  40

volatile bool g_btn_vermelho = false;
volatile bool g_btn_verde    = false;
volatile bool g_btn_azul     = false;
volatile bool g_btn_amarelo  = false;

static void buzzer_init(void) {
    gpio_set_function(PIN_BUZZER, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PIN_BUZZER);
    pwm_set_enabled(slice, false);
}

static void buzzer_play(uint freq_hz) {
    uint slice   = pwm_gpio_to_slice_num(PIN_BUZZER);
    uint channel = pwm_gpio_to_channel(PIN_BUZZER);
    uint32_t wrap = (7812500U / freq_hz) - 1U;
    pwm_set_clkdiv_int_frac(slice, 16, 0);
    pwm_set_wrap(slice, (uint16_t)wrap);
    pwm_set_chan_level(slice, channel, (uint16_t)(wrap / 2U));
    pwm_set_enabled(slice, true);
}

static void buzzer_stop(void) {
    uint slice = pwm_gpio_to_slice_num(PIN_BUZZER);
    pwm_set_enabled(slice, false);
}

static uint cor_para_led(uint32_t cor) {
    if (cor == COR_VERDE)   { return PIN_LED_VERDE;   }
    if (cor == COR_AZUL)    { return PIN_LED_AZUL;    }
    if (cor == COR_AMARELO) { return PIN_LED_AMARELO; }
    return PIN_LED_VERMELHO;
}

static uint cor_para_btn(uint32_t cor) {
    if (cor == COR_VERDE)   { return PIN_BTN_VERDE;   }
    if (cor == COR_AZUL)    { return PIN_BTN_AZUL;    }
    if (cor == COR_AMARELO) { return PIN_BTN_AMARELO; }
    return PIN_BTN_VERMELHO;
}

static uint cor_para_freq(uint32_t cor) {
    if (cor == COR_VERDE)   { return FREQ_VERDE;   }
    if (cor == COR_AZUL)    { return FREQ_AZUL;    }
    if (cor == COR_AMARELO) { return FREQ_AMARELO; }
    return FREQ_VERMELHO;
}

static void btn_callback(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_FALL) {
        if      (gpio == PIN_BTN_VERMELHO) { g_btn_vermelho = true; }
        else if (gpio == PIN_BTN_VERDE)    { g_btn_verde    = true; }
        else if (gpio == PIN_BTN_AZUL)     { g_btn_azul     = true; }
        else if (gpio == PIN_BTN_AMARELO)  { g_btn_amarelo  = true; }
    }
}

static void exibir_cor(int cor, int duracao_ms) {
    gpio_put(cor_para_led((uint32_t)cor), 1);
    buzzer_play(cor_para_freq((uint32_t)cor));
    multicore_fifo_push_timeout_us(MSG_HIGHLIGHT | (uint32_t)cor, 1000);
    sleep_ms(duracao_ms);
    gpio_put(cor_para_led((uint32_t)cor), 0);
    buzzer_stop();
    multicore_fifo_push_timeout_us(MSG_HIGHLIGHT | COR_NENHUMA, 1000);
    sleep_ms(PAUSA_ENTRE_MS);
}

static void feedback_erro(void) {
    int i;
    int j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 4; j++) {
            gpio_put(cor_para_led((uint32_t)j), 1);
        }
        buzzer_play(FREQ_ERRO);
        sleep_ms(200);
        for (j = 0; j < 4; j++) {
            gpio_put(cor_para_led((uint32_t)j), 0);
        }
        buzzer_stop();
        sleep_ms(200);
    }
}

static void feedback_acerto_nivel(void) {
    int j;
    for (j = 0; j < 4; j++) { gpio_put(cor_para_led((uint32_t)j), 1); }
    buzzer_play(FREQ_VITORIA);
    sleep_ms(400);
    for (j = 0; j < 4; j++) { gpio_put(cor_para_led((uint32_t)j), 0); }
    buzzer_stop();
    sleep_ms(150);
}

static void aguardar_botao_inicio(void) {
    g_btn_vermelho = false;
    g_btn_verde    = false;
    g_btn_azul     = false;
    g_btn_amarelo  = false;

    while (true) {
        if (g_btn_vermelho || g_btn_verde ||
            g_btn_azul    || g_btn_amarelo) {
            g_btn_vermelho = false;
            g_btn_verde    = false;
            g_btn_azul     = false;
            g_btn_amarelo  = false;
            return;
        }
        tight_loop_contents();
    }
}

static int aguardar_botao(void) {
    g_btn_vermelho = false;
    g_btn_verde    = false;
    g_btn_azul     = false;
    g_btn_amarelo  = false;

    uint64_t inicio = time_us_64();
    while ((time_us_64() - inicio) < TIMEOUT_BTN_US) {
        if (g_btn_vermelho) { g_btn_vermelho = false; return (int)COR_VERMELHO; }
        if (g_btn_verde)    { g_btn_verde    = false; return (int)COR_VERDE;    }
        if (g_btn_azul)     { g_btn_azul     = false; return (int)COR_AZUL;     }
        if (g_btn_amarelo)  { g_btn_amarelo  = false; return (int)COR_AMARELO;  }
        tight_loop_contents();
    }
    return -1;
}

void core1_entry(void) {
    int i;
    int sequencia[MAX_SEQUENCIA];

    for (i = 0; i < 4; i++) {
        gpio_init(cor_para_led((uint32_t)i));
        gpio_set_dir(cor_para_led((uint32_t)i), GPIO_OUT);
        gpio_put(cor_para_led((uint32_t)i), 0);
    }

    for (i = 0; i < 4; i++) {
        gpio_init(cor_para_btn((uint32_t)i));
        gpio_set_dir(cor_para_btn((uint32_t)i), GPIO_IN);
        gpio_pull_up(cor_para_btn((uint32_t)i));
    }

    buzzer_init();

    gpio_set_irq_enabled_with_callback(PIN_BTN_VERMELHO,
        GPIO_IRQ_EDGE_FALL, true, &btn_callback);
    gpio_set_irq_enabled(PIN_BTN_VERDE,   GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(PIN_BTN_AZUL,    GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(PIN_BTN_AMARELO, GPIO_IRQ_EDGE_FALL, true);

    multicore_fifo_push_blocking(MSG_ESTADO | ESTADO_IDLE);
    multicore_fifo_push_blocking(MSG_PONTUACAO);

    while (true) {

        aguardar_botao_inicio();

        srand((unsigned int)(time_us_64() & 0xFFFFFFFFU));

        for (i = 0; i < MAX_SEQUENCIA; i++) {
            sequencia[i] = rand() % 4;
        }

        int nivel       = 1;
        int pontuacao   = 0;
        bool jogo_ativo = true;

        multicore_fifo_push_blocking(MSG_PONTUACAO);

        while (jogo_ativo) {

            multicore_fifo_push_blocking(MSG_ESTADO | ESTADO_MOSTRANDO);
            sleep_ms(800);

            for (i = 0; i < nivel; i++) {
                exibir_cor(sequencia[i], DURACAO_NOTA_MS);
            }

            sleep_ms(300);
            multicore_fifo_push_blocking(MSG_ESTADO | ESTADO_AGUARDANDO);

            bool acertou = true;
            for (i = 0; i < nivel && acertou; i++) {
                int btn = aguardar_botao();

                if (btn == -1) {
                    acertou = false;
                } else if (btn == sequencia[i]) {
                    exibir_cor(btn, DURACAO_INPUT_MS);
                } else {
                    exibir_cor(btn, DURACAO_INPUT_MS);
                    acertou = false;
                }
            }

            if (acertou) {
                pontuacao++;
                nivel++;

                multicore_fifo_push_blocking(MSG_ESTADO    | ESTADO_ACERTO);
                multicore_fifo_push_blocking(MSG_PONTUACAO | (uint32_t)pontuacao);

                feedback_acerto_nivel();
                sleep_ms(600);

                if (nivel > MAX_SEQUENCIA) {
                    jogo_ativo = false;
                }
            } else {
                jogo_ativo = false;

                multicore_fifo_push_blocking(MSG_ESTADO | ESTADO_ERRO);
                feedback_erro();
                sleep_ms(1000);
            }

        }

        multicore_fifo_push_blocking(MSG_ESTADO | ESTADO_IDLE);
        sleep_ms(2000);

    }
}

static uint16_t lcd_cor_bright(uint32_t cor) {
    if (cor == COR_VERDE)   { return 0x07E0U; }
    if (cor == COR_AZUL)    { return 0x001FU; }
    if (cor == COR_AMARELO) { return 0xFFE0U; }
    return 0xF800U;
}

static uint16_t lcd_cor_dim(uint32_t cor) {
    if (cor == COR_VERDE)   { return 0x01E0U; }
    if (cor == COR_AZUL)    { return 0x0007U; }
    if (cor == COR_AMARELO) { return 0x3BE0U; }
    return 0x3000U;
}

static const char *lcd_nome_cor(uint32_t cor) {
    if (cor == COR_VERDE)   { return "VERDE";   }
    if (cor == COR_AZUL)    { return "AZUL";    }
    if (cor == COR_AMARELO) { return "AMARELO"; }
    return "VERM";
}

static int box_x(uint32_t cor) {
    if (cor == COR_VERDE || cor == COR_AMARELO) { return 5 + BOX_W + BOX_GAP; }
    return 5;
}

static int box_y(uint32_t cor) {
    if (cor == COR_AZUL || cor == COR_AMARELO) { return BOX_Y0 + BOX_H + BOX_GAP; }
    return BOX_Y0;
}

static void lcd_desenha_layout_base(void) {
    int i;
    gfx_clear();

    gfx_setTextSize(2);
    gfx_setTextColor(0xFFFFU);
    gfx_drawText(124, 8, "GENIUS");

    for (i = 0; i < 4; i++) {
        uint32_t cor = (uint32_t)i;
        gfx_fillRect(box_x(cor), box_y(cor), BOX_W, BOX_H, lcd_cor_dim(cor));
        gfx_setTextSize(1);
        gfx_setTextColor(0x0000U);
        gfx_drawText(box_x(cor) + 45, box_y(cor) + 38, lcd_nome_cor(cor));
    }

    gfx_setTextSize(1);
    gfx_setTextColor(0xFFFFU);
    gfx_drawText(40, 228, "Pressione um botao para iniciar");
}

static void lcd_atualiza_highlight(int cor_id) {
    int i;
    for (i = 0; i < 4; i++) {
        uint32_t cor   = (uint32_t)i;
        uint16_t fill  = ((uint32_t)cor_id == cor)
                         ? lcd_cor_bright(cor)
                         : lcd_cor_dim(cor);
        gfx_fillRect(box_x(cor), box_y(cor), BOX_W, BOX_H, fill);
        gfx_setTextSize(1);
        gfx_setTextColor(0x0000U);
        gfx_drawText(box_x(cor) + 45, box_y(cor) + 38, lcd_nome_cor(cor));
    }
}

static void lcd_atualiza_estado(uint32_t estado) {
    gfx_fillRect(0, 222, 320, 18, 0x0000U);
    gfx_setTextSize(1);

    if (estado == ESTADO_IDLE) {
        gfx_setTextColor(0xFFFFU);
        gfx_drawText(40, 228, "Pressione um botao para iniciar");
    } else if (estado == ESTADO_MOSTRANDO) {
        gfx_setTextColor(0x07FFU);
        gfx_drawText(75, 228, "Observe a sequencia...");
    } else if (estado == ESTADO_AGUARDANDO) {
        gfx_setTextColor(0xFFE0U);
        gfx_drawText(88, 228, "Repita a sequencia!");
    } else if (estado == ESTADO_ACERTO) {
        gfx_setTextColor(0x07E0U);
        gfx_drawText(98, 228, "Acertou! Avancando...");
    } else if (estado == ESTADO_ERRO) {
        gfx_setTextColor(0xF800U);
        gfx_drawText(118, 228, "Errou! Fim de jogo.");
    }
}

static void lcd_atualiza_pontuacao(int pontuacao) {
    char buf[16];
    gfx_fillRect(230, 5, 90, 20, 0x0000U);
    gfx_setTextSize(2);
    gfx_setTextColor(0xFFE0U);
    sprintf(buf, "Pts:%d", pontuacao);
    gfx_drawText(232, 7, buf);
}

int main(void) {
    stdio_init_all();

    LCD_initDisplay();
    LCD_setRotation(SCREEN_ROTATION);
    gfx_init();
    gfx_clear();

    gpio_init(15);
    gpio_set_dir(15, GPIO_OUT);
    gpio_put(15, 1);

    multicore_launch_core1(core1_entry);

    int pontuacao = 0;

    lcd_desenha_layout_base();
    lcd_atualiza_pontuacao(pontuacao);

    while (true) {
        uint32_t msg;
        if (multicore_fifo_pop_timeout_us(0, &msg)) {
            uint32_t tipo = msg & 0xFF000000U;
            uint32_t dado = msg & 0x00FFFFFFU;

            if (tipo == MSG_ESTADO) {
                if (dado == ESTADO_MOSTRANDO) {
                    gpio_put(15, 1);
                } else if (dado == ESTADO_IDLE) {
                    gpio_put(15, 0);
                }
                lcd_atualiza_estado(dado);
            } else if (tipo == MSG_PONTUACAO) {
                pontuacao = (int)dado;
                lcd_atualiza_pontuacao(pontuacao);
            } else if (tipo == MSG_HIGHLIGHT) {
                lcd_atualiza_highlight((int)dado);
            }
        }
    }
}