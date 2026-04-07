/*
 * APS 1 – Jogo Genius para Raspberry Pi Pico (RP2040)
 *
 * ARQUITETURA MULTI-CORE (Particionamento Funcional Simétrico):
 *
 *   Core 1 — Lógica do jogo + hardware físico
 *             Responsável por: geração de sequência aleatória, verificação
 *             de inputs, controle de LEDs, buzzer, e ISRs dos botões.
 *             Envia mensagens ao Core 0 via FIFO.
 *
 *   Core 0 — Interface LCD
 *             Responsável por: receber mensagens do Core 1 via FIFO e
 *             renderizar o estado do jogo no display (highlight de cores,
 *             estado atual, pontuação). Nunca acessa GPIO de jogo.
 *
 * PROTOCOLO FIFO Core 1 → Core 0 (uint32_t, 32 bits):
 *   bits 31–24 : tipo de mensagem  (MSG_ESTADO, MSG_PONTUACAO, MSG_HIGHLIGHT)
 *   bits 23–0  : dado              (id do estado, cor destacada, pontuação)
 *
 * INTEGRAÇÃO NÃO-DECORATIVA:
 *   O LCD exibe em tempo-real qual cor está sendo mostrada na sequência
 *   (caixa destacada = brilhante), qual cor o jogador pressionou (mesmo
 *   highlight durante o input), o estado atual do jogo e a pontuação.
 *   Sem o LCD o jogador não tem feedback visual de estado nem pontuação.
 */

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


/* ============================================================
 * PINOS DE HARDWARE
 * Ajuste conforme a fiação do protótipo físico.
 * ============================================================ */
const uint PIN_LED_VERMELHO = 2;
const uint PIN_LED_AMARELO  = 3;
const uint PIN_LED_VERDE    = 21;
const uint PIN_LED_AZUL     = 20;

const uint PIN_BTN_VERMELHO = 6;
const uint PIN_BTN_AMARELO  = 10;
const uint PIN_BTN_VERDE    = 8;
const uint PIN_BTN_AZUL     = 9;

const uint PIN_BUZZER = 14;

/* ============================================================
 * PARÂMETROS DO JOGO
 * ============================================================ */
#define MAX_SEQUENCIA       20
#define DURACAO_NOTA_MS     500      /* duração de cada cor na sequência    */
#define PAUSA_ENTRE_MS      100      /* silêncio entre duas cores seguidas  */
#define DURACAO_INPUT_MS    200      /* feedback ao receber input do jogador */
#define TIMEOUT_BTN_US      7000000ULL  /* 7 s para pressionar um botão     */

/* ============================================================
 * PROTOCOLO DE COMUNICAÇÃO INTER-CORE (FIFO 32 bits)
 *   bits 31–24 : tipo de mensagem
 *   bits 23–0  : dado
 * ============================================================ */
#define MSG_ESTADO      0x01000000U
#define MSG_PONTUACAO   0x02000000U
#define MSG_HIGHLIGHT   0x03000000U

/* Identificadores de estado (campo dado quando tipo == MSG_ESTADO) */
#define ESTADO_IDLE       0U
#define ESTADO_MOSTRANDO  1U
#define ESTADO_AGUARDANDO 2U
#define ESTADO_ACERTO     3U
#define ESTADO_ERRO       4U

/* Identificadores de cor (campo dado quando tipo == MSG_HIGHLIGHT) */
#define COR_VERMELHO  0U
#define COR_VERDE     1U
#define COR_AZUL      2U
#define COR_AMARELO   3U
#define COR_NENHUMA   0xFFU

/* ============================================================
 * FREQUÊNCIAS DO BUZZER (Hz) — uma frequência distinta por cor
 * Dó-Mi-Sol-Dó8 formam um acorde perceptivelmente distinto
 * ============================================================ */
#define FREQ_VERMELHO  262U   /* Dó  */
#define FREQ_VERDE     330U   /* Mi  */
#define FREQ_AZUL      392U   /* Sol */
#define FREQ_AMARELO   523U   /* Dó (oitava acima) */
#define FREQ_ERRO      150U   /* Grave (feedback de erro)    */
#define FREQ_VITORIA   784U   /* Agudo (feedback de acerto)  */

/* ============================================================
 * CONFIGURAÇÃO DO LCD (paisagem 320×240)
 * ============================================================ */
#define SCREEN_ROTATION  1

/* Grade 2×2 de caixas coloridas na tela */
#define BOX_W   145
#define BOX_H   85
#define BOX_GAP 10
#define BOX_Y0  40    /* Y da linha superior das caixas */

/* ============================================================
 * VARIÁVEIS GLOBAIS VOLÁTEIS — ISR (Core 1) → loop do Core 1
 *
 * Regra 1.1/1.2: SOMENTE estas variáveis são globais porque são
 * escritas dentro de uma interrupção de hardware e lidas no loop
 * principal. Prefixo g_ conforme padrão do curso.
 * ============================================================ */
volatile bool g_btn_vermelho = false;
volatile bool g_btn_verde    = false;
volatile bool g_btn_azul     = false;
volatile bool g_btn_amarelo  = false;

/* ============================================================
 * BUZZER — funções auxiliares (usadas apenas no Core 1)
 * Frequência gerada por PWM de hardware; sem delays em callbacks.
 * ============================================================ */

static void buzzer_init(void) {
    gpio_set_function(PIN_BUZZER, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PIN_BUZZER);
    pwm_set_enabled(slice, false);
}

static void buzzer_play(uint freq_hz) {
    uint slice   = pwm_gpio_to_slice_num(PIN_BUZZER);
    uint channel = pwm_gpio_to_channel(PIN_BUZZER);

    /* sys_clock = 125 MHz; clkdiv = 16 → pwm_clk = 7.8125 MHz
     * Divide o clock para que wrap = pwm_clk/freq caiba em 16 bits.
     * Para 150–784 Hz: wrap fica entre 9966 e 52083 (dentro de 65535).
     * wrap = 7812500/freq - 1
     * chan_level = wrap/2  →  duty cycle 50%                          */
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

/* ============================================================
 * MAPEAMENTOS COR → RECURSO DE HARDWARE
 * Funções em vez de arrays globais para respeitar Rule 1.1.
 * ============================================================ */

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

/* ============================================================
 * ISR CALLBACK — registrada no Core 1
 *
 * Regras 3.0–3.3: SOMENTE seta flags. Sem delay, sem printf,
 * sem display, sem laços. ISR mínima e rápida.
 * ============================================================ */
static void btn_callback(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_FALL) {
        if      (gpio == PIN_BTN_VERMELHO) { g_btn_vermelho = true; }
        else if (gpio == PIN_BTN_VERDE)    { g_btn_verde    = true; }
        else if (gpio == PIN_BTN_AZUL)     { g_btn_azul     = true; }
        else if (gpio == PIN_BTN_AMARELO)  { g_btn_amarelo  = true; }
    }
}

/* ============================================================
 * CORE 1 — Funções auxiliares do jogo
 * Todas as funções abaixo rodam no contexto do Core 1,
 * NUNCA dentro de ISR.
 * ============================================================ */

/**
 * Acende o LED da cor, toca a nota no buzzer por duracao_ms,
 * e envia mensagem MSG_HIGHLIGHT ao Core 0 para sincronizar o LCD.
 * Chamada tanto na exibição da sequência quanto no feedback de input.
 */
static void exibir_cor(int cor, int duracao_ms) {
    /* Liga LED + buzzer imediatamente — sem depender do LCD */
    gpio_put(cor_para_led((uint32_t)cor), 1);
    buzzer_play(cor_para_freq((uint32_t)cor));

    /* Notifica Core 0 com timeout: se FIFO cheia, descarta e segue em frente
     * para não atrasar o timing dos LEDs */
    multicore_fifo_push_timeout_us(MSG_HIGHLIGHT | (uint32_t)cor, 1000);

    sleep_ms(duracao_ms);

    /* Desliga LED + buzzer */
    gpio_put(cor_para_led((uint32_t)cor), 0);
    buzzer_stop();

    multicore_fifo_push_timeout_us(MSG_HIGHLIGHT | COR_NENHUMA, 1000);

    sleep_ms(PAUSA_ENTRE_MS);
}

/** Feedback de erro: pisca todos os LEDs 3× com tom grave */
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

/**
 * Bloqueia indefinidamente até qualquer botão ser pressionado.
 * Usada apenas na tela inicial — sem timeout.
 */
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

/**
 * Bloqueia aguardando qualquer botão ser pressionado, com timeout.
 * Limpa todas as flags antes de aguardar para descartar pressionamentos
 * residuais de fases anteriores do jogo.
 *
 * Retorna: COR_* (0–3) se pressionado, ou -1 em timeout.
 */
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

/* ============================================================
 * CORE 1 — Entry point
 *
 * Fluxo:
 *   1. Inicializa hardware (LEDs, botões, buzzer, IRQ)
 *   2. Aguarda qualquer botão para iniciar (tela IDLE)
 *   3. Gera seed aleatória com time_us_64() e preenche sequência
 *   4. Loop de rodadas:
 *      a. Exibe sequência até o nível atual (MSG_HIGHLIGHT para cada cor)
 *      b. Aguarda input do jogador botão a botão
 *      c. Acerto → avança nível, envia MSG_ACERTO + MSG_PONTUACAO
 *         Erro/timeout → envia MSG_ERRO, encerra jogo
 *   5. Volta ao passo 2
 * ============================================================ */
void core1_entry(void) {
    int i;
    int sequencia[MAX_SEQUENCIA];

    /* ### Inicialização dos LEDs */
    for (i = 0; i < 4; i++) {
        gpio_init(cor_para_led((uint32_t)i));
        gpio_set_dir(cor_para_led((uint32_t)i), GPIO_OUT);
        gpio_put(cor_para_led((uint32_t)i), 0);
    }

    /* ### Inicialização dos botões com pull-up interno */
    for (i = 0; i < 4; i++) {
        gpio_init(cor_para_btn((uint32_t)i));
        gpio_set_dir(cor_para_btn((uint32_t)i), GPIO_IN);
        gpio_pull_up(cor_para_btn((uint32_t)i));
    }

    /* ### Inicialização do buzzer via PWM */
    buzzer_init();

    /* ### Registro das IRQs dos botões no Core 1
     * A primeira chamada com _with_callback registra o callback global
     * deste core. As chamadas seguintes com gpio_set_irq_enabled reutilizam
     * o mesmo callback — padrão idêntico ao do main.c de referência. */
    gpio_set_irq_enabled_with_callback(PIN_BTN_VERMELHO,
        GPIO_IRQ_EDGE_FALL, true, &btn_callback);
    gpio_set_irq_enabled(PIN_BTN_VERDE,   GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(PIN_BTN_AZUL,    GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(PIN_BTN_AMARELO, GPIO_IRQ_EDGE_FALL, true);

    /* ### Informa Core 0 do estado inicial */
    multicore_fifo_push_blocking(MSG_ESTADO    | ESTADO_IDLE);
    multicore_fifo_push_blocking(MSG_PONTUACAO);

    /* ===== Loop principal do jogo ===== */
    while (true) {

        /* Aguarda qualquer botão para iniciar o jogo — sem timeout */
        aguardar_botao_inicio();

        /* Seed baseada no tempo desde o boot até este instante exato.
         * Diferente a cada jogo e a cada energização (Rule: seed adequada). */
        srand((unsigned int)(time_us_64() & 0xFFFFFFFFU));

        /* Gera a sequência completa de uma vez */
        for (i = 0; i < MAX_SEQUENCIA; i++) {
            sequencia[i] = rand() % 4;
        }

        int nivel       = 1;
        int pontuacao   = 0;
        bool jogo_ativo = true;

        multicore_fifo_push_blocking(MSG_PONTUACAO);

        /* ===== Loop de rodadas ===== */
        while (jogo_ativo) {

            /* --- Fase 1: exibe a sequência até o nível atual --- */
            multicore_fifo_push_blocking(MSG_ESTADO | ESTADO_MOSTRANDO);
            sleep_ms(800);  /* pausa dramática antes de mostrar */

            for (i = 0; i < nivel; i++) {
                exibir_cor(sequencia[i], DURACAO_NOTA_MS);
            }

            /* --- Fase 2: aguarda input do jogador --- */
            sleep_ms(300);
            multicore_fifo_push_blocking(MSG_ESTADO | ESTADO_AGUARDANDO);

            bool acertou = true;
            for (i = 0; i < nivel && acertou; i++) {
                int btn = aguardar_botao();

                if (btn == -1) {
                    /* Timeout: nenhuma tecla pressionada em 5 s → erro */
                    acertou = false;
                } else if (btn == sequencia[i]) {
                    /* Acerto parcial: feedback imediato (LED + buzzer + LCD) */
                    exibir_cor(btn, DURACAO_INPUT_MS);
                } else {
                    /* Input errado: mostra a cor pressionada antes de encerrar */
                    exibir_cor(btn, DURACAO_INPUT_MS);
                    acertou = false;
                }
            }

            /* --- Fase 3: resultado da rodada --- */
            if (acertou) {
                pontuacao++;
                nivel++;

                multicore_fifo_push_blocking(MSG_ESTADO    | ESTADO_ACERTO);
                multicore_fifo_push_blocking(MSG_PONTUACAO | (uint32_t)pontuacao);

                feedback_acerto_nivel();
                sleep_ms(600);

                if (nivel > MAX_SEQUENCIA) {
                    /* Jogador completou a sequência máxima: vitória total */
                    jogo_ativo = false;
                }
            } else {
                jogo_ativo = false;

                multicore_fifo_push_blocking(MSG_ESTADO | ESTADO_ERRO);
                feedback_erro();
                sleep_ms(1000);
            }

        } /* fim loop de rodadas */

        /* Volta à tela inicial */
        multicore_fifo_push_blocking(MSG_ESTADO | ESTADO_IDLE);
        sleep_ms(2000);

    } /* fim loop principal */
}

/* ============================================================
 * CORE 0 — Funções auxiliares do LCD
 * Nenhuma destas funções deve ser chamada de dentro de uma ISR.
 * ============================================================ */

/* --- Mapeamentos de cor para atributos visuais --- */

static uint16_t lcd_cor_bright(uint32_t cor) {
    if (cor == COR_VERDE)   { return 0x07E0U; }  /* verde  RGB565 */
    if (cor == COR_AZUL)    { return 0x001FU; }  /* azul   RGB565 */
    if (cor == COR_AMARELO) { return 0xFFE0U; }  /* amarelo RGB565 */
    return 0xF800U;                               /* vermelho RGB565 */
}

static uint16_t lcd_cor_dim(uint32_t cor) {
    if (cor == COR_VERDE)   { return 0x01E0U; }  /* verde escuro  */
    if (cor == COR_AZUL)    { return 0x0007U; }  /* azul escuro   */
    if (cor == COR_AMARELO) { return 0x3BE0U; }  /* ocre escuro   */
    return 0x3000U;                               /* vermelho escuro */
}

static const char *lcd_nome_cor(uint32_t cor) {
    if (cor == COR_VERDE)   { return "VERDE";   }
    if (cor == COR_AZUL)    { return "AZUL";    }
    if (cor == COR_AMARELO) { return "AMARELO"; }
    return "VERM";
}

/* Posições X das caixas: coluna esquerda (vermelho, azul) e direita (verde, amarelo) */
static int box_x(uint32_t cor) {
    if (cor == COR_VERDE || cor == COR_AMARELO) { return 5 + BOX_W + BOX_GAP; }
    return 5;
}

/* Posições Y das caixas: linha superior (vermelho, verde) e inferior (azul, amarelo) */
static int box_y(uint32_t cor) {
    if (cor == COR_AZUL || cor == COR_AMARELO) { return BOX_Y0 + BOX_H + BOX_GAP; }
    return BOX_Y0;
}

/**
 * Desenha o layout base da tela: título, 4 caixas de cor (todas dim)
 * e a linha de status. Chamada uma única vez na inicialização do Core 0.
 */
static void lcd_desenha_layout_base(void) {
    int i;
    gfx_clear();

    /* Título centralizado */
    gfx_setTextSize(2);
    gfx_setTextColor(0xFFFFU);
    gfx_drawText(124, 8, "GENIUS");

    /* Quatro caixas de cor na grade 2×2, todas escuras inicialmente */
    for (i = 0; i < 4; i++) {
        uint32_t cor = (uint32_t)i;
        gfx_fillRect(box_x(cor), box_y(cor), BOX_W, BOX_H, lcd_cor_dim(cor));
        gfx_setTextSize(1);
        gfx_setTextColor(0x0000U);
        gfx_drawText(box_x(cor) + 45, box_y(cor) + 38, lcd_nome_cor(cor));
    }

    /* Mensagem de status inicial */
    gfx_setTextSize(1);
    gfx_setTextColor(0xFFFFU);
    gfx_drawText(40, 228, "Pressione um botao para iniciar");
}

/**
 * Atualiza o destaque das caixas no LCD.
 * cor_id: COR_* (0–3) para destacar uma cor, ou COR_NENHUMA (0xFF) para apagar tudo.
 * Esta função é o coração da integração LCD: sincroniza visualmente
 * a sequência mostrada pelos LEDs com a tela.
 */
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

/** Atualiza a linha de status na parte inferior da tela */
static void lcd_atualiza_estado(uint32_t estado) {
    /* Limpa área de status */
    gfx_fillRect(0, 222, 320, 18, 0x0000U);
    gfx_setTextSize(1);

    if (estado == ESTADO_IDLE) {
        gfx_setTextColor(0xFFFFU);
        gfx_drawText(40, 228, "Pressione um botao para iniciar");
    } else if (estado == ESTADO_MOSTRANDO) {
        gfx_setTextColor(0x07FFU);  /* ciano */
        gfx_drawText(75, 228, "Observe a sequencia...");
    } else if (estado == ESTADO_AGUARDANDO) {
        gfx_setTextColor(0xFFE0U);  /* amarelo */
        gfx_drawText(88, 228, "Repita a sequencia!");
    } else if (estado == ESTADO_ACERTO) {
        gfx_setTextColor(0x07E0U);  /* verde */
        gfx_drawText(98, 228, "Acertou! Avancando...");
    } else if (estado == ESTADO_ERRO) {
        gfx_setTextColor(0xF800U);  /* vermelho */
        gfx_drawText(118, 228, "Errou! Fim de jogo.");
    }
}

/** Atualiza o placar exibido no canto superior direito */
static void lcd_atualiza_pontuacao(int pontuacao) {
    char buf[16];
    gfx_fillRect(230, 5, 90, 20, 0x0000U);
    gfx_setTextSize(2);
    gfx_setTextColor(0xFFE0U);
    sprintf(buf, "Pts:%d", pontuacao);
    gfx_drawText(232, 7, buf);
}

/* ============================================================
 * CORE 0 — main
 *
 * Responsabilidade única: inicializar o LCD, lançar o Core 1,
 * e processar em loop as mensagens FIFO recebidas do Core 1
 * para atualizar o display.
 *
 * Não há acesso a GPIO de jogo, LEDs ou buzzer neste core.
 * ============================================================ */
int main(void) {
    stdio_init_all();

    /* ### Inicialização do LCD — biblioteca primeiro
     * LCD_initDisplay() configura internamente os pinos SPI e o backlight (GP15).
     * Só controlamos o GP15 depois que a lib terminou de inicializar. */
    LCD_initDisplay();
    LCD_setRotation(SCREEN_ROTATION);
    gfx_init();
    gfx_clear();

    /* ### Backlight ligado na tela inicial para o jogador ver a mensagem
     * de "Pressione um botão para iniciar" */
    gpio_init(15);
    gpio_set_dir(15, GPIO_OUT);
    gpio_put(15, 1);

    /* ### Lança Core 1 com a lógica do jogo. */
    multicore_launch_core1(core1_entry);

    /* Estado local do Core 0 */
    int pontuacao = 0;

    /* Desenha o layout base da tela */
    lcd_desenha_layout_base();

    /* Exibe pontuação inicial antes das mensagens do Core 1 chegarem */
    lcd_atualiza_pontuacao(pontuacao);

    /* ### Loop principal do Core 0 */
    while (true) {
        uint32_t msg;
        if (multicore_fifo_pop_timeout_us(0, &msg)) {
            uint32_t tipo = msg & 0xFF000000U;
            uint32_t dado = msg & 0x00FFFFFFU;

            if (tipo == MSG_ESTADO) {
                /* Controle do backlight:
                 * - Liga quando o jogo começa (MOSTRANDO ou AGUARDANDO)
                 * - Desliga quando volta ao IDLE após fim de jogo */
                if (dado == ESTADO_MOSTRANDO) {
                    gpio_put(15, 0);  /* acende backlight */
                } else if (dado == ESTADO_IDLE) {
                    gpio_put(15, 1);  /* apaga backlight */
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