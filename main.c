/*
 * This is my realization of NES Tetris for STM32
 */

#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_i2c.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_adc.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_rtc.h"
#include "stm32f0xx_ll_pwr.h"

#include "xprintf.h"
#include "oled_driver.h"
#include <stdio.h>

#define X_SIZE          10
#define Y_SIZE          20
#define BLOCK_SIZE      5
#define N_LEVELS        30
#define N_FIGURES       7
#define STATE_SIZE      4
#define RECORDS_N       3

#define FIELD_X         Cursor.X + figure_arrays[figure][i + figure_state * STATE_SIZE][0]
#define FIELD_Y         Cursor.Y + figure_arrays[figure][i + figure_state * STATE_SIZE][1]
#define FIELD           field[FIELD_X][FIELD_Y]
#define NEW_FIELD_X     Cursor.X + figure_arrays[figure][i + new_figure_state * STATE_SIZE][0]
#define NEW_FIELD_Y     Cursor.Y + figure_arrays[figure][i + new_figure_state * STATE_SIZE][1]
#define NEW_FIELD       field[NEW_FIELD_X][NEW_FIELD_Y]
#define FIGURE_BOTTOM   Cursor.Y + figure_arrays[figure][3 + figure_state * STATE_SIZE][1]
#define FIGURE_TOP      Cursor.Y + figure_arrays[figure][0 + figure_state * STATE_SIZE][1]

const uint8_t DAS_MAX           = 16;
const uint8_t DAS_SHIFT         = 10;

const uint16_t ADC_RESOLUTION   = 0x3FF;
const uint16_t LOWTRH           = (uint16_t) (0.03 * ADC_RESOLUTION);
const uint16_t HIGHTRH          = (uint16_t) (0.97 * ADC_RESOLUTION);

const uint32_t MAX_SCORE        = 999999;
const uint8_t DROUGHT           = 12;

const uint8_t GAME_OVER_WAIT    = 20;
const uint8_t SHIFT_OPTION_WAIT = 1;

static void rcc_config()
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

    LL_RCC_HSI_Enable();
    while (LL_RCC_HSI_IsReady() != 1);

    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2,
                                LL_RCC_PLL_MUL_16);

    LL_RCC_PLL_Enable();
    while (LL_RCC_PLL_IsReady() != 1);

    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

    SystemCoreClock = 64000000;

    return;
}

static void exti_config()
{
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);

    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_0);

    NVIC_EnableIRQ(EXTI0_1_IRQn);
    NVIC_SetPriority(EXTI0_1_IRQn, 0);
    return;
}

static void rtc_config(void)
{
    LL_RCC_LSI_Enable();
    while (LL_RCC_LSI_IsReady() != 1);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    LL_PWR_EnableBkUpAccess();

    LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
    LL_RCC_EnableRTC();

    LL_RTC_DisableWriteProtection(RTC);
    LL_RTC_EnableInitMode(RTC);
    while (!LL_RTC_IsActiveFlag_INIT(RTC));

    LL_RTC_SetAsynchPrescaler(RTC, 0x7F);
    LL_RTC_SetSynchPrescaler(RTC, 0x0137);

    LL_RTC_SetHourFormat(RTC, LL_RTC_HOURFORMAT_24HOUR);
    LL_RTC_DATE_Config(RTC, LL_RTC_WEEKDAY_FRIDAY, 24,
                       LL_RTC_MONTH_MARCH, 0x2023);
    LL_RTC_TIME_Config(RTC, LL_RTC_TIME_FORMAT_AM_OR_24, 0x12, 0x38, 00);

    LL_RTC_DisableInitMode(RTC);
    LL_RTC_EnableWriteProtection(RTC);

    return;
}

static void gpio_config(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_5, LL_GPIO_PULL_UP);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_5, LL_GPIO_AF_2);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_5);
    return;
}

static void gpio_off(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA | LL_AHB1_GRP1_PERIPH_GPIOB |
                             LL_AHB1_GRP1_PERIPH_GPIOC | LL_AHB1_GRP1_PERIPH_GPIOD |
                             LL_AHB1_GRP1_PERIPH_GPIOF);

    GPIOA->MODER = 0xFFFFFFFF;
    GPIOB->MODER = 0xFFFFFFFF;
    GPIOC->MODER = 0xFFFFFFFF;
    GPIOD->MODER = 0xFFFFFFFF;
    GPIOF->MODER = 0xFFFFFFFF;    

    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_5);

    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);

    LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_GPIOA | LL_AHB1_GRP1_PERIPH_GPIOB |
                              LL_AHB1_GRP1_PERIPH_GPIOC | LL_AHB1_GRP1_PERIPH_GPIOD |
                              LL_AHB1_GRP1_PERIPH_GPIOF);
    return;
}

static void timers_config(void)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
    LL_TIM_SetPrescaler(TIM3, 63999);
    LL_TIM_SetAutoReload(TIM3, 9);
    LL_TIM_SetCounterMode(TIM3, LL_TIM_COUNTERMODE_UP);
    LL_TIM_EnableIT_UPDATE(TIM3);
    LL_TIM_EnableCounter(TIM3);

    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_SetPriority(TIM3_IRQn, 0);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    LL_TIM_SetPrescaler(TIM2, 639999);
    LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV32_N8);
    LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1,
                          LL_TIM_IC_POLARITY_BOTHEDGE);
    LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH1,
                             LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV2);
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
    LL_TIM_GenerateEvent_UPDATE(TIM2);
    LL_TIM_EnableIT_CC1(TIM2);
    LL_TIM_EnableCounter(TIM2);

    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 1);
    return;
}

void TIM3_IRQHandler(void)
{
    LL_ADC_REG_StartConversion(ADC1);
    LL_TIM_ClearFlag_UPDATE(TIM3);
    return;
}

uint16_t adc_buffer[2] = {512, 512};

static void adc_config(void)
{
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);

    LL_RCC_HSI14_Enable();
    LL_ADC_SetClock(ADC1, LL_ADC_CLOCK_SYNC_PCLK_DIV4);

    if (LL_ADC_IsEnabled(ADC1))
        LL_ADC_Disable(ADC1);
    while (LL_ADC_IsEnabled(ADC1));
    LL_ADC_StartCalibration(ADC1);
    while (LL_ADC_IsCalibrationOnGoing(ADC1));

    LL_ADC_Enable(ADC1);
    LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_10B);
    LL_ADC_SetDataAlignment(ADC1, LL_ADC_DATA_ALIGN_RIGHT);
    LL_ADC_SetLowPowerMode(ADC1, LL_ADC_LP_MODE_NONE);
    LL_ADC_SetSamplingTimeCommonChannels(ADC1,
                                         LL_ADC_SAMPLINGTIME_41CYCLES_5);
    LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);
    LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_1 |
                                          LL_ADC_CHANNEL_2);
    LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
    LL_ADC_REG_SetOverrun(ADC1, LL_ADC_REG_OVR_DATA_PRESERVED);
    LL_ADC_SetAnalogWDMonitChannels(ADC1, LL_ADC_AWD_CHANNEL_1_REG);
    LL_ADC_SetAnalogWDMonitChannels(ADC1, LL_ADC_AWD_CHANNEL_2_REG);
    LL_ADC_ConfigAnalogWDThresholds(ADC1, HIGHTRH, LOWTRH);
    LL_ADC_EnableIT_AWD1(ADC1);

    NVIC_SetPriority(ADC1_COMP_IRQn, 1);
    NVIC_EnableIRQ(ADC1_COMP_IRQn);

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1,
                                    LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1,
                            LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1,
                            LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1,
                                   LL_DMA_PRIORITY_VERYHIGH);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 2);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA));
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t) adc_buffer);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);  
    return;
}

static void systick_config(void)
{
    LL_InitTick(64000000, 60);
    LL_SYSTICK_EnableIT();
    NVIC_SetPriority(SysTick_IRQn, 1);
    return;
}

static void printf_config(void)
{
    xdev_out(oled_putc);
    return;
}

static uint64_t next = 1;

static void srandom(uint32_t seed)
{
    next = seed;
    return;
}

static int16_t random(void)
{
    next = next * 1103515245 + 12345;
    return (uint16_t)(next / 65536) % 32768;
}

static uint32_t min(uint32_t a, uint32_t b)
{
    return a < b ? a : b;
}

static uint32_t max(uint32_t a, uint32_t b)
{
    return a > b ? a : b;
}

static uint8_t field[X_SIZE][Y_SIZE] = {0};

typedef struct {
    int8_t X;
    int8_t Y;
} cursor;

cursor Cursor;
const cursor FieldCoords = {2, 26};

enum STATES {
    menu,
    choose_level,
    shift,
    fall,
    hard,
    idle,
    clear,
    spawn_shift,
    spawn,
    game_over,
    choose_option
};

enum BUTTONS {
    play,
    drop_state,
    kick_state,
    quit,
    restart,
    to_menu
};

enum FIGURE_STATES {
    state_0,
    state_1,
    state_2,
    state_3
};

enum FIGURES {
    I_shape,
    J_shape,
    L_shape,
    O_shape,
    S_shape,
    T_shape,
    Z_shape
};

enum DIRECTIONES {
    no,
    left,
    right,
    up,
    down
};

enum ON_OFF {
    off = 0,
    on = 1
};

enum SOFT_DROP_SPEED {
    drop = 2,
    no_drop = 99
};

enum WALL_KICK {
    to_left = 2,
    to_right = 3,
    no_way = 4
};

int8_t figure_arrays [N_FIGURES][STATE_SIZE * 4][2] = {{{0, 0}, {1, 0}, {2, 0}, {3, 0}, {2, -2}, {2, -1}, {2, 0}, {2, 1}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}},      // I
                                    {{0, 0}, {1, 0}, {2, 0}, {2, 1}, {1, -1}, {1, 0}, {0, 1}, {1, 1}, {0, -1}, {0, 0}, {1, 0}, {2, 0}, {1, -1}, {2, -1}, {1, 0}, {1, 1}},                       // J
                                    {{1, 0}, {2, 0}, {3, 0}, {1, 1}, {1, -1}, {2, -1}, {2, 0}, {2, 1}, {3, -1}, {1, 0}, {2, 0}, {3, 0}, {2, -1}, {2, 0}, {2, 1}, {3, 1}},                       // L
                                    {{0, 0}, {1, 0}, {0, 1}, {1, 1}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}},                           // O
                                    {{1, 0}, {2, 0}, {0, 1}, {1, 1}, {1, -1}, {1, 0}, {2, 0}, {2, 1}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}},                          // S
                                    {{0, 0}, {1, 0}, {2, 0}, {1, 1}, {1, -1}, {0, 0}, {1, 0}, {1, 1}, {1, -1}, {0, 0}, {1, 0}, {2, 0}, {1, -1}, {1, 0}, {2, 0}, {1, 1}},                        // T
                                    {{0, 0}, {1, 0}, {1, 1}, {2, 1}, {2, -1}, {1, 0}, {2, 0}, {1, 1}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}}};                         // Z


static uint8_t state                    = 0;
static uint8_t first_shift              = 0;

static uint8_t figure = 0, next_figure = 0;
static uint8_t figure_state             = 0;

static uint8_t min_depth = Y_SIZE - 1;

static uint8_t frames                   = 0;
const uint8_t level_frames[N_LEVELS]    = {48, 43, 38, 33, 28, 23, 18, 13, 8, 6, 5, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1};
static uint8_t start_lvl                = 0;
static uint8_t lvl                      = 0;

static uint8_t idle_time                = 0;

static uint8_t clear_lines[4]           = {0, 0, 0, 0};
static uint8_t n_lines                  = 0;
static uint32_t total_lines             = 0;
static uint32_t advance_lines           = 0;

static uint32_t score                   = 0;
static uint8_t soft_score               = 0;

static uint8_t das_counter              = 0;
static uint8_t soft_drop                = no_drop;
static uint8_t direction                = no;
static uint8_t hard_drop                = off;
static uint8_t wall_kick                = off;

static uint32_t I_cntr                  = 0;
static uint32_t trts                    = 0;
static uint8_t total_droughts           = 0;
static uint8_t drought_pieces           = 0;
static uint8_t max_drought              = 0;
static uint8_t curr_drought             = 0;

static uint8_t selected_button          = 0;

static void delay(void)
{
    int32_t i;
    for (i = 100000; i; i--);
    return;
}

static void set_screen(void)
{
    oled_clr(clBlack);
    oled_set_frame(FieldCoords.X, FieldCoords.Y - 1, FieldCoords.X + BLOCK_SIZE * X_SIZE + X_SIZE, FieldCoords.Y + BLOCK_SIZE * Y_SIZE, clWhite);
    oled_set_lines(FieldCoords.X - 1, FieldCoords.Y - 1);
    oled_set_frame(0, 0, 44, 8, clWhite);
    oled_set_frame(3, 8, 41, 25, clWhite);
    oled_set_cursor(8, 10);
    xprintf("SCORE");
    oled_set_frame(44, 0, 63, 25, clWhite);
    oled_set_cursor(46, 18);
    xprintf("NXT");
    return;
}

static void update_score(void)
{
    oled_set_cursor(5, 18);
    if (score > MAX_SCORE)
        score = MAX_SCORE;
    xprintf("%06d", score);
    return;
}

static void update_lines(void)
{
    oled_set_cursor(2, 1);
    xprintf("LNS-%03d", total_lines);
    return;
}

static void update_spawn(void)
{
    oled_reset_next(figure);
    oled_set_frame(44, 0, 63, 17, clWhite);
    oled_set_next(next_figure);
    return;
}

static void play_start(void)
{
    int8_t i, j;
    for (i = 0; i < X_SIZE; i++)
        for (j = 0; j < Y_SIZE; j++)
            field[i][j] = 0;

    min_depth       = Y_SIZE - 1;
    score           = 0;
    soft_score      = 0;
    total_lines     = 0;
    I_cntr          = 0;
    trts            = 0;
    total_droughts  = 0;
    drought_pieces  = 0;
    max_drought     = 0;
    curr_drought    = 0;
    das_counter     = 0;
    soft_drop       = no_drop;
    direction       = no;

    set_screen();
    update_lines();
    update_score();
    next_figure = random() % N_FIGURES;
    lvl = start_lvl;
    advance_lines = min(start_lvl * 10 + 10, max(100, start_lvl * 10 - 50));
    state = spawn;
    return;
}

static void menu_start(void)
{   
    selected_button = play;
    oled_clr(clBlack);
    oled_set_cursor (16, 10);
    xprintf("TETRIS");
    oled_set_button(7, 30, 59, 50, "PLAY", clWhite);
    if (wall_kick == off)
        oled_set_button(7, 55, 59, 75, "KICK OFF", clBlack);
    else
        oled_set_button(7, 55, 59, 75, "KICK ON", clBlack);
    if (hard_drop == off)
        oled_set_button(7, 80, 59, 100, "DROP OFF", clBlack);
    else
        oled_set_button(7, 80, 59, 100, "DROP ON", clBlack);
    oled_set_button(7, 105, 59, 125, "QUIT", clBlack);
    oled_update();
    state = menu;
    return;
}

static void set_levels(void)
{
    char digit[2] = "0";
    int8_t i;
    uint8_t x = 3, y = 25;

    for (i = 0; i < 10; i++)
    {
        digit[0] = i + '0';
        if (i == selected_button)
            oled_set_button(x + (i % 5) * 12, y + (i / 5) * 12, x + 9 + (i % 5) * 12, y + 9 + (i / 5) * 12, digit, clWhite);
        else
            oled_set_button(x + (i % 5) * 12, y + (i / 5) * 12, x + 9 + (i % 5) * 12, y + 9 + (i / 5) * 12, digit, clBlack);
    }
    oled_update();
    return;
}

static uint32_t hi_scores[RECORDS_N]    = {0};
static uint8_t hi_levels[RECORDS_N]     = {0};

static void levels_start(void)
{
    int8_t i;

    selected_button = 0;
    oled_clr(clBlack);
    oled_set_cursor(17, 6);
    xprintf("LEVEL");
    oled_set_frame(14, 4, 48, 14, clWhite);
    oled_set_cursor(3, 60);
    xprintf("  SCORE LV");
    for (i = 0; i < RECORDS_N; i++)
    {
        oled_set_cursor(3, 72 + i * 10);
        xprintf("%d", i + 1);
        oled_set_cursor(12, 72 + i * 10);
        xprintf("%06d", hi_scores[i]);
        oled_set_cursor(51, 72 + i * 10);
        xprintf("%02d", hi_levels[i]);
    }
    oled_set_frame(0, 57, 63, 69, clWhite);
    oled_set_frame(0, 69, 63, 101, clWhite);
    set_levels();
    state = choose_level;
    return;
}

static void stats_screen(void)
{
    int8_t i, j;

    for (i = 0; i < RECORDS_N; i++)
        if (score > hi_scores[i])
        {
            for (j = RECORDS_N - 1; j > i; j--)
            {  
                hi_scores[j] = hi_scores[j - 1];
                hi_levels[j] = hi_levels[j - 1];
            }
            hi_scores[i] = score;
            hi_levels[i] = lvl;
            break;                
        }
            
    oled_clr(clBlack);
    oled_set_frame(0, 0, 63, 127, clWhite);
    oled_set_cursor(2, 2);
    xprintf("LEVEL   %02d", lvl);
    oled_set_cursor(2, 10);
    xprintf("SCR %6d", score);
    oled_set_cursor(2, 18);
    xprintf("LNS   %4d", total_lines);
    oled_set_cursor(2, 26);
    xprintf("TRT   %3d%%", total_lines > 0 ? trts * 400 / total_lines : 0);
    oled_set_cursor(2, 34);
    xprintf("I PCS %4d", I_cntr);
    oled_set_cursor(2, 42);
    xprintf("PPL %6d", total_lines > 0 ? score / total_lines : 0);
    oled_set_cursor(2, 50);
    xprintf("D.TOTAL %2d", total_droughts);
    oled_set_cursor(2, 58);
    xprintf("D.AVG   %2d", total_droughts > 0 ? drought_pieces / total_droughts : 0);
    oled_set_cursor(2, 66);
    xprintf("D.MAX   %2d", max_drought);
    oled_set_button(7, 80, 59, 100, "RESTART", clWhite);
    oled_set_button(7, 105, 59, 125, "TO MENU", clBlack);
    oled_update();
    selected_button = restart;
    state = choose_option;
    return;
}

void EXTI0_1_IRQHandler()
{
    rcc_config();

    LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_0);
    LL_EXTI_DisableRisingTrig_0_31(LL_EXTI_LINE_0);
    NVIC_DisableIRQ(EXTI0_1_IRQn);

    gpio_config();
    i2c_gpio_config();
    menu_start();

    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
}

void ADC1_COMP_IRQHandler(void)
{
    switch (state) {

        case menu:

            if ((adc_buffer[0] & ADC_RESOLUTION) >= HIGHTRH)
            {
                direction = down;
                if (!frames)
                    frames = SHIFT_OPTION_WAIT;
            }
            else if ((adc_buffer[0] & ADC_RESOLUTION) <= LOWTRH)
            {
                direction = up;
                if (!frames)
                    frames = SHIFT_OPTION_WAIT;
            }
            break;

        case choose_level:

            if ((adc_buffer[0] & ADC_RESOLUTION) >= HIGHTRH)
            {
                direction = down;
                if (!frames)
                    frames = SHIFT_OPTION_WAIT;
            }
            else if ((adc_buffer[0] & ADC_RESOLUTION) <= LOWTRH)
            {
                direction = up;
                if (!frames)
                    frames = SHIFT_OPTION_WAIT;
            }
            else if ((adc_buffer[1] & ADC_RESOLUTION) >= HIGHTRH)
            {
                direction = right;
                if (!frames)
                    frames = SHIFT_OPTION_WAIT;
            }
            else if ((adc_buffer[1] & ADC_RESOLUTION) <= LOWTRH)
            {
                direction = left;
                if (!frames)
                    frames = SHIFT_OPTION_WAIT;
            }
            break;

        case fall:

            if (hard_drop == on && (adc_buffer[0] & ADC_RESOLUTION) <= LOWTRH)
            {
                state = hard;
                break;
            }

            if ((adc_buffer[0] & ADC_RESOLUTION) >= HIGHTRH)
                soft_drop = drop;

            if ((adc_buffer[1] & ADC_RESOLUTION) >= HIGHTRH)
            {
                direction = right;
                if (first_shift == on)
                    das_counter = 0;
                state = shift;
                break;
            }

            if ((adc_buffer[1] & ADC_RESOLUTION) <= LOWTRH)
            {
                direction = left;
                if (first_shift == on)
                    das_counter = 0;
                state = shift;
                break;
            }

            break;

        case spawn:

            if ((adc_buffer[0] & ADC_RESOLUTION) >= HIGHTRH)
                soft_drop = drop;

            if ((adc_buffer[1] & ADC_RESOLUTION) >= HIGHTRH)
            {
                direction = right;
                state = spawn_shift;
                break;
            }

            if ((adc_buffer[1] & ADC_RESOLUTION) <= LOWTRH)
            {
                direction = left;
                state = spawn_shift;
                break;
            }
            break;

        case choose_option:

            if ((adc_buffer[0] & ADC_RESOLUTION) >= HIGHTRH)
            {
                direction = down;
                if (!frames)
                    frames = SHIFT_OPTION_WAIT;
            }
            else if ((adc_buffer[0] & ADC_RESOLUTION) <= LOWTRH)
            {
                direction = up;
                if (!frames)
                    frames = SHIFT_OPTION_WAIT;
            }
            break;

        default:
            break;
    }

    LL_ADC_ClearFlag_AWD1(ADC1);
    return;
}

static int8_t check_wall_kick(void)
{
    if (wall_kick == on)                
    {
        wall_kick = to_left;
        Cursor.X--;
        return 1;
    }
    else if (wall_kick == to_left)
    {
        wall_kick = no_way;
        if (figure == I_shape)
            Cursor.X += 3;
        else
            Cursor.X += 2;
        return 1;
    }
    else if (wall_kick == no_way)
    {
        if (figure == I_shape)
            Cursor.X -= 2;
        else
            Cursor.X--;
        wall_kick = on;
    }
    return 0;
}

void TIM2_IRQHandler(void)
{
    int8_t i;

    switch (state) {

        case menu:

            if (selected_button == play)
                levels_start();
            else if (selected_button == kick_state)
            {
                if (wall_kick == off)
                {
                    wall_kick = on;
                    oled_set_button(7, 55, 59, 75, "KICK ON", clWhite);
                    oled_update();
                }
                else
                {
                    wall_kick = off;
                    oled_set_button(7, 55, 59, 75, "KICK OFF", clWhite);
                    oled_update();
                }
            }
            else if (selected_button == drop_state)
            {
                if (hard_drop == off)
                {
                    hard_drop = on;
                    oled_set_button(7, 80, 59, 100, "DROP ON", clWhite);
                    oled_update();
                }
                else
                {
                    hard_drop = off;
                    oled_set_button(7, 80, 59, 100, "DROP OFF", clWhite);
                    oled_update();
                }
            }
            else if (selected_button == quit)
            {
                oled_clr(clBlack);
                oled_update();

                delay();

                exti_config();
                gpio_off();
                LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
                LL_LPM_EnableDeepSleep();
                LL_PWR_SetPowerMode(LL_PWR_MODE_STOP_LPREGU);

                __WFI();
            }

            break;

        case choose_level:

            start_lvl = selected_button;
            play_start();
            break;

        case shift:

            __attribute__ ((fallthrough));

        case fall:

            for (i = 0; i < 4; i++)
                if (FIELD_Y >= 0)
                    FIELD = 0;

            uint8_t new_figure_state;

            switch (figure) {
            
                case I_shape:
                    new_figure_state = (figure_state + 1) % 2;
                    break;
                case J_shape:
                    new_figure_state = (figure_state + 1) % 4;
                    break;
                case L_shape:
                    new_figure_state = (figure_state + 1) % 4;
                    break;    
                case O_shape:
                    new_figure_state = figure_state;
                    break;
                case S_shape:
                    new_figure_state = (figure_state + 1) % 2;
                    break;
                case T_shape:
                    new_figure_state = (figure_state + 1) % 4;
                    break;
                case Z_shape:
                    new_figure_state = (figure_state + 1) % 2;
                    break;
            }

            for (i = 0; i < 4; i++)
            {
                if (NEW_FIELD_X < 0)
                {
                    if (check_wall_kick())
                    {
                        i = -1;
                        continue;
                    }
                    break;
                }
                if (NEW_FIELD_X > X_SIZE - 1)
                {
                    if (check_wall_kick())
                    {
                        i = -1;
                        continue;
                    }
                    break;
                }
                if (NEW_FIELD_Y > Y_SIZE - 1)
                    break;
                if (NEW_FIELD_Y >= 0)
                    if (NEW_FIELD == 1)
                    {
                        if (check_wall_kick())
                        {
                            i = -1;
                            continue;
                        }
                        break;
                    }
            }

            if (wall_kick != off)
                wall_kick = on;

            if (i == 4)
                figure_state = new_figure_state;

            for (i = 0; i < 4; i++)
                if (FIELD_Y >= 0)
                    FIELD = 1;

            oled_set_field(FieldCoords.X, FieldCoords.Y, field);
            oled_update();
            break;

        case choose_option:

            if (selected_button == to_menu)
                menu_start();
            else if (selected_button == restart)
                play_start();
            break;

        default:
            break;
    }

    LL_TIM_ClearFlag_CC1(TIM2);
    return;
}

static void goto_idle(void)
{
    if (soft_score)
    {
        score += soft_score;
        soft_score = 0;
        update_score();
    }

    uint8_t lock_depth = FIGURE_BOTTOM;
    min_depth = FIGURE_TOP < min_depth ? FIGURE_TOP : min_depth;

    frames = 0;
    state = idle;
    idle_time = 18 - 2 * ((lock_depth - 2) / 4);
    return;
}

void SysTick_Handler(void)
{
    int8_t i;

    switch (state) {

        case menu:

            if (frames)
            {
                frames--;
                break;
            }

            if (direction == down)
            {
                switch (selected_button) {
                    
                    case play:

                        selected_button = kick_state;
                        oled_set_button(7, 30, 59, 50, "PLAY", clBlack);
                        if (wall_kick == on)
                            oled_set_button(7, 55, 59, 75, "KICK ON", clWhite);
                        else
                            oled_set_button(7, 55, 59, 75, "KICK OFF", clWhite);
                        break;

                    case kick_state:

                        selected_button = drop_state;
                        if (wall_kick == on)
                            oled_set_button(7, 55, 59, 75, "KICK ON", clBlack);
                        else
                            oled_set_button(7, 55, 59, 75, "KICK OFF", clBlack);

                        if (hard_drop == on)
                            oled_set_button(7, 80, 59, 100, "DROP ON", clWhite);
                        else
                            oled_set_button(7, 80, 59, 100, "DROP OFF", clWhite);
                        break;

                    case drop_state:

                        selected_button = quit;
                        if (hard_drop == on)
                            oled_set_button(7, 80, 59, 100, "DROP ON", clBlack);
                        else
                            oled_set_button(7, 80, 59, 100, "DROP OFF", clBlack);
                        oled_set_button(7, 105, 59, 125, "QUIT", clWhite);
                        break;

                    case quit:

                        selected_button = play;
                        oled_set_button(7, 30, 59, 50, "PLAY", clWhite);
                        oled_set_button(7, 105, 59, 125, "QUIT", clBlack);
                        break;

                    default:
                        break;
                }
                direction = no;
                oled_update();
            }
            else if (direction == up)
            {
                switch (selected_button) {

                    case play:
    
                        selected_button = quit;
                        oled_set_button(7, 30, 59, 50, "PLAY", clBlack);
                        oled_set_button(7, 105, 59, 125, "QUIT", clWhite);
                        break;

                    case kick_state:

                        selected_button = play;
                        oled_set_button(7, 30, 59, 50, "PLAY", clWhite);
                        if (wall_kick == on)
                            oled_set_button(7, 55, 59, 75, "KICK ON", clBlack);
                        else
                            oled_set_button(7, 55, 59, 75, "KICK OFF", clBlack);
                        break;

                    case drop_state:

                        selected_button = kick_state;
                        if (wall_kick == on)
                            oled_set_button(7, 55, 59, 75, "KICK ON", clWhite);
                        else
                            oled_set_button(7, 55, 59, 75, "KICK OFF", clWhite);

                        if (hard_drop == on)
                            oled_set_button(7, 80, 59, 100, "DROP ON", clBlack);
                        else
                            oled_set_button(7, 80, 59, 100, "DROP OFF", clBlack);
                        break;

                    case quit:

                        selected_button = drop_state;
                        if (hard_drop == on)
                            oled_set_button(7, 80, 59, 100, "DROP ON", clWhite);
                        else
                            oled_set_button(7, 80, 59, 100, "DROP OFF", clWhite);
                        oled_set_button(7, 105, 59, 125, "QUIT", clBlack);
                        break;

                    default:
                        break;
                }
                direction = no;
                oled_update();
            }
            break;

        case choose_level:

            if (frames)
            {
                frames--;
                break;
            }

            if (direction == up || direction == down)
                selected_button = (selected_button + 5) % 10;
            else if (direction == left)
                selected_button = (selected_button + 9) % 10;
            else if (direction == right)
                selected_button = (selected_button + 1) % 10;

            direction = no;
            set_levels();
            break;

        case shift:

            frames = (frames + 1) % min(level_frames[min(lvl, N_LEVELS - 1)], soft_drop);

            if (das_counter % DAS_MAX == 0)
            {
                if (das_counter)
                    das_counter = DAS_SHIFT;

                for (i = 0; i < 4; i++)   
                    if (FIELD_Y >= 0)
                        FIELD = 0;

                if (direction == left)
                {
                    Cursor.X--;
                    for (i = 0; i < 4; i++)
                        if (FIELD_X < 0)
                        {
                            Cursor.X++;
                            das_counter = DAS_MAX;
                            break;
                        }
                }
                else if (direction == right)
                {
                    Cursor.X++;
                    for (i = 0; i < 4; i++)
                        if (FIELD_X > X_SIZE - 1)
                        {
                            Cursor.X--;
                            das_counter = DAS_MAX;
                            break;
                        }
                }

                if (das_counter != DAS_MAX)
                    for (i = 0; i < 4; i++)
                        if (FIELD_Y >= 0)
                            if (FIELD == 1)
                            {
                                if (direction == left)
                                    Cursor.X++;
                                else if (direction == right)
                                    Cursor.X--;

                                das_counter = DAS_MAX;
                                break;
                            }

                for (i = 0; i < 4; i++)
                    if (FIELD_Y >= 0)
                        FIELD = 1;

                if (das_counter != DAS_MAX && frames)
                {
                    oled_set_field(FieldCoords.X, FieldCoords.Y, field);
                    oled_update();
                }
            }

            das_counter = min(das_counter + 1, DAS_MAX);
            __attribute__ ((fallthrough));

        case fall:

            if (state == shift)
            {
                first_shift = off;
                state = fall;
            }
            else
            {
                first_shift = on;
                frames = (frames + 1) % min(level_frames[min(lvl, N_LEVELS - 1)], soft_drop);
            }

            if (soft_drop == no_drop)
                soft_score = 0;

            if (!frames)
            {
                for (i = 0; i < 4; i++)
                    if (FIELD_Y >= 0)
                        FIELD = 0;

                Cursor.Y++;

                if (FIGURE_BOTTOM == Y_SIZE)
                {
                    Cursor.Y--;
                    goto_idle();
                }
                else for (i = 0; i < 4; i++)
                    if (FIELD_Y >= 0)
                        if (FIELD == 1)
                        {
                            Cursor.Y--;
                            goto_idle();
                            break;
                        }

                for (i = 0; i < 4; i++)
                    if (FIELD_Y >= 0)
                        FIELD = 1;

                if (state != idle && soft_drop < level_frames[min(lvl, N_LEVELS - 1)])
                    soft_score++;

                oled_set_field(FieldCoords.X, FieldCoords.Y, field);
                oled_update();
            }
            soft_drop = no_drop;
            break;

        case hard:

            while (state == hard)
            {
                for (i = 0; i < 4; i++)
                    if (FIELD_Y >= 0)
                        FIELD = 0;

                Cursor.Y++;

                if (FIGURE_BOTTOM == Y_SIZE)
                {
                    Cursor.Y--;
                    goto_idle();
                }
                else for (i = 0; i < 4; i++)
                    if (FIELD_Y >= 0)
                        if (FIELD == 1)
                        {
                            Cursor.Y--;
                            goto_idle();
                            break;
                        }

                for (i = 0; i < 4; i++)
                    if (FIELD_Y >= 0)
                        FIELD = 1;

                if (state != idle)
                    soft_score++;
            }
            oled_set_field(FieldCoords.X, FieldCoords.Y, field);
            oled_update();
            break;

        case idle:

            frames = (frames + 1) % idle_time;
            if (!frames)
            {
                int8_t j;
                n_lines = 0;

                for (i = Y_SIZE - 1; i >= min_depth; i--)
                {
                    for (j = 0; j < X_SIZE && field[j][i]; j++);
                    if (j == X_SIZE)
                    {
                        state = clear;
                        clear_lines[n_lines] = i;
                        n_lines++;
                    }
                }

                min_depth += n_lines;

                if (state != clear)
                    state = spawn;
            }
            break;

        case clear:
            
            if (frames % 4 == 0)
            {
                if (n_lines == 4)
                {
                    oled_clr(clWhite);
                    oled_update();
                }
                for (i = 0; i < n_lines; i++)
                {
                    field[X_SIZE / 2 + frames / 4][clear_lines[i]] = 0;
                    field[X_SIZE / 2 - 1 - frames / 4][clear_lines[i]] = 0;
                }
                if (n_lines == 4)
                {
                    set_screen();
                    oled_set_field(FieldCoords.X, FieldCoords.Y, field);
                    update_lines();
                    update_score();
                    update_spawn();
                    oled_update();
                }
                else
                {
                    oled_set_field(FieldCoords.X, FieldCoords.Y, field);
                    oled_update();
                }
            }

            frames = (frames + 1) % (4 * X_SIZE / 2);

            if (!frames)
            {
                int8_t j, k;
                for (i = 0; i < 4; i++)
                {
                    for (j = clear_lines[i]; j > 0; j--)
                    {
                        for (k = 0; k < X_SIZE; k++)
                            field[k][j] = field[k][j - 1];
                    }
                    clear_lines[i] = 0;
                    for (k = i + 1; k < 4; k++)
                        clear_lines[k]++;
                }

                total_lines += n_lines;
                if (total_lines >= advance_lines)
                {
                    lvl++;
                    advance_lines += 10;
                }

                switch (n_lines)
                {
                    case 1:
                        score += (lvl + 1) * 40;
                        break;
                    case 2:
                        score += (lvl + 1) * 100;
                        break;
                    case 3:
                        score += (lvl + 1) * 300;
                        break;
                    case 4:
                        score += (lvl + 1) * 1200;
                        trts++;
                        break;
                    default:
                        break;
                }
                
                n_lines = 0;
                state = spawn;

                oled_set_field(FieldCoords.X, FieldCoords.Y, field);
                update_lines();
                update_score();
                oled_update();
            }
            break;

        case spawn_shift:

            __attribute__ ((fallthrough));

        case spawn:

            figure = next_figure;
            if (figure == I_shape)
            {
                I_cntr++;
                curr_drought = 0;
            }
            else
            {
                curr_drought++;

                if (curr_drought == DROUGHT)
                {
                    total_droughts++;
                    drought_pieces += 12;
                }
                else if (curr_drought > DROUGHT)
                {
                    drought_pieces++;
                    max_drought = max(max_drought, drought_pieces);
                }
            }

            while (next_figure == figure)
                next_figure = random() % N_FIGURES;

            figure_state = 0;
            Cursor.Y = 0;
            switch (figure)
            {
                case I_shape:
                    Cursor.X = 3;
                    break;

                case J_shape:
                    Cursor.X = 4;
                    break;

                case L_shape:
                    Cursor.X = 3;
                    break;

                case O_shape:
                    Cursor.X = 4;
                    break;

                case S_shape:
                    Cursor.X = 4;
                    break;

                case T_shape:
                    Cursor.X = 4;
                    break;

                case Z_shape:
                    Cursor.X = 4;
                    break;
            }

            if (min_depth <= 1)
            {
                for (i = 0; i < 4; i++)
                    if (FIELD_Y >= 0)
                        if (FIELD == 1)
                        {
                            state = game_over;
                            break;
                        }
            }

            for (i = 0; i < 4; i++)
                if (FIELD_Y >= 0)
                    FIELD = 1;

            oled_set_field(FieldCoords.X, FieldCoords.Y, field);
            update_spawn();
            oled_update();

            if (state == spawn_shift)
                state = shift;
            else if (state == spawn)
                state = fall;

            break;

        case game_over:

            if (frames >= GAME_OVER_WAIT && frames % 4 == 0)
            {
                oled_set_bar(FieldCoords.X, FieldCoords.Y + (frames - GAME_OVER_WAIT) / 4 * BLOCK_SIZE);
                oled_update();
            }

            frames = (frames + 1) % (4 * Y_SIZE + GAME_OVER_WAIT);

            if (!frames)
                stats_screen();
            break;

        case choose_option:

            if (frames)
            {
                frames--;
                break;
            }

            if (direction == up || direction == down)
            {
                if (selected_button == restart)
                {
                    selected_button = to_menu;
                    oled_set_button(7, 80, 59, 100, "RESTART", clBlack);
                    oled_set_button(7, 105, 59, 125, "TO MENU", clWhite);
                }
                else if (selected_button == to_menu)
                {
                    selected_button = restart;
                    oled_set_button(7, 80, 59, 100, "RESTART", clWhite);
                    oled_set_button(7, 105, 59, 125, "TO MENU", clBlack);
                }
                
                direction = no;
                oled_update();
            }
            break;

        default:
            break;
    }
    return;
}

int main(void)
{
    rcc_config();
    gpio_config();
    oled_config();
    timers_config();
    adc_config();
    printf_config();
    systick_config();
    rtc_config();

    srandom(LL_RTC_TIME_Get(RTC));

    delay();

    menu_start();

    while (1);
    return 0;
}