#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_i2c.h"

#include "oled_driver.h"
#include "xprintf.h"
#include <string.h>

static uint8_t gmem[GMEM_SIZE] = {0};
static uint8_t curX = 0;
static uint8_t curY = 0;

extern font_desc_t font_desc;

void i2c_gpio_config(void)
{
    // SCL - GPIOB6
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6,
                       LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_6,
                             LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_6, LL_GPIO_AF_1);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_6,
                        LL_GPIO_SPEED_FREQ_HIGH);

    // SDA - GPIOB7
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7,
                       LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_7,
                             LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_7, LL_GPIO_AF_1);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_7,
                        LL_GPIO_SPEED_FREQ_HIGH);
    return;
}

static void oled_hw_config(void)
{
    /*
     * Clock on the I2C port and configure it
     */
    i2c_gpio_config();

    /*
     * Clock on the I2C peripheral and set it up
     */
    LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_SYSCLK);
    LL_I2C_Disable(I2C1);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
    LL_I2C_DisableAnalogFilter(I2C1);
    LL_I2C_SetDigitalFilter(I2C1, 1);
    LL_I2C_EnableDMAReq_TX(I2C1);
    /*
     * Set I2C speed to 400 kHz, for further details refer
     * to lecture
     */
    LL_I2C_SetTiming(I2C1, __LL_I2C_CONVERT_TIMINGS(5, 3, 3, 3, 9));
    LL_I2C_DisableClockStretching(I2C1);
    LL_I2C_SetMasterAddressingMode(I2C1, LL_I2C_ADDRESSING_MODE_7BIT);
    LL_I2C_SetMode(I2C1, LL_I2C_MODE_I2C);
    LL_I2C_Enable(I2C1);

    return;
}

static uint8_t oled_cmd_send(uint8_t byte)
{
    /*
     * Initiate transmission
     * Display address = 0x78
     */
    LL_I2C_HandleTransfer(I2C1, 0x78, LL_I2C_ADDRSLAVE_7BIT,
                          2, LL_I2C_MODE_AUTOEND,
                          LL_I2C_GENERATE_START_WRITE);
    /*
     * Send Control byte (Co = 0, D/C# = 0)
     */
    while (!LL_I2C_IsActiveFlag_TXIS(I2C1));
    LL_I2C_TransmitData8(I2C1, 0x00);
    /*
     * Send cmd
     */
    while (!LL_I2C_IsActiveFlag_TXIS(I2C1));
    LL_I2C_TransmitData8(I2C1, byte);
    /*
     * Check for end of transmission
     */
    while (LL_I2C_IsActiveFlag_TC(I2C1));
    return 0;
}

static void dma_config(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2,
                                    LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2,
                            LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2,
                            LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2,
                                   LL_DMA_PRIORITY_VERYHIGH);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
    NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0);
    NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
    
    return;
}

static uint8_t oled_data_send(uint8_t *byte, uint8_t size)
{
    /*
     * Initiate transmission
     * Display address = 0x78
     */
    LL_I2C_HandleTransfer(I2C1, 0x78, LL_I2C_ADDRSLAVE_7BIT,
                          size + 1, LL_I2C_MODE_AUTOEND,
                          LL_I2C_GENERATE_START_WRITE);
    /*
     * Send Control byte (Co = 0, D/C# = 1)
     */
    while (!LL_I2C_IsActiveFlag_TXIS(I2C1));
    LL_I2C_TransmitData8(I2C1, 0x40);
    /*
     * Send data
     */
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, size);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2, LL_I2C_DMA_GetRegAddr(I2C1, LL_I2C_DMA_REG_DATA_TRANSMIT));
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t) byte);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);

    return 0;
}

void oled_clr(enum color_t color)
{
    memset(gmem, color, GMEM_SIZE);
    return;
}

static uint8_t row_number = 0;

void oled_update(void)
{   
    // Set page start address (number of vertical byte)
    oled_cmd_send(0xB0 + row_number);
    // Set lower column number
    oled_cmd_send(0x00);
    // Set higher column number
    oled_cmd_send(0x10);
    // Send 128 pixels
    oled_data_send(&gmem[GMEM_WIDTH * row_number], GMEM_WIDTH);
    row_number = (row_number + 1) % 8;

    return;
}

void DMA1_Channel2_3_IRQHandler(void)
{
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
    if (row_number)
        oled_update();

    LL_DMA_ClearFlag_TC2(DMA1);

    return;
}

void oled_config(void)
{
    // Config DMA
    dma_config();

    uint32_t delay = 2000000;
    // Config hardware
    oled_hw_config();

    // Wait a bit
    while (delay--);

    // Set display OFF
    oled_cmd_send(0xAE);

    // Set addressing mode
    // Vertical addressing mode
    oled_cmd_send(0x20);
    oled_cmd_send(0x10);

    // Vertical flip: 0xC0 - on, 0xC8 - off
    oled_cmd_send(0xC8);

    // Set start line address 0-63
    oled_cmd_send(0x40);

    // Set contrast level: 0-255
    oled_cmd_send(0x81);
    oled_cmd_send(0xFF);

    // Horizontal flip: 0xA1 - on, 0xA0 - off
    oled_cmd_send(0xA1);

    // Normal colo - 0xA6, Inverse - 0xA7
    oled_cmd_send(0xA6);

    // Number of active lines: 16 - 64
    oled_cmd_send(0xA8);
    oled_cmd_send(0x3F);

    // Render GRAM: 0xA4 - render, 0xA5 - black screen
    oled_cmd_send(0xA4);

    // Set display offset: 0-63
    oled_cmd_send(0xD3);
    oled_cmd_send(0x00);

    // Set display refresh rate:
    // 7-4 bits set osc freq, 0-3 sets resfresh ratio
    oled_cmd_send(0xD5);
    oled_cmd_send(0xF0);

    // Set flipping config
    oled_cmd_send(0xDA);
    oled_cmd_send(0x12);

    // Enable charge pump
    oled_cmd_send(0x8D);
    oled_cmd_send(0x14);

    // Turn on display
    oled_cmd_send(0xAF);

    oled_clr(clBlack);
    oled_update();
    return;
}

void oled_set_pix(uint8_t y, uint8_t x, enum color_t color)
{
    if (x >= GMEM_WIDTH || y >= GMEM_HEIGHT)
        return;

    if (color != clBlack)
        gmem[GMEM_WIDTH * (y / 8) + x] |= 1 << (y % 8);
    else
        gmem[GMEM_WIDTH * (y / 8) + x] &= ~(1 << (y % 8));
    return;
}

void oled_set_block(uint8_t x, uint8_t y)
{
    uint8_t i, j;

    for (i = 0; i < BLOCK_SIZE; i++)
        for (j = 0; j < BLOCK_SIZE; j++)
        {
            if (!(i % 2 && j && j < 4) && !(j % 2 && i && i < 4))
                oled_set_pix(x + i, y + j, clWhite);
        }
    return;
}

void oled_reset_block(uint8_t x, uint8_t y)
{
    uint8_t i, j;

    for (i = 0; i < BLOCK_SIZE; i++)
        for (j = 0; j < BLOCK_SIZE; j++)
            oled_set_pix(x + i, y + j, clBlack);

    return;
}

void oled_set_lines(uint8_t x, uint8_t y)
{
    uint8_t i, j;
    for (i = X_SIZE - 1; i; i--)
    {
        x += BLOCK_SIZE + 1;
        for (j = y; j <= y + Y_SIZE * BLOCK_SIZE; j+=2)
            oled_set_pix(x, j, clWhite);
    }
    return;
}

void oled_set_bar(uint8_t x, uint8_t y)
{
    uint8_t i, j;
    for (i = 0; i < BLOCK_SIZE; i++)
        oled_set_pix(x, y + i, clBlack);
    for (i = 0; i < BLOCK_SIZE; i++)
        oled_set_pix(x + BLOCK_SIZE * X_SIZE + X_SIZE - 2, y + i, clBlack);
    for (i = 0; i < BLOCK_SIZE; i++)
        for (j = 1; j < BLOCK_SIZE * X_SIZE + X_SIZE - 2; j++)
        {
            if (i % (BLOCK_SIZE - 1) == 0)
                oled_set_pix(x + j, y + i, clBlack);
            else
                oled_set_pix(x + j, y + i, clWhite);
        }
    return;
}

void oled_set_field(uint8_t x, uint8_t y, uint8_t field[X_SIZE][Y_SIZE])
{
    uint8_t i, j;

    for (i = 0; i < X_SIZE; i++)
    {
        for (j = 0; j < Y_SIZE; j++)
        {
            if (field[X_SIZE - 1 - i][j])
                oled_set_block(x + i * BLOCK_SIZE, y + j * BLOCK_SIZE);
            else
                oled_reset_block(x + i * BLOCK_SIZE, y + j * BLOCK_SIZE);
        }
        x++;
    }
    return;
}

void oled_set_frame(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, enum color_t color)
{
    x1 = GMEM_HEIGHT - 1 - x1;
    x2 = GMEM_HEIGHT - 1 - x2;
    
    uint8_t i;

    for (i = y1; i <= y2; i++)
        oled_set_pix(x1, i, color);
    for (i = y1; i <= y2; i++)
        oled_set_pix(x2, i, color);
    for (i = x2; i <= x1; i++)
    {
        oled_set_pix(i, y1, color);
        oled_set_pix(i, y2, color);
    }
    return;
}

void oled_reset_next(uint8_t figure)
{
    uint8_t i;

    switch (figure) {

        case I_shape:
            for (i = 0; i < 4; i++)
                oled_reset_block(0 + i * BLOCK_SIZE, 7);
            break;
        case J_shape:
            for (i = 0; i < 3; i++)
                oled_reset_block(3 + i * BLOCK_SIZE, 4);
            oled_reset_block(3, 4 + BLOCK_SIZE);
            break;
        case L_shape:
            for (i = 0; i < 3; i++)
                oled_reset_block(3 + i * BLOCK_SIZE, 4);
            oled_reset_block(3 + 2 * BLOCK_SIZE, 4 + BLOCK_SIZE);
            break;
        case O_shape:
            for (i = 0; i < 2; i++)
                oled_reset_block(5 + i * BLOCK_SIZE, 4);
            for (i = 0; i < 2; i++)
                oled_reset_block(5 + i * BLOCK_SIZE, 4 + BLOCK_SIZE);
            break;
        case S_shape:
            for (i = 0; i < 2; i++)
                oled_reset_block(3 + i * BLOCK_SIZE, 4);
            for (i = 0; i < 2; i++)
                oled_reset_block(3 + BLOCK_SIZE + i * BLOCK_SIZE, 4 + BLOCK_SIZE);
            break;
        case T_shape:
            for (i = 0; i < 3; i++)
                oled_reset_block(3 + i * BLOCK_SIZE, 4);
            oled_reset_block(3 + BLOCK_SIZE, 4 + BLOCK_SIZE);
            break;
        case Z_shape:
            for (i = 0; i < 2; i++)
                oled_reset_block(3 + BLOCK_SIZE + i * BLOCK_SIZE, 4);
            for (i = 0; i < 2; i++)
                oled_reset_block(3 + i * BLOCK_SIZE, 4 + BLOCK_SIZE);
            break;
    }
    return;
}

void oled_set_next(uint8_t figure)
{
    uint8_t i;

    switch (figure) {

        case I_shape:
            for (i = 0; i < 4; i++)
                oled_set_block(0 + i * BLOCK_SIZE, 7);
            break;
        case J_shape:
            for (i = 0; i < 3; i++)
                oled_set_block(3 + i * BLOCK_SIZE, 4);
            oled_set_block(3, 4 + BLOCK_SIZE);
            break;
        case L_shape:
            for (i = 0; i < 3; i++)
                oled_set_block(3 + i * BLOCK_SIZE, 4);
            oled_set_block(3 + 2 * BLOCK_SIZE, 4 + BLOCK_SIZE);
            break;
        case O_shape:
            for (i = 0; i < 2; i++)
                oled_set_block(5 + i * BLOCK_SIZE, 4);
            for (i = 0; i < 2; i++)
                oled_set_block(5 + i * BLOCK_SIZE, 4 + BLOCK_SIZE);
            break;
        case S_shape:
            for (i = 0; i < 2; i++)
                oled_set_block(3 + i * BLOCK_SIZE, 4);
            for (i = 0; i < 2; i++)
                oled_set_block(3 + BLOCK_SIZE + i * BLOCK_SIZE, 4 + BLOCK_SIZE);
            break;
        case T_shape:
            for (i = 0; i < 3; i++)
                oled_set_block(3 + i * BLOCK_SIZE, 4);
            oled_set_block(3 + BLOCK_SIZE, 4 + BLOCK_SIZE);
            break;
        case Z_shape:
            for (i = 0; i < 2; i++)
                oled_set_block(3 + BLOCK_SIZE + i * BLOCK_SIZE, 4);
            for (i = 0; i < 2; i++)
                oled_set_block(3 + i * BLOCK_SIZE, 4 + BLOCK_SIZE);
            break;
    }
    return;
}

void oled_set_button(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, const char *name, enum color_t color)
{
    oled_set_frame(x1, y1, x2, y2, color);
    oled_set_frame(x1 - 1, y1 - 1, x2 - 1, y2 - 1, clWhite - color);

    uint8_t i, j;
    for (i = x1; i <= x2 - 2; i++)
        for (j = y1; j <= y2 - 2; j++)
            oled_set_pix(GMEM_HEIGHT - 1 - i, j, color);

    oled_set_cursor((x1 + x2 - strlen(name) * (font_desc.width + 1)) / 2, (y1 + y2 - font_desc.height) / 2);

    if (color == clWhite)
    {
        xdev_out(oled_inverse_putc);
        xprintf(name);
        xdev_out(oled_putc);
    }
    else
        xprintf(name);

    return;
}

void oled_set_cursor(uint8_t x, uint8_t y)
{
    curX = x;
    curY = y;
    return;
}

void oled_putc(char ch)
{
    uint8_t i, j;
    uint8_t color;

    if (ch == '\n') {
        curY += font_desc.height + 1;
        return;
    }

    if (ch == '\r') {
        curX = 0;
        return;
    }

    for (j = 0; j < font_desc.height; j++)
        for (i = 0; i < font_desc.width; i++)
        {
            color = font_desc.get_pix(ch, i, j);
            oled_set_pix(GMEM_HEIGHT - 1 - (curX + i), curY + j, color);
        }

    curX += font_desc.width + 1;
    return;
}

void oled_inverse_putc(char ch)
{
    uint8_t i, j;
    uint8_t color;

    if (ch == '\n') {
        curY += font_desc.height + 1;
        return;
    }

    if (ch == '\r') {
        curX = 0;
        return;
    }

    for (j = 0; j < font_desc.height; j++)
        for (i = 0; i < font_desc.width; i++)
        {
            color = !font_desc.get_pix(ch, i, j);
            oled_set_pix(GMEM_HEIGHT - 1 - (curX + i), curY + j, color);
        }

    curX += font_desc.width + 1;
    return;
}