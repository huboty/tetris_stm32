#ifndef __OLED_DRIVER__
#define __OLED_DRIVER__

#define GMEM_WIDTH 128
#define GMEM_HEIGHT 64
#define GMEM_SIZE (GMEM_WIDTH * GMEM_HEIGHT / 8)

#define X_SIZE      10
#define Y_SIZE      20
#define BLOCK_SIZE  5

enum FIGURES {
    I_shape,
    J_shape,
    L_shape,
    O_shape,
    S_shape,
    T_shape,
    Z_shape
};

/*
 * Simple strucutre to define fonts
 * Refer to font5x7.c for example
 */
typedef struct {
    const uint8_t *font;
    uint8_t width;
    uint8_t height;
    uint8_t (*get_pix)(uint8_t, uint8_t, uint8_t);
} font_desc_t;

enum color_t {
    clBlack = 0x00,
    clWhite = 0xFF
};

void i2c_gpio_config(void);

/*
 * initialize display
 */
void oled_config(void);

/*
 * clear graphic memory
 */
void oled_clr(enum color_t);

/*
 * flush graphic mem to display
 */
void oled_update(void);

/*
 * Put a pixel with a specified color
 */
void oled_set_pix(uint8_t x, uint8_t y, enum color_t color);

void oled_set_block(uint8_t x, uint8_t y);

void oled_reset_block(uint8_t x, uint8_t y);

void oled_set_lines(uint8_t x, uint8_t y);

void oled_set_bar(uint8_t x, uint8_t y);

void oled_set_field(uint8_t x, uint8_t y, uint8_t field[X_SIZE][Y_SIZE]);

void oled_set_frame(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, enum color_t color);

void oled_reset_next(uint8_t figure);

void oled_set_next(uint8_t figure);

void oled_set_button(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, const char *name, enum color_t color);

/*
 * Put a character
 */
void oled_putc(char ch);

void oled_inverse_putc(char ch);

/*
 * Set current cursor
 */
void oled_set_cursor(uint8_t x, uint8_t y);

#endif
