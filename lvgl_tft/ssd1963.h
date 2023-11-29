/*
 */

#ifndef SSD1963_C_
#define SSD1963_C_


/*********************
 *      INCLUDES
 *********************/
#include <stdbool.h>

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

#include "sdkconfig.h"


//ESP32_VS_HW2 Pinout
#define DC		27 //0x8000000 //CONFIG_LV_DISP_PIN_DC
#define WR		05 //0x20      //CONFIG_LV_DISP_PIN_DC
#define D0		12 //0x000FF000 <<12
#define D1		13
#define D2		14
#define D3		15
#define D4		16
#define D5		17
#define D6		18
#define D7		19




// Horizontal and vertical display resolution (from the glass datasheet).
#define DISP_HOR_RESOLUTION CONFIG_LV_HOR_RES_MAX
#define DISP_VER_RESOLUTION CONFIG_LV_VER_RES_MAX

// Horizontal synchronization timing in pixels (from the glass datasheet).
#define DISP_HOR_PULSE_WIDTH        41
#define DISP_HOR_BACK_PORCH         2
#define DISP_HOR_FRONT_PORCH        2

// Vertical synchronization timing in lines (from the glass datasheet).
#define DISP_VER_PULSE_WIDTH        10
#define DISP_VER_BACK_PORCH         2
#define DISP_VER_FRONT_PORCH        2



//-------------------------------------------------------------------------------------------------
// No operation
#define SSD1963_CMD_NOP (0x00)

// Software Reset
#define SSD1963_CMD_SOFT_RESET (0x01)

// Get the current power mode
#define SSD1963_CMD_GET_POWER_MODE (0x0A)

// Get the frame memory to the display panel read order
#define SSD1963_CMD_GET_ADDRESS_MODE (0x0B)

// Get the current pixel format
// Reserved
#define SSD1963_CMD_GET_PIXEL_FORMAT (0x0C)

// The display module returns the Display Signal Mode.
#define SSD1963_CMD_GET_DISPLAY_MODE (0x0D)

// Get the current display mode from the peripheral
#define SSD1963_CMD_GET_TEAR_EFFECT_STATUS (0x0E)

// Turn off the panel. This command will pull low the GPIO0.
// If GPIO0 is configured as normal GPIO or LCD
// miscellaneous signal with command set_gpio_conf, this
// command will be ignored.
#define SSD1963_CMD_ENTER_SLEEP_MODE (0x10)

// Turn on the panel. This command will pull high the GPIO0.
// If GPIO0 is configured as normal GPIO or LCD
// miscellaneous signal with command set_gpio_conf, this
// command will be ignored.
#define SSD1963_CMD_EXIT_SLEEP_MODE (0x11)

// Part of the display area is used for image display.
#define SSD1963_CMD_ENTER_PARTIAL_MODE (0x12)

// The whole display area is used for image display.
#define SSD1963_CMD_ENTER_NORMAL_MODE (0x13)

// Displayed image colors are not inverted.
#define SSD1963_CMD_EXIT_INVERT_MODE (0x20)

// Displayed image colors are inverted.
#define SSD1963_CMD_ENTER_INVERT_MODE (0x21)

// Selects the gamma curve used by the display device.
#define SSD1963_CMD_SET_GAMMA_CURVE (0x26)

// Blanks the display device
#define SSD1963_CMD_SET_DISPLAY_OFF (0x28)

// Show the image on the display device
#define SSD1963_CMD_SET_DISPLAY_ON (0x29)

// Set the column extent
#define SSD1963_CMD_SET_COLUMN_ADDRESS (0x2A)

// Set the page extent
#define SSD1963_CMD_SET_PAGE_ADDRESS (0x2B)

// Transfer image information from the host processor interface
// to the peripheral starting at the location provided by
// set_column_address and set_page_address
#define SSD1963_CMD_WRITE_MEMORY_START (0x2C)

// Transfer image data from the peripheral to the host processor
// interface starting at the location provided by
// set_column_address and set_page_address
#define SSD1963_CMD_READ_MEMORY_START (0x2E)

// Defines the partial display area on the display device
#define SSD1963_CMD_SET_PARTIAL_AREA (0x30)

// Defines the vertical scrolling and fixed area on display area
#define SSD1963_CMD_SET_SCROLL_AREA (0x33)

// Synchronization information is not sent from the display
// module to the host processor
#define SSD1963_CMD_SET_TEAR_OFF (0x34)

// Synchronization information is sent from the display module
// to the host processor at the start of VFP
#define SSD1963_CMD_SET_TEAR_ON (0x35)

// Set the read order from frame buffer to the display panel
#define SSD1963_CMD_SET_ADDRESS_MODE (0x36)

// Defines the vertical scrolling starting point
#define SSD1963_CMD_SET_SCROLL_START (0x37)

// Full color depth is used for the display panel
#define SSD1963_CMD_EXIT_IDLE_MODE (0x38)

// Reduce color depth is used on the display panel.
#define SSD1963_CMD_ENTER_IDLE_MODE (0x39)

// Defines how many bits per pixel are used in the interface
// Reserved
 #define SSD1963_CMD_SET_PIXEL_FORMAT (0x3A)

// Transfer image information from the host processor interface
// to the peripheral from the last written location
#define SSD1963_CMD_WRITE_MEMORY_CONTINUE (0x3C)

// Read image data from the peripheral continuing after the last
// read_memory_continue or read_memory_start
#define SSD1963_CMD_READ_MEMORY_CONTINUE (0x3E)

// Synchronization information is sent from the display module
// to the host processor when the display device refresh reaches
// the provided scanline
#define SSD1963_CMD_SET_TEAR_SCANLINE (0x44)

// Get the current scan line
#define SSD1963_CMD_GET_SCANLINE (0x45)

// Read the DDB from the provided location
#define SSD1963_CMD_READ_DDB (0xA1)

// Set the LCD panel mode (RGB TFT or TTL)
#define SSD1963_CMD_SET_LCD_MODE (0xB0)

// Get the current LCD panel mode, pad strength and resolution
#define SSD1963_CMD_GET_LCD_MODE (0xB1)

// Set front porch
#define SSD1963_CMD_SET_HORIZ_PERIOD (0xB4)

// Get current front porch settings
#define SSD1963_CMD_GET_HORIZ_PERIOD (0xB5)

// Set the vertical blanking interval between last scan line and
// next LFRAME pulse
#define SSD1963_CMD_SET_VERT_PERIOD (0xB6)

// Set the vertical blanking interval between last scan line and
// next LFRAME pulse
#define SSD1963_CMD_GET_VERT_PERIOD (0xB7)

// Set the GPIO configuration. If the GPIO is not used for LCD,
// set the direction. Otherwise, they are toggled with LCD
// signals.
#define SSD1963_CMD_SET_GPIO_CONF (0xB8)

// Get the current GPIO configuration
#define SSD1963_CMD_GET_GPIO_CONF (0xB9)

// Set GPIO value for GPIO configured as output
#define SSD1963_CMD_SET_GPIO_VALUE (0xBA)

// Read current GPIO status. If the individual GPIO was
// configured as input, the value is the status of the
// corresponding pin. Otherwise, it is the programmed value.
#define SSD1963_CMD_GET_GPIO_STATUS (0xBB)

// Set the image post processor
#define SSD1963_CMD_SET_POST_PROC (0xBC)

// Get the image post processor
#define SSD1963_CMD_GET_POST_PROC (0xBD)

// Set PWM configuration
#define SSD1963_CMD_SET_PWM_CONF (0xBE)

// Get PWM configuration
#define SSD1963_CMD_GET_PWM_CONF (0xBF)

// Set the rise, fall, period and toggling properties of LCD signal
// generator 0
#define SSD1963_CMD_SET_LCD_GEN0 (0xC0)

// Get the current settings of LCD signal generator 0
#define SSD1963_CMD_GET_LCD_GEN0 (0xC1)

// Set the rise, fall, period and toggling properties of LCD signal
// generator 1
#define SSD1963_CMD_SET_LCD_GEN1 (0xC2)

// Get the current settings of LCD signal generator 1
#define SSD1963_CMD_GET_LCD_GEN1 (0xC3)

// Set the rise, fall, period and toggling properties of LCD signal
// generator 2
#define SSD1963_CMD_SET_LCD_GEN2 (0xC4)

// Get the current settings of LCD signal generator 2
#define SSD1963_CMD_GET_LCD_GEN2 (0xC5)

// Set the rise, fall, period and toggling properties of LCD signal
// generator 3
#define SSD1963_CMD_SET_LCD_GEN3 (0xC6)

// Get the current settings of LCD signal generator 3
#define SSD1963_CMD_GET_LCD_GEN3 (0xC7)

// Set the GPIO0 with respect to the LCD signal generators
// using ROP3 operation. No effect if the GPIO0 is configured
// as general GPIO.
#define SSD1963_CMD_SET_GPIO0_ROP (0xC8)

// Get the GPIO0 properties with respect to the LCD signal
// generators.
#define SSD1963_CMD_GET_GPIO0_ROP (0xC9)

// Set the GPIO1 with respect to the LCD signal generators
// using ROP3 operation. No effect if the GPIO1 is configured
// as general GPIO.
#define SSD1963_CMD_SET_GPIO1_ROP (0xCA)

// Get the GPIO1 properties with respect to the LCD signal
// generators.
#define SSD1963_CMD_GET_GPIO1_ROP (0xCB)

// Set the GPIO2 with respect to the LCD signal generators
// using ROP3 operation. No effect if the GPIO2 is configured
// as general GPIO.
#define SSD1963_CMD_SET_GPIO2_ROP (0xCC)

// Get the GPIO2 properties with respect to the LCD signal
// generators.
#define SSD1963_CMD_GET_GPIO2_ROP (0xCD)

// Set the GPIO3 with respect to the LCD signal generators
// using ROP3 operation. No effect if the GPIO3 is configured
// as general GPIO.
#define SSD1963_CMD_SET_GPIO3_ROP (0xCE)

// Get the GPIO3 properties with respect to the LCD signal
// generators.
#define SSD1963_CMD_GET_GPIO3_ROP (0xCF)

// Set the dynamic back light configuration
#define SSD1963_CMD_SET_DBC_COBF (0xD0)

// Get the current dynamic back light configuration
#define SSD1963_CMD_GET_DBC_CONF (0xD1)

// Set the threshold for each level of power saving
#define SSD1963_CMD_SET_DBC_TH (0xD4)

// Get the threshold for each level of power saving
#define SSD1963_CMD_GET_DBC_TH (0xD5)

// Start the PLL. Before the start, the system was operated with
// the crystal oscillator or clock input
#define SSD1963_CMD_SET_PLL (0xE0)

// Set the PLL
#define SSD1963_CMD_SET_PLL_MN (0xE2)

// Get the PLL settings
#define SSD1963_CMD_GET_PLL_MN (0xE3)

// Get the current PLL status
#define SSD1963_CMD_GET_PLL_STATUS (0xE4)

// Set deep sleep mode
#define SSD1963_CMD_SET_DEEP_SLEEP (0xE5)

// Set the LSHIFT (pixel clock) frequency
#define SSD1963_CMD_SET_LSHIFT_FREQ (0xE6)

// Get current LSHIFT (pixel clock) frequency setting
#define SSD1963_CMD_GET_LSHIFT_FREQ (0xE7)

// Set the pixel data format of the parallel host processor
// interface
#define SSD1963_CMD_SET_PIXEL_DATA_INTERFACE (0xF0)

// Get the current pixel data format settings
#define SSD1963_CMD_GET_PIXEL_DATA_INTERFACE (0xF1)
//-------------------------------------------------------------------------------------------------

#define COLOR_BLACK		0x000000	//(0,0,0)
#define COLOR_WHITE		0xFFFFFF	//(255,255,255)
#define COLOR_RED		0x0000FF	//(255,0,0)
#define COLOR_LIME		0x00FF00	//(0,255,0)
#define COLOR_BLUE		0xFF0000	//(0,0,255)
#define COLOR_YELLOW	0x00FFFF	//(255,255,0)
#define COLOR_AQUA		0xFFFF00	//(0,255,255)
#define COLOR_MAGENTA	0xFF00FF	//(255,0,255)
#define COLOR_SILVER	0xC0C0C0	//(192,192,192)
#define COLOR_GRAY		0x808080	//(128,128,128)
#define COLOR_MAROON	0x000080	//(128,0,0)
#define COLOR_OLIVE		0x008080 	//(128,128,0)
#define COLOR_GREEN		0x008000	//(0,128,0)
#define COLOR_PURPLE	0x800080	//(128,0,128)
#define COLOR_TEAL		0x808000	//(0,128,128)
#define COLOR_NAVY 		0x800000	//(0,0,128)

uint8_t IsLCDInited(void);


uint8_t ssd1963_read_data(void);
uint8_t ssd1963_read_command(void);
void ssd1963_write_command(uint8_t commandToWrite);
void ssd1963_write_data(uint8_t dataToWrite);
void ssd1963_color_pixel(uint32_t color);

void ssd1963_controller_init(void);
int ssd1963_low_test(void);
void ssd1963_window_set_480_272 (void);

void ssd1963_set_x(uint16_t start_x,uint16_t end_x);
void ssd1963_set_y(uint16_t start_y,uint16_t end_y);
void ssd1963_set_xy(uint16_t x, uint16_t y);
void ssd1963_set_work_area(uint16_t x, uint16_t y, uint16_t length, uint16_t width);

void ssd1963_draw_bitmap32(uint32_t size, uint32_t x1, uint32_t y1, int32_t x2, int32_t y2, uint32_t* image);

/**********************
 * GLOBAL PROTOTYPES
 **********************/

void ssd1963_init(void);
void ssd1963_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map);

#endif /* SSD1963_C_ */
