/**
 * @file ili9225.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "ili9225.h"
#include "disp_spi.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*********************
 *      DEFINES
 *********************/
#define TAG "ILI9225"

/**********************
 *      TYPEDEFS
 **********************/

/*The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct. */
typedef struct
{
	uint8_t cmd;
	uint8_t data[16];
	uint8_t databytes; // No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void ili9225_set_orientation(uint8_t orientation);

static void ili9225_send_cmd(uint8_t cmd);
static void ili9225_send_data(void *data, uint16_t length);
static void ili9225_send_color(void *data, uint16_t length);

static void setAddrWindow(uint16_t x1, uint16_t y1, uint16_t w,
						  uint16_t h);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void ili9225_init(void)
{

	const lcd_init_cmd_t ili_init_cmds[] = {
		{0x01, {0x01, 0x1C}, 2},
		{0x02, {0x01, 0x00}, 2},
		{0x03, {0x10, 0x30}, 2},
		{0x08, {0x08, 0x08}, 2},
		{0x0B, {0x11, 0x00}, 2},
		{0x0C, {0x00, 0x00}, 2},
		{0x0F, {0x14, 0x01}, 2},
		{0x15, {0x00, 0x00}, 2},
		{0x20, {0x00, 0x00}, 2},
		{0x21, {0x00, 0x00}, 0x82},
		{0x10, {0x08, 0x00}, 2},
		{0x11, {0x1f, 0x3f}, 0x82},
		{0x12, {0x01, 0x21}, 2},
		{0x13, {0x00, 0x6f}, 2},
		{0x14, {0x43, 0x49}, 2},
		{0x30, {0x0, 0x0}, 2},
		{0x31, {0x0, 0xdb}, 2},
		{0x32, {0x0, 0x0}, 2},
		{0x33, {0x0, 0x0}, 2},
		{0x34, {0x0, 0xdb}, 2},
		{0x35, {0x0, 0x0}, 2},
		{0x36, {0x0, 0xaf}, 2},
		{0x37, {0x0, 0x0}, 2},
		{0x38, {0x0, 0xdb}, 2},
		{0x39, {0x0, 0x0}, 2},
		{0x50, {0x0, 0x01}, 2},
		{0x51, {0x20, 0x0b}, 2},
		{0x52, {0x0, 0x0}, 2},
		{0x53, {0x04, 0x04}, 2},
		{0x54, {0x0c, 0x0c}, 2},
		{0x55, {0x00, 0x0C}, 2},
		{0x56, {0x01, 0x01}, 2},
		{0x57, {0x04, 0x0}, 2},
		{0x58, {0x11, 0x08}, 2},
		{0x59, {0x05, 0x0C}, 0x82},
		{0x07, {0x10, 0x17}, 2},

		{0, {0}, 0xff},
	};

#if ILI9225_BCKL == 15
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = GPIO_SEL_15;
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	gpio_config(&io_conf);
#endif

	// Initialize non-SPI GPIOs
	gpio_pad_select_gpio(ILI9225_DC);
	gpio_set_direction(ILI9225_DC, GPIO_MODE_OUTPUT);
	gpio_pad_select_gpio(ILI9225_RST);
	gpio_set_direction(ILI9225_RST, GPIO_MODE_OUTPUT);

#if ILI9225_ENABLE_BACKLIGHT_CONTROL
	gpio_pad_select_gpio(ILI9225_BCKL);
	gpio_set_direction(ILI9225_BCKL, GPIO_MODE_OUTPUT);
#endif
	// Reset the display
	gpio_set_level(ILI9225_RST, 0);
	vTaskDelay(100 / portTICK_RATE_MS);
	gpio_set_level(ILI9225_RST, 1);
	vTaskDelay(100 / portTICK_RATE_MS);

	ESP_LOGI(TAG, "Initialization.");

	// Send all the commands
	uint16_t cmd = 0;
	while (ili_init_cmds[cmd].databytes != 0xff)
	{
		ili9225_send_cmd(ili_init_cmds[cmd].cmd);
		ili9225_send_data(ili_init_cmds[cmd].data, ili_init_cmds[cmd].databytes & 0x1F);
		if (ili_init_cmds[cmd].databytes & 0x80)
		{
			vTaskDelay(100 / portTICK_RATE_MS);
		}
		cmd++;
	}

	ili9225_enable_backlight(true);

	ili9225_set_orientation(ILI9225_DISPLAY_ORIENTATION);

#if ILI9225_INVERT_COLORS == 1
///	ili9225_send_cmd(0x00);
#else
///	ili9225_send_cmd(0x00);
#endif
}

void ili9225_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
	int x1, y1, x2, y2;

	x1 = area->x1;
	x2 = area->x2;
	y1 = area->y1;
	y2 = area->y2;

	uint16_t data = 0;
	// Values passed are in current (possibly rotated) coordinate
	// system.  9225 requires hardware-native coords, so rotate inputs as needed.
	// The address counter is
	//  set to the top-left corner -- although fill operations can be
	//  done in any direction, the current screen rotation is applied
	//  because some users find it disconcerting when a fill does not
	//  occur top-to-bottom.
	int x, y, t;

	switch (ILI9225_DISPLAY_ORIENTATION) // CONFIG_LV_DISPLAY_ORIENTATION
	{
	default:
		x = x1;
		y = y1;
		break;
	case 1:
		t = y1;
		y1 = x1;
		x1 = ILI9225_TFTWIDTH - 1 - y2;
		y2 = x2;
		x2 = ILI9225_TFTWIDTH - 1 - t;
		x = x2;
		y = y1;
		break;
	case 2:
		t = x1;
		x1 = ILI9225_TFTWIDTH - 1 - x2;
		x2 = ILI9225_TFTWIDTH - 1 - t;
		t = y1;
		y1 = ILI9225_TFTHEIGHT - 1 - y2;
		y2 = ILI9225_TFTHEIGHT - 1 - t;
		x = x2;
		y = y2;
		break;
	case 3:
		t = x1;
		x1 = y1;
		y1 = ILI9225_TFTHEIGHT - 1 - x2;
		x2 = y2;
		y2 = ILI9225_TFTHEIGHT - 1 - t;
		x = x1;
		y = y2;
		break;
	}

	data = SPI_SWAP_DATA_TX(x1, 16);
	ili9225_send_cmd(0x37);
	ili9225_send_data(&data, 2);

	data = SPI_SWAP_DATA_TX(x2, 16);
	ili9225_send_cmd(0x36);
	ili9225_send_data(&data, 2);

	data = SPI_SWAP_DATA_TX(y1, 16);
	ili9225_send_cmd(0x39);
	ili9225_send_data(&data, 2);

	data = SPI_SWAP_DATA_TX(y2, 16);
	ili9225_send_cmd(0x38);
	ili9225_send_data(&data, 2);

	data = SPI_SWAP_DATA_TX(x, 16);
	ili9225_send_cmd(0x20);
	ili9225_send_data(&data, 2);

	data = SPI_SWAP_DATA_TX(y, 16);
	ili9225_send_cmd(0x21);
	ili9225_send_data(&data, 2);

	ili9225_send_cmd(0x22);

	uint32_t size = lv_area_get_width(area) * lv_area_get_height(area);

	ili9225_send_color((void *)color_map, size * 2);
}

void ili9225_enable_backlight(bool backlight)
{
#if ILI9225_ENABLE_BACKLIGHT_CONTROL
	ESP_LOGI(TAG, "%s backlight.", backlight ? "Enabling" : "Disabling");
	uint32_t tmp = 0;

#if (ILI9225_BCKL_ACTIVE_LVL == 1)
	tmp = backlight ? 1 : 0;
#else
	tmp = backlight ? 0 : 1;
#endif

	gpio_set_level(ILI9225_BCKL, tmp);
#endif
}

void ili9225_sleep_in()
{
	return;
	uint8_t data[] = {0x08};
	ili9225_send_cmd(0x10);
	ili9225_send_data(&data, 1);
}

void ili9225_sleep_out()
{
	return;
	uint8_t data[] = {0x08};
	ili9225_send_cmd(0x11);
	ili9225_send_data(&data, 1);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void ili9225_send_cmd(uint8_t cmd)
{
	disp_wait_for_pending_transactions();
	gpio_set_level(ILI9225_DC, 0); /*Command mode*/
	disp_spi_send_data(&cmd, 1);
}

static void ili9225_send_data(void *data, uint16_t length)
{
	disp_wait_for_pending_transactions();
	gpio_set_level(ILI9225_DC, 1); /*Data mode*/
	disp_spi_send_data(data, length);
}

static void ili9225_send_color(void *data, uint16_t length)
{
	//    disp_wait_for_pending_transactions();
	gpio_set_level(ILI9225_DC, 1); /*Data mode*/
	disp_spi_send_colors(data, length);
	disp_wait_for_pending_transactions();
}

static void ili9225_set_orientation(uint8_t orientation)
{

	const char *orientation_str[] = {
		"PORTRAIT", "PORTRAIT_INVERTED", "LANDSCAPE", "LANDSCAPE_INVERTED"};
	uint8_t data[] = {0x30, 0x28, 0x00, 0x18};
	ESP_LOGI(TAG, "Display orientation: %s, 0x03 command value: 0x%02X", orientation_str[orientation], data[orientation]);

	uint8_t data2[] = {0x10};
	ili9225_send_cmd(0x03);
	ili9225_send_data(&data2, 1);
	ili9225_send_data((void *)&data[orientation], 1);
}

void drawPixel(int16_t x, int16_t y, uint16_t color)
{
	// Clip first...
	//  if ((x >= 0) && (x < _width) && (y >= 0) && (y < _height)) {
	// THEN set up transaction (if needed) and draw...
	/*    startWrite();
		setAddrWindow(x, y, 1, 1);
		SPI_WRITE16(color);
		endWrite();
	*/

	setAddrWindow(x, y, 1, 1);
	ESP_LOGI(TAG, "WADDR set");
	ili9225_send_data(&color, 2);
	ESP_LOGI(TAG, "COLR set");
	//  }
}

static void setAddrWindow(uint16_t x1, uint16_t y1, uint16_t w,
						  uint16_t h)
{
	uint16_t x2 = (x1 + w - 1), y2 = (y1 + h - 1);
	uint16_t data = 0;
	// Values passed are in current (possibly rotated) coordinate
	// system.  9225 requires hardware-native coords, so rotate inputs as needed.
	// The address counter is
	//  set to the top-left corner -- although fill operations can be
	//  done in any direction, the current screen rotation is applied
	//  because some users find it disconcerting when a fill does not
	//  occur top-to-bottom.
	int x, y, t;

	int rotation = 1;
	switch (rotation)
	{
	default:
		x = x1;
		y = y1;
		break;
	case 1:
		t = y1;
		y1 = x1;
		x1 = ILI9225_TFTWIDTH - 1 - y2;
		y2 = x2;
		x2 = ILI9225_TFTWIDTH - 1 - t;
		x = x2;
		y = y1;
		break;
	case 2:
		t = x1;
		x1 = ILI9225_TFTWIDTH - 1 - x2;
		x2 = ILI9225_TFTWIDTH - 1 - t;
		t = y1;
		y1 = ILI9225_TFTHEIGHT - 1 - y2;
		y2 = ILI9225_TFTHEIGHT - 1 - t;
		x = x2;
		y = y2;
		break;
	case 3:
		t = x1;
		x1 = y1;
		y1 = ILI9225_TFTHEIGHT - 1 - x2;
		x2 = y2;
		y2 = ILI9225_TFTHEIGHT - 1 - t;
		x = x1;
		y = y2;
		break;
	}

	data = SPI_SWAP_DATA_TX(x1, 16);
	ili9225_send_cmd(0x37);
	ili9225_send_data(&data, 2);

	data = SPI_SWAP_DATA_TX(x2, 16);
	ili9225_send_cmd(0x36);
	ili9225_send_data(&data, 2);

	data = SPI_SWAP_DATA_TX(y1, 16);
	ili9225_send_cmd(0x39);
	ili9225_send_data(&data, 2);

	data = SPI_SWAP_DATA_TX(y2, 16);
	ili9225_send_cmd(0x38);
	ili9225_send_data(&data, 2);

	data = SPI_SWAP_DATA_TX(x, 16);
	ili9225_send_cmd(0x20);
	ili9225_send_data(&data, 2);

	data = SPI_SWAP_DATA_TX(y, 16);
	ili9225_send_cmd(0x21);
	ili9225_send_data(&data, 2);

	ili9225_send_cmd(0x22);
}
