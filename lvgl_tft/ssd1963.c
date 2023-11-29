/*
 */


/*********************
 *      INCLUDES
 *********************/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "hal/gpio_hal.h"
#include "driver/gpio.h"
#include "ssd1963.h"

/*********************
 *      DEFINES
 *********************/
 #define TAG "SSD1963"

/**********************
 *      TYPEDEFS
 **********************/


gpio_dev_t *dev= GPIO_HAL_GET_HW(GPIO_PORT_0);

//-------------------------------------------------------------------------------------------------
// Write command
//-------------------------------------------------------------------------------------------------
inline void ssd1963_write_command(uint8_t commandToWrite)
{
//    LCD_CMD = commandToWrite;
/* Write command sequence:
  LCD_DATA_PORT->ODR  = commandToWrite; ESP32_VS bits 
  RESET_LCD_RDS;
  RESET_LCD_WR;
  SET_LCD_WR;
*/
//dev->enable_w1ts = 0x80FF020; //Set display i\o direction output
//uint32_t cmd=commandToWrite;

//dev->out_w1ts = (cmd<< 12); 
dev->out_w1tc = (0x8000000); //DC PIN27

//cmd=((uint32_t)commandToWrite << 12);
//printf("CMD:%u\n",cmd );
dev->out_w1tc = (0xFF020); //WR PIN05 && clear DATA
dev->out_w1ts = ((uint32_t) commandToWrite<< 12); //DATA PIN12 - 19  50ns
dev->out_w1ts = (0x20); //WR PIN05 

dev->out_w1ts = (0x8000000); //DC PIN27

}
//-------------------------------------------------------------------------------------------------
// Write data
//-------------------------------------------------------------------------------------------------
inline void ssd1963_write_data(uint8_t dataToWrite)
{
//    LCD_DATA = dataToWrite;
/*Write data sequence
  LCD_DATA_PORT->ODR  = dataToWrite;
  SET_LCD_RDS;
  RESET_LCD_WR;
  SET_LCD_WR;
*/
//uint32_t cmd=dataToWrite;

dev->out_w1tc = (0xFF020); //WR PIN05 && clear DATA
dev->out_w1ts = ((uint32_t) dataToWrite<< 12); //DATA PIN12 - 19  50ns
dev->out_w1ts = (0x20); //WR PIN05 


}

inline void ssd1963_color_pixel(uint32_t color)
{
	/*
    ssd1963_write_data((color>>0)&0xFF);
	ssd1963_write_data( (color>>8)&0xFF );
	ssd1963_write_data( (color>>16)&0xFF );
*/

dev->out_w1tc = (0xFF020); //WR PIN05 && clear DATA
dev->out_w1ts = ((color<< 12)&0xFF000); //DATA PIN12 - 19  50ns
dev->out_w1ts = (0x20); //WR PIN05 

dev->out_w1tc = (0xFF020); //WR PIN05 && clear DATA
dev->out_w1ts = ((color<< 4)&0xFF000); //DATA PIN12 - 19  50ns
dev->out_w1ts = (0x20); //WR PIN05 


dev->out_w1tc = (0xFF020); //WR PIN05 && clear DATA
dev->out_w1ts = ((color>> 4)&0xFF000); //DATA PIN12 - 19  50ns
dev->out_w1ts = (0x20); //WR PIN05 

}

void ssd1963_init(void)
{
	gpio_config_t gpio_conf;
	gpio_conf.mode = GPIO_MODE_OUTPUT;
	gpio_conf.pull_up_en =	GPIO_PULLUP_DISABLE;
	gpio_conf.pull_down_en = GPIO_PULLUP_DISABLE;
	gpio_conf.intr_type = GPIO_INTR_DISABLE;   
	gpio_conf.pin_bit_mask = 0x80FF020; //!!! SET CONFIG_ DC & WR
	ESP_ERROR_CHECK(gpio_config(&gpio_conf));

dev->out_w1ts = (0x8000000); //DC PIN27 !!! SET CONFIG_ DC & WR
dev->out_w1ts = (0x20); //WR PIN05  !!! SET CONFIG_ DC & WR

	ssd1963_write_command(0xe2);
	ssd1963_write_data(0x1d);
	ssd1963_write_data(0x02);
	ssd1963_write_data(0x54);
	//Command_Write(0xe0,0x01);    //START PLL
	ssd1963_write_command(0xe0);
	ssd1963_write_data(0x01);
	vTaskDelay(5);
	//Command_Write(0xe0,0x03);    //LOCK PLL
	ssd1963_write_command(0xe0);
	ssd1963_write_data(0x03);
	vTaskDelay(5);

	ssd1963_write_command(0x01);     //Software Reset
	vTaskDelay(10);

	ssd1963_write_command(0xe6);     //SET PCLK freq=4.94MHz  ; pixel clock frequency
	
	ssd1963_write_data(0x01);		//00 WINSTAR Datasheet
	ssd1963_write_data(0x99);		//ce
	ssd1963_write_data(0x9a);		//94

	ssd1963_write_command(0xb0);  //SET LCD MODE  SET TFT 18Bits MODE
	ssd1963_write_data(0x20);   //SET TFT MODE & hsync+Vsync+DEN MODE
//	ssd1963_write_data(0x08);   //SET TFT MODE & hsync+Vsync+DEN MODE
	ssd1963_write_data(0x80);   //SET TFT MODE & hsync+Vsync+DEN MODE
	ssd1963_write_data(0x01);   //SET horizontal size=320-1 HightByte
	ssd1963_write_data(0xdf);      //SET horizontal size=320-1 LowByte
	ssd1963_write_data(0x01);   //SET vertical size=240-1 HightByte
	ssd1963_write_data(0x0f);   //SET vertical size=240-1 LowByte
	ssd1963_write_data(0x00);   //SET even/odd line RGB seq.=RGB



	ssd1963_write_command(0xb4);		//SET HBP,
	ssd1963_write_data(0x02);			//SET HSYNC Tatol = 440
	ssd1963_write_data(0x0d);
	ssd1963_write_data(0x00);			//SET HBP = 68
	ssd1963_write_data(0x14);
	ssd1963_write_data(0x05);			//SET VBP 16 = 15 + 1
	ssd1963_write_data(0x00);			//SET Hsync pulse start position
	ssd1963_write_data(0x00);
	ssd1963_write_data(0x00);			//SET Hsync pulse subpixel start position

	ssd1963_write_command(0xb6); 		//SET VBP,
	ssd1963_write_data(0x01);			//SET Vsync total 265 = 264 + 1
	ssd1963_write_data(0x24);
	ssd1963_write_data(0x00);			//SET VBP = 19
	ssd1963_write_data(0x0a);
	ssd1963_write_data(0x05);			//SET Vsync pulse 8 = 7 + 1
	ssd1963_write_data(0x00);			//SET Vsync pulse start position
	ssd1963_write_data(0x00);

	ssd1963_write_command(0xb8);   //SET GPIO
	ssd1963_write_data(0x0f);      //SET I/O
	ssd1963_write_data(0x01);
	ssd1963_write_command(0xba);   //SET GPIO
    ssd1963_write_data(0x01);      //SET I/O

	//Command_Write(0x36,0x08);   // CONFIG_LV_DISPLAY_ORIENTATION
	ssd1963_write_command(0x36);
switch (CONFIG_LV_DISPLAY_ORIENTATION) {
		default:
	ssd1963_write_data(0x00);
		break;
	case 1:
	ssd1963_write_data(0x21);
		break;	
			case 2:
	ssd1963_write_data(0x17);			
		break;	
			case 3:
	ssd1963_write_data(0x22);			
		break;	
}

// 	if(S_S == 1)
//	{
		//8bit(666)
//	/Command_Write(0xf0,0x00); 	//SET pixel data I/F format=8bit
	ssd1963_write_command(0xf0);
	ssd1963_write_data(0x00);

//	}
//	else
//	{
//		//16bit(565)
//		Command_Write(0xf0,0x03);	//SET pixel data I/F format=16bit(565 format)
//	}


	ssd1963_write_command(0x29);  //SET display on
	ssd1963_write_command(0xd0);  //SD
	ssd1963_write_data(0x0d);   //SD

}

void ssd1963_window_set_480_272 (void) { //NOT USED----------------------------------------

	ssd1963_write_command(0x2A);        // SET column address
	ssd1963_write_data(0x00);    // SET start column address=0
	ssd1963_write_data(0x00);
	ssd1963_write_data(0x01);    // SET end column address=320
	ssd1963_write_data(0xDF);   //DF

	ssd1963_write_command(0x2B);        // SET page address
	ssd1963_write_data(0x00);    // SET start page address=0
	ssd1963_write_data(0x00);
	ssd1963_write_data(0x01);   //01 // SET end page address=240
	ssd1963_write_data(0x0F);   //0F
}//end;

//-------------------------------------------------------------------------------------------------

void ssd1963_set_x(uint16_t start_x,uint16_t end_x)
{
	ssd1963_write_command(SSD1963_CMD_SET_COLUMN_ADDRESS);
	ssd1963_write_data(start_x>>8);
	ssd1963_write_data(start_x&0x00ff);

	ssd1963_write_data(end_x>>8);
	ssd1963_write_data(end_x&0x00ff);
}

//ф-ция устанавливает рабочую область по Y
void ssd1963_set_y(uint16_t start_y,uint16_t end_y)
{
	ssd1963_write_command(SSD1963_CMD_SET_PAGE_ADDRESS);
	ssd1963_write_data(start_y>>8);
	ssd1963_write_data(start_y&0x00ff);

	ssd1963_write_data(end_y>>8);
	ssd1963_write_data(end_y&0x00ff);
}



/*
void ssd1963_fill_color_screen(uint32_t color)
{

	ssd1963_set_x(x, x+DISP_HOR_RESOLUTION-1);
	ssd1963_set_y(y, y+DISP_VER_RESOLUTION-1);
	ssd1963_write_command(SSD1963_CMD_WRITE_MEMORY_START);	
	for(uint32_t pix=0; pix < (DISP_HOR_RESOLUTION*DISP_VER_RESOLUTION); pix++)
	{
		//DBG_printf(DBG_LEVEL_CURRENT, "C");
		ssd1963_color_pixel(color);
	}
}
*/

void ssd1963_draw_bitmap32(uint32_t size, uint32_t x1, uint32_t y1, int32_t x2, int32_t y2, uint32_t* image)
{
	ssd1963_set_x(x1, x2);
	ssd1963_set_y(y1, y2);
	ssd1963_write_command(SSD1963_CMD_WRITE_MEMORY_START);

	for(uint32_t pix=0; pix < size; pix++)
	{
		ssd1963_color_pixel((uint32_t)image[pix]);

	}
}

 void ssd1963_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t * color_map)
{
//    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
//if rotation &0x01=1 then swapXY
    lv_coord_t start_x ;
    lv_coord_t end_x ;
    lv_coord_t start_y ;
    lv_coord_t end_y ;
if (drv->rotated == 1) {
     start_x = area->y1;
     end_x = area->y2;
     start_y = area->x1;
     end_y = area->x2;
	
} else {
     start_x = area->x1;
     end_x = area->x2;
     start_y = area->y1;
     end_y = area->y2;

}

    uint32_t size = lv_area_get_width(area) * lv_area_get_height(area);
    // copy a buffer's content to a specific area of the display
//    ssd1963_draw_bitmap32(size, offsetx1, offsety1, offsetx2 , offsety2 , color_map);

//	ssd1963_set_x(x1, x2);
	{
	ssd1963_write_command(SSD1963_CMD_SET_COLUMN_ADDRESS);
	ssd1963_write_data(start_x>>8);
	ssd1963_write_data(start_x&0x00ff);

	ssd1963_write_data(end_x>>8);
	ssd1963_write_data(end_x&0x00ff);
}
//	ssd1963_set_y(y1, y2);
	{
	ssd1963_write_command(SSD1963_CMD_SET_PAGE_ADDRESS);
	ssd1963_write_data(start_y>>8);
	ssd1963_write_data(start_y&0x00ff);

	ssd1963_write_data(end_y>>8);
	ssd1963_write_data(end_y&0x00ff);
}
	ssd1963_write_command(SSD1963_CMD_WRITE_MEMORY_START);

	for(uint32_t pix=0; pix < size; pix++)
	{
		ssd1963_color_pixel(color_map[pix].full);

	}
	
     lv_disp_flush_ready(drv);
}
