/*
 * LCD.h
 *
 *  Created on: 8 oct. 2021
 *      Author: JoseCarlos
 */

#ifndef MAIN_LCD_H_
#define MAIN_LCD_H_
/*INCLUDES*******************************************************************************/
#include "stdint.h"
#include "string.h"
#include "stdio.h"
#include "math.h"

#include "driver/spi_master.h"

/*DEFINES********************************************************************************/
#define LCD_SPI_HOST			SPI2_HOST
#define DMA_CH					2

#define SPI_CS_PIN				22
#ifndef SPI_BUS_PORT
#define SPI_BUS_PORT
#define SPI_MOSI_PIN			23
#define SPI_MISO_PIN			19
#define SPI_CLK_PIN				18
#endif
#define DC_PIN					17
#define RST_PIN					16

#define BKL_TIMER				LEDC_TIMER_1
#define BKL_TIMER_CH			LEDC_CHANNEL_2
#define BKL_PIN					4

#define LCD_WIDTH				160
#define LCD_HEIGHT				80

#define SOFT_RST				0x01
#define SLEEP_IN				0x10
#define SLEEP_OUT				0X11
#define PARTIAL_ON				0x12
#define PARTIAL_OFF				0x13
#define INVERSION_ON			0X20
#define INVERSION_OFF			0X21
#define DISPLAY_OFF				0X28
#define DISPLAY_ON				0X29

#define WHITE					0xFFFF
#define BLACK					0x0000
#define BLUE 					0x001F
#define BRED 					0XF81F
#define GRED 					0XFFE0
#define GBLUE					0X07FF
#define RED  					0xF800
#define MAGENTA					0xF81F
#define GREEN					0x07E0
#define CYAN 					0x7FFF
#define YELLOW					0xFFE0
#define BROWN					0XBC40
#define BRRED					0XFC07
#define GRAY 					0X8430
#define DARKBLUE				0X01CF
#define LIGHTBLUE				0X7D7C
#define GRAYBLUE     	    	0X5458
#define LIGHTGREEN    			0X841F
#define LGRAY 			  		0XC618
#define LGRAYBLUE     			0XA651
#define LBBLUE        			0X2B12



/*PROTOTYPES*****************************************************************************/
esp_err_t spi_init_device(spi_device_handle_t *spi, bool init_spi);
void lcd_init(spi_device_handle_t spi);
void lcd_init_backlight();
void lcd_set_backligth(uint8_t duty_cycle);
void lcd_reset();
void lcd_write_command(spi_device_handle_t spi, uint8_t cmd);
void lcd_write_data(spi_device_handle_t spi, uint8_t *data, int len);
void lcd_write_data_byte(spi_device_handle_t spi, uint8_t data);
void lcd_spi_pre_transfer_callback(spi_transaction_t *t);
void lcd_set_cursor(spi_device_handle_t spi, uint8_t x_start, uint8_t y_start,
		uint8_t x_end, uint8_t  y_end);
void lcd_clear(spi_device_handle_t spi, uint16_t color);
void lcd_set_pixel(spi_device_handle_t spi, uint8_t x, uint8_t y, uint16_t color);
void lcd_draw_image(spi_device_handle_t spi, const uint16_t *image,
		uint8_t x_pos, uint8_t y_pos,
		uint8_t x_size, uint8_t y_size);
uint16_t convert_from_rgb_8(uint8_t r, uint8_t g, uint8_t b);
void lcd_draw_batery_widget(spi_device_handle_t spi, uint8_t x, uint8_t y, uint8_t level);
void lcd_draw_wifi_signal_widget(spi_device_handle_t spi, uint8_t x, uint8_t y, int8_t level);
void lcd_draw_bluetooth_widget(spi_device_handle_t spi, uint8_t x, uint8_t y, bool connect);
uint8_t lcd_get_constrast();
void lcd_draw_circle(spi_device_handle_t spi, uint8_t x, uint8_t y,
		uint8_t rad, uint16_t color);
void lcd_draw_circular_load_widget(spi_device_handle_t spi, uint8_t x, uint8_t y,
		uint8_t rad, uint8_t percent, uint16_t color, uint16_t background_color);
void lcd_set_window_color(spi_device_handle_t spi, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t color);
void lcd_draw_rect(spi_device_handle_t spi, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint16_t color);
void lcd_draw_line(spi_device_handle_t spi, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1,
		uint16_t color);

#endif /* MAIN_LCD_H_ */
