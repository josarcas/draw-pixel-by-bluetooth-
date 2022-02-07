/*
 * LCD.c
 *
 *  Created on: 8 oct. 2021
 *      Author: JoseCarlos
 */

/*INCLUDES******************************************************************************/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "LCD.h"


/*VARIABLES*****************************************************************************/
xSemaphoreHandle lcd_mutex;


/*	@brief	Initializate SPI device by CS pin.
 *
 * 	@param		 spi		Pointer to SPI bus.
 * 	@param	init_spi		If true init bus SPI.
 *
 * 	@return	State of operation.
 *
 */
esp_err_t spi_init_device(spi_device_handle_t *spi, bool init_spi)
{
    esp_err_t status;
    //spi_device_handle_t spi;

    if(init_spi)
    {
		spi_bus_config_t buscfg={
			.miso_io_num=SPI_MISO_PIN,
			.mosi_io_num=SPI_MOSI_PIN,
			.sclk_io_num=SPI_CLK_PIN,
			.quadwp_io_num=-1,
			.quadhd_io_num=-1,
			//.max_transfer_sz=10000
		};

	    status = spi_bus_initialize(LCD_SPI_HOST, &buscfg, DMA_CH);

	    if(status != ESP_OK)
	    	return status;
    }

    spi_device_interface_config_t devcfg={
        .clock_speed_hz=20000000,
        .mode=0,
        .spics_io_num=SPI_CS_PIN,
        .queue_size=7,
        .pre_cb=lcd_spi_pre_transfer_callback,
    };

    status = spi_bus_add_device(LCD_SPI_HOST, &devcfg, spi);

    if(status != ESP_OK)
    	return status;

    return status;
}

/*	@brief	Initializate driver of LCD ST775.
 *
 * 	@param	spi		Pointer to SPI bus.
 *
 */

void lcd_init(spi_device_handle_t spi)
{
	gpio_set_direction(DC_PIN, GPIO_MODE_OUTPUT);
	gpio_set_direction(RST_PIN, GPIO_MODE_OUTPUT);

	lcd_reset();
	lcd_mutex = xSemaphoreCreateMutex();

	if(xSemaphoreTake(lcd_mutex, portMAX_DELAY) == pdTRUE)
	{

		//lcd_init_backlight();
		lcd_write_command(spi, 0x11);
		vTaskDelay(100 / portTICK_RATE_MS);
		lcd_write_command(spi, 0x21);
		lcd_write_command(spi, 0x21);

		lcd_write_command(spi, 0xB1);
		lcd_write_data_byte(spi, 0x05);
		lcd_write_data_byte(spi, 0x3A);
		lcd_write_data_byte(spi, 0x3A);

		lcd_write_command(spi, 0xB2);
		lcd_write_data_byte(spi, 0x05);
		lcd_write_data_byte(spi, 0x3A);
		lcd_write_data_byte(spi, 0x3A);

		lcd_write_command(spi, 0xB3);
		lcd_write_data_byte(spi, 0x05);
		lcd_write_data_byte(spi, 0x3A);
		lcd_write_data_byte(spi, 0x3A);
		lcd_write_data_byte(spi, 0x05);
		lcd_write_data_byte(spi, 0x3A);
		lcd_write_data_byte(spi, 0x3A);

		lcd_write_command(spi, 0xB4);
		lcd_write_data_byte(spi, 0x03);

		lcd_write_command(spi, 0xC0);
		lcd_write_data_byte(spi, 0x62);
		lcd_write_data_byte(spi, 0x02);
		lcd_write_data_byte(spi, 0x04);

		lcd_write_command(spi, 0xC1);
		lcd_write_data_byte(spi, 0xC0);

		lcd_write_command(spi, 0xC2);
		lcd_write_data_byte(spi, 0x0D);
		lcd_write_data_byte(spi, 0x00);

		lcd_write_command(spi, 0xC3);
		lcd_write_data_byte(spi, 0x8D);
		lcd_write_data_byte(spi, 0x6A);

		lcd_write_command(spi, 0xC4);
		lcd_write_data_byte(spi, 0x8D);
		lcd_write_data_byte(spi, 0xEE);

		lcd_write_command(spi, 0xC5);
		lcd_write_data_byte(spi, 0x0E);

		lcd_write_command(spi, 0xE0);
		lcd_write_data_byte(spi, 0x10);
		lcd_write_data_byte(spi, 0x0E);
		lcd_write_data_byte(spi, 0x02);
		lcd_write_data_byte(spi, 0x03);
		lcd_write_data_byte(spi, 0x0E);
		lcd_write_data_byte(spi, 0x07);
		lcd_write_data_byte(spi, 0x02);
		lcd_write_data_byte(spi, 0x07);
		lcd_write_data_byte(spi, 0x0A);
		lcd_write_data_byte(spi, 0x12);
		lcd_write_data_byte(spi, 0x27);
		lcd_write_data_byte(spi, 0x37);
		lcd_write_data_byte(spi, 0x00);
		lcd_write_data_byte(spi, 0x0D);
		lcd_write_data_byte(spi, 0x0E);
		lcd_write_data_byte(spi, 0x10);

		lcd_write_command(spi, 0xE1);
		lcd_write_data_byte(spi, 0x10);
		lcd_write_data_byte(spi, 0x0E);
		lcd_write_data_byte(spi, 0x03);
		lcd_write_data_byte(spi, 0x03);
		lcd_write_data_byte(spi, 0x0F);
		lcd_write_data_byte(spi, 0x06);
		lcd_write_data_byte(spi, 0x02);
		lcd_write_data_byte(spi, 0x08);
		lcd_write_data_byte(spi, 0x0A);
		lcd_write_data_byte(spi, 0x13);
		lcd_write_data_byte(spi, 0x26);
		lcd_write_data_byte(spi, 0x36);
		lcd_write_data_byte(spi, 0x00);
		lcd_write_data_byte(spi, 0x0D);
		lcd_write_data_byte(spi, 0x0E);
		lcd_write_data_byte(spi, 0x10);

		lcd_write_command(spi, 0x3A);
		lcd_write_data_byte(spi, 0x05);

		lcd_write_command(spi, 0x36);
		lcd_write_data_byte(spi, 0xA8);

		lcd_write_command(spi, 0x29);

		xSemaphoreGive(lcd_mutex);
	}

}


/*	@brief	Initializate PWM for backlight.
 *
 */
void lcd_init_backlight()
{
	ledc_timer_config_t bkl_timer_config = {
	        .duty_resolution = LEDC_TIMER_8_BIT,
	        .freq_hz = 1000,
	        .speed_mode = LEDC_LOW_SPEED_MODE,
	        .timer_num = BKL_TIMER,
	        .clk_cfg = LEDC_AUTO_CLK,
	    };

	ledc_timer_config(&bkl_timer_config);

	ledc_channel_config_t bkl_control =         {
            .channel    = BKL_TIMER_CH,
            .duty       = 0,
            .gpio_num   = BKL_PIN,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = BKL_TIMER
        };

	ledc_channel_config(&bkl_control);
}


/*	@brief	Configure duty cycle of PWM for backlight.
 *
 * 	@param	duty_cycle	Value for duty cycle.
 *
 */
void lcd_set_backligth(uint8_t duty_cycle)
{
	ledc_set_duty(LEDC_LOW_SPEED_MODE, BKL_TIMER_CH, duty_cycle);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, BKL_TIMER_CH);
}


/*	@brief	LCD hard reset.
 *
 */
void lcd_reset()
{
	gpio_set_level(RST_PIN, 0);
	vTaskDelay(20);
	gpio_set_level(RST_PIN, 1);
	vTaskDelay(20);
}


/*	@brief	Write command for LCD driver.
 *
 * 	@param  spi		Pointer to SPI bus.
 * 	@param	cmd		Command for LCD.
 *
 */
void lcd_write_command(spi_device_handle_t spi, uint8_t cmd)
{
    esp_err_t status;
    spi_transaction_t spi_transaction;
    memset(&spi_transaction, 0, sizeof(spi_transaction));
    spi_transaction.length=8;
    spi_transaction.tx_buffer=&cmd;
    spi_transaction.user=(void*)0;
    status=spi_device_polling_transmit(spi, &spi_transaction);
    assert(status==ESP_OK);
}


/*	@brief	Write data for LCD driver.
 *
 * 	@param   spi	Pointer to SPI bus.
 * 	@param	data	data for write to LCD.
 * 	@param	 len	Number of data for write.
 *
 */
void lcd_write_data(spi_device_handle_t spi, uint8_t *data, int len)
{
    esp_err_t status;
    spi_transaction_t spi_transaction;
    if (len==0) return;
    memset(&spi_transaction, 0, sizeof(spi_transaction));
    spi_transaction.length=len*8;
    spi_transaction.tx_buffer=data;
    spi_transaction.user=(void*)1;
    status=spi_device_polling_transmit(spi, &spi_transaction);
    assert(status==ESP_OK);
}

/*	@brief	Write only one data for LCD driver.
 *
 * 	@param   spi		Pointer to SPI bus.
 * 	@param	data		Data for LCD.
 *
 */
void lcd_write_data_byte(spi_device_handle_t spi, uint8_t data)
{
	lcd_write_data(spi, &data, 1);
}


/*	@brief	Hanler call before init SPI transaction.
 *
 * 	@param  t	event of trasaction.
 *
 */
void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc=(int)t->user;
    gpio_set_level(DC_PIN, dc);
}

/*	@brief	Go to pixel screen.
 *
 * 	@param   spi		Pointer to SPI bus.
 * 	@param	 x_start	X pixel for begin window.
 * 	@param	 y_start	Y pixel for begin window.
 * 	@param	 x_end		X pixel for end window.
 * 	@param	 y_end		Y pixel for end window.
 *
 */
void lcd_set_cursor(spi_device_handle_t spi, uint8_t x_start, uint8_t y_start,
		uint8_t x_end, uint8_t  y_end)
{
	x_start = x_start + 1;
	x_end = x_end + 1;
	y_start = y_start + 26;
	y_end = y_end+26;
	lcd_write_command(spi, 0x2a);
	lcd_write_data_byte(spi, 0x00);
	lcd_write_data_byte(spi, x_start);
	lcd_write_data_byte(spi, 0x00);
	lcd_write_data_byte(spi, x_end );

	lcd_write_command(spi, 0x2b);
	lcd_write_data_byte(spi, 0x00);
	lcd_write_data_byte(spi, y_start);
	lcd_write_data_byte(spi, 0x00);
	lcd_write_data_byte(spi, y_end);

	lcd_write_command(spi, 0x2C);
}

/*	@brief	Clear screen with color.
 *
 * 	@param   spi		Pointer to SPI bus.
 * 	@param	 color		Color for clear screen.
 *
 */
void lcd_clear(spi_device_handle_t spi, uint16_t color)
{
  	uint8_t buffer[2]={color>>8, color};

	if(xSemaphoreTake(lcd_mutex, portMAX_DELAY) == pdTRUE)
	{

		lcd_set_cursor(spi, 0, 0, LCD_WIDTH-1, LCD_HEIGHT-1);

		for(uint8_t i = 0; i < LCD_WIDTH; i++)
		{
			for(uint8_t j = 0; j < LCD_HEIGHT; j++)
			{
				lcd_write_data(spi, buffer, 2);
			}
		 }

		xSemaphoreGive(lcd_mutex);
	}


}

/*	@brief	Set color of screen pixel.
 *
 * 	@param   spi		Pointer to SPI bus.
 * 	@param	 x 			X pixel of screen.
 * 	@param	 y			Y pixel of screen.
 * 	@param	 color		Color for set pixel.
 *
 */
void lcd_set_pixel(spi_device_handle_t spi, uint8_t x, uint8_t y, uint16_t color)
{
	uint8_t buffer[2];
	if(xSemaphoreTake(lcd_mutex, portMAX_DELAY) == pdTRUE)
	{
		lcd_set_cursor(spi, x, y, x, y);
		buffer[0] = color>>8;
		buffer[1] = color;
		lcd_write_data(spi, buffer, 2);
		xSemaphoreGive(lcd_mutex);
	}


}

/*	@brief	Set color of screen pixel.
 *
 * 	@param   spi		Pointer to SPI bus.
 * 	@param	 image		matix of image.
 * 	@param	 x_pos 		X pixel init.
 * 	@param	 y_pos		Y pixel init.
 * 	@param	 x_size		Weight of image.
 * 	@param	 y_size		Height of image.
 *
 */
void lcd_draw_image(spi_device_handle_t spi, const uint16_t *image,
		uint8_t x_pos, uint8_t y_pos,
		uint8_t x_size, uint8_t y_size)
{

	uint16_t page=0;
	uint8_t buffer[2];

	if((x_pos + x_size) > LCD_WIDTH || (y_pos+ y_size) > LCD_HEIGHT)
		return;

	if(xSemaphoreTake(lcd_mutex, portMAX_DELAY) == pdTRUE)
	{
		lcd_set_cursor(spi, x_pos, y_pos, x_pos+x_size-1, y_pos+y_size-1);

		for(uint16_t i=0; i< y_size; i++)
		{
			for(uint16_t j= 0; j<x_size; j++)
			{
				buffer[0] = image[page+j]>>8;
				buffer[1] = image[page+j];
				lcd_write_data(spi, buffer, 2);
			}
			page += x_size;
		}

		xSemaphoreGive(lcd_mutex);
	}

}

/*	@brief	Convert 888 RGB to 565 RGB.
 *
 * 	@param   r		Red color value.
 * 	@param	 g		Green color value.
 * 	@param	 b		Blue color value.
 *
 * 	@return	Value of 565 RGB.
 *
 */
uint16_t convert_from_rgb_8(uint8_t r, uint8_t g, uint8_t b)
{
	uint8_t r_5 = r>>3;
	uint8_t g_6 = g>>2;
	uint8_t b_5 = b>>3;

	return (r_5<<11) | (g_6<<5) | (b_5);
}

/*	@brief	Draw batery on screen.
 *
 * 	@param   spi		Pointer to SPI bus.
 * 	@param	 x	 		X pixel init.
 * 	@param	 y			Y pixel init.
 * 	@param	 level		Level of batery 0-10 to 0-100 percent.
 *
 */
void lcd_draw_batery_widget(spi_device_handle_t spi, uint8_t x, uint8_t y, uint8_t level)
{
	//uint8_t buffer_line[2];
	//uint8_t i,j;
	uint16_t color;
	//uint8_t page=0;

	if(level<3)
		color = 63488;
	else if(level< 6)
		color = 65504;
	else
		color = 2016;

/*
	level = 10-level;



	if(xSemaphoreTake(lcd_mutex, portMAX_DELAY) == pdTRUE)
	{
		for(i=0; i<7; i++)
		{
			lcd_set_cursor(spi, x, y, x+14, y);
			for(j=0; j<14; j++)
			{
				if(i==0 || i==6)
				{
					buffer_line[0] = batery_widget[j];
					buffer_line[1] = batery_widget[j];
				}
				else
				{
					if(batery_widget[page+j] == 0x01)
					{
						if(j<level)
						{
							buffer_line[0]= 0;
							buffer_line[1]= 0;
						}
						else
						{
							buffer_line[0]= color>>8;
							buffer_line[1]= color;
						}
					}
					else
					{
						buffer_line[0] = batery_widget[page+j];
						buffer_line[1] = batery_widget[page+j];
					}
				}
				lcd_write_data(spi, buffer_line, 2);
			}
			y++;
			page+=14;
		}

		xSemaphoreGive(lcd_mutex);
	}
	*/

	lcd_set_window_color(spi, x, y+2, x+2, y+5, WHITE);
	lcd_draw_rect(spi, x+2, y, 12, 7, WHITE);
	level = 10-level;
	x+=3;
	lcd_set_window_color(spi, x, y+1, x+level, y+6, BLACK);
	lcd_set_window_color(spi, x+level, y+1, x+10, y+6, color);

}

/*	@brief	Get brightness of screen.
 *
 * 	@param   return		Level of brightness (duty cycle).
 *
 */
uint8_t lcd_get_constrast()
{
	return ledc_get_duty(LEDC_LOW_SPEED_MODE, BKL_TIMER_CH);
}

/*	@brief	Draw circle on screen.
 *
 * 	@param   spi		Pointer to SPI bus.
 * 	@param	 x	 		X pixel init.
 * 	@param	 y			Y pixel init.
 * 	@param	 rad		Radious of circle in pixels.
 * 	@param   color		Color of circle.
 *
 */
void lcd_draw_circle(spi_device_handle_t spi, uint8_t x, uint8_t y,
		uint8_t rad, uint16_t color)
{
	int f = 1-rad;
	uint8_t ddf_x = 1;
	uint8_t ddf_y = -2*rad;
	uint8_t x_pos = 0;
	uint8_t y_pos = rad;

	lcd_set_pixel(spi, x, y+rad, color);
	lcd_set_pixel(spi, x, y-rad, color);
	lcd_set_pixel(spi, x-rad, y, color);
	lcd_set_pixel(spi, x+rad, y, color);

	while(x_pos<y_pos)
	{
		if(f >= 0)
		{
			y_pos--;
			ddf_y += 2;
			f += ddf_y;
		}

		x_pos++;
		ddf_x += 2;
		f += ddf_x;

		lcd_set_pixel(spi, x+x_pos, y+y_pos, color);
		lcd_set_pixel(spi, x-x_pos, y+y_pos, color);
		lcd_set_pixel(spi, x+x_pos, y-y_pos, color);
		lcd_set_pixel(spi, x-x_pos, y-y_pos, color);
		lcd_set_pixel(spi, x+y_pos, y+x_pos, color);
		lcd_set_pixel(spi, x-y_pos, y+x_pos, color);
		lcd_set_pixel(spi, x+y_pos, y-x_pos, color);
		lcd_set_pixel(spi, x-y_pos, y-x_pos, color);
	}
}

/*	@brief	Load animation screen.
 *
 * 	@param   spi					Pointer to SPI bus.
 * 	@param	 x	 					X pixel init.
 * 	@param	 y						Y pixel init.
 * 	@param	 rad					Radious of circle in pixels.
 * 	@param   percent				Percent of load 0-120.
 * 	@param	 color					Color of load animation.
 * 	@param	 background_color		Color of no load animation.
 *
 */
void lcd_draw_circular_load_widget(spi_device_handle_t spi, uint8_t x, uint8_t y,
		uint8_t rad, uint8_t percent, uint16_t color, uint16_t background_color)
{
    float theta = 0;
    uint8_t x_pos = rad;
    uint8_t y_pos = 0;
    float xd;
    float yd;
    float increment = (float)3*M_PI/(float)180;

    for(uint8_t i=0; i<120; i++)
    {
    	if(i<percent)
    		lcd_set_pixel(spi, x + x_pos, y + y_pos, color);
    	else
    		lcd_set_pixel(spi, x + x_pos, y + y_pos, background_color);

        theta = theta + increment;
        xd = rad * cos(theta);
        x_pos = round(xd);
        yd = rad * sin(theta);
        y_pos = yd;
    }
}

/*	@brief	Set color of window.
 *
 * 	@param   spi		Pointer to SPI bus.
 * 	@param	 x0 		X pixel to begin.
 * 	@param	 y0			Y pixel to begin.
 * 	@param	 x1			X pixel of end.
 * 	@param	 y1			Y pixel of end.
 * 	@param	 color		color for set window.
 *
 */
void lcd_set_window_color(spi_device_handle_t spi, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t color)
{
  	uint8_t buffer[2]={color>>8, color};

	if(xSemaphoreTake(lcd_mutex, portMAX_DELAY) == pdTRUE)
	{

		lcd_set_cursor(spi, x0, y0, x1-1, y1-1);

		for(uint8_t i = x0; i < x1; i++)
		{
			for(uint8_t j = y0; j < y1; j++)
			{
				lcd_write_data(spi, buffer, 2);
			}
		 }

		xSemaphoreGive(lcd_mutex);
	}
}

/*	@brief	Draw rect of screen.
 *
 * 	@param   spi		Pointer to SPI bus.
 * 	@param	 x 			X pixel to begin.
 * 	@param	 y			Y pixel to begin.
 * 	@param	 width		Width of rect.
 * 	@param	 height		Height of rect.
 * 	@param	 color		Color for rect.
 *
 */
void lcd_draw_rect(spi_device_handle_t spi, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint16_t color)
{
  	uint8_t buffer[2]={color>>8, color};
  	uint8_t i;

  	height--;
  	width--;

	if(xSemaphoreTake(lcd_mutex, portMAX_DELAY) == pdTRUE)
	{
		lcd_set_cursor(spi, x, y, x, y+height);
		for(i=0; i<=height; i++)
			lcd_write_data(spi, buffer, 2);

		lcd_set_cursor(spi, x+width, y, x+width, y+height);
		for(i=0; i<=height; i++)
			lcd_write_data(spi, buffer, 2);

		lcd_set_cursor(spi, x, y, x+width, y);
		for(i=0; i<=width; i++)
			lcd_write_data(spi, buffer, 2);

		lcd_set_cursor(spi, x, y+height, x+width, y+height);
		for(i=0; i<=width; i++)
			lcd_write_data(spi, buffer, 2);

		xSemaphoreGive(lcd_mutex);
	}
}

/*	@brief	Draw line on screen.
 *
 * 	@param   spi		Pointer to SPI bus.
* 	@param	 x0 		X pixel to begin.
 * 	@param	 y0			Y pixel to begin.
 * 	@param	 x1			X pixel of end.
 * 	@param	 y1			Y pixel of end.
 * 	@param	 color		Color of line.
 *
 */
void lcd_draw_line(spi_device_handle_t spi, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1,
		uint16_t color)
{
	int slope;
	uint8_t dx, incE, incNE, x, y;
	int dy, d;

	if(x0 > x1)
	{
		lcd_draw_line(spi, x1, y1, x0, y0, color);
		return;
	}

	dx = x1 - x0;
	dy = y1 - y0;

	if(dy < 0)
	{
		slope = -1;
		dy = -dy;
	}

	else
		slope = 1;

	incE = 2 * dy;
	incNE = 2 * dy - 2 * dx;
	d = 2 * dy - dx;
	y = y0;

	for(x=x0; x<=x1; x++)
	{
		lcd_set_pixel(spi, x, y, color);

		if(d <= 0)
			d += incE;

		else
		{
			d += incNE;
			y += slope;
		}
	}
}
