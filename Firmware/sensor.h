/*
 * sensor.h
 *
 *  Created on: Jan 25, 2020
 *      Author: gaurav singh
 *      www.circuitvalley.com
 */


/* This file defines the parameters and the interface for the MT9M114 image
   sensor driver.
 */

#ifndef _INCLUDED_SENSOR_H_
#define _INCLUDED_SENSOR_H_

#include <cyu3types.h>
#include <stdbool.h>
/* The SADDR line allows MT9M114 image sensor to select between two different I2C slave address.
   If the SADDR line is high, enable this #define to allow access to the correct I2C address for the sensor.
 */
/* #define SADDR_HIGH */

#define IMX219_SENSOR_ID                        0x0219

/* I2C Slave address for the image sensor. */
#ifdef SADDR_HIGH
#define SENSOR_ADDR_WR 0xBA             /* Slave address used to write sensor registers. */
#define SENSOR_ADDR_RD 0xBB             /* Slave address used to read from sensor registers. */
#else
#define SENSOR_ADDR_WR 0x72             /* Slave address used to write sensor registers. */
#define SENSOR_ADDR_RD 0x73             /* Slave address used to read from sensor registers. */
#endif

#define I2C_SLAVEADDR_MASK 0xFE         /* Mask to get actual I2C slave address value without direction bit. */

#define I2C_MEMORY_ADDR_WR 0xA0         /* I2C slave address used to write to an EEPROM. */
#define I2C_MEMORY_ADDR_RD 0xA1         /* I2C slave address used to read from an EEPROM. */

/* GPIO 22 on FX3 is used to reset the Image sensor. */
#define DEBUG1_GPIO 	24 //ctl7
#define DEBUG_GPIO 	27
#define SENSOR_RESET_GPIO 	21


#define _countof(array) (sizeof(array) / sizeof(array[0]))

enum
{
    IMAGE_NORMAL=0,
    IMAGE_H_MIRROR,
    IMAGE_V_MIRROR,
    IMAGE_HV_MIRROR
};

typedef enum{
	IMGSENSOR_MODE_INIT,
	IMGSENSOR_MODE_PREVIEW,
	IMGSENSOR_MODE_CAPTURE,
	IMGSENSOR_MODE_VIDEO,
	IMGSENSOR_MODE_HIGH_SPEED_VIDEO,
	IMGSENSOR_MODE_SLIM_VIDEO,
} IMGSENSOR_MODE;

/* SENSOR PRIVATE STRUCT FOR VARIABLES*/
typedef struct imgsensor_struct {
	uint8_t mirror;				//mirrorflip information

	uint8_t sensor_mode;			//record IMGSENSOR_MODE enum value

	uint32_t shutter;				//current shutter
	uint16_t gain;				//current gain

	uint32_t pclk;				//current pclk

	uint32_t frame_length;		//current framelength
	uint32_t line_length;			//current linelength

	uint32_t min_frame_length;	//current min  framelength to max framerate
	uint16_t dummy_pixel;			//current dummypixel
	uint16_t dummy_line;			//current dummline

	uint16_t current_fps;			//current max fps
	bool   autoflicker_en;		//record autoflicker enable or disable
	bool test_pattern;			//record test pattern mode or not
	uint16_t current_scenario_id;//current scenario id
	uint8_t  ihdr_en;				//ihdr enable or disable

	uint8_t i2c_write_id;			//record current sensor's i2c write id
} imgsensor_struct;


typedef struct imgsensor_mode_struct_s {
	uint32_t pclk;				//record different mode's pclk
	uint32_t linelength;			//record different mode's linelength
	uint32_t framelength;			//record different mode's framelength

	uint8_t startx;				//record different mode's startx of grabwindow
	uint8_t starty;				//record different mode's startx of grabwindow

	uint16_t grabwindow_width;	//record different mode's width of grabwindow
	uint16_t grabwindow_height;	//record different mode's height of grabwindow

	/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
	uint8_t mipi_data_lp2hs_settle_dc;

	/*	 following for GetDefaultFramerateByScenario()	*/
	uint16_t max_framerate;

} imgsensor_mode_struct_t;


/* SENSOR PRIVATE STRUCT FOR CONSTANT*/
typedef struct imgsensor_info_struct {
	uint32_t sensor_id;			//record sensor id defined in Kd_imgsensor.h
	uint32_t checksum_value;		//checksum value for Camera Auto Test
	imgsensor_mode_struct_t mode_640x480;		//640x480 30fps
	imgsensor_mode_struct_t mode_1280x720;		//1280x720 30fps
	imgsensor_mode_struct_t mode_1920x1080;		//1920x1080 30fps
	imgsensor_mode_struct_t mode_1280x720_180;	//1280x720 180fps
	imgsensor_mode_struct_t mode_1920x1080_60;	//1920x1080 60fps

	uint8_t  ae_shut_delay_frame;	//shutter delay frame for AE cycle
	uint8_t  ae_sensor_gain_delay_frame;	//sensor gain delay frame for AE cycle
	uint8_t  ae_ispGain_delay_frame;	//isp gain delay frame for AE cycle
	uint8_t  ihdr_support;		//1, support; 0,not support
	uint8_t  ihdr_le_firstline;	//1,le first ; 0, se first
	uint8_t  sensor_mode_num;		//support sensor mode num

	uint8_t  cap_delay_frame;		//enter capture delay frame num
	uint8_t  pre_delay_frame;		//enter preview delay frame num
	uint8_t  video_delay_frame;	//enter video delay frame num
	uint8_t  hs_video_delay_frame;	//enter high speed video  delay frame num
	uint8_t  slim_video_delay_frame;	//enter slim video delay frame num

	uint8_t  margin;				//sensor framelength & shutter margin
	uint32_t min_shutter;			//min shutter
	uint32_t max_frame_length;	//max framelength by sensor register's limitation

	uint8_t  isp_driving_current;	//mclk driving current
	uint8_t  sensor_interface_type;//sensor_interface_type
	uint8_t  mipi_sensor_type; //0,MIPI_OPHY_NCSI2; 1,MIPI_OPHY_CSI2, default is NCSI2, don't modify this para
	uint8_t  mipi_settle_delay_mode; //0, high speed signal auto detect; 1, use settle delay,unit is ns, default is auto detect, don't modify this para
	uint8_t  sensor_output_dataformat;//sensor output first pixel color
	uint8_t  mclk;				//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz

	uint8_t  mipi_lane_num;		//mipi lane num
	uint8_t  i2c_addr_table[5];	//record sensor support all write id addr, only supprt 4must end with 0xff
} imgsensor_info_struct;


typedef struct {
	uint16_t full_w;
	uint16_t full_h;
	uint16_t x0_offset;
	uint16_t y0_offset;
	uint16_t w0_size;
	uint16_t h0_size;
	uint16_t scale_w;
	uint16_t scale_h;
	uint16_t x1_offset;
	uint16_t y1_offset;
	uint16_t w1_size;
	uint16_t h1_size;
	uint16_t x2_tg_offset;
	uint16_t y2_tg_offset;
	uint16_t w2_tg_size;
	uint16_t h2_tg_size;
} sensor_winsize_info_struct_t;


/* Function    : SensorWrite2B
   Description : Write two bytes of data to image sensor over I2C interface.
   Parameters  :
                 slaveAddr - I2C slave address for the sensor.
                 highAddr  - High byte of memory address being written to.
                 lowAddr   - Low byte of memory address being written to.
                 highData  - High byte of data to be written.
                 lowData   - Low byte of data to be written.
 */
extern CyU3PReturnStatus_t
SensorWrite2B (
        uint8_t slaveAddr,
        uint8_t highAddr,
        uint8_t lowAddr,
        uint8_t highData,
        uint8_t lowData);

/* Function    : SensorWrite
   Description : Write arbitrary amount of data to image sensor over I2C interface.
   Parameters  :
                 slaveAddr - I2C slave address for the sensor.
                 highAddr  - High byte of memory address being written to.
                 lowAddr   - Low byte of memory address being written to.
                 count     - Size of write data in bytes. Limited to a maximum of 64 bytes.
                 buf       - Pointer to buffer containing data.
 */
extern CyU3PReturnStatus_t
SensorWrite (
        uint8_t slaveAddr,
        uint8_t highAddr,
        uint8_t lowAddr,
        uint8_t count,
        uint8_t *buf);

/* Function    : SensorRead2B
   Description : Read 2 bytes of data from image sensor over I2C interface.
   Parameters  :
                 slaveAddr - I2C slave address for the sensor.
                 highAddr  - High byte of memory address being written to.
                 lowAddr   - Low byte of memory address being written to.
                 buf       - Buffer to be filled with data. MSB goes in byte 0.
 */
extern CyU3PReturnStatus_t
SensorRead2B (
        uint8_t slaveAddr,
        uint8_t highAddr,
        uint8_t lowAddr,
        uint8_t *buf);

/* Function    : SensorRead
   Description : Read arbitrary amount of data from image sensor over I2C interface.
   Parameters  :
                 slaveAddr - I2C slave address for the sensor.
                 highAddr  - High byte of memory address being written to.
                 lowAddr   - Low byte of memory address being written to.
                 count     = Size of data to be read in bytes. Limited to a max of 64.
                 buf       - Buffer to be filled with data.
 */
extern CyU3PReturnStatus_t
SensorRead (
        uint8_t slaveAddr,
        uint8_t highAddr,
        uint8_t lowAddr,
        uint8_t count,
        uint8_t *buf);

/* Function    : SensorInit
   Description : Initialize the MT9M114 sensor.
   Parameters  : None
 */
extern void
SensorInit (
        void);

/* Function    : SensorReset
   Description : Reset the MT9M114 image sensor using FX3 GPIO.
   Parameters  : None
 */
extern void
SensorReset (
        void);




/* Function    : SensorI2cBusTest
   Description : Test whether the MT9M114 sensor is connected on the I2C bus.
   Parameters  : None
 */
extern uint8_t
SensorI2cBusTest (
        void);

/* Function    : SensorGetBrightness
   Description : Get the current brightness setting from the MT9M114 sensor.
   Parameters  : None
 */
extern uint8_t
SensorGetBrightness (
        void);

/* Function    : SensorSetBrightness
   Description : Set the desired brightness setting on the MT9M114 sensor.
   Parameters  :
                 brightness - Desired brightness level.
 */
extern void
SensorSetBrightness (
        uint8_t input);



#endif /* _INCLUDED_SENSOR_H_ */

/*[]*/

