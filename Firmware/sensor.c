/*
 ## Cypress FX3 Camera Kit source file (sensor.c)
 ## ===========================
 ##
 ##  Copyright Cypress Semiconductor Corporation, 2010-2012,
 ##  All Rights Reserved
 ##  UNPUBLISHED, LICENSED SOFTWARE.
 ##
 ##  CONFIDENTIAL AND PROPRIETARY INFORMATION
 ##  WHICH IS THE PROPERTY OF CYPRESS.
 ##
 ##  Use of this file is governed
 ##  by the license agreement included in the file
 ##
 ##     <install>/license/license.txt
 ##
 ##  where <install> is the Cypress software
 ##  installation root directory path.
 ##
 ## ===========================
*/

/* This file implements the I2C based driver for the  image sensor used
   in the FX3 HD 720p camera kit.

   Please refer to the Aptina  sensor datasheet for the details of the
   I2C commands used to configure the sensor.
 */

#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3dma.h>
#include <cyu3error.h>
#include <cyu3uart.h>
#include <cyu3i2c.h>
#include <cyu3types.h>
#include <cyu3gpio.h>
#include <cyu3utils.h>
#include "sensor.h"

static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x3D0,					//current shutter
	.gain = 0x100,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 300,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = true,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = false,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = 0,//current scenario id
	.ihdr_en = 0, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x20,//record current sensor's i2c write id
};




static imgsensor_info_struct imgsensor_info = {
	.sensor_id = IMX219_SENSOR_ID,
	.checksum_value = 0x9e08861c,		//checksum value for Camera Auto Test

	.mode_640x480 = {
		.pclk = 137600000,				//record different mode's pclk
		.linelength = 0xD78,				//record different mode's linelength
		.framelength = 0x534,			//record different mode's framelength
		.startx = 2,					//record different mode's startx of grabwindow
		.starty = 2,					//record different mode's starty of grabwindow
		.grabwindow_width = 1632,		//record different mode's width of grabwindow
		.grabwindow_height = 1224,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
	},
	.mode_1280x720 = {
		.pclk = 265600000,
		.linelength = 0xD78,
		.framelength = 0x9F0,
		.startx = 4,
		.starty = 4,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
	},
	.mode_1920x1080 = {							//capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
		.pclk = 265600000,
		.linelength = 0xD78,
		.framelength = 0x9F0,
		.startx = 4,
		.starty = 4,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 240,	//less than 13M(include 13M),cap1 max framerate is 24fps,16M max framerate is 20fps, 20M max framerate is 15fps
	},
	.mode_1280x720_180 = {
		.pclk = 265600000,
		.linelength = 0xD78,
		.framelength = 0x9F0,
		.startx = 4,
		.starty = 4,
		.grabwindow_width = 3264,
		.grabwindow_height = 2448,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
	},
	.mode_1920x1080_60 = {
		.pclk = 278400000,
		.linelength = 0xD78,
		.framelength = 0x48e,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1920,
		.grabwindow_height = 1080,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 600,
	},
//	.slim_video = {
//		.pclk = 265600000,
//		.linelength = 0xD78,
//		.framelength = 0x9F0,
//		.startx = 4,
//		.starty = 4,
//		.grabwindow_width = 1280,
//		.grabwindow_height = 720,
//		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
//		.max_framerate = 300,
//
//	},

	.margin = 5,			//sensor framelength & shutter margin
	.min_shutter = 2,		//min shutter
	.max_frame_length = 0xffff,//max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,	//shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
	.ae_sensor_gain_delay_frame = 1,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	.ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 5,	  //support sensor mode num

	.cap_delay_frame = 2,		//enter capture delay frame num
	.pre_delay_frame = 2, 		//enter preview delay frame num
	.video_delay_frame = 2,		//enter video delay frame num
	.hs_video_delay_frame = 2,	//enter high speed video  delay frame num
	.slim_video_delay_frame = 2,//enter slim video delay frame num

	.i2c_addr_table = {0x21, 0x20, 0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
};


/* Sensor output window information */
static sensor_winsize_info_struct_t imgsensor_winsize_info[5] =
{{ 3280, 2464,	0, 	0, 3280, 2464, 1640, 1232, 0, 0, 1640, 1232,		2,	2, 1632,  1224}, // Preview
 { 3280, 2464,	0,	0, 3280, 2464, 3280, 2464, 0, 0, 3280, 2464,	4,	4, 3264, 2448}, // capture
 { 3280, 2464,	0,	0, 3280, 2464, 3280, 2464, 0, 0, 3280, 2464,	4,	4, 3264, 2448},  // video
 { 3280, 2464,	0,  0, 3280, 2464, 1920, 1080, 0, 0, 1920, 1080,	0,	0, 1920, 1080}, //hight speed video
 { 3280, 2464,	0,  0, 3280, 2464, 1640,  926, 0, 0, 1640,  926,	0,	0, 1280,  720}};// slim video


#define IMX219MIPI_MaxGainIndex (97)
uint16_t IMX219MIPI_sensorGainMapping[IMX219MIPI_MaxGainIndex][2] ={
{ 64 ,0  },
{ 68 ,12 },
{ 71 ,23 },
{ 74 ,33 },
{ 77 ,42 },
{ 81 ,52 },
{ 84 ,59 },
{ 87 ,66 },
{ 90 ,73 },
{ 93 ,79 },
{ 96 ,85 },
{ 100,91 },
{ 103,96 },
{ 106,101},
{ 109,105},
{ 113,110},
{ 116,114},
{ 120,118},
{ 122,121},
{ 125,125},
{ 128,128},
{ 132,131},
{ 135,134},
{ 138,137},
{ 141,139},
{ 144,142},
{ 148,145},
{ 151,147},
{ 153,149},
{ 157,151},
{ 160,153},
{ 164,156},
{ 168,158},
{ 169,159},
{ 173,161},
{ 176,163},
{ 180,165},
{ 182,166},
{ 187,168},
{ 189,169},
{ 193,171},
{ 196,172},
{ 200,174},
{ 203,175},
{ 205,176},
{ 208,177},
{ 213,179},
{ 216,180},
{ 219,181},
{ 222,182},
{ 225,183},
{ 228,184},
{ 232,185},
{ 235,186},
{ 238,187},
{ 241,188},
{ 245,189},
{ 249,190},
{ 253,191},
{ 256,192},
{ 260,193},
{ 265,194},
{ 269,195},
{ 274,196},
{ 278,197},
{ 283,198},
{ 288,199},
{ 293,200},
{ 298,201},
{ 304,202},
{ 310,203},
{ 315,204},
{ 322,205},
{ 328,206},
{ 335,207},
{ 342,208},
{ 349,209},
{ 357,210},
{ 365,211},
{ 373,212},
{ 381,213},
{ 400,215},
{ 420,217},
{ 432,218},
{ 443,219},
{ 468,221},
{ 482,222},
{ 497,223},
{ 512,224},
{ 529,225},
{ 546,226},
{ 566,227},
{ 585,228},
{ 607,229},
{ 631,230},
{ 656,231},
{ 683,232}
};


/* This function inserts a delay between successful I2C transfers to prevent
   false errors due to the slave being busy.
 */
static void
SensorI2CAccessDelay (
        CyU3PReturnStatus_t status)
{
    /* Add a 10us delay if the I2C operation that preceded this call was successful. */
   // if (status == CY_U3P_SUCCESS)
        CyU3PBusyWait (50);
}

CyU3PReturnStatus_t sensor_i2c_write(uint16_t reg_addr, uint16_t data)
{
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	CyU3PI2cPreamble_t  preamble;
	uint8_t buf[2];
	/* Set the parameters for the I2C API access and then call the write API. */
	preamble.buffer[0] = SENSOR_ADDR_WR;
	preamble.buffer[1] = (reg_addr >> 8) & 0xFF;
	preamble.buffer[2] = (reg_addr) & 0xFF;
	preamble.length    = 3;             /*  Three byte preamble. */
	preamble.ctrlMask  = 0x0000;        /*  No additional start and stop bits. */
	buf[0] = (data>>8) & 0xFF;
	buf[1] = data & 0xFF;
	apiRetStatus = CyU3PI2cTransmitBytes (&preamble, buf, 2, 0);
	SensorI2CAccessDelay (apiRetStatus);

	return apiRetStatus;
}

uint8_t sensor_i2c_read(uint16_t reg_addr)
{
	uint8_t buff;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PI2cPreamble_t preamble;


	preamble.length    = 4;
    preamble.buffer[0] = SENSOR_ADDR_RD & I2C_SLAVEADDR_MASK;        /*  Mask out the transfer type bit. */
    preamble.buffer[1] = (reg_addr >> 8) & 0xFF;
    preamble.buffer[2] = reg_addr & 0xFF;
    preamble.buffer[3] = SENSOR_ADDR_RD ;
    preamble.ctrlMask  = 1<<2;                                /*  Send start bit after third byte of preamble. */

    apiRetStatus = CyU3PI2cReceiveBytes (&preamble, &buff, 1, 0);
    SensorI2CAccessDelay (apiRetStatus);

    return buff;
    //return apiRetStatus;

}

static void set_dummy()
{
	CyU3PDebugPrint(4,"dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
	sensor_i2c_write(0x0160, (imgsensor.frame_length >>8) & 0xFF);
	sensor_i2c_write(0x0161, imgsensor.frame_length & 0xFF);
	sensor_i2c_write(0x0162, (imgsensor.line_length >>8) & 0xFF);
	sensor_i2c_write(0x0163, imgsensor.line_length & 0xFF);

}

static void set_max_framerate(uint16_t framerate, bool min_framelength_en)
{
	uint32_t frame_length = imgsensor.frame_length;
	//unsigned long flags;

	CyU3PDebugPrint(4,"framerate = %d, min framelength should enable? \n", framerate,min_framelength_en);

	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}

	if (min_framelength_en)
	{
		imgsensor.min_frame_length = imgsensor.frame_length;
	}

	set_dummy();
}

typedef struct imx219_reg_s {
	uint16_t address;
	uint8_t val;
}imx219_reg_t;

/*
static const imx219_reg_t mode_1920_1080_regs[] = {
	{0x0100, 0x00},	//mode set 0-> idle 1 -> stream
	{0x30eb, 0x05},	//access sequence
	{0x30eb, 0x0c}, //access sequence
	{0x300a, 0xff}, //access sequence
	{0x300b, 0xff}, //access sequence
	{0x30eb, 0x05}, //access sequence
	{0x30eb, 0x09}, //access sequence
	{0x0114, 0x03}, //3-> 4Lane 1-> 2Lane
	{0x0128, 0x00}, //DPHY timing 0-> auot 1-> manual
	{0x012a, 0x18}, //external oscillator frequncy 0x18 -> 24Mhz
	{0x012b, 0x00},
	{0X0160, 0x02}, //frame length A
	{0X0161, 0x39},
	{0x0162, 0x0d}, //line length
	{0x0163, 0xe7},
	{0x0164, 0x03},	//x start
	{0x0165, 0xE8},
	{0x0166, 0x08},	//x end
	{0x0167, 0xE7},
	{0x0168, 0x02},	//y start
	{0x0169, 0xF0},
	{0x016a, 0x06},	//y stop
	{0x016b, 0xAF},
	{0x016c, 0x02},	//resolution 1280 -> 5 00
	{0x016d, 0x80},
	{0x016e, 0x00},	// 720 - 2 D0	//this setting changes how many line over wire no change on frame rate
	{0x016f, 0x83}, // 4C max at 30 frame lenght , ~83 at 4F frame length
	{0x0170, 0x01},	// x increment
	{0x0171, 0x01},	// y increment
	{0x0174, 0x00},	//binning H 0 off 1 x2 2 x4 3 x2 analog
	{0x0175, 0x00},	//binning V 0 off 1 x2 2 x4 3 x2 analog
	{0x018c, 0x0a}, //CSI Data format
	{0x018d, 0x0a}, //CSI Data format
	{0x0301, 0x05},	//vtpxclkd_div	5
	{0x0303, 0x01},	//vtsclk _div  1
	{0x0304, 0x03}, //external oscillator /3
	{0x0305, 0x03}, //external oscillator /3
	{0x0306, 0x00}, //PLL_VT multiplizer
	{0x0307, 0x58},
	{0x0309, 0x0a},	//oppxck_div
	{0x030b, 0x01}, //opsysck_div
	{0x030c, 0x00}, //PLL_OP
	{0x030d, 0x7D},
	{0x455e, 0x00},
	{0x471e, 0x4b},
	{0x4767, 0x0f},
	{0x4750, 0x14},
	{0x4540, 0x00},
	{0x47b4, 0x14},
	{0x4713, 0x30},
	{0x478b, 0x10},
	{0x478f, 0x10},
	{0x4793, 0x10},
	{0x4797, 0x0e},
	{0x479b, 0x0e},
	{0X0160, 0x00}, //frame length A
	{0X0161, 0x4F}, //@30 max 1545 FPS @ 0x4C vertical
	{0x0162, 0x0d}, //line length
	{0x0163, 0xe7},
	{0x015a, 0x00},	//integration time , really important for frame rate
	{0x015b, 0x2f},
	{0x0100, 0x01}
}; */




static const imx219_reg_t mode_1920_1080_60[]={
		 {0x0100,	0x00},
		 {0x30EB,	0x05},
		 {0x30EB,	0x0C},
		 {0x300A,	0xFF},
		 {0x300B,	0xFF},
		 {0x30EB,	0x05},
		 {0x30EB,	0x09},
		 {0x0114,	0x03},	//3-> 4Lane 1-> 2Lane
		 {0x0128,	0x00},	//DPHY timing 0-> auot 1-> manual
		 {0x012A,	0x18},	//external oscillator frequncy 0x18 -> 24Mhz
		 {0x012B,	0x00},
		 {0x0160,	0x04},	//frame length A
		 {0x0161,	0x8e},
		 {0x0162,	0x0F},	//line length D78 def f88 , does not actually affect how many bits on wire in one line does affect how many clock between lines
		 {0x0163,	0x88},	//appears to be having step in value, not every LSb change will reflect on fps
		 {0x0164,	0x02},	//x start
		 {0x0165,	0xA8},
		 {0x0166,	0x0A},	//x end
		 {0x0167,	0x27},
		 {0x0168,	0x02},	//y start
		 {0x0169,	0xB4},
		 {0x016A,	0x06},	//y end
		 {0x016B,	0xEB},
		 {0x016C,	0x07},	//resolution 1280 -> 5 00
		 {0x016D,	0x80},
		 {0x016E,	0x04},	// 720 - 2 D0	//this setting changes how many line over wire no change on frame rate
		 {0x016F,	0x38},
		 {0x0170,	0x01},	//increment
		 {0x0171,	0x01},	//increment
		 {0x0174,	0x00},	//binning H 0 off 1 x2 2 x4 3 x2 analog
		 {0x0175,	0x00},	//binning H 0 off 1 x2 2 x4 3 x2 analog
		 {0x018C,	0x0A},	//CSI Data format
		 {0x018D,	0x0A},	//CSI Data format
		 {0x0301,	0x05},	//vtpxclkd_div	5
		 {0x0303,	0x01},	//vtsclk _div  1
		 {0x0304,	0x03},	//external oscillator /3
		 {0x0305,	0x03},	//external oscillator /3
		 {0x0306,	0x00},	//PLL_VT multiplizer
		 {0x0307,	0x10},	// 0x30 ~33 , 0x57 ~60	//Changes Frame rate with , integration register 0x15a
		 {0x0309,	0x0A},	//oppxck_div
		 {0x030B,	0x01},	//opsysck_div
		 {0x030C,	0x00},	//PLL_OP
		 {0x030D,	0x20}, // 8Mhz x 0x57 ->696Mhz -> 348Mhz |  0x30 -> 200Mhz | 0x40 -> 256Mhz
		 {0x455E,	0x00},
		 {0x471E,	0x4B},
		 {0x4767,	0x0F},
		 {0x4750,	0x14},
		 {0x4540,	0x00},
		 {0x47B4,	0x14},
		 {0x4713,	0x30},
		 {0x478B,	0x10},
		 {0x478F,	0x10},
		 {0x4793,	0x10},
		 {0x4797,	0x0E},
		 {0x479B,	0x0E},
		 {0x0100,	0x01}
};


static void capture_setting()
{

	imx219_reg_t *reg = mode_1920_1080_60;

	for (uint16_t i = 0; i < _countof(mode_1920_1080_60); i++)
	{
		// CyU3PDebugPrint (4, "Reg 0x%x val 0x%x\n", reg->address, reg->val);
		sensor_i2c_write(reg->address, reg->val);
		reg++;
	}

}


/*
 * Reset the  sensor using GPIO.
 */
void
SensorReset (
        void)
{
    CyU3PReturnStatus_t apiRetStatus;





    /* Wait for some time to allow proper reset. */
    CyU3PThreadSleep (10);

    /* Drive the GPIO high to bring the sensor out of reset. */
    apiRetStatus = CyU3PGpioSetValue (SENSOR_RESET_GPIO, CyTrue);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "GPIO Set Value Error, Error Code = %d\n", apiRetStatus);
        return;
    }

    /* Delay the allow the sensor to power up. */
    CyU3PThreadSleep (10);
    return;
}

static void set_mirror_flip()
{
//	CyU3PDebugPrint(4,"image_mirror = %d\n", image_mirror);

	/********************************************************
	   *
	   *   0x3820[2] ISP Vertical flip
	   *   0x3820[1] Sensor Vertical flip
	   *
	   *   0x3821[2] ISP Horizontal mirror
	   *   0x3821[1] Sensor Horizontal mirror
	   *
	   *   ISP and Sensor flip or mirror register bit should be the same!!
	   *
	   ********************************************************/
	uint8_t  iTemp;
	uint8_t image_mirror;

	image_mirror = IMAGE_NORMAL;
	//CyU3PDebugPrint(4,"set_mirror_flip function\n");
    iTemp = sensor_i2c_read(0x0172) & 0x03;	//Clear the mirror and flip bits.
    switch (image_mirror)
    {
        case IMAGE_NORMAL:
            sensor_i2c_write(0x0172, iTemp | 0x03);	//Set normal
            break;
        case IMAGE_V_MIRROR:
            sensor_i2c_write(0x0172, iTemp | 0x01);	//Set flip
            break;
        case IMAGE_H_MIRROR:
            sensor_i2c_write(0x0172, iTemp | 0x02);	//Set mirror
            break;
        case IMAGE_HV_MIRROR:
            sensor_i2c_write(0x0172, iTemp);	//Set mirror and flip
            break;
    }
	///CyU3PDebugPrint(4,"Error image_mirror setting\n");

}






static uint32_t return_sensor_id()
{
	return ((sensor_i2c_read(0x0000) << 8) | sensor_i2c_read(0x0001));
}


void
SensorInit (
        void)
{

    SensorReset();


    if (SensorI2cBusTest () != CY_U3P_SUCCESS)        /* Verify that the sensor is connected. */
    {
        CyU3PDebugPrint (4, "Error: Reading Sensor ID failed!\r\n");
        return;
    }

//	sensor_i2c_write(0x0008, 0);
//	sensor_i2c_write(0x0202, 0x0001);

    /*

	uint8_t i = 0;
	uint8_t retry = 2;
	uint32_t sensor_id = 0;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {
				CyU3PDebugPrint(4,"i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
				break;
			}
			CyU3PDebugPrint(4,"Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
	return;

	// initail sequence write in

	imgsensor.autoflicker_en= false;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.mode_1920x1080_60.pclk;
	imgsensor.frame_length = imgsensor_info.mode_1920x1080_60.framelength;
	imgsensor.line_length = imgsensor_info.mode_1920x1080_60.linelength;
	imgsensor.min_frame_length = imgsensor_info.mode_1920x1080_60.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = false;
	imgsensor.current_fps = imgsensor_info.mode_1920x1080_60.max_framerate;


	//imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.mode_1920x1080_60.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.mode_1920x1080_60.linelength;
	imgsensor.frame_length = imgsensor_info.mode_1920x1080_60.framelength;
	imgsensor.min_frame_length = imgsensor_info.mode_1920x1080_60.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = false;
	capture_setting();
	set_mirror_flip();
	*/
}

/*
 * Verify that the sensor can be accessed over the I2C bus from FX3.
 */
uint8_t	SensorI2cBusTest (void)
{
    /* The sensor ID register can be read here to verify sensor connectivity. */
    uint8_t buf[2];

    /* Reading sensor ID */

    return CY_U3P_SUCCESS;
}


/*
   Get the current brightness setting from the sensor.
 */
uint8_t
SensorGetBrightness (
        void)
{
    uint8_t buf[2];

    sensor_i2c_read ( 0xCC);
    return (uint8_t)buf[1];
}

/* TODO #2-2 sensor function to read gain register */
/* Copy the SensorGetBrightness function (right aobve this comment),
 * paste it below this comment and rename it to SensorGetGain.
 * Change the register address accessed to 0xCC12.
 */

/*
   Update the brightness setting for the  sensor.
 */
void
SensorSetBrightness (
        uint8_t input)
{
	sensor_i2c_write (0xCC, input);
}

/* TODO #2-3 sensor function to write gain register */
/* Copy the SensorSetBrightness function (right above),
 * paste it below this comment and rename it to SensorSetGain.
 * Change the register address accessed to 0xCC12.
 */


