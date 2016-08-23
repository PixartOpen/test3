
#include "pixart_ots.h"

/* Global Variables */  
static int32_t x_sum = 0;
static int32_t pre_dsCountX = 0;
/* Local function */
static void OTS_WriteRead(uint8_t address, uint8_t wdata);

void OTS_Reset_Variables(void)
{
	//reset variables	
	x_sum=0;
	pre_dsCountX = 0;
}

uint8_t OTS_Detect_Pressing(int16_t dx16, int16_t dy16)
{
	#define PRESS			1
	#define RELEASE			0
		
	#define DX_ROTATE_TH	2	
	#define DY_VALID_TH		1
	#define ACCY_PRESS_TH	5	
	#define DY_RELEASE_TH	(-1)		
	
	uint8_t OutBtnState = OTS_BTN_NO_CHANGE;
	static int32_t AccY = 0;	
	static uint8_t State = RELEASE; //0:release, 1:press
	
	if((dx16 >= DX_ROTATE_TH)||(dx16 <= (-DX_ROTATE_TH)))
	{
		AccY = 0;	
	}
	else
	{					
		if(State == PRESS) 
		{
			if(dy16 <= DY_RELEASE_TH)
			{		
				State = RELEASE;	
				OutBtnState = OTS_BTN_RELEASE;	
			}	
		}
		else 
		{
			if(dy16 < DY_VALID_TH)
			{
				AccY = 0;
			}
			else 
			{
				AccY += dy16;			
				if(AccY >= ACCY_PRESS_TH)
				{
					AccY = 0;
					State = PRESS;			
					OutBtnState = OTS_BTN_PRESS;
				}				
			}
		}		
	}	
	
	return OutBtnState;
	
}

static uint8_t Detect_Rotation(int32_t dsCountX) 
{
#define EVENT_NUM_PER_ROUND 	10
#define EVENT_COUNT_TH 		(EXPECTED_COUNT_PER_ROUND / EVENT_NUM_PER_ROUND)

 	int32_t diff_count = 0 ;
	uint8_t OutRotState = OTS_ROT_NO_CHANGE;
	
	diff_count = dsCountX - pre_dsCountX ;
	if( diff_count >= EVENT_COUNT_TH )
	{
		pre_dsCountX = dsCountX ;
		OutRotState = OTS_ROT_UP;
	}
	else if( diff_count <= (-EVENT_COUNT_TH) )
	{
		pre_dsCountX = dsCountX ;
		OutRotState = OTS_ROT_DOWN;
	}	
	
	return OutRotState ;
}

static int32_t OTS_Resolution_Downscale(int16_t delta_count) 
{		
	x_sum += delta_count;
	return (x_sum * EXPECTED_COUNT_PER_ROUND / REAL_AVG_COUNT_PER_ROUND);
}

uint8_t OTS_Detect_Rotation(int16_t dx16, int16_t dy16)
{
	return Detect_Rotation( OTS_Resolution_Downscale(dx16) ) ;
}
bool OTS_Sensor_Init(void)
{
  uint8_t SensorPID ;
  bool SPI_OK = 0 ;
	// NCS pin must keep low for at least 1ms after sensor is powered up.
// Please refer to chapter 5 in datasheet for detail.
	nCS_Low_1ms();

	// Read SensorPID in address 0x00 to check if the SPI link is valid, read value should be 0x30.
	SensorPID = ReadData(0x00);
	
	if(SensorPID != 0x30)	
	{	
		SPI_OK=0;
	}
	else
	{
		SPI_OK=1;

		//PAJ9124 sensor recommended settings:
		WriteData(0x7F, 0x00);		// switch to bank0, not allowed to perform OTS_WriteRead		
		WriteData(0x06, 0x91);	// software reset (i.e. set bit7 to 1). It will reset to 0 automatically
													// so perform OTS_WriteRead is not allowed.
		delay_ms(1);			// delay 1ms

		OTS_WriteRead(0x09, 0x5A);	// disable write protect
		OTS_WriteRead(0x05, 0xA0);	// disable sleep mode (depends on application)
		OTS_WriteRead(0x0D, 0x07);	// set X-axis resolution (depends on application)
		OTS_WriteRead(0x0E, 0x14);	// set Y-axis resolution (depends on application)
		OTS_WriteRead(0x19, 0x04);	// set 12-bit X/Y data format (depends on application)
		OTS_WriteRead(0x2B, 0x6D);
		OTS_WriteRead(0x4B, 0x04);	// ONLY for VDD=VDDA=1.7~1.9V: turn off int. regulator for power saving
		OTS_WriteRead(0x5C, 0xD7);	// set LED current source to 7
		
		WriteData(0x7F, 0x01);		// switch to bank1, not allowed to perform OTS_WriteRead
		OTS_WriteRead(0x09, 0x22);	
		OTS_WriteRead(0x2A, 0x03);
		OTS_WriteRead(0x30, 0x4C);
		OTS_WriteRead(0x33, 0x90);
		OTS_WriteRead(0x36, 0xCC);
		OTS_WriteRead(0x37, 0x51);
		OTS_WriteRead(0x38, 0x01);
		OTS_WriteRead(0x3A, 0x7A);
		OTS_WriteRead(0x40, 0x38);
		OTS_WriteRead(0x41, 0x33);
		OTS_WriteRead(0x42, 0x4F);
		OTS_WriteRead(0x43, 0x83);
		OTS_WriteRead(0x44, 0x4F);
		OTS_WriteRead(0x45, 0x80);
		OTS_WriteRead(0x46, 0x23);
		OTS_WriteRead(0x47, 0x49);
		OTS_WriteRead(0x48, 0xC3);
		OTS_WriteRead(0x49, 0x49);
		OTS_WriteRead(0x4A, 0xC0);
		OTS_WriteRead(0x4B, 0x98);
		OTS_WriteRead(0x52, 0x00);
		OTS_WriteRead(0x61, 0x80);
		OTS_WriteRead(0x62, 0x51);
		OTS_WriteRead(0x67, 0x53);
		OTS_WriteRead(0x68, 0x13);
		OTS_WriteRead(0x6C, 0x10);
		OTS_WriteRead(0x6F, 0xF6);
		OTS_WriteRead(0x71, 0x28);
		OTS_WriteRead(0x72, 0x28);
		OTS_WriteRead(0x79, 0x08);

		WriteData(0x7F, 0x00);		// switch to bank0, not allowed to perform OTS_WriteRead
		OTS_WriteRead(0x09, 0x00);	// enable write protect
	}
	return SPI_OK;
} 


void OTS_Sensor_ReadMotion(int16_t *dx, int16_t *dy) 
{
	int16_t deltaX_l=0, deltaY_l=0, deltaXY_h=0;
	int16_t deltaX_h=0, deltaY_h=0;
		
	if( ReadData(0x02) & 0x80 )	//check motion bit in bit7
	{			
		deltaX_l = ReadData(0x03);
		deltaY_l = ReadData(0x04);
		deltaXY_h = ReadData(0x12);
		
		deltaX_h = (deltaXY_h << 4) & 0xF00;
		if(deltaX_h & 0x800)  deltaX_h |= 0xf000;
		
		deltaY_h = (deltaXY_h << 8) & 0xF00;
		if(deltaY_h & 0x800)  deltaY_h |= 0xf000;
	}
	*dx = -(deltaX_h | deltaX_l);	//inverse the data (depends on sensor's orientation and application)
	*dy = -(deltaY_h | deltaY_l);	//inverse the data (depends on sensor's orientation and application)  
}

static void OTS_WriteRead(uint8_t address, uint8_t wdata)
{	
	uint8_t read_value;
	do
	{
		WriteData(address, wdata);		// Write data to specified address
		read_value = ReadData(address);		// Read back previous written data
 	} while(read_value != wdata);			// Check if the data is correctly written
	return;
}

