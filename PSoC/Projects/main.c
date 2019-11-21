/* ========================================
 *
 * Copyright Samuel Walsh, 2014
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Samuel Walsh.
 *
 * ========================================
*/
#include <project.h>
#include <mpu6050.h>
#include <stdio.h>
#include <math.h>

#define LED_LENGTH 104

int mode = 2;
int16_t ax, ay, az, i;
	int16_t gx, gy, gz;

typedef struct _RGB_tag{
	unsigned char g;
	unsigned char r;
	unsigned char b;
} rgb_tag;

rgb_tag rgb[LED_LENGTH];
unsigned int tick, current_led;

void isr_systick();
void isr_reset();
void isr_fifo_empty();
void transfer_LEDdriver();
void set_next_color();

int main()
{
    for(;;)
    {
    char buf[50]; //just to hold text values in for writing to UART

    
	I2C_MPU6050_Start();
	SERIAL_Start();
	
    CyGlobalIntEnable;

	MPU6050_init();
	MPU6050_initialize();
	SERIAL_UartPutString(MPU6050_testConnection() ? "MPU6050 connection successful\n\r" : "MPU6050 connection failed\n\n\r");
    
    SERIAL_UartPutString("Raw values from gyroscope..\n\r");
    isr_fifo_empty_StartEx(isr_fifo_empty);
    isr_reset_done_StartEx(isr_reset);
    
    // configure systick for interrupt each 1ms
    CyIntSetSysVector(15,isr_systick);
    if (SysTick_Config( (40000000) / 1000)) { 
            while (1);  //Capture error
    }
    
    // start component
    Counter_reset_gen_Start() ;
        
    CyGlobalIntEnable;
    
    // wait for dummy interrupt is triggered.
  
    while(1)
    {
      MPU6050_getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      
      sprintf(buf, "AX:%d, AY:%d, AZ:%d || GX:%d, GY:%d, GZ:%d,\t", ax,ay,az,gx,gy,gz);
      SERIAL_UartPutString(buf);
      SERIAL_UartPutString("\n\r");
      CyDelay(300);
    }

    }
}

void isr_systick(){
    tick++;
    if( (tick&0xf) == 0 ){
        set_next_color();   // exec each 16ms
    }
}

// transfer & reset done
void isr_reset(){
    //transfer_LEDdriver();
}

// fifo empty. fill fifo.
void isr_fifo_empty(){
    if(current_led){
        transfer_LEDdriver();
    }
}

// start transfer
void transfer_LEDdriver(){
    unsigned char len;
    if(current_led+3<LED_LENGTH){
        WS2812driver_1_write2fifo((unsigned char*)&rgb[current_led], 9);
        current_led += 3;
    }else{
        len = LED_LENGTH - current_led;
        WS2812driver_1_write2fifo((unsigned char*)&rgb[current_led], len * 3);
        current_led = 0;
    }
}

// exec each 16ms
void set_next_color(){
    switch(mode){
        case 0: //RGB Mode
        {
            static unsigned char val;
            int i;
          
            // shift
            for(i=LED_LENGTH-1;i>0;i--){
                rgb[i].r = rgb[i-1].r;
                rgb[i].g = rgb[i-1].g;
                rgb[i].b = rgb[i-1].b;
            }
    
            if(val<32){
                rgb[0].r = val;
                rgb[0].g = 0;
                rgb[0].b = 32 - val;
            }else if(val<64){
                rgb[0].r = 64 - val;
                rgb[0].g = val - 32;
                rgb[0].b = 0;
            }else{
                rgb[0].r = 0;
                rgb[0].g = 96 - val;
                rgb[0].b = val - 64;
            }
    
            transfer_LEDdriver();
        
            if(val<96){
                val++;
            }else{
                val = 0;
            }
            break;
        }
        case 1:
        {
            
            
            
            break;
        }

        case 2:
        if (ay < -14000)
        {
            rgb[0].r = 255;
            rgb[1].r = 255;
            rgb[2].r = 255;
            rgb[3].r = 255;
            rgb[4].r = 255;
            rgb[5].r = 255;
            rgb[6].r = 255;
            rgb[7].r = 255;
            rgb[8].r = 255;
            rgb[9].r = 255;
            rgb[10].r = 255;
            rgb[11].r = 255;
            rgb[12].r = 255;
            rgb[13].b = 0;
            rgb[14].b = 0;
            rgb[15].b = 0;
            rgb[16].b = 0;
            rgb[17].b = 0;
            rgb[18].b = 0;
            rgb[19].b = 0;
            rgb[20].b = 0;
            rgb[21].b = 0;
            rgb[22].b = 0;
            rgb[23].b = 0;
            rgb[24].b = 0;
            rgb[25].b = 0;
            rgb[26].g = 0;
            rgb[27].g = 0;
            rgb[28].g = 0;
            rgb[29].g = 0;
            rgb[30].g = 0;
            rgb[31].g = 0;
            rgb[32].g = 0;
            rgb[33].g = 0;
            rgb[34].g = 0;
            rgb[35].g = 0;
            rgb[36].g = 0;
            rgb[37].g = 0;
            rgb[38].g = 0;
            rgb[39].r = 0;
            rgb[40].r = 0;
            rgb[41].r = 0;
            rgb[42].r = 0;
            rgb[43].r = 0;
            rgb[44].r = 0;
            rgb[45].r = 0;
            rgb[46].r = 0;
            rgb[47].r = 0;
            rgb[48].r = 0;
            rgb[49].r = 0;
            rgb[50].r = 0;
            rgb[51].r = 0;
            rgb[52].b = 0;
            rgb[53].b = 0;
            rgb[54].b = 0;
            rgb[55].b = 0;
            rgb[56].b = 0;
            rgb[57].b = 0;
            rgb[58].b = 0;
            rgb[59].b = 0;
            rgb[60].b = 0;
            rgb[61].b = 0;
            rgb[62].b = 0;
            rgb[63].b = 0;
            rgb[64].b = 0;
            rgb[65].g = 0; 
            rgb[66].g = 0;
            rgb[67].g = 0;
            rgb[68].g = 0;
            rgb[69].g = 0;
            rgb[70].g = 0;
            rgb[71].g = 0;
            rgb[72].g = 0;
            rgb[73].g = 0;
            rgb[74].g = 0;
            rgb[75].g = 0;
            rgb[76].g = 0;
            rgb[77].g = 0;
            rgb[78].r = 0;
            rgb[79].r = 0;
            rgb[80].r = 0;
            rgb[81].r = 0;
            rgb[82].r = 0;
            rgb[83].r = 0;
            rgb[84].r = 0;
            rgb[85].r = 0;
            rgb[86].r = 0;
            rgb[87].r = 0;
            rgb[88].r = 0;
            rgb[89].r = 0;
            rgb[90].r = 0;
            rgb[91].r = 0;
            rgb[92].b = 0;
            rgb[93].b = 0;
            rgb[94].b = 0;
            rgb[95].b = 0;
            rgb[96].b = 0;
            rgb[97].b = 0;
            rgb[98].b = 0;
            rgb[99].b = 0;
            rgb[100].b = 0;
            rgb[101].b = 0;
            rgb[102].b = 0;
            rgb[103].b = 0;
            transfer_LEDdriver();
        }
        else if(ay > -14000 && ay < -10000){
            rgb[0].r = 255;
            rgb[1].r = 255;
            rgb[2].r = 255;
            rgb[3].r = 255;
            rgb[4].r = 255;
            rgb[5].r = 255;
            rgb[6].r = 255;
            rgb[7].r = 255;
            rgb[8].r = 255;
            rgb[9].r = 255;
            rgb[10].r = 255;
            rgb[11].r = 255;
            rgb[12].r = 255;
            rgb[13].b = 255;
            rgb[14].b = 255;
            rgb[15].b = 255;
            rgb[16].b = 255;
            rgb[17].b = 255;
            rgb[18].b = 255;
            rgb[19].b = 255;
            rgb[20].b = 255;
            rgb[21].b = 255;
            rgb[22].b = 255;
            rgb[23].b = 255;
            rgb[24].b = 255;
            rgb[25].b = 255;
            rgb[26].g = 0;
            rgb[27].g = 0;
            rgb[28].g = 0;
            rgb[29].g = 0;
            rgb[30].g = 0;
            rgb[31].g = 0;
            rgb[32].g = 0;
            rgb[33].g = 0;
            rgb[34].g = 0;
            rgb[35].g = 0;
            rgb[36].g = 0;
            rgb[37].g = 0;
            rgb[38].g = 0;
            rgb[39].r = 0;
            rgb[40].r = 0;
            rgb[41].r = 0;
            rgb[42].r = 0;
            rgb[43].r = 0;
            rgb[44].r = 0;
            rgb[45].r = 0;
            rgb[46].r = 0;
            rgb[47].r = 0;
            rgb[48].r = 0;
            rgb[49].r = 0;
            rgb[50].r = 0;
            rgb[51].r = 0;
            rgb[52].b = 0;
            rgb[53].b = 0;
            rgb[54].b = 0;
            rgb[55].b = 0;
            rgb[56].b = 0;
            rgb[57].b = 0;
            rgb[58].b = 0;
            rgb[59].b = 0;
            rgb[60].b = 0;
            rgb[61].b = 0;
            rgb[62].b = 0;
            rgb[63].b = 0;
            rgb[64].b = 0;
            rgb[65].g = 0; 
            rgb[66].g = 0;
            rgb[67].g = 0;
            rgb[68].g = 0;
            rgb[69].g = 0;
            rgb[70].g = 0;
            rgb[71].g = 0;
            rgb[72].g = 0;
            rgb[73].g = 0;
            rgb[74].g = 0;
            rgb[75].g = 0;
            rgb[76].g = 0;
            rgb[77].g = 0;
            rgb[78].r = 0;
            rgb[79].r = 0;
            rgb[80].r = 0;
            rgb[81].r = 0;
            rgb[82].r = 0;
            rgb[83].r = 0;
            rgb[84].r = 0;
            rgb[85].r = 0;
            rgb[86].r = 0;
            rgb[87].r = 0;
            rgb[88].r = 0;
            rgb[89].r = 0;
            rgb[90].r = 0;
            rgb[91].r = 0;
            rgb[92].b = 0;
            rgb[93].b = 0;
            rgb[94].b = 0;
            rgb[95].b = 0;
            rgb[96].b = 0;
            rgb[97].b = 0;
            rgb[98].b = 0;
            rgb[99].b = 0;
            rgb[100].b = 0;
            rgb[101].b = 0;
            rgb[102].b = 0;
            rgb[103].b = 0;
        transfer_LEDdriver();   
        }
        else if(ay > -10000 && ay < -6000){
            rgb[0].r = 255;
            rgb[1].r = 255;
            rgb[2].r = 255;
            rgb[3].r = 255;
            rgb[4].r = 255;
            rgb[5].r = 255;
            rgb[6].r = 255;
            rgb[7].r = 255;
            rgb[8].r = 255;
            rgb[9].r = 255;
            rgb[10].r = 255;
            rgb[11].r = 255;
            rgb[12].r = 255;
            rgb[13].b = 255;
            rgb[14].b = 255;
            rgb[15].b = 255;
            rgb[16].b = 255;
            rgb[17].b = 255;
            rgb[18].b = 255;
            rgb[19].b = 255;
            rgb[20].b = 255;
            rgb[21].b = 255;
            rgb[22].b = 255;
            rgb[23].b = 255;
            rgb[24].b = 255;
            rgb[25].b = 255;
            rgb[26].g = 255;
            rgb[27].g = 255;
            rgb[28].g = 255;
            rgb[29].g = 255;
            rgb[30].g = 255;
            rgb[31].g = 255;
            rgb[32].g = 255;
            rgb[33].g = 255;
            rgb[34].g = 255;
            rgb[35].g = 255;
            rgb[36].g = 255;
            rgb[37].g = 255;
            rgb[38].g = 255;
            rgb[39].r = 0;
            rgb[40].r = 0;
            rgb[41].r = 0;
            rgb[42].r = 0;
            rgb[43].r = 0;
            rgb[44].r = 0;
            rgb[45].r = 0;
            rgb[46].r = 0;
            rgb[47].r = 0;
            rgb[48].r = 0;
            rgb[49].r = 0;
            rgb[50].r = 0;
            rgb[51].r = 0;
            rgb[52].b = 0;
            rgb[53].b = 0;
            rgb[54].b = 0;
            rgb[55].b = 0;
            rgb[56].b = 0;
            rgb[57].b = 0;
            rgb[58].b = 0;
            rgb[59].b = 0;
            rgb[60].b = 0;
            rgb[61].b = 0;
            rgb[62].b = 0;
            rgb[63].b = 0;
            rgb[64].b = 0;
            rgb[65].g = 0; 
            rgb[66].g = 0;
            rgb[67].g = 0;
            rgb[68].g = 0;
            rgb[69].g = 0;
            rgb[70].g = 0;
            rgb[71].g = 0;
            rgb[72].g = 0;
            rgb[73].g = 0;
            rgb[74].g = 0;
            rgb[75].g = 0;
            rgb[76].g = 0;
            rgb[77].g = 0;
            rgb[78].r = 0;
            rgb[79].r = 0;
            rgb[80].r = 0;
            rgb[81].r = 0;
            rgb[82].r = 0;
            rgb[83].r = 0;
            rgb[84].r = 0;
            rgb[85].r = 0;
            rgb[86].r = 0;
            rgb[87].r = 0;
            rgb[88].r = 0;
            rgb[89].r = 0;
            rgb[90].r = 0;
            rgb[91].r = 0;
            rgb[92].b = 0;
            rgb[93].b = 0;
            rgb[94].b = 0;
            rgb[95].b = 0;
            rgb[96].b = 0;
            rgb[97].b = 0;
            rgb[98].b = 0;
            rgb[99].b = 0;
            rgb[100].b = 0;
            rgb[101].b = 0;
            rgb[102].b = 0;
            rgb[103].b = 0;
            transfer_LEDdriver();   
        }
        else if(ay > -6000 && ay < -2000){
            rgb[0].r = 255;
            rgb[1].r = 255;
            rgb[2].r = 255;
            rgb[3].r = 255;
            rgb[4].r = 255;
            rgb[5].r = 255;
            rgb[6].r = 255;
            rgb[7].r = 255;
            rgb[8].r = 255;
            rgb[9].r = 255;
            rgb[10].r = 255;
            rgb[11].r = 255;
            rgb[12].r = 255;
            rgb[13].b = 255;
            rgb[14].b = 255;
            rgb[15].b = 255;
            rgb[16].b = 255;
            rgb[17].b = 255;
            rgb[18].b = 255;
            rgb[19].b = 255;
            rgb[20].b = 255;
            rgb[21].b = 255;
            rgb[22].b = 255;
            rgb[23].b = 255;
            rgb[24].b = 255;
            rgb[25].b = 255;
            rgb[26].g = 255;
            rgb[27].g = 255;
            rgb[28].g = 255;
            rgb[29].g = 255;
            rgb[30].g = 255;
            rgb[31].g = 255;
            rgb[32].g = 255;
            rgb[33].g = 255;
            rgb[34].g = 255;
            rgb[35].g = 255;
            rgb[36].g = 255;
            rgb[37].g = 255;
            rgb[38].g = 255;
            rgb[39].r = 255;
            rgb[40].r = 255;
            rgb[41].r = 255;
            rgb[42].r = 255;
            rgb[43].r = 255;
            rgb[44].r = 255;
            rgb[45].r = 255;
            rgb[46].r = 255;
            rgb[47].r = 255;
            rgb[48].r = 255;
            rgb[49].r = 255;
            rgb[50].r = 255;
            rgb[51].r = 255;
            rgb[52].b = 0;
            rgb[53].b = 0;
            rgb[54].b = 0;
            rgb[55].b = 0;
            rgb[56].b = 0;
            rgb[57].b = 0;
            rgb[58].b = 0;
            rgb[59].b = 0;
            rgb[60].b = 0;
            rgb[61].b = 0;
            rgb[62].b = 0;
            rgb[63].b = 0;
            rgb[64].b = 0;
            rgb[65].g = 0; 
            rgb[66].g = 0;
            rgb[67].g = 0;
            rgb[68].g = 0;
            rgb[69].g = 0;
            rgb[70].g = 0;
            rgb[71].g = 0;
            rgb[72].g = 0;
            rgb[73].g = 0;
            rgb[74].g = 0;
            rgb[75].g = 0;
            rgb[76].g = 0;
            rgb[77].g = 0;
            rgb[78].r = 0;
            rgb[79].r = 0;
            rgb[80].r = 0;
            rgb[81].r = 0;
            rgb[82].r = 0;
            rgb[83].r = 0;
            rgb[84].r = 0;
            rgb[85].r = 0;
            rgb[86].r = 0;
            rgb[87].r = 0;
            rgb[88].r = 0;
            rgb[89].r = 0;
            rgb[90].r = 0;
            rgb[91].r = 0;
            rgb[92].b = 0;
            rgb[93].b = 0;
            rgb[94].b = 0;
            rgb[95].b = 0;
            rgb[96].b = 0;
            rgb[97].b = 0;
            rgb[98].b = 0;
            rgb[99].b = 0;
            rgb[100].b = 0;
            rgb[101].b = 0;
            rgb[102].b = 0;
            rgb[103].b = 0;
            transfer_LEDdriver();   
        }
        else if(ay > -2000 && ay < 2000){
            rgb[0].r = 255;
            rgb[1].r = 255;
            rgb[2].r = 255;
            rgb[3].r = 255;
            rgb[4].r = 255;
            rgb[5].r = 255;
            rgb[6].r = 255;
            rgb[7].r = 255;
            rgb[8].r = 255;
            rgb[9].r = 255;
            rgb[10].r = 255;
            rgb[11].r = 255;
            rgb[12].r = 255;
            rgb[13].b = 255;
            rgb[14].b = 255;
            rgb[15].b = 255;
            rgb[16].b = 255;
            rgb[17].b = 255;
            rgb[18].b = 255;
            rgb[19].b = 255;
            rgb[20].b = 255;
            rgb[21].b = 255;
            rgb[22].b = 255;
            rgb[23].b = 255;
            rgb[24].b = 255;
            rgb[25].b = 255;
            rgb[26].g = 255;
            rgb[27].g = 255;
            rgb[28].g = 255;
            rgb[29].g = 255;
            rgb[30].g = 255;
            rgb[31].g = 255;
            rgb[32].g = 255;
            rgb[33].g = 255;
            rgb[34].g = 255;
            rgb[35].g = 255;
            rgb[36].g = 255;
            rgb[37].g = 255;
            rgb[38].g = 255;
            rgb[39].r = 255;
            rgb[40].r = 255;
            rgb[41].r = 255;
            rgb[42].r = 255;
            rgb[43].r = 255;
            rgb[44].r = 255;
            rgb[45].r = 255;
            rgb[46].r = 255;
            rgb[47].r = 255;
            rgb[48].r = 255;
            rgb[49].r = 255;
            rgb[50].r = 255;
            rgb[51].r = 255;
            rgb[52].b = 255;
            rgb[53].b = 255;
            rgb[54].b = 255;
            rgb[55].b = 255;
            rgb[56].b = 255;
            rgb[57].b = 255;
            rgb[58].b = 255;
            rgb[59].b = 255;
            rgb[60].b = 255;
            rgb[61].b = 255;
            rgb[62].b = 255;
            rgb[63].b = 255;
            rgb[64].b = 255;
            rgb[65].g = 0; 
            rgb[66].g = 0;
            rgb[67].g = 0;
            rgb[68].g = 0;
            rgb[69].g = 0;
            rgb[70].g = 0;
            rgb[71].g = 0;
            rgb[72].g = 0;
            rgb[73].g = 0;
            rgb[74].g = 0;
            rgb[75].g = 0;
            rgb[76].g = 0;
            rgb[77].g = 0;
            rgb[78].r = 0;
            rgb[79].r = 0;
            rgb[80].r = 0;
            rgb[81].r = 0;
            rgb[82].r = 0;
            rgb[83].r = 0;
            rgb[84].r = 0;
            rgb[85].r = 0;
            rgb[86].r = 0;
            rgb[87].r = 0;
            rgb[88].r = 0;
            rgb[89].r = 0;
            rgb[90].r = 0;
            rgb[91].r = 0;
            rgb[92].b = 0;
            rgb[93].b = 0;
            rgb[94].b = 0;
            rgb[95].b = 0;
            rgb[96].b = 0;
            rgb[97].b = 0;
            rgb[98].b = 0;
            rgb[99].b = 0;
            rgb[100].b = 0;
            rgb[101].b = 0;
            rgb[102].b = 0;
            rgb[103].b = 0;
        transfer_LEDdriver();
            
        }
        
        else if(ay > 2000 && ay < 6000){
            rgb[0].r = 255;
            rgb[1].r = 255;
            rgb[2].r = 255;
            rgb[3].r = 255;
            rgb[4].r = 255;
            rgb[5].r = 255;
            rgb[6].r = 255;
            rgb[7].r = 255;
            rgb[8].r = 255;
            rgb[9].r = 255;
            rgb[10].r = 255;
            rgb[11].r = 255;
            rgb[12].r = 255;
            rgb[13].b = 255;
            rgb[14].b = 255;
            rgb[15].b = 255;
            rgb[16].b = 255;
            rgb[17].b = 255;
            rgb[18].b = 255;
            rgb[19].b = 255;
            rgb[20].b = 255;
            rgb[21].b = 255;
            rgb[22].b = 255;
            rgb[23].b = 255;
            rgb[24].b = 255;
            rgb[25].b = 255;
            rgb[26].g = 255;
            rgb[27].g = 255;
            rgb[28].g = 255;
            rgb[29].g = 255;
            rgb[30].g = 255;
            rgb[31].g = 255;
            rgb[32].g = 255;
            rgb[33].g = 255;
            rgb[34].g = 255;
            rgb[35].g = 255;
            rgb[36].g = 255;
            rgb[37].g = 255;
            rgb[38].g = 255;
            rgb[39].r = 255;
            rgb[40].r = 255;
            rgb[41].r = 255;
            rgb[42].r = 255;
            rgb[43].r = 255;
            rgb[44].r = 255;
            rgb[45].r = 255;
            rgb[46].r = 255;
            rgb[47].r = 255;
            rgb[48].r = 255;
            rgb[49].r = 255;
            rgb[50].r = 255;
            rgb[51].r = 255;
            rgb[52].b = 255;
            rgb[53].b = 255;
            rgb[54].b = 255;
            rgb[55].b = 255;
            rgb[56].b = 255;
            rgb[57].b = 255;
            rgb[58].b = 255;
            rgb[59].b = 255;
            rgb[60].b = 255;
            rgb[61].b = 255;
            rgb[62].b = 255;
            rgb[63].b = 255;
            rgb[64].b = 255;
            rgb[65].g = 255; 
            rgb[66].g = 255;
            rgb[67].g = 255;
            rgb[68].g = 255;
            rgb[69].g = 255;
            rgb[70].g = 255;
            rgb[71].g = 255;
            rgb[72].g = 255;
            rgb[73].g = 255;
            rgb[74].g = 255;
            rgb[75].g = 255;
            rgb[76].g = 255;
            rgb[77].g = 255;
            rgb[77].g = 255;
            rgb[78].r = 0;
            rgb[79].r = 0;
            rgb[80].r = 0;
            rgb[81].r = 0;
            rgb[82].r = 0;
            rgb[83].r = 0;
            rgb[84].r = 0;
            rgb[85].r = 0;
            rgb[86].r = 0;
            rgb[87].r = 0;
            rgb[88].r = 0;
            rgb[89].r = 0;
            rgb[90].r = 0;
            rgb[91].b = 0;
            rgb[92].b = 0;
            rgb[93].b = 0;
            rgb[94].b = 0;
            rgb[95].b = 0;
            rgb[96].b = 0;
            rgb[97].b = 0;
            rgb[98].b = 0;
            rgb[99].b = 0;
            rgb[100].b = 0;
            rgb[101].b = 0;
            rgb[102].b = 0;
            rgb[103].b = 0;                       
        transfer_LEDdriver();
            
        }
        
        else if(ay > 6000 && ay < 10000){
            rgb[0].r = 255;
            rgb[1].r = 255;
            rgb[2].r = 255;
            rgb[3].r = 255;
            rgb[4].r = 255;
            rgb[5].r = 255;
            rgb[6].r = 255;
            rgb[7].r = 255;
            rgb[8].r = 255;
            rgb[9].r = 255;
            rgb[10].r = 255;
            rgb[11].r = 255;
            rgb[12].r = 255;
            rgb[13].b = 255;
            rgb[14].b = 255;
            rgb[15].b = 255;
            rgb[16].b = 255;
            rgb[17].b = 255;
            rgb[18].b = 255;
            rgb[19].b = 255;
            rgb[20].b = 255;
            rgb[21].b = 255;
            rgb[22].b = 255;
            rgb[23].b = 255;
            rgb[24].b = 255;
            rgb[25].b = 255;
            rgb[26].g = 255;
            rgb[27].g = 255;
            rgb[28].g = 255;
            rgb[29].g = 255;
            rgb[30].g = 255;
            rgb[31].g = 255;
            rgb[32].g = 255;
            rgb[33].g = 255;
            rgb[34].g = 255;
            rgb[35].g = 255;
            rgb[36].g = 255;
            rgb[37].g = 255;
            rgb[38].g = 255;
            rgb[39].r = 255;
            rgb[40].r = 255;
            rgb[41].r = 255;
            rgb[42].r = 255;
            rgb[43].r = 255;
            rgb[44].r = 255;
            rgb[45].r = 255;
            rgb[46].r = 255;
            rgb[47].r = 255;
            rgb[48].r = 255;
            rgb[49].r = 255;
            rgb[50].r = 255;
            rgb[51].r = 255;
            rgb[52].b = 255;
            rgb[53].b = 255;
            rgb[54].b = 255;
            rgb[55].b = 255;
            rgb[56].b = 255;
            rgb[57].b = 255;
            rgb[58].b = 255;
            rgb[59].b = 255;
            rgb[60].b = 255;
            rgb[61].b = 255;
            rgb[62].b = 255;
            rgb[63].b = 255;
            rgb[64].b = 255;
            rgb[65].g = 255; 
            rgb[66].g = 255;
            rgb[67].g = 255;
            rgb[68].g = 255;
            rgb[69].g = 255;
            rgb[70].g = 255;
            rgb[71].g = 255;
            rgb[72].g = 255;
            rgb[73].g = 255;
            rgb[74].g = 255;
            rgb[75].g = 255;
            rgb[76].g = 255;
            rgb[77].g = 255;
            rgb[77].g = 255;
            rgb[78].r = 255;
            rgb[79].r = 255;
            rgb[80].r = 255;
            rgb[81].r = 255;
            rgb[82].r = 255;
            rgb[83].r = 255;
            rgb[84].r = 255;
            rgb[85].r = 255;
            rgb[86].r = 255;
            rgb[87].r = 255;
            rgb[88].r = 255;
            rgb[89].r = 255;
            rgb[90].r = 255;
            rgb[91].b = 0;
            rgb[92].b = 0;
            rgb[93].b = 0;
            rgb[94].b = 0;
            rgb[95].b = 0;
            rgb[96].b = 0;
            rgb[97].b = 0;
            rgb[98].b = 0;
            rgb[99].b = 0;
            rgb[100].b = 0;
            rgb[101].b = 0;
            rgb[102].b = 0;
            rgb[103].b = 0;                     
        transfer_LEDdriver();
            
        }
        
        else if(ay > 10000 && ay < 14000){
            rgb[0].r = 255;
            rgb[1].r = 255;
            rgb[2].r = 255;
            rgb[3].r = 255;
            rgb[4].r = 255;
            rgb[5].r = 255;
            rgb[6].r = 255;
            rgb[7].r = 255;
            rgb[8].r = 255;
            rgb[9].r = 255;
            rgb[10].r = 255;
            rgb[11].r = 255;
            rgb[12].r = 255;
            rgb[13].b = 255;
            rgb[14].b = 255;
            rgb[15].b = 255;
            rgb[16].b = 255;
            rgb[17].b = 255;
            rgb[18].b = 255;
            rgb[19].b = 255;
            rgb[20].b = 255;
            rgb[21].b = 255;
            rgb[22].b = 255;
            rgb[23].b = 255;
            rgb[24].b = 255;
            rgb[25].b = 255;
            rgb[26].g = 255;
            rgb[27].g = 255;
            rgb[28].g = 255;
            rgb[29].g = 255;
            rgb[30].g = 255;
            rgb[31].g = 255;
            rgb[32].g = 255;
            rgb[33].g = 255;
            rgb[34].g = 255;
            rgb[35].g = 255;
            rgb[36].g = 255;
            rgb[37].g = 255;
            rgb[38].g = 255;
            rgb[39].r = 255;
            rgb[40].r = 255;
            rgb[41].r = 255;
            rgb[42].r = 255;
            rgb[43].r = 255;
            rgb[44].r = 255;
            rgb[45].r = 255;
            rgb[46].r = 255;
            rgb[47].r = 255;
            rgb[48].r = 255;
            rgb[49].r = 255;
            rgb[50].r = 255;
            rgb[51].r = 255;
            rgb[52].b = 255;
            rgb[53].b = 255;
            rgb[54].b = 255;
            rgb[55].b = 255;
            rgb[56].b = 255;
            rgb[57].b = 255;
            rgb[58].b = 255;
            rgb[59].b = 255;
            rgb[60].b = 255;
            rgb[61].b = 255;
            rgb[62].b = 255;
            rgb[63].b = 255;
            rgb[64].b = 255;
            rgb[65].g = 255; 
            rgb[66].g = 255;
            rgb[67].g = 255;
            rgb[68].g = 255;
            rgb[69].g = 255;
            rgb[70].g = 255;
            rgb[71].g = 255;
            rgb[72].g = 255;
            rgb[73].g = 255;
            rgb[74].g = 255;
            rgb[75].g = 255;
            rgb[76].g = 255;
            rgb[77].g = 255;
            rgb[77].g = 255;
            rgb[78].r = 255;
            rgb[79].r = 255;
            rgb[80].r = 255;
            rgb[81].r = 255;
            rgb[82].r = 255;
            rgb[83].r = 255;
            rgb[84].r = 255;
            rgb[85].r = 255;
            rgb[86].r = 255;
            rgb[87].r = 255;
            rgb[88].r = 255;
            rgb[89].r = 255;
            rgb[90].r = 255;
            rgb[91].b = 255;
            rgb[92].b = 255;
            rgb[93].b = 255;
            rgb[94].b = 255;
            rgb[95].b = 255;
            rgb[96].b = 255;
            rgb[97].b = 255;
            rgb[98].b = 255;
            rgb[99].b = 255;
            rgb[100].b = 255;
            rgb[101].b = 255;
            rgb[102].b = 255;
            rgb[103].b = 255;
            
            transfer_LEDdriver();
            
        }
              
        else{
            rgb[0].r = 0;
            rgb[1].r = 0;
            rgb[2].r = 0;
            rgb[3].r = 0;
            rgb[4].r = 0;
            rgb[5].r = 0;
            rgb[6].r = 0;
            rgb[7].r = 0;
            rgb[8].r = 0;
            rgb[9].r = 0;
            rgb[10].r = 0;
            rgb[11].r = 0;
            rgb[12].r = 0;
            rgb[13].b = 0;
            rgb[14].b = 0;
            rgb[15].b = 0;
            rgb[16].b = 0;
            rgb[17].b = 0;
            rgb[18].b = 0;
            rgb[19].b = 0;
            rgb[20].b = 0;
            rgb[21].b = 0;
            rgb[22].b = 0;
            rgb[23].b = 0;
            rgb[24].b = 0;
            rgb[25].b = 0;
            rgb[26].g = 0;
            rgb[27].g = 0;
            rgb[28].g = 0;
            rgb[29].g = 0;
            rgb[30].g = 0;
            rgb[31].g = 0;
            rgb[32].g = 0;
            rgb[33].g = 0;
            rgb[34].g = 0;
            rgb[35].g = 0;
            rgb[36].g = 0;
            rgb[37].g = 0;
            rgb[38].g = 0;
            rgb[39].r = 0;
            rgb[40].r = 0;
            rgb[41].r = 0;
            rgb[42].r = 0;
            rgb[43].r = 0;
            rgb[44].r = 0;
            rgb[45].r = 0;
            rgb[46].r = 0;
            rgb[47].r = 0;
            rgb[48].r = 0;
            rgb[49].r = 0;
            rgb[50].r = 0;
            rgb[51].r = 0;
            rgb[52].b = 0;
            rgb[53].b = 0;
            rgb[54].b = 0;
            rgb[55].b = 0;
            rgb[56].b = 0;
            rgb[57].b = 0;
            rgb[58].b = 0;
            rgb[59].b = 0;
            rgb[60].b = 0;
            rgb[61].b = 0;
            rgb[62].b = 0;
            rgb[63].b = 0;
            rgb[64].b = 0;
            rgb[65].g = 0;
            rgb[66].g = 0;
            rgb[67].g = 0;
            rgb[68].g = 0;
            rgb[69].g = 0;
            rgb[70].g = 0;
            rgb[71].g = 0;
            rgb[72].g = 0;
            rgb[73].g = 0;
            rgb[74].g = 0;
            rgb[75].g = 0;
            rgb[76].g = 0;
            rgb[77].g = 0;
            rgb[78].r = 0;
            rgb[79].r = 0;
            rgb[80].r = 0;
            rgb[81].r = 0;
            rgb[82].r = 0;
            rgb[83].r = 0;
            rgb[84].r = 0;
            rgb[85].r = 0;
            rgb[86].r = 0;
            rgb[87].r = 0;
            rgb[88].r = 0;
            rgb[89].r = 0;
            rgb[90].r = 0;
            rgb[91].b = 0;
            rgb[92].b = 0;
            rgb[93].b = 0;
            rgb[94].b = 0;
            rgb[95].b = 0;
            rgb[96].b = 0;
            rgb[97].b = 0;
            rgb[98].b = 0;
            rgb[99].b = 0;
            rgb[100].b = 0;
            rgb[101].b = 0;
            rgb[102].b = 0;
            rgb[103].b = 0;
            transfer_LEDdriver();
        }
    }

}
