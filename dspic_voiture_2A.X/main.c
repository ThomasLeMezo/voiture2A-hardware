/**
  Generated main.c file from MPLAB Code Configurator

  @Company
    Microchip Technology Inc.

  @File Name
    main.c

  @Summary
    This is the generated main.c using PIC24 / dsPIC33 / PIC32MM MCUs.

  @Description
    This source file provides main entry point for system initialization and application code development.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.171.1
        Device            :  dsPIC33CK256MP202
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.70
        MPLAB 	          :  MPLAB X v5.50
*/

/*
    (c) 2020 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

/**
  Section: Included Files
*/
#include "mcc_generated_files/system.h"
#include "mcc_generated_files/pin_manager.h"
#include "stdbool.h"
#include "mcc_generated_files/uart1.h"
#include "mcc_generated_files/tmr1.h"
#include "mcc_generated_files/i2c1.h"
#include <math.h>

#define FCY 100000000UL
#include <xc.h>
#include <libpic30.h>

const char device_name[16] = "DSPIC_VOITURE_2A";
const char code_version = 0x01;
volatile unsigned char i2c_nb_bytes = 0;
volatile unsigned char i2c_register = 0x00;

// PWM motors/servo
#define MOTOR_CMD_STOP (1500/5)
const float motor_cmd_min = 1100.0/5.0; // 1100.0/2.0;
const float motor_cmd_max = 1900.0/5.0; //1900.0/2.0;
#define PWM_PERIOD 4000
// pwm : [0 = SERVO, 1 = MOTOR]
volatile unsigned short countdown_pwm_cmd[2];
volatile unsigned short countdown_pwm[2];
volatile unsigned int countdown_pwm_period = PWM_PERIOD;

volatile unsigned char watchdog_restart_default = 10;
volatile unsigned char watchdog_countdown_restart = 10;

#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01

unsigned char channel_switch_auto = 14;
unsigned short channel_switch_auto_threshold = 180;

uint8_t battery_volt[2];

/*
 * PWM generator (10us)
 */
void timer_pwm(){
    if(countdown_pwm_period==0){
        PWM_SERVO_SetHigh();
        PWM_MOTOR_SetHigh();
        countdown_pwm_period = PWM_PERIOD;

        countdown_pwm[0] = countdown_pwm_cmd[0];
        countdown_pwm[1] = countdown_pwm_cmd[1];
    }
    else{
      countdown_pwm_period--;
      
      // ESC1 1
      if(countdown_pwm[0]==0)
          PWM_SERVO_SetLow();
      else
        countdown_pwm[0]--;
      
      // ESC 2
      if(countdown_pwm[1]==0)
          PWM_MOTOR_SetLow();
      else
        countdown_pwm[1]--;
    }
}

/*
 * Timer watchdog for pwm (every 0.2s)
 */
void SCCP1_TMR_Timer32CallBack()
{
    if(watchdog_countdown_restart>0)
      watchdog_countdown_restart--;  
    else{
      countdown_pwm_cmd[0] = MOTOR_CMD_STOP;
      countdown_pwm_cmd[1] = MOTOR_CMD_STOP;
    }
    //LED_Toggle();
}

// https://www.ordinoscope.net/index.php/Electronique/Protocoles/SBUS
uint8_t uart_buffer[25];
unsigned char idx = 0;

// channel(1..16), digital raw data, range from 0 to 4096, normal from 352 1696
// digital channel(1..2), range 0..1
unsigned short channels[18];
unsigned char failsafe = 0;
unsigned char lost = 0;
int uart_errors = 0;

void tx_uart_data(){
    // Test
    LED_Toggle();
}

void read_uart_data()
{
    if(!UART1_IsRxReady())
        return;
    uint8_t data = UART1_Read();
    if (idx == 0 && data != 0x0F && idx<25) {  // start byte
      // error - wait for the start byte
        idx = 0;
    } else {
      uart_buffer[idx++] = data;  // fill the buffer
    }
    
    if (idx >= 25) {  // decode
        idx = 0;
        if (uart_buffer[24] != 0x00) {
            uart_errors++;
        } else {
            channels[0]  = ((uart_buffer[1]    |uart_buffer[2]<<8)                      & 0x07FF);
            channels[1]  = ((uart_buffer[2]>>3 |uart_buffer[3]<<5)                      & 0x07FF);
            channels[2]  = ((uart_buffer[3]>>6 |uart_buffer[4]<<2 |uart_buffer[5]<<10)  & 0x07FF);
            channels[3]  = ((uart_buffer[5]>>1 |uart_buffer[6]<<7)                      & 0x07FF);
            channels[4]  = ((uart_buffer[6]>>4 |uart_buffer[7]<<4)                      & 0x07FF);
            channels[5]  = ((uart_buffer[7]>>7 |uart_buffer[8]<<1 |uart_buffer[9]<<9)   & 0x07FF);
            channels[6]  = ((uart_buffer[9]>>2 |uart_buffer[10]<<6)                     & 0x07FF);
            channels[7]  = ((uart_buffer[10]>>5|uart_buffer[11]<<3)                     & 0x07FF);
            channels[8]  = ((uart_buffer[12]   |uart_buffer[13]<<8)                     & 0x07FF);
            channels[9]  = ((uart_buffer[13]>>3|uart_buffer[14]<<5)                     & 0x07FF);
            channels[10] = ((uart_buffer[14]>>6|uart_buffer[15]<<2|uart_buffer[16]<<10) & 0x07FF);
            channels[11] = ((uart_buffer[16]>>1|uart_buffer[17]<<7)                     & 0x07FF);
            channels[12] = ((uart_buffer[17]>>4|uart_buffer[18]<<4)                     & 0x07FF);
            channels[13] = ((uart_buffer[18]>>7|uart_buffer[19]<<1|uart_buffer[20]<<9)  & 0x07FF);
            channels[14] = ((uart_buffer[20]>>2|uart_buffer[21]<<6)                     & 0x07FF);
            channels[15] = ((uart_buffer[21]>>5|uart_buffer[22]<<3)                     & 0x07FF);
            channels[16] = ((uart_buffer[23])      & 0x0001) ? 2047 : 0;
            channels[17] = ((uart_buffer[23] >> 1) & 0x0001) ? 2047 : 0;

            failsafe = ((uart_buffer[23] >> 2) & 0b11);
            if ((uart_buffer[23] >> 2) & 0x0001) lost++;
        }
    }
}

bool I2C1_StatusCallback(I2C1_SLAVE_DRIVER_STATUS status)
{
    static uint8_t i2c_data, i2c_address;
    static uint8_t i2c_address_rest = true;
    static uint8_t i2c_default_data = 0x00;
    static float half_pwm;

    switch (status)
    {
        case I2C1_SLAVE_TRANSMIT_REQUEST_DETECTED:
            // set up the slave driver buffer transmit pointer
            i2c_address_rest = false;
            
            switch(i2c_address){
                case 0x00 ... 0x04: // PWM values
                    I2C1_ReadPointerSet(&(((char*)countdown_pwm_cmd)[i2c_address]));
                    break;
                case 0x10 ... 0x34: // Channels
                    I2C1_ReadPointerSet(&(((char*)channels)[i2c_address-0x10]));
                    break;
                case 0x35:
                    I2C1_ReadPointerSet(&failsafe);
                    break;
                case 0x36:
                    I2C1_ReadPointerSet(&lost);
                    break;
                    
                case 0x37 ... 0x38:
                    I2C1_ReadPointerSet(&(((char*)battery_volt)[i2c_address-0x40]));
                    break;
                    
                case 0xC0:
                    I2C1_ReadPointerSet(&code_version);
                    break;
                    
                case 0xF0 ... 0xFF:
                    I2C1_ReadPointerSet(&(((char*)device_name)[i2c_address-0xF0]));
                    break;
                default:
                    I2C1_ReadPointerSet(&i2c_default_data);
            }
            i2c_address++;
            break;

        case I2C1_SLAVE_RECEIVE_REQUEST_DETECTED:
            // set up the slave driver buffer receive pointer
            I2C1_WritePointerSet(&i2c_data);
            i2c_address_rest = true;
            i2c_address = i2c_data;
            break;

        case I2C1_SLAVE_RECEIVED_DATA_DETECTED:
            if(i2c_address_rest){
                i2c_address = i2c_data;
                i2c_address_rest = false;
            }
            else{
                switch(i2c_address){
                    case 0x00 ... 0x01:
                        half_pwm = ((float)i2c_data)*((motor_cmd_max-motor_cmd_min)/255.0)+motor_cmd_min;
                        countdown_pwm_cmd[i2c_address] = round(half_pwm); // (PWM_MAX-PWM_MIN)/255+PWM_MIN => 1102 for rounding
                        watchdog_countdown_restart = watchdog_restart_default;
                        break;
                }
                i2c_address++;
            }

            break;

        default:
            break;
    }
    return true;
}

void get_battery( uint16_t adcVal ){
    battery_volt[0] = adcVal;
    battery_volt[1] = adcVal>>8;
}

/*
                         Main application
 */
int main(void)
{ 
    // initialize the device
    SYSTEM_Initialize();

    // Timers
    TMR1_SetInterruptHandler(&timer_pwm);
    UART1_SetRxInterruptHandler(&read_uart_data);
    UART1_SetTxInterruptHandler(&tx_uart_data);
    ADC1_SetV_BATTInterruptHandler(&get_battery);
    
    for(int i=0; i<20; i++){
        LED_Toggle();
        __delay_ms(100);        
    }
    
    while (1)
    {
        ClrWdt();
        // Add your application code
        
        if(failsafe != SBUS_SIGNAL_OK){
            countdown_pwm_cmd[0] = MOTOR_CMD_STOP;
            countdown_pwm_cmd[1] = MOTOR_CMD_STOP;
        }
        if(channels[channel_switch_auto]<channel_switch_auto_threshold 
                || failsafe != SBUS_SIGNAL_OK
                //|| watchdog_countdown_restart==0
                ){
            SWITCH_AUTO_SetLow();
            LED_SetLow();
        }
        else{
            SWITCH_AUTO_SetHigh();
            LED_SetHigh();
        }
        
    }
    return 1; 
}
/**
 End of File
*/

