/**
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HWDRV_H
#define __HWDRV_H

/* Includes ------------------------------------------------------------------*/

#define GPIO_sensor GPIOA
#define QTRSENSOR1 	GPIO_Pin_6
#define QTRSENSOR2 	GPIO_Pin_7
#define READSENSOR1 GPIO_sensor->IDR & QTRSENSOR1
#define READSENSOR2 GPIO_sensor->IDR & QTRSENSOR2

#define MOTORx		GPIOB
#define MOTORA1 	GPIO_Pin_6
#define MOTORA2 	GPIO_Pin_7
#define MOTORB1 	GPIO_Pin_8
#define MOTORB2 	GPIO_Pin_9

#define ROBOTADDR   98D3,32,30511C,10

/* PWM consts */
#define MINPWM      0
#define MAXPWM      127
#define MIDPWM      64
#define ARMMIDPWM   35
#define SPWM        90
#define SBPWM       34

/* Definition for new status register */
#define Status_STOP         0x00 // 0b00000000
#define Status_FORWARD      0x01 // 0b00000001 
#define Status_BACK         0x02 // 0b00000010    
#define Status_LEFT         0x03 // 0b00000011   
#define Status_RIGHT        0x04 // 0b00000100     
#define Status_SLEFT        0x05 // 0b00000101     
#define Status_SRIGHT       0x06 // 0b00000110   
#define Status_SBLEFT       0x07 // 0b00000111     
#define Status_SBRIGHT      0x08 // 0b00001000 
#define Status_WALL         0x10 // 0b00010000
#define Status_CALIB        0x20 // 0b00100000
#define Status_AUTO         0x40 // 0b01000000
#define Status_OK           0x80 // 0b10000000 
#define ARM_CMD_NO          0x00 // No command
#define ARM_CMD_UP          0x01
#define ARM_CMD_DOWN        0x02
#define ARM_CMD_GRAB        0x04
#define ARM_CMD_RELEASE     0x08

#define TXNUM 7
#define RXNUM 6
#define FAILURES 4
#define CALIBRATENUM 10
#define STARTMARKER		0x81
#define STOPMARKER 		0x8F

#endif
