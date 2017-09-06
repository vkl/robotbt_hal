/**
  ******************************************************************************
  * @file    motor.c
  * @brief   Motor Related Routines.
  ******************************************************************************
*/

#include "stm32f1xx.h"
#include "motor.h"
#include "hwdrv.h"

extern uint8_t status;
extern uint8_t drv1;
extern uint8_t drv2;
extern uint8_t arm_status;

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim2;

uint8_t tmp = 0;


void status_handler(void)
{
  
  if  (arm_status == ARM_CMD_UP) 
  {
      HAL_GPIO_WritePin(GPIOB, LIFT_UP_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, LIFT_DOWN_Pin, GPIO_PIN_RESET);
      tmp = 1;
  }
  else if (arm_status == ARM_CMD_DOWN)
  {
      HAL_GPIO_WritePin(GPIOB, LIFT_UP_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, LIFT_DOWN_Pin, GPIO_PIN_SET);
      tmp = 2;
  } 
  else if (arm_status == ARM_CMD_GRAB)
  {
    HAL_GPIO_WritePin(ARM_GRUB_GPIO_Port, ARM_GRUB_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ARM_RELEASE_GPIO_Port, ARM_RELEASE_Pin, GPIO_PIN_RESET);
    tmp = 3;
  } 
  else if (arm_status == ARM_CMD_RELEASE)
  {
    HAL_GPIO_WritePin(ARM_GRUB_GPIO_Port, ARM_GRUB_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ARM_RELEASE_GPIO_Port, ARM_RELEASE_Pin, GPIO_PIN_SET);
    tmp = 4; 
  } 
  else 
  {
      HAL_GPIO_WritePin(GPIOB, LIFT_UP_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, LIFT_DOWN_Pin, GPIO_PIN_RESET);
      tmp = 0;
      HAL_GPIO_WritePin(ARM_GRUB_GPIO_Port, ARM_GRUB_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ARM_RELEASE_GPIO_Port, ARM_RELEASE_Pin, GPIO_PIN_RESET);
  }
  
  if  (Status_AUTO & status)
  {
    switch (0x0F & status)
    {
        case Status_FORWARD:
            drv1 = MAXPWM; drv2 = MAXPWM;
            break;
        case Status_BACK:
            drv1 = MINPWM; drv2 = MINPWM;
            break;
        case Status_LEFT:
            drv1 = MINPWM; drv2 = MAXPWM;
            break;
        case Status_RIGHT:
            drv2 = MINPWM; drv1 = MAXPWM;
            break;
        case Status_SLEFT:
            drv1 = SPWM; drv2 = MAXPWM;
            break;
        case Status_SRIGHT:
            drv2 = SPWM; drv1 = MAXPWM;
            break;
        case Status_SBLEFT:
            drv1 = SBPWM; drv2 = MINPWM;
            break;
        case Status_SBRIGHT:
            drv2 = SBPWM; drv1 = MINPWM;
            break;
        default:
            drv1 = MIDPWM; drv2 = MIDPWM;
            break;
    }
  }
}

void move(void)
{
  if ((drv2 >= MIDPWM) && (drv2 <= MAXPWM))
	{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, drv2 - MIDPWM);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, MINPWM);
	}
	else if ((drv2 >= MINPWM) && (drv2 < MIDPWM))
	{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, MINPWM);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, (drv2 - MIDPWM) * -1);
	}
	if ((drv1 >= MIDPWM) && (drv1 <= MAXPWM))
	{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, drv1 - MIDPWM);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, MINPWM);
	}
	else if ((drv1 >= MINPWM) && (drv1 < MIDPWM))
	{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, MINPWM);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, (drv1 - MIDPWM) * -1);
	}
}

void forward(void)
{
	drv1 = drv2 = MAXPWM;
	move();
}

void backward(void)
{
	drv1 = drv2 = MINPWM;
	move();
}

void left(void)
{
	drv1 = MINPWM;
	drv2 = MAXPWM;
	move();
}

void right(void)
{
	drv1 = MAXPWM;
	drv2 = MINPWM;
	move();
}

void stop(void)
{
	drv1 = drv2 = MIDPWM;
	move();
}

