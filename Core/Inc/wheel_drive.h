#ifndef __WHEEL_DRIVE_H
#define __WHEEL_DRIVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tim.h"

typedef enum
{
  DRIVE_MODE_STOP = 0U,
  DRIVE_MODE_FORWARD = 1U,
  DRIVE_MODE_REVERSE = 2U,
  DRIVE_MODE_LEFT = 3U,
  DRIVE_MODE_RIGHT = 4U,
  DRIVE_MODE_CIRCLE = 5U
} DriveMode;

typedef struct
{
  DriveMode mode;
  uint32_t arr;
  uint32_t left_ccr;
  uint32_t right_ccr;
  uint32_t ccer;
  uint32_t bdtr;
  uint32_t gpioa_idr;
  uint32_t gpioa_odr;
  uint32_t gpioe_idr;
  uint32_t gpioe_odr;
  uint32_t left_encoder_count;
  uint32_t right_encoder_count;
  uint8_t left_encoder_level;
  uint8_t right_encoder_level;
  uint8_t left_a_level;
  uint8_t left_b_level;
  uint8_t right_a_level;
  uint8_t right_b_level;
} WheelDriveSnapshot;

#define WHEEL_DRIVE_DUTY_PERMILLE     350U
#define WHEEL_PWM_MAX_PERMILLE       1000U

#define MOTOR_L_EN_ACTIVE_HIGH         0U
#define MOTOR_L_DIR_ACTIVE_HIGH        0U
#define MOTOR_R_EN_ACTIVE_HIGH         0U
#define MOTOR_R_DIR_ACTIVE_HIGH        0U

#define MOTOR_L_FORWARD_DIR_STATE      0U
#define MOTOR_R_FORWARD_DIR_STATE      1U

void WheelDrive_Init(TIM_HandleTypeDef *tim, uint32_t left_channel, uint32_t right_channel);
void WheelDrive_ApplyMode(DriveMode mode, uint16_t duty_permille);
void WheelDrive_Stop(void);
void WheelDrive_RunRaw(uint16_t left_duty_permille, uint8_t left_dir,
                       uint16_t right_duty_permille, uint8_t right_dir);
DriveMode WheelDrive_GetMode(void);
void WheelDrive_GetSnapshot(WheelDriveSnapshot *snapshot);
void WheelDrive_EXTI_IRQHandler(uint16_t pin);
void WheelDrive_ResetEncoderCounts(void);
const char *WheelDrive_ModeName(DriveMode mode);

#ifdef __cplusplus
}
#endif

#endif /* __WHEEL_DRIVE_H */
