//
// Created by matej on 22/12/2019.
//

#include "BLDC_driver.h"

BLDC_driver::BLDC_driver() {

}

void BLDC_driver::phase_set_float(uint8_t phase) {
  switch(phase){
    case PHASE_Y:
      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
      HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
      phase_states[PHASE_Y] = PHASE_FLOAT;
      break;
    case PHASE_B:
      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
      HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
      phase_states[PHASE_B] = PHASE_FLOAT;
      break;
    case PHASE_G:
      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
      HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
      phase_states[PHASE_G] = PHASE_FLOAT;
      break;
  }
}

void BLDC_driver::phase_set_low(uint8_t phase) {
  switch(phase){
    case PHASE_Y:
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
      if(phase_states[PHASE_Y] == PHASE_FLOAT){ //start if prev state was float
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
      }
      phase_states[PHASE_Y] = PHASE_LOW;
      break;
    case PHASE_B:
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
      if(phase_states[PHASE_B] == PHASE_FLOAT){
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
      }
      phase_states[PHASE_B] = PHASE_LOW;
      break;
    case PHASE_G:
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
      if(phase_states[PHASE_G] == PHASE_FLOAT){
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
      }
      phase_states[PHASE_G] = PHASE_LOW;
      break;
  }
}

void BLDC_driver::phase_set_pwm(uint8_t phase, uint16_t pwmSetCompare) {
  switch(phase){
    case PHASE_Y:
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwmSetCompare);
      if(phase_states[PHASE_Y] == PHASE_FLOAT){
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
      }
      phase_states[PHASE_Y] = PHASE_PWM;
      break;
    case PHASE_B:
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwmSetCompare);
      if(phase_states[PHASE_B] == PHASE_FLOAT){
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
      }
      phase_states[PHASE_B] = PHASE_PWM;
      break;
    case PHASE_G:
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwmSetCompare);
      if(phase_states[PHASE_G] == PHASE_FLOAT){
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
      }
      phase_states[PHASE_G] = PHASE_PWM;
      break;
  }
}

void BLDC_driver::set_phases(uint8_t step, uint16_t pwmSetCompare, bool direction, bool is_enabled) {
  if(!is_enabled){
    phase_set_float(PHASE_Y);
    phase_set_float(PHASE_B);
    phase_set_float(PHASE_G);
    return;
  }
  switch(step){
    case 1:
      if(!direction){
        phase_set_float(PHASE_Y);
        phase_set_pwm(PHASE_B, pwmSetCompare);
        phase_set_low(PHASE_G);
      }
      else{
        phase_set_float(PHASE_Y);
        phase_set_pwm(PHASE_G, pwmSetCompare);
        phase_set_low(PHASE_B);
      }
      break;
    case 2:
      if(!direction){
        phase_set_float(PHASE_G);
        phase_set_pwm(PHASE_B, pwmSetCompare);
        phase_set_low(PHASE_Y);
      }
      else{
        phase_set_float(PHASE_G);
        phase_set_pwm(PHASE_Y, pwmSetCompare);
        phase_set_low(PHASE_B);
      }
      break;
    case 3:
      if(!direction){
        phase_set_float(PHASE_B);
        phase_set_pwm(PHASE_G, pwmSetCompare);
        phase_set_low(PHASE_Y);
      }
      else{
        phase_set_float(PHASE_B);
        phase_set_pwm(PHASE_Y, pwmSetCompare);
        phase_set_low(PHASE_G);
      }
      break;
    case 4:
      if(!direction){
        phase_set_float(PHASE_Y);
        phase_set_pwm(PHASE_G, pwmSetCompare);
        phase_set_low(PHASE_B);
      }
      else{
        phase_set_float(PHASE_Y);
        phase_set_pwm(PHASE_B, pwmSetCompare);
        phase_set_low(PHASE_G);
      }
      break;
    case 5:
      if(!direction){
        phase_set_float(PHASE_G);
        phase_set_pwm(PHASE_Y, pwmSetCompare);
        phase_set_low(PHASE_B);
      }
      else{
        phase_set_float(PHASE_G);
        phase_set_pwm(PHASE_B, pwmSetCompare);
        phase_set_low(PHASE_Y);
      }
      break;
    case 6:
      if(!direction){
        phase_set_float(PHASE_B);
        phase_set_pwm(PHASE_Y, pwmSetCompare);
        phase_set_low(PHASE_G);
      }
      else{
        phase_set_float(PHASE_B);
        phase_set_pwm(PHASE_G, pwmSetCompare);
        phase_set_low(PHASE_Y);
      }
      break;
    default:
      phase_set_float(PHASE_Y);
      phase_set_float(PHASE_B);
      phase_set_float(PHASE_G);
      break;
  }
}

void BLDC_driver::enable() {
  BLDC_enabled = true;
}

void BLDC_driver::disable() {
  BLDC_enabled = false;
  BLDC_pwm_set_value = 0;
}

bool BLDC_driver::is_enabled() {
  return BLDC_enabled;
}

void BLDC_driver::begin() {
  phase_set_float(PHASE_B);
  phase_set_float(PHASE_G);
  phase_set_float(PHASE_Y);
  HAL_TIM_Base_Start_IT(&htim3);  //start interrupt timer
}

void BLDC_driver::set_pwm(int16_t pwm) {
  if(pwm > 1000) pwm = 1000;
  if(pwm < -1000) pwm = -1000;
  if(pwm < 0){
    BLDC_direction = false;
    BLDC_pwm_set_value = (-pwm*(double)BLDC_MAX_PWM)/(double)1000;
  }
  else{
    BLDC_direction = true;
    BLDC_pwm_set_value = (pwm*(double)BLDC_MAX_PWM)/(double)1000;

  }
}

//run this at 10kHz
void BLDC_driver::interrupt_handler() {
  bool HALL_states [3] = {0};
  HALL_states[0] = HAL_GPIO_ReadPin(HALL_A_GPIO_Port, HALL_A_Pin) == GPIO_PIN_SET ? true : false;
  HALL_states[1] = HAL_GPIO_ReadPin(HALL_B_GPIO_Port, HALL_B_Pin) == GPIO_PIN_SET ? true : false;
  HALL_states[2] = HAL_GPIO_ReadPin(HALL_C_GPIO_Port, HALL_C_Pin) == GPIO_PIN_SET ? true : false;
  uint8_t pwmLoc = 0;
  if      (HALL_states[0] == 0 && HALL_states[1] == 0 && HALL_states[2] == 0) pwmLoc = 0; //not valid
  else if (HALL_states[0] == 0 && HALL_states[1] == 0 && HALL_states[2] == 1) pwmLoc = 1;
  else if (HALL_states[0] == 0 && HALL_states[1] == 1 && HALL_states[2] == 0) pwmLoc = 5;
  else if (HALL_states[0] == 0 && HALL_states[1] == 1 && HALL_states[2] == 1) pwmLoc = 6;
  else if (HALL_states[0] == 1 && HALL_states[1] == 0 && HALL_states[2] == 0) pwmLoc = 3;
  else if (HALL_states[0] == 1 && HALL_states[1] == 0 && HALL_states[2] == 1) pwmLoc = 2;
  else if (HALL_states[0] == 1 && HALL_states[1] == 1 && HALL_states[2] == 0) pwmLoc = 4;
  else if (HALL_states[0] == 1 && HALL_states[1] == 1 && HALL_states[2] == 1) pwmLoc = 0; //not valid

  set_phases(pwmLoc, BLDC_pwm_set_value, BLDC_direction, BLDC_enabled); //set all phases to correct states

}