#pragma once

#include "stdbool.h"
#include "main.h"
#include  "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_hal_spi.h"


typedef struct {
    GPIO_TypeDef * gpioGroup;
    uint16_t gpioPin;
} gpio_Pin;


typedef struct {
    TIM_HandleTypeDef* pwm;
    gpio_Pin hallPins[3];          // Hall channels for U, V, W pins
    gpio_Pin enablePins[3];
    gpio_Pin motorSleep;
    ADC_HandleTypeDef* adc;
    uint16_t adcData[3];
    uint8_t hallState;
    uint8_t lastHallState;
    SPI_HandleTypeDef* encoder;
    uint16_t encoderStartVal;
    TIM_HandleTypeDef* speedTim;
    float angle;
    float iuDat;
    float ivDat;
    float iwDat;
    float iaDat;
    float ibDat;
    float icDat;
    float fluxAngle;
    float convFlux;
    float averageFlux;
    float U_dutyCycle;
    float V_dutyCycle;
    float W_dutyCycle;
    float V_magnitude;
    float id;
    float iq;
    float a;
    float b;
    float newa;
    float newb;
    int adcSamples;
    float offset[3];
    float speed;
    float avg_speed;
    float accumulatedCurrent;
    float averageCurrent;
    float dutyCycle;
    float hallspeed;
    float hallCount;
    float angleOffset;
    float stateSpeed;
    int torqueLevel;
    bool driveStateChanged;
    int calibrationCount;
    int invalidCts;
    bool currentIsCalibrated[3];
    bool dir;
} motor_t;

void svpwm(motor_t * m, float V_alpha, float V_beta, float* D_a, float* D_b, float* D_c);
void MOTOR_init(motor_t* m);
void MOTOR_updateShaftAngle(motor_t* m, int angle);
void MOTOR_task(motor_t* m);
float MOTOR_getCurrent(motor_t* m);
void readHalls(motor_t* m);
void _adcHandler(motor_t* m);
uint16_t readEncoder(motor_t* m);
void _adcSelU(motor_t* m);
void _adcSelV(motor_t* m);
void _adcSelW(motor_t* m);
void MOTOR_SVPWMtask(motor_t* m);
float approxRollingAverage (float avg, float new_sample);
