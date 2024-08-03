#include "motor.h"
#include "stdbool.h"
#include "math.h"
#include <stdint.h>



#define MP6543_SHUNT_RESISTOR   6800.0f
#define MP6543_IGAIN            9200.0f

#define ADC_VREF                3.3f
#define ARR						765.0f
#define CAL_CYCLES              2000
float motorVoltage = 15.0f;


float CURRENT_FILTER_CONST =    7500.0f;


void _drive_u(motor_t* m);
void _drive_v(motor_t* m);
void _drive_w(motor_t* m);
void _drive_uv(motor_t* m);
void _drive_vw(motor_t* m);
void _drive_wu(motor_t* m);
float a;
float b;
float id;
float ki;
float kp;
float tx;
float ty;
float idErr;
float idSum;
float iq;
float iqSum;
float iqErr;
float newa;
float newb;
float outSigID;
float outSigIQ;
float pi = 3.14159265358979323846;
float currentErrorV;
float averageOffset;
float oldOffset;


float tempAvg;
int n;

float avg_speed;

uint16_t readEncoder(motor_t* m){
	uint16_t buffer;

	HAL_SPI_Receive(m->encoder, &buffer, 1, 100); // one 12 bit byte 10 ms timeout

	return (buffer & 0xFFF);
}

void getAngle(motor_t* m){
	readHalls(m);
	//uint16_t encoderVal;
	//encoderVal = readEncoder(m);
	//double angle =((double)(encoderVal/ 4096)*360.0f);
	//double angle = (motor.hallspeed * 2 * 3.14159);



	//m->angle = angle;
}

void readHalls(motor_t* m){
    int hall1 = HAL_GPIO_ReadPin(m->hallPins[0].gpioGroup, m->hallPins[0].gpioPin);
    int hall2 = HAL_GPIO_ReadPin(m->hallPins[1].gpioGroup, m->hallPins[1].gpioPin);
    int hall3 = HAL_GPIO_ReadPin(m->hallPins[2].gpioGroup, m->hallPins[2].gpioPin);
    int steps;




    switch((hall1<<2)|(hall2<<1)|(hall3))
    {
        case 0b100:
            m->angle = 0.0f;
            m->hallState = 0b100;

            break;
        case 0b110:
            //m->angle = (pi / 3.0f);
        	m->angle = 60.0f;
            m->hallState = 0b110;
            break;

        case 0b010:
            //m->angle = (2.0f * pi / 3.0f);
        	m->angle = 120.0f;
            m->hallState = 0b010;

            break;

        case 0b011:
            //m->angle = pi;
        	m->angle = 180.f;
            m->hallState = 0b011;
            break;

        case 0b001:
            //m->angle = (4.0f * pi / 3.0f);
        	m->angle = 240.0f;
            m->hallState = 0b001;
            break;

        case 0b101:
            //m->angle = (5.0f * pi / 3.0f);
        	m->angle = 300.0f;
            m->hallState = 0b101;
            break;

        default:
            m->invalidCts++;
            return;
    }

    m->avg_speed = approxRollingAverage(avg_speed, m->speed);
    m->angleOffset =  60.0f * (m->avg_speed * (float)m->hallCount / 1000000.0f);
    //altered hall input

    if(m->angleOffset > 60.0f ){
    	m->angleOffset = 60.0f;
    }
    else if (m->angleOffset < 0.0f){
    	m->angleOffset = 0.0f;
    }
    m->angle = m->angle +  m->angleOffset;
    m->lastHallState = m->hallState;
  return;
}


void _adcHandler(motor_t* m)
{
    //m->iuDat =  MP6543_IGAIN * ADC_VREF * ((m->adcData[0]/4096.0f)/MP6543_SHUNT_RESISTOR);
    //m->ivDat =  MP6543_IGAIN * ADC_VREF * ((m->adcData[1]/4096.0f)/MP6543_SHUNT_RESISTOR);
    //m->iwDat =  MP6543_IGAIN * ADC_VREF * ((m->adcData[2]/4096.0f)/MP6543_SHUNT_RESISTOR);


    //ADC data is taken in scanning mode resulting in a array of 3 adc values

    return;
}


/**
 * @brief Initialize the motor control structure
 *
 * @param m A pointer to a motor_t struct (not a vicproto MotorState!) that is already initialized with the correct references
 */
void MOTOR_init(motor_t* m)
{
    HAL_TIM_PWM_Start(m->pwm, TIM_CHANNEL_1);
    // Using channel 2 as no output for 0 point adc trigger
    HAL_TIM_PWM_Start(m->pwm, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(m->pwm, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(m->pwm, TIM_CHANNEL_4);

    m->pwm->Instance->CCR1 = round(ARR * m->dutyCycle);
    m->pwm->Instance->CCR2 = 1;
    m->pwm->Instance->CCR3 = round(ARR * m->dutyCycle);
    m->pwm->Instance->CCR4 = round(ARR * m->dutyCycle); // trigger output compare 4 whenever 0

}


void MOTOR_updateShaftAngle(motor_t* m, int angle)
{
    m->angle = angle;
}

float MOTOR_getCurrent(motor_t* m)
{
    return m->averageCurrent;
}

float approxRollingAverage (float avg, float new_sample) {
	float n = 150.0;
    avg -= avg / n;
    avg += new_sample / n;

    return avg;
}

/**
 * Sinusoidal PWM Function
 */
void MOTOR_SVPWMtask(motor_t* m)
{
    HAL_GPIO_WritePin(m->enablePins[0].gpioGroup, m->enablePins[0].gpioPin, 1);
    HAL_GPIO_WritePin(m->enablePins[1].gpioGroup, m->enablePins[1].gpioPin, 1);
    HAL_GPIO_WritePin(m->enablePins[2].gpioGroup, m->enablePins[2].gpioPin, 1);
    //m->dir = m->motorState->pwm >= 0; // hall sensors are backwards

    float convertedAngle = m->angle * pi / 180.0f;
    float commonMode = 0.001f * sin(3.0f * convertedAngle );

	if(m->dir){
		m->U_dutyCycle = (0.5f * (sin(convertedAngle)+ 0.5f) )* m->dutyCycle;
		m->V_dutyCycle = (0.5f * (sin(convertedAngle + (2.0f * pi / 3.0f)) ) + 0.5f)* m->dutyCycle;
		m->W_dutyCycle = (0.5f * (sin(convertedAngle - (2.0f * pi / 3.0f)) + 0.5f) )* m->dutyCycle;
    	m->pwm->Instance->CCR1 = round(765.0f *  m->U_dutyCycle); // pwmu = dutycycle
    	m->pwm->Instance->CCR2 = round(765.0f * m->V_dutyCycle); // pwmw = 0
    	m->pwm->Instance->CCR3 = round(765.0f * m->W_dutyCycle); // pwmw = 0
	}
	else{
		m->pwm->Instance->CCR1 = round(765.0f * (0.5f * sin(convertedAngle) + 0.5)* m->dutyCycle); // pwmu = dutycycle
    	m->pwm->Instance->CCR2 = round(765.0f * (0.5f * sin(convertedAngle -  (4.0f * pi / 3.0f)) + 0.5) * m->dutyCycle); // pwmw = 0
    	m->pwm->Instance->CCR3 = round(765.0f * (0.5f * sin(convertedAngle - (2.0f * pi / 3.0f)) + 0.5) * m->dutyCycle); // pwmw = 0
	}
}

void get_Current(motor_t * m){
	float voltageA = (3.3f *((float) m->adcData[0] / 4096.0f ))-1.65f;
	float voltageB = (3.3f *((float) m->adcData[1] / 4096.0f ))-1.65f;
	float voltageC = (3.3f *((float) m->adcData[2] / 4096.0f ))-1.65f;
	m->iuDat =  MP6543_IGAIN * voltageA / MP6543_SHUNT_RESISTOR;
	m->iwDat =  MP6543_IGAIN * voltageB / MP6543_SHUNT_RESISTOR;
	m->ivDat =  MP6543_IGAIN * voltageC / MP6543_SHUNT_RESISTOR;

	m->iaDat = (m->ivDat +  m->iwDat);
	m->ibDat = (m->iuDat +  m->iwDat);
	m->icDat = (m->ivDat +  m->iuDat);
	return;
}


/**
 * Six Step Commutation PWM Function
 */

void MOTOR_task(motor_t* m)
{
    //m->dir = m->motorState->pwm >= 0; // hall sensors are backwards
    if(m->angle < 60.0f)
    {
        if(m->dir) {
            _drive_wu(m);
            m->pwm->Instance->CCR1 = round(ARR * m->dutyCycle); // pwmu = dutycycle
            m->pwm->Instance->CCR3 = 0; // pwmw = 0
        }
        else {
           _drive_vw(m);
           m->pwm->Instance->CCR4 = round(ARR * m->dutyCycle); // pwmw = dutycycle
           m->pwm->Instance->CCR3 = 0; // pwmv = 0
        }
    }
    else if(m->angle < 120.0f)
    {
        if(m->dir){
            _drive_vw(m);
            m->pwm->Instance->CCR4 = round(ARR * m->dutyCycle); // pwmv = dutycycle
            m->pwm->Instance->CCR3 = 0; // pwmw = 0
        }
        else {
            _drive_uv(m);
            m->pwm->Instance->CCR2 = round(ARR * m->dutyCycle); // pwmu = dutycycle
            m->pwm->Instance->CCR1 = 0; // pwmu= 0
        }
    }
    else if(m->angle < 180.0f)
    {
        if(m->dir){
            _drive_uv(m);
            m->pwm->Instance->CCR4 = round(ARR * m->dutyCycle); // pwmv = dutycycle
            m->pwm->Instance->CCR1 = 0; // pwmu = 0
        }
        else {
           _drive_wu(m);
                m->pwm->Instance->CCR3 = round(ARR * m->dutyCycle); // pwmu = dutycycle
                m->pwm->Instance->CCR1 = 0; // pwmw = 0
        }
    }
    else if(m->angle < 240.0f)
    {
        if(m->dir){
            _drive_wu(m);  // pwmu = dutycycle
            m->pwm->Instance->CCR3 = round(ARR * m->dutyCycle); // pwmw = dutycycle
            m->pwm->Instance->CCR1 = 0; // pwmu = 0
        }
        else {
            _drive_vw(m);
            m->pwm->Instance->CCR3 = round(ARR * m->dutyCycle); // pwmv = dutycycle
            m->pwm->Instance->CCR4 = 0; // pwmw = 0
        }
    }
    else if(m->angle < 300.0f)
    {
        if(m->dir){
            _drive_vw(m);
            m->pwm->Instance->CCR3 = round(ARR * m->dutyCycle); // pwmw = dutycycle
            m->pwm->Instance->CCR4 = 0; // pwmv = 0
        }
        else {
            _drive_uv(m);
            m->pwm->Instance->CCR1 = round(ARR * m->dutyCycle); // pwmu = dutycycle
            m->pwm->Instance->CCR4 = 0; // pwmw = 0
        }
    }
    else if(m->angle < 360.0f)
    {
        if(m->dir){
            _drive_uv(m);
            m->pwm->Instance->CCR1 = round(ARR * m->dutyCycle); // pwmu = dutycycle
            m->pwm->Instance->CCR4 = 0; // pwmv = 0
        }
        else {
           _drive_wu(m);  // pwmu = dutycycle
           m->pwm->Instance->CCR1 = round(ARR * m->dutyCycle); // pwmw = dutycycle
           m->pwm->Instance->CCR3 = 0; // pwmu = 0
       }
    }
}

/**
 * FOC commutation control
 * Uses SVM switching scheme
 */

void MOTOR_FOCtask(motor_t* m)
{
    // Enable all three motor phases
    HAL_GPIO_WritePin(m->enablePins[0].gpioGroup, m->enablePins[0].gpioPin, 1);
    HAL_GPIO_WritePin(m->enablePins[1].gpioGroup, m->enablePins[1].gpioPin, 1);
    HAL_GPIO_WritePin(m->enablePins[2].gpioGroup, m->enablePins[2].gpioPin, 1);

    // Convert angle from degrees to radians
    float convAngle = m->angle * (pi / 180.0f);

    // Clarke transform: Convert three-phase currents to two-phase alpha-beta
    m->a = (float)(m->iaDat);
    m->b = (0.577350267f * (float)(m->iaDat + 2.0f * m->ibDat));

    // Park transform: Convert alpha-beta currents to d-q frame
    m->id =  m->a * cos(convAngle) + m->b * sin(convAngle);
    m->iq = -m->a * sin(convAngle) + m->b * cos(convAngle);

    // Current control: PI controllers for d and q currents
    float idErr =   0.0f - m->id;
    float iqErr = fabs(m->iq) - (0.001f * (float)m->torqueLevel);
    float outSigID = m->id + (0.00000258f * idErr) + (0.0000002f * idSum);
    float outSigIQ = m->iq + (0.0000003f * iqErr) + (0.00000021f * iqSum);
    idSum += idErr;
    iqSum += iqErr;

    // Inverse Park transform: Convert d-q voltages back to alpha-beta
    m->newa = (cos(convAngle) * outSigID - sin(convAngle) * outSigIQ) * motorVoltage;
    m->newb = (sin(convAngle) * outSigID + cos(convAngle) * outSigIQ) * motorVoltage;

    float tempU, tempV, tempW;

    // Space Vector PWM modulation
    svpwm(m, m->newa, m->newb, &tempU, &tempV, &tempW);

    // Limit duty cycles to valid range [0, 0.577]
    tempU = fmaxf(0.0f, fminf(tempU, 0.577f));
    tempV = fmaxf(0.0f, fminf(tempV, 0.577f));
    tempW = fmaxf(0.0f, fminf(tempW, 0.577f));

    // Calculate final duty cycles
    m->U_dutyCycle = tempU * m->dutyCycle;
    m->V_dutyCycle = tempV * m->dutyCycle;
    m->W_dutyCycle = tempW * m->dutyCycle;

    // Update PWM compare registers
    m->pwm->Instance->CCR1 = round(ARR * m->U_dutyCycle);
    m->pwm->Instance->CCR4 = round(ARR * m->V_dutyCycle);
    m->pwm->Instance->CCR3 = round(ARR * m->W_dutyCycle);
}

void svpwm(motor_t * m, float V_alpha, float V_beta, float* D_a, float* D_b, float* D_c) {
    // Calculate the magnitude of the voltage vector
    m->V_magnitude = sqrtf(V_alpha * V_alpha + V_beta * V_beta);

    // Calculate flux angle and convert to degrees
    m->fluxAngle = atan2f(V_beta, V_alpha);
    m->convFlux = m->fluxAngle * 180.0f / pi;

    float convAngle = m->angle * pi / 180.0f;

    // Calculate switching times T1 and T2 based on sector
    float T1, T2;
    if (m->angle < 60.0f) {
        T1 = m->V_magnitude  * cosf(convAngle);
        T2 = m->V_magnitude  * cosf((pi / 3.0f) - convAngle);
    } else if (m->angle < 120.0f) {
        T1 = m->V_magnitude  * cosf(convAngle - pi / 3.0f);
        T2 = m->V_magnitude  * cosf((2.0f * pi / 3.0f) - convAngle);
    } else if (m->angle < 180.0f) {
        T1 = m->V_magnitude  * cosf(convAngle - 2.0f * pi / 3.0f);
        T2 = m->V_magnitude  * cosf((pi / 3.0f) - convAngle);
    } else if (m->angle < 240.0f) {
        T1 = m->V_magnitude  * cosf(convAngle - pi);
        T2 = m->V_magnitude  * cosf((4.0f * pi / 3.0f) - convAngle);
    } else if (m->angle < 300.0f) {
        T1 = m->V_magnitude  * cosf(convAngle - 4.0f * pi / 3.0f);
        T2 = m->V_magnitude  * cosf((5.0f * pi / 3.0f) - convAngle);
    } else {
        T1 = m->V_magnitude  * cosf(convAngle - 5.0f * pi / 3.0f);
        T2 = m->V_magnitude  * cosf((2.0f * pi) - convAngle);
    }

    // Calculate T0 (zero vector time)
    float T0 = 1.0f - T1 - T2;

    if(m->dir){
		// Calculate duty cycles based on sector
		if (m->angle < 60.0f) {
			*D_a = (T1 + T2 + T0) / 2.0f;
			*D_b = (T2 + T0) / 2.0f;
			*D_c = T0 / 2.0f;
		} else if (m->angle < 120.0f) {
			*D_a = (T1 + T0) / 2.0f;
			*D_b = (T1 + T2 + T0) / 2.0f;
			*D_c = T0 / 2.0f;
		} else if (m->angle < 180.0f) {
			*D_a = T0 / 2.0f;
			*D_b = (T1 + T2 + T0) / 2.0f;
			*D_c = (T2 + T0) / 2.0f;
		} else if (m->angle < 240.0f) {
			*D_a = T0 / 2.0f;
			*D_b = (T1 + T0) / 2.0f;
			*D_c = (T1 + T2 + T0) / 2.0f;
		} else if (m->angle < 300.0f) {
			*D_a = (T2 + T0) / 2.0f;
			*D_b = T0 / 2.0f;
			*D_c = (T1 + T2 + T0) / 2.0f;
		} else {
			*D_a = (T1 + T2 + T0) / 2.0f;
			*D_b = T0 / 2.0f;
			*D_c = (T1 + T0) / 2.0f;
		}
    }
    else{
		// Calculate duty cycles based on sector
		if (m->angle < 60.0f) {
			*D_a = T0 / 2.0f;
			*D_b = (T1 + T0) / 2.0f;
			*D_c = (T1 + T2 + T0) / 2.0f;
		} else if (m->angle < 120.0f) {
			*D_a = (T2 + T0) / 2.0f;
			*D_b = T0 / 2.0f;
			*D_c = (T1 + T2 + T0) / 2.0f;
		} else if (m->angle < 180.0f) {
			*D_a = (T1 + T2 + T0) / 2.0f;
			*D_b = T0 / 2.0f;
			*D_c = (T1 + T0) / 2.0f;
		} else if (m->angle < 240.0f) {
			*D_a = (T1 + T2 + T0) / 2.0f;
			*D_b = (T2 + T0) / 2.0f;
			*D_c = T0 / 2.0f;

		} else if (m->angle < 300.0f) {
			*D_a = (T1 + T0) / 2.0f;
			*D_b = (T1 + T2 + T0) / 2.0f;
			*D_c = T0 / 2.0f;

		} else {
			*D_a = T0 / 2.0f;
			*D_b = (T1 + T2 + T0) / 2.0f;
			*D_c = (T2 + T0) / 2.0f;

		}
    }
}

void _drive_uv(motor_t* m){
    HAL_GPIO_WritePin(m->enablePins[0].gpioGroup, m->enablePins[0].gpioPin, 1);
    HAL_GPIO_WritePin(m->enablePins[1].gpioGroup, m->enablePins[1].gpioPin, 1);
    HAL_GPIO_WritePin(m->enablePins[2].gpioGroup, m->enablePins[2].gpioPin, 0);
}
void _drive_vw(motor_t* m){
    HAL_GPIO_WritePin(m->enablePins[0].gpioGroup, m->enablePins[0].gpioPin, 0);
    HAL_GPIO_WritePin(m->enablePins[1].gpioGroup, m->enablePins[1].gpioPin, 1);
    HAL_GPIO_WritePin(m->enablePins[2].gpioGroup, m->enablePins[2].gpioPin, 1);
}
void _drive_wu(motor_t* m){
    HAL_GPIO_WritePin(m->enablePins[0].gpioGroup, m->enablePins[0].gpioPin, 1);
    HAL_GPIO_WritePin(m->enablePins[1].gpioGroup, m->enablePins[1].gpioPin, 0);
    HAL_GPIO_WritePin(m->enablePins[2].gpioGroup, m->enablePins[2].gpioPin, 1);
}


