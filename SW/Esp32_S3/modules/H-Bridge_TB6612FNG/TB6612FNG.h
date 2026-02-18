/** Desired PWM settings:
 * Frequency - 20 kHz; Resolution - 
 * 
 */

#ifndef TB6612FNG_H_
#define TB6612FNG_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint64_t AIN1;
    uint64_t AIN2;
    uint64_t PWMA;
    uint64_t BIN1;
    uint64_t BIN2;
    uint64_t PWMB;
}TB6612FNG_t;

void TB6612FNG_Init(TB6612FNG_t* bridge);

void TB6612FNG_ChA_Stop(TB6612FNG_t* bridge);
void TB6612FNG_ChA_Forward(TB6612FNG_t* bridge, uint32_t pwm);
void TB6612FNG_ChA_Backward(TB6612FNG_t* bridge, uint32_t pwm);

void TB6612FNG_ChB_Stop(TB6612FNG_t* bridge);
void TB6612FNG_ChB_Forward(TB6612FNG_t* bridge, uint32_t pwm);
void TB6612FNG_ChB_Backward(TB6612FNG_t* bridge, uint32_t pwm);

#endif //TB6612FNG_H_
