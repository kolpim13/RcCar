#ifndef CAR_H_
#define CAR_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    Gear_Forward = 0,
    Gear_Backward = 1,
    Gear_Stop = 10,
}Gearbox_t;
/*=============================================================*/

void platform_init(void);
void platform_stearing_set(uint16_t us);
uint16_t platform_stearing_get(void);
/*=============================================================*/

#endif //CAR_H_
