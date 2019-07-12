#ifndef ENGG_GPIO_H
#define ENGG_GPIO_H

/* VARIABLES: SENSOR - related */
#define SONICL_Pin GPIO_PIN_0
#define SONICL_GPIO_Port GPIOI
#define SONICR_Pin GPIO_PIN_12
#define SONICR_GPIO_Port GPIOH
/* END of VARIABLES: SENSOR - related */

void ENGG_GPIO_Init(void);
int read_sonicL(void);
int read_sonicR(void);

#endif
