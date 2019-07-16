#ifndef ENGG_GPIO_H
#define ENGG_GPIO_H

/* VARIABLES: SENSOR - related */
#define SONICL_Pin GPIO_PIN_2
#define SONICL_GPIO_Port GPIOI
#define SONICR_Pin GPIO_PIN_7
#define SONICR_GPIO_Port GPIOI
/* END of VARIABLES: SENSOR - related */

/* VARIABLES: CAMERA - related */
#define CAMERA_Pin GPIO_PIN_2
#define CAMERA_GPIO_Port GPIOI
/* END of VARIABLES: CAMERA - related */

void ENGG_GPIO_Init(void);
int read_sonicL(void);
int read_sonicR(void);
void use_front_cam(void);
void use_rear_cam(void);

#endif
