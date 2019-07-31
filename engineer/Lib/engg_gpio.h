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

/* VARIABLES: PNEUMATIC - related */
#define FLIP_Pin GPIO_PIN_12	// GREEN WIRE
#define FLIP_GPIO_Port GPIOD
#define EXTEND_Pin GPIO_PIN_13 // GRAY WIRE
#define EXTEND_GPIO_Port GPIOD
#define SLIDEL_Pin GPIO_PIN_14 // ORANGE WIRE
#define SLIDEL_GPIO_Port GPIOD
#define SLIDER_Pin GPIO_PIN_15 // WHITE WIRE
#define SLIDER_GPIO_Port GPIOD
#define SEIZE_Pin GPIO_PIN_10 // PURPLE WIRE
#define SEIZE_GPIO_Port GPIOH
/* END of VARIABLES: PNEUMATIC - related */

/* VARIABLES: CAMERA - related */
#define FRONT 0
#define LEFT 1
/* END of VARIABLES: CAMERA - related */

void ENGG_GPIO_Init(void);
int read_sonicL(void);
int read_sonicR(void);
void use_front_cam(void);
void use_rear_cam(void);
int use_screen();
int use_RMcam();

#endif
