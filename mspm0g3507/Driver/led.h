/* LED 控制接口。 */
#ifndef LED_H
#define LED_H

void LED_Red_Init(void);
void LED_Red_On(void);
void LED_Red_Off(void);
void LED_Red_Toggle(void);

void LED_Green_Init(void);
void LED_Green_On(void);
void LED_Green_Off(void);
void LED_Green_Toggle(void);

void LED_Board_Init(void);
void LED_Board_On(void);
void LED_Board_Off(void);
void LED_Board_Toggle(void);

void LED_All_Init(void);
void LED_All_On(void);
void LED_All_Off(void);

#endif
