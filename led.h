#ifndef LED_H
#define LED_H

/* ===== 红灯 ===== */
void LED_Red_Init(void);
void LED_Red_On(void);
void LED_Red_Off(void);
void LED_Red_Toggle(void);

/* ===== 绿灯 ===== */
void LED_Green_Init(void);
void LED_Green_On(void);
void LED_Green_Off(void);
void LED_Green_Toggle(void);

/* ===== 板载蓝灯 ===== */
void LED_Board_Init(void);
void LED_Board_On(void);
void LED_Board_Off(void);
void LED_Board_Toggle(void);

/* ===== 全部 ===== */
void LED_All_Init(void);
void LED_All_On(void);
void LED_All_Off(void);

#endif
