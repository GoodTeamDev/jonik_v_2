#include "N76E003.h"
#include "Common.h"
#include "Delay.h"
#include "SFR_Macro.h"
#include "Function_define.h"
void I2C_Init(void) {
    // Налаштування I2C
    P13_OpenDrain_Mode; // SDA
    P14_OpenDrain_Mode; // SCL
    set_I2CEN; // Увімкнути I2C
}

void I2C_Start(void) {
    set_STA; // Генерувати стартовий сигнал
    clr_SI;
    while (!SI);
    clr_STA;
}

void I2C_Stop(void) {
    set_STO; // Генерувати стоповий сигнал
    clr_SI;
    while (STO);
}

void I2C_Write(unsigned char dataa) {
    I2DAT = dataa; // Записати дані
    clr_SI;
    while (!SI);
}

unsigned char I2C_Read(unsigned char ack) {
    
    if (ack)
        set_AA;
    else
        clr_AA;
		
		clr_SI;
		while (!SI);

    return I2DAT;
}
