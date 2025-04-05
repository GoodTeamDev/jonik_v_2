/********************************
	ДЖОНІК 2 ver 0.1 
	(варіант з затримкою для протимінної техніки) 
*********************************/

#include "N76E003.h"
#include "SFR_Macro.h"
#include "Function_define.h"
#include "Common.h"
#include "Delay.h"
#include "I2C.h"
#include <stdio.h>
#include <math.h>

#define QMC5883_ADDRESS 0x0D 															// Адреса I2C для QMC5883
#define QMC5883_ADDR_WRITE (QMC5883_ADDRESS << 1) 				// Адреса для запису
#define QMC5883_ADDR_READ ((QMC5883_ADDRESS << 1) | 1) 		// Адреса для читання 
#define QMC5883_REG_X_LSB 0x00 														// Початковий регістр для осі X

// НАДАШТУВАННЯ +++++++++++++++++
#define DELAY_AFTER_ACTIVATION 					5000 					// затримка після знаходження цілі і до активації - 1000ms = 1sec
#define deviationCoefficientFlight 			35 						// чутливість в режимі польоту
#define deviationCoefficientInit 				50						// чутливість в бойовому режимі
#define daysSelfDestruction 						60 						// час спрацювання  після активації бойового режиму  - ТЗ: 60 днів
#define TIMEUnchangedMinutes 						10 						// час збору данних в рижімі польоту та перезапуску режиму  - ТЗ: 10хв
#define SMOOTHING_WINDOW 								7 						// вікно зсуву для вирівнювання даних магнітометра
#define QMC5883L_SCALE_FACTOR 					0.732421875f 	// коефіцієнт маштабу данних магнітометра
// НАДАШТУВАННЯ -----------------

volatile int16_t x, y, z, xyz0;

volatile uint16_t timerCount 			= 0;
volatile uint8_t elapsedDays 			= 0;
volatile uint8_t elapsedHours 		= 0;
volatile uint8_t elapsedMinutes 	= 0;
volatile uint8_t elapsedSeconds 	= 0;
unsigned char main_operating_mode = 0;

int16_t x_buffer[SMOOTHING_WINDOW] = {0};
int16_t y_buffer[SMOOTHING_WINDOW] = {0};
int16_t z_buffer[SMOOTHING_WINDOW] = {0};
uint8_t buffer_index = 0;

unsigned char QMC5883_ReadReg(unsigned char reg) {
    unsigned char value;

    I2C_Start();
    if (I2STAT != 0x08) {
        //printf("I2C START error: I2STAT = %02X\r\n", I2STAT);
        I2C_Stop();
        return 0xFF; // Повертаємо значення за замовчуванням
    }

    I2C_Write(QMC5883_ADDR_WRITE);
    if (I2STAT != 0x18) {
        //printf("I2C WRITE ADDR error: I2STAT = %02X\r\n", I2STAT);
        I2C_Stop();
        return 0xFF;
    }

    I2C_Write(reg);
    if (I2STAT != 0x28) {
        //printf("I2C WRITE REG error: I2STAT = %02X\r\n", I2STAT);
        I2C_Stop();
        return 0xFF;
    }

    I2C_Start();
    if (I2STAT != 0x10) {
        //printf("I2C repeated START error: I2STAT = %02X\r\n", I2STAT);
        I2C_Stop();
        return 0xFF;
    }

    I2C_Write(QMC5883_ADDR_READ);
    if (I2STAT != 0x40) {
       // printf("I2C READ ADDR error: I2STAT = %02X\r\n", I2STAT);
        I2C_Stop();
        return 0xFF;
    }

    value = I2C_Read(0);
    I2C_Stop();

   // printf("QMC5883 ReadReg(0x%X) = 0x%02X\r\n", (unsigned int)reg, (unsigned int)value);
    return value;
}

void QMC5883_WriteReg(unsigned char reg, unsigned char value) {
    I2C_Start();
    I2C_Write(QMC5883_ADDR_WRITE);
    I2C_Write(reg);
    I2C_Write(value);
    I2C_Stop();
}

void QMC5883_Init(void) {
	QMC5883_WriteReg(0x0A, 0x80); // Soft reset
	Timer0_Delay1ms(10);
	QMC5883_WriteReg(0x09, 0x0D); // 100Hz, 2G, QMC5883 Continuous mode
}

void smoothXYZ() {
  int16_t x_sum = 0, y_sum = 0, z_sum = 0;
	uint8_t i;
    
  // Додаємо нові значення в буфер
	x_buffer[buffer_index] = x;
	y_buffer[buffer_index] = y;  
	z_buffer[buffer_index] = z;
    
  // Оновлюємо індекс буфера
	buffer_index = (buffer_index + 1) % SMOOTHING_WINDOW;
    
  // Обчислюємо середнє значення
	for (i = 0; i < SMOOTHING_WINDOW; i++) {
		x_sum += x_buffer[i];
		y_sum += y_buffer[i];
		z_sum += z_buffer[i];
	}

	x = (int16_t)(x_sum / SMOOTHING_WINDOW);
	y = (int16_t)(y_sum / SMOOTHING_WINDOW);  
	z = (int16_t)(z_sum / SMOOTHING_WINDOW);
}


void QMC5883_ReadData() {
    unsigned char x_lsb, x_msb, y_lsb, y_msb, z_lsb, z_msb;
	
		unsigned char status = QMC5883_ReadReg(0x06);
		if (status & 0x02) {
				//printf("Переповнення! Очистка...\r\n");
				QMC5883_WriteReg(0x09, 0x0D); // Перезапустити continuous mode
		}

    I2C_Start();
    I2C_Write(QMC5883_ADDR_WRITE); // Адреса з записом
    I2C_Write(QMC5883_REG_X_LSB); // Початковий регістр
	
    I2C_Start();
	
    I2C_Write(QMC5883_ADDR_READ); // Адреса з читанням

    x_lsb = I2C_Read(1);
    x_msb = I2C_Read(1);
    y_lsb = I2C_Read(1);
    y_msb = I2C_Read(1);
    z_lsb = I2C_Read(1);
    z_msb = I2C_Read(0);

    I2C_Stop();
		
		x = (int16_t)(x_msb << 8 | x_lsb);
    y = (int16_t)(y_msb << 8 | y_lsb);
    z = (int16_t)(z_msb << 8 | z_lsb);
		
		x = abs(x) * QMC5883L_SCALE_FACTOR;
		y = abs(y) * QMC5883L_SCALE_FACTOR;
		z = abs(z) * QMC5883L_SCALE_FACTOR;
		
		smoothXYZ(); // згладжую данні
}

// ****************** TIMER ********************************
// TIMER0 interrupt handler
void Timer0_ISR(void) interrupt 1 {
	TH0 = 0xFC;
	TL0 = 0x18;
	timerCount++;
	
	if (timerCount >= 1000) {
		timerCount = 0;    
		elapsedSeconds++;
		
		if (elapsedSeconds >= 60) {
			elapsedSeconds = 0;
			elapsedMinutes++;		
			
			if (elapsedMinutes >= 60) {                
				elapsedMinutes = 0;
        elapsedHours++;

				if (elapsedHours >= 24) {
					elapsedHours = 0;
					elapsedDays++;       
				}     
			}   
		} 
	}	
}

// Initializing TIMER0 in 16-bit timer mode (Mode 1)
void Timer0_Init() {
    TMOD |= 0x01;  // Timer0, mode 1 (16-bit)
    
    // Load the initial value for 1 ms
    TH0 = 0xFC;
    TL0 = 0x18;

    ET0 = 1;  // Allow interrupts from timer 0
    TR0 = 1;  // Start timer
    EA = 1;   // Globally enable interrupts
}

uint16_t magneticFieldDeviationFanc() {
	return abs( xyz0 - (x + y + z) );
}

void movementFlight() {
	int16_t magneticFieldDeviation;
  P03 = 0;
	elapsedSeconds = 0;
	
	while (elapsedMinutes <= TIMEUnchangedMinutes) {
		
		QMC5883_ReadData();

		P04 = !P04;
		
		magneticFieldDeviation = magneticFieldDeviationFanc();			

		printf("Flight Deviation: %d\r\n", magneticFieldDeviation);
		
		if(elapsedSeconds > 1)
		if ( magneticFieldDeviation > deviationCoefficientFlight) {			
			P04 = 1;
			Timer0_Delay1ms(3000);
			main_operating_mode = 0;
			elapsedSeconds = 0;
			elapsedMinutes = 0;	
			break;
		} 
		
		xyz0 = x + y + z;
		
		Timer0_Delay1ms(50);
	}

	if(main_operating_mode) main_operating_mode = 2;
}

void combatMode() {
	int16_t magneticFieldDeviation;
	uint8_t i = 0;
	
	P12 = 0; 
	P03 = 0;
	P04 = 0;
	
	while (1) {		

		QMC5883_ReadData();
	
		magneticFieldDeviation = magneticFieldDeviationFanc();
		
		printf("combat Deviation: %d\r\n", magneticFieldDeviation);
		
		if(i > 10) {
			if ( magneticFieldDeviation >= deviationCoefficientInit) {
				P03 = 1;
				Timer0_Delay1ms(DELAY_AFTER_ACTIVATION);
				i = 0;
				P12 = 1;
				Timer0_Delay1ms(3000);
			}
		} 
		
				
		if (daysSelfDestruction <= elapsedDays) {
			P12 = 1;
			P03 = 1;
			Timer0_Delay1ms(3000);
			//break;
		}
	
		
			
		P12 = 0;	
		P03 = 0;	
		
		xyz0 = x + y + z;
		i++;
		Timer0_Delay1ms(50);
	}
}

void setup(void) {
	
	InitialUART0_Timer3(9600);
	TI = 1;
	
	Timer0_Init();
  
	I2C_Init();
  QMC5883_Init();
	
	
	P04_Quasi_Mode;
	P03_Quasi_Mode;
	
	P12_Quasi_Mode;
	
	timerCount = 0;
	
	// Init 
	P12 = 0; // yellow led !!!!
	P03 = 0; // red led
	P04 = 1; // green led
	
}

void main(void) {
	
	setup();
    
	while (1) {
			switch (main_operating_mode) {
					
				case 0: // підготовка 
					QMC5883_ReadData();
					xyz0 = x + y + z;
					if (elapsedSeconds == 3) {
						main_operating_mode = 1;		
					}
					break;
						
				case 1: // режим польоту
					movementFlight();	
				break;
							
				case 2: // бойовий рижим
						combatMode();	
				break;
						
			}
    
			Timer0_Delay1ms(1000);
	}
}