#include "stm32f4xx_hal.h"
#include "font.h"
#include "main.h"


extern SPI_HandleTypeDef hspi3;

const extern unsigned char labirynth [];


void displayReset(void);
	
void displayCmd(uint8_t cmd);

void displayWrite(uint8_t data);

void displayInit(void);

void displayPrint(char* sign);

void displaySetCursor(uint8_t x, uint8_t y);

void displayClear(void);


void displayClearLine(uint8_t x, uint8_t y);

void displayDrawPixel(uint8_t x, uint8_t y, uint8_t value);

void displayFloat(float number);

void displayDrawLevel(void);



