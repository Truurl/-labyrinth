#include "uart.h"
#include "conv.h"

void uartInit(void){
	__HAL_RCC_GPIOC_CLK_ENABLE();
	RCC->APB1ENR |= (1 << 19);
	//wlaczenie alternatywej funkcji dla pinow 10 i 11
	GPIOC->MODER |= (2 << 22) | (2 << 20);
	GPIOC->AFR[1] |= (8 << 8) | (8 << 12);
	//konfiguracja uarta
	UART4->BRR = 0x1117;
	UART4->CR1 |= USART_CR1_TE | USART_CR1_RE;
	UART4->CR1 |= USART_CR1_UE;
}

void uartPrint(char* data){
	for(uint8_t i = 0;; i++){
		if(data[i] == '\0') break;
		UART4->DR = data[i];
		while (!(UART4->SR & (1 << 7)));
	}
	return;
}

void printInt(int number){
	char numb[] ={'0', '0', '0', '0', '0', '0'};
	uint8_t offset = 0;
	if(number < 0) {numb[0] = '-'; offset = 1; number*=-1;}
	if(number >= 10000){
			numb[offset + 0] = intToChar((number / 10000) % 10);
			numb[offset + 1] = intToChar((number / 1000) % 10);
			numb[offset + 2] = intToChar((number / 100) % 10);
			numb[offset + 3] = intToChar((number / 10) % 10);
			numb[offset + 4] = intToChar((number % 10));
			numb[offset + 5] = '\0';
			uartPrint(numb);
			return;
	}
	if(number >= 1000){
			numb[offset + 0] = intToChar((number / 1000) % 10);
			numb[offset + 1] = intToChar((number / 100) % 10);
			numb[offset + 2] = intToChar((number / 10) % 10);
			numb[offset + 3] = intToChar((number % 10));
			numb[offset + 4] = '\0';
			uartPrint(numb);
			return;
	}
	if(number >= 100){
			numb[offset + 0] = intToChar((number / 100) % 10);
			numb[offset + 1] = intToChar((number / 10) % 10);
			numb[offset + 2] = intToChar((number % 10));
			numb[offset + 3] = '\0';
			uartPrint(numb);
			return;
	}
	if(number >= 10){
			numb[offset + 0] = intToChar((number / 10) % 10);
			numb[offset + 1] = intToChar((number % 10));
			numb[offset + 2] = '\0';
			uartPrint(numb);
			return;
	}
	if(number <= 10){
			numb[offset + 0] = intToChar((number % 10));
			numb[offset + 1] = '\0';
			uartPrint(numb);
			return;
	}

}

void printFloat(float number){
	char numb[] ={'0', '0', '0', '0', '0', '0'};
	uint8_t offset = 0;
	if(number < 0) {numb[0] = '-'; offset = 1; number*=-1;}
	if(number >= 10){
			numb[offset + 0] = intToChar(( ((int)number) / 10 ) % 10 );
			numb[offset + 1] = intToChar(( ((int)number) % 10 ));
			numb[offset + 2] = '.';
			numb[offset + 3] = intToChar( ((int)(number * 10)) % 10 );
			numb[offset + 4] = intToChar( ((int)(number * 100)) % 10 );
			numb[offset + 5] = '\0';
			uartPrint(numb);
			return;
	}
	if(number <= 10){
			numb[offset + 0] = intToChar(( ((int)number) % 10 ));
			numb[offset + 1] = '.';
			numb[offset + 2] = intToChar( ((int)(number * 10)) % 10);
			numb[offset + 3] = intToChar(( ((int)(number * 100)) % 10));
			numb[offset + 4] = '\0';
			uartPrint(numb);
			return;
	}
}
