#include "conv.h"

char get_digit_char(int value) {
    if(value > 15) return '\0';

    if(value < 10) return '0' + value;

    return 'a' + value - 10;
}

char intToChar(int value) {
	switch(value){
	case 0:
		return '0';
	case 1:
		return '1';
	case 2:
		return '2';
	case 3:
		return '3';
	case 4:
		return '4';
	case 5:
		return '5';
	case 6:
		return '6';
	case 7:
		return '7';
	case 8:
		return '8';
	case 9:
		return '9';
	default:
		return '\0';
	}
	return '\0';
}


char* intToStr(int value, int base) {
	char buf[] = {'0', '0', '0', '0', '0', '0'};
    if(base > 16 || base < 2) {
        buf[0] = 'E';
        buf[1] = '\0';
        return;
    }

    int power = 1;

    while(power * base < value) {
        power = power * base;
    }

    int i = 0;


    if(value < 0 && base == 10){
    	buf[i] = '-';
    	i = 1;
    	value *= -1;
    }

    while(1) {
        int digit = value / power;
        buf[i] = intToChar(digit);
        i++;

        value = value - digit * power;

        if(power <= 1) break;

        power = power / base;
    }
    buf[i] = '\0';
    return *buf;
}

void int_to_str(int value, char* buf, int base) {
    if(base > 16 || base < 2) {
        buf[0] = 'E';
        buf[1] = '\0';
        return;
    }

    int power = 1;

    while(power * base < value) {
        power = power * base;
    }

    int i = 0;

    if(value < 0 && base == 10){
    	buf[i] = '-';
    	//value = value * -1;
    	i = 1;
    }

    while(1) {
        int digit = value / power;
        buf[i] = get_digit_char(digit);
        i++;

        value = value - digit * power;

        if(power <= 1) break;

        power = power / base;
    }

    buf[i] = '\0';
    return;
}
