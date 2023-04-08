int main(void){
#include <stdio.h>
#include <math.h>
unsigned char q;
short v = 0x1234;

q = v >> 8;
printf("MSB = %X\n", q);

q = v & 0xFF;
printf("LSB = %X\n", q);

return 0;
}
