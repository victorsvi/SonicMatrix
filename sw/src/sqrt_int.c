#include <stdio.h>
#include <stdint.h>

unsigned int sqrt_int (unsigned int x) {

    unsigned int l = 1;
    unsigned int h = x;
    unsigned int n = (l + h)/2;

    //int cnt = 0;

    while ( l < n ){

        if( (n * n) <= x ){
            l = n;
        }
        else if ( (n * n) > x ) {
            h = n;
        }
        n = (l + h)/2;

        cnt++;
    };

    //printf("%d \t",cnt);
    return l;
}


int main () {

    uint8_t p;

    p = (uint8_t) (( 4 * 256) / 8);
    printf("%u\r\n", p);

    p = (uint8_t) (( 8 * 256) / 8);
    printf("%u\r\n", p);

    p = (uint8_t) (( 12 * 256) / 8);
    printf("%u\r\n", p);

    p = (uint8_t) (( 16 * 256) / 8);
    printf("%u\r\n", p);

    p = (uint8_t) (( 200 * 256) / 8);
    printf("%u\r\n", p);

    p = (uint8_t) (( 400 * 256) / 8);
    printf("%u\r\n\r\n", p);

    printf("%u\r\n", sqrt_int(4));
    printf("%u\r\n", sqrt_int(10));
    printf("%u\r\n", sqrt_int(16));
    printf("%u\r\n", sqrt_int(17));
    printf("%u\r\n", sqrt_int(24));
    printf("%u\r\n", sqrt_int(25));
    printf("%u\r\n", sqrt_int(26));
    printf("%u\r\n", sqrt_int(200));
    printf("%u\r\n", sqrt_int(400));
    printf("%u\r\n", sqrt_int(89));
    printf("%u\r\n", sqrt_int(5));
    printf("%u\r\n", sqrt_int(6));
    printf("%u\r\n", sqrt_int(7));
    printf("%u\r\n", sqrt_int(8));
    printf("%u\r\n", sqrt_int(9));
    printf("%u\r\n", sqrt_int(1));
    printf("%u\r\n", sqrt_int(0));
}
