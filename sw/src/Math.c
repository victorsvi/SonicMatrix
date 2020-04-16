
#include "Math.h"

/**
 * Performs the integer square root (truncated). Uses an iterative method
 * @param x Number to take the sqare root.
 * @return The square root of x.
 */
uint32_t transd_sqrt_int (const uint32_t x) {

    uint32_t l = 1; // the lowest guess number that can be the root of x
    uint32_t h = x; // the highest guess number that can be the root of x
    uint32_t n = (l + h)/2; //the current guess being tested

    while ( l < n ){ //the stop condition is when the lowest possible guess is equal or greater the hisghest guess.

        if( (n * n) <= x ){ //if the guess is too small, updates increases the lowest guess
            l = n;
        }
        else if ( (n * n) > x ) { //if the guess is too big, updates decreases the hisghest guess
            h = n;
        }
        n = (l + h)/2; //the new guess is a midpoint

    };

    return l;
}
