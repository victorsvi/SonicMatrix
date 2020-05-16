/*
 * This file implements some custom math functions.
 * 
 * by Victor Salvi (victorsvi@gmail.com), 2020.
 */
 
#include "Math.h"

/**
 * Performs the integer square root (truncated). Uses an iterative method.
 * In applications which the sqrt can be an rough approximation, this method avoid the overhead of using floating point operations.
 * @param x Number to take the square root.
 * @return The square root of x.
 */
uint32_t transd_sqrt_int (const uint32_t x) {

    uint32_t l = 1; // the lowest guess number that can be the root of x
    uint32_t h = x; // the highest guess number that can be the root of x
    uint32_t n = (l + h)/2; //the current guess being tested

    while ( l < n ){ //the stop condition is when the lowest possible guess is equal or greater the current guess.

        if( (n * n) <= x ){ //if the guess is too small, increases the lowest guess
            l = n;
        }
        else if ( (n * n) > x ) { //if the guess is too big, decreases the highest guess
            h = n;
        }
        n = (l + h)/2; //the new guess is the midpoint

    };

    return l;
}
