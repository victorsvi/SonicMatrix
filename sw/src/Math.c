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

/**
 * Calculates the y coordinate for a given x coordinate where the (x,y) point is in a line that pass trough (x1,y1) and (x2,y2).
 * The point will always be in the first quadrant (all the inputs are unsigned)
 * @param x The x coordinate of the point to be solved
 * @param x1 The x coordinate of the first point that the line pass trough
 * @param y1 The y coordinate of the first point that the line pass trough
 * @param x1 The x coordinate of the second point that the line pass trough
 * @param y1 The y coordinate of the second point that the line pass trough
 * @returns The y coordinate of the point
 */
uint8_t line_solve_y (uint8_t x, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2 ) {
	
	int32_t _x, _y, _x1, _y1, _x2, _y2;
	uint8_t y;
	
	//using 2 decimal places fixed point (force cast before multiplication)
	_x  = ((int32_t) x) * 100; 
	_x1 = ((int32_t) x1) * 100; 
	_y1 = ((int32_t) y1) * 100; 
	_x2 = ((int32_t) x2) * 100; 
	_y2 = ((int32_t) y2) * 100;
	
	/*
		Find a coordinate of a line trough two points
		
		from geometric analysis, the slope m for the line that pass trough the points (x1,y1) and (x2,y2) is m = (y2 - y1)/(x2 - x1) (1)
		the same equation can be reorganized to find (x3, y3) given (x1,y1), x3 and m: y3 = m * (x3 - x1) + y1 (2)
		replacing m from (1) in (2) gives y3 = (y2 - y1)/(x2 - x1) * (x3 - x1) + y1 (3)
		the equation bellow is (3) reorganized to decrease the rounding error in the division (we are using integers)
	*/
	_y = ((_y2 - _y1) * (_x - _x1))/(_x2 - _x1) + _y1;
	
	//round for 2 decimal places
	if(_y % 100 >= 50) {
		_y += 50;
	}
	
	//back to integer representation (was using 2 decimal places fixed point)
	_y /= 100;
	
	//the point will always be at the first quadrant (x >= 0 and y >= 0)
	y = ((uint8_t) _y);
	
	return y;
	
} //traj_solve_y
