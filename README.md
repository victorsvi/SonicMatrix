# SonicMatrix
Hardware and software of a ultrasonic matrix for particle manipulation

This unoptimized version uses dynamic memory allocation with the Ultrasonic class.
It's useful to create a single pattern for the matrix, but it consumes too much memory when buffering patterns that will be iterated.
