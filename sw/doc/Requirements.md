# SOFTWARE REQUIREMENTS

- Run on Arduino Mega
- Generate a 40kHz signal on 64 independent channels with phase resolution of at least pi/4, which leads to a output update frequecy of 320kHz or interval of 3.125ns.
- The transducers can be hooked up to random output pins. There must be a software mapping from the transducer x and y position to the output pin its connected to. The mapping can be statically set.
- The transducers may have loose tolerances in phase delay. To accommodate that, the software must compensate for the differences in phase delay of each transducer. The phase delay will be measured manually and can be statically set.
- The control of the device will be done by commands received through the usb serial port of the Arduino Mega.
- The software must implement a function to calculate the phase and amplitude of each transducer to focus the ultrasonic radiation at a given cartesian coordinate.
- The software must implement at least a linear trajectory of constant speed to move the focus point from the current position to a given cartesian coordinate at a given speed.
		
### FUTURE UPGRADES

- Implement a calibration routine to find out the pin mapping and save it to the EEPROM (the user connects a extra transducer as a microphone and move it over each transducer in a specific order. The software pulses each transducer until it finds the echo of the pulse in the microphone and uses that information to assign the mapping).
- Implement a calibration routine to calculate the phase compensation and save it to the EEPROM (uses the same method for the mapping calibration).
- Use G-code to specify trajectories.
- Use a joystick to control the focus point position (the phase calculation needs to happen at real time!).