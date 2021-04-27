# TRANSDUCER ARRAY BOARD

This custom pcb is used to hold the transducers and to amplify the microcontroller output.
The board was designed with Eagle.

it was designed to be etched at home with photoresist film, so there were some restrictions to the routing:
- The board can have a maximum of 2 layers (top and bottom)
- Vias are just a hole with a manually soldered jumper from top to bottom. They need to be the size of at least a trough hole component pad. For this reason, vias were used just for the power and ground rails.
- As vias can't be used for signals, the transducers will be placed and soldered from the top.
- 20mil is the limit for trace width and spacing between signals.

If you will etch with tonner transfer, the trace width might be too small.
If you will use a pcb manufacturing service, I advise to remake the design without the above restrictions. In this case, place the arduino and the power mosfets on one side and the transducers on the other side as it will be easier to replace some bad part. Also put series resistors between the arduino pin and the power mosfet input to protect the microcontroller in case the mosfet goes short.