
/*
	IMPORTANTE! Calcular o foco em tempo real é muito lento (cerca de 18ms por cálculo)
	
	No main, usar a array_calc_focus e copiar as patterns para outro array. Esse outro array deve
	armazenar vários "passos", de forma que o conjunto dos passos forme uma tragetória.
	
	O destino é informado pela serial, o arduino (que sabe a origem) vai fazer uma linha reta até o destino, colocando todos os "passos" nesse array antes de iniciar (deve demorar cerca de 180ms para 100 passos).
*/

/*
	UPGRADES
	
	TRAJ: Ao invés de indexar o movimento por passo em um eixo de referência, indexar pelo tempo ou por unidade de distância na reta do deslocamento.
	TRAJ: Se a trajetória tiver mais que TRAJ_MAXSTEPS, utilizar um valor maior que TRAJ_RES entre os steps.
	Decidir qual licença atribuir ao projeto e documentar ns arquivos
*/

/*
	BUGS
	
	Se a velocidade for muito lenta no eixo, pode dar overflow no timer :o
	Se o deslocamento tiver mais de 64 milimetros em um eixo, o movimento é cortado.
*/

/* 
 * PARTICLE MANIPULATION USING ULTRASONIC MATRIX
 *
 * This code implements a driver board for a ultrasonic matrix capable
 * of generating a focal point by individually adjusting the phase of
 * each transducer of the matrix in order to make the waves arrive in
 * phase at the focal point, creating a constructive interference and 
 * increasing sound pressure.
 * 
 * The board is capable of moving the focal point along a coordinate
 * frame.
 *
 * The interface consists of a serial input and defined commands (see
 * input_parse and input_parse_token documentation for more info).
 *
 * It's made to run on the Arduino Mega and the expected hardware interface
 * between the Arduino and the actual transducers are half bridge drivers,
 * one for each channel. The mapping between each transducer and the 
 * corresponding Arduino pin is defined in software.
 *
 * It also supports phase compensation for each transducer, so the response
 * delay of each element can be measured and set in software to compensate
 * for low phase accuracy of cheap transducers.
 *
 * This version controls a 8x8 matrix but the code can be easily changed
 * to other sizes. Just be careful to not run out of dynamic memory by
 * increasing the output buffer size.
 *
 * by Victor Salvi (victorsvi@gmail.com), 2020.
 */
 
#include <stdint.h>
#include <Arduino.h>
#include "Ultrasonic.h"
#include "Debug.h"
#include "Timer4.h"
#include "Timer3.h"
#include "Math.h"

/* DEFINES */

/* Toggle these defines to enable debug routines */
//#define DEBUG_PINS 2000 //Use to debug the pin assignment using the port registers. Will set each pin high (starting  from pin 6) sequentially. The value defined is the delay between pins in microseconds
//#define DEBUG_MAP 2000 //Use to debug the pin mapping of each element of the matrix. Will set the pin for each transducer high in the matrix order {(0,0),(0,1),(0,2),(1,0),(1,1),(1,2),...}. The value defined is the delay between pins in microseconds
#define DEBUG_INPUT //Use to debug the serial input parsing. Will echo the interpretation of the inputed string
//#define DEBUG_PATTERN //Use to debug the pattern generation routine. Will serial out the pattern for each transducer of the matrix when the pattern is calculated.
//#define DEBUG_TRAJ //Use to debug the trajectory planning routine. Will serial out the trajectory way points and the speed calculation data.
#define DEBUG_TIMER 0x000F //Use to debug the transducer signal generation (waveform, frequency accuracy, jitter). Will configure the matrix to output the defined pattern on all transducers. Note that the pattern generation will be overridden and the phase compensation will be disabled. The value defined is a binary unsigned representing the pattern to be outputted (0xAAAA = #_#_#_#_#_#_#_#_)

#define TRAJ_RES 1 //trajectory maximum resolution in millimeters (not fully implemented)
#define TRAJ_MAXSTEPS 64 //maximum steps of the trajectory (max 255). 

/* MACROS */

#define MSK(b) (1 << b) //creates a mask with the bth bit high

/* PROTOTYPES */

ISR( TIMER4_COMPA_vect );
ISR( TIMER3_COMPA_vect );
void transd_array_load ( /*t_transd_array *transd_array*/ );
void output_reset ();
uint8_t traj_calc (const uint8_t from_x, const uint8_t from_y, const uint8_t from_z, const uint8_t to_x, const uint8_t to_y, const uint8_t to_z );
void traj_calc_step (uint8_t step_idx, const uint8_t duty_cycle, const uint8_t focus_x, const uint8_t focus_y, const uint8_t focus_z );
//uint8_t traj_solve_y (uint8_t x, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2 );
//uint8_t traj_solve_z (uint8_t x, uint8_t x1, uint8_t z1, uint8_t x2, uint8_t z2 );
uint32_t traj_calc_speed (const uint8_t s, const uint8_t from_x, const uint8_t from_y, const uint8_t from_z, const uint8_t to_x, const uint8_t to_y, const uint8_t to_z );
uint8_t input_parse ();
void input_parse_token (char *token);
void input_execute ();
#ifdef DEBUG_PINS
void debug_pins ();
#endif
#ifdef DEBUG_MAP
void debug_map ();
#endif
#ifdef DEBUG_INPUT
void debug_input ();
#endif
#ifdef DEBUG_PATTERN
void debug_pattern ();
#endif
#ifdef DEBUG_TRAJ
void debug_traj_header ();
void debug_traj_step (const uint8_t step_idx, const uint8_t focus_x, const uint8_t focus_y, const uint8_t focus_z);
void debug_traj_speed (const uint8_t s, const uint8_t from_x, const uint8_t from_y, const uint8_t from_z, const uint8_t to_x, const uint8_t to_y, const uint8_t to_z, const int8_t Dx, const int8_t Dy, const int8_t Dz, const uint32_t distance, const uint32_t speed_axys, const uint32_t interval);
#endif
#ifdef DEBUG_TIMER
void debug_timer ();
#endif

/* DATA DEFINITION */

enum e_mode { // lists the operation modes
	MODE_OFF,		// turned off, signal is held low
	MODE_ON, 		// turned on, with static focal point
	MODE_MOVE_OFF, 	// move the focal point from current position to target position then tuns off
	MODE_MOVE_ON, 	// move the focal point from current position to target position and keeps on
	MODE_FLAT		// turned on, flat mode, all transducers in phase (phase compensation applied)
};

struct s_traj_ctrl { // position and other parameters
	uint8_t x = 0;
	uint8_t y = 0;
	uint8_t z = 0;
	uint8_t d = 128;
	uint8_t s = 10; // represent the current focus coordinates (in millimeters), duty cycle (discrete representation, [0,255] = [0°,360°]) and speed (in millimeters per second).
	uint8_t lastx = 0;
	uint8_t lasty = 0;
	uint8_t lastz = 0; // represent the focus coordinates before the new command (or last position) (in millimeters)
};

struct s_pin { // represents a pin mask
	volatile uint8_t *bank_ptr; // pin's bank address
	uint8_t bit_msk; // pin's bit mask for the bank
};

/* GLOBAL VARIABLES */

const uint8_t ARRAY_CALIBRATION[ARRAY_SIZE_X][ARRAY_SIZE_Y][2] PROGMEM = { // maps the pin of each transducer of the matrix (Arduino pin) and it's phase compensation value
	//pin numer (arduino convention), phase compensation ([0,255] = [0°,360°])
	//x0, y0,
	//x0, y1,
	//0 , 0,
	//1 , 0,
	//2 , 0,
	//3 , 0,
	//4 , 0,
	//5 , 0, //this matrix uses pins from 6 to 69 (optimized to use the least banks possible)
	6 , 0,
	7 , 0,
	8 , 0,
	9 , 0,
	10, 0,
	11, 0,
	12, 0,
	13, 0,
	14, 0,
	15, 0,
	16, 0,
	17, 0,
	18, 0,
	19, 0,
	20, 0,
	21, 0,
	22, 0,
	23, 0,
	24, 0,
	25, 0,
	26, 0,
	27, 0,
	28, 0,
	29, 0,
	30, 0,
	31, 0,
	32, 0,
	33, 0,
	34, 0,
	35, 0,
	36, 0,
	37, 0,
	38, 0,
	39, 0,
	40, 0,
	41, 0,
	42, 0,
	43, 0,
	44, 0,
	45, 0,
	46, 0,
	47, 0,
	48, 0,
	49, 0,
	50, 0,
	51, 0,
	52, 0,
	53, 0,
	54, 0,
	55, 0,
	56, 0,
	57, 0,
	58, 0,
	59, 0,
	60, 0,
	61, 0,
	62, 0,
	63, 0,
	64, 0,
	65, 0,
	66, 0,
	67, 0,
	68, 0,
	69, 0
};

const struct s_pin PINS[70] = { // Port bank and bit mask for each Arduino Mega pin indexed by pin number
	{&PORTE, MSK(0)},
	{&PORTE, MSK(1)},
	{&PORTE, MSK(4)},
	{&PORTE, MSK(5)},
	{&PORTG, MSK(5)},
	{&PORTE, MSK(3)},
	{&PORTH, MSK(3)},
	{&PORTH, MSK(4)},
	{&PORTH, MSK(5)},
	{&PORTH, MSK(6)},
	{&PORTB, MSK(4)},
	{&PORTB, MSK(5)},
	{&PORTB, MSK(6)},
	{&PORTB, MSK(7)},
	{&PORTJ, MSK(1)},
	{&PORTJ, MSK(0)},
	{&PORTH, MSK(1)},
	{&PORTH, MSK(0)},
	{&PORTD, MSK(3)},
	{&PORTD, MSK(2)},
	{&PORTD, MSK(1)},
	{&PORTD, MSK(0)},
	{&PORTA, MSK(0)},
	{&PORTA, MSK(1)},
	{&PORTA, MSK(2)},
	{&PORTA, MSK(3)},
	{&PORTA, MSK(4)},
	{&PORTA, MSK(5)},
	{&PORTA, MSK(6)},
	{&PORTA, MSK(7)},
	{&PORTC, MSK(7)},
	{&PORTC, MSK(6)},
	{&PORTC, MSK(5)},
	{&PORTC, MSK(4)},
	{&PORTC, MSK(3)},
	{&PORTC, MSK(2)},
	{&PORTC, MSK(1)},
	{&PORTC, MSK(0)},
	{&PORTD, MSK(7)},
	{&PORTG, MSK(2)},
	{&PORTG, MSK(1)},
	{&PORTG, MSK(0)},
	{&PORTL, MSK(7)},
	{&PORTL, MSK(6)},
	{&PORTL, MSK(5)},
	{&PORTL, MSK(4)},
	{&PORTL, MSK(3)},
	{&PORTL, MSK(2)},
	{&PORTL, MSK(1)},
	{&PORTL, MSK(0)},
	{&PORTB, MSK(3)},
	{&PORTB, MSK(2)},
	{&PORTB, MSK(1)},
	{&PORTB, MSK(0)},
	{&PORTF, MSK(0)},
	{&PORTF, MSK(1)},
	{&PORTF, MSK(2)},
	{&PORTF, MSK(3)},
	{&PORTF, MSK(4)},
	{&PORTF, MSK(5)},
	{&PORTF, MSK(6)},
	{&PORTF, MSK(7)},
	{&PORTK, MSK(0)},
	{&PORTK, MSK(1)},
	{&PORTK, MSK(2)},
	{&PORTK, MSK(3)},
	{&PORTK, MSK(4)},
	{&PORTK, MSK(5)},
	{&PORTK, MSK(6)},
	{&PORTK, MSK(7)}
};

struct s_traj_ctrl traj_ctrl; // position and other parameters
volatile uint8_t traj_step_idx = 0, traj_step_num = 0; // current step and number of steps of the trajectory
volatile uint8_t array_phase_idx = 0; // current phase index of the signal (which bit of the pattern is being outputted)
enum e_mode mode; // mode of operation

/*t_transd_array *transd_array = NULL;*/

uint8_t traj_port_buffer[TRAJ_MAXSTEPS][ARRAY_PHASERES][10]; //buffers the ports state for each coordinate (x,y,z) of the movement, for each slice of the wave period, for each PORT (this project uses 10 ports)

volatile uint8_t *port_buffer_step_addr = &traj_port_buffer[0][0][0]; //holds the address for the section of the traj_port_buffer vector for the current traj_step_idx (must be assigned with &traj_port_buffer[traj_step_idx][0][0] when traj_step_idx is changed)

void setup () {
  
//	TIMSK0 = 0; 
//	TIMSK1 = 0; 
//	TIMSK2 = 0; 
//	TIMSK3 = 0; 
//	TIMSK4 = 0; 
//	TIMSK5 = 0; 
	
	Serial.begin(115200);
	
	//set ports as output (bit high = output)
	DDRA = 0xFF; //11111111
	DDRB = 0xFF; //11111111
	DDRC = 0xFF; //11111111
	DDRD = 0xFF;// |= 0x8F; //1xxx1111
	//DDRE = 0xFF;
	DDRF = 0xFF; //11111111
	DDRG = 0xFF;// |= 0x07; //xxxxx111
	DDRH = 0xFF;// |= 0x3F; //xx111111
	//DDRI = 0xFF;
	DDRJ = 0xFF;// |= 0x03; //xxxxxx11
	DDRK = 0xFF; //11111111
	DDRL = 0xFF; //11111111
  
	output_reset (); //initialize all pins as LOW
	
	#ifdef DEBUG_PINS
	debug_pins ();
	#endif
	
	setTimer4(); //programs the timer 4 (signal generation)
	
	/*transd_array = transd_array_init( ARRAY_SIZE_X, ARRAY_SIZE_Y, TRANS_DIAMETER, TRANS_SEPARATION, ARRAY_PHASERES );
	if( transd_array == NULL ) {
		#ifdef DEBUG
		DEBUG_MSG("The transducer array pointer is null")
		#endif
	}*/
	transd_array_load (/*transd_array*/); // loads the pin number and phase compensation to the matrix

	#ifdef DEBUG_MAP
	debug_map ();
	#endif

	mode = MODE_OFF; //starts with output disabled
	
	#ifdef DEBUG_TIMER
	debug_timer ();
	#endif
} //setup

void loop () {
	
	//it accepts serial data even when the output is active, but the parsing and executing of the commands will be slower as the timer 4 will generate a lot of interrupts.
	//it's recommended to send a "i" command before sending commands while the output is active;
	if(Serial.available()) { // if there's input data on serial line
		input_parse();
		#ifdef DEBUG_INPUT
		debug_input();
		#endif
		input_execute(); //execute the parsed parameters
	}

///*
	asm volatile (

//	"mov	r17,			%A[vecAddr]		\n"
//	"mov	r18,			%B[vecAddr]		\n"
//	
//	"1:								\n" //loads 10
//	"mov	%A[vecAddr],	r17				\n"
//	"mov	%B[vecAddr],	r18				\n"

	"mul	%[phIdx],		%[C10]			\n" //array_phase_idx * 10 (into r0, r1)
	"add	%A[vecAddr],	r0				\n" //address = &traj_port_buffer[traj_step_idx][array_phase_idx][0] (&traj_port_buffer[traj_step_idx] + array_phase_idx * 10)
	"adc	%B[vecAddr],	r1				\n" 

	"ld		r16,			%a[vecAddr]+	\n" //PORTA
	"out	0x02,			r16				\n" //PORTA
	"ld		r16,			%a[vecAddr]+	\n" //PORTB
	"out	0x05,			r16				\n" //PORTB
	"ld		r16,			%a[vecAddr]+	\n" //PORTC
	"out	0x08,			r16				\n" //PORTC
	"ld		r16,			%a[vecAddr]+	\n" //PORTD
	"out	0x0B,			r16				\n" //PORTD
	"ld		r16,			%a[vecAddr]+	\n" //PORTF
	"out	0x11,			r16				\n" //PORTF
	"ld		r16,			%a[vecAddr]+	\n" //PORTG
	"out	0x14,			r16				\n" //PORTG
	"ld		r16,			%a[vecAddr]+	\n" //PORTH
	"sts	0x102,			r16				\n" //PORTH
	"ld		r16,			%a[vecAddr]+	\n" //PORTJ
	"sts	0x105,			r16				\n" //PORTJ
	"ld		r16,			%a[vecAddr]+	\n" //PORTK
	"sts	0x109,			r16				\n" //PORTK
	"ld		r16,			%a[vecAddr]+	\n" //PORTL
	"sts	0x10B,			r16				\n" //PORTL

	"subi	%[phIdx],		-1				\n" //increment array_phase_idx
	"andi	%[phIdx],		0x07			\n" //masks only the frist 3 bits, to wrap the count from 0 to 7
	//"sts	(array_phase_idx),	r23	\n"
	
//	"jmp	1b	\n"
	
	: [phIdx] "+&d" (array_phase_idx) //2 cycles lds + 2 cycles sts //d = upper registers (r16 - r31)
	: [vecAddr] "e" (port_buffer_step_addr) //2 cycles? //e = pointer registers (x, y ,z)
	 ,[C10] "r" (10) //1 cycle ldi
	: "r16", "r0", "r1"
); 
//*/




/*
	//buffer the indexes to optimize access as they're volatile
	uint8_t phase_idx = array_phase_idx;
	uint8_t step_idx = traj_step_idx;
  
	// Just copy the port state from the buffer
	PORTA = traj_port_buffer[step_idx][phase_idx][0];
	PORTB = traj_port_buffer[step_idx][phase_idx][1];
	PORTC = traj_port_buffer[step_idx][phase_idx][2];
	PORTD = traj_port_buffer[step_idx][phase_idx][3];
	//PORTE = 0;
	PORTF = traj_port_buffer[step_idx][phase_idx][4];
	PORTG = traj_port_buffer[step_idx][phase_idx][5];
	PORTH = traj_port_buffer[step_idx][phase_idx][6];
	//PORTI = 0;
	PORTJ = traj_port_buffer[step_idx][phase_idx][7];
	PORTK = traj_port_buffer[step_idx][phase_idx][8];
	PORTL = traj_port_buffer[step_idx][phase_idx][9];
	
	array_phase_idx++;
	array_phase_idx &= 0x07;

//	if( array_phase_idx < ARRAY_PHASERES ) { //increments the current phase index. It will cycle between 0 and (ARRAY_PHASERES - 1)
//		array_phase_idx++;
//	}
//	else {
//		array_phase_idx = 0; 
//	}
*/
} //loop

/**
 * This function will be called every interrupt of timer 4.
 * It will update the port registers by cycle trough the buffered port registers for each step of the wave (one period is divided in many slices to allow phase control).
 * The active/inactive control is made by enabling or disabling the timer interrupts.
 */
ISR( TIMER4_COMPA_vect ) {
/*
asm volatile (

	//"lds	X,	(port_buffer_step_addr)	\n"
	//"lds	r23,	(array_phase_idx)	\n"
	//"ldi	r16,			%[C10]			\n" //loads 10
	"mul	%[phIdx],		%[C10]			\n" //array_phase_idx * 10 (into r0, r1)
	"add	%A[vecAddr],	r0				\n" //address = &traj_port_buffer[traj_step_idx][array_phase_idx][0] (&traj_port_buffer[traj_step_idx] + array_phase_idx * 10)
	"adc	%B[vecAddr],	r1				\n" 

	//9 cycles

	"ld		r16,			%a[vecAddr]+	\n" //PORTA
	"out	0x02,			r16				\n" //PORTA
	"ld		r16,			%a[vecAddr]+	\n" //PORTB
	"out	0x05,			r16				\n" //PORTB
	"ld		r16,			%a[vecAddr]+	\n" //PORTC
	"out	0x08,			r16				\n" //PORTC
	"ld		r16,			%a[vecAddr]+	\n" //PORTD
	"out	0x0B,			r16				\n" //PORTD
	"ld		r16,			%a[vecAddr]+	\n" //PORTF
	"out	0x11,			r16				\n" //PORTF
	"ld		r16,			%a[vecAddr]+	\n" //PORTG
	"out	0x14,			r16				\n" //PORTG
	"ld		r16,			%a[vecAddr]+	\n" //PORTH
	"sts	0x102,			r16				\n" //PORTH
	"ld		r16,			%a[vecAddr]+	\n" //PORTJ
	"sts	0x105,			r16				\n" //PORTJ
	"ld		r16,			%a[vecAddr]+	\n" //PORTK
	"sts	0x109,			r16				\n" //PORTK
	"ld		r16,			%a[vecAddr]+	\n" //PORTL
	"sts	0x10B,			r16				\n" //PORTL

	//9 + 34 cycles

	"subi	%[phIdx],		-1				\n" //increment array_phase_idx
	"andi	%[phIdx],		0x09			\n" //masks only the frist 3 bits, to wrap the count from 0 to 7
	//"sts	(array_phase_idx),	r23	\n"
	
	//9 + 34 + 4
	
	//47 cycles?
	
	: [phIdx] "+&d" (array_phase_idx) //2 cycles lds + 2 cycles sts //d = upper registers (r16 - r31)
	: [vecAddr] "e" (port_buffer_step_addr) //2 cycles? //e = pointer registers (x, y ,z)
	 ,[C10] "M" (10) //1 cycle ldi
	: "r16", "r0", "r1"
); */
/*
	//"lds	X,	(port_buffer_step_addr)	\n"
	//"lds	r23,	(array_phase_idx)	\n"
	//"ldi	r16,	0	\n"
	"add	%A[vecAddr],	%[phIdx]	\n" //address = &traj_port_buffer[traj_step_idx][array_phase_idx][0]
	"adc	%B[vecAddr],	r16			\n" 

	"subi	%[phIdx],	-10	\n"
	"cpi	%[phIdx],	80	\n" //00 - 70
	"brne	1f				\n" 
	"ldi	%[phIdx],	0	\n" //reset	
	"1:						\n"
*/



//	uint8_t *ptr;
//	ptr = &traj_port_buffer[traj_step_idx][array_phase_idx][0];
//  PORTA = *ptr; ptr++;
//  PORTB = *ptr; ptr++;
//  PORTC = *ptr; ptr++;
//  PORTD = *ptr; ptr++;
//  PORTF = *ptr; ptr++;
//  PORTG = *ptr; ptr++;
//  PORTH = *ptr; ptr++;
//  PORTJ = *ptr; ptr++;
//  PORTK = *ptr; ptr++;
//  PORTL = *ptr; //ptr++;
/*
  asm("
    //lds r20, traj_step_idx
    //lds r21, array_phase_idx
    //ponteiro buffer
    //lds X, &traj_port_buffer;
    //copia do buffer pra porta
    lds 0x02, X PORTA
    inc X
    ..

    //atualiza phaseidx
    lds r21, array_phase_idx //carrega
    inc r21 //incrementa
    ori r21, 0x09 //loop 0-9 reseta bit 4
    sts array_phase_idx, r21
    ..
    " : "x" (&traj_port_buffer[traj_step_idx][array_phase_idx][0]) : "" : "r21");
*/
/*
	//buffer the indexes to optimize access as they're volatile
	uint8_t phase_idx = array_phase_idx;
	uint8_t step_idx = traj_step_idx;
  
	// Just copy the port state from the buffer
	PORTA = traj_port_buffer[step_idx][phase_idx][0];
	PORTB = traj_port_buffer[step_idx][phase_idx][1];
	PORTC = traj_port_buffer[step_idx][phase_idx][2];
	PORTD = traj_port_buffer[step_idx][phase_idx][3];
	//PORTE = 0;
	PORTF = traj_port_buffer[step_idx][phase_idx][4];
	PORTG = traj_port_buffer[step_idx][phase_idx][5];
	PORTH = traj_port_buffer[step_idx][phase_idx][6];
	//PORTI = 0;
	PORTJ = traj_port_buffer[step_idx][phase_idx][7];
	PORTK = traj_port_buffer[step_idx][phase_idx][8];
	PORTL = traj_port_buffer[step_idx][phase_idx][9];
	*/
	
  //if( phase_idx < ARRAY_PHASERES ) { //increments the current phase index. It will cycle between 0 and (ARRAY_PHASERES - 1)
//	if( array_phase_idx < ARRAY_PHASERES ) { //increments the current phase index. It will cycle between 0 and (ARRAY_PHASERES - 1)
//		array_phase_idx++;
//	}
//	else {
//		array_phase_idx = 0; 
//	}
//array_phase_idx++;
//array_phase_idx &= 0x09;

	//array_phase_idx = (array_phase_idx + 1) & 0x09;


 /*
0000
0001
0010
0011
0100
0101
0110
0111
se for 8 bits, pode só incrementar a apagar o bit da esquerda
1000
1001

1010
1011

*/
} //ISR T4

/**
 * This function will be called every interrupt of timer 3.
 * It will increment the current step index, which will be read at timer 4 interrupt.
 * As the current step is incremented at every interrupt, the period of this timer will define the speed of the movement.
 * The active/inactive control is made by enabling or disabling the timer interrupts.
 */
ISR( TIMER3_COMPA_vect ) {
  
	#ifdef DEBUG_TRAJ
	Serial.print(F("Step "));//Serial.print(traj_step_idx);//Serial.print(F(" of "));//Serial.println(traj_step_num - 1);
	#endif

	if(traj_step_idx < (traj_step_num - 1) ) { //increments the step until reaches the last step
		traj_step_idx++;
		port_buffer_step_addr = &traj_port_buffer[traj_step_idx][0][0];
	}
	else { //then stay in the last step and disables the timer
		disableTimer3 ();
		#ifdef DEBUG_TRAJ
		Serial.println(F("Destiny reached"));
		#endif

		if(mode == MODE_MOVE_OFF) { // disables the output if the mode demands it
			disableTimer4 ();
			output_reset ();
		}
	}

} //ISR T3

/**
 * Calculate the way points of the trajectory and fills the output buffer with the patterns for each way point.
 * The traj_calc_speed function is tied to the same methodology used in this function. 
 * @param from_x The initial coordinate on the x axis (in millimeters)
 * @param from_y The initial coordinate on the y axis (in millimeters)
 * @param from_z The initial coordinate on the z axis (in millimeters)
 * @param to_x The final coordinate on the x axis (in millimeters)
 * @param to_y The final coordinate on the y axis (in millimeters)
 * @param to_z The final coordinate on the z axis (in millimeters)
 * @returns The number of way points created
 */
uint8_t traj_calc (const uint8_t from_x, const uint8_t from_y, const uint8_t from_z, const uint8_t to_x, const uint8_t to_y, const uint8_t to_z ) {
	
	uint8_t traj_x, traj_y, traj_z, step_idx;

	step_idx = 0;
	
	#ifdef DEBUG_TRAJ
	debug_traj_header ();
	#endif
	
	// The trajectory is indexed to an axis. The way points are integer coordinates on the indexed axis.
	// choose the first axis not perpendicular to the trajectory as the indexing axis. The order of the search is x,y,z.
	if( from_x != to_x ) { /* if the trajectory isn't perpendicular to the x axis */
		
		traj_x = from_x;
		
		for(; (from_x < to_x) ? (traj_x <= to_x) : (traj_x >= to_x); (from_x < to_x) ? (traj_x += TRAJ_RES) : (traj_x -= TRAJ_RES) ) { //iterate trough the displacement on the x axis in TRAJ_RES sized steps
			//calculates the rounded value for the other axis
			traj_y = line_solve_y (traj_x, to_x, to_y, from_x, from_y ); 
			traj_z = line_solve_y (traj_x, to_x, to_z, from_x, from_z ); //the 3D coordinates can be calculated as two independent 2D lines
			traj_calc_step (step_idx, traj_ctrl.d, traj_x, traj_y, traj_z ); //generates the patterns for the way point
			step_idx++;
			if(step_idx >= TRAJ_MAXSTEPS) { // limits the maximum number of way points to not overflow the output buffer
				break;
			}
		}
	}
	else if( from_y != to_y ) { /* if the trajectory is perpendicular to the x axis but isn't perpendicular to the y axis */
		
		traj_x = from_x;
		traj_y = from_y;
		
		for(; (from_y < to_y) ? (traj_y <= to_y) : (traj_y >= to_y); (from_y < to_y) ? (traj_y += TRAJ_RES) : (traj_y -= TRAJ_RES) ) { //iterate trough the displacement on the y axis in TRAJ_RES sized steps
			//calculates the rounded value for the other axis
			traj_z = line_solve_y (traj_y, to_y, to_z, from_y, from_z ); //the 3D coordinates can be calculated as two independent 2D lines
			traj_calc_step (step_idx, traj_ctrl.d, traj_x, traj_y, traj_z ); //generates the patterns for the way point
			step_idx++;
			if(step_idx >= TRAJ_MAXSTEPS) { // limits the maximum number of way points to not overflow the output buffer
				break;
			}
		}
	}
	else if( from_z != to_z ){ /* if the trajectory is perpendicular to the x and y axis */
		
		traj_x = from_x;
		traj_y = from_y;
		traj_z = from_z;
		
		for(; (from_z < to_z) ? (traj_z <= to_z) : (traj_z >= to_z); (from_z < to_z) ? (traj_z += TRAJ_RES) : (traj_z -= TRAJ_RES) ) { //iterate trough the displacement on the z axis in TRAJ_RES sized steps
			
			traj_calc_step (step_idx, traj_ctrl.d, traj_x, traj_y, traj_z ); //generates the patterns for the way point
			step_idx++;
			if(step_idx >= TRAJ_MAXSTEPS) { // limits the maximum number of way points to not overflow the output buffer
				break;
			}
		}
	}
	
	return step_idx; //return the number of way points
}

/**
 * Calculates the pattern to focus on one coordinate and fills one position of the output buffer with it.
 * @param step_idx The index of the output buffer that will be assigned
 * @param duty_cycle The discrete duty cycle of the signal ([0,255] = [0°,360°])
 * @param focus_x The focus coordinate on the x axis (in millimeters)
 * @param focus_y The focus coordinate on the y axis (in millimeters)
 * @param focus_z The focus coordinate on the z axis (in millimeters)
 */
void traj_calc_step (uint8_t step_idx, const uint8_t duty_cycle, const uint8_t focus_x, const uint8_t focus_y, const uint8_t focus_z ) {
	
	uint8_t x, y, phase_idx, bit, pin, port_idx = 0;
	
	//set all port registers to LOW
	for(phase_idx = 0; phase_idx < ARRAY_PHASERES; phase_idx++) {
		traj_port_buffer[step_idx][phase_idx][0] = 0x00; //PORTA
		traj_port_buffer[step_idx][phase_idx][1] = 0x00; //PORTB
		traj_port_buffer[step_idx][phase_idx][2] = 0x00; //PORTC
		traj_port_buffer[step_idx][phase_idx][3] = 0x00; //PORTD
		traj_port_buffer[step_idx][phase_idx][4] = 0x00; //PORTF
		traj_port_buffer[step_idx][phase_idx][5] = 0x00; //PORTG
		traj_port_buffer[step_idx][phase_idx][6] = 0x00; //PORTH
		traj_port_buffer[step_idx][phase_idx][7] = 0x00; //PORTJ
		traj_port_buffer[step_idx][phase_idx][8] = 0x00; //PORTK
		traj_port_buffer[step_idx][phase_idx][9] = 0x00; //PORTL
	}
	
	if(mode != MODE_FLAT) {
		transd_array_calcfocus( /*transd_array,*/ duty_cycle, focus_x, focus_y, focus_z );
	}
	else {
		transd_array_calcflat( /*transd_array,*/ duty_cycle );
	}
	
	//the buffer starts with all zeros. As the matrix is iterated, the bits of the port registers are assigned (ored)
	for(x = 0; x < ARRAY_SIZE_X; x++){
		for(y = 0; y < ARRAY_SIZE_Y; y++){
			
			//gets the pin that the transducer is connected to
			pin = transd_array[x][y].port_pin;
			
			//gets the port register for the pin
			if(PINS[pin].bank_ptr == &PORTA) port_idx = 0;
			if(PINS[pin].bank_ptr == &PORTB) port_idx = 1;
			if(PINS[pin].bank_ptr == &PORTC) port_idx = 2;
			if(PINS[pin].bank_ptr == &PORTD) port_idx = 3;
			if(PINS[pin].bank_ptr == &PORTF) port_idx = 4;
			if(PINS[pin].bank_ptr == &PORTG) port_idx = 5;
			if(PINS[pin].bank_ptr == &PORTH) port_idx = 6;
			if(PINS[pin].bank_ptr == &PORTJ) port_idx = 7;
			if(PINS[pin].bank_ptr == &PORTK) port_idx = 8;
			if(PINS[pin].bank_ptr == &PORTL) port_idx = 9;
			
			//updates the pin status of the current trajectory index for each phase index
			for(phase_idx = 0; phase_idx < ARRAY_PHASERES; phase_idx++){
			
				//access the pattern and gets the value for the bit representing the current phase index
				bit = transd_array[x][y].pattern & (1 << phase_idx);
				
				//updates only the current pin and port
				if(bit) {
					traj_port_buffer[step_idx][phase_idx][port_idx] |= PINS[pin].bit_msk; 
				}
				/* no need to set as low because all the bits of the buffer are set to zero first
				else {
					traj_port_buffer[step_idx][phase_idx][port_idx] &= ~PINS[pin].bit_idx;
				}
				*/
			}
		}
	}
	
	#ifdef DEBUG_TRAJ
	debug_traj_step (step_idx, focus_x, focus_y, focus_z);
	#endif
	
	#ifdef DEBUG_PATTERN
	debug_pattern ();
	#endif

} //traj_calc_step

/**
 * Calculates the time interval before switching to the next way point of the trajectory.
 * It's tied to the same methodology used in the traj_calc function. 
 * @param s The speed of the displacement (in millimeters per second)
 * @param from_x The initial coordinate on the x axis (in millimeters)
 * @param from_y The initial coordinate on the y axis (in millimeters)
 * @param from_z The initial coordinate on the z axis (in millimeters)
 * @param to_x The final coordinate on the x axis (in millimeters)
 * @param to_y The final coordinate on the y axis (in millimeters)
 * @param to_z The final coordinate on the z axis (in millimeters)
 * @returns The time interval between way points in microseconds
 */
uint32_t traj_calc_speed (const uint8_t s, const uint8_t from_x, const uint8_t from_y, const uint8_t from_z, const uint8_t to_x, const uint8_t to_y, const uint8_t to_z ){
	
	uint32_t distance, speed_axys, interval;
	int8_t Dx, Dy, Dz;
	
	//this values can be smaller than zero!
	Dx = to_x - from_x;
	Dy = to_y - from_y;
	Dz = to_z - from_z;
	
	distance = (Dx * Dx) + (Dy * Dy) + (Dz * Dz); //squared distance
	//distance *= 100; //multiplies by 10 squared to "keep" one decimal place after the square root.
	distance = transd_sqrt_int(distance); 
	//distance /= 10;
	
	if( from_x != to_x ) { /* if the trajectory isn't perpendicular to the x axis */
		
		speed_axys = (s * Dx) / distance; //solve the equivalent speed on the indexing axis
	}
	else if( from_y != to_y ) { /* if the trajectory is perpendicular to the x axis but isn't perpendicular to the y axis */
		
		speed_axys = (s * Dy) / distance; //solve the equivalent speed on the indexing axis
	}
	else { /* if the trajectory is perpendicular to the x and y axis */
		
		speed_axys = s; //solve the equivalent speed on the indexing axis
	}
	
	if( speed_axys == 0 ) { /* if the speed on the chosen axis is smaller than 1 */
		speed_axys = 1;
	}
  
	//v = d/t
	interval = (TRAJ_RES * 1E6) / speed_axys; //assumes that the distance between each way point on the indexing axis is TRAJ_RES. The 1E6 will transform speed from mm/s to mm/us
	

	#ifdef DEBUG_TRAJ
	debug_traj_speed (s, from_x, from_y, from_z, to_x, to_y, to_z, Dx, Dy, Dz, distance, speed_axys, interval);
	#endif
	
	return interval;
}

/**
 * Checks if there are commands in the serial buffer and break it in tokens to parse them.
 * Each token is separated by a SINGLE space. The valid tokens are described in the input_parse_token function.
 * Examples of valid input commands: "m x10 y20 z30 d127 s10", "r x10 y20 z30 d64 s15", "a x10 y20 z30 d64", "f d127", "i".
 * @returns 1 if a command was read, 0 otherwise
 */
uint8_t input_parse () {
	
	char buffer[64];
	uint8_t buffer_size, start_idx, end_idx;
	/*
	char *buffPtr;
	uint8_t n;
	*/
	
	if(Serial.available()) {
		
		buffer_size = Serial.readBytes(buffer,63);
		buffer[buffer_size] = '\0'; //put null character at the end to treat the buffer as a string
		buffer_size++;
		
		start_idx = end_idx = 0;
		while (start_idx < buffer_size) { //iterate trough the tokens of the string
						
			while( buffer[start_idx + end_idx] != ' ' && buffer[start_idx + end_idx] != '\0' ) { //iterate the characters of the string until find a single space or the end of the string
				end_idx++;
			}
			buffer[start_idx + end_idx] = '\0'; //puts a null character where the space was found so the input_parse_token will only use the current token
			input_parse_token(&buffer[start_idx]); //sends the address of the start of the current token
			start_idx += end_idx + 1; end_idx = 0; //updates the start index to the next token (remember to jump the null character)
		}
		
		return 1;
	}
	
	return 0;
		
} //input_parse

/**
 * Reads and interprets one command token. Updates the global parameters.
 * The token is divided in "Mode" and "Parameter". 
 *
 * The Parameters will change a meaningful data for the execution of a Mode. 
 * If the same Parameter is send twice in one command, only the last information will be saved.
 * A Parameter is followed by a decimal number representing it's value.
 * The list of valid Parameters:
 * |Syntax|Description|Domain|
 * |x[integer]|Sets the x focus coordinate. Value in millimeters|[0,255]|
 * |y[integer]|Sets the y focus coordinate. Value in millimeters|[0,255]|
 * |z[integer]|Sets the z focus coordinate. Value in millimeters|[0,255]|
 * |d[integer]|Sets the duty cycle. Values from 0 to 255, equating to 0% to 100%|[0,255]|
 * |s[integer]|Sets the speed of the displacement. Value in millimeters per second|[1,255]|
 * 
 * The Modes will perform a action in the system.
 * Only one Mode must be sent in the same command. If more than one Mode is send, only the last one will be executed.
 * Some Modes will demand mandatory parameters to be set. They can be set in the same command or can by set in previous commands.
 * The list of valid Modes:
 * |Syntax|Action|Parameters|
 * |m|Set the move mode (moves from the current position to the desired position then deactivates)|x, y, z, d, s|
 * |r|Set the relocate mode (moves from the current position to the desired position and keep active)|x, y, z, d, s|
 * |a|Activates the focal point at the desired position|x, y, z, d|
 * |f|Activates the flat mode|d|
 * |i|Deactivates the current mode||
 *
 * Examples of valid tokens: "m", "r", "a", "f", "i", "x10", "y20", "z30", "d128", "s1".
 *
 * @param token The address of the token
 */
void input_parse_token (char *token) {
	
	if(token == NULL) return;
	if(token[0] == '\0') return;
	
	/*
		valid tokens:
		syntax	action	mandatory_pars
		m	Set the move mode (moves from the current position to the desired position then deactivates)	x, y, z, d, s
		r	Set the relocate mode (moves from the current position to the desired position and keep active)	x, y, z, d, s
		a	Activates the focal point at the desired position	x, y, z, d
		f	Activates the flat mode	d
		i	Deactivates the current mode
		x[integer]	Sets the x focus coordinate. Value in millimeters
		y[integer]	Sets the y focus coordinate. Value in millimeters
		z[integer]	Sets the z focus coordinate. Value in millimeters
		d[integer]	Sets the duty cycle. Values from 0 to 255, equating to 0% to 100%
		s[integer]	Sets the speed of the displacement. Value in millimeters per second
	*/
	switch (token[0]) {
		case 'i':
			mode = MODE_OFF;
			break;
		case 'a':
			mode = MODE_ON;
			break;
		case 'f':
			mode = MODE_FLAT;
			break;
		case 'm':
			mode = MODE_MOVE_OFF;
			break;
		case 'r':
			mode = MODE_MOVE_ON;
			break;
		case 'x':
			traj_ctrl.x = atoi(&token[1]);
			break;
		case 'y':
			traj_ctrl.y = atoi(&token[1]);
			break;
		case 'z':
			traj_ctrl.z = atoi(&token[1]);
			break;
		case 'd':
			traj_ctrl.d = atoi(&token[1]);
			break;
		case 's':
			traj_ctrl.s = atoi(&token[1]);
			break;
	}
} //input_parse_token

/**
 * Executes the command performing the action set by a Mode or changing a Parameter.
 * It's not recommended to apply a new command before a "i" command is sent or the current movement is finished.
 */
void input_execute (){
	
	uint32_t interval;
	
	if(mode == MODE_MOVE_OFF || mode == MODE_MOVE_ON){
		
		if(	traj_ctrl.lastx == traj_ctrl.x &&
			traj_ctrl.lasty == traj_ctrl.y &&
			traj_ctrl.lastz == traj_ctrl.z) { /* no movement? */
				
				if(mode == MODE_MOVE_OFF) { //if a MODE_MOVE_OFF mode is activated but the origin and destiny are the same point, turns the output off immediately
					traj_ctrl.lastx = traj_ctrl.x;
					traj_ctrl.lasty = traj_ctrl.y;
					traj_ctrl.lastz = traj_ctrl.z;
					
					disableTimer3 ();
					disableTimer4 ();
					output_reset ();
				}
				else { //if a MODE_MOVE_ON mode is activated but the origin and destiny are the same point, calculates the output for the destiny and turns the output on. It's equivalent to a MODE_ON command
					traj_calc_step (0, traj_ctrl.d, traj_ctrl.x, traj_ctrl.y, traj_ctrl.z );
					
					traj_step_idx = 0; 
					port_buffer_step_addr = &traj_port_buffer[traj_step_idx][0][0];
					traj_step_num = 1;
					array_phase_idx = 0;
					
					traj_ctrl.lastx = traj_ctrl.x;
					traj_ctrl.lasty = traj_ctrl.y;
					traj_ctrl.lastz = traj_ctrl.z;
					
					disableTimer3 ();
					enableTimer4 ();
				}
			}
		else { /* movement */
			traj_step_num = traj_calc (traj_ctrl.lastx, traj_ctrl.lasty, traj_ctrl.lastz, traj_ctrl.x, traj_ctrl.y, traj_ctrl.z ); //calculates the trajectory and fills the output buffer
			
			interval = traj_calc_speed (traj_ctrl.s, traj_ctrl.lastx, traj_ctrl.lasty, traj_ctrl.lastz, traj_ctrl.x, traj_ctrl.y, traj_ctrl.z ); //calculates the time interval between way points
					
			setTimer3 (interval); //sets the timer 3 that will control the speed of movement
			
			traj_step_idx = 0; //resets the output buffer indexes
			port_buffer_step_addr = &traj_port_buffer[traj_step_idx][0][0];
			array_phase_idx = 0;
			
			traj_ctrl.lastx = traj_ctrl.x;
			traj_ctrl.lasty = traj_ctrl.y;
			traj_ctrl.lastz = traj_ctrl.z;
			
			enableTimer4 (); //enables the output and the movement
			enableTimer3 ();
		}
	}
	else if(mode == MODE_ON) {
		
		traj_calc_step (0, traj_ctrl.d, traj_ctrl.x, traj_ctrl.y, traj_ctrl.z ); //this mode uses the index 0 of the output buffer
		
		traj_step_idx = 0;
		port_buffer_step_addr = &traj_port_buffer[traj_step_idx][0][0];
		traj_step_num = 1;
		array_phase_idx = 0;
		
		traj_ctrl.lastx = traj_ctrl.x;
		traj_ctrl.lasty = traj_ctrl.y;
		traj_ctrl.lastz = traj_ctrl.z;
		
		
		disableTimer3 (); //no movement required
		enableTimer4 ();
	}
	else if(mode == MODE_FLAT) {
		
		traj_calc_step (0, traj_ctrl.d, 0, 0, 0 ); //this mode uses the index 0 of the output buffer
		
		traj_step_idx = 0;
		port_buffer_step_addr = &traj_port_buffer[traj_step_idx][0][0];
		traj_step_num = 1;
		array_phase_idx = 0;
		
		traj_ctrl.x = 0;
		traj_ctrl.y = 0;
		traj_ctrl.z = 0;
		traj_ctrl.lastx = traj_ctrl.x;
		traj_ctrl.lasty = traj_ctrl.y;
		traj_ctrl.lastz = traj_ctrl.z;
		
		disableTimer3 (); //no movement required
		enableTimer4 ();
	}
	else if(mode == MODE_OFF) {
		
		disableTimer3 (); //disables the output and the movement
		disableTimer4 ();
		output_reset ();
	}
	
} //input_execute

/**
 * Initializes the matrix array from the Ultrasonic.c with the designated pin numbers and phase compensation
 */
void transd_array_load ( /*t_transd_array *transd_array(*/ ){
	
	uint8_t x, y;
	
	for(x = 0; x < ARRAY_SIZE_X; x++){
		for(y = 0; y < ARRAY_SIZE_Y; y++){

      ////Serial.print((x*ARRAY_SIZE_Y + y)*2); Serial.print("\t"); 
      ////Serial.println(pgm_read_byte_near(ARRAY_CALIBRATION + (x*ARRAY_SIZE_Y + y)*2));
      ////Serial.println(pgm_read_byte_near(ARRAY_CALIBRATION[x][y][0]));
      ////Serial.println(ARRAY_CALIBRATION[x][y][0]);
			transd_array_set( /*transd_array,*/ x, y, pgm_read_byte_near(&(ARRAY_CALIBRATION[x][y][0])), pgm_read_byte_near(&(ARRAY_CALIBRATION[x][y][1])) ); //special functions to read from PROGMEM
		}
	}
} //transd_array_load

/**
 * Sets all output ports as LOW
 */
void output_reset () {
	//sets all pins as low
	PORTA = 0;
	PORTB = 0;
	PORTC = 0;
	PORTD = 0;
	//PORTE = 0;
	PORTF = 0;
	PORTG = 0;
	PORTH = 0;
	//PORTI = 0;
	PORTJ = 0;
	PORTK = 0;
	PORTL = 0;
}

#ifdef DEBUG_PINS
/**
 * Use to debug the pin assignment using the port registers. 
 * Will set each pin high (starting  from pin 6) sequentially. 
 * The value defined by DEBUG_PINS is the delay between pins in microseconds.
 */
void debug_pins () {
	uint8_t pin;
	
	Serial.println(F("DEBUG ROUTINE - PINS"));
	Serial.print(F("3"));
	delay(1000);
	Serial.print(F(" 2"));
	delay(1000);
	Serial.print(F(" 1"));
	delay(1000);
	Serial.println(F(" GO!"));
	
	for( pin = 0; pin < 70; pin++ ){
		
		output_reset ();
		
		if( pin == 0 || pin == 1 || pin == 2 || pin == 3 || pin == 4 || pin == 5 ){
			
			Serial.print(F("Skipping pin "));
			Serial.println(pin);
		}
		else {
			
			//sets the current pin as high
			*(PINS[pin].bank_ptr) |= PINS[pin].bit_msk; //the goal is to use the mapping define by PINS to test if the mapping is correct
			Serial.print(F("Testing pin "));
			Serial.println(pin);
			delay(DEBUG_PINS);
		}

	}
	
	output_reset ();
		
	Serial.println(F("End of routine"));
} //debug_pins
#endif

#ifdef DEBUG_MAP
/**
 * Use to debug the pin mapping of each element of the matrix. 
 * Will set the pin for each transducer high in the matrix order {(0,0),(0,1),(0,2),(1,0),(1,1),(1,2),...}. 
 * The value defined by DEBUG_MAP is the delay between pins in microseconds.
 */
void debug_map () {
	
	uint8_t x, y;
	uint8_t pin;
	
	Serial.println(F("DEBUG ROUTINE - PIN MAPPING"));
	Serial.print(F("3"));
	delay(1000);
	Serial.print(F(" 2"));
	delay(1000);
	Serial.print(F(" 1"));
	delay(1000);
	Serial.println(F(" GO!"));
	
	for(x = 0; x < ARRAY_SIZE_X; x++){
		for(y = 0; y < ARRAY_SIZE_Y; y++){

			output_reset ();
			
			pin = transd_array[x][y].port_pin; //the goal is to use the mapping loaded from ARRAY_CALIBRATION to the transducer array to test if the mapping is correct
		
			Serial.print(F("Testing ["));
			Serial.print(x);
			Serial.print(F(","));
			Serial.print(y);
			Serial.print(F("] on pin "));
			Serial.println(pin);
			
			*(PINS[pin].bank_ptr) |= PINS[pin].bit_msk;
			
			delay(DEBUG_MAP);
		}
	}
			
	output_reset ();
		
	Serial.println(F("End of routine"));
} //debug_map
#endif

#ifdef DEBUG_INPUT
/**
 * Use to debug the serial input parsing. 
 * Will echo the interpretation of the inputed string
 */
void debug_input () {
	
	Serial.println(F("DEBUG ROUTINE - INPUT"));
	
	if(mode == MODE_MOVE_OFF || mode == MODE_MOVE_ON){
		Serial.print(F("Move from (")); Serial.print(traj_ctrl.lastx); Serial.print(F(",")); Serial.print(traj_ctrl.lasty); Serial.print(F(",")); Serial.print(traj_ctrl.lastz); Serial.print(F(") to (")); Serial.print(traj_ctrl.x); Serial.print(F(",")); Serial.print(traj_ctrl.y); Serial.print(F(",")); Serial.print(traj_ctrl.z); Serial.print(F(") at ")); Serial.print(traj_ctrl.s); Serial.print(F("mm/s with duty cycle of ")); Serial.print(traj_ctrl.d);
		if(mode == MODE_MOVE_ON) {
			 Serial.print(F(" then keep active"));
		}
		else {
			 Serial.print(F(" then deactivate"));
		}
	}
	else if(mode == MODE_ON) {
		Serial.print(F("Activate at (")); Serial.print(traj_ctrl.x); Serial.print(F(",")); Serial.print(traj_ctrl.y); Serial.print(F(",")); Serial.print(traj_ctrl.z); Serial.print(F(") with duty cycle of ")); Serial.print(traj_ctrl.d);
	}
	else if(mode == MODE_FLAT) {
		Serial.print(F("Activate flat mode with duty cycle of ")); Serial.print(traj_ctrl.d);
	}
	else if(mode == MODE_OFF) {
		Serial.print(F("Deactivate"));
	}
	Serial.print(F("\n"));

	Serial.println(F("End of routine"));
} //debug_input
#endif

#ifdef DEBUG_PATTERN
/**
 * Use to debug the pattern generation routine. 
 * Will serial out the pattern for each transducer of the matrix when the pattern is calculated.
 * This function must be called after the pattern calculation is done because it will read directly from the transducer array.
 * A "#" means output HIGH and a "_" means output LOW.
 */
void debug_pattern () {
	
	uint8_t i,x,y;
	
	Serial.println(F("DEBUG ROUTINE - PATTERN"));
	
	//prints the header of the columns
	Serial.println(F("position_x;position_y;port_pin;phase_comp;pattern"));
	
	for(x = 0; x < ARRAY_SIZE_X; x++){
		for(y = 0; y < ARRAY_SIZE_Y; y++){

			Serial.print(x);
			Serial.print(F(";"));
			Serial.print(y);
			Serial.print(F(";"));
			Serial.print(transd_array[x][y].port_pin);
			Serial.print(F(";"));
			Serial.print(transd_array[x][y].phase_comp);
			Serial.print(F(";"));
			//converts the 16 bit pattern to a 16 characters string (not 16, but ARRAY_PHASERES)
			//the '#' represents a active output and the '_' represents a inactive output
			for(i = 0; i < ARRAY_PHASERES; i++) {

				//creates a mask with the current bit = 1
				if(transd_array[x][y].pattern & (1 << (i))) { //if the pattern have the current bit = 1
					Serial.print(F("#")); 
				}
				else {
					Serial.print(F("_"));
				}
			}
			Serial.print(F("\n"));
		}
	}

	Serial.println(F("End of routine"));
} //debug_pattern
#endif

#ifdef DEBUG_TRAJ
/**
 * Use to debug the trajectory planning routine. 
 * Will serial out a header for the trajectory data printed by debug_traj_step.
 */
void debug_traj_header () {
	
	Serial.println(F("DEBUG ROUTINE - TRAJ"));
	
	//prints the header of the columns
	Serial.println(F("step_idx;position_x;position_y;position_z"));
	
} //debug_traj_header

/**
 * Use to debug the trajectory planning routine. 
 * Will serial out the trajectory way points.
 * @param step_idx The index of the output buffer that will be read
 * @param focus_x The focus coordinate on the x axis for that index
 * @param focus_y The focus coordinate on the y axis for that index
 * @param focus_z The focus coordinate on the z axis for that index
 */
void debug_traj_step (const uint8_t step_idx, const uint8_t focus_x, const uint8_t focus_y, const uint8_t focus_z) {
	
	Serial.print(step_idx);
	Serial.print(F(";"));
	Serial.print(focus_x);
	Serial.print(F(";"));
	Serial.print(focus_y);
	Serial.print(F(";"));
	Serial.print(focus_z);
	Serial.print(F("\n"));
	
} //debug_traj_step

/**
 * Use to debug the trajectory planning routine. 
 * Will serial out the speed calculation data.
 * @param s The target displacement speed (in millimeters per second).
 * @param from_x The initial coordinate on the x axis (in millimeters)
 * @param from_y The initial coordinate on the y axis (in millimeters)
 * @param from_z The initial coordinate on the z axis (in millimeters)
 * @param to_x The final coordinate on the x axis (in millimeters)
 * @param to_y The final coordinate on the y axis (in millimeters)
 * @param to_z The final coordinate on the z axis (in millimeters)
 * @param Dx The distance along the x axis (in millimeters)
 * @param Dy The distance along the y axis (in millimeters)
 * @param Dz The distance along the z axis (in millimeters)
 * @param distance The distance between the initial and final points (in millimeters)
 * @param speed_axys The calculated displacement speed along the indexing axis (in millimeters per second).
 * @param interval The time interval between way points (in microseconds)
 */
void debug_traj_speed (const uint8_t s, const uint8_t from_x, const uint8_t from_y, const uint8_t from_z, const uint8_t to_x, const uint8_t to_y, const uint8_t to_z, const int8_t Dx, const int8_t Dy, const int8_t Dz, const uint32_t distance, const uint32_t speed_axys, const uint32_t interval) {
	
	Serial.print(F("s: "));			Serial.println(s);
	Serial.print(F("from_x: "));	Serial.println(from_x);
	Serial.print(F("from_y: "));	Serial.println(from_y);
	Serial.print(F("from_z: "));	Serial.println(from_z);
	Serial.print(F("to_x: "));		Serial.println(to_x);
	Serial.print(F("to_y: "));		Serial.println(to_y);
	Serial.print(F("to_z: "));		Serial.println(to_z);
	Serial.print(F("Dx: "));		Serial.println(Dx);
	Serial.print(F("Dy: "));		Serial.println(Dy);
	Serial.print(F("Dz: "));		Serial.println(Dz);
	Serial.print(F("distance: "));	Serial.println(distance);
	Serial.print(F("speed_axys: "));//Serial.println(speed_axys);
	Serial.print(F("interval: "));	Serial.println(interval);

	Serial.println(F("End of routine"));
} //debug_traj_speed
#endif

#ifdef DEBUG_TIMER
/**
 * Use to debug the transducer signal generation (waveform, frequency accuracy, jitter). 
 * Will configure the matrix to output the defined pattern on all transducers. 
 * Note that the pattern generation will be overridden and the phase compensation will be disabled. 
 * The value defined by DEBUG_TIMER is a binary unsigned representing the pattern to be outputted (0xAAAA = #_#_#_#_#_#_#_#_)
 */
void debug_timer () {
	
	uint8_t x, y, phase_idx, bit, pin, port_idx = 0;
		
	for(phase_idx = 0; phase_idx < ARRAY_PHASERES; phase_idx++){
		traj_port_buffer[0][phase_idx][0] = 0x00; //PORTA
		traj_port_buffer[0][phase_idx][1] = 0x00; //PORTB
		traj_port_buffer[0][phase_idx][2] = 0x00; //PORTC
		traj_port_buffer[0][phase_idx][3] = 0x00; //PORTD
		traj_port_buffer[0][phase_idx][4] = 0x00; //PORTF
		traj_port_buffer[0][phase_idx][5] = 0x00; //PORTG
		traj_port_buffer[0][phase_idx][6] = 0x00; //PORTH
		traj_port_buffer[0][phase_idx][7] = 0x00; //PORTJ
		traj_port_buffer[0][phase_idx][8] = 0x00; //PORTK
		traj_port_buffer[0][phase_idx][9] = 0x00; //PORTL
	}
		
	for(x = 0; x < ARRAY_SIZE_X; x++){
		for(y = 0; y < ARRAY_SIZE_Y; y++){
			
			//gets the pin that the transducer is connected to
			pin = transd_array[x][y].port_pin;
			
			if(PINS[pin].bank_ptr == &PORTA) port_idx = 0;
			if(PINS[pin].bank_ptr == &PORTB) port_idx = 1;
			if(PINS[pin].bank_ptr == &PORTC) port_idx = 2;
			if(PINS[pin].bank_ptr == &PORTD) port_idx = 3;
			if(PINS[pin].bank_ptr == &PORTF) port_idx = 4;
			if(PINS[pin].bank_ptr == &PORTG) port_idx = 5;
			if(PINS[pin].bank_ptr == &PORTH) port_idx = 6;
			if(PINS[pin].bank_ptr == &PORTJ) port_idx = 7;
			if(PINS[pin].bank_ptr == &PORTK) port_idx = 8;
			if(PINS[pin].bank_ptr == &PORTL) port_idx = 9;
			
			for(phase_idx = 0; phase_idx < ARRAY_PHASERES; phase_idx++){
			
				//access the pattern and gets the value for the bit representing the current step
				bit = DEBUG_TIMER & (1 << phase_idx);
				
				//updates only the current pin
				if(bit) {
					traj_port_buffer[0][phase_idx][port_idx] |= PINS[pin].bit_msk;
				}
			}
		}
	}
	
	traj_step_idx = 0; //locks the step index at 0 to repeat the same pattern
	port_buffer_step_addr = &traj_port_buffer[traj_step_idx][0][0];
	traj_step_num = 1;
	array_phase_idx = 0;
	mode = MODE_ON;
	//enableTimer4();
	
	Serial.println(traj_port_buffer[0][0][0]);
	Serial.println(traj_port_buffer[0][1][0]);
	Serial.println(traj_port_buffer[0][2][0]);
	Serial.println(traj_port_buffer[0][3][0]);
	Serial.println(traj_port_buffer[0][4][0]);
	Serial.println(traj_port_buffer[0][5][0]);
	Serial.println(traj_port_buffer[0][6][0]);
	Serial.println(traj_port_buffer[0][7][0]);
	Serial.println(F("Output signal is set up..."));
} //debug_timer
#endif