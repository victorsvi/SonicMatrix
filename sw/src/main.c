/*
	PROGRAMAS DE TESTE
	
	Que códigos precisam ser criados para testar os conceitos uilizados?
	
	# DONE Geração de sinal 40kHz * 8 (resolução) (teste do timer) (checar jitter). Fazer também em 64 canais para verificar o desempenho.
	# DONE Interpretação da entrada serial
	# Dump do cálculo das fases
	# DONE Dump das portas
	# DONE Para testar o mapeamento dos pinos (exibir a coordenada x, y e o numero do pino em display, enquanto gera um sinal no pino. a entrada serial indica qual coordenada checar)
	# Comparar a sqrt() do arduino com a de inteiro (custom), calculando a raiz de vários números e contando o tempo para a conclusão.
*/

/*
	IMPORTANTE! Calcular o foco em tempo real é muito lento (cerca de 18ms por cálculo)
	
	No main, usar a array_calc_focus e copiar as patterns para outro array. Esse outro array deve
	armazenar vários "passos", de forma que o conjunto dos passos forme uma tragetória.
	
	O destino é informado pela serial, o arduino (que sabe a origem) vai fazer uma linha reta até o destino, colocando todos os "passos" nesse array antes de iniciar (deve demorar cerca de 180ms para 100 passos).
*/


#include <stdint.h>
#include <Arduino.h>
#include "Ultrasonic.h"
#include "Debug.h"
#include "Timer4.h"
#include "Timer5.h"
#include "Math.h"

//#define DEBUG_PINS 2000 //DELAY BETWEEN PINS IN MS //coloca saída nível alto em cada pino sequencialmente (a partir do 6). Testa a parte de manipular as portas pelos registradores
//#define DEBUG_MAP 2000 //DELAY BETWEEN PINS IN MS //coloca saída nível alto no pino de cada elemento, na ordem da matriz {(0,0),(0,1),(0,2),(1,0),(1,1),(1,2),...}. Testa o mapeamento de pinos
//#define DEBUG_INPUT //retorna a interpretação da entrada. Testa o parse dos comandos
//#define DEBUG_PATTERN //retorna o pattern de cada elemento quando ele for calculado. Testa a geração dos padrões
//#define DEBUG_TRAJ //retorna as coordenadas dos pontos da trajetória e os dados de velocidade
//#define DEBUG_TIMER 0xAAAA //PATTERN MASK 0xAAAA = #_#_#_#_#_#_#_#_ //configura a saída de todos os elementos com o padrão especificado. Testa a capacidade de gerar o sinal de saída para todos os canais.

/*
#define TRANS_DIAMETER 16 //diameter of the element in millimeters (total length of the array cant exceed 255 millimeters)
#define TRANS_SEPARATION 2 //distance between two consecutive elements in the array in millimeters (total length of the array cant exceed 255 millimeters)
#define ARRAY_SIZE_X 8 //number of transducers of the array in the x dimension
#define ARRAY_SIZE_Y 8 //number of transducers of the array in the y dimension
#define ARRAY_PHASERES 10 //number of transducers of the array in the y dimension (max 16 bits)
*/
#define TRAJ_RES 1 //trajectory maximum resolution in millimeters
#define TRAJ_MAXSTEPS 64 //maximum steps of the trajectory (max 255)

/* MACROS */

#define MSK(b) (1 << b) //creates a mask with the bth bit high

/* PROTOTYPES */

ISR( TIMER4_COMPA_vect );
ISR( TIMER5_COMPA_vect );

uint8_t traj_calc (const uint8_t from_x, const uint8_t from_y, const uint8_t from_z, const uint8_t to_x, const uint8_t to_y, const uint8_t to_z );
void traj_calc_step (uint8_t step_idx, const uint8_t duty_cycle, const uint8_t focus_x, const uint8_t focus_y, const uint8_t focus_z );
uint8_t traj_solve_y (uint8_t x, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2 );
uint8_t traj_solve_z (uint8_t x, uint8_t x1, uint8_t z1, uint8_t x2, uint8_t z2 );
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
void debug_traj_step (const uint8_t step_idx, const uint8_t focus_x, const uint8_t focus_y);
void debug_traj_speed (const uint8_t s, const uint8_t from_x, const uint8_t from_y, const uint8_t from_z, const uint8_t to_x, const uint8_t to_y, const uint8_t to_z, const int8_t Dx, const int8_t Dy, const int8_t Dz, const uint32_t distance, const uint32_t speed_axys, const uint32_t interval);
#endif
#ifdef DEBUG_TIMER
void debug_timer ();
#endif

void transd_array_load ( /*t_transd_array *transd_array*/ );

/* DATA DEFINITION */

enum e_mode {	MODE_OFF,		// turned off, signal is held low
				MODE_ON, 		// turned on, with static focal point
				MODE_MOVE_OFF, 	// move to focal point to target then tuns off
				MODE_MOVE_ON, 	// move to focal point to target and keeps on
				MODE_FLAT		// turned on, flat mode, all transducers in phase
};

struct s_traj_ctrl {
	uint8_t x = 0, y = 0, z = 0, d = 0, s = 0;
	uint8_t lastx = 0, lasty = 0, lastz = 0;
};

struct s_pin {
	volatile uint8_t *bank_ptr;
	uint8_t bit_msk;
};

/* GLOBAL VARIABLES */

const uint8_t ARRAY_CALIBRATION[ARRAY_SIZE_X][ARRAY_SIZE_Y][2] PROGMEM = {
	//pin numer, phase compensation
	//x0, y0,
	//x0, y1,
	//0 , 0,
	//1 , 0,
	//2 , 0,
	//3 , 0,
	//4 , 0,
	//5 , 0,
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

/* Port bank and bit mask for each pin indexed by pin number */
const struct s_pin PINS[70] = {
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

struct s_traj_ctrl traj_ctrl;
volatile uint8_t traj_step_idx = 0, traj_step_num = 0;
volatile uint8_t array_phase_idx = 0;
enum e_mode mode;

/*t_transd_array *transd_array = NULL;*/

uint8_t traj_port_buffer[TRAJ_MAXSTEPS][ARRAY_PHASERES][10]; //buffers the ports state for each coordinate (x,y,z) of the movement, for each slice of the wave period, for each PORT

void setup () {
	
	Serial.begin(115200);
	
	//set ports as output (bit high = output)
	DDRA = 0xFF;
	DDRB = 0xFF;
	DDRC = 0xFF;
	DDRD = 0xFF;
	//DDRE = 0xFF;
	DDRF = 0xFF;
	DDRG = 0xFF;
	DDRH = 0xFF;
	//DDRI = 0xFF;
	DDRJ = 0xFF;
	DDRK = 0xFF;
	DDRL = 0xFF;
	
#ifdef DEBUG_PINS
	debug_pins ();
#endif
	
	setTimer4();
	
	/*transd_array = transd_array_init( ARRAY_SIZE_X, ARRAY_SIZE_Y, TRANS_DIAMETER, TRANS_SEPARATION, ARRAY_PHASERES );
	if( transd_array == NULL ) {
		#ifdef DEBUG
		DEBUG_MSG("The transducer array pointer is null")
		#endif
	}*/
	transd_array_load (/*transd_array*/);

#ifdef DEBUG_MAP
	debug_map ();
#endif

	mode = MODE_OFF;
	
#ifdef DEBUG_TIMER
	debug_timer ();
#endif
} //setup

void loop () {
	
	if(input_parse()) {
#ifdef DEBUG_INPUT
		debug_input();
#endif
		input_execute();
	}
	
} //loop

/*

*/
ISR( TIMER4_COMPA_vect ) {

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
	
	phase_idx < ARRAY_PHASERES ? array_phase_idx++ : array_phase_idx = 0;

} //ISR T4

/*

*/
ISR( TIMER5_COMPA_vect ) {

	
	if(traj_step_idx <= traj_step_num - 1) { //increments the step until reaches the last step
		traj_step_idx++;
	}
	else { //then stay in the last step and disables the timer
		disableTimer5 ();
	}

} //ISR T5

/*

*/
uint8_t traj_calc (const uint8_t from_x, const uint8_t from_y, const uint8_t from_z, const uint8_t to_x, const uint8_t to_y, const uint8_t to_z ) {
	
	uint8_t traj_x, traj_y, traj_z, step_idx;

	step_idx = 0;
	
#ifdef DEBUG_TRAJ
	debug_traj_header ();
#endif
	
	if( from_x != to_x ) { /* if the trajectory isn't perperdicular to the x axys */
		
		traj_x = from_x;
		
		for(; (from_x < to_x) ? (traj_x <= to_x) : (traj_x >= to_x); (from_x < to_x) ? (traj_x += TRAJ_RES) : (traj_x -= TRAJ_RES) ) {
			
			traj_y = traj_solve_y (traj_x, to_x, to_y, from_x, from_y );
			traj_z = traj_solve_z (traj_x, to_x, to_z, from_x, from_z );
			traj_calc_step (step_idx, traj_ctrl.d, traj_x, traj_y, traj_z );
			step_idx++;
			if(step_idx >= TRAJ_MAXSTEPS) {
				break;
			}
		}
	}
	else if( from_y != to_y ) { /* if the trajectory is perperdicular to the x axys but isn't perperdicular to the y axys */
		
		traj_x = from_x;
		traj_y = from_y;
		
		for(; (from_y < to_y) ? (traj_y <= to_y) : (traj_y >= to_y); (from_y < to_y) ? (traj_y += TRAJ_RES) : (traj_y -= TRAJ_RES) ) {
			
			traj_z = traj_solve_z (traj_y, to_y, to_z, from_y, from_z );
			traj_calc_step (step_idx, traj_ctrl.d, traj_x, traj_y, traj_z );
			step_idx++;
			if(step_idx >= TRAJ_MAXSTEPS) {
				break;
			}
		}
	}
	else { /* if the trajectory is perperdicular to the x and y axys */
		
		traj_x = from_x;
		traj_y = from_y;
		traj_z = from_z;
		
		for(; (from_z < to_z) ? (traj_z <= to_z) : (traj_z >= to_z); (from_z < to_z) ? (traj_z += TRAJ_RES) : (traj_z -= TRAJ_RES) ) {
			
			traj_calc_step (step_idx, traj_ctrl.d, traj_x, traj_y, traj_z );
			step_idx++;
			if(step_idx >= TRAJ_MAXSTEPS) {
				break;
			}
		}
	}
	
	return step_idx;
}

/*

*/
void traj_calc_step (uint8_t step_idx, const uint8_t duty_cycle, const uint8_t focus_x, const uint8_t focus_y, const uint8_t focus_z ) {
	
	uint8_t x, y, phase_idx, bit, pin, port_idx = 0;
	
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
				bit = transd_array[x][y].pattern & (1 << phase_idx);
				
				//updates only the current pin
				if(bit) {
					traj_port_buffer[step_idx][phase_idx][port_idx] |= PINS[pin].bit_msk;
				}
				/* no need to set as low because all the bits of the buffer are set to zero
				else {
					traj_port_buffer[step_idx][phase_idx][port_idx] &= ~PINS[pin].bit_idx;
				}
				*/
			}
		}
	}
	
#ifdef DEBUG_TRAJ
	debug_traj_step (step_idx, focus_x, focus_y);
#endif
	
#ifdef DEBUG_PATTERN
	debug_pattern ();
#endif

} //traj_calc_step

/*

*/
uint8_t traj_solve_y (uint8_t x, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2 ) {
	
	int32_t _x, _y, _x1, _y1, _x2, _y2;
	uint8_t y;
	
	//using 2 decimal places fixed point (force cast before multiplication)
	_x  = int32_t( x) * 100; 
	_x1 = int32_t(x1) * 100; 
	_y1 = int32_t(y1) * 100; 
	_x2 = int32_t(x2) * 100; 
	_y2 = int32_t(y2) * 100;
	
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
	
	//using 2 decimal places fixed point
	_y /= 100;
	
	//the point will always be at the first quadrant (x >= 0 and y >= 0)
	y = uint8_t(_y);
	
	return y;
	
} //traj_solve_y

/*

*/
uint8_t traj_solve_z (uint8_t x, uint8_t x1, uint8_t z1, uint8_t x2, uint8_t z2 ) {
	
	return traj_solve_y (x, x1, z1, x2, z2 );	
} //traj_solve_y

/*

*/
uint32_t traj_calc_speed (const uint8_t s, const uint8_t from_x, const uint8_t from_y, const uint8_t from_z, const uint8_t to_x, const uint8_t to_y, const uint8_t to_z ){
	
	uint32_t distance, speed_axys, interval;
	int8_t Dx, Dy, Dz;
	
	Dx = to_x - from_x;
	Dy = to_y - from_y;
	Dz = to_z - from_z;
	
	distance = (Dx * Dx) + (Dy * Dy) + (Dz * Dz); //squared distance
	//distance *= 100; //multiplies by 10 squared to "keep" one decimal place after the square root.
	distance = transd_sqrt_int(distance); //x10. After taking the square root, the distance is not in milimeters
	//distance /= 10;
	
	if( from_x != to_x ) { /* if the trajectory isn't perperdicular to the x axys */
		
		speed_axys = (s * Dx) / distance;
	}
	else if( from_y != to_y ) { /* if the trajectory is perperdicular to the x axys but isn't perperdicular to the y axys */
		
		speed_axys = (s * Dy) / distance;
	}
	else { /* if the trajectory is perperdicular to the x and y axys */
		
		speed_axys = s;
	}
	
	interval = (TRAJ_RES * 10E6) / speed_axys;
	

#ifdef DEBUG_TRAJ
	debug_traj_speed (s, from_x, from_y, from_z, to_x, to_y, to_z, Dx, Dy, Dz, distance, speed_axys, interval);
#endif
	
	return interval;
}

/*

*/
uint8_t input_parse () {
	
	char buffer[64];
	uint8_t buffer_size, start_idx, end_idx;
	/*
	char *buffPtr;
	uint8_t n;
	*/
	
	if(Serial.available()) {
		
		buffer_size =  Serial.readBytes(buffer,63);
		buffer[buffer_size] = '\0';
		buffer_size++;
		
		start_idx = end_idx = 0;
		while (start_idx < buffer_size) {
						
			while( buffer[start_idx + end_idx] != ' ' && buffer[start_idx + end_idx] != '\0' ) {
				end_idx++;
			}
			buffer[start_idx + end_idx] = '\0';
			input_parse_token(&buffer[start_idx]);
			start_idx += end_idx + 1; end_idx = 0;
		}
		
		return 1;
	}
	
	return 0;
		
} //input_parse

/*

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

/*

*/
void input_execute (){
	
	uint32_t interval;
	
	if(mode == MODE_MOVE_OFF || mode == MODE_MOVE_ON){
		
		traj_step_num = traj_calc (traj_ctrl.lastx, traj_ctrl.lasty, traj_ctrl.lastz, traj_ctrl.x, traj_ctrl.y, traj_ctrl.z );
		
		interval = traj_calc_speed (traj_ctrl.s, traj_ctrl.lastx, traj_ctrl.lasty, traj_ctrl.lastz, traj_ctrl.x, traj_ctrl.y, traj_ctrl.z );
				
		setTimer5 (interval);
		
		traj_step_idx = 0;
		array_phase_idx = 0;
		
		traj_ctrl.lastx = traj_ctrl.x;
		traj_ctrl.lasty = traj_ctrl.y;
		traj_ctrl.lastz = traj_ctrl.z;
		
		enableTimer5 ();
		enableTimer4 ();
	}
	else if(mode == MODE_ON) {
		
		traj_calc_step (0, traj_ctrl.d, traj_ctrl.x, traj_ctrl.y, traj_ctrl.z );
		
		traj_step_idx = 0;
		traj_step_num = 1;
		array_phase_idx = 0;
		
		traj_ctrl.lastx = traj_ctrl.x;
		traj_ctrl.lasty = traj_ctrl.y;
		traj_ctrl.lastz = traj_ctrl.z;
		
		
		disableTimer5 ();
		enableTimer4 ();
	}
	else if(mode == MODE_FLAT) {
		
		traj_calc_step (0, traj_ctrl.d, 0, 0, 0 );
		
		traj_step_idx = 0;
		traj_step_num = 1;
		array_phase_idx = 0;
		
		traj_ctrl.x = 0;
		traj_ctrl.y = 0;
		traj_ctrl.z = 0;
		traj_ctrl.lastx = traj_ctrl.x;
		traj_ctrl.lasty = traj_ctrl.y;
		traj_ctrl.lastz = traj_ctrl.z;
		
		disableTimer5 ();
		enableTimer4 ();
	}
	else if(mode == MODE_OFF) {
		
		disableTimer5 ();
		disableTimer4 ();
	}
	
} //input_execute

/*

*/
void transd_array_load ( /*t_transd_array *transd_array(*/ ){
	
	uint8_t x, y;
	
	for(x = 0; x < ARRAY_SIZE_X; x++){
		for(y = 0; y < ARRAY_SIZE_Y; y++){

			transd_array_set( /*transd_array,*/ x, y, pgm_read_byte_near(ARRAY_CALIBRATION + x*ARRAY_SIZE_Y + y), pgm_read_byte_near(ARRAY_CALIBRATION + x*ARRAY_SIZE_Y + y + 1) );
		}
	}
} //transd_array_load


#ifdef DEBUG_PINS
/*

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
		
		if( pin == 0 || pin == 1 || pin == 2 || pin == 3 || pin == 4 || pin == 5 ){
			
			Serial.print(F("Skipping pin "));
			Serial.println(pin);
		}
		else {
			
			//sets the current pin as high
			*(PINS[pin].bank_ptr) |= PINS[pin].bit_msk;
			Serial.print(F("Testing pin "));
			Serial.println(pin);
			delay(DEBUG_PINS);
		}

	}
	
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
		
	Serial.println(F("End of routine"));
} //debug_pins
#endif

#ifdef DEBUG_MAP
/*

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
			
			pin = transd_array[x][y].port_pin;
		
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
		
	Serial.println(F("End of routine"));
} //debug_map
#endif

#ifdef DEBUG_INPUT
/*

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
/*

*/
void debug_pattern () {
	
	uint8_t i,x,y;
	
	Serial.println(F("DEBUG ROUTINE - PATTERN"));
	
	//prints the header of the collumns
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
			//converts the 16 bit pattern to a 16 characters string
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
/*

*/
void debug_traj_header () {
	
	Serial.println(F("DEBUG ROUTINE - TRAJ"));
	
	//prints the header of the collumns
	Serial.println(F("step_idx;position_x;position_y"));
	
} //debug_traj_header

/*

*/
void debug_traj_step (const uint8_t step_idx, const uint8_t focus_x, const uint8_t focus_y) {
	
	Serial.print(step_idx);
	Serial.print(F(";"));
	Serial.print(focus_x);
	Serial.print(F(";"));
	Serial.print(focus_y);
	Serial.print(F("\n"));
	
} //debug_traj_step

/*

*/
void debug_traj_speed (const uint8_t s, const uint8_t from_x, const uint8_t from_y, const uint8_t from_z, const uint8_t to_x, const uint8_t to_y, const uint8_t to_z, const int8_t Dx, const int8_t Dy, const int8_t Dz, const uint32_t distance, const uint32_t speed_axys, const uint32_t interval) {
	
	Serial.print(F("s"));			Serial.println(s);
	Serial.print(F("from_x"));		Serial.println(from_x);
	Serial.print(F("from_y"));		Serial.println(from_y);
	Serial.print(F("from_z"));		Serial.println(from_z);
	Serial.print(F("to_x"));		Serial.println(to_x);
	Serial.print(F("to_y"));		Serial.println(to_y);
	Serial.print(F("to_z"));		Serial.println(to_z);
	Serial.print(F("Dx"));			Serial.println(Dx);
	Serial.print(F("Dy"));			Serial.println(Dy);
	Serial.print(F("Dz"));			Serial.println(Dz);
	Serial.print(F("distance"));	Serial.println(distance);
	Serial.print(F("speed_axys"));	Serial.println(speed_axys);
	Serial.print(F("interval"));	Serial.println(interval);

	Serial.println(F("End of routine"));
} //debug_traj_speed
#endif

#ifdef DEBUG_TIMER
/*

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
	
	traj_step_idx = 0;
	traj_step_num = 1;
	array_phase_idx = 0;
	mode = MODE_ON;
	enableTimer4();
	
	Serial.println(F("Output signal is set up..."));
} //debug_timer
#endif
