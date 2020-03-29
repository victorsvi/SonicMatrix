
//pinToBank() //converte pino para porta
//criar um timer para gerar o sinal de 40khz. com resolução de 1/10, o timer deve ter frequência de 400kHz (2.5us) (criar um lib para operar o timer?)
//algum jeito de efetuar o cálculo do delay mais rápido? SIM! posso fazer, para uma coordenada z, preenher uma matriz com o delay vs distância do foco. Depois, para cada transdutor, colocar o valor relativo à distância do foco em x, y. (será que é mas rápido? irá ter que calcular a d = (dx^2 + dy^2)^1/2
//algum jeito de atribuir as portas de forma rápida?
//interpretar a serial de forma assíncrona? implementar de forma que a geração do sinal execute com prioridade e a interpretação da entrada seja feita "quando sobrar ciclos" (software interrupt?)


/*
	PROGRAMAS DE TESTE
	
	Que códigos precisam ser criados para testar os conceitos uilizados?
	
	# Geração de sinal 40kHz * 8 (resolução) (teste do timer) (checar jitter). Fazer também em 64 canais para verificar o desempenho.
	# Interpretação da entrada serial
	# Dump do cálculo das fases
	# Dump das portas
	# Para testar o mapeamento dos pinos (exibir a coordenada x, y e o numero do pino em display, enquanto gera um sinal no pino. a entrada serial indica qual coordenada checar)
	# Comparar a sqrt() do arduino com a de inteiro (custom), calculando a raiz de vários números e contando o tempo para a conclusão.
*/

/*
	IMPORTANTE! Calcular o foco em tempo real é muito lento (cerca de 18ms por cálculo)
	
	No main, usar a array_calc_focus e copiar as patterns para outro array. Esse outro array deve
	armazenar vários "passos", de forma que o conjunto dos passos forme uma tragetória.
	
	O destino é informado pela serial, o arduino (que sabe a origem) vai fazer uma linha reta até o destino, colocando todos os "passos" nesse array antes de iniciar (deve demorar cerca de 180ms para 100 passos).
*/


//#include <stdlib.h>
#include <stdint.h>
//#include <EEPROM.h>
#include <Arduino.h>
#include "Ultrasonic.h"
#include "Debug.h"
#include "Timer4.h"

#define TRANS_DIAMETER 16 //diameter of the element in millimeters
#define TRANS_SEPARATION 2 //distance between two consecutive elements in the array in millimeters
#define ARRAY_SIZE_X 8 //number of transducers of the array in the x dimension
#define ARRAY_SIZE_Y 8 //number of transducers of the array in the y dimension
#define ARRAY_PHASERES 10 //number of transducers of the array in the y dimension
#define TRAJ_RES 1 //trajectory resolution in millimeters
#define TRAJ_MAXSTEPS 100 //maximum steps of the trajectory

/* PROTOTYPES */

void trand_array_load ( t_transd_array *transd_array );
ISR( TIMER4_COMPA_vect );
uint8_t parseInput ();
void parseInputToken (char *token);
void clearState ();
void printCommand ();

/* DATA DEFINITION */
struct s_state {
	uint8_t isActiveMove = 0, isActiveStatic = 0, isStop = 0, isFlat = 0;
	uint8_t x = 0, y = 0, z = 0, d = 0, s = 0;
	uint8_t currStep = 0;
	uint8_t moveCurrStep = 0, moveNSteps = 0;
	uint8_t lastx = 0, lasty = 0, lastz = 0;
};

{
	if(isActiveMove || isActiveStatic) {
		//state.lastx = state.x;
		//state.lasty = state.y;
		//state.lastz = state.z;
		
		moveCurrStep = 0;
		moveNSteps = 0;
		
		//calculate output buffer
		if(isActiveMove) { //movo to
			
		}
		else if (!isFlat) { //focus static
			
		}
		else { //flat
			
			transd_array_calcflat( transd_array, state.d );

		}
		
		enableTimer4 ();
		
	}
	
	if(isStop) {
		
		disableTimer4 ();
		
	}
}

struct s_pin {
	volatile uint8_t *bank_ptr;
	uint8_t bit_idx;
};

/* GLOBAL VARIABLES */

const uint8_t ARRAY_CALIBRATION[ARRAY_SIZE_X][ARRAY_SIZE_Y][2] = {
	//pin numer, phase compensation
	4 , 0,//x0, y0
	6 , 0,//x0, y1
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

const struct s_pin PINS[70] = {
	{&PORTE, 0},
	{&PORTE, 1},
	{&PORTE, 4},
	{&PORTE, 5},
	{&PORTG, 5},
	{&PORTE, 3},
	{&PORTH, 3},
	{&PORTH, 4},
	{&PORTH, 5},
	{&PORTH, 6},
	{&PORTB, 4},
	{&PORTB, 5},
	{&PORTB, 6},
	{&PORTB, 7},
	{&PORTJ, 1},
	{&PORTJ, 0},
	{&PORTH, 1},
	{&PORTH, 0},
	{&PORTD, 3},
	{&PORTD, 2},
	{&PORTD, 1},
	{&PORTD, 0},
	{&PORTA, 0},
	{&PORTA, 1},
	{&PORTA, 2},
	{&PORTA, 3},
	{&PORTA, 4},
	{&PORTA, 5},
	{&PORTA, 6},
	{&PORTA, 7},
	{&PORTC, 7},
	{&PORTC, 6},
	{&PORTC, 5},
	{&PORTC, 4},
	{&PORTC, 3},
	{&PORTC, 2},
	{&PORTC, 1},
	{&PORTC, 0},
	{&PORTD, 7},
	{&PORTG, 2},
	{&PORTG, 1},
	{&PORTG, 0},
	{&PORTL, 7},
	{&PORTL, 6},
	{&PORTL, 5},
	{&PORTL, 4},
	{&PORTL, 3},
	{&PORTL, 2},
	{&PORTL, 1},
	{&PORTL, 0},
	{&PORTB, 3},
	{&PORTB, 2},
	{&PORTB, 1},
	{&PORTB, 0},
	{&PORTF, 0},
	{&PORTF, 1},
	{&PORTF, 2},
	{&PORTF, 3},
	{&PORTF, 4},
	{&PORTF, 5},
	{&PORTF, 6},
	{&PORTF, 7},
	{&PORTK, 0},
	{&PORTK, 1},
	{&PORTK, 2},
	{&PORTK, 3},
	{&PORTK, 4},
	{&PORTK, 5},
	{&PORTK, 6},
	{&PORTK, 7}
};

struct s_state state;

t_transd_array *transd_array = NULL;

uint8_t traj_steps[TRAJ_MAXSTEPS][ARRAY_SIZE_X][ARRAY_SIZE_Y];
//uint8_t traj_steps[TRAJ_MAXSTEPS][ARRAY_PHASERES][10];

void setup () {
	
	Serial.begin(115200);
	
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
	
	setTimer4();
	
	transd_array = transd_array_init( ARRAY_SIZE_X, ARRAY_SIZE_Y, TRANS_DIAMETER, TRANS_SEPARATION, ARRAY_PHASERES );
	if( transd_array == NULL ) {
		#ifdef DEBUG
		DEBUG_MSG("The transducer array pointer is null")
		#endif
	}
	trand_array_load (transd_array);
	
}

void loop () {
	
	
}

ISR( TIMER4_COMPA_vect ) {
	
	uint8_t x, y, bit, pin;
		
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
	
	//if(state.isActiveStatic && !state.isActiveMove) {
			
		
		for(x = 0; x < ARRAY_SIZE_X; x++){
			for(y = 0; y < ARRAY_SIZE_Y; y++){
				
				//access the pattern and gets the value for the bit representing the current step
				bit = (transd_array->transd_ptr + x * ARRAY_SIZE_Y + y)->pattern & (1 << state.currStep);
				//gets the pin that the transducer is connected to
				pin = (transd_array->transd_ptr + x * ARRAY_SIZE_Y + y)->port_pin;
				
				//updates only the current pin
				//the bank and bit of each pin is available on the PINS constant indexed by pin number
				*(PINS[pin].bank_ptr) |= bit << *(PINS[pin].bank_ptr);
							
			}
		}
		
		state.currStep < ARRAY_PHASERES ? state.currStep = 0 : state.currStep++;
	//}
	
}

uint8_t parseInput () {
	
	char buffer[64];
	uint8_t buffSize, start, end;
	/*
	char *buffPtr;
	uint8_t n;
	*/
	
	if(Serial.available()) {
		
		buffSize =  Serial.readBytes(buffer,63);
		buffer[buffSize] = '\0';
		buffSize++;
		
		start = end = 0;
		while (start < buffSize) {
						
			while( buffer[start + end] != ' ' && buffer[start + end] != '\0' ) {
				end++;
			}
			buffer[start + end] = '\0';
			parseInputToken(&buffer[start]);
			start += end + 1; end = 0;
		}
		
		return 1;
	}
	
	return 0;
	
		/*
		start = 0; end = start; 
		while( buffer[start + end] != ' ' && buffer[start + end] != '\0' ) {
			end++;
		}
		buffer[start + end] = '\0';
		focusTarget.x = atoi(&buffer[start]);
		
		if( start + end < buffSize ) {
			start += end + 1; end = 0;
			while( buffer[start + end] != ' ' && buffer[start + end] != '\0' ) {
				end++;
			}
			buffer[start + end] = '\0';
			focusTarget.y = atoi(&buffer[start]);
		}
		
		if( start + end < buffSize ) {
			start += end + 1; end = 0;
			while( buffer[start + end] != ' ' && buffer[start + end] != '\0' ) {
				end++;
			}
			buffer[start + end] = '\0';
			focusTarget.z = atoi(&buffer[start]);
		}
		*/
		/*
		buffPtr = &buffer; n = 0; 
		while( *(buffPtr + n) != ' ' && *(buffPtr + n) != '\0' ) {
			n++;
		}
		*(buffPtr + n) = '\0';
		focusTarget.x = atoi(buffPtr);
		
		buffPtr = (buffPtr + ++n); n = 0; 
		if( buffPtr - &buffer < buffSize ) {
			while( *(buffPtr + n) != ' ' && *(buffPtr + n) != '\0' ) {
				n++;
			}
			*(buffPtr + n) = '\0';
			focusTarget.y = atoi(buffPtr);
		}
		
		buffPtr = (buffPtr + ++n); n = 0; 
		if( buffPtr - &buffer < buffSize ) {
			while( *(buffPtr + n) != ' ' && *(buffPtr + n) != '\0' ) {
				n++;
			}
			*(buffPtr + n) = '\0';
			focusTarget.z = atoi(buffPtr);
		}
		*/
	
}

/*

*/
void parseInputToken (char *token) {
	
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
			state.isStop = 1;
			break;
		case 'a':
			state.isActiveStatic = 1;
			break;
		case 'f':
			state.isActiveStatic = 1;
			state.isFlat = 1;
			break;
		case 'm':
			state.isActiveMove = 1;
			break;
		case 'r':
			state.isActiveStatic = 1;
			state.isActiveMove = 1;
			break;
		case 'x':
			state.x = atoi(&token[1]);
			break;
		case 'y':
			state.y = atoi(&token[1]);
			break;
		case 'z':
			state.z = atoi(&token[1]);
			break;
		case 'd':
			state.d = atoi(&token[1]);
			break;
		case 's':
			state.s = atoi(&token[1]);
			break;
	}
}

void clearState () {
	
	state.lastx = state.x; state.lasty = state.y; state.lastz = state.z;
	
	state.isActiveMove = 0, state.isActiveStatic = 0, state.isStop = 0, state.isFlat = 0;
	//state.x = 0, state.y = 0, state.z = 0, state.d = 0, state.s = 0;
	state.currStep = 0;
	state.moveCurrStep = 0, state.moveNSteps = 0;
	
}

void printCommand () {
	
	if(state.isActiveMove){
		Serial.print("Move from ("); Serial.print(state.lastx); Serial.print(","); Serial.print(state.lasty); Serial.print(","); Serial.print(state.lastz); Serial.print(") to ("); Serial.print(state.x); Serial.print(","); Serial.print(state.y); Serial.print(","); Serial.print(state.z); Serial.print(") at "); Serial.print(state.s); Serial.print("mm/s with duty cycle of "); Serial.print(state.d);
		if(state.isActiveStatic) {
			 Serial.print(" then keep active");
		}
		else {
			 Serial.print(" then deactivate");
		}
	}
	else if(state.isActiveStatic) {
    if(state.isFlat) {
      Serial.print("Activate flat mode with duty cycle of "); Serial.print(state.d);
    }
		else {
		  Serial.print("Activate at ("); Serial.print(state.x); Serial.print(","); Serial.print(state.y); Serial.print(","); Serial.print(state.z); Serial.print(") with duty cycle of "); Serial.print(state.d);
		}
	}
	if(state.isFlat) {
		Serial.print("Activate flat mode with duty cycle of "); Serial.print(state.d);
	}
	if(state.isStop) {
		Serial.print("Deactivate");
	}
	Serial.print("\n");
	
}

void trand_array_load ( t_transd_array *transd_array ){
	
	uint8_t x, y;
	
	for(x = 0; x < ARRAY_SIZE_X; x++){
		for(y = 0; y < ARRAY_SIZE_Y; y++){

			transd_array_set( transd_array, x, y, ARRAY_CALIBRATION[x][y][0], ARRAY_CALIBRATION[x][y][1] );
		}
	}
}








