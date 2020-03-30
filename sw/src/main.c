
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

#define TRANS_DIAMETER 16 //diameter of the element in millimeters (total leght of the array cant exceed 255 millimeters)
#define TRANS_SEPARATION 2 //distance between two consecutive elements in the array in millimeters (total leght of the array cant exceed 255 millimeters)
#define ARRAY_SIZE_X 8 //number of transducers of the array in the x dimension
#define ARRAY_SIZE_Y 8 //number of transducers of the array in the y dimension
#define ARRAY_PHASERES 10 //number of transducers of the array in the y dimension (max 16 bits)
#define TRAJ_RES 1 //trajectory maximum resolution in millimeters
#define TRAJ_MAXSTEPS 255 //maximum steps of the trajectory (max 255)

/* MACROS */

#define MSK(b) (1 << b) //creates a mask with the bth bit high

/* PROTOTYPES */

void trand_array_load ( t_transd_array *transd_array );
ISR( TIMER4_COMPA_vect );
uint8_t parseInput ();
void parseInputToken (char *token);
void clearState ();
void printCommand ();
void calcStep (uint8_t step_idx, const uint8_t duty_cycle, const uint8_t focus_x, const uint8_t focus_y, const uint8_t focus_z );

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
	uint8_t bit_msk;
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

struct s_state state;

t_transd_array *transd_array = NULL;

//uint8_t traj_steps[TRAJ_MAXSTEPS][ARRAY_SIZE_X][ARRAY_SIZE_Y];
uint8_t outPortBuffer[TRAJ_MAXSTEPS][ARRAY_PHASERES][10]; //buffers the ports state for each coordinate (x,y,z) of the movement, for each slice of the wave period, for each PORT

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
		
	//if(state.isActiveStatic && !state.isActiveMove) {
			
		// Just copy the port state from the buffer
		PORTA = outPortBuffer[step_idx][phaseStep][0];
		PORTB = outPortBuffer[step_idx][phaseStep][1];
		PORTC = outPortBuffer[step_idx][phaseStep][2];
		PORTD = outPortBuffer[step_idx][phaseStep][3];
		//PORTE = 0;
		PORTF = outPortBuffer[step_idx][phaseStep][4];
		PORTG = outPortBuffer[step_idx][phaseStep][5];
		PORTH = outPortBuffer[step_idx][phaseStep][6];
		//PORTI = 0;
		PORTJ = outPortBuffer[step_idx][phaseStep][7];
		PORTK = outPortBuffer[step_idx][phaseStep][8];
		PORTL = outPortBuffer[step_idx][phaseStep][9];
		
		state.currStep < ARRAY_PHASERES ? state.currStep++ : state.currStep = 0;
	//}
	
}

void calcStep (uint8_t step_idx, const uint8_t duty_cycle, const uint8_t focus_x, const uint8_t focus_y, const uint8_t focus_z ) {
	
	uint8_t x, y, phaseStep, bit, pin, portIdx;
	
	for(phaseStep = 0; phaseStep < ARRAY_PHASERES; phaseStep++){
		outPortBuffer[step_idx][phaseStep][0] = 0x00; //PORTA
		outPortBuffer[step_idx][phaseStep][1] = 0x00; //PORTB
		outPortBuffer[step_idx][phaseStep][2] = 0x00; //PORTC
		outPortBuffer[step_idx][phaseStep][3] = 0x00; //PORTD
		outPortBuffer[step_idx][phaseStep][4] = 0x00; //PORTF
		outPortBuffer[step_idx][phaseStep][5] = 0x00; //PORTG
		outPortBuffer[step_idx][phaseStep][6] = 0x00; //PORTH
		outPortBuffer[step_idx][phaseStep][7] = 0x00; //PORTJ
		outPortBuffer[step_idx][phaseStep][8] = 0x00; //PORTK
		outPortBuffer[step_idx][phaseStep][9] = 0x00; //PORTL
	}
	
	transd_array_calcfocus( transd_array, duty_cycle, focus_x, focus_y, focus_z );
	
	for(x = 0; x < ARRAY_SIZE_X; x++){
		for(y = 0; y < ARRAY_SIZE_Y; y++){
			
			//gets the pin that the transducer is connected to
			pin = (transd_array->transd_ptr + x * ARRAY_SIZE_Y + y)->port_pin;
			
			if(PINS[pin].bank_ptr == &PORTA) portIdx = 0;
			if(PINS[pin].bank_ptr == &PORTB) portIdx = 1;
			if(PINS[pin].bank_ptr == &PORTC) portIdx = 2;
			if(PINS[pin].bank_ptr == &PORTD) portIdx = 3;
			if(PINS[pin].bank_ptr == &PORTF) portIdx = 4;
			if(PINS[pin].bank_ptr == &PORTG) portIdx = 5;
			if(PINS[pin].bank_ptr == &PORTH) portIdx = 6;
			if(PINS[pin].bank_ptr == &PORTJ) portIdx = 7;
			if(PINS[pin].bank_ptr == &PORTK) portIdx = 8;
			if(PINS[pin].bank_ptr == &PORTL) portIdx = 9;
			
			for(phaseStep = 0; phaseStep < ARRAY_PHASERES; phaseStep++){
			
				//access the pattern and gets the value for the bit representing the current step
				bit = (transd_array->transd_ptr + x * ARRAY_SIZE_Y + y)->pattern & (1 << phaseStep);
				
				//updates only the current pin
				if(bit) {
					outPortBuffer[step_idx][phaseStep][portIdx] |= PINS[pin].bit_msk;
				}
				/* no need to set as low because all the bits of the buffer are set to zero
				else {
					outPortBuffer[step_idx][phaseStep][portIdx] &= ~PINS[pin].bit_idx;
				}
				*/
			}
		}
	}
} //calcStep

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
		
} //parseInput

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
} //parseInputToken

void clearState () {
	
	state.lastx = state.x; state.lasty = state.y; state.lastz = state.z;
	
	state.isActiveMove = 0, state.isActiveStatic = 0, state.isStop = 0, state.isFlat = 0;
	//state.x = 0, state.y = 0, state.z = 0, state.d = 0, state.s = 0;
	state.currStep = 0;
	state.moveCurrStep = 0, state.moveNSteps = 0;
	
} //clearState

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
	
} //printCommand

void trand_array_load ( t_transd_array *transd_array ){
	
	uint8_t x, y;
	
	for(x = 0; x < ARRAY_SIZE_X; x++){
		for(y = 0; y < ARRAY_SIZE_Y; y++){

			transd_array_set( transd_array, x, y, ARRAY_CALIBRATION[x][y][0], ARRAY_CALIBRATION[x][y][1] );
		}
	}
} /trand_array_load








