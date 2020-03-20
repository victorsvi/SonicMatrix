
#include <stdlib.h>
#include <stdint.h>
#include <EEPROM.h>
#include <Arduino.h>
#include "Ultrasonic.h"

#define TRANS_DIAMETER 16 //diameter of the element in milimeters
#define TRANS_SEPARATION 2 //distance between two consecutive elements in the array in milimeters
#define ARRAY_SIZE_X 8 //number of transducers of the array in the x dimension
#define ARRAY_SIZE_Y 8 //number of transducers of the array in the y dimension

struct s_memory {
	uint8_t id;
	uint8_t pin;
	uint8_t phase;
}

union u_memory {
	struct s_memory memory;
	uint8_t *buf;
}

t_array *array;

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

void setup () {
	
	array = Array_init( ARRAY_SIZE_X, ARRAY_SIZE_Y, TRANS_DIAMETER, TRANS_SEPARATION );
	if( array == NULL ) {
		Serial.println("E001 - Error creating the array");
	}
	
}

void loop () {
	
	
}
/*
void Array_load ( t_array *array ) {
	
	if( array != NULL ) {
		
		
	}
}
*/










