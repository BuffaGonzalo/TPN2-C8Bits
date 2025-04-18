/*! \CommunicationProtocol
 * \date 29/03/2025
 * \author Gonzalo Martin Buffa
 * \section 
 *
 * \section changelog - Registro de cambios
 *
 * |   Fecha    | Descripcion                                    |
 * |:----------:|:-----------------------------------------------|
 * |:06/04/2025:|:Andando HCSR04---------------------------------|

*/


/* Includes ------------------------------------------------------------------*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "hcsr04.h" //myLibrary
/* END Includes --------------------------------------------------------------*/

/* typedef -------------------------------------------------------------------*/
typedef union{
	struct{
		uint8_t bit7 : 1;
		uint8_t bit6 : 1;
		uint8_t bit5 : 1;
		uint8_t bit4 : 1;
		uint8_t bit3 : 1;
		uint8_t bit2 : 1;
		uint8_t bit1 : 1;
		uint8_t bit0 : 1;
	}bits;
	uint8_t bytes;
}_uFlag;

/**
 * @brief estructura para la recepci?n de datos por puerto serie
 * 
 */
typedef struct{
	uint8_t *buff;      /*!< Puntero para el buffer de recepci?n*/
	uint8_t indexR;     /*!< indice de lectura del buffer circular*/
	uint8_t indexW;     /*!< indice de escritura del buffer circular*/
	uint8_t indexData;  /*!< indice para identificar la posici?n del dato*/
	uint8_t mask;       /*!< m?scara para controlar el tama?o del buffer*/
	uint8_t chk;        /*!< variable para calcular el checksum*/
	uint8_t nBytes;     /*!< variable para almacenar el n?mero de bytes recibidos*/
	uint8_t header;     /*!< variable para mantener el estado dela MEF del protocolo*/
	uint8_t timeOut;    /*!< variable para resetear la MEF si no llegan m?s caracteres luego de cierto tiempo*/
	uint8_t isComannd;
}_sRx;

/**
 * @brief Estructura para la transmisi?n de datos por el puerto serie
 * 
 */
typedef struct{
    uint8_t *buff;      /*!< Puntero para el buffer de transmisi?n*/
    uint8_t indexR;     /*!<indice de lectura del buffer circular*/
    uint8_t indexW;     /*!<indice de escritura del buffer circular*/
    uint8_t mask;       /*!<m?scara para controlar el tama?o del buffer*/
    uint8_t chk;        /*!< variable para calcular el checksum*/
}_sTx;


/**
 * @brief Enumeraci?n para la maquina de estados
 * que se encarga de decodificar el protocolo
 * de comunicaci?n
 *  
 */
typedef enum{
    HEADER_U,
    HEADER_N,
    HEADER_E,
    HEADER_R,
    NBYTES,
    TOKEN,
    PAYLOAD
}_eDecode;


typedef enum{
	ONTRIGGER,
	OFFTRIGGER,
	UPFLANK,
	DOWNFLANK,
	IDLE
}_eDistance;

/**
 * @brief Enumeraci?n de los comandos del protocolo
 *
 * 
 */
typedef enum{
    ALIVE = 0xF0,
    FIRMWARE= 0xF1,
    GETDISTANCE = 0xA3,
	ACK = 0x0D,
	UNKNOWN = 0xFF
}_eCmd;

/**
 * 
 * @brief Uni?n ara la descomposici?n/composici?n de n?meros mayores a 1 byte
 * 
 */
typedef union{
    uint32_t    ui32;
    int32_t     i32;
    uint16_t    ui16[2];
    int16_t     i16[2];
    uint8_t     ui8[4];
    int8_t      i8[4];
}_uWord;

/**
 * @brief Estructura para manejar el tiempo
 * 
 */
typedef struct{
    int32_t    startTime;   /*!< Almacena el tiempo leido del timer*/
    uint16_t    interval;   /*!< intervalo de comparaci?n para saber si ya transcurrio el tiempo leido*/
    uint8_t     isRunnig;   /*!< Indica si el delay est? activo o no*/
}_delay_t;

/* END typedef ---------------------------------------------------------------*/

/* define --------------------------------------------------------------------*/
//Comunicaciones
#define RXBUFSIZE           256
#define TXBUFSIZE           256
#define GENERALTIME         10

//Botones
#define DEBOUNCE            40
#define HEARBEATIME         100
#define NUMBUTTONS          1
#define MASK				0x01

//Delcaraci?n de pines
#define LEDBUILTIN			PB5

#define ECHO				PB0 //D8 del Arduino
#define TRIG				PB1 //D9 del Arduino

//Banderas - flags
#define RESETFLAGS          flags.bytes 
#define OKDISTANCE			flags.bits.bit0
#define TRIGGERDONE			flags.bits.bit1

#define NEWMEASURE			flags.bits.bit3
#define ECHOTIMEOUT			flags.bits.bit4
#define IS10MS				flags.bits.bit6
#define IS100MS				flags.bits.bit7




/* END define ----------------------------------------------------------------*/

/* hardware configuration ----------------------------------------------------*/

/* END hardware configuration ------------------------------------------------*/


/* Function prototypes -------------------------------------------------------*/
//CONEXION SERIAL - WIFI
/**
 * @brief Ejecuta las tareas del puerto serie Decodificaci?n/trasnmisi?n
 * 
 * @param dataRx Estructura de datos de la recepci?n
 * @param dataTx Estructura de datos de la trasnmisi?n
 * @param source Identifica la fuente desde donde se enviaron los datos
 */
void serialTask(_sRx *dataRx, _sTx *dataTx);

/**
 * @brief Recepci?n de datos por el puerto serie
 * 
 */
void onRxData();

/**
 * @brief Pone el encabezado del protocolo, el ID y la cantidad de bytes a enviar
 * 
 * @param dataTx Estructura para la trasnmisi?n de datos
 * @param ID Identificaci?n del comando que se env?a
 * @param frameLength Longitud de la trama del comando
 * @return uint8_t devuelve el Checksum de los datos agregados al buffer de trasnmisi?n
 */
uint8_t putHeaderOnTx(_sTx  *dataTx, _eCmd ID, uint8_t frameLength);

/**
 * @brief Agrega un byte al buffer de transmisi?n
 * 
 * @param dataTx Estructura para la trasnmisi?n de datos
 * @param byte El elemento que se quiere agregar
 * @return uint8_t devuelve el Checksum del dato agregado al buffer de trasnmisi?n
 */
uint8_t putByteOnTx(_sTx    *dataTx, uint8_t byte);

/**
 * @brief Agrega un String al buffer de transmisi?n
 * 
 * @param dataTx Estructura para la trasnmisi?n de datos
 * @param str String a agregar
 * @return uint8_t devuelve el Checksum del dato agregado al buffer de trasnmisi?n
 */
uint8_t putStrOntx(_sTx *dataTx, const char *str);

uint8_t getByteFromRx(_sRx *dataRx, uint8_t iniPos, uint8_t finalPos);

/**
 * @brief Decodifica la trama recibida
 * 
 * @param dataRx Estructura para la recepci?n de datos
 */
void decodeHeader(_sRx *dataRx);

/**
 * @brief Decodifica el comando recibido en la transmisi?n y ejecuita las tareas asociadas a dicho comando
 * 
 * @param dataRx Estructura para la recepci?n de datos
 * @param dataTx Estructura para la trasnmisi?n de datos
 */
void decodeCommand(_sRx *dataRx, _sTx *dataTx);
/**
 * @brief Env?a de manera autom?tica el alive
 * 
 */
void aliveAutoTask(_delay_t *aliveAutoTime);

//HEARTBEAT
/**
 * @brief Hearbeat, indica el funcionamiento del sistema
 * 
 * @param timeHeartbeat Variable para el intervalo de tiempo
 * @param mask Secuencia de encendido/apagado del led de Hearbeat
 */
void hearbeatTask(uint8_t index);

/**
 * @brief Configura el entervalo del Delay
 * 
 * @param delay     Estructura para manejar el tiempo
 * @param interval  intervalo de comparaci?n para saber si ya transcurrio el tiempo leido
 */
void delayConfig(_delay_t *delay, uint16_t  interval);

/**
 * @brief Lee el Delay para determinar si se ha cumplido el tiempo establecido
 * 
 * @param delay     Estructura para manejar el tiempo
 * @return uint8_t  Returna True o False de acuerdo a si se cumplio el tiempo o no
 */
uint8_t delayRead(_delay_t *delay);

/**
 * @brief PErmite modificar el intervalo establecido para un Delay y lo resetea
 * 
 * @param delay     Estructura para manejar el tiempo
 * @param interval  intervalo de comparaci?n para saber si ya transcurrio el tiempo leido
 */
void delayWrite(_delay_t *delay, uint16_t interval);


uint32_t millis(); //Contador de milisegundos

void do10ms();

void initUSART0();
void initTimer0();
void initTimer1();
void initPCINT0();
/* END Function prototypes ---------------------------------------------------*/

/* Global variables ----------------------------------------------------------*/

//VALOR FIRMWARE
const char firmware[] = "EX100923v01\n";

//MASCARAS
uint32_t heartBeatMask[] = {0x55555555, 0x1, 0x2010080, 0x5F, 0x5, 0x28140A00, 0x15F, 0x15, 0x2A150A08, 0x55F}; //IDLE, MODO1, MODO1(1-5), MODO1ON, MODO2, MODO2(1-5), MODO2ON, MODO3, MODO3(1-5), MODO3ON

//BANDERAS
volatile _uFlag  flags; //declaración de esta forma debido a su uso en la interrupción

//COMUNICACION SERIAL - WIFI
_uWord myWord;

_sTx dataTx;

volatile _sRx dataRx;

volatile uint8_t buffRx[RXBUFSIZE];

uint8_t buffTx[TXBUFSIZE];

uint8_t globalIndex, index2;

_delay_t generalTime;

volatile uint32_t MillisCounter;

volatile uint32_t startTime = 0;
volatile uint32_t endTime = 0;

volatile uint8_t time10ms = 10;
uint8_t time100ms = 10;


//uint8_t triggerDone = 0;
uint32_t distance = 0;
uint32_t echoTimeout = 0;
_eDistance hcSr04Modes;

/* END Global variables ------------------------------------------------------*/

/* Function prototypes user code ----------------------------------------------*/

//CONEXION SERIAL - WIFI
void serialTask(_sRx *dataRx, _sTx *dataTx)
{
	
    if(dataRx->isComannd){
        dataRx->isComannd=0; //false reemplazado por 0
		
        decodeCommand(dataRx,dataTx);
    }

	//Revisamos si llego un nuevo dato en el serie
	if(delayRead(&generalTime)){
		if(dataRx->header){
			dataRx->timeOut--;
			if(!dataRx->timeOut)
			dataRx->header = HEADER_U;
		}
	}
	
    if(dataRx->indexR!=dataRx->indexW){
		
        decodeHeader(dataRx);
    } 	

    if(dataTx->indexR!=dataTx->indexW)
	{
		if(UCSR0A & (1 << UDRE0))
		{
			UDR0=dataTx->buff[dataTx->indexR++];
			dataTx->indexR &=dataTx->mask;
		}
    }

}

uint8_t putHeaderOnTx(_sTx  *dataTx, _eCmd ID, uint8_t frameLength)
{
    dataTx->chk = 0;
    dataTx->buff[dataTx->indexW++]='U';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]='N';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]='E';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]='R';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]=frameLength+1;
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]=':';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]=ID;
    dataTx->indexW &= dataTx->mask;
    dataTx->chk ^= (frameLength+1);
    dataTx->chk ^= ('U' ^'N' ^'E' ^'R' ^ID ^':') ;
    return  dataTx->chk;
}

uint8_t putByteOnTx(_sTx *dataTx, uint8_t byte)
{
    dataTx->buff[dataTx->indexW++]=byte;
    dataTx->indexW &= dataTx->mask;
    dataTx->chk ^= byte;
    return dataTx->chk;
}

uint8_t putStrOntx(_sTx *dataTx, const char *str)
{
    globalIndex=0;
    while(str[globalIndex]){
        dataTx->buff[dataTx->indexW++]=str[globalIndex];
        dataTx->indexW &= dataTx->mask;
        dataTx->chk ^= str[globalIndex++];
    }
    return dataTx->chk ;
}

uint8_t getByteFromRx(_sRx *dataRx, uint8_t iniPos, uint8_t finalPos)
{
    uint8_t getByte;
    dataRx->indexData += iniPos;
    dataRx->indexData &=dataRx->mask;
    getByte = dataRx->buff[dataRx->indexData];
    dataRx->indexData += finalPos;
    dataRx->indexData &=dataRx->mask;
    return getByte;
}

void decodeHeader(_sRx *dataRx)
{
    uint8_t auxIndex=dataRx->indexW;
    while(dataRx->indexR != auxIndex){
        switch(dataRx->header)
        {
            case HEADER_U:
                if(dataRx->buff[dataRx->indexR] == 'U'){
                    dataRx->header = HEADER_N;
                    dataRx->timeOut = 5;
                }
            break;
            case HEADER_N:
                if(dataRx->buff[dataRx->indexR] == 'N'){
                    dataRx->header = HEADER_E;
                }else{
                    if(dataRx->buff[dataRx->indexR] != 'U'){
                        dataRx->header = HEADER_U;
                        dataRx->indexR--;
                    }
                }
            break;
            case HEADER_E:
                if(dataRx->buff[dataRx->indexR] == 'E'){
                    dataRx->header = HEADER_R;
                }else{
                    dataRx->header = HEADER_U;
                    dataRx->indexR--;
                }
            break;
            case HEADER_R:
                if(dataRx->buff[dataRx->indexR] == 'R'){
                    dataRx->header = NBYTES;
                }else{
                    dataRx->header = HEADER_U;
                    dataRx->indexR--;
                }
            break;
            case NBYTES:
                dataRx->nBytes=dataRx->buff[dataRx->indexR];
                dataRx->header = TOKEN;
            break;
            case TOKEN:
                if(dataRx->buff[dataRx->indexR] == ':'){
                    dataRx->header = PAYLOAD;
                    dataRx->indexData = dataRx->indexR+1;
                    dataRx->indexData &= dataRx->mask;
                    dataRx->chk = 0;
                    dataRx->chk ^= ('U' ^'N' ^'E' ^'R' ^dataRx->nBytes ^':') ;
                }else{
                    dataRx->header = HEADER_U;
                    dataRx->indexR--;
                }
            break;
            case PAYLOAD:
                dataRx->nBytes--;
                if(dataRx->nBytes>0){
                   dataRx->chk ^= dataRx->buff[dataRx->indexR];
                }else{
                    dataRx->header = HEADER_U;
                    if(dataRx->buff[dataRx->indexR] == dataRx->chk)
                        dataRx->isComannd = 1; //True reemplazado por 1
                }
            break;
            default:
                dataRx->header = HEADER_U;
            break;
        }
        dataRx->indexR++;
        dataRx->indexR &= dataRx->mask;
    }
}

void decodeCommand(_sRx *dataRx, _sTx *dataTx)
{
    switch(dataRx->buff[dataRx->indexData]){
        case ALIVE:
            putHeaderOnTx(dataTx, ALIVE, 2);
            putByteOnTx(dataTx, ACK );
            putByteOnTx(dataTx, dataTx->chk);
        break;
        case FIRMWARE:
            putHeaderOnTx(dataTx, FIRMWARE, 12);
            putStrOntx(dataTx, firmware);
            putByteOnTx(dataTx, dataTx->chk);
        break;
        case GETDISTANCE:
			myWord.ui32 = distance; //Cast para indicar que trate la union como uint_8t
			putHeaderOnTx(dataTx, GETDISTANCE, 5);
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );
			putByteOnTx(dataTx, myWord.ui8[2] );
			putByteOnTx(dataTx, myWord.ui8[3] );
			
			putByteOnTx(dataTx, dataTx->chk);		
        break;
        default:
            putHeaderOnTx(dataTx, (_eCmd)dataRx->buff[dataRx->indexData], 2);
            putByteOnTx(dataTx,UNKNOWN);
            putByteOnTx(dataTx, dataTx->chk);
        break;
        
    }
}

//HEARTBEAT
void hearbeatTask(uint8_t index)
{
	static uint8_t times=0;
	if(~heartBeatMask[index] & (1<<times))
	{
		PORTB |= (1<<LEDBUILTIN); ///LedOn
	} else{
		PORTB &= ~(1<<LEDBUILTIN); //LedOff
	}
		
	times++;
	times &= 31; //control de times
}

//DELAYS
void delayConfig(_delay_t *delay, uint16_t  interval){
	delay->interval =interval;
	delay->isRunnig = 0;
}

uint8_t delayRead(_delay_t *delay){
	uint8_t timeReach=0;
	if(!delay->isRunnig){
		delay->isRunnig = 1;
		delay->startTime = millis();
		}else{
		if((millis()-delay->startTime)>=delay->interval){
			timeReach = 1;
			delay->isRunnig = 0;
		}
	}
	return timeReach;
}

void delayWrite(_delay_t *delay, uint16_t interval){
	delay->interval = interval;
	delay->isRunnig = 0;
}

uint32_t millis()
{
	uint32_t savedMillis;
	sei();
		savedMillis = MillisCounter;	
	cli();
	return savedMillis;
}

void initUSART0()//USART -> 115200 bd
{
	UCSR0A = 0xFE;
	UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = 0x06;
	UBRR0L = 16;
	UBRR0H = 0;
}

void initTimer0()
{
	//Timer -> modo CTC - fclk/64 - CR0A: 249 -> 1ms
	TCCR0A = 0x02;
	TCCR0B = 0b00000011; //Seteo del prescaler
	TCNT0 = 0x00;
	TIFR0 = 0x07;
	OCR0A = 249; //Comienza en 0 
	TIMSK0 = 0x02;
}

void initTimer1()
{
	//Configuraci?n para funcionamiento normal - fclk/8 -> 2Mhz - 0,5uS
	TCNT1 = 0;
	TCCR1A = 0;
	TCCR1B=(1<<ICNC1) | (1<<ICES1) | (0<<CS12) | (1<<CS11) | (0<<CS10); //Activamos la cancelacion de ruido y el preescaler a 8
	TIMSK1 = (1 << OCIE1B) | (1 << ICIE1); //Habilitamos la interrupci?n por captura de entrada ICIE1
	TIFR1 = TIFR1; //Hacemos 0 las banderas
}

void initPCINT0()
{
	DDRB &= ~((1<<PORTB2));
	PORTB |= (1<<PORTB2);
	PCIFR |= (1<<PCIF0);
	PCICR |= (1 << PCIE0); // Habilita interrupciones del grupo PCINT0 (PB0 - PB7)
	PCMSK0 |= (1<< PCINT2); //Habilita la interrupción para el pin PB2
}

void do10ms()
{
	if (time10ms == 0)
	{
		IS10MS = 1;
		time10ms = 10;
	}
}

void do100ms()
{
	if(time100ms == 0)//reiniciamos el valor
	{
		IS100MS = 1; //Activamos la bandera del pasode 100ms
		time100ms = 10;
	}
	if(IS10MS==1)
	{
		time100ms--;
		IS10MS = 0;//limpiamos la bandera de 10ms
		IS100MS = 0;//limpiamos la bandera de 100ms
	}
}

void hcsr04Control(){	
	if(NEWMEASURE)
	{
		NEWMEASURE=0;
		hcSr04Modes=ONTRIGGER;
	}
	switch(hcSr04Modes){
		case ONTRIGGER://ejecucion cada 100ms y que no haya ocurrido un trigger
			PORTB |= (1<<TRIG); //Comenzamos el pulso
			OCR1B = TCNT1 + 20; //Espera de 10us en alto
			echoTimeout=millis();
			hcSr04Modes=IDLE;
			break;	
		case OFFTRIGGER:
			TRIGGERDONE = 1; //Control de que no salte de nuevo el trigger
			PORTB &= ~(1<<TRIG); //Finalizamos el pulso		
			hcSr04Modes=IDLE;
			break;
		case UPFLANK:
			TCCR1B &= ~(1 << ICES1);  // Configurar a 0 valor de ICES1 para flanco descendente
			hcSr04Modes=IDLE;
			break;
		case DOWNFLANK:
			OKDISTANCE = 1;
			TCCR1B |= (1 << ICES1);   // Resetear para proximo flanco ascendente
			hcSr04Modes=IDLE;
			break;
		case IDLE:
			break;
	}	
	if((echoTimeout-millis())>100)
	{
		ECHOTIMEOUT=1;
		return;
	}
}


//INTERRUPTS
// Rutina de servicio de interrupci?n (ISR) para recepci?n USART
ISR(USART_RX_vect) // Vector 19 (0x0024)
{	
	dataRx.buff[dataRx.indexW++] = UDR0;
	dataRx.indexW &= dataRx.mask;
}

// Rutina de servicio de interrupci?n (ISR) para Timer0 Compare Match A
ISR(TIMER0_COMPA_vect) // Vector 15 (0x001C)
{
	MillisCounter++;
	time10ms--;
}

//Interrupcion cada 10us
ISR(TIMER1_COMPB_vect)
{		
	hcSr04Modes=OFFTRIGGER; //Bandera para seguir adelante con la ejecuci?n del codigo
}

ISR(PCINT0_vect)
{
	if (PINB & (1<<PINB2)) // Flanco ascendente (inicio echo)
	{
		hcSr04Modes=UPFLANK;
		startTime = TCNT1; //guardamos el tiempo en ese instante
	}
	else // Flanco descendente (final eco)
	{
		PORTB |= (1<<LEDBUILTIN); //ledOn
		hcSr04Modes=DOWNFLANK;
		endTime = TCNT1;
	}
}

/*
ICES = 0 -> falling (negative) edge 
ICES = 1 -> rising (positive) edge
*/
//Captura del flanco 
ISR(TIMER1_CAPT_vect){
}



/* END Function prototypes user code ------------------------------------------*/

int main()
{
	cli();
    //INICIALIZAMOS VARIABLES
    dataRx.buff = (uint8_t *)buffRx;
    dataRx.indexR = 0;
    dataRx.indexW = 0;
    dataRx.header = HEADER_U;
    dataRx.mask = RXBUFSIZE - 1;
	dataRx.isComannd = 0;
	
    dataTx.buff = buffTx;
    dataTx.indexR = 0;
    dataTx.indexW = 0;
    dataTx.mask = TXBUFSIZE -1;
    RESETFLAGS = 0;
	
	MillisCounter = 0;
	
	//Inicializamos los pines salida/entrada
	DDRB |= (1<<LEDBUILTIN); //Seteo del led como salida	
	DDRB |= (1<<TRIG) | (0<<ECHO);//Definimos el trig como salida y el echo como entrada
	
	//Seteamos el valor del led
	PORTB &= ~(1<<LEDBUILTIN); //ledOff
	//PORTB |= (1<<LEDBUILTIN); //ledOn
	PORTB &= ~(1<<TRIG);//Inicializamos con el trigger apagado

	//Inicializamos interrupciones
	initTimer0();
	initTimer1();
	initUSART0();
	initPCINT0();
	
	initHcSr04(); //Inicializamos el sensor
	hcSr04Modes = IDLE;
	
	//Configuracion de temporizadores
	delayConfig(&generalTime,GENERALTIME);
	
	sei();
/* Local variables -----------------------------------------------------------*/

/* END Local variables -------------------------------------------------------*/


/* User code -----------------------------------------------------------------*/	
    while(1){
		do10ms();
		do100ms(); //función de librería
				
		hcSr04Task(hcsr04Control, (uint8_t*)&flags);
		getDistance(&distance, startTime, endTime);
        //CONEXIONES SERIAL Y WIFI
        serialTask((_sRx *)&dataRx, &dataTx); //serialTask -> conexion serial
    }
/* END User code -------------------------------------------------------------*/
}