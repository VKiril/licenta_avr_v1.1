#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#define F_CPU 8000000UL

#define STX 0x30
#define ETX 0x31

#define IN_STX1_POS          0
#define IN_STX2_POS          1
#define IN_LENGTH_BIT_POS    2
#define IN_CMD_POS           3
#define IN_CRC_POS           4
#define IN_ETX1_POS          5
#define IN_ETX2_POS          6


#define OUT_STX1_POS         0
#define OUT_STX2_POS         1
#define OUT_LENGTH_BIT_POS   2
#define OUT_CMD_POS          3
#define OUT_BIT0_POS         4
#define OUT_BIT1_POS         5
#define OUT_CRCL_POS         6
#define OUT_CRCH_POS         7
#define OUT_ETX1_POS         8
#define OUT_ETX2_POS         9


#define INCOMMING_ARRAY_LENGTH   7
#define OUT_COMMING_ARRAY_LENGTH 10

//cmd commands

#define START_CONVERSION_AND_TRANSMITING 0x32
#define STOP_CONVERSION_AND_TRANSMITING  0x33

//aliases
#define INCOMMING_ARRAY_ALIAS    0x04
#define OUT_COMMING_ARRAY_ALIAS  0x05

int  counter   = 0;
char startFlag = 0 ; 

char higgerBit = 0; 
char lowerBit  = 0 ; 

//                                               |stx1|stx2|length|cmd|crc|etx1|etx2|                 
char inCommingArray [INCOMMING_ARRAY_LENGTH]   = { 0,    0,    0,   0,  0,  0,    0  }      ;
//                                               |stx1|stx2|length|cmd|bit0|bit1|crcL|crchH|etx1|etx2|
char outCommingArray[OUT_COMMING_ARRAY_LENGTH] = { 0,    0,    0,   0,   0,   0,   0,    0,  0,   0  }	;
//functions prototypes

void _portInit(void);
void _ADCinit(void);
void _timer1Init(void);
void _uartInit(void);
void _construct(void);
void _externInterruptInit(void);
void adcConversion(void);
void ping(void);
void clearCommand(void);
void receiveArray(char);

void send(void);
void stopConversion(void);
void startConversion(char);

void initArray(char );

ISR(ADC_vect){
	startFlag = 1 ; 
	adcConversion();
	UDR = 0xFC;	
}

ISR(TIMER0_OVF_vect){
	//
}

ISR(INT0_vect){
	PORTB = 0xFF;
	//send();
}

ISR(INT1_vect){
	PORTB = 0x00;
	counter = 0 ;
}

ISR(USART_TXC_vect){
	send();
	//char temp = UDR ; 
	//UDR = temp ; 
}

ISR(USART_RXC_vect){
	PORTA = UDR ; 
	receiveArray(UDR);
	//char temp = UDR ; 
	//UDR  = temp ; 
}
char acdConverted = 0; 
void adcConversion(void){	
	DDRC = 0XFF;
	if(startFlag == 1){
		PORTC=0b11111110;
		acdConverted = 1 ;
		lowerBit  = ADCL ;
		higgerBit = ADCH;
		
		int crc = lowerBit + higgerBit + inCommingArray[IN_CMD_POS] + OUT_COMMING_ARRAY_LENGTH  ;		
		
		outCommingArray [OUT_CMD_POS ] = inCommingArray[IN_CMD_POS] ;
		outCommingArray [OUT_BIT0_POS] = lowerBit ;
		outCommingArray [OUT_BIT1_POS] = higgerBit ;
		outCommingArray [OUT_CRCH_POS] = (char) crc >> 8 ;
		outCommingArray [OUT_CRCL_POS] = (char) crc ;
		
		send();
	} else {
		PORTC=0b00000001;
	}
}

void initArray(char array){
	//if(array == OUT_COMMING_ARRAY_ALIAS ){
		outCommingArray [OUT_LENGTH_BIT_POS] = OUT_COMMING_ARRAY_LENGTH ;
		outCommingArray [OUT_STX1_POS]  = STX ; 
		outCommingArray [OUT_STX2_POS]  = STX ; 
		outCommingArray [OUT_ETX1_POS]  = ETX ; 
		outCommingArray [OUT_ETX2_POS]  = ETX ; 
	//} 
}

char  firstCommingBit = 0 ; 
char secondCommingBit = 0 ; 

char  firstCommingBitFlag = 0 ; 
char secondCommingBitFlag = 0 ; 

char consecutiveCounter = 0; 
char consecutiveCounterBuffer = 0 ;


void receiveArray(char bit){
	
	consecutiveCounter ++ ; 
	PORTC = inCommingArray[IN_CMD_POS];
	
	if(firstCommingBitFlag == 0 && bit == ETX ){
		firstCommingBit = bit ;		
		firstCommingBitFlag = 1 ; 
		consecutiveCounterBuffer = consecutiveCounter ; 
		PORTB = 0b11111000;
		return ; 
	}
	
	if(secondCommingBitFlag == 0 && bit == ETX && consecutiveCounterBuffer == (consecutiveCounter -1)  ){
		secondCommingBit = bit ; 
		secondCommingBitFlag = 1; 
		PORTB = 0b11111100;
		return ;  		
	} 
	
	if(firstCommingBitFlag == 1 && secondCommingBitFlag == 0){
		firstCommingBitFlag = 0 ; 
		secondCommingBitFlag = 0; 
		consecutiveCounter = 0 ; 
		consecutiveCounterBuffer = 0 ; 
		PORTB = 0b10101010 ; 
		return ; 
	}
	
	if( secondCommingBitFlag == 1 && firstCommingBitFlag == 1 ){
		PORTB = 0b11111110;
		for (int i = 0 ; i < INCOMMING_ARRAY_LENGTH-2 ; i ++ ){
			inCommingArray[i+1] = inCommingArray[i] ;		
		}
		
		inCommingArray[0] = bit ; 
		char crc  = 0 ;
	
		/*for(int i = 1 ; i < INCOMMING_ARRAY_LENGTH-2 ; i++){
			crc += inCommingArray[i] ; 
		}*/
		
		if(	inCommingArray[IN_STX1_POS] == STX && inCommingArray[IN_STX2_POS] == STX && consecutiveCounter == INCOMMING_ARRAY_LENGTH  ){
			
			PORTB = inCommingArray[IN_CMD_POS]  ;
			consecutiveCounter = 0 ; 
			if( inCommingArray[IN_CMD_POS] == START_CONVERSION_AND_TRANSMITING ){
				PORTB = 0x0f;
				startConversion(inCommingArray[IN_CMD_POS]);
				clearCommand();
				return ; 
			}
			if( inCommingArray[IN_CMD_POS] == STOP_CONVERSION_AND_TRANSMITING  ){
				PORTB = 0xf0;
				stopConversion();
				clearCommand();
				return ; 
			}
		} 
		
		
		
	}
	if(  consecutiveCounter >= INCOMMING_ARRAY_LENGTH ){
		clearCommand();
	}
		
	
}
void clearCommand(void){
	firstCommingBitFlag = 0 ;
	secondCommingBitFlag = 0;
	consecutiveCounter = 0 ;
	consecutiveCounterBuffer = 0 ;
	
	for(int i = 0 ; i > INCOMMING_ARRAY_LENGTH ; i ++){
		inCommingArray[i] = 0 ;
	}
}

void startConversion(char cmd){
	UCSRC |= (1<<URSEL);
	startFlag = 1 ; 
	send();
}

void stopConversion(void){
	UCSRB &=~(1<<UDRIE);
	startFlag = 0 ; 
}

void send(void)
{
	if( acdConverted == 1 )
	{
		if(counter < OUT_COMMING_ARRAY_LENGTH){
			UCSRB |=(1<<TXCIE);
			UDR = outCommingArray[counter] ;			
			counter++;
		} else {
			UCSRB &=~(1<<TXCIE);
			counter = 0;
		}
	}
}

void ping(void){
	UCSRC |= (1<<URSEL);
	UDR = 0xff;
	UCSRC &= ~(1<<URSEL);
}




////////////////////////////////////////////////////////////////////////////////////////////////////////////     MAIN    //////
int main(void){
	
	_construct();	
	sei();
	ping();
	initArray(OUT_COMMING_ARRAY_ALIAS);
	
    while(1) 
	{ 	
		;	
    }
	return 1;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////     END MAIN   /////////   











void _construct(void){
	_portInit   ();
	_uartInit   ();
	_timer1Init ();
	//_externInterruptInit();
	_ADCinit    ();
}

void _portInit(void){
	//adc port
	PORTA = 0X00;
	DDRA = 0XFF;
	
	//uart port
	PORTD = 0b00111100;
	DDRD  = 0b11111111;
	
	//led port
	PORTB = firstCommingBitFlag;
	DDRB = 0xFF;
}

void _ADCinit(void){	
	ADMUX  |= ( 1<<REFS1 ) | ( 1<<REFS0 ) ;//Internal 2.56V Voltage Reference with external capacitor at AREF pin
	ADCSRA |= ( 1<<ADEN  ) | ( 1<<ADSC  ) | ( 1<<ADIE ) | ( 1<<ADPS2 ) | ( 1<<ADPS1 ) | ( 1 << ADIF );//Division Factor 64 frequency 125000Hz	
	SFIOR  |= ( 1<<ADTS1 ) | ( 1<<ADTS0 ) ;// trigger Timer/Counter0 Overflow
}

void _timer1Init(void){
	TCCR0 |= ( 1<<CS01)|(1<<CS00);//division factor 64
	TIMSK |= ( 1<<TOIE0) ;
	OCR0 = 255 ; 
}

void _uartInit(void){
	int baud = 12 ;
	UBRRH = (unsigned char)(baud>>8);
	UBRRL = (unsigned char) baud;
	UCSRB |= (1<<RXCIE)|(1<<TXCIE)|(1<<RXEN)|(1<<TXEN)|(1<<RXB8)|(1<<TXB8)|(0<<UDRIE) ;
	UCSRC |= (1<<URSEL)|(0<<USBS) |(1<< UCSZ1)|(1<< UCSZ0);//
}

void _externInterruptInit(void){
	//The falling edge of INT0 generates an interrupt request.
	MCUCR |= (1<<ISC01);
	GICR  |= (1<<INT0) ;
	GIFR  |= (1<<INTF0) ;

	MCUCR |= ( 1 << ISC11 ) ;//falling edge front
	GICR  |= ( 1 << INT1  ) ;//int1
	GIFR  |= ( 1 << INTF1 ) ;//int1_vect
}
