//Neil Funk, ECE554 Summer 2011
//Fuses for ATmega8: (external crystal)
// bad fuses: l:e0,h:d9 - "bricked; needs external clock source on xtal1
// good fuses: -U lfuse:w:0x1f:m -U hfuse:w:0xc9:m 
//	CKSEL=1111, SUT=01, CKOPT=1, BOD enabled @ 4V
//CKOPT allows high freq by giving wider voltage swing on clk.

#define F_CPU 8000000UL //8MHz
#define USART_BAUDRATE 19200 //For serial communications, also default baud of bluetooth module.

#include <inttypes.h>
#include <avr/io.h>
#include <stdio.h> // sprintf()
#include <util/delay.h> // _delay_ms()
#include <avr/interrupt.h>
#include <util/atomic.h> // Requires compiler option --std=c99 or --std=gnu99
#include <avr/wdt.h> // watchdog timer
//#include <avr/eeprom.h>

// _delay_ms function from util/delay.h can ONLY take a constant,
// so use this to acheive a variable delay.
unsigned char _delayx = 0;
#define delay100ms(x) for(_delayx = 0; _delayx < (x); _delayx++) _delay_ms(100);

// Convenience macros for set bit and clear bit
#define sbi(port, bit) (port) |= (1 << (bit))
#define cbi(port, bit) (port) &= ~(1 << (bit))

// Unused (so far) interrupts available on the atmega8:
//ISR(ADC_vect) // ADC Conversion Complete
//ANA_COMP_vect
//EE_RDY_vect
//INT1_vect 
//SPM_RDY_vect
//TIMER0_OVF_vect
//TIMER1_CAPT_vect
//TIMER1_COMPA_vect, TIMER1_COMPB_vect
//TIMER1_OVF_vect
//TIMER2_COMP_vect
//TIMER2_OVF_vect
//TWI_vect


//GLOBALS
volatile unsigned int exhaustTemp;
volatile unsigned int roomTemp;
volatile unsigned int roomTempApprox; // degF conversion
volatile unsigned char fanSpeed;
volatile unsigned char autoFan;
const unsigned char fanStep = 5;
const unsigned int minTemp = 1792; //5.0F; 3584==78.8, for degF conversion 
const unsigned int maxTemp = 5056; //161F, for degF conversion
unsigned int tempRange;


void initTimer1(void) // Fan PWM on ICP1/PB0
{
	sbi(DDRB,PB1); //Enable output for pwm pin
	OCR1A=128; // Duty cycle = OCR1A/255
	TCCR1A=0x91; //Output compare OC1A 8 bit non inverted PWM
	TCCR1B=0x04;  //start timer, prescaled by 1/256 for 31.25KHz (fan wants 21-28KHz; close enough)
}

void initSPI(void)
{
	sbi(SPCR,CPHA);//TODO: set mode 1? CPOL=0 CPHA=1 of SPCR
	sbi(SPCR,SPE); //enable SPI
	sbi(DDRB,PB5); //Set SCK pin (PB5) as an output (required for master mode)
	sbi(DDRB,PB3); //Set MOSI pin (PB3) as an output (required for master mode)
	sbi(DDRB,PB2); //Set SS pin (PB2) as an output (required for master mode)
	sbi(PORTB,PB2); // Drive SS high by default, until we're ready to do a read
	sbi(SPCR,MSTR); //set SPI master  (read from SPDR)
	sbi(SPCR,SPR1); // SCK = F_CPU/64
	//sbi(SPCR,SPIE); //enable SPI interrupts
}

// Swap a single byte over SPI
unsigned char SPI_byte(unsigned char data)
{
	cbi(PORTB,PB2); //SS low initiates transfer
	//TODO; delay 1us 
	SPDR = data; // load byte into buffer
	// Wait for transfer completion
	while(!(SPSR & (1<<SPIF))); // Use use polling instead of ISR; 2 bytes difficult w/ interrrupts 
	sbi(PORTB,PB2); //SS high = done
	return SPSR;  // Should now have whatever came from the slave
}

// Swap a pair of bytes over SPI, one right after the other
unsigned int SPI_word(unsigned int data)
{
	unsigned int value;
	cbi(PORTB,PB2); //SS low initiates transfer
	//asm("nop");asm("nop");asm("nop");asm("nop");asm("nop"); //delay ~1us 

	SPDR = (unsigned char)(data >> 8); // load upper byte into buffer
	// Wait for transfer completion
	while(!(SPSR & (1<<SPIF))); // Use use polling instead of ISR 
	value = SPDR; value <<= 8; // Store result in upper byte

	SPDR = (unsigned char)(data & 0xFF); // load lower byte into buffer
	// Wait for transfer completion
	while(!(SPSR & (1<<SPIF))); // Use use polling instead of ISR
	value |= SPDR; // Store result in lower byte

	sbi(PORTB,PB2); //SS high = done
	return value;
}

unsigned int readThermocouple(void)
{
	unsigned int thermocouple = SPI_word(0xFFFF);

 	if (thermocouple & _BV(2)) { // bit 2 indicates not connected
		thermocouple = 0;
 	} else {
		thermocouple >>= 3; // 3 LSB are not part of temperature value
	}
	// 2 LSB is fractional portion, and the other 10 bits are whole degC.
	// thermocouple value is # of 0.25 Celsius steps, so /4 to get degC 
	exhaustTemp = thermocouple >> 2; //set global 
	return exhaustTemp;
}


// Ring buffer for USART serial
#define serialBufferSize 32
unsigned char serialBuffer[serialBufferSize];
unsigned char serialBufferRead;
unsigned char serialBufferWrite;

void initSerial(unsigned int baud)
{
	serialBufferRead = serialBufferWrite = 0;
	UCSRB |= (1 << RXEN) | (1 << TXEN);   // Turn on the transmission and reception circuit
	UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1); // Use 8-bit character sizes
	UCSRB |= (1 << RXCIE); // Enable the USART Recieve Complete interrupt (USART_RXC) 
	// Set baud prescale per Atmel datasheet:
	baud = (((F_CPU / (baud * 16UL))) - 1);
	UBRRH = (baud >> 8);
	UBRRL = baud;
}

// Append the string to the ring buffer, and trigger a hardware send 
void serialSend(unsigned char* data)
{
	unsigned char i;
	for (i = 0; data[i]; /*serialBufferRead != serialBufferWrite;*/ i++)
	{
		serialBuffer[serialBufferWrite++] = data[i];
		if (serialBufferWrite == serialBufferSize) serialBufferWrite=0;
	}
	sbi(UCSRB,UDRIE); // Enable the USART Data Register Empty (UDRE) interrupt 
}

// serial is ready to send more data; either give it some more, or disable it
ISR(USART_UDRE_vect)
{
	if (serialBufferRead == serialBufferWrite) // no data to write
	{
		cbi(UCSRB,UDRIE); //disable interrupt
	} else { // Write a byte from the buffer to the serial out
		UDR = serialBuffer[serialBufferRead++];
		if (serialBufferRead == serialBufferSize) serialBufferRead=0;
	}
}

// USART Rx Complete on mega8 (others call it USART_RX_vect)
// This is called when an input byte comes in over serial.
// This byte will be command code, so determine what action we should take. 
ISR(USART_RXC_vect)
{
	unsigned char temp = UDR; // read serial byte
	char* tosend = "                  "; // string buffer for reply
	// The trailing '\4' are for the Android side of the interface.
	// The trailing '\0' are for the ring buffer, and might not be necessary.
 
	if (temp == '&') //request exhaustTemp
	{
		if (exhaustTemp==0) {
			serialSend("0 degC (probably disconnected)");
		} else {
			sprintf(tosend,"%u degC\4\0",exhaustTemp);
			serialSend(tosend);
		}
	} else if (temp == '*')  //request roomTemp
	{
		sprintf(tosend,"ADC:%u %u F\4\0",roomTemp,roomTempApprox);
		serialSend(tosend);
	}
	else if (temp == '!')  //switch mode
	{
		autoFan = !autoFan;
		sprintf(tosend,"fan autopilot %s\4\0", autoFan?"on":"off");
		serialSend(tosend);
	}
	else if (temp == '+')  //fan faster
	{
		fanSpeed += fanStep;
		sprintf(tosend,"fan increase %u\4\0",fanSpeed);
		serialSend(tosend);
	}
	else if (temp == '-')  //fan slower
	{
		fanSpeed -= fanStep;
		sprintf(tosend,"fan decrease %u\4\0",fanSpeed);
		serialSend(tosend);
	}
	else if (temp == '#')
	{
		//Intentially trigger watchdog reset (for testing)
		sprintf(tosend,"testing watchdog\4\0",fanSpeed);
		serialSend(tosend);
		delay100ms(40); // hog ISR for 4s; watchdog should reset
	} else UDR = temp; // Echo char
}

ISR(USART_TXC_vect) // Similar to above, but for Tx
{
	// Nothing currently needed to be done on TX completion.
}

// Physical button press
ISR(INT0_vect)
{
	//switch mode
	autoFan = !autoFan;
	//char* tosend = "                  "; // string buffer for reply
	//sprintf(tosend,"fan autopilot %s", autoFan?"on":"off");
	//serialSend(tosend);
}


//void init(void) __attribute__ ((naked, section(".init3"))); // Execute before main()
void init(void) {
	ATOMIC_BLOCK(ATOMIC_FORCEON) // global interrupts disabled; enable after init
	{
	tempRange = maxTemp - minTemp;

	sbi(GICR,INT0); // trigger INT0 on button press
	sbi(MCUCR,ISC01); sbi(MCUCR,ISC00); // trigger INT0 on rising edge

	// initialize timer/counter
	initTimer1();

	//Setup ADC:
	sbi(ADMUX,REFS0); // ADC Aref=Vcc 
	sbi(ADMUX,ADLAR); // ADC 8-bit, left-align (read from ADCH instead of ADC)
	sbi(ADCSRA,ADEN); // ADC enable
	//sbi(DIDR0,0); // Disable Digital Input on ADC pin
	//TODO: Look into Noise Reduction Mode
	//TODO: ADC prescale/20
	
	initSerial(USART_BAUDRATE);

	initSPI();

	wdt_enable(WDTO_2S); // init Watchdog - 2 seconds
	} //global interrupts now enabled
}


unsigned int readTemp(void)
{
	//unsigned int adc;
	cbi(ADMUX,1); cbi(ADMUX,0); // Use ADC0
	//ADMUX |= 2; // Use ADC2

	sbi(ADCSRA,ADSC); // Start conversion
	while(!(ADCSRA & (1<<ADIF))); //Wait for conversion to complete
	roomTemp = ADC; // store converted 10-bit value
	//adc = ADCL; adc = ADCH; // store converted 8-bit value
	sbi(ADCSRA,ADIF); //Clear ADIF by writing one to it (yes, really)

	// Convert ADC value to degrees F
	roomTempApprox = (roomTemp-1792)*0.047794118;
	roomTempApprox -= 4; // estimated empirical correction factor
	return roomTemp;	
}

void adjustFan()
{
		// Map temp range proportionally to fan speed range (0-255) 
		fanSpeed = (unsigned char) ( (float)(roomTemp-minTemp) / (float)tempRange * 255.0f ); //TODO: -1?

		//fanSpeed = (unsigned char)(((((roomTemp<<4)-(minTemp<<4))<<8) / (tempRange<<4)) >>4);

		char* tosend = "                        "; // string buffer for reply

		//unsigned long x = (((roomTemp<<6)-(minTemp<<6))<<8);
		unsigned long x = roomTemp;
		x <<= 6;
		unsigned long y = minTemp;
		y <<= 6;
		x -= y;
		x <<= 8;

		sprintf(tosend,"auto fanSpeed:%u",fanSpeed);
		serialSend(tosend);
}


int main(void) {
	init();

	unsigned char delaytime=5;
	
	sbi(DDRB,PB0); // set PORTB0 output (status LED)
	sbi(DDRB,PB1); // set PORTB1 output (fan PWM)
	
	while(1) {
		wdt_reset(); // "Pet the dog" - We haven't crashed.

		sbi(PORTB,PORTB0); //status led

		roomTemp = readTemp();
		exhaustTemp = readThermocouple();

		if (autoFan) adjustFan();

		OCR1A = fanSpeed; // Adjust hardware PWM duty for fan on PB1

		delay100ms(delaytime);
		cbi(PORTB,PORTB0); //status led
		delay100ms(delaytime);
	}

	return 0;
}
