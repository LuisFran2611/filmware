#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
 
#include "ModuloBME.h"
#include "ModuloGPS.h" 
#include "SPI.h"
#include "LoRa.h"
#include "MCP3004.h"
#include "ModuloMQ9.h"
#include "GP2Y1010.h"

typedef unsigned char  u8;
typedef unsigned short u16;
typedef unsigned int   u32;
  
typedef signed char  s8; 
typedef signed short s16;   
typedef signed int   s32;
 
volatile int estado; 
volatile int lectura_CO;   
volatile int lectura_CH4;  
 

//-- Registros mapeados, declaramos los registros, el nombre debe coincidir
#define UARTDAT  (*(volatile uint32_t*)0xE0000080)
#define UARTSTA  (*(volatile uint32_t*)0xE0000084)
#define UARTBAUD (*(volatile uint32_t*)0xE0000084)

#define UART1DAT  (*(volatile uint32_t*)0xE0000090)
#define UART1STA  (*(volatile uint32_t*)0xE0000094)
#define UART1BAUD (*(volatile uint32_t*)0xE0000094)

#define UART2DAT  (*(volatile uint32_t*)0xE00000A0)
#define UART2STA  (*(volatile uint32_t*)0xE00000A4)
#define UART2BAUD (*(volatile uint32_t*)0xE00000A4)

#define SPIDAT	 (*(volatile uint32_t*)0xE0000070)
#define SPICTRL	 (*(volatile uint32_t*)0xE0000074)
#define SPISTA	 (*(volatile uint32_t*)0xE0000074)
#define SPISS	 (*(volatile uint32_t*)0xE0000078)

#define SPI1DAT	 (*(volatile uint32_t*)0xE0000060)
#define SPI1CTRL (*(volatile uint32_t*)0xE0000064)
#define SPI1STA	 (*(volatile uint32_t*)0xE0000064)
#define SPI1SS	 (*(volatile uint32_t*)0xE0000068)

#define MAX_COUNT (*(volatile uint32_t*)0xE0000040)
#define TIMER     (*(volatile uint32_t*)0xE0000040)

#define GPOUT    (*(volatile uint32_t*)0xE0000030)
#define GPIN     (*(volatile uint32_t*)0xE0000034)

#define IRQEN	 (*(volatile uint32_t*)0xE0000020)	//Enable
#define IRQVECT0 (*(volatile uint32_t*)0xE0000000)	//TRAP
#define IRQVECT1 (*(volatile uint32_t*)0xE0000004)	//Timer
#define IRQVECT2 (*(volatile uint32_t*)0xE0000008)	//RX0
#define IRQVECT3 (*(volatile uint32_t*)0xE000000C)	//TX0
#define IRQVECT4 (*(volatile uint32_t*)0xE0000010)	//RX1
#define IRQVECT5 (*(volatile uint32_t*)0xE0000014)	//TX1
#define IRQVECT6 (*(volatile uint32_t*)0xE0000018)	//RX2
#define IRQVECT7 (*(volatile uint32_t*)0xE000001C)	//TX2

// GPOUT
#define LORA_RESET  (1u<<10)
#define L_RX        (1u<<9)
#define L_TX        (1u<<8)
#define EN_5V_UP    (1u<<7)
#define EN_5V_M4    (1u<<6)
#define EN_1V4_M4   (1u<<5)
#define M2_ON_OFF   (1u<<4)
#define LED3        (1u<<3)
#define LED2        (1u<<2)
#define LED1        (1u<<1)
#define LED0        (1u<<0)

// GPIN
#define LORA_DIO1   (1u<<1)
#define LORA_BUSY   (1u<<0)

// IRQ bits
#define IRQ_TIMER     (1u<<1)
#define IRQ_UART0_RX  (1u<<2)
#define IRQ_UART0_TX  (1u<<3)
#define IRQ_UART1_RX  (1u<<4)
#define IRQ_UART1_TX  (1u<<5)
#define IRQ_UART2_RX  (1u<<6)
#define IRQ_UART2_TX  (1u<<7)


void delay_loop(uint32_t val);	// (3 + 3*val) cycles
#define CCLK (18000000u)
#define _delay_us(n) delay_loop((n*(CCLK/1000)-3000)/3000)
#define _delay_ms(n) delay_loop((n*(CCLK/1000)-30)/3)
 
void _putch(int c)
{
	while((UARTSTA&2)==0);
	//if (c == '\n') _putch('\r');
	UARTDAT = c;
}

void _puts(const char *p)
{
	while (*p)
		_putch(*(p++));
}
 

uint8_t _getch()
{
	while((UARTSTA&1)==0);
	return UARTDAT;
}

uint8_t haschar() {return UARTSTA&1;}


// --------------------------------------------------------
//FIFOs
uint8_t udat[32];				//FIFO UART0
volatile uint8_t rdix,wrix;		//Punteros de lectura y escritura

//CHANGED
uint8_t udat1[32]; 				//FIFO UART1
volatile uint8_t rdix1,wrix1; 	//Punteros de lectura y escritura

uint8_t udat2[32]; 				//FIFO UART2
volatile uint8_t rdix2,wrix2; 	//Punteros de lectura y escritura

//Lectura UART0
uint8_t _getchUART0()
{
	uint8_t d;
	while(rdix==wrix);
	d=udat[rdix++];
	rdix&=31;
	return d;
}

//Lectura UART1 (para GPS)
uint8_t _getchUART1()
{
	uint8_t d;
	while(rdix1==wrix1);
	d=udat1[rdix1++];
	rdix1&=127;
	return d;
}

#define putchar(d) _putch(d)
#include "printf.c"
#include "ModuloGPS.c" 
#include "ModuloBME.c"
#include "SPI.c"
#include "LoRa.c"
#include "MCP3004.c"
#include "ModuloMQ9.c"
#include "GP2Y1010.c"
 
const static char *menutxt="\n"
"\n\n"
"888                8888888b.  888     888\n"         
"888                888   Y88b 888     888\n"        
"888                888    888 888     888\n"       
"888        8888b.  888   d88P Y88b   d88P 8888b.\n"  
"888           '88b 8888888P'   Y88b d88P     '88b\n"
"888       .d888888 888 T88b     Y88o88P  .d888888\n" 
"888888888 888  888 888  T88b     Y888P   888  888\n"
"888888888 'Y888888 888   T88b     Y8P    'Y888888\n"
"\nIts Alive :-)\n" 
"\n";             

                  
//uint8_t haschar() {return wrix-rdix;}

uint32_t __attribute__((naked)) getMEPC()
{
	asm volatile(
	//"	csrrw	a0,0x341,zero	\n"
	//"	csrrw	zero,0x341,a0	\n"
	"	.word	0x34101573 		\n"
	"	.word   0x34151073		\n"
	"	ret						\n"
	);
}


//INTERRUPCIONES

void __attribute__((interrupt ("machine"))) irq0_handler()
{
	_printf("\nTRAP at 0x%x\n",getMEPC());
}

// UART RX
void __attribute__((interrupt ("machine"))) irq1_handler()
{ 
	udat[wrix++]=UARTDAT;
	wrix&=31;
} 
// UART TX
void  __attribute__((interrupt ("machine"))) irq2_handler(){
	static uint8_t a=32;
	UARTDAT=a;
	if (++a>=128) a=32;
} 
	 
// TIMER
void __attribute__((interrupt("machine"))) irq3_handler()
{
	IRQEN = 0b00000000;	//Deshabilitamos la interrupción del timer
	switch(estado){ 
		case 0: 
			Lectura_C0();
			break;
		case 1:
			Lectura_CH4();
			break;
		case 2:
			_printf("Medidas obtenidas. \n");
			break;
			
		default:
			break;
	}
	//_printf("Fin interrupcion");
}

// UART RX1
void __attribute__((interrupt("machine"))) irq4_handler()
{
	lectura_GPS();
	IRQEN = 0b00000000;
}

// UART TX1
void __attribute__((interrupt("machine"))) irq5_handler()
{
	static uint8_t a = 32;
	UART1DAT = a;
	if (++a >= 128)
		a = 32;
}

// UART RX2
void __attribute__((interrupt("machine"))) irq6_handler()
{
	static uint8_t a = 32;
	UART2DAT = a;
	if (++a >= 128)
		a = 32;
}

// UART TX2
void __attribute__((interrupt("machine"))) irq7_handler()
{
	static uint8_t a = 32;
	UART2DAT = a;
	if (++a >= 128)
		a = 32;
}

// --------------------------------------------------------

#define BAUD 115200u
#define BAUD1 9600u
#define BAUD2 115200u
#define NULL ((void *)0)


const unsigned static char *menu="\n"
"________________________________________________\n"
"\n"
"    Seleccione: s -> Valores de los sensores    \n"
"    Seleccione: a -> Leer canales ADC           \n"
"    Seleccione: m -> Leer gases MQ9             \n"
"    Seleccione: u -> EN_5V_UP on/off            \n"
"    Seleccione: v -> EN_5V_M4 on/off            \n"
"    Seleccione: w -> EN_1V4_M4 on/off           \n"
"    Seleccione: i -> Leer TIMER                 \n"
"    Seleccione: p -> Leer GP2Y1010              \n"
"    Seleccione: r -> Recibir LoRa               \n"
"    Seleccione: t -> Transmitir LoRa            \n"
"    Seleccione: g -> GPS                        \n"
"    Seleccione: q -> Salir                      \n"
"________________________________________________\n"
"\n\n";

static void toggle_gpout(uint32_t mask, const char *label)
{
	if (GPOUT & mask) {
		GPOUT &= ~mask;
		_printf("%s OFF\n", label);
	} else {
		GPOUT |= mask;
		_printf("%s ON\n", label);
	}
}

   
  
void main()
{
	char c,buf[17];
	uint8_t *p;
	unsigned int i,j;
	int n;
	void (*pcode)();
	uint32_t *pi;
	uint16_t *ps;
	int temp_comp,press_comp, hum_comp;
	unsigned char readDATA;
	   
	//registro control spi 0
	SPICTRL = (8<<8)|8;  
	//registro control spi 1
	SPI1CTRL = (8<<8)|8;
	UARTBAUD  =(CCLK+BAUD/2)/BAUD - 1; 
	//CHANGED
	UART1BAUD =(CCLK+BAUD1/2)/BAUD1 - 1;	
	UART2BAUD =(CCLK+BAUD2/2)/BAUD2 - 1;
         
	//_delay_ms(100); 
	c = UARTDAT;		// Clear RX garbage
	c = UART1DAT;		// Clear RX garbage
	         
	//Inicializamos vectores de interrupcion
	IRQVECT0=(uint32_t)irq0_handler;
	IRQVECT1=(uint32_t)irq1_handler;
	IRQVECT2=(uint32_t)irq2_handler;
	IRQVECT3=(uint32_t)irq3_handler;
	IRQVECT4=(uint32_t)irq4_handler;
	IRQVECT5=(uint32_t)irq5_handler;
	IRQVECT6=(uint32_t)irq6_handler;
	IRQVECT7=(uint32_t)irq7_handler;
	         
	IRQEN=1;			// Enable UART RX IRQ
	
	uint8_t Buff[500];
	
	asm volatile ("ecall");
	asm volatile ("ebreak");
 
	//unsigned int inicio = LoraSx1262_begin();

	
	while (1)      
	{     
			_puts(menu);
			char cmd = _getch();
			if (cmd > 32 && cmd < 127)
				_putch(cmd); 
			_puts("\n"); 
           
			switch (cmd){ 
			case '2':
				IRQEN^=2;	// Toggle IRQ enable for UART TX
				_delay_ms(100);
				break;  
			
			
			//Transmisión del LoRa	
			case 't':   
				if(LoraSx1262_begin()){
					_printf("Modulo LoRa iniciado\n");
					LoraSx1262_transmit("aaoo", 5);
				}
				else {
					_printf("Error al iniciar el LoRa\n");
				}
				break;


			//Recepción del LoRa
			case 'r':
			if(LoraSx1262_begin()){
					_printf("Modulo LoRa iniciado\n");
					LoraSx1262_lora_receive_async(Buff, 500);
				} 
				else {
					_printf("Error al iniciar el LoRa\n");
				}
				break;
				
				
			//Sensores de humedad, temperatura y presión.	
			case 's':               
				startBME680(); 
			    temp_comp =returnTemp();
				press_comp = returnPressure();
				hum_comp = returnHUMIDITY(temp_comp);
				
				_printf("----------------------------------------------\n");
				_printf("--------------- DATOS OBTENIDOS --------------\n");
				_printf("----------------------------------------------\n");
				_printf("                                                ");
				_printf("\n La temperatura registrada es: ");
				_printf("%02d.%d%c", temp_comp/100, temp_comp%100, 167);
				_printf("\n La presion registrada es: ");
				_printf("%d Pascuales",press_comp/100);
				_printf("\n La humedad registrada es: ");
				_printf("%02d.%03d%c", hum_comp/1000, hum_comp%1000, 37);
				_printf("                                              \n");
				_printf("\n\n");
				_printf("----------------------------------------------\n");    
				break;

			//Lectura inmediata MQ9.
			case 'm': {
				int co_ppm = MQ9_ReadCOppm(MCP3004_CH0);
				int ch4_ppm = MQ9_ReadCH4ppm(MCP3004_CH0);

				_printf("----------------------------------------------\n");
				_printf("--------------- MQ9 GASES -------------------\n");
				_printf("----------------------------------------------\n");
				_printf("CO: %d ppm\n", co_ppm);
				_printf("CH4: %d ppm\n", ch4_ppm);
				_printf("----------------------------------------------\n");
				break;
			}
			//Control manual de alimentaciones.
			case 'u':
				toggle_gpout(EN_5V_UP, "EN_5V_UP");
				break;
			case 'v':
				toggle_gpout(EN_5V_M4, "EN_5V_M4");
				break;
			case 'w':
				toggle_gpout(EN_1V4_M4, "EN_1V4_M4");
				break;
			//Estado del TIMER.
			case 'i':
				_printf("TIMER: %u\n", TIMER);
				break;
			//Sensor polvo GP2Y1010.
			case 'p': {
				int raw = GP2Y1010_ReadRaw(MCP3004_CH1);
				_printf("GP2Y1010 ADC: %d\n", raw);
				break;
			}

			//Lectura ADC MCP3004.
			case 'a': {
				int adc0, adc1, adc2, adc3;

				adc0 = MCP3004_Read(MCP3004_CH0);
				adc1 = MCP3004_Read(MCP3004_CH1);
				adc2 = MCP3004_Read(MCP3004_CH2);
				adc3 = MCP3004_Read(MCP3004_CH3);

				_printf("----------------------------------------------\n");
				_printf("--------------- ADC MCP3004 -----------------\n");
				_printf("----------------------------------------------\n");
				_printf("CH0: %d\n", adc0);
				_printf("CH1: %d\n", adc1);
				_printf("CH2: %d\n", adc2);
				_printf("CH3: %d\n", adc3);
				_printf("----------------------------------------------\n");
				break;
			}
				
			//GPS 	
			case 'g': 
				lectura_GPS(); 
				break;
				  
			case 'q':
				return;
				
				
			default:
				continue;
			}
	}
}
