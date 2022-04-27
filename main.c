/* 
 * File:   main.c
 * Author: Sergio Boch 20887
 *
 * Created on April 24, 2022, 2:58 PM
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <stdint.h>  // Para poder usar los int de 8 bits

#define _XTAL_FREQ 4000000

uint8_t REGISTRO;
uint8_t VALOR;
void setup(void);

void __interrupt() isr(void){
    if(PIR1bits.ADIF){
        if(ADCON0bits.CHS == 0){       
            
            CCPR1L = (ADRESH>>1)+45;            //Bits mas significativos
            CCP1CONbits.DC1B1 = ADRESH % 0b01;  // Mayor resolución
            CCP1CONbits.DC1B0 = (ADRESH>>7);    // Bits menos significativos
        }
        else if (ADCON0bits.CHS == 1){
            CCPR2L = (ADRESH>>1)+45;            //Bits mas significativos   
            CCP2CONbits.DC2B1 = ADRESH % 0b01;  // Mayor resolución
            CCP2CONbits.DC2B0 = (ADRESH>>7);    // Bits menos significativos
        }
        else 
            REGISTRO = ADRESH;
        PIR1bits.ADIF = 0;
    }
    if(INTCONbits.T0IF){
        INTCONbits.T0IF = 0;    // Reinicio del conteo del timer
        VALOR++;             //Incrementar el puerto A
        TMR0 = 250;
        if(VALOR < REGISTRO)            // Comparación de contador y potenciometro
            PORTDbits.RD0 = 1;          // Encender LED
        else
            PORTDbits.RD0 = 0;      // Aopagar LED
        INTCONbits.T0IF = 0;    // Reinicio del conteo del timer
    }
}

void main(void){
    setup();
    ADCON0bits.GO = 1;  
    while(1){
        __delay_us(70);                     // Tiempo para los cambios 
        if(ADCON0bits.GO == 0){             //Cambio entre los canales AN
            if(ADCON0bits.CHS == 0)
                ADCON0bits.CHS = 1;         // AN1
            else if (ADCON0bits.CHS == 1)
                ADCON0bits.CHS = 2;         // AN2
            else
                ADCON0bits.CHS = 0;         //AN0
            __delay_us(70);
            ADCON0bits.GO = 1; 
        }
    }
}

void setup(void){
    
    ANSEL = 0b00000111;
    ANSELH = 0;
    
    TRISA = 0xFF;
    TRISD = 0;
    
    PORTA = 0;
    PORTD = 0;
    
    OSCCONbits.IRCF = 0b0110; // Oscilador interno a 4Mhz 
    OSCCONbits.SCS = 1; 
    
   
    OPTION_REGbits.T0CS = 0;  // bit 5  Selección de oscilador usado, en este caso oscilador interno
    OPTION_REGbits.T0SE = 0;  // bit 4 Bits de pulsos
    OPTION_REGbits.PSA = 0;   // bit 3  Preescaler asignado para el timer0
    OPTION_REGbits.PS2 = 0;   // bits 2-0  Selección de bits de preescaler
    OPTION_REGbits.PS1 = 1;
    OPTION_REGbits.PS0 = 0;
    TMR0 = 250;
    
    INTCONbits.T0IF = 0;        // Limpiamos bandera del timer0
    INTCONbits.T0IE = 1;        // Se habilita la interrupción del timer0
    
    ADCON0bits.ADCS = 0b01;     //Fosc/8
    ADCON0bits.CHS = 0;         // Usando AN0
    ADCON1bits.ADFM = 0;        //Justificado a la izquierda
    ADCON1bits.VCFG0 = 0;       // Voltajes de referencia VSS y VDD
    ADCON1bits.VCFG1 = 0; 
    
    
    TRISCbits.TRISC2 = 1;   // Pin RC2 como entrada
    TRISCbits.TRISC1 = 1;   // Pin RC1 como entrada
    PR2 = 155;              // Configuración del periodo 
    CCP1CONbits.P1M = 0;    // Ajuste de una salida
    CCP1CONbits.CCP1M = 0b1100;     // Modo PWM potenciometro 1
    CCP2CONbits.CCP2M = 0b1100;     // Modo PWM potenciometro 2
    
    CCPR1L = 0x0f;          // Primer periodo 
    CCP1CONbits.DC1B = 0;   // Usando bits menos significativos
    
    CCPR2L = 0x0f;          // Ciclo de trabajo
    CCP2CONbits.DC2B0 = 0; 
    
    PIR1bits.TMR2IF = 0;    //Bandera de timer2 apagada 
    T2CONbits.T2CKPS = 0b11;    //Preescalador 1:16
    
    T2CONbits.TMR2ON = 1;       // Habilitar timer2
    
    while(PIR1bits.TMR2IF == 0);    //Espera un ciclo del timer2
    PIR1bits.TMR1IF = 0;            // Apagar bandera
    
    TRISCbits.TRISC2 = 0;   //RC2 como salida
    TRISCbits.TRISC1 = 0;   // RC1 como salida
    
    PIR1bits.ADIF = 0;      //Bandera del ADC apagada
    PIE1bits.ADIE = 1;      // Interrupciones analogicas habilitadas
    INTCONbits.PEIE = 1;    // Interrupciones por perifericos habilitadas
    INTCONbits.GIE = 1;     // Interrupciones globales
    
    __delay_us(70);
    ADCON0bits.ADON = 1;    // Habilitar el modulo 
    
    return;
}
