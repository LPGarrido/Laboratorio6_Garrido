; Archivo:	main_laboratorio6.s
; Dispositivo:	PIC16F887
; Autor:	Luis Pedro Garrido
; Compilador:	pic-as (v2.35), MPLABX V6.00
;                
; Programa:	TMR0 - mostrar valores en display
;		TMR1 - aumentar los segundos (valor que va al display)
;		TMR2 - Led intermitente
; Hardware:	LED en el RA0, Display en paralelo en el PORTC con conexión en 		
;		RD0 y RD1
;    
; Creado:	25 feb 2022
; Última modificación: 25 feb 2022
    
PROCESSOR 16F887
    
; PIC16F887 Configuration Bit Settings
; Assembly source line config statements

; CONFIG1
  CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
  CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
  CONFIG  PWRTE = ON            ; Power-up Timer Enable bit (PWRT enabled)
  CONFIG  MCLRE = OFF           ; RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
  CONFIG  CP = OFF              ; Code Protection bit (Program memory code protection is disabled)
  CONFIG  CPD = OFF             ; Data Code Protection bit (Data memory code protection is disabled)
  CONFIG  BOREN = OFF           ; Brown Out Reset Selection bits (BOR disabled)
  CONFIG  IESO = OFF            ; Internal External Switchover bit (Internal/External Switchover mode is disabled)
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
  CONFIG  LVP = ON              ; Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

; CONFIG2
  CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
  CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)

// config statements should precede project file includes.
#include <xc.inc>
  
; -------------- MACROS --------------- 
  ; Macro para reiniciar el valor del TMR0
  RESET_TMR0 MACRO TMR_VAR
    BANKSEL TMR0	    ; cambiamos de banco
    MOVLW   TMR_VAR
    MOVWF   TMR0	    ; configuramos tiempo de retardo
    BCF	    T0IF	    ; limpiamos bandera de interrupción
    ENDM

; Macro para reiniciar el valor del TMR1
; Recibe el valor a configurar en TMR1_H y TMR1_L
RESET_TMR1 MACRO TMR1_H, TMR1_L	 ; 
    BANKSEL TMR1H
    MOVLW   TMR1_H	    ; Literal a guardar en TMR1H
    MOVWF   TMR1H	    ; Guardamos literal en TMR1H
    MOVLW   TMR1_L	    ; Literal a guardar en TMR1L
    MOVWF   TMR1L	    ; Guardamos literal en TMR1L
    BCF	    TMR1IF	    ; Limpiamos bandera de int. TMR1
    ENDM
  
; ------- VARIABLES EN MEMORIA --------
PSECT udata_shr		    ; Memoria compartida
    W_TEMP:		DS 1
    STATUS_TEMP:	DS 1
    
PSECT udata_bank0
    VALOR:		DS 1	; Contiene valor a mostrar en los displays de 7-seg
    DECENAS:		DS 1	; Decenas del valor
    UNIDADES:		DS 1	; Unidades del valor
    BANDERAS:		DS 1	; Indica que display hay que encender
    NIBBLES:		DS 2	; Nibbles del valor (alto y bajo)
    DISPLAY:		DS 2	; Representación de cada nibble en el display de 7-seg

PSECT resVect, class=CODE, abs, delta=2
ORG 00h			    ; posición 0000h para el reset
;------------ VECTOR RESET --------------
resetVec:
    PAGESEL MAIN	; Cambio de pagina
    GOTO    MAIN
    
PSECT intVect, class=CODE, abs, delta=2
ORG 04h			    ; posición 0004h para interrupciones
;------- VECTOR INTERRUPCIONES ----------
PUSH:
    MOVWF   W_TEMP	    ; Guardamos W
    SWAPF   STATUS, W
    MOVWF   STATUS_TEMP	    ; Guardamos STATUS
    
ISR:
    BTFSC   T0IF	    ; Interrupcion de TMR0?
    CALL    INT_TMR0
    BTFSC   TMR1IF	    ; Interrupcion de TMR1?
    CALL    INT_TMR1
    BTFSC   TMR2IF	    ; Interrupcion de TMR2?
    CALL    INT_TMR2

POP:
    SWAPF   STATUS_TEMP, W  
    MOVWF   STATUS	    ; Recuperamos el valor de reg STATUS
    SWAPF   W_TEMP, F	    
    SWAPF   W_TEMP, W	    ; Recuperamos valor de W
    RETFIE		    ; Regresamos a ciclo principal

; ------ SUBRUTINAS DE INTERRUPCIONES ------
INT_TMR0:
    RESET_TMR0 252	    ; Reiniciamos TMR0 para 2ms
    CALL MOSTRAR_VALORES    ; Mostrar valores en el display
    RETURN
    
INT_TMR1:
    RESET_TMR1 0xC2, 0xF7   ; Reiniciamos TMR1 para 1000ms
    INCF    UNIDADES	    ; Incremento en UNIDADES
    RETURN
    
INT_TMR2:
    BCF	    TMR2IF	    ; Limpiamos bandera de interrupcion de TMR1
    INCF    PORTA	    ; Incremento en PORTA
    RETURN
    
PSECT code, delta=2, abs
ORG 100h		    ; posición 100h para el codigo
;------------- CONFIGURACION ------------
MAIN:
    CALL    CONFIG_IO	    ; Configuración de I/O
    CALL    CONFIG_RELOJ    ; Configuración de Oscilador
    CALL    CONFIG_TMR0	    ; Configuración de TMR0
    CALL    CONFIG_TMR1	    ; Configuración de TMR1
    CALL    CONFIG_TMR2	    ; Configuración de TMR2
    CALL    CONFIG_INT	    ; Configuración de interrupciones
    BANKSEL PORTD	    ; Cambio a banco 00
    
LOOP:
    ; Código que se va a estar ejecutando mientras no hayan interrupciones
    CALL    CHECK_UNI	    ;Chequeo Unidades
    CALL    CHECK_DEC	    ;Chequeo Decenas
    
    MOVF    DECENAS,W	    ;W = DECENAS (0-6)
    MOVWF   VALOR	    ;VALOR = DECENAS (0-6) - 0000 (Num 0 al 6)
    SWAPF   VALOR	    ;VALOR = (Num 0 al 6) 0000
    MOVF    UNIDADES,W	    ;W = UNIDADES
    ADDWF   VALOR	    ;VALOR = (Num 0 al 6) (Num 0 al 9)
    
    CALL    OBTENER_NIBBLE  ;Obtener nibbles
    CALL    SET_DISPLAY	    ;Set Display
    GOTO    LOOP	    
    
;------------- SUBRUTINAS ---------------
CONFIG_RELOJ:
    BANKSEL OSCCON	    ; cambiamos a banco 1
    BSF	    OSCCON, 0	    ; SCS -> 1, Usamos reloj interno
    BCF	    OSCCON, 6
    BSF	    OSCCON, 5
    BSF	    OSCCON, 4	    ; IRCF<2:0> -> 011 500kHz
    RETURN
    
; Configuramos el TMR0 para obtener un retardo de 2ms
CONFIG_TMR0:
    BANKSEL OPTION_REG	    ; cambiamos de banco
    BCF	    T0CS	    ; TMR0 como temporizador
    BCF	    PSA		    ; prescaler a TMR0
    BSF	    PS2
    BCF	    PS1
    BSF	    PS0		    ; PS<2:0> -> 101 prescaler 1 : 64
    
    BANKSEL TMR0	    ; cambiamos de banco
    MOVLW   252
    MOVWF   TMR0	    ; 2ms retardo
    BCF	    T0IF	    ; limpiamos bandera de interrupción
    RETURN 
    
; Configuramos el TMR1 para obtener un retardo de 1000ms
CONFIG_TMR1:
    BANKSEL T1CON	    ; Cambiamos a banco 00
    BCF	    TMR1CS	    ; Reloj interno
    BCF	    T1OSCEN	    ; Apagamos LP
    BSF	    T1CKPS1	    ; Prescaler 1:8
    BSF	    T1CKPS0
    BCF	    TMR1GE	    ; TMR1 siempre contando
    BSF	    TMR1ON	    ; Encendemos TMR1
    
    RESET_TMR1 0xC2, 0xF7   ; TMR1 a 1000ms
    RETURN

; Configuramos el TMR2 para obtener un retardo de 500ms
CONFIG_TMR2:
    BANKSEL PR2		    ; Cambiamos a banco 01
    MOVLW   244		    ; Valor para interrupciones cada 500ms
    MOVWF   PR2		    ; Cargamos litaral a PR2
    
    BANKSEL T2CON	    ; Cambiamos a banco 00
    BSF	    T2CKPS1	    ; Prescaler 1:16
    BSF	    T2CKPS0
    
    BSF	    TOUTPS3	    ;Postscaler 1:16
    BSF	    TOUTPS2
    BSF	    TOUTPS1
    BSF	    TOUTPS0
    
    BSF	    TMR2ON	    ; Encendemos TMR2
    RETURN

    
 CONFIG_IO:
    BANKSEL ANSEL
    CLRF    ANSEL
    CLRF    ANSELH	    ; I/O digitales
    BANKSEL TRISD
    CLRF    TRISC	    ; PORTC como salida
    MOVLW   0xFC
    MOVWF   TRISD	    ;RD0 y RD1 como salida
    MOVLW   0xFE
    MOVWF   TRISA	    ;RA0 como salida
    BANKSEL PORTD
    CLRF    PORTC	    ; Apagamos PORTC
    CLRF    PORTD	    ; Apagamos PORTD
    CLRF    PORTA	    ; Apagamos PORTA
    RETURN
    
CONFIG_INT:
    BANKSEL PIE1	    ; Cambiamos a banco 01
    BSF	    TMR1IE	    ; Habilitamos int. TMR1
    BSF	    TMR2IE	    ; Habilitamos int. TMR2
    
    BANKSEL INTCON	    ; Cambiamos a banco 00
    BSF	    PEIE	    ; Habilitamos int. perifericos
    BSF	    GIE		    ; Habilitamos interrupciones
    BSF	    T0IE	    ; Habilitamos interrupcion TMR0
    BCF	    T0IF	    ; Limpiamos bandera de TMR0
    BCF	    TMR1IF	    ; Limpiamos bandera de TMR1
    BCF	    TMR2IF	    ; Limpiamos bandera de TMR2
    RETURN
;--------
    
CHECK_UNI:
    MOVF    UNIDADES,W	;W=UNIDADES
    SUBLW   10		;W-10 = UNIDADES - 10
    BTFSS   STATUS,2	;Si Z=0, regresa; si Z=1 funcion
    RETURN		;regresa
    INCF    DECENAS	;incrementar decenas
    CLRF    UNIDADES	;limpiar unidades
    RETURN
    
CHECK_DEC:
    MOVF    DECENAS,W	;W=DECENAS
    SUBLW   6		;W = DECENAS - 6
    BTFSS   STATUS,2	;Si Z=0, regresa; si Z=1 funcion
    RETURN		;regresa
    CLRF    DECENAS	;limpiar decenas
    RETURN		;regresa
    
OBTENER_NIBBLE:		    ;VALOR = 0110 1101
    MOVLW   0x0F	    ;W = 0000 1111
    ANDWF   VALOR,W	    ;AND Valor y W = 0000 Nibble_bajo
    MOVWF   NIBBLES	    ;Guardar valor en NIBBLES
    
    MOVLW   0xF0	    ;W = 1111 0000
    ANDWF   VALOR,W	    ;AND Valor y W = Nibble_alto 0000
    MOVWF   NIBBLES+1	    ;Guardar valor en NIBBLES+1
    SWAPF   NIBBLES+1	    ;NIBBLES+1 = 0000 Nibble_alto
    RETURN
    
SET_DISPLAY:
    MOVF    NIBBLES,W	    ;W = NIBBLES
    CALL    TABLA_7SEG	    ;Llamar la tabla de 7seg
    MOVWF   DISPLAY	    ;DISPLAY = valor con el que regresa la tabla
    
    MOVF    NIBBLES+1,W	    ;W = NIBBLES+1
    CALL    TABLA_7SEG	    ;Llamar la tabla de 7seg
    MOVWF   DISPLAY+1	    ;DISPLAY+1 = alor con el que regresa la tabla
    
    RETURN
    
MOSTRAR_VALORES:    
    BCF	    PORTD,0	    ;Limpiar RD0
    BCF	    PORTD,1	    ;Limpiar RD1
    BTFSC   BANDERAS,0	    ;Si Banderas0=1, GOTO DISPLAY_1; si Banderas0=0 DISPLAY_0
    GOTO    DISPLAY_1
    ;GOTO    DISPLAY_0
    
    DISPLAY_0:
	MOVF    DISPLAY,W   ;W = DISPLAY
	MOVWF   PORTC	    ;PORTC = DISPLAY
	BSF	PORTD,1	    ;RD1=1 
	BSF	BANDERAS,0  ;BANDERAS0=1 - para que cuando regrese ejecute luego el DISPLAY_1
	RETURN

    DISPLAY_1:
	MOVF    DISPLAY+1,W ;W = DISPLAY+1
	MOVWF   PORTC	    ;PORTC = DISPLAY+1
	BSF	PORTD,0	    ;RD0=1
	BCF	BANDERAS,0  ;BANDERAS0=0 - para que cuando regrese ejecute luego el DISPLAY_0
	RETURN
    
    
ORG 200h
TABLA_7SEG:
    CLRF    PCLATH		; Limpiamos registro PCLATH
    BSF	    PCLATH, 1		; Posicionamos el PC en dirección 02xxh
    ANDLW   0x0F		; no saltar más del tamaño de la tabla
    ADDWF   PCL
    RETLW   00111111B	;0
    RETLW   00000110B	;1
    RETLW   01011011B	;2
    RETLW   01001111B	;3
    RETLW   01100110B	;4
    RETLW   01101101B	;5
    RETLW   01111101B	;6
    RETLW   00000111B	;7
    RETLW   01111111B	;8
    RETLW   01101111B	;9
    RETLW   01110111B	;A
    RETLW   01111100B	;b
    RETLW   00111001B	;C
    RETLW   01011110B	;d
    RETLW   01111001B	;E
    RETLW   01110001B	;F