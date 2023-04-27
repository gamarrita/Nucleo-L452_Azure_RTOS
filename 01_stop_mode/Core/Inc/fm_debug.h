/* @file fm_debug.h
 *
 * @brief Para incluir este header se debe hacer de la siguiente manera:
 *
 *
 * @par
 *
 * COPYRIGHT NOTE: (c) 2023 FLOWMEET. All right reserved.
 *
 */

#ifndef FM_DEBUG_H_
#define FM_DEBUG_H_

// includes.
#include "main.h"
#include "string.h"
#include "stdio.h"

// Macros, defines, microcontroller pins (dhs).

/*
 * LA siguiente macro controla los mensajes de debug a nivel global, comentar
 * la macro para deshabilitar todo mensaje de macro.
 *
 */
//#define FM_DEBUG

/*
 * Si FM_DEBUG esta definida contro individualmente cada mensaje de debug
 * el siguiente condicional. Tipicamene solo uno, o unos pocos, estaran
 * habilitados.
 */
#ifdef FM_DEBUG
	#define FM_DEBUG_UART_TX_TIME_ON_IDLE
#endif

// Typedef.

// Defines.


#define FM_DEBUG_MENU 1

// Function prototypes

void fm_debug_msg_itm(const uint8_t *msg, uint8_t len);
void fm_debug_msg_uart(const uint8_t *p_msg, uint8_t len);
void fm_debug_uint8_uart(uint8_t num);
void fm_debug_uint32_uart(uint32_t num);

#endif /* FM_DEBUG_H */

/*** end of file ***/
