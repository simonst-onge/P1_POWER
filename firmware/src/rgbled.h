/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Digilent

  @File Name
    rgbled.h

  @Description
    This file groups the declarations of the functions that implement
        the RGBLed library (defined in rgbled.c).
        Include the file in the project when this library is needed.
        Use #include "rgbled.h" in the source files where the functions are needed.
 */
/* ************************************************************************** */

#ifndef _RGBLED_H    /* Guard against multiple inclusion */
#define _RGBLED_H

typedef enum
{
	/* Application's state machine's initial state. */
	RGBLED_WAIT=0,
	RGBLED_STOCK_VALUE,
    RGBLED_TRAITEMENT,
	/* TODO: Define states used by the application state machine. */

} RGBLED_STATES;

typedef struct
{
    /* The application's current state */
    RGBLED_STATES state;

    /* TODO: Define any additional data used by the application. */
/*
   USART Read/Write model variables used by the application:
   
    handleUSART0 : the USART driver handle returned by DRV_USART_Open
*/
    

} RGBLED_DATA;

void RGBLED_Init();
void RGBLED_SetValue(unsigned char bValR, unsigned char bValG, unsigned char bValB);
void RGBLED_SetValueGrouped(unsigned int uiValRGB);
void RGBLED_Close();

//private functions:
void RGBLED_ConfigurePins();
void RGBLED_Timer5Setup();

void RGBLED_Tasks( void );

#endif /* _RGBLED_H */

/* *****************************************************************************
 End of File
 */
