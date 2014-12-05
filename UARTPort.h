#ifndef UARTPORT_H
#define UARTPORT_H

#if defined( PLIB_PIC16 )

#include <xc.h>
#include "stdint.h"
// Include project specific header file
#include "Config.h"
// On PIC16 xUARTHandle is a char
typedef char xUARTHandle;

#elif defined( PLIB_PIC18 )

#include <xc.h>
#include "stdint.h"
// Include project specific header file
#include "Config.h"
// On PIC18 xUARTHandle is a char
typedef char xUARTHandle;

#endif

#endif
// End of header file
