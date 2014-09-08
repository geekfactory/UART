#ifndef COMPILER_H
#define COMPILER_H

// Define "boolean" values
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

/* Alternate definitions */
typedef void                    VOID;
typedef char                    CHAR8;
typedef unsigned char           UCHAR8;

typedef unsigned char           BYTE;                           /* 8-bit unsigned  */
typedef unsigned short int      WORD;                           /* 16-bit unsigned */
typedef unsigned long           DWORD;                          /* 32-bit unsigned */
typedef unsigned long long      QWORD;                          /* 64-bit unsigned */

typedef signed char             CHAR;                           /* 8-bit signed    */
typedef signed short int        SHORT;                          /* 16-bit signed   */
typedef signed long             LONG;                           /* 32-bit signed   */
typedef signed long long        LONGLONG; 

typedef unsigned char			PORTBASE;
#endif
