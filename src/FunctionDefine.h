#ifndef __FUNCTIONDEFINE_H
#define __FUNCTIONDEFINE_H

#define F_VERSION "0.01.00"

#define deUR_SampleCode			0
#define deMCU_UR_SampleCode		0
/* -------------------------------------------------- */
#define deShareFunction			1
#define deUartFunction			1
#define deButtonFunction		1

#define deDebugFunction			1
/* -------------------------------------------------- */
#if deUartFunction
#define deMCU_URFunc			1
#endif

#if deButtonFunction
#define deCapSwButton			1
#define deChangePinMod			1
#endif
/* -------------------------------------------------- */
#if deDebugFunction
/*deLED1_Chack1 in function "MCU_UR_Interrupt" start 
to check Enter Interrupt */
#define deLED1_Chack1			0 
/*deLED1_Chack2 in function "MCU_UR_Interrupt" check c == 0x81 
and get complete data to check */
#define deLED1_Chack2			0
/*deLED1_Chack3 in function "MCU_DataPro" check thread Enter function 
to Process UR data*/
#define deLED1_Chack3			1
 /* deLED2_Chack1 in function "CapRdyInterrupt" to check Enter the function,
 it can use together with deLED1_Chack2*/
#define deLED2_Chack1			0
 /* deLED2_Chack2 in function "MCU_DataPro" to check thread Enter 
 the function*/
#define deLED2_Chack2			1
 /* deUR_CheckValue1 in function "MCU_UR_Interrupt" to 
 check Enter the buffer data correct or not*/
#define deUR_CheckValue1		0
#endif
/* -------------------------------------------------- */
#endif

