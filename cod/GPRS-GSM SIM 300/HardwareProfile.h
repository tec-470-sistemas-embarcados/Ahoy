/********************************************************************
 FileName:      HardwareProfile.h
 Dependencies:  See INCLUDES section
 Processor:     PIC18, PIC24, or PIC32 USB Microcontrollers
 Hardware:      The code is natively intended to be used on the 
                  following hardware platforms: 
                    PICDEM™ FS USB Demo Board
                    PIC18F46J50 FS USB Plug-In Module
                    PIC18F87J50 FS USB Plug-In Module
                    Explorer 16 + PIC24 or PIC32 USB PIMs
                    PIC24F Starter Kit
                    Low Pin Count USB Development Kit
                  The firmware may be modified for use on other USB 
                    platforms by editing this file (HardwareProfile.h)
 Compiler:  	Microchip C18 (for PIC18), C30 (for PIC24), 
                  or C32 (for PIC32)
 Company:       Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the “Company”) for its PIC® Microcontroller is intended and
 supplied to you, the Company’s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Date         Description
  1.0   11/19/2004   Initial release
  2.1   02/26/2007   Updated for simplicity and to use common
                     coding style
  2.3   09/15/2008   Broke out each hardware platform into its own
                     "HardwareProfile - xxx.h" file
********************************************************************/

#ifndef HARDWARE_PROFILE_H
#define HARDWARE_PROFILE_H

#define MY_BOARD	// Usuário define características de sua placa


    /*******************************************************************/
    /******** USB stack hardware selection options *********************/
    /*******************************************************************/
    //This section is the set of definitions required by the MCHPFSUSB
    //  framework.  These definitions tell the firmware what mode it is
    //  running in, and where it can find the results to some information
    //  that the stack needs.
    //These definitions are required by every application developed with
    //  this revision of the MCHPFSUSB framework.  Please review each
    //  option carefully and determine which options are desired/required
    //  for your application.

    //The PICDEM FS USB Demo Board platform supports the USE_SELF_POWER_SENSE_IO
    //and USE_USB_BUS_SENSE_IO features.  Uncomment the below line(s) if
    //it is desireable to use one or both of the features.
    //#define USE_SELF_POWER_SENSE_IO
    #define tris_self_power     TRISAbits.TRISA2    // Input
    #if defined(USE_SELF_POWER_SENSE_IO)
    #define self_power          PORTAbits.RA2
    #else
    #define self_power          1
    #endif

    //#define USE_USB_BUS_SENSE_IO
    #define tris_usb_bus_sense  TRISAbits.TRISA1    // Input
    #if defined(USE_USB_BUS_SENSE_IO)
    #define USB_BUS_SENSE       PORTAbits.RA1
    #else
    #define USB_BUS_SENSE       1
    #endif


    //Uncomment the following line to make the output HEX of this  
    //  project work with the MCHPUSB Bootloader    
    //#define PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER
	
    //Uncomment the following line to make the output HEX of this 
    //  project work with the HID Bootloader
    #define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER		

    /*******************************************************************/
    /*******************************************************************/
    /*******************************************************************/
    /******** Application specific definitions *************************/
    /*******************************************************************/
    /*******************************************************************/
    /*******************************************************************/

	/** Message Defines **********************************************/

	//--------------------------------------------
	// MENSAGENS DOS MOTORES
	#define GPS	0
		#define GET_GPS	47

    /** Board features ***********************************************/
    #define CLOCK_FREQ 48000000

    /** LED ************************************************************/

	#define mInitAllLEDs()      TRISA = 0xF0; PORTA = 0x0F;
    
    #define mLED_1              PORTAbits.RA0
    #define mLED_2              PORTAbits.RA1
    #define mLED_3              PORTAbits.RA2
    #define mLED_4              PORTAbits.RA3

    #define mGetLED_1()         mLED_1
    #define mGetLED_2()         mLED_2
    #define mGetLED_3()         mLED_3
    #define mGetLED_4()         mLED_4

    #define mLED_1_On()         mLED_1 = 1;
    #define mLED_2_On()         mLED_2 = 1;
    #define mLED_3_On()         mLED_3 = 1;
    #define mLED_4_On()         mLED_4 = 1;
    
    #define mLED_1_Off()        mLED_1 = 0;
    #define mLED_2_Off()        mLED_2 = 0;
    #define mLED_3_Off()        mLED_3 = 0;
    #define mLED_4_Off()        mLED_4 = 0;
    
    #define mLED_1_Toggle()     mLED_1 = !mLED_1;
    #define mLED_2_Toggle()     mLED_2 = !mLED_2;
    #define mLED_3_Toggle()     mLED_3 = !mLED_3;
    #define mLED_4_Toggle()     mLED_4 = !mLED_4;

	/** MOTORES ********************************************************/
	#define InitMotors()	TRISA = 0XF0; PORTA = 0X0F;

	#define MOTOR_1			PORTAbits.RA0
	#define MOTOR_2			PORTAbits.RA1
	#define MOTOR_3			PORTAbits.RA2
	#define MOTOR_4			PORTAbits.RA3
    
    /** SWITCH *********************************************************/

	#define mInitAllSwitches()  TRISBbits.TRISB7 = 1; TRISBbits.TRISB4 = 1;
    #define mInitSwitch2()      TRISBbits.TRISB7 = 1;
    #define mInitSwitch3()      TRISBbits.TRISB4 = 1;
    #define sw2                 PORTBbits.RB7
    #define sw3                 PORTBbits.RB4	

#endif  //HARDWARE_PROFILE_H
