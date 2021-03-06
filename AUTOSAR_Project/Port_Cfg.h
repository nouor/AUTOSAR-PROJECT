/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Nour
 ******************************************************************************/

#ifndef PORT_CFG_H
#define PORT_CFG_H

/*
 * Module Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION              (1U)
#define PORT_CFG_SW_MINOR_VERSION              (0U)
#define PORT_CFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION     (3U)

/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT                (STD_ON)

/* Pre-compile option for Version Info API */
#define PORT_VERSION_INFO_API                (STD_ON)

/* Number of comfigured pins*/
#define PORT_CONFIGURED_PINS                 (39U)

/*No. OF CHANNELS FOR PORT */
#define NUMBER_OF_CHANNELS_PER_PORT             8

// Min. & Max. pin number
#define MIN_PIN_NUM                          0
#define MAX_PIN_NUM                          38


/**********************************************************************
 *                     Configured Port ID's                           *
 **********************************************************************/
#define PORT_A_ID                           (Port_IDType)0 /* PORTA */
#define PORT_B_ID                           (Port_IDType)1 /* PORTB */
#define PORT_C_ID                           (Port_IDType)2 /* PORTC */
#define PORT_D_ID                           (Port_IDType)3 /* PORTD */
#define PORT_E_ID                           (Port_IDType)4 /* PORTE */
#define PORT_F_ID                           (Port_IDType)5 /* PORTF */ //SW1 BUTTON PORT //LED1 PORT

/********************************************************************
 *                      GPIO pin numbers                            *
 ********************************************************************/
#define PORT_PIN_NO_0                (Port_PinType)0
#define PORT_PIN_NO_1                (Port_PinType)1 //LED1 PIN
#define PORT_PIN_NO_2                (Port_PinType)2
#define PORT_PIN_NO_3                (Port_PinType)3
#define PORT_PIN_NO_4                (Port_PinType)4 //SW1 BUTTON PIN IN PORTF
#define PORT_PIN_NO_5                (Port_PinType)5
#define PORT_PIN_NO_6                (Port_PinType)6
#define PORT_PIN_NO_7                (Port_PinType)7


/**********************************************************************
 *                      PIN MODE TYPE                                 *
 **********************************************************************/
#define DIO_MODE                        0
#define ANALOG_MODE                     16
   
//PORTA PIN0 MOODES
#define PA0_DIO                         0
#define PA0_U0Rx                        1
#define PA0_CAN1Rx                      8

//PORTA PIN1 MODES
#define PA1_DIO                         0
#define PA1_U0Tx                        1
#define PA1_CAN1Tx                      8

//PORTA PIN2 MODES
#define PA2_DIO                         0
#define PA2_SSI0Clk                     2

//PORTA PIN3 MODES
#define PA3_DIO                         0
#define PA3_SSI0Fss                     2

//PORTA PIN4 MODES
#define PA4_DIO                         0
#define PA4_SSI0Rx                      2

//PORTA PIN5 MODES
#define PA5_DIO                         0
#define PA5_SSI0Tx                      2

//PORTA PIN6 MODES
#define PA6_DIO                         0
#define PA6_I2C1SCL                     3
#define PA6_M1PWM2                      5

//PORTA PIN7 MODES
#define PA7_DIO                         0
#define PA7_I2C1SDA                     3
#define PA7_M1PWM3                      5

//PORTB PIN0 MOODES
#define PB0_DIO                         0
#define PB0_U1Rx                        1
#define T2CCP0                          7

//PORTB PIN1 MOODES
#define PB1_DIO                         0
#define PB1_U1Tx                        1
#define PB1_T2CPP1                      7

//PORTB PIN2 MODES
#define PB2_DIO                         0
#define PB2_I2C0SCL                     3
#define PB2_T3CPP0                      7

//PORTB PIN3 MODES
#define PB3_DIO                         0
#define PB3_I2C0SDA                     3
#define PB3_T3CCP1                      7

//PORTB PIN4 MODES
#define PB4_DIO                         0
#define PB4_SSI2Clk                     2
#define PB4_M0PWM2                      4
#define PB4_T1CCP0                      7
#define PB4_CAN0Rx                      8

#define PB4_Ain10                       ANALOG_MODE //analog
   
//PORTB PIN5 MODES
#define PB5_DIO                         0
#define PB5_SSI2Fss                     2
#define PB5_M0PWM3                      4
#define PB5_T1CCP1                      7
#define PB5_CAN0Tx                      8

#define PB5_Ain11                       ANALOG_MODE //analog

//PORTB PIN6 MODES
#define PB6_DIO                         0
#define PB6_SSI2Rx                      2
#define PB6_M0PWM0                      4
#define PB6_T0CCP0                      7

//PORTB PIN7 MODES
#define PB7_DIO                         0
#define PB7_SSI2Tx                      2
#define PB7_M0PWM1                      4
#define PB7_T0CCP1                      7

//PORTC PIN4 MODES
#define PC4_DIO                         0
#define PC4_U4Rx                        1
#define PC4_U1Rx                        2
#define PC4_M0PWM6                      4
#define PC4_IDX1                        6
#define PC4_WT0CCP0                     7
#define PC4_U1RTS                       8

#define PC4_C1-                         ANALOG_MODE //analog
   
//PORTC PIN5 MODES
#define PC5_DIO                         0
#define PC5_U4Tx                        1
#define PC5_U1Tx                        2
#define PC5_M0PWM7                      4
#define PC5_PhA1                        6
#define PC5_WT0CCP1                     7
#define PC5_U1CTS                       8

#define PC5_C1+                         ANALOG_MODE //analog
   
//PORTC PIN6 MODES
#define PC6_DIO                         0
#define PC6_U3Rx                        1
#define PC6_PhB1                        6
#define PC6_WT1CCP0                     7
#define PC6_USB0epen                    8

#define PC6_C0+                         ANALOG_MODE //analog
   
//PORTC PIN7 MODES
#define PC7_DIO                         0
#define PC7_U3Tx                        1
#define PC7_WT1CCP1                     7
#define PC7_USB0pflt                    8

#define PC6_C0-                         ANALOG_MODE //analog
   
//PORTD PIN0 MODES
#define PD0_DIO                         0
#define PD0_SSI3Clk                     1
#define PD0_SSI1Clk                     2
#define PD0_I2C3SCL                     3
#define PD0_M0PWM6                      4
#define PD0_M1PWM0                      5
#define PD0_WT2CCP0                     7

#define PD0_Ain7                        ANALOG_MODE //analog
   
//PORTD PIN1 MODES
#define PD1_DIO                         0
#define PD1_SSI3Fss                     1
#define PD1_SSI1Fss                     2
#define PD1_I2C3SDA                     3
#define PD1_M0PWM7                      4
#define PD1_M1PWM1                      5
#define PD1_WT2CCP1                     7

#define PD1_Ain6                        ANALOG_MODE //analog
   
//PORTD PIN2 MODES
#define PD2_DIO                         0
#define PD2_SSI3Rx                      1
#define PD2_SSI1Rx                      2
#define PD2_M0Fault0                    4
#define PD2_WT3CCP0                     7
#define PD2_USB0epen                    8
  
#define PD2_Ain5                        ANALOG_MODE //analog
   
//PORTD PIN3 MODES
#define PD3_DIO                         0
#define PD3_SSI3Tx                      1
#define PD3_SSI1Tx                      2
#define PD3_IDX0                        6
#define PD3_WT3CCP1                     7
#define PD3_USB0pflt                    8

#define PD3_Ain4                        ANALOG_MODE //analog
   
//PORTD PIN4 MODES
#define PD4_DIO                         0
#define PD4_U6Rx                        1
#define PD4_WT4CCP0                     7

#define PD4_USB0DM                      ANALOG_MODE //analog
   
//PORTD PIN5 MODES
#define PD5_DIO                         0
#define PD5_U6Tx                        1
#define PD5_WT4CCP1                     7

#define PD5_USB0DP                      ANALOG_MODE //analog
   
//PORTD PIN6 MODES
#define PD6_DIO                         0
#define PD6_U2Rx                        1
#define PD6_M0Fault0                    4
#define PD6_PhA0                        6
#define PD6_WT5CCP0                     7

//PORTD PIN7 MODES
#define PD7_DIO                         0
#define PD7_U2Tx                        1
#define PD7_PhB0                        6
#define PD7_WT5CCP1                     7
#define PD7_NMI                         8

//PORTE PIN0 MODES
#define PE0_DIO                         0
#define PE0_U7Rx                        1

#define PE0_Ain3                        ANALOG_MODE //analog
   
//PORTE PIN1 MODES
#define PE1_DIO                         0
#define PE1_U7Tx                        1

#define PE1_Ain2                        ANALOG_MODE //analog
   
//PORTE PIN2 MODES
#define PE2_DIO                         0

#define PE2_Ain1                        ANALOG_MODE //analog
   
//PORTE PIN3 MODES
#define PE3_DIO                         0
   
#define PE3_Ain0                        ANALOG_MODE //analog
   
//PORTE PIN4 MODES
#define PE4_DIO                         0
#define PE4_U5Rx                        1
#define PE4_I2C2SCL                     3
#define PE4_M0PWM4                      4
#define PE4_M1PWM2                      5
#define PE4_CAN0Rx                      8

#define PE4_Ain9                        ANALOG_MODE //analog
   
//PORTE PIN5 MODES
#define PE5_DIO                         0
#define PE5_U5Tx                        1
#define PE5_I2C2SDA                     3
#define PE5_M0PWM5                      4
#define PE5_M1PWM3                      5
#define PE5_CAN0Tx                      8

#define PE5_Ain8                        ANALOG_MODE //analog
   
//PORTF PIN0 MODES
#define PF0_DIO                         0
#define PF0_U1RTS                       1
#define PF0_SSI1Rx                      2
#define PF0_CAN0Rx                      3
#define PF0_M1PWM4                      5
#define PF0_PhA0                        6
#define PF0_T0CCP0                      7
#define PF0_NMI                         8
#define PF0_C0o                         9
   
//PORTF PIN1 MODES
#define PF1_DIO                         0
#define PF1_U1CTS                       1
#define PF1_SSI1Tx                      2
#define PF1_M1PWM5                      5
#define PF1_PhB0                        6
#define PF1_T0CCP1                      7
#define PF1_C1o                         9
#define PF1_TRD1                        14

//PORTF PIN2 MODES
#define PF2_DIO                         0
#define PF2_SSI1ClK                     2
#define PF2_M0Fault0                    4
#define PF2_M1PWM6                      5
#define PF2_T1CCP0                      7
#define PF2_TRD0                        14

//PORTF PIN3 MODES
#define PF3_DIO                         0
#define PF3_SSI1Fss                     2
#define PF3_CAN0Tx                      3
#define PF3_M1PWM7                      5
#define PF3_T1CCP1                      7
#define PF3_TRCLK                       14

//PORTF PIN4 MODES
#define PF4_DIO                         0
#define PF4_M1Fault0                    5
#define PF4_IDX0                        6
#define PF4_T2CCP0                      7
#define PF4_USB0epen                    14





/* Configured Channel ID's */
#define PORT_A_PIN_0                        (Port_PinType)0
#define PORT_A_PIN_1                        (Port_PinType)1
#define PORT_A_PIN_2                        (Port_PinType)2
#define PORT_A_PIN_3                        (Port_PinType)3
#define PORT_A_PIN_4                        (Port_PinType)4
#define PORT_A_PIN_5                        (Port_PinType)5
#define PORT_A_PIN_6                        (Port_PinType)6
#define PORT_A_PIN_7                        (Port_PinType)7

#define PORT_B_PIN_0                        (Port_PinType)8
#define PORT_B_PIN_1                        (Port_PinType)9
#define PORT_B_PIN_2                        (Port_PinType)10
#define PORT_B_PIN_3                        (Port_PinType)11
#define PORT_B_PIN_4                        (Port_PinType)12
#define PORT_B_PIN_5                        (Port_PinType)13
#define PORT_B_PIN_6                        (Port_PinType)14
#define PORT_B_PIN_7                        (Port_PinType)15

#define PORT_C_PIN_4                        (Port_PinType)16
#define PORT_C_PIN_5                        (Port_PinType)17
#define PORT_C_PIN_6                        (Port_PinType)18
#define PORT_C_PIN_7                        (Port_PinType)19

#define PORT_D_PIN_0                        (Port_PinType)20
#define PORT_D_PIN_1                        (Port_PinType)21
#define PORT_D_PIN_2                        (Port_PinType)22
#define PORT_D_PIN_3                        (Port_PinType)23
#define PORT_D_PIN_4                        (Port_PinType)24
#define PORT_D_PIN_5                        (Port_PinType)25
#define PORT_D_PIN_6                        (Port_PinType)26
#define PORT_D_PIN_7                        (Port_PinType)27

#define PORT_E_PIN_0                        (Port_PinType)28
#define PORT_E_PIN_1                        (Port_PinType)29
#define PORT_E_PIN_2                        (Port_PinType)30
#define PORT_E_PIN_3                        (Port_PinType)31
#define PORT_E_PIN_4                        (Port_PinType)32
#define PORT_E_PIN_5                        (Port_PinType)33

#define PORT_F_PIN_0                        (Port_PinType)34
#define PORT_F_PIN_1                        (Port_PinType)35
#define PORT_F_PIN_2                        (Port_PinType)36
#define PORT_F_PIN_3                        (Port_PinType)37
#define PORT_F_PIN_4                        (Port_PinType)38


/*********************************************************************
 *                        Pin direction                              *
 *********************************************************************/
#define PORT_A_PIN_0_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_A_PIN_1_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_A_PIN_2_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_A_PIN_3_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_A_PIN_4_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_A_PIN_5_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_A_PIN_6_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_A_PIN_7_DIR            (Port_PinDirectionType)PORT_PIN_IN

#define PORT_B_PIN_0_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_B_PIN_1_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_B_PIN_2_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_B_PIN_3_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_B_PIN_4_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_B_PIN_5_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_B_PIN_6_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_B_PIN_7_DIR            (Port_PinDirectionType)PORT_PIN_IN

#define PORT_C_PIN_4_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_C_PIN_5_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_C_PIN_6_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_C_PIN_7_DIR            (Port_PinDirectionType)PORT_PIN_IN

#define PORT_D_PIN_0_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_D_PIN_1_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_D_PIN_2_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_D_PIN_3_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_D_PIN_4_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_D_PIN_5_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_D_PIN_6_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_D_PIN_7_DIR            (Port_PinDirectionType)PORT_PIN_IN

#define PORT_E_PIN_0_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_E_PIN_1_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_E_PIN_2_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_E_PIN_3_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_E_PIN_4_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_E_PIN_5_DIR            (Port_PinDirectionType)PORT_PIN_IN

#define PORT_F_PIN_0_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_F_PIN_1_DIR            (Port_PinDirectionType)PORT_PIN_OUT //LED1
#define PORT_F_PIN_2_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_F_PIN_3_DIR            (Port_PinDirectionType)PORT_PIN_IN
#define PORT_F_PIN_4_DIR            (Port_PinDirectionType)PORT_PIN_IN  //SW1 


/*********************************************************************
 *              CONFIGURED INTERNAL RESISTANCE                       *
 *********************************************************************/

#define PORT_A_PIN_0_RES            (Port_InternalResistor)PULL_UP
#define PORT_A_PIN_1_RES            (Port_InternalResistor)PULL_UP
#define PORT_A_PIN_2_RES            (Port_InternalResistor)PULL_UP
#define PORT_A_PIN_3_RES            (Port_InternalResistor)PULL_UP
#define PORT_A_PIN_4_RES            (Port_InternalResistor)PULL_UP
#define PORT_A_PIN_5_RES            (Port_InternalResistor)PULL_UP
#define PORT_A_PIN_6_RES            (Port_InternalResistor)PULL_UP
#define PORT_A_PIN_7_RES            (Port_InternalResistor)PULL_UP

#define PORT_B_PIN_0_RES            (Port_InternalResistor)PULL_UP
#define PORT_B_PIN_1_RES            (Port_InternalResistor)PULL_UP
#define PORT_B_PIN_2_RES            (Port_InternalResistor)PULL_UP
#define PORT_B_PIN_3_RES            (Port_InternalResistor)PULL_UP
#define PORT_B_PIN_4_RES            (Port_InternalResistor)PULL_UP
#define PORT_B_PIN_5_RES            (Port_InternalResistor)PULL_UP
#define PORT_B_PIN_6_RES            (Port_InternalResistor)PULL_UP
#define PORT_B_PIN_7_RES            (Port_InternalResistor)PULL_UP

#define PORT_C_PIN_4_RES            (Port_InternalResistor)PULL_UP
#define PORT_C_PIN_5_RES            (Port_InternalResistor)PULL_UP
#define PORT_C_PIN_6_RES            (Port_InternalResistor)PULL_UP
#define PORT_C_PIN_7_RES            (Port_InternalResistor)PULL_UP

#define PORT_D_PIN_0_RES            (Port_InternalResistor)PULL_UP
#define PORT_D_PIN_1_RES            (Port_InternalResistor)PULL_UP
#define PORT_D_PIN_2_RES            (Port_InternalResistor)PULL_UP
#define PORT_D_PIN_3_RES            (Port_InternalResistor)PULL_UP
#define PORT_D_PIN_4_RES            (Port_InternalResistor)PULL_UP
#define PORT_D_PIN_5_RES            (Port_InternalResistor)PULL_UP
#define PORT_D_PIN_6_RES            (Port_InternalResistor)PULL_UP
#define PORT_D_PIN_7_RES            (Port_InternalResistor)PULL_UP

#define PORT_E_PIN_0_RES            (Port_InternalResistor)PULL_UP
#define PORT_E_PIN_1_RES            (Port_InternalResistor)PULL_UP
#define PORT_E_PIN_2_RES            (Port_InternalResistor)PULL_UP
#define PORT_E_PIN_3_RES            (Port_InternalResistor)PULL_UP
#define PORT_E_PIN_4_RES            (Port_InternalResistor)PULL_UP
#define PORT_E_PIN_5_RES            (Port_InternalResistor)PULL_UP

#define PORT_F_PIN_0_RES            (Port_InternalResistor)PULL_UP
#define PORT_F_PIN_1_RES            (Port_InternalResistor)OFF      //LED1
#define PORT_F_PIN_2_RES            (Port_InternalResistor)PULL_UP
#define PORT_F_PIN_3_RES            (Port_InternalResistor)PULL_UP
#define PORT_F_PIN_4_RES            (Port_InternalResistor)PULL_UP //SW1

/*****************************************************************
 *             CONFIGURED INITIAL VALUE OF LEVEL                 *
 *****************************************************************/
#define PORT_A_PIN_0_LEVEL          STD_LOW
#define PORT_A_PIN_1_LEVEL          STD_LOW
#define PORT_A_PIN_2_LEVEL          STD_LOW
#define PORT_A_PIN_3_LEVEL          STD_LOW
#define PORT_A_PIN_4_LEVEL          STD_LOW
#define PORT_A_PIN_5_LEVEL          STD_LOW
#define PORT_A_PIN_6_LEVEL          STD_LOW
#define PORT_A_PIN_7_LEVEL          STD_LOW

#define PORT_B_PIN_0_LEVEL          STD_LOW
#define PORT_B_PIN_1_LEVEL          STD_LOW
#define PORT_B_PIN_2_LEVEL          STD_LOW
#define PORT_B_PIN_3_LEVEL          STD_LOW
#define PORT_B_PIN_4_LEVEL          STD_LOW
#define PORT_B_PIN_5_LEVEL          STD_LOW
#define PORT_B_PIN_6_LEVEL          STD_LOW
#define PORT_B_PIN_7_LEVEL          STD_LOW

#define PORT_C_PIN_4_LEVEL          STD_LOW
#define PORT_C_PIN_5_LEVEL          STD_LOW 
#define PORT_C_PIN_6_LEVEL          STD_LOW
#define PORT_C_PIN_7_LEVEL          STD_LOW

#define PORT_D_PIN_0_LEVEL          STD_LOW
#define PORT_D_PIN_1_LEVEL          STD_LOW
#define PORT_D_PIN_2_LEVEL          STD_LOW 
#define PORT_D_PIN_3_LEVEL          STD_LOW
#define PORT_D_PIN_4_LEVEL          STD_LOW
#define PORT_D_PIN_5_LEVEL          STD_LOW
#define PORT_D_PIN_6_LEVEL          STD_LOW
#define PORT_D_PIN_7_LEVEL          STD_LOW

#define PORT_E_PIN_0_LEVEL          STD_LOW
#define PORT_E_PIN_1_LEVEL          STD_LOW
#define PORT_E_PIN_2_LEVEL          STD_LOW 
#define PORT_E_PIN_3_LEVEL          STD_LOW
#define PORT_E_PIN_4_LEVEL          STD_LOW
#define PORT_E_PIN_5_LEVEL          STD_LOW

#define PORT_F_PIN_0_LEVEL          STD_LOW
#define PORT_F_PIN_1_LEVEL          STD_LOW //LED1
#define PORT_F_PIN_2_LEVEL          STD_LOW 
#define PORT_F_PIN_3_LEVEL          STD_LOW
#define PORT_F_PIN_4_LEVEL          STD_HIGH //SW1

/*******************************************************
 *       CONFIGURED DIRECTION CHANGEABLE               *
 *******************************************************/
#define PORT_A_PIN_0_DIR_CHANG         STD_ON
#define PORT_A_PIN_1_DIR_CHANG         STD_ON
#define PORT_A_PIN_2_DIR_CHANG         STD_ON
#define PORT_A_PIN_3_DIR_CHANG         STD_ON
#define PORT_A_PIN_4_DIR_CHANG         STD_ON
#define PORT_A_PIN_5_DIR_CHANG         STD_ON
#define PORT_A_PIN_6_DIR_CHANG         STD_ON
#define PORT_A_PIN_7_DIR_CHANG         STD_ON

#define PORT_B_PIN_0_DIR_CHANG         STD_ON
#define PORT_B_PIN_1_DIR_CHANG         STD_ON
#define PORT_B_PIN_2_DIR_CHANG         STD_ON
#define PORT_B_PIN_3_DIR_CHANG         STD_ON
#define PORT_B_PIN_4_DIR_CHANG         STD_ON
#define PORT_B_PIN_5_DIR_CHANG         STD_ON
#define PORT_B_PIN_6_DIR_CHANG         STD_ON
#define PORT_B_PIN_7_DIR_CHANG         STD_ON

#define PORT_C_PIN_4_DIR_CHANG         STD_ON
#define PORT_C_PIN_5_DIR_CHANG         STD_ON 
#define PORT_C_PIN_6_DIR_CHANG         STD_ON
#define PORT_C_PIN_7_DIR_CHANG         STD_ON

#define PORT_D_PIN_0_DIR_CHANG         STD_ON
#define PORT_D_PIN_1_DIR_CHANG         STD_ON
#define PORT_D_PIN_2_DIR_CHANG         STD_ON 
#define PORT_D_PIN_3_DIR_CHANG         STD_ON
#define PORT_D_PIN_4_DIR_CHANG         STD_ON
#define PORT_D_PIN_5_DIR_CHANG         STD_ON
#define PORT_D_PIN_6_DIR_CHANG         STD_ON
#define PORT_D_PIN_7_DIR_CHANG         STD_ON

#define PORT_E_PIN_0_DIR_CHANG         STD_ON
#define PORT_E_PIN_1_DIR_CHANG         STD_ON
#define PORT_E_PIN_2_DIR_CHANG         STD_ON
#define PORT_E_PIN_3_DIR_CHANG         STD_ON
#define PORT_E_PIN_4_DIR_CHANG         STD_ON
#define PORT_E_PIN_5_DIR_CHANG         STD_ON 

#define PORT_F_PIN_0_DIR_CHANG         STD_ON
#define PORT_F_PIN_1_DIR_CHANG         STD_ON
#define PORT_F_PIN_2_DIR_CHANG         STD_ON 
#define PORT_F_PIN_3_DIR_CHANG         STD_ON
#define PORT_F_PIN_4_DIR_CHANG         STD_ON

/*******************************************************
 *         CONFIGURED MODE CHANGEABLE                  *
 *******************************************************/
#define PORT_A_PIN_0_MODE_CHANG         STD_ON
#define PORT_A_PIN_1_MODE_CHANG         STD_ON
#define PORT_A_PIN_2_MODE_CHANG         STD_ON
#define PORT_A_PIN_3_MODE_CHANG         STD_ON
#define PORT_A_PIN_4_MODE_CHANG         STD_ON
#define PORT_A_PIN_5_MODE_CHANG         STD_ON
#define PORT_A_PIN_6_MODE_CHANG         STD_ON
#define PORT_A_PIN_7_MODE_CHANG         STD_ON

#define PORT_B_PIN_0_MODE_CHANG         STD_ON
#define PORT_B_PIN_1_MODE_CHANG         STD_ON
#define PORT_B_PIN_2_MODE_CHANG         STD_ON
#define PORT_B_PIN_3_MODE_CHANG         STD_ON
#define PORT_B_PIN_4_MODE_CHANG         STD_ON
#define PORT_B_PIN_5_MODE_CHANG         STD_ON
#define PORT_B_PIN_6_MODE_CHANG         STD_ON
#define PORT_B_PIN_7_MODE_CHANG         STD_ON

#define PORT_C_PIN_4_MODE_CHANG         STD_ON
#define PORT_C_PIN_5_MODE_CHANG         STD_ON 
#define PORT_C_PIN_6_MODE_CHANG         STD_ON
#define PORT_C_PIN_7_MODE_CHANG         STD_ON

#define PORT_D_PIN_0_MODE_CHANG         STD_ON
#define PORT_D_PIN_1_MODE_CHANG         STD_ON
#define PORT_D_PIN_2_MODE_CHANG         STD_ON 
#define PORT_D_PIN_3_MODE_CHANG         STD_ON
#define PORT_D_PIN_4_MODE_CHANG         STD_ON
#define PORT_D_PIN_5_MODE_CHANG         STD_ON
#define PORT_D_PIN_6_MODE_CHANG         STD_ON
#define PORT_D_PIN_7_MODE_CHANG         STD_ON

#define PORT_E_PIN_0_MODE_CHANG         STD_ON
#define PORT_E_PIN_1_MODE_CHANG         STD_ON
#define PORT_E_PIN_2_MODE_CHANG         STD_ON 
#define PORT_E_PIN_3_MODE_CHANG         STD_ON
#define PORT_E_PIN_4_MODE_CHANG         STD_ON
#define PORT_E_PIN_5_MODE_CHANG         STD_ON

#define PORT_F_PIN_0_MODE_CHANG         STD_ON
#define PORT_F_PIN_1_MODE_CHANG         STD_ON
#define PORT_F_PIN_2_MODE_CHANG         STD_ON 
#define PORT_F_PIN_3_MODE_CHANG         STD_ON
#define PORT_F_PIN_4_MODE_CHANG         STD_ON


/*******************************************************
 *         CONFIGURED PIN MODE                         *
 *******************************************************/
#define PORT_A_PIN_0_MODE         PA0_DIO
#define PORT_A_PIN_1_MODE         PA1_DIO
#define PORT_A_PIN_2_MODE         PA2_DIO
#define PORT_A_PIN_3_MODE         PA3_DIO
#define PORT_A_PIN_4_MODE         PA4_DIO
#define PORT_A_PIN_5_MODE         PA5_DIO
#define PORT_A_PIN_6_MODE         PA6_DIO
#define PORT_A_PIN_7_MODE         PA7_DIO

#define PORT_B_PIN_0_MODE         PB0_DIO
#define PORT_B_PIN_1_MODE         PB1_DIO
#define PORT_B_PIN_2_MODE         PB2_DIO
#define PORT_B_PIN_3_MODE         PB3_DIO
#define PORT_B_PIN_4_MODE         PB4_DIO
#define PORT_B_PIN_5_MODE         PB5_DIO
#define PORT_B_PIN_6_MODE         PB6_DIO
#define PORT_B_PIN_7_MODE         PB7_DIO

#define PORT_C_PIN_4_MODE         PC4_DIO
#define PORT_C_PIN_5_MODE         PC5_DIO 
#define PORT_C_PIN_6_MODE         PC6_DIO
#define PORT_C_PIN_7_MODE         PC7_DIO

#define PORT_D_PIN_0_MODE         PD0_DIO
#define PORT_D_PIN_1_MODE         PD1_DIO
#define PORT_D_PIN_2_MODE         PD2_DIO 
#define PORT_D_PIN_3_MODE         PD3_DIO
#define PORT_D_PIN_4_MODE         PD4_DIO
#define PORT_D_PIN_5_MODE         PD5_DIO
#define PORT_D_PIN_6_MODE         PD6_DIO
#define PORT_D_PIN_7_MODE         PD7_DIO

#define PORT_E_PIN_0_MODE         PE0_DIO
#define PORT_E_PIN_1_MODE         PE1_DIO
#define PORT_E_PIN_2_MODE         PE2_DIO 
#define PORT_E_PIN_3_MODE         PE3_DIO
#define PORT_E_PIN_4_MODE         PE4_DIO
#define PORT_E_PIN_5_MODE         PE5_DIO

#define PORT_F_PIN_0_MODE         PF0_DIO
#define PORT_F_PIN_1_MODE         PF1_DIO //LED1
#define PORT_F_PIN_2_MODE         PF2_DIO 
#define PORT_F_PIN_3_MODE         PF3_DIO
#define PORT_F_PIN_4_MODE         PF4_DIO //SW1


#endif