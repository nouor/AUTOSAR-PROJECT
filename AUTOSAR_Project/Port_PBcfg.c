/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_PBcfg.c
 *
 * Description: Post Build Configuration Source file for TM4C123GH6PM Microcontroller - Dio Driver
 *
 * Author: Nour El-din Mohamed
 ******************************************************************************/

#include "Port.h"

/*
 * Module Version 1.0.0
 */
#define PORT_PBCFG_SW_MAJOR_VERSION              (1U)
#define PORT_PBCFG_SW_MINOR_VERSION              (0U)
#define PORT_PBCFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_PBCFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_PBCFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_PBCFG_AR_RELEASE_PATCH_VERSION     (3U)

/* AUTOSAR Version checking between Port_PBcfg.c and Port.h files */
#if ((PORT_PBCFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_PBCFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_PBCFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of PBcfg.c does not match the expected version"
#endif

/* Software Version checking between Port_PBcfg.c and Port.h files */
#if ((PORT_PBCFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_PBCFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_PBCFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of PBcfg.c does not match the expected version"
#endif

/* PB structure used with Port_Init API */
const Port_ConfigType Port_Configuration ={
  PORT_A_ID, PORT_PIN_NO_0, PORT_A_PIN_0_DIR, PORT_A_PIN_0_RES, PORT_A_PIN_0_LEVEL, PORT_A_PIN_0_DIR_CHANG, PORT_A_PIN_0_MODE_CHANG, PORT_A_PIN_0_MODE,
  PORT_A_ID, PORT_PIN_NO_1, PORT_A_PIN_1_DIR, PORT_A_PIN_1_RES, PORT_A_PIN_1_LEVEL, PORT_A_PIN_1_DIR_CHANG, PORT_A_PIN_1_MODE_CHANG, PORT_A_PIN_1_MODE,
  PORT_A_ID, PORT_PIN_NO_2, PORT_A_PIN_2_DIR, PORT_A_PIN_2_RES, PORT_A_PIN_2_LEVEL, PORT_A_PIN_2_DIR_CHANG, PORT_A_PIN_2_MODE_CHANG, PORT_A_PIN_2_MODE,
  PORT_A_ID, PORT_PIN_NO_3, PORT_A_PIN_3_DIR, PORT_A_PIN_3_RES, PORT_A_PIN_3_LEVEL, PORT_A_PIN_3_DIR_CHANG, PORT_A_PIN_3_MODE_CHANG, PORT_A_PIN_3_MODE,
  PORT_A_ID, PORT_PIN_NO_4, PORT_A_PIN_4_DIR, PORT_A_PIN_4_RES, PORT_A_PIN_4_LEVEL, PORT_A_PIN_4_DIR_CHANG, PORT_A_PIN_4_MODE_CHANG, PORT_A_PIN_4_MODE,
  PORT_A_ID, PORT_PIN_NO_5, PORT_A_PIN_5_DIR, PORT_A_PIN_5_RES, PORT_A_PIN_5_LEVEL, PORT_A_PIN_5_DIR_CHANG, PORT_A_PIN_5_MODE_CHANG, PORT_A_PIN_5_MODE,
  PORT_A_ID, PORT_PIN_NO_6, PORT_A_PIN_6_DIR, PORT_A_PIN_6_RES, PORT_A_PIN_6_LEVEL, PORT_A_PIN_6_DIR_CHANG, PORT_A_PIN_6_MODE_CHANG, PORT_A_PIN_6_MODE,
  PORT_A_ID, PORT_PIN_NO_7, PORT_A_PIN_7_DIR, PORT_A_PIN_7_RES, PORT_A_PIN_7_LEVEL, PORT_A_PIN_7_DIR_CHANG, PORT_A_PIN_7_MODE_CHANG, PORT_A_PIN_7_MODE,
  
  PORT_B_ID, PORT_PIN_NO_0, PORT_B_PIN_0_DIR, PORT_B_PIN_0_RES, PORT_B_PIN_0_LEVEL, PORT_B_PIN_0_DIR_CHANG, PORT_B_PIN_0_MODE_CHANG, PORT_B_PIN_0_MODE,
  PORT_B_ID, PORT_PIN_NO_1, PORT_B_PIN_1_DIR, PORT_B_PIN_1_RES, PORT_B_PIN_1_LEVEL, PORT_B_PIN_1_DIR_CHANG, PORT_B_PIN_1_MODE_CHANG, PORT_B_PIN_1_MODE,
  PORT_B_ID, PORT_PIN_NO_2, PORT_B_PIN_2_DIR, PORT_B_PIN_2_RES, PORT_B_PIN_2_LEVEL, PORT_B_PIN_2_DIR_CHANG, PORT_B_PIN_2_MODE_CHANG, PORT_B_PIN_2_MODE,
  PORT_B_ID, PORT_PIN_NO_3, PORT_B_PIN_3_DIR, PORT_B_PIN_3_RES, PORT_B_PIN_3_LEVEL, PORT_B_PIN_3_DIR_CHANG, PORT_B_PIN_3_MODE_CHANG, PORT_B_PIN_3_MODE,
  PORT_B_ID, PORT_PIN_NO_4, PORT_B_PIN_4_DIR, PORT_B_PIN_4_RES, PORT_B_PIN_4_LEVEL, PORT_B_PIN_4_DIR_CHANG, PORT_B_PIN_4_MODE_CHANG, PORT_B_PIN_4_MODE,
  PORT_B_ID, PORT_PIN_NO_5, PORT_B_PIN_5_DIR, PORT_B_PIN_5_RES, PORT_B_PIN_5_LEVEL, PORT_B_PIN_5_DIR_CHANG, PORT_B_PIN_5_MODE_CHANG, PORT_B_PIN_5_MODE,
  PORT_B_ID, PORT_PIN_NO_6, PORT_B_PIN_6_DIR, PORT_B_PIN_6_RES, PORT_B_PIN_6_LEVEL, PORT_B_PIN_6_DIR_CHANG, PORT_B_PIN_6_MODE_CHANG, PORT_B_PIN_6_MODE,
  PORT_B_ID, PORT_PIN_NO_7, PORT_B_PIN_7_DIR, PORT_B_PIN_7_RES, PORT_B_PIN_7_LEVEL, PORT_B_PIN_7_DIR_CHANG, PORT_B_PIN_7_MODE_CHANG, PORT_B_PIN_7_MODE,
  
  
  PORT_C_ID, PORT_PIN_NO_4, PORT_C_PIN_4_DIR, PORT_C_PIN_4_RES, PORT_C_PIN_4_LEVEL, PORT_C_PIN_4_DIR_CHANG, PORT_C_PIN_4_MODE_CHANG, PORT_C_PIN_4_MODE,
  PORT_C_ID, PORT_PIN_NO_5, PORT_C_PIN_5_DIR, PORT_C_PIN_5_RES, PORT_C_PIN_5_LEVEL, PORT_C_PIN_5_DIR_CHANG, PORT_C_PIN_5_MODE_CHANG, PORT_C_PIN_5_MODE,
  PORT_C_ID, PORT_PIN_NO_6, PORT_C_PIN_6_DIR, PORT_C_PIN_6_RES, PORT_C_PIN_6_LEVEL, PORT_C_PIN_6_DIR_CHANG, PORT_C_PIN_6_MODE_CHANG, PORT_C_PIN_6_MODE,
  PORT_C_ID, PORT_PIN_NO_7, PORT_C_PIN_7_DIR, PORT_C_PIN_7_RES, PORT_C_PIN_7_LEVEL, PORT_C_PIN_7_DIR_CHANG, PORT_C_PIN_7_MODE_CHANG, PORT_C_PIN_7_MODE,
  
  PORT_D_ID, PORT_PIN_NO_0, PORT_D_PIN_0_DIR, PORT_D_PIN_0_RES, PORT_D_PIN_0_LEVEL, PORT_D_PIN_0_DIR_CHANG, PORT_D_PIN_0_MODE_CHANG, PORT_D_PIN_0_MODE,
  PORT_D_ID, PORT_PIN_NO_1, PORT_D_PIN_1_DIR, PORT_D_PIN_1_RES, PORT_D_PIN_1_LEVEL, PORT_D_PIN_1_DIR_CHANG, PORT_D_PIN_1_MODE_CHANG, PORT_D_PIN_1_MODE,
  PORT_D_ID, PORT_PIN_NO_2, PORT_D_PIN_2_DIR, PORT_D_PIN_2_RES, PORT_D_PIN_2_LEVEL, PORT_D_PIN_2_DIR_CHANG, PORT_D_PIN_2_MODE_CHANG, PORT_D_PIN_2_MODE,
  PORT_D_ID, PORT_PIN_NO_3, PORT_D_PIN_3_DIR, PORT_D_PIN_3_RES, PORT_D_PIN_3_LEVEL, PORT_D_PIN_3_DIR_CHANG, PORT_D_PIN_3_MODE_CHANG, PORT_D_PIN_3_MODE,
  PORT_D_ID, PORT_PIN_NO_4, PORT_D_PIN_4_DIR, PORT_D_PIN_4_RES, PORT_D_PIN_4_LEVEL, PORT_D_PIN_4_DIR_CHANG, PORT_D_PIN_4_MODE_CHANG, PORT_D_PIN_4_MODE,
  PORT_D_ID, PORT_PIN_NO_5, PORT_D_PIN_5_DIR, PORT_D_PIN_5_RES, PORT_D_PIN_5_LEVEL, PORT_D_PIN_5_DIR_CHANG, PORT_D_PIN_5_MODE_CHANG, PORT_D_PIN_5_MODE,
  PORT_D_ID, PORT_PIN_NO_6, PORT_D_PIN_6_DIR, PORT_D_PIN_6_RES, PORT_D_PIN_6_LEVEL, PORT_D_PIN_6_DIR_CHANG, PORT_D_PIN_6_MODE_CHANG, PORT_D_PIN_6_MODE,
  PORT_D_ID, PORT_PIN_NO_7, PORT_D_PIN_7_DIR, PORT_D_PIN_7_RES, PORT_D_PIN_7_LEVEL, PORT_D_PIN_7_DIR_CHANG, PORT_D_PIN_7_MODE_CHANG, PORT_D_PIN_7_MODE,
  
  PORT_E_ID, PORT_PIN_NO_0, PORT_E_PIN_0_DIR, PORT_E_PIN_0_RES, PORT_E_PIN_0_LEVEL, PORT_E_PIN_0_DIR_CHANG, PORT_E_PIN_0_MODE_CHANG, PORT_E_PIN_0_MODE,
  PORT_E_ID, PORT_PIN_NO_1, PORT_E_PIN_1_DIR, PORT_E_PIN_1_RES, PORT_E_PIN_1_LEVEL, PORT_E_PIN_1_DIR_CHANG, PORT_E_PIN_1_MODE_CHANG, PORT_E_PIN_1_MODE,
  PORT_E_ID, PORT_PIN_NO_2, PORT_E_PIN_2_DIR, PORT_E_PIN_2_RES, PORT_E_PIN_2_LEVEL, PORT_E_PIN_2_DIR_CHANG, PORT_E_PIN_2_MODE_CHANG, PORT_E_PIN_2_MODE,
  PORT_E_ID, PORT_PIN_NO_3, PORT_E_PIN_3_DIR, PORT_E_PIN_3_RES, PORT_E_PIN_3_LEVEL, PORT_E_PIN_3_DIR_CHANG, PORT_E_PIN_3_MODE_CHANG, PORT_E_PIN_3_MODE,
  PORT_E_ID, PORT_PIN_NO_4, PORT_E_PIN_4_DIR, PORT_E_PIN_4_RES, PORT_E_PIN_4_LEVEL, PORT_E_PIN_4_DIR_CHANG, PORT_E_PIN_4_MODE_CHANG, PORT_E_PIN_4_MODE,
  PORT_E_ID, PORT_PIN_NO_5, PORT_E_PIN_5_DIR, PORT_E_PIN_5_RES, PORT_E_PIN_5_LEVEL, PORT_E_PIN_5_DIR_CHANG, PORT_E_PIN_5_MODE_CHANG, PORT_E_PIN_5_MODE,
  
  PORT_F_ID, PORT_PIN_NO_0, PORT_F_PIN_0_DIR, PORT_F_PIN_0_RES, PORT_F_PIN_0_LEVEL, PORT_F_PIN_0_DIR_CHANG, PORT_F_PIN_0_MODE_CHANG, PORT_F_PIN_0_MODE,
  PORT_F_ID, PORT_PIN_NO_1, PORT_F_PIN_1_DIR, PORT_F_PIN_1_RES, PORT_F_PIN_1_LEVEL, PORT_F_PIN_1_DIR_CHANG, PORT_F_PIN_1_MODE_CHANG, PORT_F_PIN_1_MODE,
  PORT_F_ID, PORT_PIN_NO_2, PORT_F_PIN_2_DIR, PORT_F_PIN_2_RES, PORT_F_PIN_2_LEVEL, PORT_F_PIN_2_DIR_CHANG, PORT_F_PIN_2_MODE_CHANG, PORT_F_PIN_2_MODE,
  PORT_F_ID, PORT_PIN_NO_3, PORT_F_PIN_3_DIR, PORT_F_PIN_3_RES, PORT_F_PIN_3_LEVEL, PORT_F_PIN_3_DIR_CHANG, PORT_F_PIN_3_MODE_CHANG, PORT_F_PIN_3_MODE,
  PORT_F_ID, PORT_PIN_NO_4, PORT_F_PIN_4_DIR, PORT_F_PIN_4_RES, PORT_F_PIN_4_LEVEL, PORT_F_PIN_4_DIR_CHANG, PORT_F_PIN_4_MODE_CHANG, PORT_F_PIN_4_MODE,
  
};