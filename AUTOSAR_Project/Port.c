/******************************************************************************
*
* Module: Port
*
* File Name: Port.c
*
* Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
*
* Author: Nour
******************************************************************************/

#include "Port.h"
#include "tm4c123gh6pm_registers.h"
#include "Port_Regs.h"

#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and Dio Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
|| (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
  || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of Det.h does not match the expected version"
#endif

#endif

STATIC const Pin_ConfigType * Port_Pin_Config = NULL_PTR;
STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;

/************************************************************************************
* Service Name: Port_Init
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Setup the pin configuration:
*              - Setup the pin as Digital GPIO pin
*              - Setup the direction of the GPIO pin
*              - Provide initial value for o/p pin
*              - Setup the internal resistor for i/p pin
************************************************************************************/
void Port_Init(const Port_ConfigType * ConfigPtr )
{
  volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
  volatile uint32 delay = 0;
  
#if (DIO_DEV_ERROR_DETECT == STD_ON)
  /* check if the input configuration pointer is not a NULL_PTR */
  if (NULL_PTR == ConfigPtr)
  {
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID,
                    PORT_E_PARAM_CONFIG);
  }
  else
#endif
  {
    Port_Pin_Config = ConfigPtr->Pin_config;
    volatile uint8 i;
    
    for(i = 0; i < PORT_CONFIGURED_PINS; i++)
    {
      switch(Port_Pin_Config[i].port_num)
      {
      case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
      break;
      case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
      break;
      case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
      break;
      case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
      break;
      case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
      break;
      case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
      break;
      }
      
      /* Enable clock for PORT and allow time for clock to start*/
      SYSCTL_REGCGC2_REG |= (1<<Port_Pin_Config[i].port_num);
      delay = SYSCTL_REGCGC2_REG;
      
      if( ((Port_Pin_Config[i].port_num == PORT_D_ID) && (Port_Pin_Config[i].pin_num == PORT_PIN_NO_7)) || ((Port_Pin_Config[i].port_num == PORT_F_ID) && (Port_Pin_Config[i].pin_num == PORT_PIN_NO_0)) ) /* PD7 or PF0 */
      {
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                     /* Unlock the GPIOCR register */   
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , Port_Pin_Config[i].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
      }
      else if( (Port_Pin_Config[i].port_num == PORT_C_ID) && (Port_Pin_Config[i].pin_num <= PORT_PIN_NO_3) ) /* PC0 to PC3 */
      {
        /* Do Nothing ...  this is the JTAG pins */
        return;
      }
      else
      {
        /* Do Nothing ... No need to unlock the commit register for this pin */
      }
      
      if(Port_Pin_Config[i].mode == DIO_MODE){
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_Pin_Config[i].pin_num);      /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_Pin_Config[i].pin_num);             /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_Pin_Config[i].pin_num *  4));     /* Clear the PMCx bits for this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_Pin_Config[i].pin_num);        /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
        
      }
      else if(Port_Pin_Config[i].mode == ANALOG_MODE)
      {
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_Pin_Config[i].pin_num);      /* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_Pin_Config[i].pin_num);             /* enable Alternative function for this pin by set the corresponding bit in GPIOAFSEL register */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_Pin_Config[i].pin_num);        /* Clear the corresponding bit in the GPIODEN register to disable digital functionality on this pin */
      }
      else //other modes like (UART, i2c, etc..)
      {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_Pin_Config[i].pin_num);      /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_Pin_Config[i].pin_num);             /* Enable Alternative function for this pin by set the corresponding bit in GPIOAFSEL register */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (Port_Pin_Config[i].mode << (Port_Pin_Config[i].pin_num *  4));     /* Set the PMCx bits for this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_Pin_Config[i].pin_num);        /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
      }
      
      if(Port_Pin_Config[i].direction == PORT_PIN_OUT)
      {
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), Port_Pin_Config[i].pin_num);                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
        
        if(Port_Pin_Config[i].initial_value == STD_HIGH)
        {
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), Port_Pin_Config[i].pin_num);          /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
        }
        else
        {
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), Port_Pin_Config[i].pin_num);        /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
        }
      }
      else if(Port_Pin_Config[i].direction == PORT_PIN_IN)
      {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), Port_Pin_Config[i].pin_num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
        
        if(Port_Pin_Config[i].resistor == PULL_UP)
        {
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET), Port_Pin_Config[i].pin_num);       /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
        }
        else if(Port_Pin_Config[i].resistor == PULL_DOWN)
        {
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET), Port_Pin_Config[i].pin_num);     /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
        }
        else
        {
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET), Port_Pin_Config[i].pin_num);     /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET), Port_Pin_Config[i].pin_num);   /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
        }
           
        
      }
      else{
       
      }
    }
  }
}

/************************************************************************************
* Service Name: Port_SetPinDirection
* Service ID[hex]: 0x01
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): Pin - Port Pin ID number , Direction - Port Pin Direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Sets the port pin direction.
************************************************************************************/

void Port_SetPinDirection(Port_PinType Pin, Port_PinDirectionType Direction){
  
  volatile uint32 * PortGpio_Ptr = NULL_PTR;
  boolean error = FALSE;
#if (PORT_DEV_ERROR_DETECT == STD_ON)
  /* check if the Port driver is initialized*/
  if (PORT_NOT_INITIALIZED == Port_Status)
  {
    /* Report to DET  */
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID,
                    PORT_E_UNINIT );
    error = TRUE;
  }
  else {}
  /* check if Incorrect Port Pin ID passed*/
  if ((Pin > MAX_PIN_NUM) || (Pin < MIN_PIN_NUM ))
  {
    /* Report to DET  */
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID,
                    PORT_E_PARAM_PIN);
    error = TRUE;
  }
  else {}
  /* check if Port Pin not configured as changeable */
  if (STD_OFF == Port_Pin_Config[Pin].direction_changeable)
  {
    /* Report to DET  */
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID,
                    PORT_E_DIRECTION_UNCHANGEABLE);
    error = TRUE;
  }
  else {}
#endif
  
  if (FALSE == error)
  {
    switch(Port_Pin_Config[Pin].port_num)
    {
    case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
    break;
    case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
    break;
    case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
    break;
    case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
    break;
    case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
    break;
    case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
    break;
    }
    
    if(Direction == PORT_PIN_OUT)
    {
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_Pin_Config[Pin].pin_num);                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */            
    }
    
    else if(Direction == PORT_PIN_IN)
    {
      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_Pin_Config[Pin].pin_num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
    }
    
    else
    {
      /* Do Nothing */
    }
    
  }
}


/************************************************************************************
* Service Name: Port_RefreshPortDirection
* Service ID[hex]: 0x02
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Refreshes port direction.
************************************************************************************/
void Port_RefreshPortDirection( void )
{
  volatile uint32 * PortGpio_Ptr = NULL_PTR;
  boolean error = FALSE;
#if (PORT_DEV_ERROR_DETECT == STD_ON)
  /* check if the Port driver is initialized*/
  if (PORT_NOT_INITIALIZED == Port_Status)
  {
    /* Report to DET  */
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_REFRESH_PORT_DIRECTION_SID,
                    PORT_E_UNINIT );
    error = TRUE;
  }
  else {}
  
#endif
  
  if (FALSE == error)
  {
    volatile int i;
    for(i = 0; i < PORT_CONFIGURED_PINS; i++){
      switch(Port_Pin_Config[i].port_num)
      {
      case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
      break;
      case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
      break;
      case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
      break;
      case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
      break;
      case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
      break;
      case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
      break;
      }
      
      if(Port_Pin_Config[i].direction == PORT_PIN_OUT)
      {
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_Pin_Config[i].pin_num);                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
      }
      
      else 
      {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) ,  Port_Pin_Config[i].pin_num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
      }
      
      
    }
  }  /* end of for loop*/
}

/************************************************************************************
* Service Name: Port_GetVersionInfo
* Service ID[hex]: 0x03
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): versioninfo - Pointer to where to store the version information of this module.
* Return value: None
* Description: Returns the version information of this module.
************************************************************************************/
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo( Std_VersionInfoType* versioninfo )
{
#if (PORT_DEV_ERROR_DETECT == STD_ON)
  /* Check if input pointer is not Null pointer */
  if(NULL_PTR == versioninfo)
  {
    /* Report to DET  */
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                    PORT_GET_VERSION_INFO_SID, PORT_E_PARAM_POINTER);
  }
  else
#endif /* (PORT_DEV_ERROR_DETECT == STD_ON) */
  {
    /* Copy the vendor Id */
    versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
    /* Copy the module Id */
    versioninfo->moduleID = (uint16)PORT_MODULE_ID;
    /* Copy Software Major Version */
    versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
    /* Copy Software Minor Version */
    versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
    /* Copy Software Patch Version */
    versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
  }
}
#endif

/************************************************************************************
* Service Name: Port_SetPinMode
* Service ID[hex]: 0x04
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): Pin - Port Pin ID number, Mode - New Port Pin mode to be set on port pin
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin mode.
************************************************************************************/
void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode )
{
  volatile uint32 * PortGpio_Ptr = NULL_PTR;
  boolean error = FALSE;
#if (PORT_DEV_ERROR_DETECT == STD_ON)
  /* check if the Port driver is initialized*/
  if (PORT_NOT_INITIALIZED == Port_Status)
  {
    /* Report to DET  */
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID,
                    PORT_E_UNINIT );
    error = TRUE;
  }
  else {}
  /* check if Incorrect Port Pin ID passed*/
  if ((Pin > MAX_PIN_NUM) || (Pin < MIN_PIN_NUM ))
  {
    /* Report to DET  */
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID,
                    PORT_E_PARAM_PIN);
    error = TRUE;
  }
  else {}
  /* check if Incorrect Port Pin Mode passed*/
  if ((Mode > 14) || (Mode < DIO_MODE ))
  {
    /* Report to DET  */
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID,
                    PORT_E_PARAM_INVALID_MODE);
    error = TRUE;
  }
  else {}
  /* check if Port Pin mode not configured as changeable */
  if (STD_OFF == Port_Pin_Config[Pin].mode_changeable)
  {
    /* Report to DET  */
    Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID,
                    PORT_E_MODE_UNCHANGEABLE);
    error = TRUE;
  }
  else {}
#endif
  if (FALSE == error)
  {
    switch(Port_Pin_Config[Pin].port_num)
    {
    case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
    break;
    case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
    break;
    case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
    break;
    case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
    break;
    case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
    break;
    case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
    break;
    }
    if(Mode == DIO_MODE){
      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_Pin_Config[Pin].pin_num);      /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_Pin_Config[Pin].pin_num);             /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
      *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_Pin_Config[Pin].pin_num *  4));     /* Clear the PMCx bits for this pin */
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_Pin_Config[Pin].pin_num);        /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
      
    }
    
    else if(Mode == ANALOG_MODE)
    {
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_Pin_Config[Pin].pin_num);      /* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_Pin_Config[Pin].pin_num);             /* enable Alternative function for this pin by set the corresponding bit in GPIOAFSEL register */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_Pin_Config[Pin].pin_num);        /* Clear the corresponding bit in the GPIODEN register to disable digital functionality on this pin */
    }
    
    else
    {
      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_Pin_Config[Pin].pin_num);      /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_Pin_Config[Pin].pin_num);             /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
      *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (Mode << (Port_Pin_Config[Pin].pin_num *  4));     /* Set the PMCx bits for this pin */
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_Pin_Config[Pin].pin_num);        /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
    }
  }
}         