/*******************************************************************************
* File Name: VCC_hall.c  
* Version 2.20
*
* Description:
*  This file contains APIs to set up the Pins component for low power modes.
*
* Note:
*
********************************************************************************
* Copyright 2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "cytypes.h"
#include "VCC_hall.h"

static VCC_hall_BACKUP_STRUCT  VCC_hall_backup = {0u, 0u, 0u};


/*******************************************************************************
* Function Name: VCC_hall_Sleep
****************************************************************************//**
*
* \brief Stores the pin configuration and prepares the pin for entering chip 
*  deep-sleep/hibernate modes. This function must be called for SIO and USBIO
*  pins. It is not essential if using GPIO or GPIO_OVT pins.
*
* <b>Note</b> This function is available in PSoC 4 only.
*
* \return 
*  None 
*  
* \sideeffect
*  For SIO pins, this function configures the pin input threshold to CMOS and
*  drive level to Vddio. This is needed for SIO pins when in device 
*  deep-sleep/hibernate modes.
*
* \funcusage
*  \snippet VCC_hall_SUT.c usage_VCC_hall_Sleep_Wakeup
*******************************************************************************/
void VCC_hall_Sleep(void)
{
    #if defined(VCC_hall__PC)
        VCC_hall_backup.pcState = VCC_hall_PC;
    #else
        #if (CY_PSOC4_4200L)
            /* Save the regulator state and put the PHY into suspend mode */
            VCC_hall_backup.usbState = VCC_hall_CR1_REG;
            VCC_hall_USB_POWER_REG |= VCC_hall_USBIO_ENTER_SLEEP;
            VCC_hall_CR1_REG &= VCC_hall_USBIO_CR1_OFF;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(VCC_hall__SIO)
        VCC_hall_backup.sioState = VCC_hall_SIO_REG;
        /* SIO requires unregulated output buffer and single ended input buffer */
        VCC_hall_SIO_REG &= (uint32)(~VCC_hall_SIO_LPM_MASK);
    #endif  
}


/*******************************************************************************
* Function Name: VCC_hall_Wakeup
****************************************************************************//**
*
* \brief Restores the pin configuration that was saved during Pin_Sleep().
*
* For USBIO pins, the wakeup is only triggered for falling edge interrupts.
*
* <b>Note</b> This function is available in PSoC 4 only.
*
* \return 
*  None
*  
* \funcusage
*  Refer to VCC_hall_Sleep() for an example usage.
*******************************************************************************/
void VCC_hall_Wakeup(void)
{
    #if defined(VCC_hall__PC)
        VCC_hall_PC = VCC_hall_backup.pcState;
    #else
        #if (CY_PSOC4_4200L)
            /* Restore the regulator state and come out of suspend mode */
            VCC_hall_USB_POWER_REG &= VCC_hall_USBIO_EXIT_SLEEP_PH1;
            VCC_hall_CR1_REG = VCC_hall_backup.usbState;
            VCC_hall_USB_POWER_REG &= VCC_hall_USBIO_EXIT_SLEEP_PH2;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(VCC_hall__SIO)
        VCC_hall_SIO_REG = VCC_hall_backup.sioState;
    #endif
}


/* [] END OF FILE */
