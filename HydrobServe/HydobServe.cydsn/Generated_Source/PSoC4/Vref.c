/*******************************************************************************
* File Name: Vref.c  
* Version 2.10
*
* Description:
*  This file contains API to enable firmware control of a Pins component.
*
* Note:
*
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "cytypes.h"
#include "Vref.h"

#define SetP4PinDriveMode(shift, mode)  \
    do { \
        Vref_PC =   (Vref_PC & \
                                (uint32)(~(uint32)(Vref_DRIVE_MODE_IND_MASK << (Vref_DRIVE_MODE_BITS * (shift))))) | \
                                (uint32)((uint32)(mode) << (Vref_DRIVE_MODE_BITS * (shift))); \
    } while (0)


/*******************************************************************************
* Function Name: Vref_Write
********************************************************************************
*
* Summary:
*  Assign a new value to the digital port's data output register.  
*
* Parameters:  
*  prtValue:  The value to be assigned to the Digital Port. 
*
* Return: 
*  None 
*  
*******************************************************************************/
void Vref_Write(uint8 value) 
{
    uint8 drVal = (uint8)(Vref_DR & (uint8)(~Vref_MASK));
    drVal = (drVal | ((uint8)(value << Vref_SHIFT) & Vref_MASK));
    Vref_DR = (uint32)drVal;
}


/*******************************************************************************
* Function Name: Vref_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  Vref_DM_STRONG     Strong Drive 
*  Vref_DM_OD_HI      Open Drain, Drives High 
*  Vref_DM_OD_LO      Open Drain, Drives Low 
*  Vref_DM_RES_UP     Resistive Pull Up 
*  Vref_DM_RES_DWN    Resistive Pull Down 
*  Vref_DM_RES_UPDWN  Resistive Pull Up/Down 
*  Vref_DM_DIG_HIZ    High Impedance Digital 
*  Vref_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void Vref_SetDriveMode(uint8 mode) 
{
	SetP4PinDriveMode(Vref__0__SHIFT, mode);
}


/*******************************************************************************
* Function Name: Vref_Read
********************************************************************************
*
* Summary:
*  Read the current value on the pins of the Digital Port in right justified 
*  form.
*
* Parameters:  
*  None 
*
* Return: 
*  Returns the current value of the Digital Port as a right justified number
*  
* Note:
*  Macro Vref_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 Vref_Read(void) 
{
    return (uint8)((Vref_PS & Vref_MASK) >> Vref_SHIFT);
}


/*******************************************************************************
* Function Name: Vref_ReadDataReg
********************************************************************************
*
* Summary:
*  Read the current value assigned to a Digital Port's data output register
*
* Parameters:  
*  None 
*
* Return: 
*  Returns the current value assigned to the Digital Port's data output register
*  
*******************************************************************************/
uint8 Vref_ReadDataReg(void) 
{
    return (uint8)((Vref_DR & Vref_MASK) >> Vref_SHIFT);
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(Vref_INTSTAT) 

    /*******************************************************************************
    * Function Name: Vref_ClearInterrupt
    ********************************************************************************
    *
    * Summary:
    *  Clears any active interrupts attached to port and returns the value of the 
    *  interrupt status register.
    *
    * Parameters:  
    *  None 
    *
    * Return: 
    *  Returns the value of the interrupt status register
    *  
    *******************************************************************************/
    uint8 Vref_ClearInterrupt(void) 
    {
		uint8 maskedStatus = (uint8)(Vref_INTSTAT & Vref_MASK);
		Vref_INTSTAT = maskedStatus;
        return maskedStatus >> Vref_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 


/* [] END OF FILE */
