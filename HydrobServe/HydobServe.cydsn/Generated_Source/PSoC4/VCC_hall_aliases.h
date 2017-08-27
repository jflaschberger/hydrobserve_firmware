/*******************************************************************************
* File Name: VCC_hall.h  
* Version 2.20
*
* Description:
*  This file contains the Alias definitions for Per-Pin APIs in cypins.h. 
*  Information on using these APIs can be found in the System Reference Guide.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_VCC_hall_ALIASES_H) /* Pins VCC_hall_ALIASES_H */
#define CY_PINS_VCC_hall_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define VCC_hall_0			(VCC_hall__0__PC)
#define VCC_hall_0_PS		(VCC_hall__0__PS)
#define VCC_hall_0_PC		(VCC_hall__0__PC)
#define VCC_hall_0_DR		(VCC_hall__0__DR)
#define VCC_hall_0_SHIFT	(VCC_hall__0__SHIFT)
#define VCC_hall_0_INTR	((uint16)((uint16)0x0003u << (VCC_hall__0__SHIFT*2u)))

#define VCC_hall_INTR_ALL	 ((uint16)(VCC_hall_0_INTR))


#endif /* End Pins VCC_hall_ALIASES_H */


/* [] END OF FILE */
