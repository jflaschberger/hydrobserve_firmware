/*******************************************************************************
* File Name: VCC_hall.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_VCC_hall_H) /* Pins VCC_hall_H */
#define CY_PINS_VCC_hall_H

#include "cytypes.h"
#include "cyfitter.h"
#include "VCC_hall_aliases.h"


/***************************************
*     Data Struct Definitions
***************************************/

/**
* \addtogroup group_structures
* @{
*/
    
/* Structure for sleep mode support */
typedef struct
{
    uint32 pcState; /**< State of the port control register */
    uint32 sioState; /**< State of the SIO configuration */
    uint32 usbState; /**< State of the USBIO regulator */
} VCC_hall_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   VCC_hall_Read(void);
void    VCC_hall_Write(uint8 value);
uint8   VCC_hall_ReadDataReg(void);
#if defined(VCC_hall__PC) || (CY_PSOC4_4200L) 
    void    VCC_hall_SetDriveMode(uint8 mode);
#endif
void    VCC_hall_SetInterruptMode(uint16 position, uint16 mode);
uint8   VCC_hall_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void VCC_hall_Sleep(void); 
void VCC_hall_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(VCC_hall__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define VCC_hall_DRIVE_MODE_BITS        (3)
    #define VCC_hall_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - VCC_hall_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the VCC_hall_SetDriveMode() function.
         *  @{
         */
        #define VCC_hall_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define VCC_hall_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define VCC_hall_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define VCC_hall_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define VCC_hall_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define VCC_hall_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define VCC_hall_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define VCC_hall_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define VCC_hall_MASK               VCC_hall__MASK
#define VCC_hall_SHIFT              VCC_hall__SHIFT
#define VCC_hall_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in VCC_hall_SetInterruptMode() function.
     *  @{
     */
        #define VCC_hall_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define VCC_hall_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define VCC_hall_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define VCC_hall_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(VCC_hall__SIO)
    #define VCC_hall_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(VCC_hall__PC) && (CY_PSOC4_4200L)
    #define VCC_hall_USBIO_ENABLE               ((uint32)0x80000000u)
    #define VCC_hall_USBIO_DISABLE              ((uint32)(~VCC_hall_USBIO_ENABLE))
    #define VCC_hall_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define VCC_hall_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define VCC_hall_USBIO_ENTER_SLEEP          ((uint32)((1u << VCC_hall_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << VCC_hall_USBIO_SUSPEND_DEL_SHIFT)))
    #define VCC_hall_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << VCC_hall_USBIO_SUSPEND_SHIFT)))
    #define VCC_hall_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << VCC_hall_USBIO_SUSPEND_DEL_SHIFT)))
    #define VCC_hall_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(VCC_hall__PC)
    /* Port Configuration */
    #define VCC_hall_PC                 (* (reg32 *) VCC_hall__PC)
#endif
/* Pin State */
#define VCC_hall_PS                     (* (reg32 *) VCC_hall__PS)
/* Data Register */
#define VCC_hall_DR                     (* (reg32 *) VCC_hall__DR)
/* Input Buffer Disable Override */
#define VCC_hall_INP_DIS                (* (reg32 *) VCC_hall__PC2)

/* Interrupt configuration Registers */
#define VCC_hall_INTCFG                 (* (reg32 *) VCC_hall__INTCFG)
#define VCC_hall_INTSTAT                (* (reg32 *) VCC_hall__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define VCC_hall_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(VCC_hall__SIO)
    #define VCC_hall_SIO_REG            (* (reg32 *) VCC_hall__SIO)
#endif /* (VCC_hall__SIO_CFG) */

/* USBIO registers */
#if !defined(VCC_hall__PC) && (CY_PSOC4_4200L)
    #define VCC_hall_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define VCC_hall_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define VCC_hall_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define VCC_hall_DRIVE_MODE_SHIFT       (0x00u)
#define VCC_hall_DRIVE_MODE_MASK        (0x07u << VCC_hall_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins VCC_hall_H */


/* [] END OF FILE */
