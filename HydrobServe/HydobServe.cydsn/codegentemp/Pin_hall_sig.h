/*******************************************************************************
* File Name: Pin_hall_sig.h  
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

#if !defined(CY_PINS_Pin_hall_sig_H) /* Pins Pin_hall_sig_H */
#define CY_PINS_Pin_hall_sig_H

#include "cytypes.h"
#include "cyfitter.h"
#include "Pin_hall_sig_aliases.h"


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
} Pin_hall_sig_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   Pin_hall_sig_Read(void);
void    Pin_hall_sig_Write(uint8 value);
uint8   Pin_hall_sig_ReadDataReg(void);
#if defined(Pin_hall_sig__PC) || (CY_PSOC4_4200L) 
    void    Pin_hall_sig_SetDriveMode(uint8 mode);
#endif
void    Pin_hall_sig_SetInterruptMode(uint16 position, uint16 mode);
uint8   Pin_hall_sig_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void Pin_hall_sig_Sleep(void); 
void Pin_hall_sig_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(Pin_hall_sig__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define Pin_hall_sig_DRIVE_MODE_BITS        (3)
    #define Pin_hall_sig_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - Pin_hall_sig_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the Pin_hall_sig_SetDriveMode() function.
         *  @{
         */
        #define Pin_hall_sig_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define Pin_hall_sig_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define Pin_hall_sig_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define Pin_hall_sig_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define Pin_hall_sig_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define Pin_hall_sig_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define Pin_hall_sig_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define Pin_hall_sig_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define Pin_hall_sig_MASK               Pin_hall_sig__MASK
#define Pin_hall_sig_SHIFT              Pin_hall_sig__SHIFT
#define Pin_hall_sig_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in Pin_hall_sig_SetInterruptMode() function.
     *  @{
     */
        #define Pin_hall_sig_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define Pin_hall_sig_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define Pin_hall_sig_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define Pin_hall_sig_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(Pin_hall_sig__SIO)
    #define Pin_hall_sig_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(Pin_hall_sig__PC) && (CY_PSOC4_4200L)
    #define Pin_hall_sig_USBIO_ENABLE               ((uint32)0x80000000u)
    #define Pin_hall_sig_USBIO_DISABLE              ((uint32)(~Pin_hall_sig_USBIO_ENABLE))
    #define Pin_hall_sig_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define Pin_hall_sig_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define Pin_hall_sig_USBIO_ENTER_SLEEP          ((uint32)((1u << Pin_hall_sig_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << Pin_hall_sig_USBIO_SUSPEND_DEL_SHIFT)))
    #define Pin_hall_sig_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << Pin_hall_sig_USBIO_SUSPEND_SHIFT)))
    #define Pin_hall_sig_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << Pin_hall_sig_USBIO_SUSPEND_DEL_SHIFT)))
    #define Pin_hall_sig_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(Pin_hall_sig__PC)
    /* Port Configuration */
    #define Pin_hall_sig_PC                 (* (reg32 *) Pin_hall_sig__PC)
#endif
/* Pin State */
#define Pin_hall_sig_PS                     (* (reg32 *) Pin_hall_sig__PS)
/* Data Register */
#define Pin_hall_sig_DR                     (* (reg32 *) Pin_hall_sig__DR)
/* Input Buffer Disable Override */
#define Pin_hall_sig_INP_DIS                (* (reg32 *) Pin_hall_sig__PC2)

/* Interrupt configuration Registers */
#define Pin_hall_sig_INTCFG                 (* (reg32 *) Pin_hall_sig__INTCFG)
#define Pin_hall_sig_INTSTAT                (* (reg32 *) Pin_hall_sig__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define Pin_hall_sig_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(Pin_hall_sig__SIO)
    #define Pin_hall_sig_SIO_REG            (* (reg32 *) Pin_hall_sig__SIO)
#endif /* (Pin_hall_sig__SIO_CFG) */

/* USBIO registers */
#if !defined(Pin_hall_sig__PC) && (CY_PSOC4_4200L)
    #define Pin_hall_sig_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define Pin_hall_sig_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define Pin_hall_sig_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define Pin_hall_sig_DRIVE_MODE_SHIFT       (0x00u)
#define Pin_hall_sig_DRIVE_MODE_MASK        (0x07u << Pin_hall_sig_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins Pin_hall_sig_H */


/* [] END OF FILE */
