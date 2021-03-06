/***************************************************************************//**
* \file CapSense_1_Control.h
* \version 3.0
*
* \brief
*   This file provides the function prototypes of the Control Block.
*
* \see CapSense P4 v3.0 Datasheet
*
*//*****************************************************************************
* Copyright (2016), Cypress Semiconductor Corporation.
********************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and
* foreign), United States copyright laws and international treaty provisions.
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the
* Cypress Source Code and derivative works for the sole purpose of creating
* custom software in support of licensee product to be used only in conjunction
* with a Cypress integrated circuit as specified in the applicable agreement.
* Any reproduction, modification, translation, compilation, or representation of
* this software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes without further notice to the
* materials described herein. Cypress does not assume any liability arising out
* of the application or use of any product or circuit described herein. Cypress
* does not authorize its products for use as critical components in life-support
* systems where a malfunction or failure may reasonably be expected to result in
* significant injury to the user. The inclusion of Cypress' product in a life-
* support systems application implies that the manufacturer assumes all risk of
* such use and in doing so indemnifies Cypress against all charges. Use may be
* limited by and subject to the applicable Cypress software license agreement.
*******************************************************************************/

#if !defined(CY_CAPSENSE_CapSense_1_CONTROL_H)
#define CY_CAPSENSE_CapSense_1_CONTROL_H

#include "CapSense_1_Configuration.h"

/*******************************************************************************
* Function Prototypes 
*******************************************************************************/

/*******************************************************************************
* HIGH LEVEL API 
*******************************************************************************/
/**
* \if SECTION_CAPSENSE_HIGH_LEVEL
* \addtogroup group_capsense_high_level
* \{
*/

cystatus CapSense_1_Start(void);
cystatus CapSense_1_Stop(void);
cystatus CapSense_1_Resume(void);

cystatus CapSense_1_ProcessAllWidgets(void);
cystatus CapSense_1_ProcessWidget(uint32 widgetId);

void CapSense_1_Sleep(void);
void CapSense_1_Wakeup(void);

/** \}
* \endif */

#if (0u != CapSense_1_SELF_TEST_EN)
    uint32 CapSense_1_RunSelfTest(uint32 testEnMask);
#endif /* #if (0u != CapSense_1_SELF_TEST_EN) */

/**
* \if SECTION_CAPSENSE_LOW_LEVEL
* \addtogroup group_capsense_low_level
* \{
*/

cystatus CapSense_1_ProcessWidgetExt(uint32 widgetId, uint32 mode);
cystatus CapSense_1_ProcessSensorExt(uint32 widgetId, uint32 sensorId, uint32 mode);

/** \}
* \endif */

/*******************************************************************************
* Function Prototypes - internal functions
*******************************************************************************/
/**
* \if SECTION_CAPSENSE_INTERNAL
* \addtogroup group_capsense_internal
* \{
*/
cystatus CapSense_1_Initialize(void);

/** \}
* \endif */

/*******************************************************************************
* API Constants
*******************************************************************************/

/* Define Self-Test Masks */
#if CapSense_1_SELF_TEST_EN
    #define  CapSense_1_TST_SHORTS             (0x01Lu)
    #define  CapSense_1_TST_GLOBAL_CRC         (0x02Lu)
    #define  CapSense_1_TST_BSL_RAW_LIMTS      (0x04Lu)
    #define  CapSense_1_TST_WIDGET_CRC         (0x08Lu)
    #define  CapSense_1_TST_CSD_CAPS           (0x10Lu)
    #define  CapSense_1_TST_CSX_CAPS           (0x20Lu)
    #define  CapSense_1_TST_BASELINES          (0x40Lu)
#endif /* #if CapSense_1_SELF_TEST_EN */

#endif /* End CY_CAPSENSE_CapSense_1_CONTROL_H */

/* [] END OF FILE */
