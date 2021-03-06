/***************************************************************************//**
* \file CapSense_SensingCSX_LL.c
* \version 3.0
*
* \brief
*   This file defines the data structure global variables and provides
*   implementation for the low-level APIs of the CSX part of
*   the Sensing module. The file contains the APIs used for the CSD block
*   initialization, calibration and scanning.
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
#include <cytypes.h>
#include "CyLib.h"

#include "CapSense_Structure.h"
#include "CapSense_Configuration.h"
#include "CapSense_SensingCSX_LL.h"
#include "CapSense_Sensing.h"

#if (0u != CapSense_CSX_EN)

/*******************************************************************************
* Module local function declarations
*******************************************************************************/
/**
* \if SECTION_CAPSENSE_INTERNAL
* \addtogroup group_capsense_internal
* \{
*/

static void CapSense_SsCSXStartSample (void);
static void CapSense_SsCSXPostMultiScan (void);
static void CapSense_SsCSXPostSingleScan (void);

#if (CapSense_CSX_GANGED_SNS_EN == 1u)
static void CapSense_SsCSXPostMultiScanGanged (void);
#endif

/** \}
* \endif */

#if(CapSense_ENABLE == CapSense_CSDV2)
    CY_INLINE static void CapSense_SsCSXStartSampleExt(void);
#endif /* (CapSense_ENABLE == CapSense_CSDV2) */
static uint16 CapSense_SsCalcCsxScanPeriod (CapSense_RAM_WD_BASE_STRUCT const *wdgtPtr);

/*******************************************************************************
* Define module variables
*******************************************************************************/

/*  Local variable used to store the Scan period */
volatile uint16 CapSense_csxScanPeriod = 0u;
/*  Local variable used to return RawCount from the post scan ISR */
volatile uint16 CapSense_csxRawCount = 0u;

#if(CapSense_ENABLE == CapSense_CSDV2)
    volatile uint32 CapSense_resamplingCyclesCnt = CapSense_RESAMPLING_CYCLES_MAX_NUMBER;
#else
    /*  Local variable used by ASM function for raw count correction    */
    volatile uint32 CapSense_csxRawGarbage = 0u;
#endif /* (CapSense_ENABLE == CapSense_CSDV2) */

/*  Pointer to Flash structure holding info of sensor to be scanned     */
#if (CapSense_CSX_GANGED_SNS_EN == 1u)
static CapSense_FLASH_SNS_STRUCT const *CapSense_curGangRxPtr = 0u;
static CapSense_FLASH_SNS_STRUCT const *CapSense_curGangTxPtr = 0u;
#endif
/*  Pointer to Flash structure to hold Tx electrode that was connected previously   */
static CapSense_FLASH_IO_STRUCT const *CapSense_curTxIOPtr;
/*  Pointer to Flash structure to hold Rx electrode that was connected previously   */
static CapSense_FLASH_IO_STRUCT const *CapSense_curRxIOPtr;
/*  Pointer to Pointer to function used to register callback in ISR     */
void (*CapSense_CSXPostScanApiPtr)(void) = &CapSense_SsCSXPostMultiScan;

/*  Local variable to hold total Tx in widget, while scanning all sensors in widget */
static uint16 CapSense_curWdgtTotalRx = 0u;
/*  Local variable to hold total Rx in widget, while scanning all sensors in widget */
static uint16 CapSense_curWdgtTotalTx = 0u;
/*  Flag to indicate that electrodes were connected previously  */
static uint16 CapSense_eleCsxDisconnectFlag = 0u;

/*******************************************************************************
* Function constants
*******************************************************************************/

#define  CapSense_NUM_HALF_CYCLES       2u

/*******************************************************************************
* Function Name: CapSense_CSXInitialize
****************************************************************************//**
*
* \brief
*   Performs hardware and firmware initialization required for the CSX operation
*   of the CapSense component.
*
* \details
*   This function initializes hardware to perform the CSX sensing operation. This
*   function is called by the CapSense_Start() API during the component
*   initialization. If both CSX and CSD sensing methods are used in the component,
*   this function is called by the CapSense_SetupWidget() API to change hardware
*   configured for CSD to re-initialise for the CSX sensing.
*
*   If the CSD and CSX widgets are used in the component, it is recommended not to
*   mix the CSD widgets between the CSX widgets, instead, place all CSX widgets in
*   the required scanning order and then place the CSD widgets in the customizer.
*   For the component API, this action will eliminate the need for changing the hardware
*   configuration for every widget scan and will increase the execution speed
*   in the CapSense_ScanAllWidgets() when the API is called.
*
*   Similarly, it is recommended to set up and scan all the CSX widgets in such
*   a sequence that the CapSense_SetupWidget() API  shouldn't need
*   hardware  sensing-configuration change.
*
*   It is highly not recommended to call this API directly from the application layer.
*
*******************************************************************************/

void CapSense_CSXInitialize(void)
{
    #if(CapSense_ENABLE == CapSense_CSDV2)
        uint8  interruptState;
        uint32 tmpRegVal;

        CY_SET_REG32(CapSense_INTR_PTR,             CapSense_DEFAULT_CSD_INTR_CFG);
        CY_SET_REG32(CapSense_INTR_SET_PTR,         CapSense_DEFAULT_CSD_INTR_SET_CFG);
        CY_SET_REG32(CapSense_INTR_MASK_PTR,        CapSense_DEFAULT_CSD_INTR_MASK_CFG);
        CY_SET_REG32(CapSense_HSCMP_PTR,            CapSense_DEFAULT_CSD_HSCMP_CFG);
        CY_SET_REG32(CapSense_AMBUF_PTR,            CapSense_DEFAULT_CSD_AMBUF_CFG);
        CY_SET_REG32(CapSense_REFGEN_PTR,           CapSense_DEFAULT_CSD_REFGEN_CFG);
        CY_SET_REG32(CapSense_CSDCMP_PTR,           CapSense_DEFAULT_CSD_CSDCMP_CFG);
        CY_SET_REG32(CapSense_IDAC_MOD_PTR,         CapSense_DEFAULT_CSD_IDACA_CFG);

        #if((0u != CapSense_ADC_EN) || (0u != CapSense_IDACB_USED))
            CY_SET_REG32(CapSense_IDAC_COMP_PTR,    CapSense_DEFAULT_CSD_IDACB_CFG);
        #endif /* (0u != CapSense_ADC_EN) || (0u != CapSense_IDACB_USED) */

        CY_SET_REG32(CapSense_SW_RES_PTR,           CapSense_DEFAULT_CSD_SW_RES_CFG);
        CY_SET_REG32(CapSense_SENSE_PERIOD_PTR,     CapSense_DEFAULT_CSD_SENSE_PERIOD_CFG);
        CY_SET_REG32(CapSense_SENSE_DUTY_PTR,       CapSense_DEFAULT_CSD_SENSE_DUTY_CFG);
        CY_SET_REG32(CapSense_SW_HS_P_SEL_PTR,      CapSense_DEFAULT_CSD_SW_HS_P_SEL_CFG);
        CY_SET_REG32(CapSense_SW_HS_N_SEL_PTR,      CapSense_DEFAULT_CSD_SW_HS_N_SEL_CFG);
        CY_SET_REG32(CapSense_SW_SHIELD_SEL_PTR,    CapSense_DEFAULT_CSD_SW_SHIELD_SEL_CFG);
        CY_SET_REG32(CapSense_SW_AMUXBUF_SEL_PTR,   CapSense_DEFAULT_CSD_SW_AMUXBUF_SEL_CFG);

        tmpRegVal = CY_GET_REG32(CapSense_SW_BYP_SEL_PTR);
        tmpRegVal |= CapSense_DEFAULT_CSD_SW_BYP_SEL_CFG;
        CY_SET_REG32(CapSense_SW_BYP_SEL_PTR,       tmpRegVal);

        CY_SET_REG32(CapSense_SW_CMP_P_SEL_PTR,     CapSense_DEFAULT_CSD_SW_CMP_P_SEL_CFG);
        CY_SET_REG32(CapSense_SW_CMP_N_SEL_PTR,     CapSense_DEFAULT_CSD_SW_CMP_N_SEL_CFG);

        tmpRegVal = CY_GET_REG32(CapSense_SW_REFGEN_SEL_PTR);
        tmpRegVal |= CapSense_DEFAULT_CSD_SW_REFGEN_SEL_CFG;
        CY_SET_REG32(CapSense_SW_REFGEN_SEL_PTR,    tmpRegVal);

        CY_SET_REG32(CapSense_SW_FW_MOD_SEL_PTR,    CapSense_DEFAULT_CSD_SW_FW_MOD_SEL_CFG);
        CY_SET_REG32(CapSense_SW_FW_TANK_SEL_PTR,   CapSense_DEFAULT_CSD_SW_FW_TANK_SEL_CFG);
        CY_SET_REG32(CapSense_SW_DSI_SEL_PTR,       CapSense_DEFAULT_CSD_SW_DSI_SEL_CFG);
        CY_SET_REG32(CapSense_SEQ_TIME_PTR,         CapSense_DEFAULT_CSD_SEQ_TIME_CFG);
        CY_SET_REG32(CapSense_SEQ_INIT_CNT_PTR,     CapSense_DEFAULT_CSD_SEQ_INIT_CNT_CFG);
        CY_SET_REG32(CapSense_SEQ_NORM_CNT_PTR,     CapSense_DEFAULT_CSD_SEQ_NORM_CNT_CFG);
        CY_SET_REG32(CapSense_ADC_CTL_PTR,          CapSense_DEFAULT_CSD_ADC_CTL_CFG);
        CY_SET_REG32(CapSense_SEQ_START_PTR,        CapSense_DEFAULT_CSD_SEQ_START_CFG);

        interruptState = CyEnterCriticalSection();
        tmpRegVal = CY_GET_REG32(CapSense_CintA_PC_PTR);
        tmpRegVal &= (~CapSense_GPIO_PC_MASK << (CapSense_CintA_SHIFT * CapSense_GPIO_PC_BIT_SIZE ));
        CY_SET_REG32(CapSense_CintA_PC_PTR, tmpRegVal);
        CyExitCriticalSection(interruptState);

        interruptState = CyEnterCriticalSection();
        tmpRegVal = CY_GET_REG32(CapSense_CintB_PC_PTR);
        tmpRegVal &= (~CapSense_GPIO_PC_MASK << (CapSense_CintB__SHIFT * CapSense_GPIO_PC_BIT_SIZE ));
        CY_SET_REG32(CapSense_CintB_PC_PTR, tmpRegVal);
        CyExitCriticalSection(interruptState);

        /* Connect CintA_tank pin to AMUXBUS-A using HSIOM registers. */
        interruptState = CyEnterCriticalSection();
        tmpRegVal = CY_GET_REG32(CapSense_CintA_HSIOM_PTR);
        tmpRegVal &= ~CapSense_CintA_HSIOM_MASK;
        tmpRegVal |= (CapSense_HSIOM_SEL_AMUXA << CapSense_CintA_HSIOM_SHIFT);
        CY_SET_REG32(CapSense_CintA_HSIOM_PTR, tmpRegVal);
        CyExitCriticalSection(interruptState);

        /* Connect CintB to AMUXBUS-A using HSIOM registers. */
        interruptState = CyEnterCriticalSection();
        tmpRegVal = CY_GET_REG32(CapSense_CintB_HSIOM_PTR);
        tmpRegVal &= ~CapSense_CintB_HSIOM_MASK;
        tmpRegVal |= (CapSense_HSIOM_SEL_AMUXA << CapSense_CintB_HSIOM_SHIFT);
        CY_SET_REG32(CapSense_CintB_HSIOM_PTR, tmpRegVal);
        CyExitCriticalSection(interruptState);

        CapSense_SsSetModClkClockDivider((uint32)CapSense_dsRam.modCsxClk);

        #if (CapSense_ENABLE == CapSense_CSX_COMMON_TX_CLK_EN)
            CapSense_SsSetSnsClockDivider((uint32)CapSense_dsRam.snsCsxClk);
        #endif

        /*  Set all IO states to default state  */
        CapSense_SsSetIOsInDefaultState ();

    #else
        uint32 trimValue;

        /*  Set all IO states to default state  */
        CapSense_SsSetIOsInDefaultState ();

        /*  Connect CintB to AMUXBUS-A using HSIOM registers.   */

        trimValue = CY_GET_REG32(CapSense_CintB_HSIOM_PTR);
        trimValue &= ~CapSense_CintB_HSIOM_MASK;
        trimValue |= (CapSense_HSIOM_SEL_AMUXA << CapSense_CintB_HSIOM_SHIFT);
        CY_SET_REG32(CapSense_CintB_HSIOM_PTR, trimValue);

        /*  Set drive mode of CintB pin to High-Z Analog state in PC register
            and set logic high on DR register.  */
        trimValue = CY_GET_REG32(CapSense_CintB_PC_PTR);
        trimValue &= ~(CapSense_GPIO_PC_MASK << (CapSense_CintB__SHIFT * CapSense_GPIO_PC_BIT_SIZE ));
        CY_SET_REG32(CapSense_CintB_PC_PTR, trimValue);

        trimValue = CY_GET_REG32(CapSense_CintB_DR_PTR);
        trimValue &= ~(uint32)((uint32)0x01u << CapSense_CintB__SHIFT);
        CY_SET_REG32(CapSense_CintB_DR_PTR, trimValue);

        /*  Connect CintA_tank pin to AMUXBUS-A using HSIOM registers.  */
        trimValue = CY_GET_REG32(CapSense_CintA_HSIOM_PTR);
        trimValue &= ~CapSense_CintA_HSIOM_MASK;
        trimValue |= (CapSense_HSIOM_SEL_AMUXA << CapSense_CintA_HSIOM_SHIFT);
        CY_SET_REG32(CapSense_CintA_HSIOM_PTR, trimValue);

        /*  Set drive mode of CintA_tank pin to High-Z Analog state in PC register
            and set logic high on DR register.  */
        trimValue = CY_GET_REG32(CapSense_CintA_PC_PTR);
        trimValue &= ~(CapSense_GPIO_PC_MASK << (CapSense_CintA_SHIFT * CapSense_GPIO_PC_BIT_SIZE ));
        CY_SET_REG32(CapSense_CintA_PC_PTR, trimValue);

        trimValue = CY_GET_REG32(CapSense_CintA_DR_PTR);
        trimValue &= ~(uint32)((uint32)0x01u << CapSense_CintA_SHIFT);
        CY_SET_REG32(CapSense_CintA_DR_PTR, trimValue);

        /*  Connect CSD comparator to CintB via AMUXBUS-A   */
        trimValue = CY_GET_REG32(CapSense_CONFIG_PTR);
        trimValue |= CapSense_CONFIG_SENSE_INSEL_MASK;
        CY_SET_REG32(CapSense_CONFIG_PTR, trimValue);

        /*  If common clocks are enabled for component, set clock
            divider for sensor and modulator clocks and start/restart them  */
        #if (CapSense_CSX_COMMON_TX_CLK_EN == CapSense_ENABLE)
            CapSense_SsSetClockDividers((uint32)CapSense_dsRam.snsCsxClk, (uint32)CapSense_dsRam.modCsxClk);
        #endif
    #endif /* (CapSense_ENABLE == CapSense_CSDV2) */
    /* Enable CapSense ISR  and set ISR for CSX sensing */
    CapSense_SsIsrInitialize(&CapSense_SsCSXScanISR);
}


/*******************************************************************************
* Function Name: CapSense_CSXElectrodeCheck
****************************************************************************//**
*
* \brief
*   Check if electrodes were previously connected using
*   the CapSense_CSXSetupWidgetExt() API and if yes, disconnect them.
*
* \details
*   This function checks if CapSense_eleCsxDisconnectFlag is set.
*   If it is set, the function disconnects the previously connected electrode.
*
*******************************************************************************/
void CapSense_CSXElectrodeCheck(void)
{
    #if (0u != CapSense_CSX_GANGED_SNS_EN)
        uint32 numElectrodes;
    #endif /* (0u != CapSense_CSX_GANGED_SNS_EN)  */

    if (0u != CapSense_eleCsxDisconnectFlag)
    {
       /* Disconnect if electrodes were previous connected by SetupWidgetExt() API */
        #if (0u != CapSense_CSX_GANGED_SNS_EN)
            /* Check ganged sns flag */
            if (CapSense_GANGED_SNS_MASK == (CapSense_curFlashWdgtPtr->staticConfig & CapSense_GANGED_SNS_MASK))
            {
                /*  1. Get number of pins in previous connected Tx electrode
                    3. Get Tx pointer
                    4. Disconnect all Tx   */
                numElectrodes = CapSense_curGangTxPtr->numPins;
                CapSense_curTxIOPtr = &CapSense_ioList[CapSense_curGangTxPtr->firstPinId];

                do
                {
                    CapSense_CSXDisconnectTx(CapSense_curTxIOPtr++);
                    numElectrodes--;
                } while (0u != numElectrodes);

                /*  1. Get number of pins in previous connected Rx electrode
                    3. Get Rx pointer
                    4. Disconnect all Rx   */
                numElectrodes = CapSense_curGangRxPtr->numPins;
                CapSense_curRxIOPtr = &CapSense_ioList[CapSense_curGangRxPtr->firstPinId];

                do
                {
                    CapSense_CSXDisconnectRx(CapSense_curRxIOPtr++);
                    numElectrodes--;
                } while (0u != numElectrodes);
            }
            else
            {
                /*  Disconnect if electrodes were previous connected by SetupWidgetExt API  */
                CapSense_CSXDisconnectTx(CapSense_curTxIOPtr);
                CapSense_CSXDisconnectRx(CapSense_curRxIOPtr);
            }
        #else
            /*  Disconnect if electrodes were previous connected by SetupWidgetExt API  */
            CapSense_CSXDisconnectTx(CapSense_curTxIOPtr);
            CapSense_CSXDisconnectRx(CapSense_curRxIOPtr);
        #endif
        CapSense_eleCsxDisconnectFlag = 0u;
    }
}


/*******************************************************************************
* Function Name: CapSense_CSXSetupWidget
****************************************************************************//**
*
* \brief
*  Performs hardware and firmware initialization required for scanning sensors
*  in a specific widget using the CSX sensing method. The
*  CapSense_CSDScan() function should be used to start scanning when
*  using this function.
*
* \details
*  This function initializes the widgets specific common parameters to perform
*  the CSX scanning. The initialization includes the following:
*    1. The CSD_CONFIG register.
*    2. The IDAC register.
*    3. The Sense clock frequency
*    4. The phase alignment of the sense and modulator clocks.
*
*  This function does not connect any specific sensors to the scanning hardware
*  and also does not start a scanning process. The CapSense_CSXScan()
*  function must be called after initializing the widget to start scanning.
*
*  This function should be called when no scanning in progress. I.e.
*  CapSense_IsBusy() returns anon-busy status.
*
*  This function is called by the CapSense_SetupWidget() API if the
*  given widget uses the CSX sensing method.
*
*  It is not recommended to call this function directly from the application .
*  layer. This function should be used to implement only the user�s specific
*  use cases (for faster execution time or pipeline scanning for example).
*
* \param widgetId
*  Specify the ID number of the widget to perform hardware and firmware
*  initialization required for scanning sensors in the specific widget.
*  A macro for the widget ID can be found in CapSense_Configuration.h
*  file defined as CapSense_<WidgetName>_WDGT_ID.
*
*******************************************************************************/

void CapSense_CSXSetupWidget(uint32 widgetIndex)
{
    /* variable to access widget details    */
    CapSense_RAM_WD_BASE_STRUCT * curRamWdgtBasePtr;

    #if (CapSense_ENABLE == CapSense_CSDV2)
        uint32 tmpRegVal;
        uint32 snsClkVal;
    #endif /* (CapSense_DISABLE == CapSense_CSDV2) */

    #if ((CapSense_ENABLE == CapSense_CSD_CSX_EN) || \
        (CapSense_ENABLE == CapSense_ADC_EN))
         CapSense_SsSwitchSensingMode(CapSense_SENSE_METHOD_CSX_E);
    #endif /* ((CapSense_ENABLE == CapSense_CSD_CSX_EN) || \
               (CapSense_ENABLE == CapSense_ADC_EN)) */

    /*  Check if CSX electrodes were previously connected using
        CapSense_CSXSetupWidgetExt API and if yes, disconnect them  */
    CapSense_CSXElectrodeCheck();

    /*  Configure hardware block */
    #if (CapSense_DISABLE == CapSense_CSDV2)
        /*  Initialise access pointers for current:
            1. Pointer to widget configuration in Flash
            2. Widget configuration in RAM
            3. Make a local copy of widget index    */
        CapSense_curFlashWdgtPtr = &CapSense_dsFlash.wdgtArray[widgetIndex];
        curRamWdgtBasePtr = (CapSense_RAM_WD_BASE_STRUCT *)CapSense_curFlashWdgtPtr->ptr2WdgtRam;

        CY_SET_REG32(CapSense_CONFIG_PTR, CapSense_CONFIG_ENABLE_MASK);
        CY_SET_REG32(CapSense_CONFIG_PTR, (CY_GET_REG32(CapSense_CONFIG_PTR) |
                                CapSense_CONFIG_MUTUALCAP_EN_MASK     |
                                CapSense_CONFIG_SENSE_COMP_BW_MASK    |
                                CapSense_CONFIG_SENSE_COMP_EN_MASK    |
                                CapSense_CONFIG_SENSE_INSEL_MASK      |
                                CapSense_CONFIG_REFBUF_EN_MASK        |
                                CapSense_CONFIG_REFBUF_DRV_MASK));

        /*  If individual clocks are enabled for each widget, set clock
            divider for sensor and modulator clocks and start/restart them.  */
        #if (CapSense_CSX_COMMON_TX_CLK_EN == CapSense_DISABLE)
            CapSense_SsSetClockDividers((uint32)curRamWdgtBasePtr->snsClk, (uint32)CapSense_dsRam.modCsxClk);
        #endif

        /*  Find scan period */
        CapSense_csxScanPeriod = CapSense_SsCalcCsxScanPeriod(curRamWdgtBasePtr);
    #else
        CapSense_curFlashWdgtPtr = &CapSense_dsFlash.wdgtArray[widgetIndex];

        curRamWdgtBasePtr = (CapSense_RAM_WD_BASE_STRUCT *)CapSense_curFlashWdgtPtr->ptr2WdgtRam;

        #if (0u != CapSense_CSX_COMMON_TX_CLK_EN)
            snsClkVal = (uint32)CapSense_dsRam.snsCsxClk;
        #else
            snsClkVal = (uint32)curRamWdgtBasePtr->snsClk;
        #endif /* (0u != CapSense_CSX_COMMON_TX_CLK_EN) */

        CY_SET_REG32(CapSense_SEQ_NORM_CNT_PTR, (uint32)curRamWdgtBasePtr->resolution);
        CapSense_csxScanPeriod  = CapSense_SsCalcCsxScanPeriod(curRamWdgtBasePtr);

        tmpRegVal = (uint32)curRamWdgtBasePtr->snsClkSource;

        snsClkVal = (snsClkVal - 1uL) & CapSense_SENSE_PERIOD_SENSE_DIV_MASK;
        tmpRegVal = (tmpRegVal << CapSense_SENSE_PERIOD_LFSR_SIZE_SHIFT) | snsClkVal;
        CY_SET_REG32(CapSense_SENSE_PERIOD_PTR, tmpRegVal);
        CapSense_SsSetModClkClockDivider((uint32)CapSense_dsRam.modCsxClk);
    #endif /* (CapSense_DISABLE == CapSense_CSDV2) */
}

/*******************************************************************************
* Function Name: CapSense_CSXSetupWidgetExt
****************************************************************************//**
*
* \brief
*  Performs extended initialization for the CSX widget and also performs
*  initialization required for a specific sensor in the widget. The
*  CapSense_CSXScan() function should be called to initiate the scan
*  when using this function.
*
* \details
*  This function does the same tasks as CapSense_CSXSetupWidget() and
*  also connects a sensor in the widget for scanning. Once this function is
*  called to initialize a widget and a sensor, the CapSense_CSXScanExt()
*  function should be called to scan the sensor.
*
*  This function should be called when no scanning in progress. I.e.
*  CapSense_IsBusy() returns a non-busy status.
*
*  It is not recommended to call this function directly from the application
*  layer. This function should be used to implement only the user�s specific
*  use cases (for faster execution time or pipeline scanning for example).
*
* \param widgetId
*  Specify the ID number of the widget to perform hardware and firmware
*  initialization required for scanning a specific sensor in a specific widget.
*  A macro for the widget ID can be found in CapSense_Configuration.h
*  file defined as CapSense_<WidgetName>_WDGT_ID.
*
* \param SnsId
*  Specify the ID number of the sensor within the widget to perform hardware
*  and firmware initialization required for scanning a specific sensor in a
*  specific widget.
*  A macro for the sensor ID within a specified widget can be found in the
*  CapSense_Configuration.h file defined as
*  CapSense_<WidgetName>_SNS<SensorNumber>_ID
*
*******************************************************************************/

void CapSense_CSXSetupWidgetExt(uint32 widgetIndex, uint32 snsIndex)
{
    uint32 txIndex;
    uint32 rxIndex;
    #if (CapSense_ENABLE == CapSense_CSX_GANGED_SNS_EN)
        uint16 numElectrodes;
    #endif /* (CapSense_ENABLE == CapSense_CSX_GANGED_SNS_EN) */

    /*  Initialize widget   */
    CapSense_CSXSetupWidget(widgetIndex);

    /*  Initialise sensor data structure pointer to appropriate address */
    CapSense_curRamSnsPtr = CapSense_curFlashWdgtPtr->ptr2SnsRam;
    CapSense_curRamSnsPtr = &CapSense_curRamSnsPtr[snsIndex];

    /*  Find Tx and Rx index for given snsIndex
        rxIndex = snsIndex / TxCount
        txIndex = snsIndex % TxCount    */
    rxIndex = snsIndex /CapSense_curFlashWdgtPtr->numRows;
    txIndex = snsIndex % CapSense_curFlashWdgtPtr->numRows;

    /*  If widget is APA, find node (Tx/Rx) to be connected for scanning    */
    #if (CapSense_ENABLE == CapSense_CSX_GANGED_SNS_EN)
        if (CapSense_GANGED_SNS_MASK == (CapSense_curFlashWdgtPtr->staticConfig & CapSense_GANGED_SNS_MASK))
        {
            /*  1. Get access to Rx sensor pointer  (Rx = Base + rxIndex).
                2. Get number of Rx electrodes
                3. Get Rx pointer
                4. Connect all Rx   */
            CapSense_curGangRxPtr = CapSense_curFlashWdgtPtr->ptr2SnsFlash;
            CapSense_curGangRxPtr += rxIndex;
            numElectrodes = CapSense_curGangRxPtr->numPins ;
            CapSense_curRxIOPtr = &CapSense_ioList[CapSense_curGangRxPtr->firstPinId];

            do
            {
                CapSense_CSXConnectRx(CapSense_curRxIOPtr++);
                numElectrodes--;
            } while (0u != numElectrodes);

            /*  1. Get access to Tx sensor pointer (Tx = Base + NumberRx + TxIndex).
                2. Get number of Tx electrodes
                3. Get Tx pointer
                4. Connect all Tx   */
            CapSense_curGangTxPtr = CapSense_curFlashWdgtPtr->ptr2SnsFlash;
            CapSense_curGangTxPtr += (CapSense_curFlashWdgtPtr->numCols + txIndex);

            numElectrodes = CapSense_curGangTxPtr->numPins ;
            CapSense_curTxIOPtr = &CapSense_ioList[CapSense_curGangTxPtr->firstPinId];

            do
            {
                CapSense_CSXConnectTx(CapSense_curTxIOPtr++);
                numElectrodes--;
            }while (0u != numElectrodes);

            /*  Set callback to scan connected ganged sensor only */
            CapSense_CSXPostScanApiPtr = &CapSense_SsCSXPostSingleScan;
        }
        else
    #endif  /* (CapSense_ENABLE == CapSense_CSD_CSX_EN) */
        {
            /*  If no ganged sensor in component OR current widget no contains ganged sns, then:
                1. Find Rx pointer: Rx = Base address + RxIndex
                2. Find Tx pointer: Tx = Base address + (total Rx + TxIndex) */
            CapSense_curRxIOPtr = (CapSense_FLASH_IO_STRUCT const *)CapSense_curFlashWdgtPtr->ptr2SnsFlash;
            CapSense_curRxIOPtr = &CapSense_curRxIOPtr[rxIndex];

            CapSense_curTxIOPtr = (CapSense_FLASH_IO_STRUCT const *)CapSense_curFlashWdgtPtr->ptr2SnsFlash;
            CapSense_curTxIOPtr = &CapSense_curTxIOPtr[(CapSense_curFlashWdgtPtr->numCols + txIndex)];

            /*  Connect Tx and Rx IOs for scan ad set flag to indicate that IOs should be disconnected   */
            CapSense_CSXConnectTx(CapSense_curTxIOPtr);
            CapSense_CSXConnectRx(CapSense_curRxIOPtr);
            CapSense_eleCsxDisconnectFlag = CapSense_DISCONNECT_IO_FLAG;

            /*   Set callback to scan all sensors in widget */
            CapSense_CSXPostScanApiPtr = &CapSense_SsCSXPostSingleScan;
        }
}

/*******************************************************************************
* Function Name: CapSense_CSXScan
****************************************************************************//**
*
* \brief
*  This function initiates the scan for sensors of the widget initialized by
*  the CapSense_CSXSetupWidget() function.
*
* \details
*  This function performs scanning of all sensors in the widget configured by
*  the CapSense_CSXSetupWidget() function. It does the following tasks:
*    1. Connects the first sensor of the widget.
*    2. Initializes an interrupt callback function to initialize a scan of the
*       next sensors in a widget.
*    3. Starts scanning for the first sensor in the widget.
*
*  This function is called by the CapSense_Scan() API if the given
*  widget uses the CSX sensing method.
*
*  It is not recommended to call this function directly from the application
*  layer. This function should be used to implement only the user�s specific
*  use cases (for faster execution time or pipeline scanning for example).
*
*  This function should be called when no scanning in progress. I.e.
*  CapSense_IsBusy() returns a non-busy status. The widget must be
*  preconfigured by the CapSense_CSXSetupWidget() function if other
*  widget was previously scanned or other type of scan functions were used.
*
*******************************************************************************/

void CapSense_CSXScan(void)
{
    #if (CapSense_CSX_GANGED_SNS_EN == 1u)
        uint32 numElectrodes;
    #endif

    /*  Initialise pointer to access sensor RAM elements    */
    CapSense_curRamSnsPtr = CapSense_curFlashWdgtPtr->ptr2SnsRam;

    /*  Calculate total sensor to be scanned. If buttons, one Tx and an Rx for each sensor
        if Matrix button or Touchpad, M x N rule applies    */
    CapSense_curWdgtTotalTx = CapSense_curFlashWdgtPtr->numRows;
    CapSense_curWdgtTotalRx  = CapSense_curFlashWdgtPtr->numCols;

    /*  If component contains CSX ganged sensor, then check if widget contains ganged sensor.   */
    #if (CapSense_CSX_GANGED_SNS_EN == 1u)
        if ((CapSense_curFlashWdgtPtr->staticConfig & CapSense_GANGED_SNS_MASK) != 0u)
        {
            /*  1. Get sns pointer for first Rx
                2. Get number of Rx pin
                3. Get pointer to the Rx pin
                4. Connect all Rx pins   */
            CapSense_curGangRxPtr = CapSense_curFlashWdgtPtr->ptr2SnsFlash;
            numElectrodes = CapSense_curGangRxPtr->numPins ;
            CapSense_curRxIOPtr = &CapSense_ioList[CapSense_curGangRxPtr->firstPinId];
            do
            {
                CapSense_CSXConnectRx(CapSense_curRxIOPtr++);
                numElectrodes--;
            } while (0u != numElectrodes);

            /*  1. Get sns pointer for first Tx (Tx = Base + number of Rx).
                2. Get number of Tx electrodes
                3. Get pointer the Tx electrode
                4. Connect all Tx   */
            CapSense_curGangTxPtr = CapSense_curFlashWdgtPtr->ptr2SnsFlash;
            CapSense_curGangTxPtr += CapSense_curFlashWdgtPtr->numCols;
            numElectrodes = CapSense_curGangTxPtr->numPins ;
            CapSense_curTxIOPtr = &CapSense_ioList[CapSense_curGangTxPtr->firstPinId];

            do
            {
                CapSense_CSXConnectTx(CapSense_curTxIOPtr++);
                numElectrodes--;
            }while (0u != numElectrodes);

            /*  Assign callback to ISR  */
            CapSense_CSXPostScanApiPtr = &CapSense_SsCSXPostMultiScanGanged;

        }
        else
    #endif
        {
            /*  If no ganged sensor in the component OR current widget contains no ganged sns, then:
                1. Find Rx pointer: Rx = Base address.
                2. Find Rx pointer: Tx = Base address + total Rx */
            CapSense_curRxIOPtr = (CapSense_FLASH_IO_STRUCT const *)CapSense_curFlashWdgtPtr->ptr2SnsFlash;
            CapSense_curTxIOPtr = CapSense_curRxIOPtr;
            CapSense_curTxIOPtr = &CapSense_curTxIOPtr[(CapSense_curFlashWdgtPtr->numCols)];

            /*  Connect first Tx and first Rx IOs for scan  */
            CapSense_CSXConnectTx(CapSense_curTxIOPtr);
            CapSense_CSXConnectRx(CapSense_curRxIOPtr);

            /*   Set callback to scan all sensors in widget */
            CapSense_CSXPostScanApiPtr = &CapSense_SsCSXPostMultiScan;
        }

        /*  If frequency hopping is enabled, reset hopping channel counter  */
        #if (CapSense_MULTI_FREQ_SCAN_EN == CapSense_ENABLE)
            CapSense_scanFreqIndex = 0u;
        #endif

    /*  Set scan status, enter critical section and initiate scan   */
    CapSense_dsRam.status |= CapSense_SW_STS_BUSY;
    CapSense_SsCSXStartSample();
}

/*******************************************************************************
* Function Name: CapSense_CSXScanExt()
****************************************************************************//**
*
* \brief
*  Starts the CSX conversion on the preconfigured sensor. The
*  CapSense_CSXSetupWidgetExt() function should be used to setup a
*  widget when using this function.
*
* \details
*  This function performs single scanning of one sensor in the widget configured
*  by CapSense_CSXSetupWidgetExt()  function. It does the following
*  tasks:
*    1. Sets a busy flag in the CapSense_dsRam structure.
*    2. Configures the Tx clock frequency.
*    3. Configures the Modulator clock frequency.
*    4. Configures the IDAC value.
*    5. Starts single scanning.
*
*  It is not recommended to call this function directly from the application
*  layer. This function should be used to implement only the user�s specific
*  use cases (for faster execution time or pipeline scanning for example). This
*  function should be called when no scanning in progress. I.e.
*  CapSense_IsBusy() returns a non-busy status.
*
*  The sensor must be preconfigured by using the
*  CapSense_CSXSetupWidgetExt() API prior to calling this function.
*  The sensor remains ready for the next scan if a previous scan was triggered
*  by using the CapSense_CSXScanExt() function. In this case, calling
*  CapSense_CSXSetupWidgetExt() is not required every time before the
*  CapSense_CSXScanExt() function. If a previous scan was triggered in
*  any other way: CapSense_Scan(), CapSense_ScanAllWidgets()
*  or CapSense_RunTuner() (see the CapSense_RunTuner() function
*  description for more details), the sensor must be preconfigured again by
*  using the CapSense_CSXSetupWidgetExt() API prior to calling the
*  CapSense_CSXScanExt() function.
*
*  If disconnection of the sensors is required after calling
*  CapSense_CSXScanExt(), the CapSense_CSXDisconnectTx() and
*  CapSense_CSXDisconnectRx() APIs can be used.
*
*******************************************************************************/

void CapSense_CSXScanExt(void)
{
    /*  If frequency hopping is enabled, reset hopping channel counter  */
    #if (CapSense_MULTI_FREQ_SCAN_EN == CapSense_ENABLE)
        CapSense_scanFreqIndex = 0u;
    #endif

    /*  Set busy flag and start conversion  */
    CapSense_dsRam.status |= CapSense_SW_STS_BUSY;
    CapSense_SsCSXStartSample();
}


/*******************************************************************************
* Function Name: CapSense_SsCSXPostMultiScan
****************************************************************************//**
*
* \brief
*   ISR function implements the multiple sensor scanning for un-ganged CSX widgets.
*   This function is called by the component ISR only, it should not be used by
*   the application layer.
*
*******************************************************************************/

static void CapSense_SsCSXPostMultiScan(void)
{
    #ifdef CapSense_ENTRY_CALLBACK
        CapSense_EntryCallback();
    #endif /* CapSense_ENTRY_CALLBACK */

    /*  Copy scan result into sensor raw count array    */
    CapSense_curRamSnsPtr->raw[CapSense_scanFreqIndex] = \
                             (CapSense_csxScanPeriod - CapSense_csxRawCount);

    #if (CapSense_MULTI_FREQ_SCAN_EN == CapSense_ENABLE)
    /*  Increment frequency hopping index and imitate next scan if all channels
        are not scanned */
        CapSense_scanFreqIndex++;
        if (CapSense_scanFreqIndex < 3u)
        {
            /*  Change IMO frequency and start sample   */
                CapSense_SsChangeImoFreq (CapSense_scanFreqIndex);
                CapSense_SsCSXStartSample ();
        }
        else
    #endif
        {
            /*  Restore IMO clock, Reset frequency hopping channel counter, if FH is enabled.   */
            #if (CapSense_MULTI_FREQ_SCAN_EN == CapSense_ENABLE)
                CapSense_SsChangeImoFreq (0u);
                CapSense_scanFreqIndex = 0u;
            #endif

            /*  Disconnect Tx electrodes of previously scanned sensor as
                preparation for next sensor scan    */
            CapSense_CSXDisconnectTx (CapSense_curTxIOPtr);

            /*  Initiate scan for next sensor in widget, if all sensors in
                widget are not scanned */
            CapSense_curWdgtTotalTx--;
            if (CapSense_curWdgtTotalTx > 0u)
            {
                /*  1. Increment pointer to next Tx
                    2. Connect next Rx for scan
                    3. Increment pointer to sns data    */
                CapSense_curTxIOPtr++;
                CapSense_CSXConnectTx (CapSense_curTxIOPtr);
                CapSense_curRamSnsPtr++;

                /*  Reset scan result holder, and initiate scan */
                CapSense_SsCSXStartSample();
            }

            /*  Check if all Rx are looped through  */
            else
            {
                CapSense_curWdgtTotalRx--;
                if (CapSense_curWdgtTotalRx > 0u)
                {
                    /*  1. Disconnect current Rx electrode
                        2. Connect next Rx electrode
                        3. Re-initialize total Tx to be scanned per Rx
                        4. Find first Tx electrode again
                        5. Connect first Tx electrode   */
                    CapSense_CSXDisconnectRx(CapSense_curRxIOPtr);
                    CapSense_curRxIOPtr++;
                    CapSense_CSXConnectRx(CapSense_curRxIOPtr);
                    CapSense_curWdgtTotalTx  = CapSense_curFlashWdgtPtr->numRows;
                    CapSense_curTxIOPtr = (CapSense_FLASH_IO_STRUCT const *)CapSense_curFlashWdgtPtr->ptr2SnsFlash;
                    CapSense_curTxIOPtr = & CapSense_curTxIOPtr[(CapSense_curFlashWdgtPtr->numCols)];
                    CapSense_CSXConnectTx(CapSense_curTxIOPtr);

                    /*  Increment RAM sensor pointer pointer    */
                    CapSense_curRamSnsPtr++;

                    /*  Initiate sample */
                    CapSense_SsCSXStartSample();
                }
                else
                {
                    /*  If all sensors are scanned, disconnect Tx   */
                    CapSense_CSXDisconnectRx(CapSense_curRxIOPtr);

                    /*  Call scan next widget API if requested, if not, complete scan   */
                    if (CapSense_requestScanAllWidget != 0u)
                    {
                        CapSense_SsPostAllWidgetsScan();
                    }
                    else
                    {
                        /* All pending scans completed,
                            1. update the counter to indicat same
                            2. Clear busy state as scanning is completed      */
                        CapSense_dsRam.scanCounter++;
                        CapSense_dsRam.status &= ~CapSense_SW_STS_BUSY;
                    }
                }
            }
        }

    #ifdef CapSense_EXIT_CALLBACK
        CapSense_ExitCallBack();
    #endif /* CapSense_EXIT_CALLBACK */
}


/*******************************************************************************
* Function Name: CapSense_SsCSXPostMultiScanGanged
****************************************************************************//**
*
* \brief
*   The ISR function implements the multiple sensor scanning for ganged CSX widgets.
*   This function is called by the Component ISR only, should not be used by
*   the application layer.
*
* \details
*   The ISR function implements the multiple sensor scanning for ganged CSX widgets.
*   This function is called by the Component ISR only, should not be used by
*   the application layer.
*
*******************************************************************************/
#if (CapSense_CSX_GANGED_SNS_EN == 1u)

static void CapSense_SsCSXPostMultiScanGanged(void)
{
    uint32 numElectrodes;

    #ifdef CapSense_ENTRY_CALLBACK
        CapSense_EntryCallback();
    #endif /* CapSense_ENTRY_CALLBACK */

    /*  Copy scan result into sensor raw count array    */
    CapSense_curRamSnsPtr->raw[CapSense_scanFreqIndex] = \
                    (CapSense_csxScanPeriod - CapSense_csxRawCount);

    #if (CapSense_MULTI_FREQ_SCAN_EN == CapSense_ENABLE)
    /*  Increment frequency hopping index and initiate next scan if all channels are not scanned */
        CapSense_scanFreqIndex++;
        if (CapSense_scanFreqIndex < 3u)
        {
            /*  Change IMO frequency and start sample   */
            CapSense_SsChangeImoFreq (CapSense_scanFreqIndex);
            CapSense_SsCSXStartSample();
        }
        else
    #endif
        {
            /*  Restore IMO clock, Reset frequency hopping channel counter, if FH is enabled.   */
            #if (CapSense_MULTI_FREQ_SCAN_EN == CapSense_ENABLE)
                CapSense_SsChangeImoFreq (0u);
                CapSense_scanFreqIndex = 0u;
            #endif

            /*  Disconnect ALL pins of previously scanned Tx     */
            numElectrodes = CapSense_curGangTxPtr->numPins ;
            CapSense_curTxIOPtr = &CapSense_ioList[CapSense_curGangTxPtr->firstPinId];
            do
            {
                CapSense_CSXDisconnectTx (CapSense_curTxIOPtr++);
                numElectrodes--;
            } while (0u != numElectrodes);

            /*  Check if all Tx are scanned, if not, initiate scan for next Tx  */
            CapSense_curWdgtTotalTx--;
            if (CapSense_curWdgtTotalTx > 0u)
            {
                /*  1. Find pointer of next Tx (ganged)
                    2. Connect all Tx pins        */
                CapSense_curGangTxPtr++;
                numElectrodes = CapSense_curGangTxPtr->numPins ;
                CapSense_curTxIOPtr = &CapSense_ioList[CapSense_curGangTxPtr->firstPinId];
                do
                {
                    CapSense_CSXConnectTx(CapSense_curTxIOPtr++);
                    numElectrodes--;
                } while (0u != numElectrodes);

                /*  Increment RAM pointer to next sensor    */
                CapSense_curRamSnsPtr++;

                /*  Reset scan result holder and initiate scan */
                CapSense_SsCSXStartSample();
            }
            else
            {
                /*  Check if all Rx are scanned, if not, initiate scan for next Rx  */
                CapSense_curWdgtTotalRx--;
                if (CapSense_curWdgtTotalRx > 0u)
                {
                    /*  1. Disconnect current Rx ALL pins   */
                    numElectrodes = CapSense_curGangRxPtr->numPins ;
                    CapSense_curRxIOPtr = &CapSense_ioList[CapSense_curGangRxPtr->firstPinId];
                    do
                    {
                        CapSense_CSXDisconnectRx(CapSense_curRxIOPtr++);
                        numElectrodes--;
                    } while (0u != numElectrodes);

                    /*  2. Connect next Rx ALL pins */
                    CapSense_curGangRxPtr++;
                    numElectrodes = CapSense_curGangRxPtr->numPins ;
                    CapSense_curRxIOPtr = &CapSense_ioList[CapSense_curGangRxPtr->firstPinId];
                    do
                    {
                        CapSense_CSXConnectRx(CapSense_curRxIOPtr++);
                        numElectrodes--;
                    } while (0u != numElectrodes);

                    /*  3. Re-initialize total Tx to be scanned per Rx
                        4. Find first Tx  again
                        5. Connect first Tx ALL pins    */
                    CapSense_curWdgtTotalTx = CapSense_curFlashWdgtPtr->numRows;
                    CapSense_curGangTxPtr = CapSense_curFlashWdgtPtr->ptr2SnsFlash;
                    CapSense_curGangTxPtr += CapSense_curFlashWdgtPtr->numCols;

                    /*  5. Connect Tx ALL pins    */
                    numElectrodes = CapSense_curGangTxPtr->numPins ;
                    CapSense_curTxIOPtr = &CapSense_ioList[CapSense_curGangTxPtr->firstPinId];
                    do
                    {
                        CapSense_CSXConnectTx(CapSense_curTxIOPtr++);
                        numElectrodes--;
                    } while (0u != numElectrodes);

                    /*  Increment RAM pointer to next sensor    */
                    CapSense_curRamSnsPtr++;

                    /*  Reset scan result holder and initiate scan */
                    CapSense_SsCSXStartSample();
                }
                else
                {
                    /*  Disconnect ALL electrodes of previously scanned Rx   */
                    numElectrodes = CapSense_curGangRxPtr->numPins ;
                    CapSense_curRxIOPtr = &CapSense_ioList[CapSense_curGangRxPtr->firstPinId];
                    do
                    {
                        CapSense_CSXDisconnectRx(CapSense_curRxIOPtr++);
                        numElectrodes--;
                    } while (0u != numElectrodes);

                    /*  Call scan next widget API if requested, if not, complete scan   */
                    if (CapSense_requestScanAllWidget != 0u)
                    {
                        CapSense_SsPostAllWidgetsScan();
                    }
                    else
                    {
                        /* All pending scans completed,
                            1. update the counter to indicat same
                            2. Clear busy state as scanning is completed      */
                        CapSense_dsRam.scanCounter++;
                        CapSense_dsRam.status &= ~CapSense_SW_STS_BUSY;
                    }
                }
            }
        }

    #ifdef CapSense_EXIT_CALLBACK
        CapSense_ExitCallBack();
    #endif /* CapSense_EXIT_CALLBACK */
}

#endif  /*  end of (CapSense_CSX_GANGED_SNS_EN == 1u)   */

/*******************************************************************************
* Function Name: CapSense_SsCSXPostSingleScan
****************************************************************************//**
*
* \brief
*   The ISR function for single-sensor scanning implementation.
*
* \details
*   This function is used by CapSense ISR in the CSX scanning mode to implement
*   the single-sensor scanning.
*   This function should not be used by the application layer.
*
*******************************************************************************/

static void CapSense_SsCSXPostSingleScan(void)
{
    #ifdef CapSense_ENTRY_CALLBACK
        CapSense_EntryCallback();
    #endif /* CapSense_ENTRY_CALLBACK */

    /*  Copy scan result into sensor raw count array and clear busy flag
        No sensor is connected or disconnected in this function
        Calling CapSense_ScanExt from application would perform
        one more scan of same sensor. Specifically for low-power mode.   */
    CapSense_curRamSnsPtr->raw[CapSense_scanFreqIndex] = \
                        (CapSense_csxScanPeriod - CapSense_csxRawCount);

    /*  If frequency hopping is enabled */
    #if (CapSense_MULTI_FREQ_SCAN_EN == CapSense_ENABLE)
        /*  Increment frequency hopping index and initiate next scan if all
            channels are not scanned */
        if (++CapSense_scanFreqIndex < 3u)
        {
            /*  Change IMO frequency and start sample   */
            CapSense_SsChangeImoFreq (CapSense_scanFreqIndex);
            CapSense_SsCSXStartSample ();
        }
        else
    #endif
        {
            /*  Restore IMO clock   */
            #if (CapSense_MULTI_FREQ_SCAN_EN == CapSense_ENABLE)
                CapSense_SsChangeImoFreq (0u);
            #endif

            /* All pending scans completed,
                1. update the counter to indicat same
                2. Clear busy state as scanning is completed    */
            CapSense_dsRam.scanCounter++;
            CapSense_dsRam.status &= ~CapSense_SW_STS_BUSY;
        }

    #ifdef CapSense_EXIT_CALLBACK
        CapSense_ExitCallBack();
    #endif /* CapSense_EXIT_CALLBACK */
}

/*******************************************************************************
* Function Name: CapSense_CSXCalibrateWidget
****************************************************************************//**
*
* \brief
*  Calibrates the raw count values of all sensors/nodes in a CSX widget.
*
* \details
*  Performs a successive approximation search algorithm to find appropriate
*  IDAC values for sensors in the specified widget that provides a raw count
*  to the level specified by the target parameter.
*
*  This function is available when the CSX Enable IDAC auto-calibration
*  parameter is enabled.
*
* \param  widgetIndex
*  Specify the ID number of the CSX widget to calibrate its raw count.
*  A macro for the widget ID can be found in the
*  CapSense_Configuration.h file defined as
*  CapSense_<WidgetName>_WDGT_ID.
*
* \param  target
*  Specify the calibration target in percentages of the maximum raw count.
*
*******************************************************************************/
#if (CapSense_CSX_IDAC_AUTOCAL_EN == CapSense_ENABLE)

void CapSense_CSXCalibrateWidget(uint32 widgetIndex, uint16 target)
{
    /*  Declare and initialise ptr to widget and sensor structures        */
    CapSense_RAM_SNS_STRUCT *snsRamPtr = CapSense_dsFlash.wdgtArray[widgetIndex].ptr2SnsRam;
    CapSense_RAM_WD_BASE_STRUCT *ptrWdgt = (CapSense_RAM_WD_BASE_STRUCT *)
                                          CapSense_dsFlash.wdgtArray[widgetIndex].ptr2WdgtRam;
    /*  Current IDAC bit in use
        next idac bit to be used
        counters for loop execution
        variable to calculate raw count target      */
    uint8 curIdacMask = (1u << (CapSense_CSX_IDAC_BITS_USED-1u));
    uint8 nextIdacMask = (curIdacMask >> 1u);
    uint32 calibrationIndex, totalSns;
    uint32 rawTarget;

    #if(CapSense_ENABLE == CapSense_CSDV2)
        #if (0u != CapSense_CSX_COMMON_TX_CLK_EN)
            rawTarget = (uint32)CapSense_dsRam.snsCsxClk;
        #else
            rawTarget = (uint32)ptrWdgt->snsClk;
        #endif /* (0u != CapSense_CSX_COMMON_TX_CLK_EN) */

        CapSense_csxScanPeriod  = CapSense_SsCalcCsxScanPeriod (ptrWdgt);
        rawTarget = CapSense_csxScanPeriod;
    #else
        /*  Find max count       */
        rawTarget = CapSense_csxScanPeriod = CapSense_SsCalcCsxScanPeriod (ptrWdgt);
    #endif /* (CapSense_ENABLE == CapSense_CSDV2) */

    /*  Calculate target raw count      */
    rawTarget = (rawTarget * target);
    rawTarget = (rawTarget / CapSense_PERCENTAGE_100);

    if (CapSense_SENSE_METHOD_CSX_E == \
       (CapSense_SENSE_METHOD_ENUM)CapSense_GET_SENSE_METHOD(&CapSense_dsFlash.wdgtArray[widgetIndex]))
    {
        /*  Find out total sensors/nodes in widget  */
        totalSns = ((uint32)CapSense_dsFlash.wdgtArray[widgetIndex].numCols *
                    (uint32)CapSense_dsFlash.wdgtArray[widgetIndex].numRows);

        /*  Clear raw count registers and IDAC registers of all sensors/nodes   */
        for (calibrationIndex = 0u; calibrationIndex < totalSns; calibrationIndex++)
        {
            snsRamPtr->raw[0u] = 0u;
            snsRamPtr->idacComp[0u] = curIdacMask;
            #if (CapSense_MULTI_FREQ_SCAN_EN == CapSense_ENABLE)
                snsRamPtr->raw[1] = 0u;
                snsRamPtr->raw[2] = 0u;
                snsRamPtr->idacComp[1] = curIdacMask;
                snsRamPtr->idacComp[2] = curIdacMask;
            #endif
            snsRamPtr++;
        }

        /*  1. Perform binary search for accurate IDAC value for each sensor/node   */
        do
        {
            /*  1. Get access to sensor data    */
            snsRamPtr = (CapSense_RAM_SNS_STRUCT *)CapSense_dsFlash.wdgtArray[widgetIndex].ptr2SnsRam;

            /*  Scan all sensors/nodes in widget    */
            (void)CapSense_SetupWidget(widgetIndex);
            (void)CapSense_Scan();
            while ((*(volatile uint8 *)&CapSense_dsRam.status & CapSense_SW_STS_BUSY) != 0u)
            {
            }

            /*  1.2 Check raw count and adjust IDAC, loop through all sensors/nodes */
            for (calibrationIndex = 0u; calibrationIndex < totalSns; calibrationIndex++)
            {
                /*  1.2.1 Check if current raw count is above target,
                        if yes, clear the MS bit of bit
                        if no, keep the MS bit and set next bit     */
                if (snsRamPtr->raw[0u] > rawTarget)
                {
                    snsRamPtr->idacComp[0u] &= (uint8)(~curIdacMask);
                }
                snsRamPtr->idacComp[0u] |= nextIdacMask;

                #if (CapSense_MULTI_FREQ_SCAN_EN == CapSense_ENABLE)
                    if (snsRamPtr->raw[1u] > rawTarget)
                    {
                        snsRamPtr->idacComp[1u] &= (uint8)(~curIdacMask);
                    }

                    if (snsRamPtr->raw[2u] > rawTarget)
                    {
                        snsRamPtr->idacComp[2u] &= (uint8)(~curIdacMask);
                    }

                    snsRamPtr->idacComp[1u] |= nextIdacMask;
                    snsRamPtr->idacComp[2u] |= nextIdacMask;
                #endif
                snsRamPtr++;
            }

            /*  Shift both current idac and pre Idac values to right by 1   */
            curIdacMask = nextIdacMask;
            nextIdacMask = nextIdacMask >> 1u;
        }
        while (curIdacMask != 0u);
    }
}

#endif /* (CapSense_CSX_IDAC_AUTOCAL_EN == CapSense_ENABLE) */

#if(CapSense_ENABLE == CapSense_CSDV2)
/*******************************************************************************
* Function Name: CapSense_SsCSXStartSample
****************************************************************************//**
*
* \brief
*   Starts scanning for the CSX widget.
*
* \details
*   Starts scanning for the CSX widget.
*
*******************************************************************************/
static void CapSense_SsCSXStartSample(void)
{
    uint32 tmpRegVal;

    tmpRegVal = (uint32)CapSense_curRamSnsPtr->idacComp[CapSense_scanFreqIndex];
    tmpRegVal = (tmpRegVal & CapSense_IDAC_MOD_VAL_MASK) | CapSense_DEFAULT_CSD_IDACA_CFG;
    CY_SET_REG32(CapSense_IDAC_MOD_PTR, tmpRegVal);

    /*  Clear scan result holder    */
    CapSense_csxRawCount = 0u;
    CapSense_SsCSXStartSampleExt();
}
#else
/*******************************************************************************
* Function Name: CapSense_SsCSXStartSample
****************************************************************************//**
*
* \brief
*   Starts scanning for the CSX widget.
*
* \param  widgetIndex The ID of the widget to be calibrated.
*
* \details
*
*******************************************************************************/
static void CapSense_SsCSXStartSample(void)
{
    uint8 critSect;

    CY_SET_REG32(CapSense_IDAC_PTR, (CapSense_IDAC_MOD_MODE_VARIABLE |
                                (uint32)CapSense_curRamSnsPtr->idacComp[CapSense_scanFreqIndex]));

    /*  Clear scan result holder    */
    CapSense_csxRawCount = 0u;

    /*  Disable interrupts and initiate scan  */
    critSect = CyEnterCriticalSection();
    CapSense_SsCSXStartSampleExt();
    CyExitCriticalSection(critSect);
}
#endif /* (CapSense_ENABLE == CapSense_CSDV2) */


/*******************************************************************************
* Function Name: CapSense_CSXConnectTx
****************************************************************************//**
*
* \brief
*  Connects a TX electrode to the CSX scanning hardware.
*
* \details
*  This function connects a port pin (Tx electrode) to the CSD_SENSE signal.
*  It is assumed that the drive mode of the port pin is already set to STRONG
*  in the HSIOM_PORT_SELx register.
*
*  It is not recommended to call this function directly from the application
*  layer. This function should be used to implement only the user�s specific
*  use cases (for faster execution time when there is only one port pin for an
*  electrode for example).
*
* \param  txPtr
*  Specify the pointer to the FLASH_IO_STRUCT object belonging to a sensor
*  which should be connected to the CapSense block as Tx pin.
*
*******************************************************************************/
void CapSense_CSXConnectTx(CapSense_FLASH_IO_STRUCT const *txPtr)
{
    uint32 regValue;

    regValue = CY_GET_REG32 (txPtr->pcPtr);
    regValue |= (CapSense_GPIO_STRGDRV << txPtr->shift);
    CY_SET_REG32 (txPtr->pcPtr, regValue);

    regValue = CY_GET_REG32 (txPtr->hsiomPtr);
    regValue |= (CapSense_HSIOM_SEL_CSD_SENSE << txPtr->hsiomShift);
    CY_SET_REG32 (txPtr->hsiomPtr, regValue);
}

/*******************************************************************************
* Function Name: CapSense_CSXConnectRx
****************************************************************************//**
*
* \brief
*  Connects an RX electrode to the CSX scanning hardware.
*
* \details
*  This function connects a port pin (Rx electrode)to AMUXBUS-A and sets the
*  drive mode of the port pin to HIgh-Z in the GPIO_PRT_PCx register.
*
*  It is not recommended to call this function directly from the application
*  layer. This function should be used to implement only the user�s specific
*  use cases (for faster execution time when there is only one port pin for an
*  electrode for example).
*
* \param  rxPtr
*  Specify the pointer to the FLASH_IO_STRUCT object belonging to a sensor
*  which should be connected to the CapSense block as Rx pin.
*
*******************************************************************************/
void CapSense_CSXConnectRx(CapSense_FLASH_IO_STRUCT const *rxPtr)
{
    uint32 regValue;

    regValue = CY_GET_REG32 (rxPtr->pcPtr);
    regValue &= ~(CapSense_GPIO_PC_MASK << rxPtr->shift);
    CY_SET_REG32 (rxPtr->pcPtr, regValue);

    regValue = CY_GET_REG32 (rxPtr->hsiomPtr);
    regValue |=  (CapSense_HSIOM_SEL_AMUXA << rxPtr->hsiomShift);
    CY_SET_REG32 (rxPtr->hsiomPtr, regValue);
}

/*******************************************************************************
* Function Name: CapSense_CSXDisconnectTx
****************************************************************************//**
*
* \brief
*  Disconnects a TX electrode from the CSX scanning hardware.
*
* \details
*  This function disconnects a port pin (Tx electrode) from the CSD_SENSE
*  signal and configures the port pin to the strong drive mode. It is assumed
*  that the data register (GPIO_PRTx_DR) of the port pin is already 0.
*
*  It is not recommended to call this function directly from the application
*  layer. This function should be used to implement only the user�s specific
*  use cases (for faster execution time when there is only one port pin for an
*  electrode for example).
*
* \param  txPtr
*  Specify the pointer to the FLASH_IO_STRUCT object belonging to a Tx pin
*  sensor which should be disconnected from the CapSense block.
*
*******************************************************************************/
void CapSense_CSXDisconnectTx(CapSense_FLASH_IO_STRUCT const *txPtr)
{
    uint32 regValue;

    regValue = CY_GET_REG32 (txPtr->hsiomPtr);
    regValue &= ~(txPtr->hsiomMask);
    CY_SET_REG32 (txPtr->hsiomPtr, regValue);
}

/*******************************************************************************
* Function Name: CapSense_CSXDisconnectRx
****************************************************************************//**
*
* \brief
*  Disconnects an RX electrode from the CSX scanning hardware.
*
* \details
*  This function disconnects a port pin (Rx electrode) from AMUXBUS_A and
*  configures the port pin to the strong drive mode. It is assumed that the
*  data register (GPIO_PRTx_DR) of the port pin is already 0.
*
*  It is not recommended to call this function directly from the application
*  layer. This function should be used to implement only the user�s specific
*  use cases (for faster execution time when there is only one port pin for
*  an electrode for example).
*
* \param  rxPtr
*  Specify the pointer to the FLASH_IO_STRUCT object belonging to a Rx pin
*  sensor which should be disconnected from the CapSense block.
*
*******************************************************************************/
void CapSense_CSXDisconnectRx(CapSense_FLASH_IO_STRUCT const *rxPtr)
{
    uint32 regValue;

    regValue = CY_GET_REG32 (rxPtr->hsiomPtr);
    regValue &= ~(rxPtr->hsiomMask);
    CY_SET_REG32 (rxPtr->hsiomPtr, regValue);

    regValue = CY_GET_REG32 (rxPtr->pcPtr);
    regValue &= ~(CapSense_GPIO_PC_MASK << rxPtr->shift);
    regValue |=  (CapSense_GPIO_STRGDRV << rxPtr->shift);
    CY_SET_REG32 (rxPtr->pcPtr, regValue);

    regValue = CY_GET_REG32 (rxPtr->drPtr);
    regValue &=  ~(rxPtr->mask);
    CY_SET_REG32 (rxPtr->drPtr, regValue);
}


/*******************************************************************************
* Function Name: CapSense_SsCSXGetTxClkDivider
****************************************************************************//**
*
* \brief
*  This function gets the Sense Clock Divider value for one-dimension widgets
*  and the Column Sense Clock divider value for two-dimension widgets.
*
* \details
*  This function gets the Sense Clock Divider value based on the component
*  configuration. The function is applicable for one-dimension widgets and for
*  two-dimension widgets.
*
* \param
*  wdgtIndex Specifies the ID of the widget.
*
* \return The Sense Clock Divider value for one-dimension widgets
*         and the Column Sense Clock divider value for two-dimension widgets.
*
*******************************************************************************/
uint32 CapSense_SsCSXGetTxClkDivider(uint32 wdgtIndex)
{
    uint32 retVal;

    /* Get sense divider based on configuration */
    #if (CapSense_ENABLE != CapSense_CSX_COMMON_TX_CLK_EN)
        CapSense_RAM_WD_BASE_STRUCT *ptrWdgt;

        ptrWdgt = (CapSense_RAM_WD_BASE_STRUCT *)
        CapSense_dsFlash.wdgtArray[wdgtIndex].ptr2WdgtRam;

        retVal = (uint32)(ptrWdgt->snsClk);
    #else
        retVal = (uint32)CapSense_dsRam.snsCsxClk;
    #endif /* (CapSense_ENABLE == CapSense_CSX_COMMON_TX_CLK_EN) */

    if(wdgtIndex == 0uL)
    {
        ;
    }

    return (retVal);
}


/*******************************************************************************
* Function Name: CapSense_SsCalcCsxScanPeriod
****************************************************************************//**
*
* \brief
*   Calculates scan  period for CSX
*
* \details
*   This function calculates scan period for CSX considering
*
* \param  wdgtPtr The pointer to a register map object that contains widget
*                     information.
*
* \return     Returns the CSX scan period for the widget.
*
*******************************************************************************/
static uint16 CapSense_SsCalcCsxScanPeriod(CapSense_RAM_WD_BASE_STRUCT const *wdgtPtr)
{
    uint32 temp;

    #if(CapSense_DISABLE == CapSense_CSX_COMMON_TX_CLK_EN)
        temp = (uint32)(wdgtPtr->snsClk);
    #else
        temp = (uint32)(CapSense_dsRam.snsCsxClk);
    #endif /* (CapSense_DISABLE == CapSense_CSX_COMMON_TX_CLK_EN) */

    #if(CapSense_ENABLE == CapSense_CSDV2)
        temp  = LO16((uint32)wdgtPtr->resolution * temp);
        temp -= LO16((uint32)wdgtPtr->resolution * CapSense_CSX_DEADBAND_CYCLES_NUMBER);
    #else
        #if(CapSense_ENABLE == CapSense_IS_M0S8PERI_BLOCK)
            temp = (temp / (uint32)CapSense_dsRam.modCsxClk);
        #endif /* (CapSense_ENABLE == CapSense_IS_M0S8PERI_BLOCK) */
            
        temp = ((uint32)wdgtPtr->resolution * CapSense_NUM_HALF_CYCLES * temp);
    #endif /* (CapSense_ENABLE == CapSense_CSDV2) */

    /*  return the scan period */
    return ((uint16)temp);
}


#if(CapSense_ENABLE == CapSense_CSDV2)

    /*******************************************************************************
    * Function Name: CapSense_SsCSXScanISR
    ****************************************************************************//**
    *
    * \brief
    *  This is the handler of the WDT interrupt in CPU NVIC. The handler is executed
    *  when normal sample is complete.
    *
    * \details
    *  This handler covers the following functionality:
    *   - Read result of measurement a store it into the corresponding register of
    *     the data structure.
    *   - If the Noise Metric functionality is enabled then check the number of bad
    *     conversions and repeat scan of the current sensor of the number of bad
    *     conversions is greater than Noise Metric Threshold.
    *   - Initiate scan of the next sensor for the multiple sensor scanning mode.
    *   - Update the Status register in the data structure.
    *   - Switch CSDv2 HW IP block to the default state is scanning of all sensors is
    *     completed.
    *
    *******************************************************************************/
    CY_ISR(CapSense_SsCSXScanISR)
    {
    #if(0u != CapSense_CSX_NOISE_METRIC_EN)
        uint32 tmpRegVal;
    #endif /* (0u != CapSense_CSX_NOISE_METRIC_EN) */

        CyIntDisable(CapSense_ISR_NUMBER);

        CY_SET_REG32(CapSense_INTR_PTR, CapSense_DEFAULT_CSD_INTR_CFG);

        CapSense_csxRawCount  = (uint16)(CY_GET_REG32(CapSense_RESULT_VAL1_PTR) &\
                                                             CapSense_RESULT_VAL1_VALUE_MASK);

        CapSense_csxRawCount += (uint16)(CY_GET_REG32(CapSense_RESULT_VAL2_PTR) &\
                                                             CapSense_RESULT_VAL2_VALUE_MASK);

        /* This workaround neded to prevent overflow in the SW register map. Cypress ID #234358 */
        if(CapSense_csxRawCount > CapSense_csxScanPeriod)
        {
            CapSense_csxRawCount = CapSense_csxScanPeriod;
        }

    #if(0u != CapSense_CSX_NOISE_METRIC_EN)
        tmpRegVal = CY_GET_REG32(CapSense_RESULT_VAL1_PTR) >> CapSense_RESULT_VAL1_BAD_CONVS_SHIFT;

        if((tmpRegVal > CapSense_CSX_NOISE_METRIC_TH) && (CapSense_resamplingCyclesCnt > 0uL))
        {
            CY_SET_REG32(CapSense_SEQ_START_PTR, CapSense_SCAN_CSD_SEQ_START_CFG);
            CapSense_resamplingCyclesCnt--;
        }
        else
        {
            CapSense_CSXPostScanApiPtr();
            CapSense_resamplingCyclesCnt = CapSense_RESAMPLING_CYCLES_MAX_NUMBER;
        }
    #else
        CapSense_CSXPostScanApiPtr();
    #endif /* (0u != CapSense_CSX_NOISE_METRIC_EN) */

        if(CapSense_NOT_BUSY == (CapSense_dsRam.status & CapSense_SW_STS_BUSY))
        {
            CY_SET_REG32(CapSense_CONFIG_PTR, CapSense_DEFAULT_CSD_CONFIG_CFG);
            CY_SET_REG32(CapSense_CSDCMP_PTR, CapSense_DEFAULT_CSD_CSDCMP_CFG);
            CY_SET_REG32(CapSense_IDAC_MOD_PTR, CapSense_DEFAULT_CSD_IDACA_CFG);
        }

        CyIntEnable(CapSense_ISR_NUMBER);
    }


    /*******************************************************************************
    * Function Name: CapSense_SsCSXStartSampleExt
    ****************************************************************************//**
    *
    * \brief
    *   Starts the CSDV2 sequencer to perform the CSX conversion.
    *   specific widget.
    *
    * \details
    *   This function covers the following functionality:
    *      1. Configures the CSDV2 sequencer to perform the coarse initialization.
    *      2. Waiting for completion of the coarse initialization.
    *      3. Configures the CSDV2 sequencer to perform the normal conversion.
    *      4. Starts the normal conversion
    *
    *******************************************************************************/
    CY_INLINE static void CapSense_SsCSXStartSampleExt(void)
    {
        uint32 filterDelay;
        uint32 watchdogCounter;
        #if (CapSense_MOD_CSD_CLK_12MHZ < CYDEV_BCLK__HFCLK__HZ)
            uint32 sampleClkFreqHz;
        #endif /* (CapSense_MOD_CSD_CLK_12MHZ < CYDEV_BCLK__HFCLK__HZ)  */

        /* Configure the CSDV2 sequencer to perform the coarse initialization. */
        CY_SET_REG32(CapSense_CONFIG_PTR,           CapSense_DEFAULT_CSD_CONFIG_CFG);
        CY_SET_REG32(CapSense_HSCMP_PTR,            CapSense_PRECHARGE_CSD_HSCMP_CFG);
        CY_SET_REG32(CapSense_SW_HS_P_SEL_PTR,      CapSense_PRECHARGE_CSD_SW_HS_P_SEL_CFG);
        CY_SET_REG32(CapSense_SW_HS_N_SEL_PTR,      CapSense_PRECHARGE_CSD_SW_HS_N_SEL_CFG);
        CY_SET_REG32(CapSense_SW_DSI_SEL_PTR,       CapSense_PRECHARGE_CSD_SW_DSI_SEL_CFG);
        CY_SET_REG32(CapSense_SW_SHIELD_SEL_PTR,    CapSense_PRECHARGE_CSD_SW_SHIELD_SEL_CFG);
        CY_SET_REG32(CapSense_SW_FW_MOD_SEL_PTR,    CapSense_PRECHARGE_CSD_SW_FW_MOD_SEL_CFG);
        CY_SET_REG32(CapSense_SW_FW_TANK_SEL_PTR,   CapSense_PRECHARGE_CSD_SW_FW_TANK_SEL_CFG);
        CY_SET_REG32(CapSense_SEQ_START_PTR,        CapSense_PRECHARGE_CSD_SEQ_START_CFG);

        /* Wait for the HSCMP trigger and retutn the sequencer to the IDLE state. */
        watchdogCounter = CapSense_CSX_PRECHARGE_WATCHDOG_CYCLES_NUM;
        while((0u != (CapSense_SEQ_START_START_MASK & CY_GET_REG32(CapSense_SEQ_START_PTR))) && (0u != watchdogCounter))
        {
            watchdogCounter--;
        }

        /* Reset the sequencer to the IDLE state if HSCMP not triggered till watchdog period is out. */
        if(0u != (CapSense_SEQ_START_START_MASK & CY_GET_REG32(CapSense_SEQ_START_PTR)))
        {
            CY_SET_REG32(CapSense_SEQ_START_PTR,  CapSense_DEFAULT_CSD_SEQ_START_CFG);
        }

        #if (CapSense_MOD_CSD_CLK_12MHZ < CYDEV_BCLK__HFCLK__HZ)
            sampleClkFreqHz = CYDEV_BCLK__HFCLK__HZ / (uint32)CapSense_dsRam.modCsxClk;
            if(sampleClkFreqHz <= CapSense_MOD_CSD_CLK_12MHZ)
            {
                filterDelay = CapSense_CONFIG_FILTER_DELAY_12MHZ;
            }
            else if(sampleClkFreqHz <= CapSense_MOD_CSD_CLK_24MHZ)
            {
                filterDelay = CapSense_CONFIG_FILTER_DELAY_24MHZ;
            }
            else
            {
                filterDelay = CapSense_CONFIG_FILTER_DELAY_48MHZ;
            }
        #else
            filterDelay = CapSense_CONFIG_FILTER_DELAY_12MHZ;
        #endif /* (CapSense_MOD_CSD_CLK_12MHZ < CYDEV_BCLK__HFCLK__HZ)  */

        /* Configure the CSDV2 sequencer to perform the normal conversion. */
        CY_SET_REG32(CapSense_CONFIG_PTR, CapSense_PRESCAN_CSD_CONFIG_CFG | filterDelay);
        CY_SET_REG32(CapSense_CSDCMP_PTR, CapSense_PRESCAN_CSD_CSDCMP_CFG);
        CY_SET_REG32(CapSense_HSCMP_PTR,            CapSense_DEFAULT_CSD_HSCMP_CFG);
        CY_SET_REG32(CapSense_SW_HS_P_SEL_PTR,      CapSense_DEFAULT_CSD_SW_HS_P_SEL_CFG);
        CY_SET_REG32(CapSense_SW_HS_N_SEL_PTR,      CapSense_DEFAULT_CSD_SW_HS_N_SEL_CFG);
        CY_SET_REG32(CapSense_SW_DSI_SEL_PTR,       CapSense_DEFAULT_CSD_SW_DSI_SEL_CFG);
        CY_SET_REG32(CapSense_SW_SHIELD_SEL_PTR,    CapSense_DEFAULT_CSD_SW_SHIELD_SEL_CFG);
        CY_SET_REG32(CapSense_SW_FW_MOD_SEL_PTR,    CapSense_DEFAULT_CSD_SW_FW_MOD_SEL_CFG);
        CY_SET_REG32(CapSense_SW_FW_TANK_SEL_PTR,   CapSense_DEFAULT_CSD_SW_FW_TANK_SEL_CFG);
        CY_SET_REG32(CapSense_INTR_PTR,             CapSense_DEFAULT_CSD_INTR_CFG);

        /* Start the normal conversion. */
        CY_SET_REG32(CapSense_SEQ_START_PTR, CapSense_SCAN_CSD_SEQ_START_CFG);
    }


#endif /* (CapSense_ENABLE == CapSense_CSDV2) */
#endif  /*  (0u != CapSense_CSX_EN) */


/* [] END OF FILE */
