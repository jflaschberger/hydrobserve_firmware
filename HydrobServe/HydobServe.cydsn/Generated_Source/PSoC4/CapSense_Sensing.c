/***************************************************************************//**
* \file CapSense_Sensing.c
* \version 3.0
*
* \brief
*   This file contains the source of functions common for
*   different sensing methods.
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

#include "CapSense_Sensing.h"
#include "CapSense_Structure.h"
#include "cyfitter.h"

/***************************************
* API Constants
***************************************/

#define CapSense_WIDGET_NUM_32                          (32u)
#define CapSense_WIDGET_NUM_32_DIV_SHIFT                (5u)
#define CapSense_WIDGET_NUM_32_MASK                     (0x0000001FLu)
#define CapSense_CALIBRATION_RESOLUTION                 (12u)
#define CapSense_COARSE_TRIM_THRESHOLD_1                (40u)
#define CapSense_COARSE_TRIM_THRESHOLD_2                (215u)
#define CapSense_FREQUENCY_OFFSET_5                     (20u)
#define CapSense_FREQUENCY_OFFSET_10                    (40u)
#define CapSense_CALIBRATION_FREQ_KHZ                   (1500u)
#define CapSense_CALIBRATION_MD                         (2u)
#define CapSense_MIN_IMO_FREQ_KHZ                       (6000u)
#define CapSense_CSD_AUTOTUNE_CAL_LEVEL                 (CapSense_CSD_RAWCOUNT_CAL_LEVEL)
#define CapSense_CP_MIN                                 (0u)
#define CapSense_CP_MAX                                 (65000Lu)
#define CapSense_CP_ERROR                               (4000Lu)
#define CapSense_CLK_SOURCE_LFSR_SCALE_OFFSET           (4u)

#if (CapSense_CLK_SOURCE_DIRECT != CapSense_CSD_SNS_CLK_SOURCE)
    #if (CapSense_ENABLE == CapSense_CSDV2)
        #define CapSense_PRS_FACTOR_DIV                 (2u)
    #else
        #define CapSense_PRS_FACTOR_DIV                 (1u)
    #endif /* (CapSense_ENABLE == CapSense_CSDV2) */
#else
    #define CapSense_PRS_FACTOR_DIV                     (0u)
#endif /* (CapSense_CLK_SOURCE_DIRECT != CapSense_CSD_SNS_CLK_SOURCE) */

#define CapSense_FLIP_FLOP_DIV                          (1u)

#define CapSense_MOD_CSD_CLK_12000KHZ                   (12000uL)
#define CapSense_MOD_CSD_CLK_24000KHZ                   (24000uL)
#define CapSense_MOD_CSD_CLK_48000KHZ                   (48000uL)

#if ((CapSense_CLK_SOURCE_PRS8 == CapSense_CSD_SNS_CLK_SOURCE) || \
    (CapSense_CLK_SOURCE_PRS12 == CapSense_CSD_SNS_CLK_SOURCE) || \
    (CapSense_CLK_SOURCE_PRSAUTO == CapSense_CSD_SNS_CLK_SOURCE))
    #define CapSense_FACTOR_FILTER_DELAY_12MHZ          (2u)
#else
    #define CapSense_FACTOR_FILTER_DELAY_12MHZ          (4u)
#endif /* (CapSense_CLK_SOURCE_DIRECT != CapSense_CSD_SNS_CLK_SOURCE) */

#define CapSense_FACTOR_MOD_SNS                         (8u)
#define CapSense_UINT8_MAX_VAL                          (0xFFu)
#define CapSense_MSB_OFFSET                             (8u)

/*****************************************************************************/
/* Enumeration types definition                                               */
/*****************************************************************************/

typedef enum
{
    CapSense_RES_PULLUP_E   = 0x02u,
    CapSense_RES_PULLDOWN_E = 0x03u
} CapSense_PORT_TEST_DM;

typedef enum
{
    CapSense_STS_RESET      = 0x01u,
    CapSense_STS_NO_RESET   = 0x02u
} CapSense_TEST_TYPE;


/*******************************************************************************
* Static Function Prototypes
*******************************************************************************/
/**
* \if SECTION_CAPSENSE_INTERNAL
* \addtogroup group_capsense_internal
* \{
*/

#if (CapSense_DISABLE == CapSense_CSDV2)
    static void CapSense_SsTrimIdacs(void);
    #if ((CapSense_ENABLE == CapSense_CSX_EN) || \
         (CapSense_IDAC_SINKING == CapSense_CSD_IDAC_CONFIG))
        static void CapSense_SsTrimIdacsSinking(void);
    #endif /* ((CapSense_ENABLE == CapSense_CSX_EN) || \
               (CapSense_IDAC_SINKING == CapSense_CSD_IDAC_CONFIG)) */
    #if ((CapSense_ENABLE == CapSense_CSX_EN) || \
         (CapSense_IDAC_SOURCING == CapSense_CSD_IDAC_CONFIG))
        static void CapSense_SsTrimIdacsSourcing(void);
    #endif /* ((CapSense_ENABLE == CapSense_CSX_EN) || \
               (CapSense_IDAC_SOURCING == CapSense_CSD_IDAC_CONFIG)) */
#endif /* (CapSense_ENABLE == CapSense_CSDV2) */
#if ((CapSense_ENABLE == CapSense_CSD_CSX_EN) || \
     (CapSense_ENABLE == CapSense_ADC_EN))
    #if (CapSense_ENABLE == CapSense_CSD_EN)
        static void CapSense_SsCSDDisableMode(void);
    #endif /* (CapSense_ENABLE == CapSense_CSD_EN) */
    #if (CapSense_ENABLE == CapSense_CSX_EN)
        static void CapSense_SsDisableCSXMode(void);
    #endif /* (CapSense_ENABLE == CapSense_CSX_EN) */
#endif /* ((CapSense_ENABLE == CapSense_CSD_CSX_EN) || \
           (CapSense_ENABLE == CapSense_ADC_EN)) */
#if (CapSense_CSD_SS_DIS != CapSense_CSD_AUTOTUNE)
    static void CapSense_SsSetDirectClockMode(void);
#endif /* (CapSense_CSD_SS_DIS != CapSense_CSD_AUTOTUNE) */


/** \}
* \endif */

/*******************************************************************************
* Define module variables
*******************************************************************************/

#if ((CapSense_ENABLE == CapSense_CSD_CSX_EN) || \
     (CapSense_ENABLE == CapSense_ADC_EN))
    CapSense_SENSE_METHOD_ENUM CapSense_currentSenseMethod = CapSense_UNDEFINED_E;
#endif /* ((CapSense_ENABLE == CapSense_CSD_CSX_EN) || \
           (CapSense_ENABLE == CapSense_ADC_EN))) */

#if(CapSense_ENABLE == CapSense_MULTI_FREQ_SCAN_EN)
    /*  Module variable keep track of frequency hopping channel index   */
    uint8 CapSense_scanFreqIndex = 0u;
    /*  Variable keep frequency offsets */
    uint8 CapSense_immunity[CapSense_NUM_SCAN_FREQS] = {0u, 0u, 0u};
#else
    /* const allows C-compiler to do optimization */
    const uint8 CapSense_scanFreqIndex = 0u;
#endif /* (CapSense_ENABLE == CapSense_MULTI_FREQ_SCAN_EN) */

/* Global software variables */
volatile uint8 CapSense_widgetIndex = 0u;    /* Index of the scanning widget */
volatile uint8 CapSense_sensorIndex = 0u;    /* Index of the scanning sensor */
uint8 CapSense_requestScanAllWidget = 0u;
#if (CapSense_CSD_SS_DIS != CapSense_CSD_AUTOTUNE)
    uint8 CapSense_prescalersTuningDone = 0u;
#endif /* (CapSense_CSD_SS_DIS != CapSense_CSD_AUTOTUNE) */

/* Pointer to RAM_SNS_STRUCT structure  */
CapSense_RAM_SNS_STRUCT *CapSense_curRamSnsPtr;

#if ((CapSense_ENABLE == CapSense_CSD_GANGED_SNS_EN) || \
     (CapSense_ENABLE == CapSense_CSX_EN))
    /*  Pointer to Flash structure holding configuration of widget to be scanned  */
    CapSense_FLASH_WD_STRUCT const *CapSense_curFlashWdgtPtr = 0u;
#endif /* ((CapSense_ENABLE == CapSense_CSD_GANGED_SNS_EN) || \
           (CapSense_ENABLE == CapSense_CSX_EN))  */
#if (CapSense_ENABLE == CapSense_CSD_GANGED_SNS_EN)
    /*  Pointer to Flash structure holding info of sensor to be scanned  */
    CapSense_FLASH_SNS_STRUCT const *CapSense_curFlashSnsPtr = 0u;
#endif /* (CapSense_ENABLE == CapSense_CSD_GANGED_SNS_EN) */

#if(((CapSense_ENABLE == CapSense_CSX_EN) && (CapSense_ENABLE == CapSense_CSDV2) && (CapSense_CLK_SOURCE_PRSAUTO == CapSense_CSX_TX_CLK_SOURCE)) ||\
    ((CapSense_ENABLE == CapSense_CSD_EN) && (CapSense_ENABLE == CapSense_CSDV2) && (CapSense_CLK_SOURCE_PRSAUTO == CapSense_CSD_SNS_CLK_SOURCE)))
    static CY_INLINE uint8 CapSense_SsCalcLfsrSize(uint32 modClkDivider, uint32 snsClkDivider, uint32 conversionsNum);
    static CY_INLINE uint8 CapSense_SsCalcLfsrScale(uint32 snsClkDivider, uint8 lfsrSize);
#endif /* (((CapSense_ENABLE == CapSense_CSX_EN) && (CapSense_ENABLE == CapSense_CSDV2) && (CapSense_CLK_SOURCE_PRSAUTO == CapSense_CSX_TX_CLK_SOURCE)) ||\
    ((CapSense_ENABLE == CapSense_CSD_EN) && (CapSense_ENABLE == CapSense_CSDV2) && (CapSense_CLK_SOURCE_PRSAUTO == CapSense_CSD_SNS_CLK_SOURCE))) */

#if (CapSense_ENABLE == CapSense_CSD_EN)
    void CapSense_SsSetWidgetSenseClkSrc(uint32 wdgtIndex, CapSense_RAM_WD_BASE_STRUCT * ptrWdgt);
#endif /* (CapSense_ENABLE == CapSense_CSD_EN) */

#if ((CapSense_ENABLE == CapSense_CSX_EN) && (CapSense_ENABLE == CapSense_CSDV2))
    void CapSense_SsSetWidgetTxClkSrc(uint32 wdgtIndex, CapSense_RAM_WD_BASE_STRUCT * ptrWdgt);
#endif /* ((CapSense_ENABLE == CapSense_CSX_EN) && (CapSense_ENABLE == CapSense_CSDV2)) */

/*******************************************************************************
* Function Name: CapSense_IsBusy
****************************************************************************//**
*
* \brief
*  Returns the current status of the component (scan completed or scan in
*  progress).
*
* \details
*  This function returns the status of the hardware block whether a scan is
*  currently in progress or not. If the component is busy, no new scan or setup
*  widgets should be made. It is recommended to use the critical section (i.e.
*  disable global interrupt) in the application when the device transitions from
*  the active mode to sleep or deep sleep mode.
*
* \return
*  Returns the current status of the component:
*    - CapSense_NOT_BUSY if there is no scan in progress and a next scan
*      can be initiated.
*    - CapSense_SW_STS_BUSY if the previous scanning is not completed
*      and the hardware block is busy.
*
*******************************************************************************/
uint32 CapSense_IsBusy(void)
{
    return ((*(volatile uint8 *)&CapSense_dsRam.status) & CapSense_SW_STS_BUSY);
}

/*******************************************************************************
* Function Name: CapSense_SetupWidget
****************************************************************************//**
*
* \brief
*  Performs the initialization required to scan the specified widget.
*
* \details
*  This function prepares the component to scan all sensors in the specified
*  widget, by executing the following tasks.
*    1. Re-initialize the hardware if it is not configured to perform the
*       sensing method used by the specified widget, this happens only if the
*       CSD and CSX methods are used in the component.
*    2. Initialize the hardware with specific sensing configuration (e.g.
*       sensor clock, scan resolution) used by the widget.
*    3. Disconnect all previously connected electrodes, if the electrodes
*       connected by the CapSense_CSDSetupWidgetExt(),
*       CapSense_CSXSetupWidgetExt() or CapSense_CSDConnectSns()
*       functions and not disconnected.
*
*  This function does not start sensor scanning, the CapSense_Scan()
*  function must be called to start the scan sensors in the widget. If this
*  function is called more than once, it does not break the component operation,
*  but only the last initialized widget is in effect.

*
* \param wdgtIndex
*  Specify the ID number of the widget to be initialized for scanning.
*  A macro for the widget ID can be found in the
*  CapSense_Configuration.h file defined as
*  CapSense_<WidgetName>_WDGT_ID
*
* \return
*  Returns the status of the widget setting up operation:
*    - CYRET_SUCCESS if the operation is successfully completed.
*    - CYRET_BAD_PARAM if the widget is invalid or if the specified widget is
*      disabled
*    - CYRET_INVALID_STATE if the previous scanning is not completed and the
*      hardware block is busy.
*    - CYRET_UNKNOWN if an unknown sensing method is used by the widget or
*      other spurious errors.
*
**********************************************************************************/
cystatus CapSense_SetupWidget(uint32 wdgtIndex)
{
    cystatus widgetStatus;

    if (CapSense_WDGT_SW_STS_BUSY == (CapSense_dsRam.status & CapSense_WDGT_SW_STS_BUSY))
    {
        /* Previous widget is being scanned. Return error. */
        widgetStatus = CYRET_INVALID_STATE;
    }
    /*
     *  Check if widget id is valid, specified widget is enabled and widget did not
     *  detect any fault conditions if BIST is enabled. If all conditions are met,
     *  set widgetStatus as good, if not, set widgetStatus as bad.
     */
    else if ((CapSense_TOTAL_WIDGETS > wdgtIndex) &&
        (0u != CapSense_GET_WIDGET_EN_STATUS(wdgtIndex)))

    {
        widgetStatus = CYRET_SUCCESS;
    }
    else
    {
        widgetStatus = CYRET_BAD_PARAM;
    }

    /*
     * Check widgetStatus flag that is set earlier, if flag is good, then set up only
     * widget
     */
    if (CYRET_SUCCESS == widgetStatus)
    {
        /*  Check if CSD2X is enabled   */
        #if (CapSense_ENABLE == CapSense_CSD2X_EN)
            /******************************
            * This is the place where CapSense_SetupWidget2x(uint32 slotIndex)
            * API should be implemented.
            * CSD2X will be implemented
            * in the next component version.
            *********************************/
            widgetStatus = CYRET_UNKNOWN;
            #error "CSD2x not implemented in this version of component"

        #elif (CapSense_ENABLE == CapSense_CSD_CSX_EN)
            /*  Check widget sensing method is CSX and call CSX APIs    */
            if (CapSense_SENSE_METHOD_CSX_E ==
                (CapSense_SENSE_METHOD_ENUM)CapSense_GET_SENSE_METHOD(&CapSense_dsFlash.wdgtArray[wdgtIndex]))
            {
                /*  Set up widget for CSX scan  */
                CapSense_CSXSetupWidget(wdgtIndex);
            }
            /*  Check widget sensing method is CSD and call appropriate API */
            else if (CapSense_SENSE_METHOD_CSD_E ==
                     (CapSense_SENSE_METHOD_ENUM)CapSense_GET_SENSE_METHOD(&CapSense_dsFlash.wdgtArray[wdgtIndex]))
            {
                /*  Set up widget for CSD scan  */
                CapSense_CSDSetupWidget(wdgtIndex);
            }
            else
            {
                /*  Sensing method is invalid, return error to caller  */
                widgetStatus = CYRET_UNKNOWN;
            }
        #elif (CapSense_ENABLE == CapSense_CSD_EN)
            /*  Set up widget for scan */
            CapSense_CSDSetupWidget(wdgtIndex);
        #elif (CapSense_ENABLE == CapSense_CSX_EN)
            /*  Set up widgets for scan     */
            CapSense_CSXSetupWidget(wdgtIndex);
        #else
            widgetStatus = CYRET_UNKNOWN;
            #error "No sensing method enabled, component cannot work in this mode"
        #endif  /* (CapSense_ENABLE == CapSense_CSD2X_EN) */
    }

    return (widgetStatus);
}


/*******************************************************************************
* Function Name: CapSense_Scan
****************************************************************************//**
*
* \brief
*  Initiates scan of all sensors in the widget which is initialized by
*  CapSense_SetupWidget(), if the no scan is in progress.
*
* \details
*  This function should be called only after the CapSense_SetupWidget()
*  function is called to start the scanning of the sensors in the widget. The
*  status of a sensor scan must be checked using the CapSense_IsBusy()
*  API prior to starting a next scan or setting up another widget.
*
* \return
*  Returns the status of the scan initiation operation:
*    - CYRET_SUCCESS if scanning is successfully started.
*    - CYRET_INVALID_STATE if the previous scanning is not completed and the
*      hardware block is busy.
*    - CYRET_UNKNOWN if an unknown sensing method is used by the widget.
*
********************************************************************************/
cystatus CapSense_Scan(void)
{
    cystatus scanStatus = CYRET_SUCCESS;

    if (CapSense_WDGT_SW_STS_BUSY == (CapSense_dsRam.status & CapSense_WDGT_SW_STS_BUSY))
    {
        /* Previous widget is being scanned. Return error. */
        scanStatus = CYRET_INVALID_STATE;
    }
    else
    {
        /*  If CSD2X is enabled, call CSD2X scan    */
    #if (CapSense_ENABLE == CapSense_CSD2X_EN)
        scanStatus = CapSense_Scan2x();

    /*  If both CSD and CSX are enabled, call scan API based on widget sensing method    */
    #elif (CapSense_ENABLE == CapSense_CSD_CSX_EN)
        /*  Check widget sensing method and call appropriate APIs   */
        switch (CapSense_currentSenseMethod)
        {
            case CapSense_SENSE_METHOD_CSX_E:
                CapSense_CSXScan();
                break;

            case CapSense_SENSE_METHOD_CSD_E:
                 CapSense_CSDScan();
                break;

            default:
                scanStatus = CYRET_UNKNOWN;
                break;
        }

    /*  If only CSD is enabled, call CSD scan   */
    #elif (CapSense_ENABLE == CapSense_CSD_EN)
        CapSense_CSDScan();

    /*  If only CSX is enabled, call CSX scan   */
    #elif (CapSense_ENABLE == CapSense_CSX_EN)
        CapSense_CSXScan();

    #else
        scanStatus = CYRET_UNKNOWN;
        #error "No sensing method enabled, component cannot work in this mode"
    #endif  /* (CapSense_ENABLE == CapSense_CSD2X_EN) */
    }

    return (scanStatus);
}


/*******************************************************************************
* Function Name: CapSense_ScanAllWidgets
****************************************************************************//**
*
* \brief
*  Initializes the first enabled widget and scans of all the sensors in the
*  widget, then the same process is repeated for all widgets in the component.
*  I.e. scan all the widgets in the component.
*
* \details
*  This function initializes a widget and scans all the sensors in the widget,
*  and then repeats the same for all the widgets in the component. The tasks of
*  the CapSense_SetupWidget() and CapSense_Scan() functions are
*  executed by these functions. The status of a sensor scan must be checked
*  using the the CapSense_IsBusy() API prior to starting a next scan
*  or setting up another widget.
*
* \return
*  Returns the status of operation:
*    - CYRET_SUCCESS if scanning is successfully started.
*    - CYRET_BAD_PARAM if all the widgets are disabled.
*    - CYRET_INVALID_STATE if the previous scanning is not completed and the HW block is busy.
*    - CYRET_UNKNOWN if there are unknown errors.
*
*******************************************************************************/
cystatus CapSense_ScanAllWidgets(void)
{
    cystatus scanStatus = CYRET_UNKNOWN;

    #if (CapSense_DISABLE == CapSense_CSD2X_EN)
        uint32 wdgtIndex;

        if (CapSense_SW_STS_BUSY == (CapSense_dsRam.status & CapSense_SW_STS_BUSY))
        {
            /* Previous widget is being scanned. Return error. */
            scanStatus = CYRET_INVALID_STATE;
        }
        else
        {
            /*
             *  Set up widget first widget.
             *  If widget returned error, set up next, continue same until widget does not return error.
             */
            for (wdgtIndex = 0u;
                 wdgtIndex < CapSense_TOTAL_WIDGETS;
                 wdgtIndex++)
            {

                scanStatus = CapSense_SetupWidget(wdgtIndex);

                if (CYRET_SUCCESS == scanStatus)
                {
                    #if (0u != (CapSense_TOTAL_WIDGETS - 1u))
                        /* If there are more than one widget to be scanned, request callback to scan next widget */
                        if ((CapSense_TOTAL_WIDGETS - 1u) > wdgtIndex)
                        {
                             /* Request callback to scan next widget in ISR */
                            CapSense_requestScanAllWidget = CapSense_ENABLE;
                        }
                        else
                        {
                            /* Request to exit in ISR (Do not scan the next widgets) */
                            CapSense_requestScanAllWidget = CapSense_DISABLE;
                        }
                    #else
                        {
                            /* Request to exit in ISR (We have only one widget) */
                            CapSense_requestScanAllWidget = CapSense_DISABLE;
                        }
                    #endif  /* (0u != (CapSense_TOTAL_WIDGETS - 1u)) */

                    /*  Initiate scan and quit loop */
                    scanStatus = CapSense_Scan();

                    break;
                }
            }
        }
    #else
        /******************************
        * This is the place where the Scan2x
        * setup should be implemented.
        * Scan2x will be implemented
        * in the next component version.
        *********************************/
    #endif /* (CapSense_DISABLE == CapSense_CSD2X_EN) */

    return (scanStatus);
}


/*******************************************************************************
* Function Name: CapSense_SsInitialize
****************************************************************************//**
*
* \brief
*   Performs hardware and firmware initialization required for proper operation
*   of the CapSense component. This function is called from
*   the CapSense_Start() API prior to calling any other APIs of the component.
*
* \details
*   Performs hardware and firmware initialization required for proper operation
*   of the CapSense component. This function is called from
*   the CapSense_Start() API prior to calling any other APIs of the component.
*   1. The function initializes immunity offsets when the frequency hopping is
*      enabled.
*   2. Depending on the configuration, the function initializes the CSD block
*      for the CSD2X, CSD, CSX, or CSD+CSX modes.
*   3. The function updates the dsRam.wdgtWorking variable with 1 when Self Test
*      is enabled.
*
*   Calling the CapSense_Start API is the recommended method to initialize
*   the CapSense component at power-up. The CapSense_SsInitialize()
*   API should not be used for initialization, resume, or wake-up operations.
*   The dsRam.wdgtWorking variable is updated.
*
* \return status
*   Returns status of operation:
*   - Zero        - Indicates successful initialization.
*   - Non-zero    - One or more errors occurred in the initialization process.
*
*******************************************************************************/
cystatus CapSense_SsInitialize(void)
{
    cystatus initStatus = CYRET_SUCCESS;

    #if (CapSense_ENABLE == CapSense_MULTI_FREQ_SCAN_EN)
        CapSense_immunity[CapSense_FREQ_CHANNEL_0] = (uint8)CY_GET_REG32(CY_SYS_CLK_IMO_TRIM1_PTR);
        if (CapSense_COARSE_TRIM_THRESHOLD_1 > CapSense_immunity[CapSense_FREQ_CHANNEL_0])
        {
            CapSense_immunity[CapSense_FREQ_CHANNEL_1] =
            CapSense_immunity[CapSense_FREQ_CHANNEL_0] + CapSense_FREQUENCY_OFFSET_5;
            CapSense_immunity[CapSense_FREQ_CHANNEL_2] =
            CapSense_immunity[CapSense_FREQ_CHANNEL_0] + CapSense_FREQUENCY_OFFSET_10;
        }
        else if (CapSense_COARSE_TRIM_THRESHOLD_2 > CapSense_immunity[CapSense_FREQ_CHANNEL_0])
        {
            CapSense_immunity[CapSense_FREQ_CHANNEL_1] =
            CapSense_immunity[CapSense_FREQ_CHANNEL_0] - CapSense_FREQUENCY_OFFSET_5;
            CapSense_immunity[CapSense_FREQ_CHANNEL_2] =
            CapSense_immunity[CapSense_FREQ_CHANNEL_0] + CapSense_FREQUENCY_OFFSET_5;
        }
        else
        {
            CapSense_immunity[CapSense_FREQ_CHANNEL_1] =
            CapSense_immunity[CapSense_FREQ_CHANNEL_0] - CapSense_FREQUENCY_OFFSET_5;
            CapSense_immunity[CapSense_FREQ_CHANNEL_2] =
            CapSense_immunity[CapSense_FREQ_CHANNEL_0] - CapSense_FREQUENCY_OFFSET_10;
        }
    #endif /* (CapSense_ENABLE == CapSense_MULTI_FREQ_SCAN_EN) */

    #if (CapSense_DISABLE == CapSense_CSDV2)
        CapSense_SsTrimIdacs();
    #endif /* (CapSense_ENABLE == CapSense_CSDV2) */

    #if((CapSense_ENABLE == CapSense_CSD_EN) ||\
    ((CapSense_ENABLE == CapSense_CSX_EN) && (CapSense_ENABLE == CapSense_CSDV2)))
        CapSense_SsInitializeSourceSenseClk();
    #endif /* ((CapSense_ENABLE == CapSense_CSD_EN) ||\
              ((CapSense_ENABLE == CapSense_CSX_EN) && (CapSense_ENABLE == CapSense_CSDV2))) */

    /*
     *  Check if CSD2X is enabled, if yes, initialize CapSense component
     *  for CSD2x operation
     */
    #if (CapSense_ENABLE == CapSense_CSD2X_EN)
        /*
         *  Check if CSX or CSD enable along with CSD2X, if yes, provide the error
         *  message as CSD2X can not coexist with CSD or CSX
         */
        #if (CapSense_ENABLE == CapSense_CSX_EN)
            #error "CSD2X and CSX can not be enabled together"
            initStatus = CYRET_UNKNOWN;
        #elif (CapSense_ENABLE == CapSense_CSD_EN)
            #error "CSD2X and CSD can not be enabled together"
            initStatus = CYRET_UNKNOWN;
        #endif /* (CapSense_CSX_EN == CapSense_ENABLE) */

        /* Initialize both CSD blocks for CSD2X scanning    */
        CapSense_CSD2XInitialize();

    #elif (CapSense_ENABLE == CapSense_CSD_CSX_EN)
        /*
         * CSD hardware shall be initialized in the Setup Widget based
         * on widget sensing method
         */
        CapSense_currentSenseMethod = CapSense_UNDEFINED_E;

        /* Set all IO states to default state  */
        CapSense_SsSetIOsInDefaultState();
        #if (0u != CapSense_CSD_TOTAL_SHIELD_COUNT)
            CapSense_SsCSDDisableShieldElectrodes();
        #endif /* (0u != CapSense_CSD_TOTAL_SHIELD_COUNT) */
    #elif (CapSense_ENABLE == CapSense_CSD_EN)
        /*  Initialize CSD block for CSD scanning   */
        CapSense_SsCSDInitialize();

    #elif (CapSense_ENABLE == CapSense_CSX_EN)
        /*  Initialize CSD block for CSX scanning   */
        CapSense_CSXInitialize();

    #else
        #error "No sensing method enabled, component cannot work in this mode"
        initStatus = CYRET_UNKNOWN;
    #endif  /* (CapSense_ENABLE == CapSense_CSD2X_EN) */

    return (initStatus);
}


/*******************************************************************************
* Function Name: CapSense_SetPinState
****************************************************************************//**
*
* \brief
*  Sets the state (drive mode and output state) of port pin used by a sensor.
*  The possible states are GND, Shield, High-Z, Tx or Rx, sensor. If the sensor
*  specified in the input parameter is a ganged sensor, then state of all pins
*  associated with ganged sensor is updated.
*
* \details
*  This function sets a specified state to the specified sensor pin. If the
*  sensor is a ganged sensor, then the specified state is also set for all
*  ganged pins of this sensor. Scanning should be completed before calling
*  this API.
*
*  The CapSense_SHIELD and CapSense_SENSOR states are not
*  allowed if there is no CSD widget configured in the user’s project.
*  The CapSense_TX_PIN and CapSense_RX_PIN states are not
*  allowed if there is no CSX widget configured in the user’s project.
*
*  It is not recommended to call this function directly from the application
*  layer. This function should be used to implement only the user’s specific
*  use cases. Functions that perform a setup and scan of the sensor/widget
*  automatically set needed pin states. They do not take into account changes
*  in the design made by the CapSense_SetPinState() function.
*
*  \param wdgtIndex
*  Specify the ID number of the widget to change the pin state of the specified
*  sensor.
*  A macro for the widget ID can be found in CapSense_Configuration.h
*  file defined as CapSense_<WidgetName>_WDGT_ID Specifies the ID of
*  the widget.
*
*  \param snsIndex
*  Specify the ID number of the sensor within the widget to change its pin
*  state.
*  A macro for the sensor ID within a specified widget can be found in
*  CapSense_Configuration.h file defined as
*  CapSense_<WidgetName>_SNS<SensorNumber>_ID
*
*  \param state
*   Specify the state of the sensor that needs to be set:
*     1. CapSense_GROUND - The pin is connected to ground.
*     2. CapSense_HIGHZ - The drive mode of the pin is set to High-Z
*        Analog.
*     3. CapSense_SHIELD - The shield signal is routed to the pin (only
*        in CSD sensing method when shield electrode is enabled).
*     4. CapSense_SENSOR – The pin is connected to the scanning bus
*        (only in CSD sensing method).
*     5. CapSense_TX_PIN – The TX signal is routed to the sensor
*        (only in CSX sensing method).
*     6. CapSense_RX_PIN – The pin is connected to the scanning bus
*        (only in CSX sensing method).
*
*******************************************************************************/
void CapSense_SetPinState(uint32 wdgtIndex, uint32 snsIndex, uint32 state)
{
    CapSense_FLASH_IO_STRUCT const *curSnsIOPtr;
    #if (CapSense_ENABLE == CapSense_CSD_GANGED_SNS_EN)
        CapSense_FLASH_SNS_STRUCT const *curFlashSnsPtr;
        uint32 tempVal;
    #endif /* (CapSense_ENABLE == CapSense_CSD_GANGED_SNS_EN) */
    uint32 newRegisterValue;
    uint8  interruptState;
    uint32 pinHSIOMShift;
    uint32 pinModeShift;

    #if (CapSense_ENABLE == CapSense_CSD_GANGED_SNS_EN)
        /* Check ganged sns flag  */
        if (CapSense_GANGED_SNS_MASK == (CapSense_dsFlash.wdgtArray[wdgtIndex].staticConfig &
            CapSense_GANGED_SNS_MASK))
        {
            curFlashSnsPtr = CapSense_dsFlash.wdgtArray[wdgtIndex].ptr2SnsFlash;
            curFlashSnsPtr += snsIndex;
            curSnsIOPtr = &CapSense_ioList[curFlashSnsPtr->firstPinId];

            /* Get number of ganged pins  */
            tempVal = curFlashSnsPtr->numPins;
        }
        else
        {
            curSnsIOPtr = (CapSense_FLASH_IO_STRUCT const *)
                                    CapSense_dsFlash.wdgtArray[wdgtIndex].ptr2SnsFlash + snsIndex;
            /* There are no ganged pins */
            tempVal = 1u;
        }
    #else
        curSnsIOPtr = (CapSense_FLASH_IO_STRUCT const *)
                                    CapSense_dsFlash.wdgtArray[wdgtIndex].ptr2SnsFlash + snsIndex;
    #endif  /* (CapSense_ENABLE == CapSense_CSD_GANGED_SNS_EN) */

    #if (CapSense_ENABLE == CapSense_CSD_GANGED_SNS_EN)
        /* Reconfigure all ganged sensors  */
        do
        {
    #endif  /* (CapSense_ENABLE == CapSense_CSD_GANGED_SNS_EN) */

            /* Get HSIOM and port mode shifts */
            pinHSIOMShift = (uint32)curSnsIOPtr->hsiomShift;
            pinModeShift = (uint32)curSnsIOPtr->shift;

            /* Reset HSIOM register */
            CY_SET_REG32(curSnsIOPtr->hsiomPtr, CY_GET_REG32(curSnsIOPtr->hsiomPtr) & ~(CapSense_HSIOM_SEL_MASK << pinHSIOMShift));

            switch (state)
            {
            case CapSense_GROUND:
                interruptState = CyEnterCriticalSection();

                /* Update port configuration register (drive mode) for sensor */
                newRegisterValue = CY_GET_REG32(curSnsIOPtr->pcPtr);
                newRegisterValue &= ~(CapSense_GPIO_PC_MASK << pinModeShift);
                newRegisterValue |= (CapSense_SNS_GROUND_CONNECT << pinModeShift);
                CY_SET_REG32(curSnsIOPtr->pcPtr, newRegisterValue);

                CyExitCriticalSection(interruptState);

                /* Set logic 0 to port data register */
                CY_SET_REG32(curSnsIOPtr->drPtr, CY_GET_REG32(curSnsIOPtr->drPtr) & (uint32)~(uint32)((uint32)1u << curSnsIOPtr->drShift));
                break;

            case CapSense_HIGHZ:
                interruptState = CyEnterCriticalSection();

                /* Update port configuration register (drive mode) for sensor */
                newRegisterValue = CY_GET_REG32(curSnsIOPtr->pcPtr);
                newRegisterValue &= ~(CapSense_GPIO_PC_MASK << pinModeShift);
                CY_SET_REG32(curSnsIOPtr->pcPtr, newRegisterValue);

                CyExitCriticalSection(interruptState);

                /* Set logic 0 to port data register */
                CY_SET_REG32(curSnsIOPtr->drPtr, CY_GET_REG32(curSnsIOPtr->drPtr) & (uint32)~(uint32)((uint32)1u << curSnsIOPtr->drShift));
                break;

            #if (CapSense_ENABLE == CapSense_CSD_SHIELD_EN)
                case CapSense_SHIELD:
                    /* Set drive mode to Analog */
                    CY_SET_REG32(curSnsIOPtr->pcPtr, CY_GET_REG32(curSnsIOPtr->pcPtr) &
                               ~(CapSense_GPIO_PC_MASK <<(curSnsIOPtr->shift)));

                    /* Set appropriate bit in HSIOM register to configure pin to shield mode */
                    CY_SET_REG32(curSnsIOPtr->hsiomPtr, CY_GET_REG32(curSnsIOPtr->hsiomPtr) |
                                                       (CapSense_HSIOM_SEL_CSD_SHIELD << pinHSIOMShift));
                    break;
            #endif  /* (CapSense_ENABLE == CapSense_CSD_SHIELD_EN) */

            #if ((CapSense_ENABLE == CapSense_CSD_EN) || \
                 (CapSense_ENABLE == CapSense_CSD_CSX_EN))
                case CapSense_SENSOR:
                    /* Enable sensor */
                    CapSense_CSDConnectSns(curSnsIOPtr);
                    break;
            #endif  /* ((CapSense_ENABLE == CapSense_CSD_EN) || \
                        (CapSense_ENABLE == CapSense_CSD_CSX_EN)) */

            #if ((CapSense_ENABLE == CapSense_CSX_EN) || \
                 (CapSense_ENABLE == CapSense_CSD_CSX_EN))
                case CapSense_TX_PIN:
                    CY_SET_REG32(curSnsIOPtr->hsiomPtr, CY_GET_REG32(curSnsIOPtr->hsiomPtr) | (CapSense_HSIOM_SEL_CSD_SENSE << pinHSIOMShift));
                    CY_SET_REG32(curSnsIOPtr->pcPtr, CY_GET_REG32(curSnsIOPtr->pcPtr) | (CapSense_GPIO_STRGDRV << pinModeShift));
                    break;

                case CapSense_RX_PIN:
                    CY_SET_REG32(curSnsIOPtr->hsiomPtr, CY_GET_REG32(curSnsIOPtr->hsiomPtr) | (CapSense_HSIOM_SEL_AMUXA << pinHSIOMShift));
                    CY_SET_REG32(curSnsIOPtr->pcPtr, CY_GET_REG32(curSnsIOPtr->pcPtr) & ~(CapSense_GPIO_PC_MASK << pinModeShift));
                    break;
            #endif  /* ((CapSense_ENABLE == CapSense_CSX_EN) || \
                        (CapSense_ENABLE == CapSense_CSD_CSX_EN)) */

            default:
                /* Wrong input */
                break;
            }

    #if (CapSense_ENABLE == CapSense_CSD_GANGED_SNS_EN)
            curSnsIOPtr++;
            tempVal--;
        } while (0u != tempVal);
    #endif  /* (CapSense_ENABLE == CapSense_CSD_GANGED_SNS_EN) */
}


#if ((CapSense_ENABLE == CapSense_CSD_CSX_EN) || \
     (CapSense_ENABLE == CapSense_ADC_EN))

    #if (CapSense_ENABLE == CapSense_CSD_EN)
        /*******************************************************************************
        * Function Name: CapSense_SsCSDDisableMode
        ****************************************************************************//**
        *
        * \brief
        *  This function disables CSD mode.
        *
        * \details
        *  To disable CSD mode the following tasks are performed:
        *  1. Disconnect Cmod from AMUXBUS-A;
        *  2. Disconnect previous CSX electrode if it has been connected;
        *  3. Disconnect Csh from AMUXBUS-B;
        *  4. Disable Shield Electrodes.
        *
        *******************************************************************************/
        static void CapSense_SsCSDDisableMode(void)
        {
            uint32 newRegValue;

            /* Disconnect Cmod from AMUXBUS-A using HSIOM registers */
            newRegValue = CY_GET_REG32(CapSense_CMOD_HSIOM_PTR);
            newRegValue &= (uint32)(~(uint32)CapSense_CMOD_HSIOM_MASK);
            CY_SET_REG32(CapSense_CMOD_HSIOM_PTR, newRegValue);

            #if ((CapSense_ENABLE == CapSense_CSDV2) && \
                 ((CapSense_ENABLE == CapSense_CSD_IDAC_COMP_EN) && \
                 (CapSense_ENABLE == CapSense_CSD_DEDICATED_IDAC_COMP_EN)))
                /* Disconnect IDACA and IDACB */
                newRegValue = CY_GET_REG32(CapSense_SW_REFGEN_SEL_PTR);
                newRegValue &= (uint32)(~CapSense_SW_REFGEN_SEL_SW_IAIB_MASK);
                CY_SET_REG32(CapSense_SW_REFGEN_SEL_PTR, newRegValue);
            #endif /* ((CapSense_ENABLE == CapSense_CSDV2) && \
                       ((CapSense_ENABLE == CapSense_CSD_IDAC_COMP_EN) && \
                       (CapSense_ENABLE == CapSense_CSD_DEDICATED_IDAC_COMP_EN))) */

            /* Disconnect previous CSD electrode if it has been connected */
            CapSense_SsCSDElectrodeCheck();

            /* Disconnect Csh from AMUXBUS-B */
            #if ((CapSense_ENABLE == CapSense_CSD_SHIELD_EN) && \
                 (CapSense_ENABLE == CapSense_CSD_SHIELD_TANK_EN))
                newRegValue = CY_GET_REG32(CapSense_CSH_HSIOM_PTR);
                newRegValue &= (uint32)(~(uint32)(CapSense_CSH_TO_AMUXBUS_B_MASK << CapSense_CSH_HSIOM_SHIFT));
                CY_SET_REG32(CapSense_CSH_HSIOM_PTR, newRegValue);
            #endif /* ((CapSense_ENABLE == CapSense_CSD_SHIELD_EN) && \
                       (CapSense_ENABLE == CapSense_CSD_SHIELD_TANK_EN)) */

            #if ((CapSense_ENABLE == CapSense_CSD_SHIELD_EN) && \
                 (0u != CapSense_CSD_TOTAL_SHIELD_COUNT))
                CapSense_SsCSDDisableShieldElectrodes();
            #endif /* ((CapSense_ENABLE == CapSense_CSD_SHIELD_EN) && \
                       (0u != CapSense_CSD_TOTAL_SHIELD_COUNT)) */
        }
    #endif /* (CapSense_ENABLE == CapSense_CSD_EN) */


    #if (CapSense_ENABLE == CapSense_CSX_EN)
        /*******************************************************************************
        * Function Name: CapSense_SsDisableCSXMode
        ****************************************************************************//**
        *
        * \brief
        *  This function disables CSX mode.
        *
        * \details
        *  To disable CSX mode the following tasks are performed:
        *  1. Disconnect CintA and CintB from AMUXBUS-A
        *  2. Disconnect previous CSX electrode if it has been connected
        *
        *******************************************************************************/
        static void CapSense_SsDisableCSXMode(void)
        {
            uint32 newRegValue;

            /*  Disconnect CintA from AMUXBUS-A using HSIOM registers.   */
            newRegValue = CY_GET_REG32(CapSense_CintA_HSIOM_PTR);
            newRegValue &= (uint32)(~(uint32)CapSense_CintA_HSIOM_MASK);
            CY_SET_REG32(CapSense_CintA_HSIOM_PTR, newRegValue);

            /*  Disconnect CintB from AMUXBUS-A using HSIOM registers.   */
            newRegValue = CY_GET_REG32(CapSense_CintB_HSIOM_PTR);
            newRegValue &= (uint32)(~(uint32)CapSense_CintB_HSIOM_MASK);
            CY_SET_REG32(CapSense_CintB_HSIOM_PTR, newRegValue);

            /* Disconnect previous CSX electrode if it has been connected */
            CapSense_CSXElectrodeCheck();
        }
    #endif /* (CapSense_ENABLE == CapSense_CSX_EN) */


    /*******************************************************************************
    * Function Name: CapSense_SsSwitchSensingMode
    ****************************************************************************//**
    *
    * \brief
    *  This function changes the mode for case when both CSD and CSX widgets are
    *  scanned.
    *
    * \details
    *  To switch to the CSD mode the following tasks are performed:
    *  1. Disconnect CintA and CintB from AMUXBUS-A;
    *  2. Disconnect previous CSD electrode if it has been connected;
    *  3. Initialize CSD mode.
    *
    *  To switch to the CSX mode the following tasks are performed:
    *  1. Disconnect Cmod from AMUXBUS-A;
    *  2. Disconnect previous CSX electrode if it has been connected;
    *  3. Initialize CSX mode.
    *
    * \param mode Specifies the scan mode:
    *           -  (1) CapSense_SENSE_METHOD_CSD_E
    *           -  (2) CapSense_SENSE_METHOD_CSX_E
    *
    *******************************************************************************/
    void CapSense_SsSwitchSensingMode(CapSense_SENSE_METHOD_ENUM mode)
    {
        if (CapSense_currentSenseMethod != mode)
        {
            if (CapSense_SENSE_METHOD_CSD_E ==  CapSense_currentSenseMethod)
            {
                #if (CapSense_ENABLE == CapSense_CSD_EN)
                    CapSense_SsCSDDisableMode();
                #endif /* (CapSense_ENABLE == CapSense_CSD_EN) */
            }
            else if (CapSense_SENSE_METHOD_CSX_E ==  CapSense_currentSenseMethod)
            {
                #if (CapSense_ENABLE == CapSense_CSX_EN)
                    CapSense_SsDisableCSXMode();
                #endif /* (CapSense_ENABLE == CapSense_CSX_EN) */
            }
            else
            {
                #if (CapSense_ENABLE == CapSense_ADC_EN)
                    /* Release ADC resources */
                    (void)CapSense_AdcReleaseResources();
                #endif /* (CapSense_ENABLE == CapSense_ADC_EN) */
            }

            if (CapSense_SENSE_METHOD_CSD_E == mode)
            {
                #if (CapSense_ENABLE == CapSense_CSD_EN)
                    /* Initialize CSD mode to guarantee configured CSD mode */
                    CapSense_SsCSDInitialize();
                    CapSense_currentSenseMethod = CapSense_SENSE_METHOD_CSD_E;
                #endif /* (CapSense_ENABLE == CapSense_CSD_EN) */
            }
            else if (CapSense_SENSE_METHOD_CSX_E == mode)
            {
                #if (CapSense_ENABLE == CapSense_CSX_EN)
                    /* Initialize CSX mode to guarantee configured CSX mode */
                    CapSense_CSXInitialize();
                    CapSense_currentSenseMethod = CapSense_SENSE_METHOD_CSX_E;
                #endif /* (CapSense_ENABLE == CapSense_CSX_EN) */
            }
            else
            {
                CapSense_currentSenseMethod = CapSense_UNDEFINED_E;
            }
        }
    }
#endif  /* ((CapSense_ENABLE == CapSense_CSD_CSX_EN) || \
            (CapSense_ENABLE == CapSense_ADC_EN)) */

#if (CapSense_ENABLE == CapSense_CSX_EN)
    /*******************************************************************************
    * Function Name: CapSense_SsSetIOsInDefaultState
    ****************************************************************************//**
    *
    * \brief
    *   Sets all electrodes into a default state.
    *
    * \details
    *   Sets all IOs into a default state:
    *   - HSIOM   - Disconnected, the GPIO mode.
    *   - DM      - Strong drive.
    *   - State   - Zero.
    *
    *   It is not recommended to call this function directly from the application
    *   layer.
    *
    *******************************************************************************/
    void CapSense_SsSetIOsInDefaultState(void)
    {
        CapSense_FLASH_IO_STRUCT const *ioPtr = &CapSense_ioList[0u];
        uint32 loopIndex;
        uint32 regValue;

        /*  Loop through all electrodes */
        for (loopIndex = 0u; loopIndex < CapSense_TOTAL_ELECTRODES; loopIndex++)
        {
            /*  1. Disconnect HSIOM
                2. Set strong DM
                3. Set pin state to logic 0     */
            regValue = CY_GET_REG32 (ioPtr->hsiomPtr);
            regValue &= ~(ioPtr->hsiomMask);
            CY_SET_REG32 (ioPtr->hsiomPtr, regValue);

            regValue = CY_GET_REG32 (ioPtr->pcPtr);
            regValue &= ~(CapSense_GPIO_PC_MASK << ioPtr->shift);
            regValue |=  (CapSense_GPIO_STRGDRV << ioPtr->shift);
            CY_SET_REG32 (ioPtr->pcPtr, regValue);

            regValue = CY_GET_REG32 (ioPtr->drPtr);
            regValue &= ~(ioPtr->mask);
            CY_SET_REG32 (ioPtr->drPtr, regValue);

            /*  Get next electrode  */
            ioPtr++;
        }
    }
#endif  /* (CapSense_ENABLE == CapSense_CSX_EN) */


#if (CapSense_ENABLE == CapSense_ADC_EN)
/*******************************************************************************
* Function Name: CapSense_SsReleaseResources()
****************************************************************************//**
*
* \brief
*  This function sets the resources that do not belong to the CSDv2 HW block to
*  default state.
*
* \details
*  The function performs following tasks:
*  1. Checks if CSD block busy and returns error if it is busy
*  2. Disconnects integration capacitors (CintA, CintB for CSX mode and
*     Cmod for CSD mode)
*  3. Disconnect electroded if they have been connected.
*
* \return
*   Returns the status of operation:
*   - Zero        - Resources released successfully.
*   - Non-zero    - One or more errors occurred in releasing process.
*
*******************************************************************************/
cystatus CapSense_SsReleaseResources(void)
{
    cystatus busyStatus = CYRET_SUCCESS;

    if (CapSense_SW_STS_BUSY == (CapSense_dsRam.status & CapSense_SW_STS_BUSY))
    {
        /* Previous widget is being scanned. Return error. */
        busyStatus = CYRET_INVALID_STATE;
    }
    else
    {
        #if (CapSense_ENABLE == CapSense_CSX_EN)
            CapSense_SsDisableCSXMode();
        #endif /* (CapSense_ENABLE == CapSense_CSX_EN) */

        #if (CapSense_ENABLE == CapSense_CSD_EN)
            CapSense_SsCSDDisableMode();
        #endif /* (CapSense_ENABLE == CapSense_CSD_EN) */

        /*
         * Reset of the currentSenseMethod variable to make sure that the next
         * call of SetupWidget() API setups the correct widget mode
         */
        CapSense_currentSenseMethod = CapSense_UNDEFINED_E;
    }

    return busyStatus;
}
#endif /* (CapSense_ENABLE == CapSense_ADC_EN) */


/*******************************************************************************
* Function Name: CapSense_SsPostAllWidgetsScan
****************************************************************************//**
*
* \brief
*   The ISR function for multiple widget scanning implementation.
*
* \details
*   This is the function used by the CapSense ISR to implement multiple widget
*   scanning.
*   Should not be used by the application layer.
*
*******************************************************************************/
void CapSense_SsPostAllWidgetsScan(void)
{
    #if (CapSense_DISABLE == CapSense_CSD2X_EN)
        /*
        *   1. Increment widget index
        *   2. Check if all widgets are scanned
        *   3. If all widgets are not scanned, set up and scan next widget
        */
        #if (1u != CapSense_TOTAL_WIDGETS)
            cystatus postScanStatus;

            do
            {
                CapSense_widgetIndex++;

                postScanStatus = CapSense_SetupWidget((uint32)CapSense_widgetIndex);

                if (CYRET_SUCCESS == postScanStatus)
                {
                    if((CapSense_TOTAL_WIDGETS - 1u) == CapSense_widgetIndex)
                    {
                        /* The last widget will be scanned. Reset flag to skip configuring the next widget setup in ISR. */
                        CapSense_requestScanAllWidget = CapSense_DISABLE;
                    }
                    (void)CapSense_Scan();
                }
                else if((CapSense_TOTAL_WIDGETS - 1u) == CapSense_widgetIndex)
                {
                    #if ((CapSense_ENABLE == CapSense_BLOCK_OFF_AFTER_SCAN_EN) && \
                         (CapSense_ENABLE == CapSense_CSD_EN))
                        if (CapSense_SENSE_METHOD_CSD_E ==
                             (CapSense_SENSE_METHOD_ENUM)CapSense_GET_SENSE_METHOD(&CapSense_dsFlash.wdgtArray[CapSense_widgetIndex]))
                        {
                            /*  Disable the CSD block */
                            CY_SET_REG32(CapSense_CONFIG_PTR, CapSense_configCsd);
                        }
                    #endif /* ((CapSense_ENABLE == CapSense_BLOCK_OFF_AFTER_SCAN_EN) && \
                               (CapSense_ENABLE == CapSense_CSD_EN)) */

                    /* All widgets are totally processed. Reset BUSY flag */
                    CapSense_dsRam.status &= ~CapSense_SW_STS_BUSY;

                    /* Update status with with the failure */
                    CapSense_dsRam.status &= ~CapSense_STATUS_ERR_MASK;
                    CapSense_dsRam.status |= ((postScanStatus & CapSense_STATUS_ERR_SIZE) << CapSense_STATUS_ERR_SHIFT);

                    /* Set postScanStatus to exit the while loop */
                    postScanStatus = CYRET_SUCCESS;
                }
                else
                {
                    /* Update status with with the failure. Configure the next widget in while() loop */
                    CapSense_dsRam.status &= ~CapSense_STATUS_ERR_MASK;
                    CapSense_dsRam.status |= ((postScanStatus & CapSense_STATUS_ERR_SIZE) << CapSense_STATUS_ERR_SHIFT);
                }
            } while (CYRET_SUCCESS != postScanStatus);
        #endif /* (1u != CapSense_TOTAL_WIDGETS) */
    #else
        /******************************
        * This is the place where the CSD2X
        * postscan for 2 widgets should be implemented.
        * The CSD2X postscan will be implemented
        * in the next component version.
        *********************************/
    #endif /* (CapSense_CSD2X_EN == CapSense_DISABLE) */
}


/*******************************************************************************
* Function Name: CapSense_SsIsrInitialize
****************************************************************************//**
*
* \brief
*   Enables and initializes for the function pointer for a callback for the ISR.
*
* \details
*   The  "address" is a special type cyisraddress defined by Cylib. This function
*   is used by component APIs and should not be used by an application program for
*   proper working of the component.
*
* \param  address The address of the function to be called when interrupt is fired.
*
*******************************************************************************/
void CapSense_SsIsrInitialize(cyisraddress address)
{
#if (CapSense_ENABLE == CapSense_CSD2X_EN)
    /******************************
    * This is the place where the CSD2X ISR
    * Setup should be implemented.
    * The CSD2X ISR Setup will be implemented
    * in the next component version.
    *********************************/
#else
    CapSense_ISR_StartEx(address);
#endif /* (CapSense_ENABLE == CapSense_CSD2X_EN) */
}


/*******************************************************************************
* Function Name: CapSense_SsSetModClkClockDivider
****************************************************************************//**
*
* \brief
*   Sets the divider values for the modulator clock and then starts
*   the modulator clock.
*
* \details
*   It is not recommended to call this function directly by the application layer.
*   It is used by initialization, widget APIs or wakeup functions to
*   enable the clocks.
*
* \param
*   modClk The divider value for the modulator clock.
*
*******************************************************************************/
void CapSense_SsSetModClkClockDivider(uint32 modClk)
{
#if (CapSense_ENABLE == CapSense_IS_M0S8PERI_BLOCK)
    /*  Stop modulator clock   */
    CY_SET_REG32(CapSense_MODCLK_CMD_PTR,
                 ((uint32)CapSense_ModClk__DIV_ID <<
                 CapSense_MODCLK_CMD_DIV_SHIFT)|
                 ((uint32)CapSense_MODCLK_CMD_DISABLE_MASK));

    /*
     * Set divider value for sense modClk.
     * 1u is subtracted from modClk because Divider register value 0 corresponds
     * to dividing by 1.
     */
    CY_SET_REG32(CapSense_MODCLK_DIV_PTR, ((modClk - 1u) << 8u));

    /*  Check whether previous modulator clock start command has finished. */
    while(0u != (CY_GET_REG32(CapSense_MODCLK_CMD_PTR) & CapSense_MODCLK_CMD_ENABLE_MASK))
    {
        /*  Wait until previous modulator clock start command has finished. */
    }

    /*  Start modulator clock, aligned to HFCLK */
    CY_SET_REG32(CapSense_MODCLK_CMD_PTR,
                 (uint32)(((uint32)CapSense_ModClk__DIV_ID << CapSense_MODCLK_CMD_DIV_SHIFT) |
                  ((uint32)CapSense_ModClk__PA_DIV_ID << CapSense_MODCLK_CMD_PA_DIV_SHIFT) |
                  CapSense_MODCLK_CMD_ENABLE_MASK));
#else
    uint32 newRegValue;

    /* Clear bit to disable ModClk clock. */
    CY_SET_REG32(CapSense_MODCLK_CMD_PTR,
                 CY_GET_REG32(CapSense_MODCLK_CMD_PTR) &
                 (uint32)(~CapSense_ModClk__ENABLE_MASK));

    /*
     * Update ModClk clock divider.
     * 1u is subtracted from modClk because Divider register value has range (0-65535).
     */
    newRegValue = CY_GET_REG32(CapSense_MODCLK_DIV_PTR) & (uint32)(~(uint32)CapSense_ModClk__DIVIDER_MASK);
    newRegValue |= (modClk - 1u);
    CY_SET_REG32(CapSense_MODCLK_DIV_PTR, newRegValue);

    /* Set bit to enable ModClk clock. */
    CY_SET_REG32(CapSense_MODCLK_CMD_PTR,
                 CY_GET_REG32(CapSense_MODCLK_CMD_PTR) |
                CapSense_ModClk__ENABLE_MASK);
#endif /* (CapSense_ENABLE == CapSense_IS_M0S8PERI_BLOCK) */
}


/*******************************************************************************
* Function Name: CapSense_SsSetSnsClockDivider
****************************************************************************//**
*
* \brief
*   Sets the divider values for the sense clock and then starts
*   the sense clock.
*
* \details
*   It is not recommended to call this function directly by the application layer.
*   It is used by initialization, widget APIs or wakeup functions to
*   enable the clocks.
*
* \param
*   snsClk The divider value for the sense clock.
*
*******************************************************************************/
void CapSense_SsSetSnsClockDivider(uint32 snsClk)
{
#if (CapSense_ENABLE == CapSense_CSDV2)
    uint32 newRegValue;

    /*
     * Set divider value for sense clock.
     * 1u is subtracted from snsClk because SENSE_DIV value 0 corresponds
     * to dividing by 1.
     */
    newRegValue = CY_GET_REG32(CapSense_SENSE_PERIOD_PTR);
    newRegValue &= (uint32)(~CapSense_SENSE_PERIOD_SENSE_DIV_MASK);
    newRegValue |= snsClk - 1u;
    CY_SET_REG32(CapSense_SENSE_PERIOD_PTR, newRegValue);
#else
    #if (CapSense_ENABLE == CapSense_IS_M0S8PERI_BLOCK)
        /*  Stop sense clock clock   */
        CY_SET_REG32(CapSense_SNSCLK_CMD_PTR,
                     ((uint32)CapSense_SnsClk__DIV_ID <<
                     CapSense_SNSCLK_CMD_DIV_SHIFT)|
                     ((uint32)CapSense_SNSCLK_CMD_DISABLE_MASK));

        /*
         * Set divider value for sense clock.
         * 1u is subtracted from snsClk because Divider register value 0 corresponds
         * to dividing by 1.
         */
        CY_SET_REG32(CapSense_SNSCLK_DIV_PTR, ((snsClk - 1u) << 8u));

        /*  Check whether previous sense clock start command has finished. */
        while(0u != (CY_GET_REG32(CapSense_SNSCLK_CMD_PTR) & CapSense_SNSCLK_CMD_ENABLE_MASK))
        {
            /*  Wait until previous sense clock start command has finished. */
        }

        /* Start sense clock aligned to previously started modulator clock. */
        CY_SET_REG32(CapSense_SNSCLK_CMD_PTR,
                     (uint32)(((uint32)CapSense_SnsClk__DIV_ID << CapSense_SNSCLK_CMD_DIV_SHIFT) |
                      ((uint32)CapSense_ModClk__PA_DIV_ID << CapSense_SNSCLK_CMD_PA_DIV_SHIFT) |
                      CapSense_SNSCLK_CMD_ENABLE_MASK));
    #else
        uint32 newRegValue;

        /* Clear bit to disable SnsClk clock. */
        CY_SET_REG32(CapSense_SNSCLK_CMD_PTR,
                     CY_GET_REG32(CapSense_SNSCLK_CMD_PTR) &
                     (uint32)(~(uint32)CapSense_SnsClk__ENABLE_MASK));

        /*
         * Update snsClk clock divider.
         * 1u is subtracted from snsClk because Divider register value has range (0-65535).
         */
        newRegValue = CY_GET_REG32(CapSense_SNSCLK_DIV_PTR) & (uint32)(~(uint32)CapSense_SnsClk__DIVIDER_MASK);
        newRegValue |= (snsClk - 1u);
        CY_SET_REG32(CapSense_SNSCLK_DIV_PTR, newRegValue);

        /* Set bit to enable SnsClk clock. */
        CY_SET_REG32(CapSense_SNSCLK_CMD_PTR,
                     CY_GET_REG32(CapSense_SNSCLK_CMD_PTR) |
                    CapSense_SnsClk__ENABLE_MASK);
    #endif /* (CapSense_ENABLE == CapSense_IS_M0S8PERI_BLOCK) */
#endif /* (CapSense_ENABLE == CapSense_CSDV2) */
}


/*******************************************************************************
* Function Name: CapSense_SsSetClockDividers
****************************************************************************//**
*
* \brief
*   Sets the divider values for sense and modulator clocks and then starts
*   a modulator clock-phase aligned to HFCLK and sense clock-phase aligned to
*   the modulator clock.
*
* \details
*   It is not recommended to call this function directly by the application layer.
*   It is used by initialization, widget APIs or wakeup functions to
*   enable the clocks.
*
* \param
*   snsClk The divider value for the sense clock.
*   modClk The divider value for the modulator clock.
*
*******************************************************************************/
void CapSense_SsSetClockDividers(uint32 snsClk, uint32 modClk)
{
    /* Configure Mod clock */
    CapSense_SsSetModClkClockDivider(modClk);

    /* Configure Sns clock */
    CapSense_SsSetSnsClockDivider(snsClk);
}


#if ((CapSense_ENABLE == CapSense_CSD_IDAC_AUTOCAL_EN) || \
     (CapSense_ENABLE == CapSense_CSX_IDAC_AUTOCAL_EN))
    /*******************************************************************************
    * Function Name: CapSense_CalibrateWidget
    ****************************************************************************//**
    *
    * \brief
    *  Calibrates the IDACs for all sensors in the specified widget to default
    *  target (85%) value, this function detects sensing method used by the
    *  widget prior to calibration.
    *
    * \details
    *  This function performs exactly the same tasks as
    *  CapSense_CalibrateAllWidgets, but only for a specified widget.
    *  This function detects the sensing method used by the widgets and takes
    *  into account the Enable compensation IDAC parameter.
    *
    *  This function is available when the CSD and/or CSX Enable IDAC
    *  auto-calibration parameter is enabled.

    *
    * \param wdgtIndex
    *  Specify the ID number of the widget to calibrate its raw count.
    *  A macro for the widget ID can be found in the
    *  CapSense_Configuration.h file defined as
    *  CapSense_<WidgetName>_WDGT_ID.
    *
    * \return
    *  Returns the status of the specified widget calibration:
    *    - CYRET_SUCCESS if operation is successfully completed.
    *    - CYRET_BAD_PARAM if the input parameter is invalid.
    *    - CYRET_BAD_DATA if the calibration failed and the component may not
    *      operate as expected.
    *
    *******************************************************************************/
    cystatus CapSense_CalibrateWidget(uint32 wdgtIndex)
    {
        cystatus calibrateStatus = CYRET_SUCCESS;

        do
        {
            if (CapSense_TOTAL_WIDGETS <= wdgtIndex)
            {
                calibrateStatus = CYRET_BAD_PARAM;
                break;
            }

            /*
             *  Check if widget id is valid, specified widget did not
             *  detect any faults conditions if BIST is enabled.
             */
            #if (CapSense_ENABLE == CapSense_SELF_TEST_EN)
                if (0u != CapSense_GET_WIDGET_EN_STATUS(wdgtIndex))
                {
                    calibrateStatus = CYRET_SUCCESS;
                }
                else
                {
                    calibrateStatus = CYRET_INVALID_STATE;
                    /*  Exit because widget is not working  */
                    break;
                }
            #endif  /* (CapSense_ENABLE == CapSense_SELF_TEST_EN) */

            /* If CSD2X is enabled, calibrate CSD sensor    */
            #if (CapSense_ENABLE == CapSense_CSD2X_EN)
                /******************************
                * This is the place where the CSD2X mode
                * API should be implemented.
                * CSD2X will be implemented
                * in the next component version.
                *********************************/

            /* If both CSD and CSX are enabled, calibrate widget using sensing method */
            #elif (CapSense_ENABLE == CapSense_CSD_CSX_EN)

                /* Check widget sensing method and call appropriate APIs */
                #if (CapSense_ENABLE == CapSense_CSX_IDAC_AUTOCAL_EN)
                    if (CapSense_SENSE_METHOD_CSX_E ==
                        (CapSense_SENSE_METHOD_ENUM)CapSense_GET_SENSE_METHOD(&CapSense_dsFlash.wdgtArray[wdgtIndex]))
                    {
                        /* Calibrate CSX widget  */
                       CapSense_CSXCalibrateWidget(wdgtIndex, CapSense_CSX_RAWCOUNT_CAL_LEVEL);
                    }
                #endif  /* (CapSense_ENABLE == CapSense_CSX_IDAC_AUTOCAL_EN) */

                #if (CapSense_ENABLE == CapSense_CSD_IDAC_AUTOCAL_EN)
                    if (CapSense_SENSE_METHOD_CSD_E ==
                        (CapSense_SENSE_METHOD_ENUM)CapSense_GET_SENSE_METHOD(&CapSense_dsFlash.wdgtArray[wdgtIndex]))
                    {
                        /* Calibrate CSD widget */
                        calibrateStatus = CapSense_CSDCalibrateWidget(wdgtIndex, CapSense_CSD_RAWCOUNT_CAL_LEVEL);
                    }
                #endif  /* (CapSense_ENABLE == CapSense_CSD_IDAC_AUTOCAL_EN) */

            /*  If only CSD is enabled, calibrate CSD sensor  */
            #elif (CapSense_ENABLE == CapSense_CSD_EN)
                calibrateStatus = CapSense_CSDCalibrateWidget(wdgtIndex, CapSense_CSD_RAWCOUNT_CAL_LEVEL);

            /*  If only CSX is enabled, call CSX scan   */
            #elif (CapSense_ENABLE == CapSense_CSX_EN)
                CapSense_CSXCalibrateWidget(wdgtIndex, CapSense_CSX_RAWCOUNT_CAL_LEVEL);

            #else
                calibrateStatus = CYRET_UNKNOWN;
            #endif  /* (CapSense_ENABLE == CapSense_CSD2X_EN) */

        } while (0u);

        return calibrateStatus;
    }


    /*******************************************************************************
    * Function Name: CapSense_CalibrateAllWidgets
    ****************************************************************************//**
    *
    * \brief
    *  Calibrates the IDACs for all widgets in the component to default target
    *  (85%) value, this function detects sensing method used by the widgets
    *  prior to calibration.
    *
    * \details
    *  Calibrates the IDACs for all widgets in the component to the default
    *  target (85% of maximum possible raw count) value. This function detects
    *  the sensing method used by the widgets and takes into account the Enable
    *  compensation IDAC parameter.
    *
    *  This function is available when the CSD and/or CSX Enable IDAC
    *  auto-calibration parameter is enabled.
    *
    * \return
    *  Returns the status of the calibration process:
    *    - CYRET_SUCCESS if the operation is successfully completed.
    *    - CYRET_BAD_DATA if the calibration failed and the component may not
    *      operate as expected.
    *
    *******************************************************************************/
    cystatus CapSense_CalibrateAllWidgets(void)
    {
        cystatus calibrateStatus = CYRET_SUCCESS;
        uint32 wdgtIndex;

        for (wdgtIndex = 0u; wdgtIndex < CapSense_TOTAL_WIDGETS; wdgtIndex++)
        {
            calibrateStatus |= CapSense_CalibrateWidget(wdgtIndex);
        }

        return calibrateStatus;
    }
#endif /* ((CapSense_ENABLE == CapSense_CSD_IDAC_AUTOCAL_EN) ||
           (CapSense_ENABLE == CapSense_CSX_IDAC_AUTOCAL_EN)) */


#if (CapSense_CSD_SS_DIS != CapSense_CSD_AUTOTUNE)
    /*******************************************************************************
    * Function Name: CapSense_SsSetDirectClockMode
    ****************************************************************************//**
    *
    * \brief
    *  Sets Direct Clock Mode.
    *
    * \details
    *  For CSDv1: Resets PRS bit in CapSense_configCsd variable;
    *  For CSDv2: Resets CapSense_SENSE_PERIOD_SEL_LFSR_MSB_MASK and
    *  CapSense_SENSE_PERIOD_SEL_LFSR_MSB_MASK bits in
    *  CapSense_SENSE_PERIOD register.
    *
    *******************************************************************************/
    static void CapSense_SsSetDirectClockMode(void)
    {
        #if (CapSense_ENABLE == CapSense_CSDV2)
            CY_SET_REG32(CapSense_SENSE_PERIOD_PTR, CY_GET_REG32(CapSense_SENSE_PERIOD_PTR) &
                                                                 (uint32)~(CapSense_SENSE_PERIOD_SEL_LFSR_MSB_MASK | \
                                                                 CapSense_SENSE_PERIOD_LFSR_SIZE_MASK));
        #else
            CapSense_configCsd = CapSense_configCsd &(uint32)~(CapSense_CONFIG_PRS_SELECT_MASK);
        #endif /* (CapSense_ENABLE == CapSense_CSDV2) */
    }


    /*******************************************************************************
    * Function Name: CapSense_SsAutoTune
    ****************************************************************************//**
    *
    * \brief
    *  This API performs the parameters auto-tuning for the optimal CapSense operation.
    *
    * \details
    *  This API performs the following:
    *  - Calibrates Modulator and Compensation IDACs.
    *  - Tunes the Sense Clock optimal value to get a Sense Clock period greater than
    *     2*5*R*Cp.
    *  - Calculates the resolution for the optimal finger capacitance.
    *
    * \return
    *   Returns the status of operation:
    *   - Zero     - All the widgets are auto-tuned successfully.
    *   - Non-zero - Auto-tuning failed for any widget.
    *
    *******************************************************************************/
    cystatus CapSense_SsAutoTune(void)
    {
        cystatus autoTuneStatus = CYRET_SUCCESS;
        uint32 wdgtIndex;
        uint32 cp;
        #if (CapSense_CSD_MATRIX_WIDGET_EN | CapSense_CSD_TOUCHPAD_WIDGET_EN)
            uint32 cpRow = 0uL;
        #endif /* (CapSense_CSD_MATRIX_WIDGET_EN | CapSense_CSD_TOUCHPAD_WIDGET_EN) */
        CapSense_RAM_WD_BASE_STRUCT *ptrWdgt;
        AUTO_TUNE_CONFIG_TYPE autoTuneConfig;

        /* Configure common config variables */
        autoTuneConfig.snsClkConstantR = CapSense_CSD_SNSCLK_R_CONST;
        autoTuneConfig.vRef = CapSense_CSD_VREF_MV;
        autoTuneConfig.iDacGain = CapSense_CSD_IDAC_GAIN_VALUE_NA * CapSense_CSD_DUAL_IDAC_FACTOR;
        autoTuneConfig.calTarget = CapSense_CSD_AUTOTUNE_CAL_LEVEL;

        /* Calculate snsClk Input Clock in KHz */
        #if (CapSense_ENABLE == CapSense_CSDV2)
            /*  Dividers are chained (CSDV2). Flip flop is not available */
            autoTuneConfig.snsClkInputClock = (CYDEV_BCLK__HFCLK__KHZ / CapSense_dsRam.modCsdClk);
        #elif ((CapSense_ENABLE == CapSense_IS_M0S8PERI_BLOCK) && (CapSense_DISABLE == CY_PSOC4_4000))
            /*  Dividers are not chained */
            autoTuneConfig.snsClkInputClock = CYDEV_BCLK__HFCLK__KHZ >> CapSense_FLIP_FLOP_DIV;
        #elif (CapSense_ENABLE == CapSense_IS_M0S8PERI_BLOCK)
            /*  Dividers are not chained (PSoC 4000) */
            autoTuneConfig.snsClkInputClock = CYDEV_BCLK__HFCLK__KHZ >> CapSense_FLIP_FLOP_DIV;
        #else
            /*  Dividers are chained (PSoC 4100, PSoC 4200) */
            autoTuneConfig.snsClkInputClock = (CYDEV_BCLK__HFCLK__KHZ / CapSense_dsRam.modCsdClk) >> CapSense_FLIP_FLOP_DIV;
        #endif /* (CapSense_ENABLE == CapSense_CSDV2) */

        /* If both CSD and CSX are enabled, calibrate widget using sensing method */
        #if (CapSense_ENABLE == CapSense_CSD_CSX_EN)
            /* Initialize CSD mode */
            CapSense_SsCSDInitialize();
        #endif /* (CapSense_ENABLE == CapSense_CSD_CSX_EN) */

        for (wdgtIndex = 0u; wdgtIndex < CapSense_TOTAL_WIDGETS; wdgtIndex++)
        {
            if (CapSense_SENSE_METHOD_CSD_E ==
                (CapSense_SENSE_METHOD_ENUM)CapSense_GET_SENSE_METHOD(&CapSense_dsFlash.wdgtArray[wdgtIndex]))
            {
                ptrWdgt = (CapSense_RAM_WD_BASE_STRUCT *)
                          CapSense_dsFlash.wdgtArray[wdgtIndex].ptr2WdgtRam;

                /* Switch charge clock source to direct clock mode */
                CapSense_SsSetDirectClockMode();

                /* Get finger capacitance */
                autoTuneConfig.fingerCap = ptrWdgt->fingerCap;

                /* Init pointer to sigPFC */
                autoTuneConfig.sigPFC = &ptrWdgt->sigPFC;

                /* Set calibration resolution to 12 bits */
                ptrWdgt->resolution = CapSense_CALIBRATION_RESOLUTION;

                /* Set Sense clock frequency to 1.5 MHz */
                #if (CapSense_ENABLE == CapSense_CSD_COMMON_SNS_CLK_EN)
                    CapSense_dsRam.snsCsdClk = (uint8)((uint32)autoTuneConfig.snsClkInputClock /
                                                       CapSense_CALIBRATION_FREQ_KHZ);
                #elif (CapSense_CSD_MATRIX_WIDGET_EN | CapSense_CSD_TOUCHPAD_WIDGET_EN)
                    ptrWdgt->rowSnsClk = (uint8)((uint32)autoTuneConfig.snsClkInputClock /
                                         CapSense_CALIBRATION_FREQ_KHZ);
                    ptrWdgt->snsClk = (uint8)((uint32)autoTuneConfig.snsClkInputClock /
                                      CapSense_CALIBRATION_FREQ_KHZ);
                #else
                    ptrWdgt->snsClk = (uint8)((uint32)autoTuneConfig.snsClkInputClock /
                                      CapSense_CALIBRATION_FREQ_KHZ);
                #endif /*  (CapSense_ENABLE == CapSense_CSD_COMMON_SNS_CLK_EN) */

                /* Set flag to calibrate in PWM mode only */
                CapSense_prescalersTuningDone = CapSense_DISABLE;

                #if (CapSense_DISABLE == CapSense_CSD2X_EN)
                    /* Calibrate CSD widget to 85% */
                    (void)CapSense_CSDCalibrateWidget(wdgtIndex, CapSense_CSD_AUTOTUNE_CAL_LEVEL);
                #endif /* (CapSense_DISABLE == CapSense_CSD2X_EN) */

                #if (CapSense_CSD_MATRIX_WIDGET_EN | CapSense_CSD_TOUCHPAD_WIDGET_EN)
                    if ((CapSense_WD_TOUCHPAD_E == (CapSense_WD_TYPE_ENUM)CapSense_dsFlash.wdgtArray[(wdgtIndex)].wdgtType) ||
                        (CapSense_WD_MATRIX_BUTTON_E == (CapSense_WD_TYPE_ENUM)CapSense_dsFlash.wdgtArray[(wdgtIndex)].wdgtType))
                    {
                        /* Get pointer to Modulator IDAC for columns */
                        autoTuneConfig.ptrModIDAC = &ptrWdgt->rowIdacMod[0u];

                        /* Get pointer to Sense Clock Divider for columns */
                        autoTuneConfig.ptrSenseClk = &ptrWdgt->rowSnsClk;

                        /* Find correct sense clock value */
                        cpRow = SmartSense_TunePrescalers(&autoTuneConfig);

                        if ((CapSense_CP_MAX + CapSense_CP_ERROR) <= cpRow)
                        {
                            autoTuneStatus = CYRET_BAD_DATA;
                        }

                        #if (CapSense_ENABLE == CapSense_CSDV2)
                            /* Make sure that ModClk >= 4 * rowSnsClk for ModClk <= 12 MHz and rowSnsClk <= 6MHz */
                            if (autoTuneConfig.snsClkInputClock <= CapSense_MOD_CSD_CLK_24000KHZ)
                            {
                                if (ptrWdgt->rowSnsClk < CapSense_FACTOR_FILTER_DELAY_12MHZ)
                                {
                                    ptrWdgt->rowSnsClk = CapSense_FACTOR_FILTER_DELAY_12MHZ;
                                }
                            }
                            else if (autoTuneConfig.snsClkInputClock <= CapSense_MOD_CSD_CLK_48000KHZ)
                            {
                                if (ptrWdgt->rowSnsClk < CapSense_FACTOR_MOD_SNS)
                                {
                                    ptrWdgt->rowSnsClk = CapSense_FACTOR_MOD_SNS;
                                }
                            }
                            else
                            {
                                /* rowSnsClk is valid*/
                            }
                        #else
                            #if (CapSense_ENABLE == CapSense_IS_M0S8PERI_BLOCK)
                                /* Make sure that ModClk >= 2 * rowSnsClk */
                                if (CapSense_dsRam.modCsdClk >= ((uint8)(ptrWdgt->rowSnsClk << CapSense_FLIP_FLOP_DIV)))
                                {
                                     ptrWdgt->rowSnsClk = CapSense_dsRam.modCsdClk;
                                }
                            #else
                                /* Sense clock never equals to Modulator clock for chained dividers because of Flip Flop */
                            #endif /* (CapSense_ENABLE == CapSense_CSDV2) */
                        #endif /* (CapSense_ENABLE == CapSense_CSDV2) */
                    }
                #endif /* (CapSense_CSD_MATRIX_WIDGET_EN | CapSense_CSD_TOUCHPAD_WIDGET_EN) */

                /* Get pointer to Modulator IDAC  for rows */
                autoTuneConfig.ptrModIDAC = &ptrWdgt->idacMod[0u];

                /* Get pointer to Sense Clock Divider for columns */
                autoTuneConfig.ptrSenseClk = &ptrWdgt->snsClk;

                /* Find correct sense clock value */
                cp = SmartSense_TunePrescalers(&autoTuneConfig);

                if ((CapSense_CP_MAX + CapSense_CP_ERROR) <= cp)
                {
                    autoTuneStatus = CYRET_BAD_DATA;
                }

                #if (CapSense_ENABLE == CapSense_CSDV2)
                    /* Make sure that ModClk >= 4 * SnsClk for ModClk <= 12 MHz and SnsClk <= 6MHz */
                    if (autoTuneConfig.snsClkInputClock <= CapSense_MOD_CSD_CLK_24000KHZ)
                    {
                        if (ptrWdgt->snsClk < CapSense_FACTOR_FILTER_DELAY_12MHZ)
                        {
                            ptrWdgt->snsClk = CapSense_FACTOR_FILTER_DELAY_12MHZ;
                        }
                    }
                    else if (autoTuneConfig.snsClkInputClock <= CapSense_MOD_CSD_CLK_48000KHZ)
                    {
                        if (ptrWdgt->snsClk < CapSense_FACTOR_MOD_SNS)
                        {
                            ptrWdgt->snsClk = CapSense_FACTOR_MOD_SNS;
                        }
                    }
                    else
                    {
                        /* SnsClk is valid*/
                    }
                #else
                    #if (CapSense_ENABLE == CapSense_IS_M0S8PERI_BLOCK)
                        /* Make sure that ModClk >= 2 * snsClk */
                        if (CapSense_dsRam.modCsdClk >= ((uint8)(ptrWdgt->snsClk << CapSense_FLIP_FLOP_DIV)))
                        {
                             ptrWdgt->snsClk = CapSense_dsRam.modCsdClk;
                        }
                    #else
                        /* Sense clock never equals to Modulator clock for chained dividers because of Flip Flop */
                    #endif /* (CapSense_ENABLE == CapSense_CSDV2) */
                #endif /* (CapSense_ENABLE == CapSense_CSDV2) */

                /* Set flag to indicate that calibration in PWM mode has been performed */
                CapSense_prescalersTuningDone = 1u;

                /* Multiply Clk divider to 2 for PRS mode to take into account average PRS frequency */
                autoTuneConfig.prsFactor = CapSense_PRS_FACTOR_DIV;

                #if (CapSense_CLK_SOURCE_PRSAUTO == CapSense_CSD_SNS_CLK_SOURCE)
                    CapSense_SsSetWidgetSenseClkSrc(wdgtIndex, ptrWdgt);
                #endif /* (CapSense_CLK_SOURCE_PRSAUTO == CapSense_CSD_SNS_CLK_SOURCE) */

                #if (CapSense_DISABLE == CapSense_CSD2X_EN)
                    /* Calibrate CSD widget to 85% */
                    autoTuneStatus |= CapSense_CSDCalibrateWidget(wdgtIndex, CapSense_CSD_AUTOTUNE_CAL_LEVEL);
                #endif /* (CapSense_DISABLE == CapSense_CSD2X_EN) */

                /* Set parasitic capacitance for columns */
                autoTuneConfig.sensorCap = cp;

                #if (CapSense_CSD_MATRIX_WIDGET_EN | CapSense_CSD_TOUCHPAD_WIDGET_EN)
                    if ((CapSense_WD_TOUCHPAD_E == (CapSense_WD_TYPE_ENUM)CapSense_dsFlash.wdgtArray[(wdgtIndex)].wdgtType) ||
                        (CapSense_WD_MATRIX_BUTTON_E == (CapSense_WD_TYPE_ENUM)CapSense_dsFlash.wdgtArray[(wdgtIndex)].wdgtType))
                    {
                        /* Set the minimum sense clock frequency to calculate the resolution */
                        if (ptrWdgt->snsClk < ptrWdgt->rowSnsClk)
                        {
                            /* Rewrite pointer to Sense Clock Divider for rows */
                            autoTuneConfig.ptrSenseClk = &ptrWdgt->rowSnsClk;

                            /* Set parasitic capacitance for rows */
                            autoTuneConfig.sensorCap = cpRow;
                        }
                    }
                #endif /* (CapSense_CSD_MATRIX_WIDGET_EN | CapSense_CSD_TOUCHPAD_WIDGET_EN) */

                /* Find resolution */
                ptrWdgt->resolution = SmartSense_TuneSensitivity(&autoTuneConfig);

                #if (CapSense_CLK_SOURCE_PRSAUTO == CapSense_CSD_SNS_CLK_SOURCE)
                    CapSense_SsSetWidgetSenseClkSrc(wdgtIndex, ptrWdgt);

                    #if (CapSense_CSD_MATRIX_WIDGET_EN | CapSense_CSD_TOUCHPAD_WIDGET_EN)
                        if (((uint8)CapSense_CLK_SOURCE_DIRECT == ptrWdgt->snsClkSource) ||
                            ((uint8)CapSense_CLK_SOURCE_DIRECT == ptrWdgt->rowSnsClkSource))
                        {
                            /* Recalibrate CSD widget to 85% becasue source is changed to direct */
                            autoTuneStatus |= CapSense_CSDCalibrateWidget(wdgtIndex, CapSense_CSD_AUTOTUNE_CAL_LEVEL);
                        }
                    #else
                        if ((uint8)CapSense_CLK_SOURCE_DIRECT == ptrWdgt->snsClkSource)
                        {
                            /* Recalibrate CSD widget to 85% becasue source is changed to direct */
                            autoTuneStatus |= CapSense_CSDCalibrateWidget(wdgtIndex, CapSense_CSD_AUTOTUNE_CAL_LEVEL);
                        }
                    #endif /* (CapSense_CSD_MATRIX_WIDGET_EN | CapSense_CSD_TOUCHPAD_WIDGET_EN) */
                #endif /* (CapSense_CLK_SOURCE_PRSAUTO == CapSense_CSD_SNS_CLK_SOURCE) */
            }
        }
        return autoTuneStatus;
    }
#endif /* (CapSense_CSD_SS_DIS != CapSense_CSD_AUTOTUNE)) */


#if (CapSense_DISABLE == CapSense_CSDV2)
    #if ((CapSense_ENABLE == CapSense_CSX_EN) || \
         (CapSense_IDAC_SINKING == CapSense_CSD_IDAC_CONFIG))
        /*******************************************************************************
        * Function Name: CapSense_SsTrimIdacsSinking
        ****************************************************************************//**
        *
        * \brief
        *  This function loads trim values from SFLASH rows to calibrate
        *  IDAC1 and IDAC2 for Sinking CSD Mode
        *
        * \details
        *  Function reads trim value from SFLASH and loads it into IDAC trim register.
        *  If Compensation IDAC is disabled the function loads trim values for IDAC1 only.
        *
        *******************************************************************************/
        static void CapSense_SsTrimIdacsSinking(void)
        {
            uint32 trimValue;

            /* iDAC1 Sinking Mode */
            trimValue = CY_GET_REG32(CapSense_TRIM2_PTR) & ~CapSense_IDAC_TRIM2_MOD_SNK_MASK;
            trimValue |= ((uint32)CY_GET_REG8(CapSense_SFLASH_TRIM2_PTR) &
                                                      CapSense_SFLASH_TRIM_IDAC_MOD_MASK) ;

            #if (CapSense_ENABLE == CapSense_CSD_IDAC_COMP_EN)
                /* iDAC2 Sinking Mode */
                trimValue &= ~CapSense_IDAC_TRIM2_COMP_SNK_MASK;
                trimValue |= ((uint32)CY_GET_REG8(CapSense_SFLASH_TRIM2_PTR) &
                                                          CapSense_SFLASH_TRIM_IDAC_COMP_MASK);
            #endif /* (CapSense_ENABLE == CapSense_CSD_IDAC_COMP_EN) */

            /* Update IDAC trim bits for gain control in current sink mode */
            CY_SET_REG32(CapSense_TRIM2_PTR, trimValue);
        }
    #endif /* ((CapSense_ENABLE == CapSense_CSX_EN) || \
               (CapSense_IDAC_SINKING == CapSense_CSD_IDAC_CONFIG)) */


    #if ((CapSense_ENABLE == CapSense_CSX_EN) || \
         (CapSense_IDAC_SOURCING == CapSense_CSD_IDAC_CONFIG))
        /*******************************************************************************
        * Function Name: CapSense_SsTrimIdacsSourcing
        ****************************************************************************//**
        *
        * \brief
        *  This function loads trim values from SFLASH rows to calibrate
        *  IDAC1 and IDAC2 for Sourcing CSD Mode
        *
        * \details
        *  Function reads trim value from SFLASH and loads it into IDAC trim register.
        *  If Compensation IDAC is disabled the function loads trim values for IDAC1 only.
        *
        *******************************************************************************/
        static void CapSense_SsTrimIdacsSourcing(void)
        {
            uint32 trimValue;

            /* iDAC1 Sourcing Mode */
            trimValue = CY_GET_REG32(CapSense_TRIM1_PTR) & ~CapSense_IDAC_TRIM1_MOD_SRC_MASK;
            trimValue |= ((uint32)CY_GET_REG8(CapSense_SFLASH_TRIM1_PTR) &
                                                      CapSense_SFLASH_TRIM_IDAC_MOD_MASK);

            #if (CapSense_ENABLE == CapSense_CSD_IDAC_COMP_EN)
                 /* iDAC2 Sourcing Mode */
                trimValue &= ~CapSense_IDAC_TRIM1_COMP_SRC_MASK;
                trimValue |= ((uint32)CY_GET_REG8(CapSense_SFLASH_TRIM1_PTR) &
                                                          CapSense_SFLASH_TRIM_IDAC_COMP_MASK);
            #endif /* (CapSense_ENABLE == CapSense_CSD_IDAC_COMP_EN) */

            /* Update IDAC trim bits for gain control in current source mode */
            CY_SET_REG32(CapSense_TRIM1_PTR, trimValue);
        }
    #endif /* ((CapSense_ENABLE == CapSense_CSX_EN) || \
               (CapSense_IDAC_SOURCING == CapSense_CSD_IDAC_CONFIG))  */

    /*******************************************************************************
    * Function Name: CapSense_SsTrimIdacs
    ****************************************************************************//**
    *
    * \brief
    *  This function loads trim values from SFLASH rows to calibrate
    *  IDAC1 and IDAC2 for CSD Mode
    *
    * \details
    *  If CSX mode is enabled the function loads trim values for both sink and source
    *  mode. If CSX mode is disabled the function loads trim values for sink or
    *  source mode based on sink/source mode configuration.
    *  If Compensation IDAC is disabled the function loads trim values for IDAC1 only.
    *
    *******************************************************************************/
    static void CapSense_SsTrimIdacs(void)
    {
        #if (CapSense_ENABLE == CapSense_CSX_EN)
            CapSense_SsTrimIdacsSinking();
            CapSense_SsTrimIdacsSourcing();
        #elif (CapSense_IDAC_SINKING == CapSense_CSD_IDAC_CONFIG)
            CapSense_SsTrimIdacsSinking();
        #elif (CapSense_IDAC_SOURCING == CapSense_CSD_IDAC_CONFIG)
            CapSense_SsTrimIdacsSourcing();
        #else
            #error "Not supported Mode, component cannot work in this mode"
        #endif /* (CapSense_ENABLE == CapSense_CSX_EN) */
    }
#endif  /* (CapSense_DISABLE == CapSense_CSDV2) */


#if (CapSense_ENABLE == CapSense_MULTI_FREQ_SCAN_EN)
    /*******************************************************************************
    * Function Name: CapSense_SsChangeImoFreq
    ****************************************************************************//**
    *
    * \brief
    *  This function changes the IMO frequency.
    *
    * \details
    *   The IMO frequency can have three offsets: 0%, -5% and +5%. The frequency
    *   trims are contained in the CapSense_immunity[value] array for each
    *   frequency channel.
    *
    * \param value The frequency channel ID.
    *
    *******************************************************************************/
    void CapSense_SsChangeImoFreq(uint32 value)
    {
        CY_SET_REG32(CY_SYS_CLK_IMO_TRIM1_PTR, CapSense_immunity[value]);
    }
#endif  /* (CapSense_ENABLE == CapSense_MULTI_FREQ_SCAN_EN) */


#if((CapSense_ENABLE == CapSense_CSD_EN) ||\
    ((CapSense_ENABLE == CapSense_CSX_EN) && (CapSense_ENABLE == CapSense_CSDV2)))
/*******************************************************************************
* Function Name: CapSense_SsInitializeSourceSenseClk
****************************************************************************//**
*
* \brief
*  Sets a source for Sense Clk for all CSD widgets.
*
* \details
*  Updates snsClkSource and rowSnsClkSource with a source for the sense Clk.
*  for all CSD widgets.
*
*******************************************************************************/
void CapSense_SsInitializeSourceSenseClk(void)
{
    uint32 wdgtIndex;
    CapSense_RAM_WD_BASE_STRUCT *ptrWdgt;

    for (wdgtIndex = 0u; wdgtIndex < CapSense_TOTAL_WIDGETS; wdgtIndex++)
    {
        ptrWdgt = (CapSense_RAM_WD_BASE_STRUCT *)CapSense_dsFlash.wdgtArray[wdgtIndex].ptr2WdgtRam;
        if (CapSense_SENSE_METHOD_CSD_E ==
        (CapSense_SENSE_METHOD_ENUM)CapSense_GET_SENSE_METHOD(&CapSense_dsFlash.wdgtArray[wdgtIndex]))
        {
            #if (CapSense_ENABLE == CapSense_CSD_EN)
                CapSense_SsSetWidgetSenseClkSrc(wdgtIndex, ptrWdgt);
            #endif /* (CapSense_ENABLE == CapSense_CSD_EN) */
        }
        else
        {
            #if ((CapSense_ENABLE == CapSense_CSX_EN) && (CapSense_ENABLE == CapSense_CSDV2))
                CapSense_SsSetWidgetTxClkSrc(wdgtIndex, ptrWdgt);
            #endif /* ((CapSense_ENABLE == CapSense_CSX_EN) && (CapSense_ENABLE == CapSense_CSDV2)) */
        }
    }
}
#endif /* ((CapSense_ENABLE == CapSense_CSD_EN) ||\
           ((CapSense_ENABLE == CapSense_CSX_EN) && (CapSense_ENABLE == CapSense_CSDV2))) */


#if (CapSense_ENABLE == CapSense_CSD_EN)
    /*******************************************************************************
    * Function Name: CapSense_SsSetWidgetSenseClkSrc
    ****************************************************************************//**
    *
    * \brief
    *  Sets a source for the sense clock for a widget.
    *
    * \param wdgtIndex
    *   Specifies the ID of the widget.
    * \param ptrWdgt
    *   The pointer to the RAM_WD_BASE_STRUCT structure.
    *
    * \details
    *  Updates snsClkSource and rowSnsClkSource with a source for the sense Clk for a
    *  widget.
    *
    *******************************************************************************/
    void CapSense_SsSetWidgetSenseClkSrc(uint32 wdgtIndex, CapSense_RAM_WD_BASE_STRUCT * ptrWdgt)
    {
        uint8 lfsrSize;
        uint8 lfsrScale;

        #if(CapSense_CLK_SOURCE_PRSAUTO == CapSense_CSD_SNS_CLK_SOURCE)
            #if(CapSense_ENABLE == CapSense_CSDV2)
                uint32 conversionsNum;
                uint32 sampleClkDivider;
            #endif /* (CapSense_ENABLE == CapSense_CSDV2) */
            uint32 snsClkDivider;
        #endif /* (CapSense_CLK_SOURCE_PRSAUTO == CapSense_CSD_SNS_CLK_SOURCE) */

        #if(CapSense_CLK_SOURCE_PRSAUTO == CapSense_CSD_SNS_CLK_SOURCE)
            snsClkDivider = CapSense_SsCSDGetColSnsClkDivider(wdgtIndex);

            #if(CapSense_ENABLE == CapSense_CSDV2)
                sampleClkDivider = CapSense_dsRam.modCsdClk;
                conversionsNum = CapSense_SsCSDGetNumberOfConversions(snsClkDivider, (uint32)ptrWdgt->resolution);
                lfsrSize = CapSense_SsCalcLfsrSize(sampleClkDivider, snsClkDivider, conversionsNum);
                if (CapSense_CLK_SOURCE_DIRECT == lfsrSize)
                {
                    lfsrSize = CapSense_SsCSDCalcPrsSize(snsClkDivider, (uint32)ptrWdgt->resolution);
                }
                lfsrScale = CapSense_SsCalcLfsrScale(snsClkDivider, lfsrSize);
            #else
                lfsrSize = CapSense_SsCSDCalcPrsSize(snsClkDivider, (uint32)ptrWdgt->resolution);
                lfsrScale = 0u;
            #endif /* (CapSense_ENABLE == CapSense_CSDV2) */
        #else
            lfsrSize = (uint8)CapSense_DEFAULT_MODULATION_MODE;
            lfsrScale = 0u;
        #endif /* (CapSense_CLK_SOURCE_PRSAUTO == CapSense_CSD_SNS_CLK_SOURCE) */

        ptrWdgt->snsClkSource = lfsrSize | (uint8)(lfsrScale << CapSense_CLK_SOURCE_LFSR_SCALE_OFFSET);

        #if (CapSense_ENABLE == (CapSense_CSD_MATRIX_WIDGET_EN | CapSense_CSD_TOUCHPAD_WIDGET_EN))
            if ((CapSense_WD_TOUCHPAD_E == (CapSense_WD_TYPE_ENUM)CapSense_dsFlash.wdgtArray[wdgtIndex].wdgtType) ||
                (CapSense_WD_MATRIX_BUTTON_E == (CapSense_WD_TYPE_ENUM)CapSense_dsFlash.wdgtArray[wdgtIndex].wdgtType))
            {
                #if(CapSense_CLK_SOURCE_PRSAUTO == CapSense_CSD_SNS_CLK_SOURCE)
                    snsClkDivider = CapSense_SsCSDGetRowSnsClkDivider(wdgtIndex);

                    #if(CapSense_ENABLE == CapSense_CSDV2)
                        lfsrSize = CapSense_SsCalcLfsrSize(sampleClkDivider, snsClkDivider, conversionsNum);
                        if (CapSense_CLK_SOURCE_DIRECT == lfsrSize)
                        {
                            lfsrSize = CapSense_SsCSDCalcPrsSize(snsClkDivider, (uint32)ptrWdgt->resolution);
                        }
                        lfsrScale = CapSense_SsCalcLfsrScale(snsClkDivider, lfsrSize);
                    #else
                        lfsrSize = CapSense_SsCSDCalcPrsSize(snsClkDivider, (uint32)ptrWdgt->resolution);
                        lfsrScale = 0u;
                    #endif /* (CapSense_ENABLE == CapSense_CSDV2) */
                #else
                    lfsrSize = (uint8)CapSense_DEFAULT_MODULATION_MODE;
                    lfsrScale = 0u;
                #endif /* (CapSense_CLK_SOURCE_PRSAUTO == CapSense_CSD_SNS_CLK_SOURCE) */
                ptrWdgt->rowSnsClkSource = lfsrSize | (uint8)(lfsrScale << CapSense_CLK_SOURCE_LFSR_SCALE_OFFSET);
            }
        #endif /* (CapSense_ENABLE == (CapSense_CSD_MATRIX_WIDGET_EN | CapSense_CSD_TOUCHPAD_WIDGET_EN)) */
    }
#endif /* (CapSense_ENABLE == CapSense_CSD_EN) */


#if ((CapSense_ENABLE == CapSense_CSX_EN) && (CapSense_ENABLE == CapSense_CSDV2))
    /*******************************************************************************
    * Function Name: CapSense_SsSetWidgetTxClkSrc
    ****************************************************************************//**
    *
    * \brief
    *  Sets a source for the Tx clock for a widget.
    *
    * \param wdgtIndex
    *   Specifies the ID of the widget.
    * \param ptrWdgt
    *   The pointer to the RAM_WD_BASE_STRUCT structure.
    *
    * \details
    *  Updates snsClkSourcewith with a source for Tx Clk for a widget.                  !!
    *
    *******************************************************************************/
    void CapSense_SsSetWidgetTxClkSrc(uint32 wdgtIndex, CapSense_RAM_WD_BASE_STRUCT * ptrWdgt)
    {
        uint8 lfsrSize;
        uint8 lfsrScale;

        #if(CapSense_CLK_SOURCE_PRSAUTO == CapSense_CSX_TX_CLK_SOURCE)
            uint32 conversionsNum;
            uint32 snsClkDivider;
            uint32 sampleClkDivider;
        #endif /* (CapSense_CLK_SOURCE_PRSAUTO == CapSense_CSX_TX_CLK_SOURCE) */

        #if(CapSense_CLK_SOURCE_PRSAUTO == CapSense_CSX_TX_CLK_SOURCE)
            sampleClkDivider = CapSense_dsRam.modCsxClk;
            conversionsNum = (uint32)ptrWdgt->resolution;
            snsClkDivider = CapSense_SsCSXGetTxClkDivider(wdgtIndex);
            lfsrSize = CapSense_SsCalcLfsrSize(sampleClkDivider, snsClkDivider, conversionsNum);
            lfsrScale = CapSense_SsCalcLfsrScale(snsClkDivider, lfsrSize);
        #else
            lfsrSize = (uint8)CapSense_CSX_TX_CLK_SOURCE;
            lfsrScale = 0u;
        #endif /* (CapSense_CLK_SOURCE_PRSAUTO == CapSense_CSX_TX_CLK_SOURCE) */

        ptrWdgt->snsClkSource = lfsrSize | (uint8)(lfsrScale << CapSense_CLK_SOURCE_LFSR_SCALE_OFFSET);
    }
#endif /* ((CapSense_ENABLE == CapSense_CSX_EN) && (CapSense_ENABLE == CapSense_CSDV2)) */


#if(((CapSense_ENABLE == CapSense_CSX_EN) && (CapSense_ENABLE == CapSense_CSDV2) && (CapSense_CLK_SOURCE_PRSAUTO == CapSense_CSX_TX_CLK_SOURCE)) ||\
    ((CapSense_ENABLE == CapSense_CSD_EN) && (CapSense_ENABLE == CapSense_CSDV2) && (CapSense_CLK_SOURCE_PRSAUTO == CapSense_CSD_SNS_CLK_SOURCE)))
/*******************************************************************************
* Function Name: CapSense_SsCSDCalcLfsr
****************************************************************************//**
*
* \brief
*   The function finds a SSC polynomial size in the auto mode.
*
* \details
*   The SSC polynomial size in the auto mode is found based on the following
*   requirements:
*   - an LFSR value should be selected so that the max clock dither is limited with +/-10%          !!?
*   - at least one full spread spectrum polynomial should pass during the scan time.
*
* \param
*  snsClkDivider The divider value for the sense clock.
*  resolution The widget resolution.
*
* \return lfsrSize The LFSRSIZE value for the SENSE_PERIOD register.
*
*******************************************************************************/
static CY_INLINE  uint8 CapSense_SsCalcLfsrSize(uint32 modClkDivider, uint32 snsClkDivider, uint32 conversionsNum)
{
    uint8 lfsrSize = 0u;

    if((snsClkDivider * modClkDivider) > CapSense_HFCLK_SNSCLK_FACTOR)
    {
        /* Find LFSR value */
        if ((CapSense_SNSCLK_SSC4_LENGTH <= conversionsNum) &&
            (0uL == (conversionsNum % CapSense_SNSCLK_SSC4_LENGTH)) &&
            (CapSense_SNSCLK_SSC4_THRESHOLD < snsClkDivider))
        {
            lfsrSize = CapSense_CLK_SOURCE_SSC4;
        }
        else if ((CapSense_SNSCLK_SSC3_LENGTH <= conversionsNum) &&
                 (0uL == (conversionsNum % CapSense_SNSCLK_SSC3_LENGTH)) &&
                 (CapSense_SNSCLK_SSC3_THRESHOLD < snsClkDivider))
        {
            lfsrSize = CapSense_CLK_SOURCE_SSC3;
        }
        else if ((CapSense_SNSCLK_SSC2_LENGTH <= conversionsNum) &&
                 (0uL == (conversionsNum % CapSense_SNSCLK_SSC2_LENGTH)) &&
                 (CapSense_SNSCLK_SSC2_THRESHOLD < snsClkDivider))
        {
            lfsrSize = CapSense_CLK_SOURCE_SSC2;
        }
        else if ((CapSense_SNSCLK_SSC1_LENGTH <= conversionsNum) &&
                 (0uL == (conversionsNum % CapSense_SNSCLK_SSC1_LENGTH)) &&
                 (CapSense_SNSCLK_SSC1_THRESHOLD < snsClkDivider))
        {
            lfsrSize = CapSense_CLK_SOURCE_SSC1;
        }
        else
        {
            lfsrSize = (uint8)CapSense_CLK_SOURCE_DIRECT;
        }
    }
    else
    {
        lfsrSize = (uint8)CapSense_CLK_SOURCE_DIRECT;
    }

    return (lfsrSize);
}


/*******************************************************************************
* Function Name: CapSense_SsCSDCalcLfsr
****************************************************************************//**
*
* \brief
*   This function calculates the LFSR scale value.
*
* \details
*   The LFSR scale value is used to increase the clock dither if the desired dither
*   is wider than can be achieved with the current Sense Clock Divider and current LFSR
*   period.
*
*   This returns the LFSR scale value needed to keep the clock dither in
*   range +/-10%.
*
* \param
*  snsClkDivider The divider value for the sense clock.
*  lfsrSize The period of the LFSR sequence.
*           For devices with CapSense_CSDV2_REF9P6UA_EN = TRUE, the
*           mode parameters can take the following values:
*           CapSense_CLK_SOURCE_DIRECT The spreadspectrum is not used.
*           CapSense_CLK_SOURCE_SSC1   The length of LFSR sequence is 2-bits
*           CapSense_CLK_SOURCE_SSC2   The length of LFSR sequence is 3-bits
*           CapSense_CLK_SOURCE_SSC3   The length of LFSR sequence is 4-bits
*           CapSense_CLK_SOURCE_SSC4   The length of LFSR sequence is 5-bits
*
*           For devices with CapSense_CSDV2_REF9P6UA_EN = TRUE, the
*           mode parameters can take the following values:
*           CapSense_CLK_SOURCE_DIRECT The spreadspectrum is not used.
*           CapSense_CLK_SOURCE_SSC1   The length of LFSR sequence is 6-bits
*           CapSense_CLK_SOURCE_SSC2   The length of LFSR sequence is 7-bits
*           CapSense_CLK_SOURCE_SSC3   The length of LFSR sequence is 9-bits
*           CapSense_CLK_SOURCE_SSC4   The length of LFSR sequence is 10-bits
*
* \return The LFSR scale value needed to keep the clock dither in range +/-10%.
*
*******************************************************************************/
static CY_INLINE uint8 CapSense_SsCalcLfsrScale(uint32 snsClkDivider, uint8 lfsrSize)
{
    uint16 lfsrTmp;
    uint8 lfsrScale = 0u;

    switch(lfsrSize)
    {
        case CapSense_CLK_SOURCE_SSC1:
        {
            lfsrTmp = CapSense_SNSCLK_SSC1_DITHER;
            break;
        }
        case CapSense_CLK_SOURCE_SSC2:
        {
            lfsrTmp = CapSense_SNSCLK_SSC2_DITHER;
            break;
        }
        case CapSense_CLK_SOURCE_SSC3:
        {
            lfsrTmp = CapSense_SNSCLK_SSC3_DITHER;
            break;
        }
        case CapSense_CLK_SOURCE_SSC4:
        {
            lfsrTmp = CapSense_SNSCLK_SSC4_DITHER;
            break;
        }
        default:
        {
            lfsrTmp = 0u;
            break;
        }
    }

    if((lfsrSize != CapSense_CLK_SOURCE_DIRECT) && (lfsrTmp > 0u))
    {
        lfsrTmp = (uint16)(snsClkDivider / ((uint32)lfsrTmp * 10u));

        if(lfsrTmp > CapSense_UINT8_MAX_VAL)
        {
            lfsrScale = CapSense_MSB_OFFSET;
            lfsrTmp = lfsrTmp >> CapSense_MSB_OFFSET;
        }
        else
        {
            lfsrScale = 0u;
            lfsrTmp = lfsrTmp & CapSense_UINT8_MAX_VAL;
        }

        lfsrTmp >>= 1u;
        while(lfsrTmp > 0u)
        {
            lfsrScale++;
            lfsrTmp >>= 1u;
        }
    }

    return (lfsrScale);
}
#endif /* (((CapSense_ENABLE == CapSense_CSX_EN) && (CapSense_ENABLE == CapSense_CSDV2) && (CapSense_CLK_SOURCE_PRSAUTO == CapSense_CSX_TX_CLK_SOURCE)) ||\
          ((CapSense_ENABLE == CapSense_CSD_EN) && (CapSense_ENABLE == CapSense_CSDV2) && (CapSense_CLK_SOURCE_PRSAUTO == CapSense_CSD_SNS_CLK_SOURCE))) */


/* [] END OF FILE */
