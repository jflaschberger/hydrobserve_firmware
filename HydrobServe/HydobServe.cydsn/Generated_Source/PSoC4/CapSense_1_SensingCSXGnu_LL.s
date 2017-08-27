/*******************************************************************************
* \file CapSense_1_SensingCSXGnu_LL.s
* \version 3.0
*
* \brief
*   This file defines ISR for CSX module and a time crtical funciton to start 
*   sensor scan. This file is to support GCC compiler.
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
**********************************************************************************/

.include "cyfittergnu.inc"

/* ******************************************************************************
*  CSD block register definitions and masks
******************************************************************************* */

/* CSD Registers offsets 				*/
.set    CSD_BASE,       (CapSense_1_CSD__CSD_ID)
.set    CSD_CONFIG,     (CapSense_1_CSD__CSD_CONFIG  - CSD_BASE)
.set    CSD_INTR,       (CapSense_1_CSD__CSD_INTR    - CSD_BASE)
.set    CSD_COUNTER,    (CapSense_1_CSD__CSD_COUNTER - CSD_BASE)
.set    CSD_STATUS,     (CapSense_1_CSD__CSD_STATUS  - CSD_BASE)

/* CSD Config register bits and masks 	*/
.set    CSD_CFG_FILTER_EN,          (1 << 3)
.set    CSD_CFG_DUAL_CAP_EN,        (1 << 4)
.set    CSD_CFG_SENSE_EN,           (1 << 12)
.set    CSD_CFG_REFBUF_OUTSEL_MUXB, (1 << 21)

.set    CSD_START_MASK,             (CSD_CFG_FILTER_EN | CSD_CFG_DUAL_CAP_EN | CSD_CFG_SENSE_EN | CSD_CFG_REFBUF_OUTSEL_MUXB)
.set    CSD_STOP_MASK,              (~CSD_START_MASK)

.set    CSD_STS_SENSE,              (1 << 1)
.set    CSD_STS_SAMPLE,             (1 << 3)
.set    CSD_STS_MASK,               (CSD_STS_SENSE | CSD_STS_SAMPLE)

/* ******************************************************************************
* External definition for C variables
******************************************************************************* */

.extern CapSense_1_csxRawGarbage
.extern CapSense_1_csxScanPeriod
.extern CapSense_1_csxRawCount
.extern CapSense_1_CSXPostScanApiPtr
	
.syntax unified
.text
    
/*******************************************************************************
* Function Name: CapSense_1_SsCSXScanISR
********************************************************************************
*
* @par Summary:
*	ISR for CapSense component for CSX sensing. 
*
**********************************************************************************/

.align 3                                    /* Align to 8 byte boundary (2^n) */    
.global CapSense_1_SsCSXScanISR
.func 	CapSense_1_SsCSXScanISR, CapSense_1_SsCSXScanISR
.thumb_func
CapSense_1_SsCSXScanISR:
    PUSH 	{R4, LR}

    LDR    	R4, .A_CSDHW_BASE_ADDR          /* CSDCounter = CSD_COUNTER_REG */
    LDR    	R0, [r4, #CSD_COUNTER]
    
    LDR    	R1, .A_CSX_RAW_GARBAGE          /* R1 = &csxRawGarbage */
    LDR    	R2, .A_CSX_RAW_COUNT            /* R2 = &csxRawCount */
    LDR    	R1, [R1]                        /* R1 = *csxRawGarbage */
    SUBS   	R0, R0, R1                      /* R0 = CSDCounter (R0) = csxRawGarbage (R1) */
    STRH   	R0, [R2, #0]                     /* *csxRawCount (R2) = R0 */

    MOVS	R0, #1                          /* Clear CSD interrupt */
    STR		R0, [R4, #CSD_INTR]
    
    LDR    	R0, [R4, #CSD_CONFIG]           /* Stop the CSD block */
    LDR    	R1, .A_CSD_STOP_MASK
    ANDS   	R0, R0, R1
    STR		R0, [R4, #CSD_CONFIG]
    
    LDR		R0, .A_CSX_POSTSCAN_API_PTR     /* Call the C function   */
    LDR    	R0, [R0]
    BLX    	R0
   
    POP		{R4, PC}                        /* Return from the function */

.endfunc

/*******************************************************************************
* Function Name: CapSense_1_SsCSXStartSampleExt
********************************************************************************
*
* @par Summary:
*	The assembly function to start the CSX scanning, this is written in the assembly 
*	because of the tight timing requirement.
*
**********************************************************************************/

.align 3                    /* Align to 8 byte boundary (2^n) */
.global CapSense_1_SsCSXStartSampleExt
.func CapSense_1_SsCSXStartSampleExt, CapSense_1_SsCSXStartSampleExt
.thumb_func
CapSense_1_SsCSXStartSampleExt:
    PUSH	{R4, LR}    

    LDR		R4, .A_CSDHW_BASE_ADDR           /* Start scan seqeunce     */
    LDR    	R0, [R4, #CSD_CONFIG]
    LDR		R2, .A_CSD_START_MASK
    ORRS	R2, R2, R0
    
 	LDR     R3, .A_CSX_SCAN_PERIOD          /* r1 (period) = (csxScanPeriod << 16);	*/
	LDRH    R1, [R3, #0]
    LSLS    R1, R1, #16
    MOVS    R3, #CSD_STS_MASK               /* r3 = status mask	*/  		
    STR		R1, [R4, #CSD_COUNTER]          /* Execute start-up sequence    */

.wait_clk_sync:                             /* while (CSD_COUNTER_REG == period)	*/
    LDR     R0, [R4, #CSD_COUNTER]  		/* 2	*/
    CMP     R1, R0                  		/* 1	*/
    BEQ     .wait_clk_sync          		/* 3 <- max 9 cpu clock cycle delay */
    
    STR     R2, [R4, #CSD_CONFIG]   		/* 2, Enable CSD block */
    
.wait_half_cycle:                           /* while (0x00 == (CSD_STATUS_REG & (CSD_STS_SAMPLE | CSD_STS_SENSE))) */
    LDR     R0, [R4, #CSD_STATUS]  			/* 2 */
    ANDS    R0, R0, R3              		/* 1 */
    BEQ     .wait_half_cycle        		/* 3 */ /* <- max 20 cpu clock cycle delay */
    
    LDR     R0, [R4, #CSD_COUNTER]          /* csxRawGarbage = CSD_COUNTER_REG */
    LDR     R1, .A_CSX_RAW_GARBAGE
    STR     R0, [R1] 
    
    POP     {R4, PC}                        /* Return from the function */

/* ***** Macros and definitions ***** */

.align 3                    /* Align to 8 byte boundary (2^n) */

.A_CSDHW_BASE_ADDR:
	.word   CSD_BASE
.A_CSD_STOP_MASK:
    .word   CSD_STOP_MASK
.A_CSD_START_MASK:
    .word   CSD_START_MASK
	
.A_CSX_SCAN_PERIOD:
	.word	CapSense_1_csxScanPeriod	
.A_CSX_RAW_GARBAGE:
	.word 	CapSense_1_csxRawGarbage
.A_CSX_RAW_COUNT:
	.word	CapSense_1_csxRawCount
.A_CSX_POSTSCAN_API_PTR:
	.word	CapSense_1_CSXPostScanApiPtr
	
.endfunc
.end


/* [] END OF FILE */
