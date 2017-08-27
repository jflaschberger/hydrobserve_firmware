;/*******************************************************************************
;* \file CapSense_1_SensingCSXIar_LL.s
;* \version 3.0
;*
;* \brief
;*   This file defines ISR for CSX scan module and function to start the CSX scanning.
;*   The functions are written in assembly due to timing criticality.
;*   This file is for IAR.
;*
;* \see CapSense P4 v3.0 Datasheet
;* 
;*//*****************************************************************************
;* Copyright (2016), Cypress Semiconductor Corporation.
;********************************************************************************
;* This software is owned by Cypress Semiconductor Corporation (Cypress) and is
;* protected by and subject to worldwide patent protection (United States and
;* foreign), United States copyright laws and international treaty provisions.
;* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
;* license to copy, use, modify, create derivative works of, and compile the
;* Cypress Source Code and derivative works for the sole purpose of creating
;* custom software in support of licensee product to be used only in conjunction
;* with a Cypress integrated circuit as specified in the applicable agreement.
;* Any reproduction, modification, translation, compilation, or representation of
;* this software except as specified above is prohibited without the express
;* written permission of Cypress.
;*
;* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH
;* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
;* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
;* Cypress reserves the right to make changes without further notice to the
;* materials described herein. Cypress does not assume any liability arising out
;* of the application or use of any product or circuit described herein. Cypress
;* does not authorize its products for use as critical components in life-support
;* systems where a malfunction or failure may reasonably be expected to result in
;* significant injury to the user. The inclusion of Cypress' product in a life-
;* support systems application implies that the manufacturer assumes all risk of
;* such use and in doing so indemnifies Cypress against all charges. Use may be
;* limited by and subject to the applicable Cypress software license agreement.
;**********************************************************************************/

    INCLUDE cyfitteriar.inc

CSD_BASE EQU (CapSense_1_CSD__CSD_ID)
CSD_CONFIG EQU (CapSense_1_CSD__CSD_CONFIG - CSD_BASE)
CSD_INTR EQU (CapSense_1_CSD__CSD_INTR - CSD_BASE)
CSD_COUNTER EQU (CapSense_1_CSD__CSD_COUNTER - CSD_BASE)
CSD_STATUS EQU (CapSense_1_CSD__CSD_STATUS - CSD_BASE)

CSD_CFG_FILTER_EN EQU (1 << 3)
CSD_CFG_DUAL_CAP_EN EQU (1 << 4)
CSD_CFG_SENSE_EN EQU (1 << 12)
CSD_CFG_REFBUF_OUTSEL_MUXB EQU (1 << 21)
CSD_START_MASK EQU (CSD_CFG_FILTER_EN | CSD_CFG_DUAL_CAP_EN | CSD_CFG_SENSE_EN | CSD_CFG_REFBUF_OUTSEL_MUXB)
CSD_STOP_MASK EQU (~CSD_START_MASK)
CSD_STS_SENSE EQU (1 << 1)
CSD_STS_SAMPLE EQU (1 << 3)
CSD_STS_MASK EQU (CSD_STS_SENSE | CSD_STS_SAMPLE)
 
    IMPORT CapSense_1_csxScanPeriod
    IMPORT CapSense_1_csxRawGarbage
    IMPORT CapSense_1_csxRawCount
    IMPORT CapSense_1_CSXPostScanApiPtr
    
    SECTION .text:CODE:ROOT(3)
    PUBLIC CapSense_1_SsCSXScanISR
    PUBLIC CapSense_1_SsCSXStartSampleExt
    THUMB

;-------------------------------------------------------------------------------
; Function Name: CapSense_1_SsCSXScanISR
;-------------------------------------------------------------------------------
;
; @par Summary:
;	ISR for CSX module to support IAR.
;
;-------------------------------------------------------------------------------
; void CapSense_1_SsCSXScanISR (void)
.thumb_func
CapSense_1_SsCSXScanISR:
 
    PUSH 	{R4, LR}

    LDR    	R4, A_CSDHW_BASE_ADDR           ; CSDCounter = CSD_COUNTER_REG
    LDR    	R0, [r4, #CSD_COUNTER]

    LDR    	R1, A_CSX_RAW_GARBAGE           ; R1 = &csxRawGarbage
    LDR    	R2, A_CSX_RAW_COUNT             ; R2 = &csxRawCount 
    LDR    	R1, [R1]                        ; R1 = *csxRawGarbage 
    SUBS   	R0, R0, R1                      ; R0 = CSDCounter (R0) = csxRawGarbage (R1) 
    STRH   	R0, [R2, #0]                    ; *csxRawCount (R2) = R0 
    
    MOVS	R0, #1                          ; Clear CSD interrupt
    STR		R0, [R4, #CSD_INTR]
    
    LDR    	R0, [R4, #CSD_CONFIG]           ; Stop the CSD block
    LDR    	R1, A_CSD_STOP_MASK
    ANDS   	R0, R0, R1
    STR		R0, [R4, #CSD_CONFIG]
    
    LDR		R0, A_CSX_POSTSCAN_API_PTR      ; Call the C function
    LDR    	R0, [R0]
    BLX    	R0
   
    POP		{R4, PC}                        ; Return from the function
        
;-------------------------------------------------------------------------------
; Function Name: CapSense_1_SsCSXStartSampleExt
;-------------------------------------------------------------------------------
;
; @par Summary:
;	The assembly function to start the CSX scanning. This function is written in
;   the assembly because of the tight timing requirement. This is for IAR.
;
;-------------------------------------------------------------------------------
; void CapSense_1_SsCSXStartSampleExt (void)
CapSense_1_SsCSXStartSampleExt:
    
    PUSH	{R4, LR}    

    LDR		R4, A_CSDHW_BASE_ADDR           ; Start scan seqeunce
    LDR    	R0, [R4, #CSD_CONFIG]
    LDR		R2, A_CSD_START_MASK
    ORRS	R2, R2, R0
    
 	LDR     R3, A_CSX_SCAN_PERIOD          ; r1 (period) = (csxScanPeriod << 16)
	LDRH    R1, [R3, #0]
    LSLS    R1, R1, #16
    MOVS    R3, #CSD_STS_MASK               ; r3 = status mask
    STR		R1, [R4, #CSD_COUNTER]          ; Execute start-up sequence

wait_clk_sync:                              ; while (CSD_COUNTER_REG == period)
    LDR     R0, [R4, #CSD_COUNTER]  		; 2
    CMP     R1, R0                  		; 1
    BEQ     wait_clk_sync          		    ; 3 <- max 9 cpu clock cycle delay
    
    STR     R2, [R4, #CSD_CONFIG]   		; 2, Enable CSD block
    
wait_half_cycle:                            ; while (0x00 == (CSD_STATUS_REG & (CSD_STS_SAMPLE | CSD_STS_SENSE)))
    LDR     R0, [R4, #CSD_STATUS]  			; 2
    ANDS    R0, R0, R3              		; 1
    BEQ     wait_half_cycle        		    ; 3 <- max 20 cpu clock cycle delay
    
    LDR     R0, [R4, #CSD_COUNTER]          ; csxRawGarbage = CSD_COUNTER_REG
    LDR     R1, A_CSX_RAW_GARBAGE
    STR     R0, [R1] 
    
    POP     {R4, PC}                        ; Return from the function


; ------- Macros and definitions -------
    DATA
   
A_CSD_STOP_MASK         dc32 CSD_STOP_MASK
A_CSD_START_MASK        dc32 CSD_START_MASK

A_CSDHW_BASE_ADDR       dc32 CSD_BASE
A_CSX_SCAN_PERIOD       dc32 CapSense_1_csxScanPeriod   
A_CSX_RAW_GARBAGE       dc32 CapSense_1_csxRawGarbage
A_CSX_RAW_COUNT         dc32 CapSense_1_csxRawCount
A_CSX_POSTSCAN_API_PTR  dc32 CapSense_1_CSXPostScanApiPtr
    
    END

;[] END OF FILE
