/***************************************************************************//**
* \file CYBLE_custom.h
* \version 3.10
* 
* \brief
*  Contains the function prototypes and constants for the Custom Service.
* 
********************************************************************************
* \copyright
* Copyright 2014-2016, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_BLE_CYBLE_CUSTOM_H)
#define CY_BLE_CYBLE_CUSTOM_H

#include "BLE_gatt.h"


/***************************************
* Conditional Compilation Parameters
***************************************/

/* Maximum supported Custom Services */
#define CYBLE_CUSTOMS_SERVICE_COUNT                  (0x03u)
#define CYBLE_CUSTOMC_SERVICE_COUNT                  (0x00u)
#define CYBLE_CUSTOM_SERVICE_CHAR_COUNT              (0x04u)
#define CYBLE_CUSTOM_SERVICE_CHAR_DESCRIPTORS_COUNT  (0x00u)

/* Below are the indexes and handles of the defined Custom Services and their characteristics */
#define CYBLE_VOLUME_DATA_SERVICE_INDEX   (0x00u) /* Index of Volume data service in the cyBle_customs array */
#define CYBLE_VOLUME_DATA_DRUNK_VOLUME_CHAR_INDEX   (0x00u) /* Index of drunk_volume characteristic */
#define CYBLE_VOLUME_DATA_NUMBER_DRINK_EVTS_CHAR_INDEX   (0x01u) /* Index of number_drink_evts characteristic */
#define CYBLE_VOLUME_DATA_PULSE_MULTIPLIER_CHAR_INDEX   (0x02u) /* Index of pulse_multiplier characteristic */
#define CYBLE_VOLUME_DATA_PULSE_COUNT_CHAR_INDEX   (0x03u) /* Index of pulse_count characteristic */

#define CYBLE_USER_DATA_SERVICE_INDEX   (0x01u) /* Index of user_data service in the cyBle_customs array */
#define CYBLE_USER_DATA_FIRST_NAME_CHAR_INDEX   (0x00u) /* Index of First_Name characteristic */
#define CYBLE_USER_DATA_LAST_NAME_CHAR_INDEX   (0x01u) /* Index of Last_Name characteristic */
#define CYBLE_USER_DATA_USER_ID_CHAR_INDEX   (0x02u) /* Index of User_ID characteristic */
#define CYBLE_USER_DATA_GENER_CHAR_INDEX   (0x03u) /* Index of Gener characteristic */

#define CYBLE_BATTERY_SERVICE_INDEX   (0x02u) /* Index of Battery service in the cyBle_customs array */
#define CYBLE_BATTERY_BATTERY_LEVEL_CHAR_INDEX   (0x00u) /* Index of Battery_level characteristic */


#define CYBLE_VOLUME_DATA_SERVICE_HANDLE   (0x000Eu) /* Handle of Volume data service */
#define CYBLE_VOLUME_DATA_DRUNK_VOLUME_DECL_HANDLE   (0x000Fu) /* Handle of drunk_volume characteristic declaration */
#define CYBLE_VOLUME_DATA_DRUNK_VOLUME_CHAR_HANDLE   (0x0010u) /* Handle of drunk_volume characteristic */
#define CYBLE_VOLUME_DATA_NUMBER_DRINK_EVTS_DECL_HANDLE   (0x0011u) /* Handle of number_drink_evts characteristic declaration */
#define CYBLE_VOLUME_DATA_NUMBER_DRINK_EVTS_CHAR_HANDLE   (0x0012u) /* Handle of number_drink_evts characteristic */
#define CYBLE_VOLUME_DATA_PULSE_MULTIPLIER_DECL_HANDLE   (0x0013u) /* Handle of pulse_multiplier characteristic declaration */
#define CYBLE_VOLUME_DATA_PULSE_MULTIPLIER_CHAR_HANDLE   (0x0014u) /* Handle of pulse_multiplier characteristic */
#define CYBLE_VOLUME_DATA_PULSE_COUNT_DECL_HANDLE   (0x0015u) /* Handle of pulse_count characteristic declaration */
#define CYBLE_VOLUME_DATA_PULSE_COUNT_CHAR_HANDLE   (0x0016u) /* Handle of pulse_count characteristic */

#define CYBLE_USER_DATA_SERVICE_HANDLE   (0x0017u) /* Handle of user_data service */
#define CYBLE_USER_DATA_FIRST_NAME_DECL_HANDLE   (0x0018u) /* Handle of First_Name characteristic declaration */
#define CYBLE_USER_DATA_FIRST_NAME_CHAR_HANDLE   (0x0019u) /* Handle of First_Name characteristic */
#define CYBLE_USER_DATA_LAST_NAME_DECL_HANDLE   (0x001Au) /* Handle of Last_Name characteristic declaration */
#define CYBLE_USER_DATA_LAST_NAME_CHAR_HANDLE   (0x001Bu) /* Handle of Last_Name characteristic */
#define CYBLE_USER_DATA_USER_ID_DECL_HANDLE   (0x001Cu) /* Handle of User_ID characteristic declaration */
#define CYBLE_USER_DATA_USER_ID_CHAR_HANDLE   (0x001Du) /* Handle of User_ID characteristic */
#define CYBLE_USER_DATA_GENER_DECL_HANDLE   (0x001Eu) /* Handle of Gener characteristic declaration */
#define CYBLE_USER_DATA_GENER_CHAR_HANDLE   (0x001Fu) /* Handle of Gener characteristic */

#define CYBLE_BATTERY_SERVICE_HANDLE   (0x0020u) /* Handle of Battery service */
#define CYBLE_BATTERY_BATTERY_LEVEL_DECL_HANDLE   (0x0021u) /* Handle of Battery_level characteristic declaration */
#define CYBLE_BATTERY_BATTERY_LEVEL_CHAR_HANDLE   (0x0022u) /* Handle of Battery_level characteristic */



#if(CYBLE_CUSTOMS_SERVICE_COUNT != 0u)
    #define CYBLE_CUSTOM_SERVER
#endif /* (CYBLE_CUSTOMS_SERVICE_COUNT != 0u) */
    
#if(CYBLE_CUSTOMC_SERVICE_COUNT != 0u)
    #define CYBLE_CUSTOM_CLIENT
#endif /* (CYBLE_CUSTOMC_SERVICE_COUNT != 0u) */

/***************************************
* Data Struct Definition
***************************************/

/**
 \addtogroup group_service_api_custom
 @{
*/

#ifdef CYBLE_CUSTOM_SERVER

/** Contains information about Custom Characteristic structure */
typedef struct
{
    /** Custom Characteristic handle */
    CYBLE_GATT_DB_ATTR_HANDLE_T customServCharHandle;
    /** Custom Characteristic Descriptors handles */
    CYBLE_GATT_DB_ATTR_HANDLE_T customServCharDesc[     /* MDK doesn't allow array with zero length */
        CYBLE_CUSTOM_SERVICE_CHAR_DESCRIPTORS_COUNT == 0u ? 1u : CYBLE_CUSTOM_SERVICE_CHAR_DESCRIPTORS_COUNT];
} CYBLE_CUSTOMS_INFO_T;

/** Structure with Custom Service attribute handles. */
typedef struct
{
    /** Handle of a Custom Service */
    CYBLE_GATT_DB_ATTR_HANDLE_T customServHandle;
    
    /** Information about Custom Characteristics */
    CYBLE_CUSTOMS_INFO_T customServInfo[                /* MDK doesn't allow array with zero length */
        CYBLE_CUSTOM_SERVICE_CHAR_COUNT == 0u ? 1u : CYBLE_CUSTOM_SERVICE_CHAR_COUNT];
} CYBLE_CUSTOMS_T;


#endif /* (CYBLE_CUSTOM_SERVER) */

/** @} */

/** \cond IGNORE */
/* The custom Client functionality is not functional in current version of 
* the component.
*/
#ifdef CYBLE_CUSTOM_CLIENT

typedef struct
{
    /** Custom Descriptor handle */
    CYBLE_GATT_DB_ATTR_HANDLE_T descHandle;
	/** Custom Descriptor 128 bit UUID */
	const void *uuid;           
    /** UUID Format - 16-bit (0x01) or 128-bit (0x02) */
	uint8 uuidFormat;
   
} CYBLE_CUSTOMC_DESC_T;

typedef struct
{
    /** Characteristic handle */
    CYBLE_GATT_DB_ATTR_HANDLE_T customServCharHandle;
	/** Characteristic end handle */
    CYBLE_GATT_DB_ATTR_HANDLE_T customServCharEndHandle;
	/** Custom Characteristic UUID */
	const void *uuid;           
    /** UUID Format - 16-bit (0x01) or 128-bit (0x02) */
	uint8 uuidFormat;
    /** Properties for value field */
    uint8  properties;
	/** Number of descriptors */
    uint8 descCount;
    /** Characteristic Descriptors */
    CYBLE_CUSTOMC_DESC_T * customServCharDesc;
} CYBLE_CUSTOMC_CHAR_T;

/** Structure with discovered attributes information of Custom Service */
typedef struct
{
    /** Custom Service handle */
    CYBLE_GATT_DB_ATTR_HANDLE_T customServHandle;
	/** Custom Service UUID */
	const void *uuid;           
    /** UUID Format - 16-bit (0x01) or 128-bit (0x02) */
	uint8 uuidFormat;
	/** Number of characteristics */
    uint8 charCount;
    /** Custom Service Characteristics */
    CYBLE_CUSTOMC_CHAR_T * customServChar;
} CYBLE_CUSTOMC_T;

#endif /* (CYBLE_CUSTOM_CLIENT) */
/** \endcond */

#ifdef CYBLE_CUSTOM_SERVER

extern const CYBLE_CUSTOMS_T cyBle_customs[CYBLE_CUSTOMS_SERVICE_COUNT];

#endif /* (CYBLE_CUSTOM_SERVER) */

/** \cond IGNORE */
#ifdef CYBLE_CUSTOM_CLIENT

extern CYBLE_CUSTOMC_T cyBle_customc[CYBLE_CUSTOMC_SERVICE_COUNT];

#endif /* (CYBLE_CUSTOM_CLIENT) */
/** \endcond */


/***************************************
* Private Function Prototypes
***************************************/

/** \cond IGNORE */
void CyBle_CustomInit(void);

#ifdef CYBLE_CUSTOM_CLIENT

void CyBle_CustomcDiscoverServiceEventHandler(const CYBLE_DISC_SRVC128_INFO_T *discServInfo);
void CyBle_CustomcDiscoverCharacteristicsEventHandler(uint16 discoveryService, const CYBLE_DISC_CHAR_INFO_T *discCharInfo);
CYBLE_GATT_ATTR_HANDLE_RANGE_T CyBle_CustomcGetCharRange(uint8 incrementIndex);
void CyBle_CustomcDiscoverCharDescriptorsEventHandler(const CYBLE_DISC_DESCR_INFO_T *discDescrInfo);

#endif /* (CYBLE_CUSTOM_CLIENT) */

/** \endcond */

/***************************************
* External data references 
***************************************/

#ifdef CYBLE_CUSTOM_CLIENT

extern CYBLE_CUSTOMC_T cyBle_customCServ[CYBLE_CUSTOMC_SERVICE_COUNT];

#endif /* (CYBLE_CUSTOM_CLIENT) */


/** \cond IGNORE */
/***************************************
* The following code is DEPRECATED and
* should not be used in new projects.
***************************************/
#define customServiceCharHandle         customServCharHandle
#define customServiceCharDescriptors    customServCharDesc
#define customServiceHandle             customServHandle
#define customServiceInfo               customServInfo
/** \endcond */


#endif /* CY_BLE_CYBLE_CUSTOM_H  */

/* [] END OF FILE */
