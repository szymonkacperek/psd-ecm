/**
  ******************************************************************************
  * File Name          : canopen_object_dict.c
  * Description        : Zbior obiektow znajdujacych sie w sieci. Parametry dobrane
  *  					 wg standardu CANopen.
  ******************************************************************************
  */


#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "main.h"
#include "gpio.h"

/*******************************************************************************
 CANopen DATA DYPES
 *******************************************************************************/
typedef unsigned char bool_t; /**< bool_t */
typedef float float32_t; /**< float32_t */
typedef long double float64_t; /**< float64_t */
typedef char char_t; /**< char_t */
typedef unsigned char oChar_t; /**< oChar_t */
typedef unsigned char domain_t; /**< domain_t */

typedef uint8_t UNSIGNED8;
typedef uint16_t UNSIGNED16;
typedef uint32_t UNSIGNED32;
typedef uint64_t UNSIGNED64;
typedef int8_t INTEGER8;
typedef int16_t INTEGER16;
typedef int32_t INTEGER32;
typedef int64_t INTEGER64;
typedef float32_t REAL32;
typedef float64_t REAL64;
typedef char_t VISIBLE_STRING;
typedef oChar_t OCTET_STRING;
typedef domain_t DOMAIN;


/*******************************************************************************
 DEVICE INFO:
 VendorName:     STMicroelectronics
 ProductName:    STM32F407VGT6
 *******************************************************************************/

/*******************************************************************************
 FEATURES
 *******************************************************************************/
#define CO_NO_SYNC                  0   //Associated objects:
#define CO_NO_EMERGENCY             0   //Associated objects:
#define CO_NO_SDO_SERVER            0   //Associated objects:
#define CO_NO_SDO_CLIENT            0
#define CO_NO_RPDO                  0   //Associated objects:
#define CO_NO_TPDO                  0   //Associated objects:
#define CO_NO_NMT_MASTER            0
#define CO_NO_TRACE                 0
#define SDO_Download				0b00101111
#define SDO_Upload					0b0100

typedef struct {
	UNSIGNED8 node_id;
	UNSIGNED16 index;
	UNSIGNED8 subindex_1;
	UNSIGNED8 subindex_2;
	UNSIGNED8 subindex_3;
	UNSIGNED32 sdo_upload_id;
	UNSIGNED32 sdo_download_id;
	UNSIGNED32 pdo_consumer_id;
	UNSIGNED32 pdo_producer_id;
	UNSIGNED8 emcy_id;
} CanopenNode;

/*******************************************************************************
 OBJECT DICTIONARY
 *******************************************************************************/
#define CO_OD_NoOfElements             9
CanopenNode dashboard;				// ECM
CanopenNode inverter_1;				// ECM
CanopenNode inverter_2;				// ECM
CanopenNode mppt_1;					// ECM
CanopenNode mppt_2;					// ECM
CanopenNode mppt_3;					// ECM
CanopenNode ecm;						// MPPTs, Inverters, Dashboard, Lights controller
CanopenNode lights_controller;		// ECM
CanopenNode bms;						// ECM

void CanopenObjectDictInit();


