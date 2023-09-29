/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 *
 * Battery Client Header file
 *
 */
#ifndef _BATTERY_CLIENT_H_
#define _BATTERY_CLIENT_H_

#include "wiced_transport.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_battery_client.h"
#include "app.h"

/******************************************************************************
 *  defines
 ******************************************************************************/

#define BAC_TRACE_DBG(format, ...)  WICED_BT_TRACE("[%s] " format, __FUNCTION__, ##__VA_ARGS__)
#define BAC_TRACE_ERR(format, ...)  WICED_BT_TRACE("Err: [%s] " format, __FUNCTION__, ##__VA_ARGS__)

/* App Timer Timeout in seconds  */
#define BATTC_APP_TIMEOUT_IN_SECONDS           1

#ifndef WICED_BUTTON_PRESSED_VALUE
#define WICED_BUTTON_PRESSED_VALUE 1
#endif

#if !defined(CYW20819A1)
 #define BUTTON_PRESSED              WICED_BUTTON_PRESSED_VALUE
#endif

#ifdef CYW20706A2
 #define APP_BUTTON                  WICED_GPIO_BUTTON
 #define APP_BUTTON_SETTINGS        (WICED_GPIO_BUTTON_SETTINGS(GPIO_EN_INT_BOTH_EDGE))
 #define APP_BUTTON_DEFAULT_STATE    WICED_GPIO_BUTTON_DEFAULT_STATE
 #define APP_LED                     WICED_PLATFORM_LED_1
#endif

#define BATTERY_CLIENT_LOCAL_KEYS_VS_ID         ( WICED_NVRAM_VSID_START )
#define BATTERY_CLIENT_PAIRED_KEYS_START_VS_ID  ( WICED_NVRAM_VSID_START + 1 )

/******************************************************************************
 *                                Structures
 ******************************************************************************/
/* structure to store GATT attributes for read/write operations */
typedef struct
{
    uint16_t    handle;
    uint16_t    attr_len;
    const void *p_attr;
} gatt_attribute_t;

/* Peer Info */
typedef struct
{
    uint16_t conn_id;                   // Connection Identifier
    uint8_t  role;                      // central or peripheral in the current connection
    uint8_t  addr_type;                 // peer address type
    uint8_t  transport;                 // peer connected transport
    uint8_t  peer_addr[BD_ADDR_LEN];    // Peer BD Address
} battery_service_client_peer_info_t;

/* Battery Service client application info */
typedef struct
{
#define BAC_DISCOVERY_STATE_SERVICE     0
#define BAC_DISCOVERY_STATE_CHAR        1
    uint8_t discovery_state;

    // Current value of the client configuration descriptor for characteristic 'Report'
    uint16_t                    bac_s_handle;
    uint16_t                    bac_e_handle;
}battery_service_client_app_t;

/******************************************************************************
 * extern data
 ******************************************************************************/
extern const wiced_transport_cfg_t   transport_cfg;
extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern battery_service_client_app_t battery_client_app_state;
extern battery_service_client_peer_info_t  battery_client_app_data;

/******************************************************************************
 *                          Function Protoyping
 ******************************************************************************/
void battery_client_handle_op_complete(wiced_bt_gatt_operation_complete_t *p_data);
wiced_result_t battery_client_management_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
wiced_result_t app_stack_init();
wiced_bt_gatt_status_t battery_client_gatts_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data);
void battery_client_connection_evt( wiced_bt_gatt_event_data_t * p_data );
wiced_bt_gatt_status_t battery_client_gatt_discovery_result(wiced_bt_gatt_discovery_result_t *p_data);
wiced_bt_gatt_status_t battery_client_gatt_operation_complete(wiced_bt_gatt_operation_complete_t *p_data);
void battery_client_process_read_rsp(wiced_bt_gatt_operation_complete_t *p_data);
void battery_client_indication_handler(wiced_bt_gatt_operation_complete_t *p_data);
void battery_client_notification_handler(wiced_bt_gatt_operation_complete_t *p_data);
wiced_bt_gatt_status_t battery_client_gatt_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data);

#endif // _BATTERY_CLIENT_H_
