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
* Battery Service Client
*
* The Battery Service Client application is designed to connect and access services of the Battery Server.
* Battery Client connects to a device advertising Battery Service UUID.
*
* Features demonstrated
*  - Registration with LE stack for various events.
*  - Read characteristic from a Battery Service Server device.
*  - Process the notifications received from the server.
*
* To demonstrate the app, work through the following steps.
* 1. Plug the AIROC eval board into your computer
* 2. Build and download the application (to the AIROC board)
* 3. On start of the application, push the application button on the board and release after
*    the message "Release now to start pairing." is displayed in the traces, so that Battery Client App scans
*    and connects to the Battery Service Server, which would have UUID_SERVICE_BATTERY in it's advertisements.
*
*    Note:- If no Battery Service Server device is found nearby for 90 secs, then scan stops automatically.
*    To restart the scan, push the button on the board and release after the message "Release now to start pairing."
*    is displayed.
*
* 4. Upon successful Connection, the Battery Client App would discover all the characteristics/descriptors
*    of the server device.
* 5. Once the connection is established with the LE peripheral (Battery Service found in the Peripheral),
*    the application can enable/disable for notifications, to receive the change in the battery level.
*    To enable/disable notifications from the server, push the user button on the board and release after
*    the message "Release now to enable/disable battery level notifications." is displayed.
* 6. To read the battery level of the server, push the button on the board and release
*    after the message "Release now to read battery level." is displayed.
*
*/

#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_battery_client.h"
#include "wiced_bt_battery_server.h"
#include "wiced_hal_nvram.h"
#include "wiced_platform.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_gatt_util.h"
#include "hci_control_api.h"
#include "wiced_timer.h"
#include "battery_client.h"
// remove 43022C1 after enabling design.modus for it
#if !defined(CYW20706A2) && !defined(CYW43012C0) && !defined(CYW43022C1)
 #include "cycfg_pins.h"
#endif

//#define ENABLE_HCI_TRACE 1 // configures HCI traces to be routed to the AIROC HCI interface
#define TEST_HCI_CONTROL

/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define APP_TIMER_IDLE 0xffffffff

/******************************************************************************
 *                           Variables Definitions
 ******************************************************************************/
uint32_t button_pushed_time = APP_TIMER_IDLE;;
battery_service_client_peer_info_t  battery_client_app_data;
battery_service_client_app_t battery_client_app_state;
uint32_t bac_app_timer_count = 0;
wiced_bool_t is_bas_enabled = WICED_FALSE, is_broadcast_enabled = WICED_FALSE;
wiced_timer_t bac_app_timer;

static uint8_t battery_service_found  = 0;
/******************************************************************************
 *                             External Definitions
 ******************************************************************************/
#ifdef BATTERY_LEVEL_BROADCAST
extern void battery_client_gatts_enable_broadcast ( uint16_t conn_id, uint16_t handle, uint8_t enable_broadcast );
extern uint16_t wicked_bt_battery_client_server_characterstic_handle;
#endif

#ifdef TEST_HCI_CONTROL
static uint32_t  battery_client_hci_rx_cmd(uint8_t *p_data, uint32_t length);
static void battery_client_hci_transport_status( wiced_transport_type_t type );
static void battery_client_hci_handle_get_version();
static void battery_client_hci_send_connect_event( uint8_t addr_type, BD_ADDR addr, uint16_t con_handle, uint8_t role );
static void battery_client_hci_send_disconnect_event( uint8_t reason, uint16_t con_handle );
#endif

#define TRANS_UART_BUFFER_SIZE          1024

const wiced_transport_cfg_t transport_cfg =
{
    .type = WICED_TRANSPORT_UART,
    .cfg =
    {
        .uart_cfg =
        {
            .mode = WICED_TRANSPORT_UART_HCI_MODE,
            .baud_rate =  HCI_UART_DEFAULT_BAUD
        },
    },
#if NEW_DYNAMIC_MEMORY_INCLUDED
    .heap_config =
    {
        .data_heap_size = 1024 * 4 + 1500 * 2,
        .hci_trace_heap_size = 1024 * 2,
        .debug_trace_heap_size = 1024,
    },
#else
    .rx_buff_pool_cfg =
    {
        .buffer_size  = TRANS_UART_BUFFER_SIZE,
        .buffer_count = 1
    },
#endif

#ifdef TEST_HCI_CONTROL
    .p_status_handler = battery_client_hci_transport_status,
    .p_data_handler = battery_client_hci_rx_cmd,
#else
    .p_status_handler = NULL,
    .p_data_handler = NULL,
#endif

    .p_tx_complete_cback = NULL
};


/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

static void battery_client_enable( wiced_bool_t enable )
{
    is_bas_enabled = enable;

    if(is_bas_enabled)
    {
        wiced_bt_battery_client_enable( battery_client_app_data.conn_id );
    }
    else
    {
        wiced_bt_battery_client_disable( battery_client_app_data.conn_id );
    }
}

static void battery_client_out_bytes(char * msg, uint16_t len, uint8_t  *p_data)
{
    WICED_BT_TRACE("%s data: ", msg);
    while (len--)
    {
        WICED_BT_TRACE("%02x ",*p_data++);
    }
    WICED_BT_TRACE("\n");
}

static void battery_client_show_data(char * msg, wiced_bt_battery_client_event_t event, wiced_bt_battery_client_event_data_t *p_data)
{
//  WICED_BT_TRACE("show data, uuid=%04x  %s\n", p_data->data.uuid, msg);
    uint8_t flags8 = 0, flags16 = 0;

    switch (p_data->data.uuid)
    {
    case UUID_CHARACTERISTIC_BATTERY_LEVEL:
        WICED_BT_TRACE("[%13s] Battery Level:%d\n", msg, p_data->data.p_data[0]);
        break;

 #ifdef BAS_1_1
    case UUID_CHARACTERISTIC_BATTERY_LEVEL_STATUS:
        {
            UINT16 identifier = 0, power_state = 0, flags8 = 0;
            UINT8  battery_level = 0, additional_status = 0;

            STREAM_TO_UINT8(flags8, p_data->data.p_data);
            STREAM_TO_UINT16(power_state, p_data->data.p_data);
            batt_power_state_t *batt_power_state = (batt_power_state_t *)&power_state;

            if ( flags8 & BATTERY_LEVEL_STATUS_ID_PRESENT )
            {
                STREAM_TO_UINT16(identifier, p_data->data.p_data);
            }
            if ( flags8 & BATTERY_LEVEL_STATUS_LEVEL_PRESENT )
            {
                STREAM_TO_UINT8(battery_level, p_data->data.p_data);
            }
            if ( flags8 & BATTERY_LEVEL_STATUS_ADD_ST_PRESENT )
            {
               STREAM_TO_UINT8(additional_status, p_data->data.p_data);
            }
            batt_add_status_t *batt_add_status = (batt_add_status_t *)&additional_status;

            WICED_BT_TRACE("BATTERY_LEVEL_STATUS: flags8:%d  batt_present:%d wired_ext_pwr_connected:%d wireless_ext_pwr_connected:%d \
                           batt_charge_state:%d  batt_charge_level:%d batt_charge_type:%d batt_charge_fault:%d, rfu:%d\n", flags8, batt_power_state->batt_present,
                           batt_power_state->wired_ext_pwr_connected, batt_power_state->wireless_ext_pwr_connected, batt_power_state->batt_charge_state,
                           batt_power_state->batt_charge_level, batt_power_state->batt_charge_type, batt_power_state->batt_charge_fault, batt_power_state->rfu);

            WICED_BT_TRACE("identifier:%d  battery_level:%d service_required:%d rfu:%d\n\n", identifier, battery_level,
                            batt_add_status->service_required, batt_add_status->rfu );


            break;
        }
    case UUID_CHARACTERISTIC_BATTERY_ESTIMATED_SERVICE_DATE:
        {
            // okay to cast p_date into batt_estimated_service_date_t as no flags field or optional fields
            batt_estimated_service_date_t  *p_date = (batt_estimated_service_date_t *) p_data->data.p_data;
            uint32_t date = 0;
            uint8_t *buffer = (uint8_t *) p_date->estimated_service_date;
            STREAM_TO_UINT24(date, buffer);
            WICED_BT_TRACE("ESTIMATED_SERVICE_DATE:%d  %s \n\n", date, msg );
            break;
        }
    case UUID_CHARACTERISTIC_BATTERY_CRITICAL_STATUS:
        {
            // same as above - no flags field or optional fields
            batt_critical_status_t  *p_critical = (batt_critical_status_t *) p_data->data.p_data;
            WICED_BT_TRACE("CRITICAL_STATUS:%s critical_power_state:%d immediate_service_required:%d critical_status.rfu:%d\n\n",  msg,
            p_critical->batt_critical_status.critical_power_state,  p_critical->batt_critical_status.immediate_service_required, p_critical->batt_critical_status.rfu );
            break;
        }
    case UUID_CHARACTERISTIC_BATTERY_ENERGY_STATUS:
        {
            UINT16 external_source_power = 0, present_voltage = 0, available_energy = 0;
            UINT16 available_power_capacity = 0, change_rate = 0, available_energy_at_last_change = 0;

            // batt_energy_status_t *p_energy = (batt_energy_status_t *) p_data->data.p_data;
            STREAM_TO_UINT8(flags8, p_data->data.p_data);
            if ( flags8 & BATTERY_ENERGY_STATUS_FLAG_EXT_SOURCE_PRESENT )
            {
                STREAM_TO_UINT16( external_source_power, p_data->data.p_data );
            }
            if ( flags8 & BATTERY_ENERGY_STATUS_FLAG_PRESENT_VOLTAGE_PRESENT )
            {
                STREAM_TO_UINT16( present_voltage, p_data->data.p_data );
            }
            if ( flags8 & BATTERY_ENERGY_STATUS_FLAG_AVAILABLE_ENERGY_PRESENT )
            {
                STREAM_TO_UINT16( available_energy, p_data->data.p_data );
            }
            if ( flags8 & BATTERY_ENERGY_STATUS_FLAG_AVAILABLE_ENERGY_CAPACITY_PRESENT )
            {
                STREAM_TO_UINT16( available_power_capacity, p_data->data.p_data );
            }
             if ( flags8 & BATTERY_ENERGY_STATUS_FLAG_CHANGE_RATE_PRESENT )
            {
                STREAM_TO_UINT16( change_rate, p_data->data.p_data );
            }
            if ( flags8 & BATTERY_ENERGY_STATUS_FLAG_ENERGY_AT_LAST_CHANGE_PRESENT )
            {
                STREAM_TO_UINT16( available_energy_at_last_change, p_data->data.p_data );
            }
            WICED_BT_TRACE("BATTERY_ENERGY_STATUS:%s flags8:%d external_source_power:%d present_voltage:%d \n\n",
                            msg, flags8, external_source_power, present_voltage);
            WICED_BT_TRACE("available_energy:%d  available_power_capacity:%d change_rate:%d  available_energy_at_last_change:%d \n",
                            available_energy, available_power_capacity,  change_rate, available_energy_at_last_change );
            break;
        }
    case UUID_CHARACTERISTIC_BATTERY_TIME_STATUS:
        {
            UINT32  time_until_discharged = 0,  time_until_discharged_on_standby = 0, time_until_recharged = 0;
            //batt_time_status_t *p_time = (batt_time_status_t *)p_data->data.p_data;
            STREAM_TO_UINT8(flags8, p_data->data.p_data);
            STREAM_TO_UINT24( time_until_discharged, p_data->data.p_data );

            if ( flags8 & BATTERY_TIME_STATUS_FLAG_TIME_DISCHARGED_ON_STANDBY_PRESENT )
            {
                STREAM_TO_UINT24( time_until_discharged_on_standby, p_data->data.p_data );
            }
            if ( flags8 & BATTERY_TIME_STATUS_FLAG_TIME_UNTIL_RECHARGED_PRESENT )
            {
                STREAM_TO_UINT24( time_until_recharged, p_data->data.p_data );
            }

            WICED_BT_TRACE("TIME_STATUS:%s flags8:%d time_until_discharged:%d time_until_discharged_on_standby:%d time_until_recharged:%d\n\n",
                            msg, flags8, time_until_discharged, time_until_discharged_on_standby, time_until_recharged);
            break;
        }
    case UUID_CHARACTERISTIC_BATTERY_HEALTH_STATUS:
        {
            UINT8   health_summary = 0, current_temp = 0;
            UINT16  cycle_count = 0, deep_discharge_count = 0;

            //batt_health_status_t *p_health = (batt_health_status_t *)p_data->data.p_data;
            STREAM_TO_UINT8(flags8, p_data->data.p_data);

            if ( flags8 & BATTERY_HEALTH_STATUS_FLAG_SUMMARY_PRESENT )
            {
                STREAM_TO_UINT8( health_summary, p_data->data.p_data );
            }

            if ( flags8 & BATTERY_HEALTH_STATUS_FLAG_CYCLE_COUNT_PRESENT )
            {
                STREAM_TO_UINT16( cycle_count, p_data->data.p_data );
            }
            if ( flags8 & BATTERY_HEALTH_STATUS_FLAG_CURRENT_TEMP_PRESENT )
            {
                STREAM_TO_UINT8( current_temp, p_data->data.p_data );
            }

            if ( flags16 & BATTERY_HEALTH_STATUS_FLAG_DEEP_DISCHARGE_COUNT_PRESENT )
            {
                STREAM_TO_UINT16( deep_discharge_count, p_data->data.p_data );
            }

            WICED_BT_TRACE("HEALTHSTATUS:%s flags16:%d health_summary:%d cycle_count:%d current_temp:%d deep_discharge_count:%d \n\n", msg, flags16,
                           health_summary, cycle_count, current_temp, deep_discharge_count);
            break;
        }
    case UUID_CHARACTERISTIC_BATTERY_HEALTH_INFO:
        {
            //batt_health_info_t *p_health = (batt_health_info_t *)p_data->data.p_data;
            UINT16 cycle_count = 0;
            UINT8 min_op_temp = 0, max_op_temp = 0;
            STREAM_TO_UINT8(flags8, p_data->data.p_data);

            if ( flags8 & BATTERY_HEALTH_INFO_CYCLE_COUNT_DESIGNED_LIFETIME_PRESENT )
            {
                STREAM_TO_UINT16( cycle_count, p_data->data.p_data );
            }

            if ( flags8 & BATTERY_HEALTH_INFO_MIN_DESIGNED_OP_TEMP_PRESENT )
            {
                STREAM_TO_UINT8( min_op_temp, p_data->data.p_data );
                STREAM_TO_UINT8( max_op_temp, p_data->data.p_data );
            }

            WICED_BT_TRACE("HEALTH_STATUS:%s flags:%d cycle_count :%d min__op_temp:%d max_op_temp:%d \n\n", msg,
                            flags8, cycle_count, min_op_temp, max_op_temp );
            break;
        }
    case UUID_CHARACTERISTIC_BATTERY_INFO:
        {
            UINT8    features = 0, batt_aggr_grp = 0, batt_chemistry = 0;
            UINT32   batt_manuf_date = 0, batt_expr_date = 0;
            UINT16 batt_designed_cap = 0, batt_low_energy = 0, batt_critical_energy = 0, batt_nominal_volage = 0;

            STREAM_TO_UINT16(flags16, p_data->data.p_data);
            STREAM_TO_UINT8(features, p_data->data.p_data);
            batt_features_t *batt_features = (batt_features_t *)&features;

            //batt_info_t *p_info = (batt_info_t *)p_data->data.p_data;
            if ( flags16 & BATTERY_INFO_FLAG_BATT_MANUF_DATE_PRESENT )
            {
                STREAM_TO_UINT24( batt_manuf_date, p_data->data.p_data );
            }
            if ( flags16 & BATTERY_INFO_FLAG_BATT_EXPRATION_DATE_PRESENT )
            {
                STREAM_TO_UINT24( batt_expr_date, p_data->data.p_data );
            }
            if ( flags16 & BATTERY_INFO_FLAG_BATT_DESIGNED_CAP_PRESENT )
            {
                STREAM_TO_UINT16( batt_designed_cap, p_data->data.p_data );
            }
            if ( flags16 & BATTERY_INFO_FLAG_BATT_LOW_ENGERGY_PRESENT )
            {
                STREAM_TO_UINT16( batt_low_energy, p_data->data.p_data );
            }
            if ( flags16 & BATTERY_INFO_FLAG_BATT_CRITICAL_ENERGY_PRESENT )
            {
                STREAM_TO_UINT16( batt_critical_energy, p_data->data.p_data );
            }
            if ( flags16 & BATTERY_INFO_FLAG_BATT_CHEMISTRY_PRESENT )
            {
                STREAM_TO_UINT8( batt_chemistry, p_data->data.p_data )
            }
            if ( flags16 & BATTERY_INFO_FLAG_NOMINAL_VOLTAGE_PRESENT )
            {
                STREAM_TO_UINT16( batt_nominal_volage, p_data->data.p_data );
            }
            if ( flags16 & BATTERY_INFO_FLAG_BATT_AGGR_GRP_PRESENT )
            {
                STREAM_TO_UINT8( batt_aggr_grp, p_data->data.p_data )
            }

            WICED_BT_TRACE("BATTERY INFO:[%5s] flags16:%d battery_features:%02X battery_replace:%d battery_recharge:%d batt_features_rfu:%d \n",
                            msg, flags16, features, batt_features->batt_replaceable, batt_features->batt_rechargeable, batt_features->rfu);
            WICED_BT_TRACE("batt_manuf_date:%d batt_expr_date:%d  batt_designed_cap:%d batt_low_energy:%d batt_critical_energy:%d \n",
                            batt_manuf_date, batt_expr_date, batt_designed_cap, batt_low_energy, batt_critical_energy);
            WICED_BT_TRACE("batt_chemistry:%d batt_nominal_volage:%d batt_aggr_grp:%d \n\n",
                            batt_chemistry, batt_nominal_volage, batt_aggr_grp);
            break;
        }
    case UUID_CHARACTERISTIC_BATTERY_MANUFACTURE_NAME:
        p_data->data.p_data[p_data->data.len] = 0; // terminate string
        WICED_BT_TRACE("MANUFACTURE NAME [%13s]:%s \n\n",  msg, p_data->data.p_data );
        break;
    case UUID_CHARACTERISTIC_BATTERY_MANUFACTURE_NUMBER:
        p_data->data.p_data[p_data->data.len] = 0;
        WICED_BT_TRACE("MODEL NUMBER [%13s]:%s \n\n",  msg, p_data->data.p_data );
        break;
    case UUID_CHARACTERISTIC_BATTERY_SERIAL_NUMBER:
        p_data->data.p_data[p_data->data.len] = 0;
        WICED_BT_TRACE("SERIAL NUMBER [%13s]: %s \n\n",  msg, p_data->data.p_data );
        break;
#endif

    default:
        WICED_BT_TRACE("DEFAULT [%13s] Battery %s:", msg, wiced_bt_battery_client_uuid_to_str(p_data->data.uuid));
        battery_client_out_bytes("", p_data->data.len, p_data->data.p_data);
        break;
    }
}

static void battery_client_callback(wiced_bt_battery_client_event_t event, wiced_bt_battery_client_event_data_t *p_data)
{
    switch (event)
    {
    case WICED_BT_BAC_EVENT_DISCOVERY_COMPLETE:
//        BAC_TRACE_DBG("Discovery Complete conn_id:%d status:%d\n", p_data->discovery.conn_id, p_data->discovery.status);
        /* If Battery Service successfully discovered */
        if (p_data->discovery.status == WICED_BT_GATT_SUCCESS)
        {
            /* Enable Battery Level services */
            battery_client_enable( WICED_TRUE );

            /* Perform a BAS read all data */
            wiced_bt_battery_client_read(p_data->discovery.conn_id);
        }
        break;

    case WICED_BT_BAC_EVENT_RSP:
//        WICED_BT_TRACE("WICED_BT_BAC_EVENT_RSP\n");
        battery_client_show_data("Read Response", event, p_data);
        break;

    case WICED_BT_BAC_EVENT_NOTIFICATION:
//        WICED_BT_TRACE("WICED_BT_BAC_EVENT_NOTIFICATION\n");
        battery_client_show_data("Notification", event, p_data);
        break;

    case WICED_BT_BAC_EVENT_INDICATION:
//        WICED_BT_TRACE("WICED_BT_BAC_EVENT_INDICATION\n");
        battery_client_show_data("Indication", event, p_data);
        break;

    default:
        WICED_BT_TRACE("Unknown BAC Event:%d\n", event);
        break;
    }
}

/*
 * GATT discovery has been completed
 */
wiced_bt_gatt_status_t battery_client_gatt_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data)
{
    wiced_result_t result;

//    WICED_BT_TRACE("[%s] conn %d, discovery type %d, state %d\n", __FUNCTION__, p_data->conn_id, p_data->discovery_type, battery_client_app_state.discovery_state);

    switch (battery_client_app_state.discovery_state)
    {
    case BAC_DISCOVERY_STATE_CHAR:
        wiced_bt_battery_client_discovery_complete(p_data);
        break;

    default:
        if (bac_gatt_discovery_complete_type(p_data) == GATT_DISCOVER_SERVICES_ALL)
        {
            WICED_BT_TRACE("bac handles:%04x-%04x\n", battery_client_app_state.bac_s_handle, battery_client_app_state.bac_e_handle);

            /* If bac Service found tell AIROC Bluetooth bac library to start its discovery */
            if ((battery_client_app_state.bac_s_handle != 0) && (battery_client_app_state.bac_e_handle != 0))
            {
                battery_client_app_state.discovery_state = BAC_DISCOVERY_STATE_CHAR;
                if (wiced_bt_battery_client_discover(battery_client_app_data.conn_id, battery_client_app_state.bac_s_handle, battery_client_app_state.bac_e_handle))
                    break;
            }
        }
        else
        {
            WICED_BT_TRACE("!!!! invalid op:%d\n", bac_gatt_discovery_complete_type(p_data));
        }
    }
    return WICED_BT_GATT_SUCCESS;
}

wiced_bt_gatt_status_t battery_client_gatt_discovery_result(wiced_bt_gatt_discovery_result_t *p_data)
{
//    WICED_BT_TRACE("[%s] conn %d, discovery type %d, state %d\n", __FUNCTION__, p_data->conn_id, p_data->discovery_type, battery_client_app_state.discovery_state);

    switch (battery_client_app_state.discovery_state)
    {
        case BAC_DISCOVERY_STATE_CHAR:
            wiced_bt_battery_client_discovery_result(p_data);
            break;

        default:
            if (p_data->discovery_type  == GATT_DISCOVER_SERVICES_ALL)
            {
                if (p_data->discovery_data.group_value.service_type.len == LEN_UUID_16 )
                {
     //                 WICED_BT_TRACE("uuid:%04x start_handle:%04x end_handle:%04x\n",
     //                       p_data->discovery_data.group_value.service_type.uu.uuid16,
     //                        p_data->discovery_data.group_value.s_handle,
     //                        p_data->discovery_data.group_value.e_handle);

                   if( (p_data->discovery_data.group_value.service_type.uu.uuid16 == UUID_SERVICE_BATTERY)  && (battery_service_found != 1) )
                   {
                       battery_service_found = 1; // ignore any more battery services, connect to first discovered one
  //                     WICED_BT_TRACE("Battery Service found s:%04x e:%04x\n",
  //                             p_data->discovery_data.group_value.s_handle,
  //                             p_data->discovery_data.group_value.e_handle);
                        battery_client_app_state.bac_s_handle = p_data->discovery_data.group_value.s_handle;
                        battery_client_app_state.bac_e_handle = p_data->discovery_data.group_value.e_handle;
                    }
                }
            }
            else
            {
                WICED_BT_TRACE("!!!! invalid op:%d\n", p_data->discovery_type);
            }
    }
    return WICED_BT_GATT_SUCCESS;
}

void battery_client_add_peer_info( uint16_t conn_id, uint8_t* p_bd_addr, uint8_t role , uint8_t transport, uint8_t address_type )
{
    battery_client_app_data.addr_type = address_type;
    battery_client_app_data.conn_id = conn_id;
    memcpy(battery_client_app_data.peer_addr,p_bd_addr,BD_ADDR_LEN);
    battery_client_app_data.role = role;
    battery_client_app_data.transport = transport;
}

static wiced_bt_gatt_status_t battery_client_connection_up( wiced_bt_gatt_connection_status_t *p_conn_status )
{
    uint8_t dev_role;
    wiced_bt_dev_status_t status ;
    wiced_bt_ble_sec_action_type_t  encryption_type = BTM_BLE_SEC_ENCRYPT;
    battery_service_found = 0;

    wiced_bt_dev_get_role( p_conn_status->bd_addr, &dev_role, BT_TRANSPORT_LE );

    // Adding the peer info
    battery_client_add_peer_info( p_conn_status->conn_id, p_conn_status->bd_addr, dev_role , p_conn_status->transport, p_conn_status->addr_type );

    WICED_BT_TRACE( "battery client_connection_up Conn Id:%d Addr:<%B> role:%d\n",
       p_conn_status->conn_id, p_conn_status->bd_addr, dev_role );

    // need to notify BAS library that the connection is up
    wiced_bt_battery_client_connection_up( p_conn_status );

        /* Initialize AIROC Bluetooth bac library Start discovery */
    battery_client_app_state.discovery_state = BAC_DISCOVERY_STATE_SERVICE;
    battery_client_app_state.bac_s_handle = 0;
    battery_client_app_state.bac_e_handle = 0;

#ifdef TEST_HCI_CONTROL
    battery_client_hci_send_connect_event( p_conn_status->addr_type, p_conn_status->bd_addr, p_conn_status->conn_id, dev_role );
#endif

    // perform primary service search
    status = wiced_bt_util_send_gatt_discover( p_conn_status->conn_id, GATT_DISCOVER_SERVICES_ALL, UUID_ATTRIBUTE_PRIMARY_SERVICE, 1, 0xffff);
    WICED_BT_TRACE("start discover status:%d\n", status);

    return WICED_BT_GATT_SUCCESS;
}

static wiced_bt_gatt_status_t battery_client_connection_down( wiced_bt_gatt_connection_status_t *p_conn_status )
{
    WICED_BT_TRACE("battery client connection down\n");
    battery_client_app_data.conn_id = 0;

    battery_client_app_state.discovery_state = BAC_DISCOVERY_STATE_SERVICE;
    battery_client_app_state.bac_s_handle = 0;
    battery_client_app_state.bac_e_handle = 0;

#ifdef TEST_HCI_CONTROL
    battery_client_hci_send_disconnect_event( p_conn_status->reason, p_conn_status->conn_id );
#endif

    wiced_bt_battery_client_connection_down( p_conn_status );

    return WICED_BT_GATT_SUCCESS;
}

/* Check for device entry exists in NVRAM list */
static wiced_bool_t battery_client_is_device_bonded(wiced_bt_device_address_t bd_address)
{
    wiced_bt_device_link_keys_t temp_keys;
    uint8_t                     bytes_read;
    uint16_t                    i;
    wiced_result_t              result;

    // search through all available NVRAM IDs.
    for ( i = BATTERY_CLIENT_PAIRED_KEYS_START_VS_ID; i < WICED_NVRAM_VSID_END; i++ )
    {
        bytes_read = wiced_hal_read_nvram( i, sizeof( temp_keys ), (uint8_t *)&temp_keys, &result );

        WICED_BT_TRACE("[%s] read status %d bytes read %d \n", __FUNCTION__, result, bytes_read);

        // if failed to read NVRAM, there is nothing saved at that location
        if ( result == WICED_SUCCESS )
        {
            if ( memcmp( temp_keys.bd_addr, bd_address, BD_ADDR_LEN ) == 0 )
            {
                return WICED_TRUE;
            }
        }
        else
        {
            break;
        }
    }
    return WICED_FALSE;
}

/*
 * GATT operation started by the client has been completed
 */
wiced_bt_gatt_status_t battery_client_gatt_operation_complete(wiced_bt_gatt_operation_complete_t *p_data)
{
    wiced_result_t              status;
    wiced_bt_ble_sec_action_type_t  encryption_type = BTM_BLE_SEC_ENCRYPT;

//    WICED_BT_TRACE("battery_client_gatt_operation_complete conn %d op %d st %d\n", p_data->conn_id, p_data->op, p_data->status );

    battery_client_handle_op_complete(p_data);

    /* server puts authentication requirement. Encrypt the link */
    if ( p_data->status == WICED_BT_GATT_INSUF_AUTHENTICATION )
    {
        WICED_BT_TRACE("Insufficient Authentication\n");
        if ( battery_client_is_device_bonded(battery_client_app_data.peer_addr) )
        {
            WICED_BT_TRACE("Authentified. Start Encryption\n");
            status = wiced_bt_dev_set_encryption( battery_client_app_data.peer_addr,
                    battery_client_app_data.transport, &encryption_type );
            WICED_BT_TRACE( "wiced_bt_dev_set_encryption %d \n", status );
        }
        else
        {
            WICED_BT_TRACE("Start Authentification/Pairing\n");
            status = wiced_bt_dev_sec_bond( battery_client_app_data.peer_addr,
                    battery_client_app_data.addr_type, battery_client_app_data.transport,0, NULL );
            WICED_BT_TRACE( "wiced_bt_dev_sec_bond %d \n", status );
        }
    }

    return WICED_BT_GATT_SUCCESS;
}


/*
 *  Pass protocol traces up through the UART
 */
#ifdef ENABLE_HCI_TRACE
static void battery_client_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
#if NEW_DYNAMIC_MEMORY_INCLUDED
    wiced_transport_send_hci_trace( type, p_data, length );
#else
    wiced_transport_send_hci_trace( NULL, type, length, p_data );
#endif
}
#endif

static void battery_client_load_keys_to_addr_resolution_db()
{
    uint8_t                     bytes_read;
    wiced_result_t              result;
    wiced_bt_device_link_keys_t keys;
    uint16_t                    i;

    for ( i = BATTERY_CLIENT_PAIRED_KEYS_START_VS_ID; i < WICED_NVRAM_VSID_END; i++ )
    {
        bytes_read = wiced_hal_read_nvram(i, sizeof(keys), (uint8_t *)&keys, &result);

//        WICED_BT_TRACE("[%s] read status %d bytes read %d \n", __FUNCTION__, result, bytes_read);

        // if failed to read NVRAM, there is nothing saved at that location
        if ((result == WICED_SUCCESS) && (bytes_read == sizeof(wiced_bt_device_link_keys_t)))
        {
#ifdef CYW20706A2
            result = wiced_bt_dev_add_device_to_address_resolution_db(&keys, keys.key_data.ble_addr_type);
#else
            result = wiced_bt_dev_add_device_to_address_resolution_db(&keys);
#endif
        }
        else
        {
            break;
        }
    }
}

static void battery_client_app_init()
{
    wiced_bt_gatt_status_t gatt_status;

#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW43012C0) )
    /* Initialize wiced app */
    wiced_bt_app_init();
#endif

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(battery_client_gatts_callback);

    WICED_BT_TRACE("wiced_bt_gatt_register: %d\n", gatt_status);

#ifdef ENABLE_HCI_TRACE
    /* Register callback for receiving hci traces */
    wiced_bt_dev_register_hci_trace( battery_client_hci_trace_cback );
#endif

    /* Load the address resolution DB with the keys stored in the NVRAM */
    battery_client_load_keys_to_addr_resolution_db();

#ifdef BATTERY_LEVEL_BROADCAST
    wicked_bt_battery_client_server_characterstic_handle = 0;
#endif
    battery_client_app_data.conn_id = 0;

#if defined(CYW43022C1)
    /* Configure LED1 on the platform */
    // Set WICED_BT_GPIO_03 (J4, pin 7) as LED1 (J11, pin 2) connected to
    wiced_hal_gpio_select_function(WICED_GPIO_PIN_LED_1, WICED_GPIO);
    wiced_hal_gpio_configure_pin(WICED_GPIO_PIN_LED_1, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH );
#endif

}

/*
 * This function handles the scan results and attempt to connect to Battery Service Server.
 */
static void battery_client_scan_result_cback( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data )
{
    wiced_result_t          status;
    wiced_bool_t            ret_status;
    uint8_t                 length = 0;
    uint8_t *               p_data;
    uint16_t                service_uuid16=0;

    if ( p_scan_result )
    {

#ifdef BATTERY_LEVEL_BROADCAST
        // check to see if the advertisement is battery level broadcast
        p_data = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_SERVICE_DATA, &length );
        if ( p_data != NULL )
        {
            WICED_BT_TRACE( "Broadcast advertisement with length %d\n", length);

            uint8_t len = 0, battery_level = 0, rfu = 0;
            uint16_t flags = 0, identifier = 0;

            STREAM_TO_UINT16( service_uuid16, p_data );
            STREAM_TO_UINT8( len, p_data );

            if ( (service_uuid16 == UUID_SERVICE_BATTERY) /*&& (len == 7) */) // len = uuid(2) + battry level(1)
            {
                STREAM_TO_UINT16( flags, p_data );
                STREAM_TO_UINT16( identifier, p_data );

                STREAM_TO_UINT8( battery_level, p_data );
                STREAM_TO_UINT8( rfu, p_data );
                WICED_BT_TRACE( "Broadcast advertisement with len:%d flags:%02X identifier:%02X level:%d rfu:%d \n",
                                 len, flags, identifier, battery_level, rfu );
            }
            return;     // safe to return from here as the broadcast adv packet not used for connection purpose

        }
#endif
        if ( battery_client_app_data.conn_id )
        {
            // return if connected to a server already
            return;
        }

        // Search for SERVICE_UUID_16 element in the Advertisement data received.Check for both
        // complete and partial list
        p_data = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_16SRV_COMPLETE, &length );
        if ( p_data == NULL )
        {
            p_data = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_16SRV_PARTIAL, &length );

            if (p_data == NULL){
                return;     // No UUID_16 element
            }
        }

        while (length >= LEN_UUID_16)
        {
            STREAM_TO_UINT16(service_uuid16, p_data);
            if (service_uuid16 == UUID_SERVICE_BATTERY)
            {
                // UUID16 Battery Service found
                break;
            }
            length -= LEN_UUID_16;
        }

        if (service_uuid16 != UUID_SERVICE_BATTERY)
        {
            // UUID16 Battery Service not found. Ignore device
            return;
        }

        WICED_BT_TRACE("Battery Server Device found: %B addr_type:%d\n",
                p_scan_result->remote_bd_addr, p_scan_result->ble_addr_type);

        /* Stop the scan since the desired device is found */
        status = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_NONE, WICED_TRUE, battery_client_scan_result_cback );
        WICED_BT_TRACE( "scan off status %d\n", status );

        /* Initiate the connection */
        ret_status = wiced_bt_gatt_le_connect( p_scan_result->remote_bd_addr, p_scan_result->ble_addr_type, BLE_CONN_MODE_HIGH_DUTY, TRUE );
        WICED_BT_TRACE( "wiced_bt_gatt_connect status %d\n", ret_status );
    }
    else
    {
        //WICED_BT_TRACE( "Scan completed:\n" );
    }
}

static void battery_client_enter_pairing()
{
    wiced_result_t result;

    WICED_BT_TRACE("Entering BAS discovery\n");
    /*start scan if not connected and no scan in progress*/
    if (( battery_client_app_data.conn_id == 0 ) && (wiced_bt_ble_get_current_scan_state() == BTM_BLE_SCAN_TYPE_NONE))
    {
        result = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE, battery_client_scan_result_cback );
        WICED_BT_TRACE("wiced_bt_ble_scan: %d \n", result);
    }
    else
    {
         WICED_BT_TRACE("Scanning not initiated, scan in progress ??  \n");
    }
}

static void battery_client_interrupt_handler( void *user_data, uint8_t value )
{
    wiced_result_t result;
    wiced_bt_gatt_status_t status;
    int button_down = 0;

#if defined(CYW20706A2)
    button_down = wiced_hal_gpio_get_pin_input_status(APP_BUTTON) == BUTTON_PRESSED;
#else
#ifdef CYW43022C1
    button_down = wiced_hal_gpio_get_pin_input_status(WICED_GPIO_04) != BUTTON_PRESSED;
#else
    button_down = wiced_hal_gpio_get_pin_input_status( WICED_GET_PIN_FOR_BUTTON(WICED_PLATFORM_BUTTON_1) ) == wiced_platform_get_button_pressed_value(WICED_PLATFORM_BUTTON_1);
#endif
#endif

    if ( ( button_pushed_time == APP_TIMER_IDLE ) && button_down )
    {
        WICED_BT_TRACE("User button down\n"
                       "Release now to start pairing.\n");
        button_pushed_time = bac_app_timer_count;
    }
    else if ( button_pushed_time != APP_TIMER_IDLE )
    {
        uint32_t duration = bac_app_timer_count - button_pushed_time;
        WICED_BT_TRACE("User button released, duration=%d\n", duration);

#ifdef BATTERY_LEVEL_BROADCAST
        if(duration >= 8)
        {
            WICED_BT_TRACE("BROADCAST toggle Button\n");
            // Toggle broadcast flag on the sever
            is_broadcast_enabled = ( is_broadcast_enabled == WICED_TRUE ) ? WICED_FALSE : WICED_TRUE;
            battery_client_gatts_enable_broadcast ( battery_client_app_data.conn_id, wicked_bt_battery_client_server_characterstic_handle, is_broadcast_enabled );
        }
        else
#endif
        if ( duration >= 5 )
        {
            WICED_BT_TRACE("BAC %s battery level notifications.", is_bas_enabled ? "Disabling" : "Enabling\n");
            battery_client_enable(!is_bas_enabled);
        }
        else if(duration >= 3)
        {
            WICED_BT_TRACE("BAC Read battery level.\n");
            wiced_bt_battery_client_read( battery_client_app_data.conn_id );
        }
        else if( ! battery_client_app_data.conn_id )
        {
            WICED_BT_TRACE("BAC Enter pairing mode.\n");
            battery_client_enter_pairing();
        }
        button_pushed_time = APP_TIMER_IDLE;
    }
}

/* The function invoked on timeout of app seconds timer. */
static void battery_client_app_timer( TIMER_PARAM_TYPE arg )
{
    bac_app_timer_count++;

    // if user button is pressed
    if (button_pushed_time)
    {
        uint32_t duration = bac_app_timer_count - button_pushed_time;

        switch (duration)
        {
        case 3:
            WICED_BT_TRACE("Release now to read battery level.\n");
            break;
        case 5:
            WICED_BT_TRACE("Release now to %s battery level notifications.\n", is_bas_enabled ? "disable" : "enable");
            break;
#ifdef BATTERY_LEVEL_BROADCAST
        case 8:
            WICED_BT_TRACE("Release now to %s broadcast\n", (is_broadcast_enabled) ? "enable" : "disable" );
            break;
#endif
        }
    }

#ifdef BATTERY_LEVEL_BROADCAST
    /* Connected or not, scan for battery level advertisements, if not already scanning  */
    if ( wiced_bt_ble_get_current_scan_state() == BTM_BLE_SCAN_TYPE_NONE  )
    {
        //wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE, battery_client_scan_result_cback );
        wiced_bt_ble_observe (WICED_TRUE, 3, battery_client_scan_result_cback);
        //WICED_BT_TRACE("Scanning for battery level advertisements %d\n", bac_app_timer_count);
    }
#endif

}

static void battery_client_set_input_interrupt(void)
{
#if defined(CYW20706A2)
    wiced_hal_gpio_register_pin_for_interrupt( APP_BUTTON, battery_client_interrupt_handler, NULL );
    wiced_hal_gpio_configure_pin( APP_BUTTON, APP_BUTTON_SETTINGS, APP_BUTTON_DEFAULT_STATE );
#else
#if defined(CYW43022C1)
    /* Configure buttons available on the platform */
    // Set WICED_BT_GPIO_04 (J4, pin SDA) as User button (J12, pin 1) connected to
    wiced_hal_gpio_select_function(WICED_GPIO_PIN_BUTTON_1, WICED_GPIO);
    wiced_hal_gpio_configure_pin(WICED_GPIO_PIN_BUTTON_1, GPIO_INPUT_ENABLE | WICED_GPIO_EN_INT_BOTH_EDGE, 1);
    wiced_hal_gpio_register_pin_for_interrupt(WICED_GPIO_PIN_BUTTON_1, battery_client_interrupt_handler, NULL);
#else
    /* Configure buttons available on the platform */
    wiced_platform_register_button_callback( WICED_PLATFORM_BUTTON_1, battery_client_interrupt_handler, NULL, WICED_PLATFORM_BUTTON_BOTH_EDGE);
#endif
#endif

    /* Starting the app timer */
    wiced_init_timer(&bac_app_timer, battery_client_app_timer, 0, WICED_SECONDS_PERIODIC_TIMER);
    wiced_start_timer(&bac_app_timer,BATTC_APP_TIMEOUT_IN_SECONDS);
    WICED_BT_TRACE( "App timer started\n" );
}

static wiced_bool_t battery_client_save_link_keys( wiced_bt_device_link_keys_t *p_keys )
{
    uint8_t                     bytes_written, bytes_read;
    wiced_bt_device_link_keys_t temp_keys;
    uint16_t                    id = 0;
    uint32_t i;
    wiced_result_t              result;

    // search through all available NVRAM IDs.
    for ( i = BATTERY_CLIENT_PAIRED_KEYS_START_VS_ID; i < WICED_NVRAM_VSID_END; i++ )
    {
        bytes_read = wiced_hal_read_nvram( i, sizeof( temp_keys ), (uint8_t *)&temp_keys, &result );

        WICED_BT_TRACE( "Read NVRAM at:%d bytes:%d result:%d\n", i, bytes_read, result );

        // if failed to read NVRAM, there is nothing saved at that location
        if ( ( result != WICED_SUCCESS ) || ( bytes_read != sizeof( temp_keys ) ) )
        {
            id = i;
            break;
        }
        else
        {
            if ( memcmp( temp_keys.bd_addr, p_keys->bd_addr, BD_ADDR_LEN ) == 0 )
            {
                // keys for this device have been saved, reuse the ID
                id = i;
                break;
            }
        }
    }
    if ( id == 0 )
    {
        // all NVRAM locations are already occupied.  Cann't save anything.
        WICED_BT_TRACE( "Failed to save NVRAM\n" );
        return WICED_FALSE;
    }
    WICED_BT_TRACE( "writing to id:%d\n", id );
    bytes_written = wiced_hal_write_nvram( id, sizeof( wiced_bt_device_link_keys_t ), (uint8_t *)p_keys, &result );
    WICED_BT_TRACE( "Saved %d bytes at id:%d %d\n", bytes_written, id );
    return WICED_TRUE;
}

static wiced_bool_t battery_client_read_link_keys(wiced_bt_device_link_keys_t *p_keys)
{
    wiced_bt_device_link_keys_t temp_keys;
    uint8_t                     bytes_read;
    uint16_t                    i;
    wiced_result_t              result;

    // search through all available NVRAM IDs.
    for ( i = BATTERY_CLIENT_PAIRED_KEYS_START_VS_ID; i < WICED_NVRAM_VSID_END; i++ )
    {
        bytes_read = wiced_hal_read_nvram( i, sizeof( temp_keys ), (uint8_t *)&temp_keys, &result );

        WICED_BT_TRACE("[%s] read status %d bytes read %d \n", __FUNCTION__, result, bytes_read);

        // if failed to read NVRAM, there is nothing saved at that location
        if ( result == WICED_SUCCESS )
        {
            if ( memcmp( temp_keys.bd_addr, p_keys->bd_addr, BD_ADDR_LEN ) == 0 )
            {
                // keys for this device have been saved
                memcpy( &p_keys->key_data, &temp_keys.key_data, sizeof( temp_keys.key_data ) );
                return WICED_TRUE;
            }
        }
        else
        {
            break;
        }
    }
    return WICED_FALSE;
}

/*
 * link status change event
 */
void battery_client_connection_evt( wiced_bt_gatt_event_data_t * p_data )
{
    if (p_data->connection_status.connected)
    {
        battery_client_connection_up(&p_data->connection_status);
    }
    else
    {
        battery_client_connection_down(&p_data->connection_status);
    }
}

/*
 * Pass read response to appropriate client based on the attribute handle
 */
void battery_client_process_read_rsp(wiced_bt_gatt_operation_complete_t *p_data)
{
//    WICED_BT_TRACE("read response handle:%04x s:%04x e:%04x\n", p_data->response_data.handle, battery_client_app_state.bac_s_handle, battery_client_app_state.bac_e_handle);

    // Verify that read response is for our service
    if ((p_data->response_data.handle >= battery_client_app_state.bac_s_handle) &&
        (p_data->response_data.handle <= battery_client_app_state.bac_e_handle))
    {
        wiced_bt_battery_client_read_rsp(p_data);
    }
}

/*
 * Pass notification to appropriate client based on the attribute handle
 */
void battery_client_indication_handler(wiced_bt_gatt_operation_complete_t *p_data)
{
//    WICED_BT_TRACE("indication handle:%04x\n", p_data->response_data.att_value.handle);

    if ((p_data->response_data.handle >= battery_client_app_state.bac_s_handle) &&
        (p_data->response_data.handle <= battery_client_app_state.bac_e_handle))
    {
        wiced_bt_battery_client_process_indication(p_data);
    }
}

/*
 * Pass notification to appropriate client based on the attribute handle
 */
void battery_client_notification_handler(wiced_bt_gatt_operation_complete_t *p_data)
{
//    WICED_BT_TRACE("notification handle:%04x\n", p_data->response_data.att_value.handle);

    if ((p_data->response_data.handle >= battery_client_app_state.bac_s_handle) &&
        (p_data->response_data.handle <= battery_client_app_state.bac_e_handle))
    {
        wiced_bt_battery_client_process_notification(p_data);
    }
}

wiced_result_t battery_client_management_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t                    result = WICED_BT_SUCCESS;
    wiced_bt_dev_encryption_status_t *p_status;
    wiced_bt_dev_ble_pairing_info_t  *p_info;
    wiced_bt_ble_advert_mode_t       *p_mode;
    wiced_bt_device_link_keys_t       paired_device_link_keys_request;
    uint8_t                           bytes_written, bytes_read;
    uint8_t                          *p_keys;

    WICED_BT_TRACE("battery_client_management_cback:%d\n", event);

    switch(event)
    {
    /* Bluetooth stack enabled */
    case BTM_ENABLED_EVT:
        battery_client_app_init();
        battery_client_set_input_interrupt();
        break;

    case BTM_DISABLED_EVT:
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap  = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_ble_request.oob_data      = BTM_OOB_NONE;
        p_event_data->pairing_io_capabilities_ble_request.auth_req      = BTM_LE_AUTH_REQ_SC_BOND;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 0x10;
        p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
        break;

    case BTM_PAIRING_COMPLETE_EVT:
        WICED_BT_TRACE("Pairing Complete: %d\n", p_event_data->pairing_complete.pairing_complete_info.ble.reason);
        break;

    case BTM_ENCRYPTION_STATUS_EVT:
        WICED_BT_TRACE("Encryption Status Event: bd (%B) res %d\n", p_event_data->encryption_status.bd_addr, p_event_data->encryption_status.result);
        break;

    case BTM_SECURITY_REQUEST_EVT:
        wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        battery_client_save_link_keys(&p_event_data->paired_device_link_keys_update);
        break;

    case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        if (battery_client_read_link_keys(&p_event_data->paired_device_link_keys_request))
        {
            result = WICED_BT_SUCCESS;
            WICED_BT_TRACE("Key retrieval success\n");
        }
        else
        {
            result = WICED_BT_ERROR;
            WICED_BT_TRACE("Key retrieval failure\n");
        }
        break;

    case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
        /* save keys to NVRAM */
        p_keys = (uint8_t*)&p_event_data->local_identity_keys_update;
        wiced_hal_write_nvram ( BATTERY_CLIENT_LOCAL_KEYS_VS_ID, sizeof( wiced_bt_local_identity_keys_t ), p_keys ,&result );
        WICED_BT_TRACE("local keys save to NVRAM result: %d \n", result);
        break;


    case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
        /* read keys from NVRAM */
        p_keys = (uint8_t *)&p_event_data->local_identity_keys_request;
        wiced_hal_read_nvram( BATTERY_CLIENT_LOCAL_KEYS_VS_ID, sizeof(wiced_bt_local_identity_keys_t), p_keys, &result );
        WICED_BT_TRACE("local keys read from NVRAM result: %d \n",  result);
        break;

    case BTM_BLE_SCAN_STATE_CHANGED_EVT:
        WICED_BT_TRACE( "Scan State Change: %d\n", p_event_data->ble_scan_state_changed );
        break;

    default:
        break;
    }
    return result;
}

/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that Bluetooth device is ready.
 */
APPLICATION_START( )
{
    wiced_transport_init( &transport_cfg );

#ifdef WICED_BT_TRACE_ENABLE
    // Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_NONE);

#if defined(CYW43022C1)
    // 43022 transport clock rate is 24Mhz for baud rates <= 1.5 Mbit/sec, and 48Mhz for baud rates > 1.5 Mbit/sec.
    // HCI UART and Debug UART both use the Transport clock, so if the HCI UART rate is <= 1.5 Mbps, please do not set the Debug UART > 1.5 Mbps.
    // The default Debug UART baud rate is 115200, and default HCI UART baud rate is 3Mbps
    debug_uart_set_baudrate(115200);

    // CYW943022M2BTBGA doesn't have GPIO pin for PUART.
    // CYW943022M2BTBGA Debug UART Tx (WICED_GPIO_02) is connected to UART2_RX (ARD_D1) on PUART Level Translators of CYW9BTM2BASE1.
    // We need to set to WICED_ROUTE_DEBUG_TO_DBG_UART to see traces on PUART of CYW9BTM2BASE1.
    // wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_WICED_UART );
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_DBG_UART );
#else
    // Set to PUART to see traces on peripheral uart(puart)
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );

#if ( defined(CYW20706A2) || defined(CYW43012C0) )
    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);
#endif

    // Set to HCI to see traces on HCI uart - default if no call to wiced_set_debug_uart()
    // wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_HCI_UART );

#ifdef ENABLE_HCI_TRACE
    // Use WICED_ROUTE_DEBUG_TO_WICED_UART to send formatted debug strings over the WICED
    // HCI debug interface to be parsed by ClientControl/BtSpy.
    // Note: WICED HCI must be configured to use this - see wiced_trasnport_init(), must
    // be called with wiced_transport_cfg_t.wiced_trbasort_data_handler_t callback present
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif
#endif  //CYW43022C1
#endif  //WICED_BT_TRACE_ENABLE

#ifdef BAS_1_1
    WICED_BT_TRACE( "\nBattery Service Client v1.1 Start\n" );
#else
    WICED_BT_TRACE( "\nBattery Service Client v1.0 Start\n" );
#endif

    wiced_bt_battery_client_init(battery_client_callback);

    memset(&battery_client_app_data, 0, sizeof(battery_client_app_data));
    memset(&battery_client_app_state, 0, sizeof(battery_client_app_state));

    // Register call back and configuration with stack
    app_stack_init();
}



#ifdef TEST_HCI_CONTROL
/*
 *  transfer connection event to uart
 */
void battery_client_hci_send_connect_event( uint8_t addr_type, BD_ADDR addr, uint16_t con_handle, uint8_t role )
{

    int i;
    uint8_t   tx_buf [30];
    uint8_t   *p = tx_buf;

    *p++ = addr_type;
    for ( i = 0; i < 6; i++ )
        *p++ = addr[5 - i];
    *p++ = con_handle & 0xff;
    *p++ = ( con_handle >> 8 ) & 0xff;
    *p++ = role;

    wiced_transport_send_data ( HCI_CONTROL_BATT_CLIENT_EVENT_CONNECTED, tx_buf, ( int )( p - tx_buf ) );
}


void battery_client_hci_handle_get_version(void)
{
    uint8_t   tx_buf[20];
    uint8_t   cmd = 0;

// If this is 20819 or 20820, we do detect the device from hardware
#define RADIO_ID    0x006007c0
#define RADIO_20820 0x80
#define CHIP_20820  20820
#define CHIP_20819  20819
#if (CHIP==CHIP_20819) || (CHIP==CHIP_20820)
    uint32_t chip = CHIP_20819;
    if (*(UINT32*) RADIO_ID & RADIO_20820)
    {
        chip = CHIP_20820;
    }
#else
    uint32_t  chip = CHIP;
#endif

    tx_buf[cmd++] = WICED_SDK_MAJOR_VER;
    tx_buf[cmd++] = WICED_SDK_MINOR_VER;
    tx_buf[cmd++] = WICED_SDK_REV_NUMBER;
    tx_buf[cmd++] = WICED_SDK_BUILD_NUMBER & 0xFF;
    tx_buf[cmd++] = (WICED_SDK_BUILD_NUMBER>>8) & 0xFF;
    tx_buf[cmd++] = chip & 0xFF;
    tx_buf[cmd++] = (chip>>8) & 0xFF;
    tx_buf[cmd++] = (chip>>24) & 0xFF;
    tx_buf[cmd++] = 0; // not used

    uint8_t  addr[]         = {0x0A, 0X0B, 0xC, 0x0D, 0x0E, 0X0F};

    /* Send MCU app the supported features */
    tx_buf[cmd++] = HCI_CONTROL_GROUP_BATT_CLIENT;

    wiced_transport_send_data(HCI_CONTROL_MISC_EVENT_VERSION, tx_buf, cmd);

   if ( battery_client_app_data.conn_id != 0 )
   {
        // if connected already, simply send connection details. else one shall be sent when connected
        battery_client_hci_send_connect_event( 1, addr, battery_client_app_data.conn_id, 1 );
    }

}

/* transport status */
void battery_client_hci_transport_status( wiced_transport_type_t type )
{
    WICED_BT_TRACE("battery_client_transport connected type: %d ", type);
}

/*
 *  transfer disconnection event to UART
 */
void battery_client_hci_send_disconnect_event( uint8_t reason, uint16_t con_handle )
{
    uint8_t   tx_buf [3];
    uint8_t   *p = tx_buf;

    *p++ = con_handle & 0xff;
    *p++ = ( con_handle >> 8 ) & 0xff;
    *p++ = reason;

    wiced_transport_send_data ( HCI_CONTROL_BATT_CLIENT_EVENT_DISCONNECTED, tx_buf, ( int )( p - tx_buf ) );
}

#ifdef BAS_1_1
static wiced_bt_gatt_status_t  battery_client_hci_handle_broadcast_modify( wiced_bool_t enable )
{
    WICED_BT_TRACE("HCI Control broadcast toggle cmd\n");

    if( battery_client_app_data.conn_id )
    {
        battery_client_gatts_enable_broadcast ( battery_client_app_data.conn_id, wicked_bt_battery_client_server_characterstic_handle, enable );
        is_broadcast_enabled = enable;
    }
    else
    {
        WICED_BT_TRACE("Not connected to the battery server to enable/disable broadcasts \n");
    }

    return WICED_BT_GATT_SUCCESS;
}
#endif

uint32_t  battery_client_hci_rx_cmd( uint8_t *p_buffer, uint32_t length )
{
    uint16_t                opcode;
    uint8_t*                p_data = p_buffer;
    uint16_t                payload_len;
    uint8_t                 status = HCI_CONTROL_STATUS_SUCCESS;
    wiced_bt_gatt_status_t  gatt_status = WICED_BT_GATT_SUCCESS;
    uint8_t  addr[]         = {0x0A, 0X0B, 0xC, 0x0D, 0x0E, 0X0F};

    WICED_BT_TRACE("hci_control_proc_rx_cmd:%d\n", length);

    if ( !p_data )
    {
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    //Expected minimum 4 byte as the wiced header
    if (length < 4)
    {
        WICED_BT_TRACE( "invalid params\n" );
        wiced_transport_free_buffer( p_data );
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }
    else
    {

        STREAM_TO_UINT16(opcode, p_data);  // Get OpCod
        STREAM_TO_UINT16(payload_len, p_data);  // Gen Payload Length
        WICED_BT_TRACE("cmd_opcode 0x%02x payload_len %d \n", opcode, payload_len);

        switch ( opcode )
        {

            case HCI_CONTROL_BATT_CLIENT_COMMAND_CONNECT:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_CONNECT\n");

                if ( battery_client_app_data.conn_id != 0 )
                {
                    // if connected already, simply send connection details. else one shall be sent when connected
                    battery_client_hci_send_connect_event( 1, addr, battery_client_app_data.conn_id, 1 );
                }
                break;

            case HCI_CONTROL_BATT_CLIENT_COMMAND_DISCONNECT:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_DISCONNECT\n");
                break;
#ifdef BAS_1_1
            case HCI_CONTROL_BATT_CLIENT_COMMAND_ENABLE_BROADCAST:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_ENABLE_BROADCAST\n");
                battery_client_hci_handle_broadcast_modify( WICED_TRUE );
                break;

            case HCI_CONTROL_BATT_CLIENT_COMMAND_DISABLE_BROADCAST:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_DISABLE_BROADCAST\n");
                battery_client_hci_handle_broadcast_modify( WICED_FALSE );
                break;
#endif
            case HCI_CONTROL_BATT_CLIENT_COMMAND_BAS_READ:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_BAS_READ\n");
                wiced_bt_battery_client_read(battery_client_app_data.conn_id);
                break;

            case HCI_CONTROL_MISC_COMMAND_GET_VERSION:
             WICED_BT_TRACE("HCI_CONTROL_MISC_COMMAND_GET_VERSION\n");
                battery_client_hci_handle_get_version();
                break;

            default:
                WICED_BT_TRACE("ignored:%02X payload_len:%d \n", opcode, payload_len);
                status = HCI_CONTROL_STATUS_UNKNOWN_COMMAND;
                break;
            }
    }

    wiced_transport_send_data(HCI_CONTROL_BATT_CLIENT_EVENT_STATUS, &status, 1);

    // Freeing the buffer in which data is received
    wiced_transport_free_buffer( p_buffer );
    return HCI_CONTROL_STATUS_SUCCESS;
}

#endif
