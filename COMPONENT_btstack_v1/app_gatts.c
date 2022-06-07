/*
 * Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
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

/* This file is gatts functions for all devices with BTSTACK version lower than 3.0, i.e. 20xxx and 43012C0 */

#include "wiced_bt_gatt.h"
#include "wiced_bt_trace.h"
#include "wiced_memory.h"
#include "battery_client.h"

/*
 * GATT operation started by the client has been completed
 */
void battery_client_handle_op_complete(wiced_bt_gatt_operation_complete_t *p_data)
{
    switch ( p_data->op )
    {
        case GATTC_OPTYPE_READ:
//            WICED_BT_TRACE( "read_rsp status:%d\n", p_data->status );
            battery_client_process_read_rsp(p_data);
            break;

        case GATTC_OPTYPE_WRITE:
//            WICED_BT_TRACE( "write_rsp status:%d desc_handle:%x \n", p_data->status,p_data->response_data.handle );
            break;

        case GATTC_OPTYPE_CONFIG:
            WICED_BT_TRACE( "peer mtu:%d\n", p_data->response_data.mtu );
            break;

        case GATTC_OPTYPE_NOTIFICATION:
//            WICED_BT_TRACE( "notification status:%d\n", p_data->status );
            battery_client_notification_handler( p_data );
            break;

        case GATTC_OPTYPE_INDICATION:
//            WICED_BT_TRACE( "indication status:%d\n", p_data->status );
            battery_client_indication_handler( p_data );
            break;
    }
}

wiced_bt_gatt_status_t battery_client_gatts_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch(event)
    {
        case GATT_CONNECTION_STATUS_EVT:
            battery_client_connection_evt( p_data );
            break;

        case GATT_DISCOVERY_RESULT_EVT:
//            WICED_BT_TRACE("*** discovery result event\n");
            result = battery_client_gatt_discovery_result(&p_data->discovery_result);
            break;

        case GATT_DISCOVERY_CPLT_EVT:
//            WICED_BT_TRACE("*** discovery complete event\n");
            result = battery_client_gatt_discovery_complete(&p_data->discovery_complete);
            break;

        case GATT_OPERATION_CPLT_EVT:
//            WICED_BT_TRACE("*** operation complete event\n");
            result = battery_client_gatt_operation_complete(&p_data->operation_complete);
            break;

        default:
            break;
    }

    return result;
}

#ifdef BATTERY_LEVEL_BROADCAST
/*
 * This function writes into bas server character descriptor to enable broadcast of battery level status
 */
void battery_client_gatts_enable_broadcast ( uint16_t conn_id, uint16_t handle, uint8_t enable_broadcast )
{
    wiced_bt_gatt_status_t status;
    uint16_t  u16 = (enable_broadcast == WICED_TRUE) ? GATT_SERVER_CONFIG_BROADCAST : GATT_SERVER_CONFIG_NONE;

    if( handle == 0 )
    {
        // looks like a battery server with broadcast char was not
        // found during the gatt discovery, safe to return
        WICED_BT_TRACE("No battery server broadcast found \n");
        return;
    }

    // Allocating a buffer to send the write request
    wiced_bt_gatt_value_t *p_write = ( wiced_bt_gatt_value_t* )wiced_bt_get_buffer( sizeof( wiced_bt_gatt_value_t ) + 2 );

    if ( p_write )
    {
        uint8_t * value_p = &p_write->value[0];
        p_write->handle   = handle;
        p_write->offset   = 0;
        p_write->len      = 2;
        p_write->auth_req = GATT_AUTH_REQ_NONE;
        value_p[0] = u16 & 0xff;
        value_p[1] = (u16 >> 8) & 0xff;

        // enable broadcasts on the battery server, so this or any other client can receive them
        status = wiced_bt_gatt_send_write ( conn_id, GATT_WRITE, p_write );
        wiced_bt_free_buffer( p_write );
        WICED_BT_TRACE("battery_client_gatts_enable_broadcast:%d \n", status);
    }
}

#endif
