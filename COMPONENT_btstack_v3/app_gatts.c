/*
 * Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
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

/* This file is applicable for all devices with BTSTACK version 3.0 and greater, for example 55572 */

#include "wiced_bt_gatt.h"
#include "wiced_memory.h"
#include "bt_types.h"
#include "wiced_bt_trace.h"
#include "battery_client.h"

uint8_t *battery_client_alloc_buffer(uint16_t len)
{
    uint8_t *p = (uint8_t *) wiced_bt_get_buffer( len );
    WICED_BT_TRACE( "[%s] len %d alloc 0x%x", __FUNCTION__, len, p );

    return p;
}

void battery_client_free_buffer(uint8_t *p_data)
{
    wiced_bt_free_buffer( p_data );

    WICED_BT_TRACE( "[%s] 0x%x", __FUNCTION__, p_data );
}

void battery_client_handle_op_complete(wiced_bt_gatt_operation_complete_t *p_data)
{
    switch ( p_data->op )
    {
    case GATTC_OPTYPE_READ_HANDLE:
    case GATTC_OPTYPE_READ_BY_TYPE:
    case GATTC_OPTYPE_READ_MULTIPLE:
//            WICED_BT_TRACE( "read_rsp status:%d\n", p_data->status );
        battery_client_process_read_rsp(p_data);
        break;

    case GATTC_OPTYPE_WRITE_WITH_RSP:
    case GATTC_OPTYPE_WRITE_NO_RSP:
//            WICED_BT_TRACE( "write_rsp status:%d desc_handle:%x \n", p_data->status,p_data->response_data.handle );
        break;

    case GATTC_OPTYPE_CONFIG_MTU:
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

        case GATT_GET_RESPONSE_BUFFER_EVT:
            p_data->buffer_request.buffer.p_app_rsp_buffer = battery_client_alloc_buffer (p_data->buffer_request.len_requested);
            p_data->buffer_request.buffer.p_app_ctxt       = battery_client_free_buffer;
            result = WICED_BT_GATT_SUCCESS;
            break;

        case GATT_APP_BUFFER_TRANSMITTED_EVT:
            {
                pfn_free_buffer_t pfn_free = (pfn_free_buffer_t)p_data->buffer_xmitted.p_app_ctxt;

                /* If the buffer is dynamic, the context will point to a function to free it. */
                if (pfn_free)
                    pfn_free(p_data->buffer_xmitted.p_app_data);

                result = WICED_BT_GATT_SUCCESS;
            }
            break;

        default:
            break;
    }

    return result;
}
