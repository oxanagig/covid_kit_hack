/**
 * Copyright (c) 2012 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
    Copyright (C) 2023 Xiang Gao <xiang.gao@outlook.com>
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "app_error.h"
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_assert.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advertising.h"
#include "ble_advdata.h"
#include "ble_hids.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "bsp_btn_ble.h"
#include "app_scheduler.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "peer_manager_handler.h"

#include "nrfx_saadc.h"
#include "nrf_drv_saadc.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define DEVICE_NAME                         "COVID Camera Remote"                       /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                   "Xiang's Home"                             /**< Manufacturer. Will be passed to Device Information Service. */

#define APP_BLE_OBSERVER_PRIO               3                                          /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG                1                                          /**< A tag identifying the SoftDevice BLE configuration. */

#define BATTERY_LEVEL_MEAS_INTERVAL         APP_TIMER_TICKS(2000)                      /**< Battery level measurement interval (ticks). */

#define PNP_ID_VENDOR_ID_SOURCE             0x02                                       /**< Vendor ID Source. */
#define PNP_ID_VENDOR_ID                    0x1915                                     /**< Vendor ID. */
#define PNP_ID_PRODUCT_ID                   0xEEEE                                     /**< Product ID. */
#define PNP_ID_PRODUCT_VERSION              0x0001                                     /**< Product Version. */

#define APP_ADV_FAST_INTERVAL               0x0028                                     /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 25 ms.). */
#define APP_ADV_SLOW_INTERVAL               0x0C80                                     /**< Slow advertising interval (in units of 0.625 ms. This value corrsponds to 2 seconds). */

#define APP_ADV_FAST_DURATION               3000                                       /**< The advertising duration of fast advertising in units of 10 milliseconds. */
#define APP_ADV_SLOW_DURATION               18000                                      /**< The advertising duration of slow advertising in units of 10 milliseconds. */

/**@brief Macro to convert the result of ADC conversion in millivolts.
 *
 * @param[in]  ADC_VALUE   ADC result.
 *
 * @retval     Result converted to millivolts.
 */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)



/*lint -emacro(524, MIN_CONN_INTERVAL) // Loss of precision */
#define MIN_CONN_INTERVAL                   MSEC_TO_UNITS(7.5, UNIT_1_25_MS)           /**< Minimum connection interval (7.5 ms) */
#define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(30, UNIT_1_25_MS)            /**< Maximum connection interval (30 ms). */
#define SLAVE_LATENCY                       6                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    MSEC_TO_UNITS(430, UNIT_10_MS)             /**< Connection supervisory timeout (430 ms). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY      APP_TIMER_TICKS(5000)                      /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY       APP_TIMER_TICKS(30000)                     /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT        3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                      1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                      0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      0                                          /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                  0                                          /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                       0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16                                         /**< Maximum encryption key size. */

#define OUTPUT_REPORT_INDEX                 0                                          /**< Index of Output Report. */
#define OUTPUT_REPORT_MAX_LEN               1                                          /**< Maximum length of Output Report. */
#define INPUT_REPORT_KEYS_INDEX             0                                          /**< Index of Input Report. */
#define OUTPUT_REPORT_BIT_MASK_CAPS_LOCK    0x02                                       /**< CAPS LOCK bit in Output Report (based on 'LED Page (0x08)' of the Universal Serial Bus HID Usage Tables). */
#define INPUT_REP_REF_ID                    1                                          /**< Id of reference to Keyboard Input Report. */
#define OUTPUT_REP_REF_ID                   1                                          /**< Id of reference to Keyboard Output Report. */
#define FEATURE_REP_REF_ID                  0                                          /**< ID of reference to Keyboard Feature Report. */
#define FEATURE_REPORT_MAX_LEN              2                                          /**< Maximum length of Feature Report. */
#define FEATURE_REPORT_INDEX                0                                          /**< Index of Feature Report. */

#define MAX_BUFFER_ENTRIES                  5                                          /**< Number of elements that can be enqueued */

#define BASE_USB_HID_SPEC_VERSION           0x0101                                     /**< Version number of base USB HID Specification implemented by this application. */

#define INPUT_REPORT_KEYS_MAX_LEN           8                                          /**< Maximum length of the Input Report characteristic. */

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS   600                                             /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION    6                                               /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS  270                                             /**< Typical forward voltage drop of the diode . */
#define ADC_RES_10BIT                   1024                                            /**< Maximum digital value for 10-bit ADC conversion. */


#define DEAD_BEEF                           0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define SCHED_MAX_EVENT_DATA_SIZE           APP_TIMER_SCHED_EVENT_DATA_SIZE            /**< Maximum size of scheduler events. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE                    20                                         /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE                    10                                         /**< Maximum number of events in the scheduler queue. */
#endif

#define MODIFIER_KEY_POS                    0                                          /**< Position of the modifier byte in the Input Report. */
#define SCAN_CODE_POS                       2                                          /**< The start position of the key scan code in a HID Report. */
#define SHIFT_KEY_CODE                      0x02                                       /**< Key code indicating the press of the Shift Key. */

#define MAX_KEYS_IN_ONE_REPORT              (INPUT_REPORT_KEYS_MAX_LEN - SCAN_CODE_POS)/**< Maximum number of key presses that can be sent in one Input Report. */

#define REPORT_ID_CONSUMER_CTRL		1
#define REPORT_IDX_CONSUMER_CTRL	0
#define REPORT_SIZE_CONSUMER_CTRL	2  /* bytes */

#define POWER_EN_PIN                0
#define BUTTON_PIN                  25


static void turn_off_device(void);

APP_TIMER_DEF(power_off_timer);
APP_TIMER_DEF(m_battery_timer_id);                                  /**< Battery timer. */


BLE_HIDS_DEF(m_hids,                                                /**< Structure used to identify the HID service. */
             NRF_SDH_BLE_TOTAL_LINK_COUNT,
             INPUT_REPORT_KEYS_MAX_LEN,
             OUTPUT_REPORT_MAX_LEN,
             FEATURE_REPORT_MAX_LEN);
BLE_BAS_DEF(m_bas);                                                 /**< Structure used to identify the battery service. */
NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                 /**< Advertising module instance. */

static bool              m_in_boot_mode = false;                    /**< Current protocol mode. */
static uint16_t          m_conn_handle  = BLE_CONN_HANDLE_INVALID;  /**< Handle of the current connection. */
static pm_peer_id_t      m_peer_id;                                 /**< Device reference handle to the current bonded central. */

static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE, BLE_UUID_TYPE_BLE}};

static nrf_saadc_value_t adc_buf[2];

/**@brief Function for handling the ADC interrupt.
 *
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        nrf_saadc_value_t adc_result;
        uint16_t          batt_lvl_in_milli_volts;
        uint8_t           percentage_batt_lvl;
        uint32_t          err_code;

        adc_result = p_event->data.done.p_buffer[0];

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, 1);
        APP_ERROR_CHECK(err_code);

        batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) +
                                  DIODE_FWD_VOLT_DROP_MILLIVOLTS;
        percentage_batt_lvl = battery_level_in_percent(batt_lvl_in_milli_volts);
        
        if(percentage_batt_lvl<=20)
        {
            // Red LED Indicate Battery low
            nrf_gpio_pin_write(20,0);
        }
        else
        {
            nrf_gpio_pin_write(20,1);
        }

        err_code = ble_bas_battery_level_update(&m_bas, percentage_batt_lvl, BLE_CONN_HANDLE_ALL);
        if ((err_code != NRF_SUCCESS) &&
            (err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != NRF_ERROR_RESOURCES) &&
            (err_code != NRF_ERROR_BUSY) &&
            (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
           )
        {
            APP_ERROR_HANDLER(err_code);
        }
    }
}

/**@brief Function for configuring ADC to do battery level conversion.
 */
static void adc_configure(void)
{
    ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);
    err_code = nrf_drv_saadc_channel_init(0, &config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[0], 1);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[1], 1);
    APP_ERROR_CHECK(err_code);
}



static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt);

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
    turn_off_device();
}


/**@brief Function for setting filtered whitelist.
 *
 * @param[in] skip  Filter passed to @ref pm_peer_id_list.
 */
static void whitelist_set(pm_peer_id_list_skip_t skip)
{
    pm_peer_id_t peer_ids[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
    uint32_t     peer_id_count = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

    ret_code_t err_code = pm_peer_id_list(peer_ids, &peer_id_count, PM_PEER_ID_INVALID, skip);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("\tm_whitelist_peer_cnt %d, MAX_PEERS_WLIST %d",
                   peer_id_count + 1,
                   BLE_GAP_WHITELIST_ADDR_MAX_COUNT);

    err_code = pm_whitelist_set(peer_ids, peer_id_count);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for setting filtered device identities.
 *
 * @param[in] skip  Filter passed to @ref pm_peer_id_list.
 */
static void identities_set(pm_peer_id_list_skip_t skip)
{
    pm_peer_id_t peer_ids[BLE_GAP_DEVICE_IDENTITIES_MAX_COUNT];
    uint32_t     peer_id_count = BLE_GAP_DEVICE_IDENTITIES_MAX_COUNT;

    ret_code_t err_code = pm_peer_id_list(peer_ids, &peer_id_count, PM_PEER_ID_INVALID, skip);
    APP_ERROR_CHECK(err_code);

    err_code = pm_device_identities_list_set(peer_ids, peer_id_count);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        whitelist_set(PM_PEER_ID_LIST_SKIP_NO_ID_ADDR);

        ret_code_t ret = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(ret);
    }
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_CONN_SEC_SUCCEEDED:
            m_peer_id = p_evt->peer_id;
            break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
            if (     p_evt->params.peer_data_update_succeeded.flash_changed
                 && (p_evt->params.peer_data_update_succeeded.data_id == PM_PEER_DATA_ID_BONDING))
            {
                NRF_LOG_INFO("New Bond, add the peer to the whitelist if possible");
                // Note: You should check on what kind of white list policy your application should use.

                whitelist_set(PM_PEER_ID_LIST_SKIP_NO_ID_ADDR);
            }
            break;

        default:
            break;
    }
}


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
    turn_off_device();
}


/**@brief Function for handling advertising errors.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void ble_advertising_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
    turn_off_device();
}



/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    ret_code_t err_code;
    err_code = nrf_drv_saadc_sample();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create battery timer.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_HID);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
    turn_off_device();
}


/**@brief Function for initializing the Queued Write Module.
 */
static void qwr_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init_obj = {0};

    qwr_init_obj.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing Device Information Service.
 */
static void dis_init(void)
{
    ret_code_t       err_code;
    ble_dis_init_t   dis_init_obj;
    ble_dis_pnp_id_t pnp_id;

    pnp_id.vendor_id_source = PNP_ID_VENDOR_ID_SOURCE;
    pnp_id.vendor_id        = PNP_ID_VENDOR_ID;
    pnp_id.product_id       = PNP_ID_PRODUCT_ID;
    pnp_id.product_version  = PNP_ID_PRODUCT_VERSION;

    memset(&dis_init_obj, 0, sizeof(dis_init_obj));

    ble_srv_ascii_to_utf8(&dis_init_obj.manufact_name_str, MANUFACTURER_NAME);
    dis_init_obj.p_pnp_id = &pnp_id;

    dis_init_obj.dis_char_rd_sec = SEC_JUST_WORKS;

    err_code = ble_dis_init(&dis_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing Battery Service.
 */
static void bas_init(void)
{
    ret_code_t     err_code;
    ble_bas_init_t bas_init_obj;

    memset(&bas_init_obj, 0, sizeof(bas_init_obj));

    bas_init_obj.evt_handler          = NULL;
    bas_init_obj.support_notification = true;
    bas_init_obj.p_report_ref         = NULL;
    bas_init_obj.initial_batt_level   = 100;

    bas_init_obj.bl_rd_sec        = SEC_JUST_WORKS;
    bas_init_obj.bl_cccd_wr_sec   = SEC_JUST_WORKS;
    bas_init_obj.bl_report_rd_sec = SEC_JUST_WORKS;

    err_code = ble_bas_init(&m_bas, &bas_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing HID Service.
 */
static void hids_init(void)
{
    ret_code_t                    err_code;
    ble_hids_init_t               hids_init_obj;
    ble_hids_inp_rep_init_t     * p_input_report;
    ble_hids_outp_rep_init_t    * p_output_report;
    ble_hids_feature_rep_init_t * p_feature_report;
    uint8_t                       hid_info_flags;

    static ble_hids_inp_rep_init_t     input_report_array[1];
    static ble_hids_outp_rep_init_t    output_report_array[1];
    static ble_hids_feature_rep_init_t feature_report_array[1];
    static uint8_t                     report_map_data[] =
    {
        0x05, 0x0C,				/* Usage page (Consumer Control) */
		0x09, 0x01,				/* Usage (Consumer Control) */
		0xA1, 0x01,				/* Collection (Application) */
		0x85, REPORT_ID_CONSUMER_CTRL,		/* Report ID */
		0x15, 0x00,				/* Logical minimum (0) */
		0x26, 0xFF, 0x03,			/* Logical maximum (0x3FF) */
		0x19, 0x00,				/* Usage minimum (0) */
		0x2A, 0xFF, 0x03,			/* Usage maximum (0x3FF) */
		0x75, 0x10,				/* Report Size (16) */
		0x95, 0x01,				/* Report Count */
		0x81, 0x00,				/* Input (Data,Array,Absolute) */
		0xC0,					/* End Collection (Application) */
    };

    memset((void *)input_report_array, 0, sizeof(ble_hids_inp_rep_init_t));
    memset((void *)output_report_array, 0, sizeof(ble_hids_outp_rep_init_t));
    memset((void *)feature_report_array, 0, sizeof(ble_hids_feature_rep_init_t));

    // Initialize HID Service
    p_input_report                      = &input_report_array[INPUT_REPORT_KEYS_INDEX];
    p_input_report->max_len             = INPUT_REPORT_KEYS_MAX_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    p_input_report->sec.cccd_wr = SEC_JUST_WORKS;
    p_input_report->sec.wr      = SEC_JUST_WORKS;
    p_input_report->sec.rd      = SEC_JUST_WORKS;

    p_output_report                      = &output_report_array[OUTPUT_REPORT_INDEX];
    p_output_report->max_len             = OUTPUT_REPORT_MAX_LEN;
    p_output_report->rep_ref.report_id   = OUTPUT_REP_REF_ID;
    p_output_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_OUTPUT;

    p_output_report->sec.wr = SEC_JUST_WORKS;
    p_output_report->sec.rd = SEC_JUST_WORKS;

    p_feature_report                      = &feature_report_array[FEATURE_REPORT_INDEX];
    p_feature_report->max_len             = FEATURE_REPORT_MAX_LEN;
    p_feature_report->rep_ref.report_id   = FEATURE_REP_REF_ID;
    p_feature_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_FEATURE;

    p_feature_report->sec.rd              = SEC_JUST_WORKS;
    p_feature_report->sec.wr              = SEC_JUST_WORKS;

    hid_info_flags = HID_INFO_FLAG_REMOTE_WAKE_MSK | HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK;

    memset(&hids_init_obj, 0, sizeof(hids_init_obj));

    hids_init_obj.evt_handler                    = on_hids_evt;
    hids_init_obj.error_handler                  = service_error_handler;
    hids_init_obj.is_kb                          = false;
    hids_init_obj.is_mouse                       = false;
    hids_init_obj.inp_rep_count                  = 1;
    hids_init_obj.p_inp_rep_array                = input_report_array;
    hids_init_obj.outp_rep_count                 = 1;
    hids_init_obj.p_outp_rep_array               = output_report_array;
    hids_init_obj.feature_rep_count              = 1;
    hids_init_obj.p_feature_rep_array            = feature_report_array;
    hids_init_obj.rep_map.data_len               = sizeof(report_map_data);
    hids_init_obj.rep_map.p_data                 = report_map_data;
    hids_init_obj.hid_information.bcd_hid        = BASE_USB_HID_SPEC_VERSION;
    hids_init_obj.hid_information.b_country_code = 0;
    hids_init_obj.hid_information.flags          = hid_info_flags;
    hids_init_obj.included_services_count        = 0;
    hids_init_obj.p_included_services_array      = NULL;

    hids_init_obj.rep_map.rd_sec         = SEC_JUST_WORKS;
    hids_init_obj.hid_information.rd_sec = SEC_JUST_WORKS;

    hids_init_obj.boot_kb_inp_rep_sec.cccd_wr = SEC_JUST_WORKS;
    hids_init_obj.boot_kb_inp_rep_sec.rd      = SEC_JUST_WORKS;

    hids_init_obj.boot_kb_outp_rep_sec.rd = SEC_JUST_WORKS;
    hids_init_obj.boot_kb_outp_rep_sec.wr = SEC_JUST_WORKS;

    hids_init_obj.protocol_mode_rd_sec = SEC_JUST_WORKS;
    hids_init_obj.protocol_mode_wr_sec = SEC_JUST_WORKS;
    hids_init_obj.ctrl_point_wr_sec    = SEC_JUST_WORKS;

    err_code = ble_hids_init(&m_hids, &hids_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    qwr_init();
    dis_init();
    bas_init();
    hids_init();
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
    turn_off_device();
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
 */
static void timers_start(void)
{
    ret_code_t err_code;

    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief   Function for transmitting a key scan Press & Release Notification.
 *
 * @warning This handler is an example only. You need to analyze how you wish to send the key
 *          release.
 *
 * @param[in]  p_instance     Identifies the service for which Key Notifications are requested.
 * @param[in]  p_key_pattern  Pointer to key pattern.
 * @param[in]  pattern_len    Length of key pattern. 0 < pattern_len < 7.
 * @param[in]  pattern_offset Offset applied to Key Pattern for transmission.
 * @param[out] actual_len     Provides actual length of Key Pattern transmitted, making buffering of
 *                            rest possible if needed.
 * @return     NRF_SUCCESS on success, NRF_ERROR_RESOURCES in case transmission could not be
 *             completed due to lack of transmission buffer or other error codes indicating reason
 *             for failure.
 *
 */
static uint32_t send_key_scan_press_release(ble_hids_t * p_hids,
                                            uint8_t    * p_key_pattern,
                                            uint16_t     pattern_len,
                                            uint16_t     pattern_offset,
                                            uint16_t   * p_actual_len)
{
    ret_code_t err_code;

    err_code = ble_hids_inp_rep_send(p_hids,
                                             INPUT_REPORT_KEYS_INDEX,
                                             pattern_len,
                                             p_key_pattern,
                                             m_conn_handle);

    *p_actual_len = pattern_len;

    return err_code;
}


/**
 *  @brief Put a 16-bit integer as little-endian to arbitrary location.
 *
 *  Put a 16-bit integer, originally in host endianness, to a
 *  potentially unaligned memory location in little-endian format.
 *
 *  @param val 16-bit integer in host endianness.
 *  @param dst Destination memory address to store the result.
 */
static inline void sys_put_le16(uint16_t val, uint8_t dst[2])
{
	dst[0] = val;
	dst[1] = val >> 8;
}

/**@brief Function for sending sample key presses to the peer.
 *
 * @param[in]   key_pattern_len   Pattern length.
 * @param[in]   p_key_pattern     Pattern to be sent.
 */
static void keys_send(uint8_t key_pattern_len, uint8_t * p_key_pattern)
{
    ret_code_t err_code;
    uint16_t actual_len;

    err_code = send_key_scan_press_release(&m_hids,
                                           p_key_pattern,
                                           key_pattern_len,
                                           0,
                                           &actual_len);

    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
        turn_off_device();
    }
}


/**@brief Function for handling the HID Report Characteristic Write event.
 *
 * @param[in]   p_evt   HID service event.
 */
static void on_hid_rep_char_write(ble_hids_evt_t * p_evt)
{
    if (p_evt->params.char_write.char_id.rep_type == BLE_HIDS_REP_TYPE_OUTPUT)
    {
        ret_code_t err_code;
        uint8_t  report_val;
        uint8_t  report_index = p_evt->params.char_write.char_id.rep_index;

        if (report_index == OUTPUT_REPORT_INDEX)
        {
            // This code assumes that the output report is one byte long. Hence the following
            // static assert is made.
            STATIC_ASSERT(OUTPUT_REPORT_MAX_LEN == 1);

            err_code = ble_hids_outp_rep_get(&m_hids,
                                             report_index,
                                             OUTPUT_REPORT_MAX_LEN,
                                             0,
                                             m_conn_handle,
                                             &report_val);
            APP_ERROR_CHECK(err_code);

        }
    }
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling HID events.
 *
 * @details This function will be called for all HID events which are passed to the application.
 *
 * @param[in]   p_hids  HID service structure.
 * @param[in]   p_evt   Event received from the HID service.
 */
static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_HIDS_EVT_BOOT_MODE_ENTERED:
            m_in_boot_mode = true;
            break;

        case BLE_HIDS_EVT_REPORT_MODE_ENTERED:
            m_in_boot_mode = false;
            break;

        case BLE_HIDS_EVT_REP_CHAR_WRITE:
            on_hid_rep_char_write(p_evt);
            break;

        case BLE_HIDS_EVT_NOTIF_ENABLED:
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_DIRECTED_HIGH_DUTY:
            NRF_LOG_INFO("High Duty Directed advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_DIRECTED:
            NRF_LOG_INFO("Directed advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_SLOW:
            NRF_LOG_INFO("Slow advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_FAST_WHITELIST:
            NRF_LOG_INFO("Fast advertising with whitelist.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_SLOW_WHITELIST:
            NRF_LOG_INFO("Slow advertising with whitelist.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        case BLE_ADV_EVT_WHITELIST_REQUEST:
        {
            ble_gap_addr_t whitelist_addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            ble_gap_irk_t  whitelist_irks[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            uint32_t       addr_cnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
            uint32_t       irk_cnt  = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

            err_code = pm_whitelist_get(whitelist_addrs, &addr_cnt,
                                        whitelist_irks,  &irk_cnt);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_DEBUG("pm_whitelist_get returns %d addr in whitelist and %d irk whitelist",
                          addr_cnt, irk_cnt);

            // Set the correct identities list (no excluding peers with no Central Address Resolution).
            identities_set(PM_PEER_ID_LIST_SKIP_NO_IRK);

            // Apply the whitelist.
            err_code = ble_advertising_whitelist_reply(&m_advertising,
                                                       whitelist_addrs,
                                                       addr_cnt,
                                                       whitelist_irks,
                                                       irk_cnt);
            APP_ERROR_CHECK(err_code);
        } break; //BLE_ADV_EVT_WHITELIST_REQUEST

        case BLE_ADV_EVT_PEER_ADDR_REQUEST:
        {
            pm_peer_data_bonding_t peer_bonding_data;

            // Only Give peer address if we have a handle to the bonded peer.
            if (m_peer_id != PM_PEER_ID_INVALID)
            {
                err_code = pm_peer_data_bonding_load(m_peer_id, &peer_bonding_data);
                if (err_code != NRF_ERROR_NOT_FOUND)
                {
                    APP_ERROR_CHECK(err_code);

                    // Manipulate identities to exclude peers with no Central Address Resolution.
                    identities_set(PM_PEER_ID_LIST_SKIP_ALL);

                    ble_gap_addr_t * p_peer_addr = &(peer_bonding_data.peer_ble_id.id_addr_info);
                    err_code = ble_advertising_peer_addr_reply(&m_advertising, p_peer_addr);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; //BLE_ADV_EVT_PEER_ADDR_REQUEST

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}



/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
    uint32_t         err_code;
    static const uint16_t usage_id_vol_down = 0x00EA;
    static const uint16_t usage_id_empty = 0;
    uint8_t buf[REPORT_SIZE_CONSUMER_CTRL];
    NRF_LOG_INFO("BSP EVENT %d",event);
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        case BSP_EVENT_KEY_0:
            NRF_LOG_INFO("m_conn_handle:%x",m_conn_handle);
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                sys_put_le16(usage_id_vol_down, buf);
                keys_send(sizeof(buf), buf);
            }
            break;
        case BSP_EVENT_KEY_LAST:
            // this is the button release event
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                sys_put_le16(usage_id_empty, buf);
                keys_send(sizeof(buf), buf);
            }
            break;
        case BSP_EVENT_KEY_1:
            // this is the button long press event
            if(m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                NRF_LOG_INFO("Delete Bonds");
                //only delete the bond while not connected
                delete_bonds();
            }
            break;
        default:
            break;
    }
    //reschedule the timer once there is button pressed.
    app_timer_stop(power_off_timer);
    app_timer_start(power_off_timer, APP_TIMER_TICKS(1000*60*3), NULL);
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    uint8_t                adv_flags;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    adv_flags                            = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = adv_flags;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_whitelist_enabled          = true;
    init.config.ble_adv_directed_high_duty_enabled = true;
    init.config.ble_adv_directed_enabled           = false;
    init.config.ble_adv_directed_interval          = 0;
    init.config.ble_adv_directed_timeout           = 0;
    init.config.ble_adv_fast_enabled               = true;
    init.config.ble_adv_fast_interval              = APP_ADV_FAST_INTERVAL;
    init.config.ble_adv_fast_timeout               = APP_ADV_FAST_DURATION;
    init.config.ble_adv_slow_enabled               = true;
    init.config.ble_adv_slow_interval              = APP_ADV_SLOW_INTERVAL;
    init.config.ble_adv_slow_timeout               = APP_ADV_SLOW_DURATION;

    init.evt_handler   = on_adv_evt;
    init.error_handler = ble_advertising_error_handler;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * 
 */
static void buttons_leds_init(void)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
    // startup_event is not used
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    app_sched_execute();
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


static void power_off_handler(void * p_context)
{
    turn_off_device();
}
/**@brief Function for turning off the power of the Soc 
 *
 * @details turn off the device
 */
static void turn_off_device(void)
{
    nrf_gpio_pin_write(POWER_EN_PIN,0);
}

/**@brief Function for Keep the power of the Soc 
 *
 * @details keep the power on for the device
 */
static void keep_device_on(void)
{
    nrf_gpio_pin_write(POWER_EN_PIN,1);
}

/**@brief Function for configuring the unused i/O pins for this application
 *
 * @details all of the circuits are turned off to save power
 */
static void io_init()
{   

    nrf_gpio_cfg_input(28,NRF_GPIO_PIN_NOPULL); // boosted voltage sense pin 1
    nrf_gpio_cfg_input(30,NRF_GPIO_PIN_NOPULL); // boosted voltage sense pin 2
    nrf_gpio_cfg_output(15); // LED power rail control pin 
    nrf_gpio_pin_write(15,0); // turn off strip dection LED power rail

    // not using the LEDs
    nrf_gpio_cfg_output(9); // Blue LED 0
    nrf_gpio_cfg_output(10); // Blue LED 1
    nrf_gpio_cfg_output(12); // Red LED 0
    nrf_gpio_cfg_output(14); // Red LED 1
    nrf_gpio_pin_write(9,0);
    nrf_gpio_pin_write(10,0);
    nrf_gpio_pin_write(12,0);
    nrf_gpio_pin_write(14,0);

    // Blue LED sense
    nrf_gpio_cfg_input(5,NRF_GPIO_PIN_NOPULL); // sense pin
    nrf_gpio_cfg_output(6);// sensor power pin
    nrf_gpio_pin_write(6,0);

    // Red LED sense
    nrf_gpio_cfg_input(4,NRF_GPIO_PIN_NOPULL); // sense pin
    nrf_gpio_cfg_output(1); // sense power pin
    nrf_gpio_pin_write(1,0);

    // power pin
    nrf_gpio_cfg_output(POWER_EN_PIN);
}


/**@brief Function for application main entry.
 */
int main(void)
{

    // Initialize.
    io_init();
    keep_device_on();
    log_init();
    timers_init();
    buttons_leds_init();
    power_management_init();
    ble_stack_init();
    adc_configure();
    scheduler_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
    conn_params_init();
    peer_manager_init();

    // Start execution.
    NRF_LOG_INFO("Application Started");
    timers_start();
    advertising_start(false);

    app_timer_create(&power_off_timer,APP_TIMER_MODE_SINGLE_SHOT,power_off_handler);
    app_timer_start(power_off_timer, APP_TIMER_TICKS(1000*60*3), NULL);
    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}


/**
 * @}
 */
