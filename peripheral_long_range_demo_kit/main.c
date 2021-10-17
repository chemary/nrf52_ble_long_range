/**
 * Copyright (c) 2016 - 2018, Nordic Semiconductor ASA
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
/**@cond To Make Doxygen skip documentation generation for this file.
 * @{
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_error.h"
#include "app_timer.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_srv_common.h"
#include "bsp_btn_ble.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_power.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "sdk_config.h"

#include "battery_voltage_saadc.h"
#include "main.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

APP_TIMER_DEF(m_battery_timer_id);              /**< Battery timer. */

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[2][BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set
                                                                      We use 2 because the one in use can not be modified.*/

static uint32_t adv_interval = ADV_INTERVAL; // Advertising interval (in units of 0.625 ms) 

static adv_scan_type_seclection_t m_adv_scan_type_selected = SELECTION_CONNECTABLE; /**< Global variable holding the current scan selection mode. */
static adv_scan_phy_seclection_t m_adv_scan_phy_selected = SELECTION_CODED_PHY;     /**< Global variable holding the current phy selection. */
static output_power_seclection_t m_output_power_selected = SELECTION_8_dBm;         /**< Global variable holding the current output power selection. */
static power_mode_t m_power_mode = POWER_MODE_NORMAL;                                /**< Global variable holding the current power mode. */
static bool m_app_initiated_disconnect = false;                                     //The application has initiated disconnect. Used to "tell" on_ble_gap_evt_disconnected() to not start advertising.
static bool m_waiting_for_disconnect_evt = false;                                   // Disconnect is initiated. The application has to wait for BLE_GAP_EVT_DISCONNECTED before proceeding.

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current BLE connection .*/
static uint8_t m_gap_role = BLE_GAP_ROLE_INVALID;        /**< BLE role for this connection, see @ref BLE_GAP_ROLES */

static custom_raw_adv_data_t m_custom_raw_adv_data; // Variable to hold voltage, temperature and time since start
static custom_adv_data_t m_custom_adv_data; // Variable to hold voltage, temperature and time since start

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data = {
    .adv_data = {
        .p_data = m_enc_advdata[0],
        .len = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data = {
        .p_data = NULL,
        .len = 0,
    }
};

// Connection parameters requested for connection.
static ble_gap_conn_params_t m_conn_param = {
    .min_conn_interval = CONN_INTERVAL_MIN, // Minimum connection interval.
    .max_conn_interval = CONN_INTERVAL_MAX, // Maximum connection interval.
    .slave_latency     = SLAVE_LATENCY,         // Slave latency.
    .conn_sup_timeout  = CONN_SUP_TIMEOUT    // Supervisory timeout.
};

static void advdata_update_handler(void *p_context) {
    UNUSED_PARAMETER(p_context);

    increase_sec_cnt();

    // Update batt and temp less often in low power mode
    if (m_power_mode == POWER_MODE_LOW_BATT && m_custom_raw_adv_data.le_sec_cnt % 10 != 0) {
        return;
    }

    update_temp();
    update_vbatt();

    NRF_LOG_INFO("Counter: %d, Voltage: %d.%03d, Temperature: %d.%02d", 
        m_custom_raw_adv_data.le_sec_cnt, 
        m_custom_raw_adv_data.vbatt / 1000,
        m_custom_raw_adv_data.vbatt % 1000,
        m_custom_raw_adv_data.temp / 4,
        (m_custom_raw_adv_data.temp % 4) * 25
   );

    if (m_conn_handle == BLE_CONN_HANDLE_INVALID) {
        advertising_data_set(false);
    }
}

/**@brief Function for handling BLE_GAP_EVT_CONNECTED events.
 * Save the connection handle and GAP role.
 * Turn on the "connected state" LED.
 */
static void on_ble_gap_evt_connected(ble_gap_evt_t const *p_gap_evt) {
    ret_code_t err_code;

    m_conn_handle = p_gap_evt->conn_handle;
    m_gap_role = p_gap_evt->params.connected.role;

    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, m_output_power_selected);
    APP_ERROR_CHECK(err_code);

    if (m_gap_role == BLE_GAP_ROLE_PERIPH) {
        NRF_LOG_INFO("Connected as a peripheral.");
    } else if (m_gap_role == BLE_GAP_ROLE_CENTRAL) {
        NRF_LOG_INFO("Connected as a central. Should not happen.");
    }

    // If advertising, stop advertising.
    (void)sd_ble_gap_adv_stop(m_adv_handle);
}

/**@brief Function for handling BLE_GAP_EVT_DISCONNECTED events.
 * Unset the connection handle and restart advertising if needed.
 */
static void on_ble_gap_evt_disconnected(ble_gap_evt_t const *p_gap_evt) {
    m_conn_handle = BLE_CONN_HANDLE_INVALID;
    m_waiting_for_disconnect_evt = false;

    NRF_LOG_INFO("Disconnected: reason 0x%x.", p_gap_evt->params.disconnected.reason);

    if (m_app_initiated_disconnect == false) // (If the app itself (button push) has initiated a disconnect, bsp_evt_handler will start the advertising.)
    {
        // Start advertising with the current setup.
        set_current_adv_params_and_start_advertising();
    }
}

/**@brief Function for handling scan request report.
 * Print the RSSI and address of the initiator if the RSSI has changed.
 */
static void on_scan_req_report(ble_gap_evt_scan_req_report_t const *p_scan_req_report) {
    static int8_t rssi_value = 0;

    if (rssi_value != p_scan_req_report->rssi) {
    rssi_value = p_scan_req_report->rssi;
    NRF_LOG_INFO("Received scan request with RSSI %d .", rssi_value);
    NRF_LOG_INFO("addr %02x:%02x:%02x:%02x:%02x:%02x",
        p_scan_req_report->peer_addr.addr[0],
        p_scan_req_report->peer_addr.addr[1],
        p_scan_req_report->peer_addr.addr[2],
        p_scan_req_report->peer_addr.addr[3],
        p_scan_req_report->peer_addr.addr[4],
        p_scan_req_report->peer_addr.addr[5]);
    }
}

/**@brief Function for handling BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context) {
    uint32_t err_code;
    ble_gap_evt_t const *p_gap_evt = &p_ble_evt->evt.gap_evt;
    ble_gap_conn_params_t params;

    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
            on_ble_gap_evt_connected(p_gap_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_ble_gap_evt_disconnected(p_gap_evt);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
            NRF_LOG_INFO("Connection interval updated: 0x%x, 0x%x.",
            p_gap_evt->params.conn_param_update.conn_params.min_conn_interval,
            p_gap_evt->params.conn_param_update.conn_params.max_conn_interval);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accept parameters requested by the peer.
            params = p_gap_evt->params.conn_param_update_request.conn_params;
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle, &params);
            APP_ERROR_CHECK(err_code);

            NRF_LOG_INFO("Connection interval updated (upon request): 0x%x, 0x%x.",
            p_gap_evt->params.conn_param_update_request.conn_params.min_conn_interval,
            p_gap_evt->params.conn_param_update_request.conn_params.max_conn_interval);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            NRF_LOG_DEBUG("BLE_GATTS_EVT_SYS_ATTR_MISSING");
            err_code = sd_ble_gatts_sys_attr_set(p_gap_evt->conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT: // Fallthrough.
        case BLE_GATTS_EVT_TIMEOUT:
            NRF_LOG_DEBUG("GATT timeout, disconnecting.");
            err_code = sd_ble_gap_disconnect(m_conn_handle,
            BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE:
            NRF_LOG_DEBUG("BLE_GAP_EVT_PHY_UPDATE: not implemented.");
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
            NRF_LOG_DEBUG("BLE_GAP_EVT_PHY_UPDATE_REQUEST: not implemented.");
            break;

        case BLE_GAP_EVT_SCAN_REQ_REPORT:
            on_scan_req_report(&p_gap_evt->params.scan_req_report);
            break;

        default:
            NRF_LOG_DEBUG("Received an unimplemented BLE event.");
            break;
    }
}

/**@brief This function will disconnect if connected, and stop advertising if advertising. */
static void disconnect_stop_adv(void) {
    ret_code_t err_code;
    // If advertising, stop advertising.
    (void)sd_ble_gap_adv_stop(m_adv_handle);

    // If connected, disconnect.
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
        NRF_LOG_INFO("Disconnecting...");

        m_app_initiated_disconnect = true;
        m_waiting_for_disconnect_evt = true;
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);

        if (err_code != NRF_SUCCESS) {
            NRF_LOG_ERROR("disconnect_stop_adv, sd_ble_gap_disconnect() failed: 0x%0x.", err_code);
            m_app_initiated_disconnect = false;
            m_waiting_for_disconnect_evt = false;
        }
        while (m_waiting_for_disconnect_evt == true) {
        // Wait until BLE_GAP_EVT_DISCONNECT has occured.
        }
    }
}

/**@brief Function for setting up advertising data. */
static void advertising_data_set(bool set_adv_params) {
    ret_code_t ret;

    ble_advdata_service_data_t service_data[3];

    service_data[0].service_uuid =  BLE_UUID_BATTERY_SERVICE;
    service_data[0].data.size = sizeof(m_custom_adv_data.vbatt);
    service_data[0].data.p_data = (uint8_t *)&m_custom_adv_data.vbatt;

    service_data[1].service_uuid = BLE_UUID_HEALTH_THERMOMETER_SERVICE;
    service_data[1].data.size = sizeof(m_custom_adv_data.temp);
    service_data[1].data.p_data = (uint8_t *)&m_custom_adv_data.temp;

    service_data[2].service_uuid = 0x1801;
    service_data[2].data.size = sizeof(m_custom_adv_data.sec_cnt);
    service_data[2].data.p_data = (uint8_t *)&m_custom_adv_data.sec_cnt;

    ble_advdata_t const adv_data = {
        .name_type = BLE_ADVDATA_FULL_NAME,
        .flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE,
        .include_appearance = false,
        .service_data_count = 3,
        .p_service_data_array = service_data
    };

    /* swap adv data buffer - from API doc: "In order to update advertising 
    data while advertising, new advertising buffers must be provided"*/
    m_adv_data.adv_data.p_data = (m_adv_data.adv_data.p_data == m_enc_advdata[0])
                                ? m_enc_advdata[1] : m_enc_advdata[0];

    ret = ble_advdata_encode(&adv_data, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(ret);

    if (!set_adv_params) {
        // Only update advertising data while advertising
        ret = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, NULL);
        APP_ERROR_CHECK(ret);
        return;
    }

    ble_gap_adv_params_t adv_params = {
        .properties = {
            .type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED,
        },
        .p_peer_addr = NULL,
        .filter_policy = BLE_GAP_ADV_FP_ANY,
        .interval = adv_interval,
        .duration = 0,

        .primary_phy = BLE_GAP_PHY_1MBPS, // Must be changed to connect in long range. (BLE_GAP_PHY_CODED)
        .secondary_phy = BLE_GAP_PHY_1MBPS,
        .scan_req_notification = 1,
    };

    if (m_adv_scan_phy_selected == SELECTION_1M_PHY) {
        // 1M coded for adv
        // NRF_LOG_INFO("Setting adv params PHY to 1M. ");
        adv_params.primary_phy = BLE_GAP_PHY_1MBPS;
        adv_params.secondary_phy = BLE_GAP_PHY_1MBPS;

        if (m_adv_scan_type_selected == SELECTION_CONNECTABLE) {
            // NRF_LOG_INFO("Advertising type set to CONNECTABLE_SCANNABLE_UNDIRECTED ");
            adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
        } else if (m_adv_scan_type_selected == SELECTION_NON_CONNECTABLE) {
            // NRF_LOG_INFO("Advertising type set to NONCONNECTABLE_SCANNABLE_UNDIRECTED ");
            adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED;
        }
    }
    else if (m_adv_scan_phy_selected == SELECTION_CODED_PHY) {
        // only extended advertising will allow primary phy to be coded
        // NRF_LOG_INFO("Setting adv params phy to coded phy .. ");
        adv_params.primary_phy  = BLE_GAP_PHY_CODED;
        adv_params.secondary_phy = BLE_GAP_PHY_CODED;

        if (m_adv_scan_type_selected == SELECTION_CONNECTABLE) {
            //NRF_LOG_INFO("Advertising type set to EXTENDED_CONNECTABLE_NONSCANNABLE_UNDIRECTED ");
            adv_params.properties.type = BLE_GAP_ADV_TYPE_EXTENDED_CONNECTABLE_NONSCANNABLE_UNDIRECTED;
        } else if (m_adv_scan_type_selected == SELECTION_NON_CONNECTABLE) {
            // NRF_LOG_INFO("Advertising type set to EXTENDED_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED ");
            adv_params.properties.type = BLE_GAP_ADV_TYPE_EXTENDED_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
        }
    }

    ret = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    APP_ERROR_CHECK(ret);
}

/**@brief Function for starting advertising. */
static void advertising_start(void) {
    ret_code_t err_code;
    //NRF_LOG_INFO("Starting advertising.");

    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_adv_handle, m_output_power_selected);
    APP_ERROR_CHECK(err_code);

    //NRF_LOG_INFO("Output power set to %d dBm", m_output_power_selected);

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);
    m_app_initiated_disconnect = false;
}

/**@brief Function for initializing the log module.
 */
static void log_init(void) {
    uint32_t log_timestamp_func(void) {
        return m_custom_raw_adv_data.le_sec_cnt;
    }
    ret_code_t err_code = NRF_LOG_INIT(log_timestamp_func, 1); // Counter is incremented at 1Hz rate
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates application timers.
 */
static void timers_init(void) {
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_battery_timer_id, APP_TIMER_MODE_REPEATED, advdata_update_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void) {
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

/**@brief Function for initializing GAP parameters.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name and the preferred connection parameters.
 */
static void gap_params_init(void) {
    ret_code_t err_code;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode, (uint8_t const *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_ppcp_set(&m_conn_param);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the GATT library. */

/**@brief Function for initializing power management.
 */
static void power_management_init(void) {
    ret_code_t ret;
    ret = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(ret);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details Handle any pending operation(s), then sleep until the next event occurs.
 */
static void idle_state_handle(void) {
    if (NRF_LOG_PROCESS() == false) {
        nrf_pwr_mgmt_run();
    }
}

/**@brief Function for switching between "non-connectable and connectable advertising" selection.
 * Note: this function does only set the internal "advertising state", it does not start advertising.
 *       The new advertising mode will be used when (re-)starting to advertise.
 */
static void on_non_conn_or_conn_adv_selection(void) {
    // Change the advertising type
    adv_scan_type_seclection_t new_adv_selection;

    switch (m_adv_scan_type_selected) {
        case SELECTION_CONNECTABLE: // Connectable advertising is the previous state.
            // Current state is non-connectable advertising. Start blinking associated LED.
            new_adv_selection = SELECTION_NON_CONNECTABLE;
            break;
        case SELECTION_NON_CONNECTABLE:
            // Current state is connectable advertising. Start blinking associated LED.
            new_adv_selection = SELECTION_CONNECTABLE;
            break;
    }
    m_adv_scan_type_selected = new_adv_selection;
    NRF_LOG_INFO("Mode changed to: %d", m_adv_scan_type_selected);
}

/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
void bsp_evt_handler(bsp_event_t event) {
    ret_code_t err_code;

    // Set the correct parameters, depending on the button pushed.
    switch (event) {
        case NON_CONN_OR_CONN_ADV_BUTTON_EVENT:
            on_non_conn_or_conn_adv_selection();
            break;
        default:
            break;
    }

    disconnect_stop_adv();
    advertising_data_set(true);
    advertising_start();
}

/**@brief Function for initializing buttons and leds.
 */
static void buttons_leds_init(void) {
    ret_code_t err_code;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_evt_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting advertising with the current selections of output power, phy, and connectable or non-connectable advertising.
 */
static void set_current_adv_params_and_start_advertising(void) {
    advertising_data_set(true);
    advertising_start();
}

/**@brief Function for updating the VBATT field of TLM*/
static void update_vbatt(void) {
    battery_voltage_get(&m_custom_raw_adv_data.vbatt); // Get new battery voltage
    m_custom_adv_data.vbatt[0] = (uint8_t)(m_custom_raw_adv_data.vbatt/100); // 1V = 10

    if (m_custom_raw_adv_data.vbatt < LOW_POWER_VOLTAGE && m_power_mode == POWER_MODE_NORMAL) {
        NRF_LOG_ERROR("Low voltage (%d), set slow advertisement", m_custom_raw_adv_data.vbatt);
        m_power_mode = POWER_MODE_LOW_BATT;
        adv_interval = ADV_INTERVAL_SLOW;
        disconnect_stop_adv();
        advertising_data_set(true);
        advertising_start();
    }

    if (m_custom_raw_adv_data.vbatt < CUT_OFF_VOLTAGE && m_custom_raw_adv_data.le_sec_cnt > CUT_OFF_VOLTAGE_POWER_UP_DELAY) {
        NRF_LOG_ERROR("Cut-off voltage reached (%d), going to sleep", m_custom_raw_adv_data.vbatt);
        nrf_power_system_off();
        while(1);
    }
}

/**@brief Function for updating the TEMP field of TLM*/
static void update_temp(void) {
    (void)sd_temp_get(&m_custom_raw_adv_data.temp);                            // get new temperature
    int16_t temp_new = (int16_t) m_custom_raw_adv_data.temp;                   // convert from int32_t to int16_t
    //temp[0] = (uint8_t)((temp_new >> 2) & 0xFFUL); // Right-shift by two to remove decimal part
    //temp[1] = (uint8_t)((temp_new << 6) & 0xFFUL); // Left-shift 6 to get fractional part with 0.25 degrees C resolution

    temp_new = temp_new * 25; // temp in 10bits converted to 0.01C units (100*t/4) 
    m_custom_adv_data.temp[1] = (uint8_t)(temp_new >> 8);
    m_custom_adv_data.temp[0] = (uint8_t)temp_new;
}

static void increase_sec_cnt(void)
{
    m_custom_raw_adv_data.le_sec_cnt++;

    uint32_t be_sec_cnt = BYTES_REVERSE_32BIT(m_custom_raw_adv_data.le_sec_cnt);
    memcpy(m_custom_adv_data.sec_cnt, (uint8_t *)(&be_sec_cnt), 4);
}

static void update_vbatt_init(void) {
    uint32_t err_code;
    err_code = app_timer_start(m_battery_timer_id, UPDATE_ADV_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

int main(void) {
    // Initialize.
    #if NRF_LOG_ENABLED
    log_init();
    #endif
 
    nrf_power_dcdcen_set(true);

    timers_init();
    // buttons_leds_init();

    power_management_init();
    ble_stack_init();
    gap_params_init();
    battery_voltage_init();

    // Start execution.
    NRF_LOG_INFO("Long range Tile started");
    set_current_adv_params_and_start_advertising();
    update_vbatt_init();

    // Enter main loop.
    for (;;) {
        idle_state_handle();
    }
}

/**
 * @}
 */