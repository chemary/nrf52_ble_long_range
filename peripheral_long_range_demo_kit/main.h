


#define BYTES_REVERSE_32BIT(x)                                                                     \
    ((x << 24 | ((x << 8) & 0x00FF0000)) | (((x >> 8) & 0x0000FF00) | x >> 24))


#define CONN_INTERVAL_DEFAULT (uint16_t)(MSEC_TO_UNITS(7.5, UNIT_1_25_MS)) /**< Default connection interval used at connection establishment by central side. */
#define CONN_INTERVAL_MIN (uint16_t)(MSEC_TO_UNITS(7.5, UNIT_1_25_MS))     /**< Minimum acceptable connection interval, in 1.25 ms units. */
#define CONN_INTERVAL_MAX (uint16_t)(MSEC_TO_UNITS(500, UNIT_1_25_MS))     /**< Maximum acceptable connection interval, in 1.25 ms units. */
#define CONN_SUP_TIMEOUT (uint16_t)(MSEC_TO_UNITS(8000, UNIT_10_MS))       /**< Connection supervisory timeout (4 seconds). */

#define SLAVE_LATENCY 0 /**< Slave latency. */

#define NON_CONN_OR_CONN_ADV_BUTTON BSP_BUTTON_0
#define NON_CONN_OR_CONN_ADV_BUTTON_EVENT BSP_EVENT_KEY_0

#define UPDATE_ADV_INTERVAL APP_TIMER_TICKS(1*1000) // 1s timer to update sensor values and count time

#define APP_BLE_CONN_CFG_TAG 1  /**< A tag that refers to the BLE stack configuration. */
#define APP_BLE_OBSERVER_PRIO 3 /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define CUT_OFF_VOLTAGE_3000 3000 // Common value for CR2450 and other batteries
#define CUT_OFF_VOLTAGE_LIPO 3600
#define CUT_OFF_VOLTAGE CUT_OFF_VOLTAGE_3000

#define LOW_POWER_VOLTAGE 3400 // Enter low power mode

#define CUT_OFF_VOLTAGE_POWER_UP_DELAY 30 // Don't shut down just after power on

// Type holding the two output power options for this application.
typedef enum {
    SELECTION_0_dBm = 0,
    SELECTION_8_dBm = 8
} output_power_seclection_t;

// Type holding the two advertising selection modes.
typedef enum {
    SELECTION_CONNECTABLE = 0,
    SELECTION_NON_CONNECTABLE
} adv_scan_type_seclection_t;

// Type holding the two possible phy options.
typedef enum {
    SELECTION_1M_PHY = 0,
    SELECTION_CODED_PHY
} adv_scan_phy_seclection_t;

// Type holding the power modes modes.
typedef enum {
    POWER_MODE_NORMAL = 0,
    POWER_MODE_LOW_BATT
} power_mode_t;

typedef struct
{
    uint8_t vbatt[1];    // Variable to hold voltage reading
    uint8_t temp[2];     // variable to hold temp reading
    uint8_t sec_cnt[4];  // Time since power-on or reboot.
} custom_adv_data_t;

typedef struct
{
    uint16_t vbatt;      // Variable to hold voltage reading
    int32_t temp;        // variable to hold temp reading
    uint32_t le_sec_cnt; // Time since power-on or reboot.
} custom_raw_adv_data_t;


static void log_init(void);
static void advdata_update(void);
static void set_current_adv_params_and_start_advertising(void);
static void update_vbatt(void);
static void update_temp(void);
static void update_time(void);
static void increase_sec_cnt(void);
static void disconnect_stop_adv(void);
static void advertising_start(void);
static void advertising_data_set(bool set_adv_params);