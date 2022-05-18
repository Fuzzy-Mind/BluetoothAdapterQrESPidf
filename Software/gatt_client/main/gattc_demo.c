/****************************************************************************
*
* Choistec BLE Adapter Firmware 
* - Added reconnect.
* - Added External DAC function.
* - Added ADC read for CNY70.
* - Added UART to read encrypted QR code.
*
****************************************************************************/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/gpio.h"    //  GPIO Driver
#include "driver/adc.h"     //  ADC Driver
#include "esp_adc_cal.h"    //  ADC Calibration
#include "driver/uart.h"    //  UART Driver

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define GATTC_TAG "GATTC_DEMO"
#define REMOTE_SERVICE_UUID        0xFFF0
#define REMOTE_NOTIFY_CHAR_UUID    0xFFF4
#define PROFILE_NUM      1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE   0

#define grn1 32
#define red1 33
#define grn2 25
#define red2 26
#define grn3 27
#define red3 14
#define grn4 12
#define red4 13

#define sda_pin 23
#define sck_pin 22
#define cs_pin  21
#define trg_pin 19

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

#define ECHO_TEST_TXD 17            // UART Tx Pin
#define ECHO_TEST_RXD 16            // UART Rx Pin
#define ECHO_TEST_RTS (-1)          // NO RTS
#define ECHO_TEST_CTS (-1)          // NO CTS

#define ECHO_UART_PORT_NUM      (2) // Port 2
#define ECHO_UART_BAUD_RATE     (9600)  // 9600 baudrate
#define ECHO_TASK_STACK_SIZE    (2048)  // 2048 stack size

#define BUF_SIZE (1024)             // 1024 buffer size

static esp_adc_cal_characteristics_t *adc_chars;

static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 ADC Pin
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;

static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;

static uint16_t adcData;
static uint16_t adcDataOld = 0;
static const int cnt = 10;
static const int dacOffset = 14;
static uint8_t decryptedBuffer[50];
static uint8_t decryptedMac[6];


//static const char device_mac_address[] = {0xd8, 0x71, 0x4d, 0xbc, 0xf5, 0xf1};
static const char remote_device_name[] = "Thermosafer 9F5F1";
static bool connect    = false;
static bool scanStatus = false;
static bool get_server = false;
static esp_gattc_char_elem_t *char_elem_result   = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;

/* Declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_SERVICE_UUID,},
};

static esp_bt_uuid_t remote_filter_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_NOTIFY_CHAR_UUID,},
};

static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,},
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

void ext_dac(uint8_t sdaPin, uint8_t sckPin, uint8_t csPin, uint16_t cmdData, int dacPeriodMs)
{
    gpio_set_level(csPin, false);
    bool data_ser=0;
    for(int i=0; i<16; i++)
    {
        data_ser = (cmdData<<i)&(0x8000);
        gpio_set_level(sdaPin, data_ser);
        vTaskDelay(dacPeriodMs / portTICK_PERIOD_MS);
        gpio_set_level(sckPin, true);
        vTaskDelay(dacPeriodMs / portTICK_PERIOD_MS);
        gpio_set_level(sckPin, false);
    }
    gpio_set_level(sdaPin, false);
    gpio_set_level(csPin, true);

}

static void echo_task(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    uint8_t incomingBuffer[50];
    static uint8_t incomingBufferOld[50];

    uint8_t firstBit = 0;
    uint8_t secondBit = 0;
    uint8_t thirdBit = 0;
    uint8_t fourthBit = 0;
    char incomingByte = 0;
    uint8_t hexDecrypted = 0;

    uint8_t hexCnt = 0;
    
    memset(incomingBufferOld, 0, 50);

    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        memcpy(incomingBuffer, data, len);
        if(memcmp(incomingBuffer, incomingBufferOld, 50) != 0){
            for(int i=0; i<len; i++){
                if(incomingBuffer[i] >= 48 && incomingBuffer[i] <= 57){         // "0" - "9" aralığında ise 48 çıkar.
                    decryptedBuffer[i] = incomingBuffer[i] - 48;
                }
                else if(incomingBuffer[i] >= 97 && incomingBuffer[i] <= 102){   // "a" - "f" aralığında ise 87 çıkar.
                    decryptedBuffer[i] = incomingBuffer[i] - 87;
                }
                if(i%2 == 0){
                    firstBit = (decryptedBuffer[i] & 8) >> 3;
                    secondBit = (decryptedBuffer[i] & 2) >> 1;
                }
                else{
                    thirdBit = (decryptedBuffer[i] & 8) >> 3;
                    fourthBit = (decryptedBuffer[i] & 2) >> 1;
                    incomingByte = (firstBit<<3) + (secondBit<<2) + (thirdBit<<1) + fourthBit;

                    if(hexCnt%2 == 0){
                        hexDecrypted = incomingByte << 4;
                    }
                    else{
                        hexDecrypted = hexDecrypted + incomingByte;
                        decryptedMac[hexCnt/2] = hexDecrypted;
                    }
                    hexCnt++;                    
                }
            }
            memcpy(incomingBufferOld, incomingBuffer, 50);
            for(int i=0; i<6; i++){
                printf("Decrypted MAC Address : %x\n", decryptedMac[i]);
            }
            
            hexCnt = 0;
        }
    }
    
}

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "REG_EVT");
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret){
            ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
        }
        break;
    case ESP_GATTC_CONNECT_EVT:{
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d", p_data->connect.conn_id, gattc_if);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "REMOTE BDA:");
        esp_log_buffer_hex(GATTC_TAG, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->connect.conn_id);
        if (mtu_ret){
            ESP_LOGE(GATTC_TAG, "config MTU error, error code = %x", mtu_ret);
        }
        break;
    }
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "open failed, status %d", p_data->open.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "open success");
        break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "discover service failed, status %d", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "discover service complete conn_id %d", param->dis_srvc_cmpl.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_uuid);
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG,"config mtu failed, error status = %x", param->cfg_mtu.status);
        }
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(GATTC_TAG, "SEARCH RES: conn_id = %x is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(GATTC_TAG, "start handle %d end handle %d current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
            ESP_LOGI(GATTC_TAG, "service found");
            get_server = true;
            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = p_data->search_res.end_handle;
            ESP_LOGI(GATTC_TAG, "UUID16: %x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }
        if(p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE) {
            ESP_LOGI(GATTC_TAG, "Get service information from remote device");
        } else if (p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH) {
            ESP_LOGI(GATTC_TAG, "Get service information from flash");
        } else {
            ESP_LOGI(GATTC_TAG, "unknown service source");
        }
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SEARCH_CMPL_EVT");
        if (get_server){
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     p_data->search_cmpl.conn_id,
                                                                     ESP_GATT_DB_CHARACTERISTIC,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                     INVALID_HANDLE,
                                                                     &count);
            if (status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
            }

            if (count > 0){
                char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result){
                    ESP_LOGE(GATTC_TAG, "gattc no mem");
                }else{
                    status = esp_ble_gattc_get_char_by_uuid( gattc_if,
                                                             p_data->search_cmpl.conn_id,
                                                             gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                             gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                             remote_filter_char_uuid,
                                                             char_elem_result,
                                                             &count);
                    if (status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_char_by_uuid error");
                    }

                    /*  Every service have only one char in our 'ESP_GATTS_DEMO' demo, so we used first 'char_elem_result' */
                    if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)){
                        gl_profile_tab[PROFILE_A_APP_ID].char_handle = char_elem_result[0].char_handle;
                        esp_ble_gattc_register_for_notify (gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, char_elem_result[0].char_handle);
                    }
                }
                /* free char_elem_result */
                free(char_elem_result);
            }else{
                ESP_LOGE(GATTC_TAG, "no char found");
            }
        }
         break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "REG FOR NOTIFY failed: error status = %d", p_data->reg_for_notify.status);
        }else{
            uint16_t count = 0;
            uint16_t notify_en = 1;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                         ESP_GATT_DB_DESCRIPTOR,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                                         &count);
            if (ret_status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
            }
            if (count > 0){
                descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result){
                    ESP_LOGE(GATTC_TAG, "malloc error, gattc no mem");
                }else{
                    ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                         p_data->reg_for_notify.handle,
                                                                         notify_descr_uuid,
                                                                         descr_elem_result,
                                                                         &count);
                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                    }
                    /* Every char has only one descriptor in our 'ESP_GATTS_DEMO' demo, so we used first 'descr_elem_result' */
                    if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG){
                        ret_status = esp_ble_gattc_write_char_descr( gattc_if,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                     descr_elem_result[0].handle,
                                                                     sizeof(notify_en),
                                                                     (uint8_t *)&notify_en,
                                                                     ESP_GATT_WRITE_TYPE_RSP,
                                                                     ESP_GATT_AUTH_REQ_NONE);
                    }

                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_write_char_descr error");
                    }

                    /* free descr_elem_result */
                    free(descr_elem_result);
                }
            }
            else{
                ESP_LOGE(GATTC_TAG, "decsr not found");
            }

        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify){
            ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, receive notify value:\n");
        }else{
            ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, receive indicate value:\n");
        }
        /////////////// MY Code Starts ////////////////////

        char myBuffer[40];
        uint8_t battValue;
        uint16_t tempValue;
        uint16_t dacValue = 0;
        float tempFloat;

        memcpy(myBuffer, p_data->notify.value, p_data->notify.value_len);
        battValue = myBuffer[1];

        tempValue = myBuffer[2];
        tempValue = (tempValue<<8) + myBuffer[3];
        tempFloat = tempValue/100.00;

        ESP_LOGI(GATTC_TAG, "Battery     : %%%d", battValue);
        ESP_LOGI(GATTC_TAG, "Temperature : %.2f\n", tempFloat);

        if(battValue >= 75){
            gpio_set_level(grn1, false);
            gpio_set_level(red1, true);
            gpio_set_level(grn2, false);
            gpio_set_level(red2, true);
            gpio_set_level(grn3, false);
            gpio_set_level(red3, true);
            gpio_set_level(grn4, false);
            gpio_set_level(red4, true);
        }
        else if(battValue >= 50){
            gpio_set_level(grn1, true);
            gpio_set_level(red1, true);
            gpio_set_level(grn2, false);
            gpio_set_level(red2, true);
            gpio_set_level(grn3, false);
            gpio_set_level(red3, true);
            gpio_set_level(grn4, false);
            gpio_set_level(red4, true);
        }
        else if(battValue >= 25){
            gpio_set_level(grn1, true);
            gpio_set_level(red1, true);
            gpio_set_level(grn2, true);
            gpio_set_level(red2, true);
            gpio_set_level(grn3, false);
            gpio_set_level(red3, true);
            gpio_set_level(grn4, false);
            gpio_set_level(red4, true);        
        }
        else{
            gpio_set_level(grn1, true);
            gpio_set_level(red1, true);
            gpio_set_level(grn2, true);
            gpio_set_level(red2, true);
            gpio_set_level(grn3, true);
            gpio_set_level(red3, true);
            gpio_set_level(grn4, false);
            gpio_set_level(red4, true);
            
        }

        //ESP_LOGI(GATTC_TAG, "Temp Value : %d\n", tempValue);

        if(1660<=tempValue && tempValue<=2069){
            dacValue = 46*tempFloat+29953.4 + dacOffset;
            for(int i=0; i<cnt; i++){
                ext_dac(sda_pin, sck_pin, cs_pin, dacValue, 0.1);
            }
        }
        if(2070<=tempValue && tempValue<=2479){
            dacValue = 45.25*tempFloat+29968.325 + dacOffset;
            for(int i=0; i<cnt; i++){
                ext_dac(sda_pin, sck_pin, cs_pin, dacValue, 0.1);
            }
        }
        if(2480<=tempValue && tempValue<=2889){
        dacValue = 42.25*tempFloat+30043.2 + dacOffset;
            for(int i=0; i<cnt; i++){
                ext_dac(sda_pin, sck_pin, cs_pin, dacValue, 0.1);
            }
        }
        if(2890<=tempValue && tempValue<=3299){
            dacValue = 39*tempFloat+30136.9 + dacOffset;
            for(int i=0; i<cnt; i++){
                ext_dac(sda_pin, sck_pin, cs_pin, dacValue, 0.1);
            }
        }
        if(3300<=tempValue && tempValue<=3669){
            dacValue = 36.38*tempFloat+30223.46 + dacOffset;
            for(int i=0; i<cnt; i++){
                ext_dac(sda_pin, sck_pin, cs_pin, dacValue, 0.1);
            }
        }
        if(3670<=tempValue)
        {
            dacValue = 33.63*tempFloat+30223.78 + dacOffset;
            for(int i=0; i<cnt; i++){
            ext_dac(sda_pin, sck_pin, cs_pin, dacValue, 0.1);
            }
        }

        //ESP_LOGI(GATTC_TAG, "DAC Value : %d\n", dacValue);

        //////////////// My Code Ends //////////////////

        esp_log_buffer_hex(GATTC_TAG, p_data->notify.value, p_data->notify.value_len);
        break;
    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
        esp_log_buffer_hex(GATTC_TAG, bda, sizeof(esp_bd_addr_t));
        break;
    }
    case ESP_GATTC_DISCONNECT_EVT:
        connect = false;
        get_server = false;
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
        break;
    default:
        break;
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        //the unit of the duration is second
        uint32_t duration = 30;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTC_TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "scan start success");
        scanStatus = true;
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            esp_log_buffer_hex(GATTC_TAG, scan_result->scan_rst.bda, 6);

            /////////// Check MAC Address ////////////////

            char macBuffer[12];
            memcpy(macBuffer, scan_result->scan_rst.bda, 6);

            if(memcmp(decryptedMac, macBuffer, 6) == 0){
                ESP_LOGI(GATTC_TAG, "\nMAC Address  : %s\n", macBuffer);
                if (connect == false) {
                    connect = true;
                    ESP_LOGI(GATTC_TAG, "connect to the remote device.");
                    esp_ble_gap_stop_scanning();
                    esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                }
            }

            /////////// Check MAC Address ////////////////

            //ESP_LOGI(GATTC_TAG, "searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            //ESP_LOGI(GATTC_TAG, "searched Device Name Len %d", adv_name_len);
            esp_log_buffer_char(GATTC_TAG, adv_name, adv_name_len);

#if CONFIG_EXAMPLE_DUMP_ADV_DATA_AND_SCAN_RESP
            if (scan_result->scan_rst.adv_data_len > 0) {
                ESP_LOGI(GATTC_TAG, "adv data:");
                esp_log_buffer_hex(GATTC_TAG, &scan_result->scan_rst.ble_adv[0], scan_result->scan_rst.adv_data_len);
            }
            if (scan_result->scan_rst.scan_rsp_len > 0) {
                ESP_LOGI(GATTC_TAG, "scan resp:");
                esp_log_buffer_hex(GATTC_TAG, &scan_result->scan_rst.ble_adv[scan_result->scan_rst.adv_data_len], scan_result->scan_rst.scan_rsp_len);
            }
#endif
            //ESP_LOGI(GATTC_TAG, "\n");

            if (adv_name != NULL) {

                if (strlen(remote_device_name) == adv_name_len && strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0) {
                    ESP_LOGI(GATTC_TAG, "searched device %s\n", remote_device_name);
                    /*if (connect == false) {
                        connect = true;
                        ESP_LOGI(GATTC_TAG, "connect to the remote device.");
                        esp_ble_gap_stop_scanning();
                        esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                    }*/
                }
            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            break;
        default:
            break;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "scan stop failed, error status = %x", param->scan_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "stop scan successfully");
        scanStatus = false;
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "adv stop failed, error status = %x", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "stop adv successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(GATTC_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(GATTC_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

uint16_t read_adc_value(){
    uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, width, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        //uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        return adc_reading;
}

void app_main(void)
{
    ///////////// INITIALIZE GPIOS ///////////////////////

    gpio_reset_pin(grn1);
    gpio_reset_pin(red1);
    gpio_reset_pin(grn2);
    gpio_reset_pin(red2);
    gpio_reset_pin(grn3);
    gpio_reset_pin(red3);
    gpio_reset_pin(grn4);
    gpio_reset_pin(red4);

    gpio_reset_pin(sda_pin);
    gpio_reset_pin(sck_pin);
    gpio_reset_pin(cs_pin);
    gpio_reset_pin(trg_pin);

    gpio_set_direction(grn1, GPIO_MODE_OUTPUT);
    gpio_set_direction(red1, GPIO_MODE_OUTPUT);
    gpio_set_direction(grn2, GPIO_MODE_OUTPUT);
    gpio_set_direction(red2, GPIO_MODE_OUTPUT);
    gpio_set_direction(grn3, GPIO_MODE_OUTPUT);
    gpio_set_direction(red3, GPIO_MODE_OUTPUT);
    gpio_set_direction(grn4, GPIO_MODE_OUTPUT);
    gpio_set_direction(red4, GPIO_MODE_OUTPUT);

    gpio_set_direction(sda_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(sck_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(cs_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(trg_pin, GPIO_MODE_OUTPUT);

    gpio_set_level(grn1, true);
    gpio_set_level(red1, false);
    gpio_set_level(grn2, true);
    gpio_set_level(red2, false);
    gpio_set_level(grn3, true);
    gpio_set_level(red3, false);
    gpio_set_level(grn4, true);
    gpio_set_level(red4, false);

    gpio_set_level(sda_pin, false);
    gpio_set_level(sck_pin, false);
    gpio_set_level(cs_pin, true);
    gpio_set_level(trg_pin, true);

    /////////////////////// Configure ADC ///////////////////////

    adc1_config_width(width);
    adc1_config_channel_atten(channel, atten);

    ///////////////////// Characterize ADC /////////////////////

    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    //esp_adc_cal_value_t val_type = 

    ///////////////////// UART Task Create /////////////////////

    xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);

    // Initialize NVS.

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

//startLabel :
    ESP_LOGI(GATTC_TAG, "INIT BLUETOOTH");
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    //register the  callback function to the gap module
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret){
        ESP_LOGE(GATTC_TAG, "%s gap register failed, error code = %x\n", __func__, ret);
        return;
    }

    //register the callback function to the gattc module
    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if(ret){
        ESP_LOGE(GATTC_TAG, "%s gattc register failed, error code = %x\n", __func__, ret);
        return;
    }

    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret){
        ESP_LOGE(GATTC_TAG, "%s gattc app register failed, error code = %x\n", __func__, ret);
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTC_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    uint8_t timeOutCnt = 0;

    while(1){
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if(connect == 0){
            gpio_set_level(grn1, true);
            gpio_set_level(red1, false);
            gpio_set_level(grn2, true);
            gpio_set_level(red2, false);
            gpio_set_level(grn3, true);
            gpio_set_level(red3, false);
            gpio_set_level(grn4, true);
            gpio_set_level(red4, false);

            ext_dac(sda_pin, sck_pin, cs_pin, 0, 1);

            adcData = read_adc_value();
            if(adcData>adcDataOld){
                if((adcData-adcDataOld)>50){
                    ESP_LOGI(GATTC_TAG, "Yakın Tetikle");
                    gpio_set_level(trg_pin, false);
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                }
                gpio_set_level(trg_pin, true);
            }
            else{
                if((adcDataOld-adcData)>50){
                    ESP_LOGI(GATTC_TAG, "Uzak Tetikle");
                    gpio_set_level(trg_pin, false);
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                }
                gpio_set_level(trg_pin, true);
            }

            adcDataOld = adcData;
            //printf("ADC Raw: %d\n", adcData );
        }
        //ESP_LOGI(GATTC_TAG, "Connect Flag = %d", connect);
        //for(int k=0; k<6; k++){
        //    printf("%d. Decrypted MAC = %d\n", k, decryptedMac[k]);
        //}
        //ESP_LOGI(GATTC_TAG, "Decrypted Buufer = %d\n", incomingBufferOld);
        if(connect == 0 && timeOutCnt>30){
            //ESP_LOGI(GATTC_TAG, "Connection Failed");
            esp_ble_gap_stop_scanning();
            vTaskDelay(100 / portTICK_PERIOD_MS);
            esp_ble_gap_start_scanning(30);
            timeOutCnt = 0;
        }
        timeOutCnt++;
    }

}
