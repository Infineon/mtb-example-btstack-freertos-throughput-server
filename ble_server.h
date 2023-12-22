/*******************************************************************************
 * File Name: ble_server.h
 *
 * Description: This file consists of the utility functions that will help
*              debugging and developing the applications easier with much
*              more meaningful information.
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
 ******************************************************************************/

#include "wiced_bt_stack.h"
#include "cybsp.h"
#include <FreeRTOS.h>
#include <task.h>
#include "cybt_platform_trace.h"
#include "wiced_memory.h"
#include "app_bt_utils.h"
#include "wiced_bt_ble.h"

/*******************************************************************************
*        Macro Definitions
*******************************************************************************/

/* PWM frequency of LED's in Hz when blinking */
#define ADV_LED_PWM_FREQUENCY           (1)
#define CONNECTION_INTERVAL_P6BLE       (30)
#define CONNECTION_INTERVAL_20829       (22)
#define SUPERVISION_TIMEOUT             (1000)
#define CONN_INTERVAL_MULTIPLIER        (1.25f)
#define FREQUENCY                      (10000)
#define TIMER_INTERRUPT_PRIORITY        (3)

/* Data packet sizes when 247 <= ATT MTU <= 498 */
#define DATA_PACKET_SIZE_1               (244u)
#define DATA_PACKET_SIZE_2               (495u)
#define ATT_HEADER                       (3u)

/* PWM Duty Cycle of LED's for different states */
typedef enum
{
    LED_ON_DUTY_CYCLE = 0,
    LED_BLINKING_DUTY_CYCLE= 50,
    LED_OFF_DUTY_CYCLE = 100
} led_duty_cycles;

/* This enumeration combines the advertising, connection states from two different
 * callbacks to maintain the status in a single state variable */
typedef enum
{
    APP_BT_ADV_OFF_CONN_OFF,
    APP_BT_ADV_ON_CONN_OFF,
    APP_BT_ADV_OFF_CONN_ON
} app_bt_adv_conn_mode_t;

/*******************************************************************************
*        Structures
*******************************************************************************/

typedef struct
{
    wiced_bt_device_address_t             remote_addr;   /* remote peer device address */
    uint16_t                              conn_id;       /* connection ID referenced by the stack */
    uint16_t                              mtu;           /* MTU exchanged after connection */
    double                                conn_interval; /* connection interval negotiated */
    wiced_bt_ble_host_phy_preferences_t   rx_phy;        /* RX PHY selected */
    wiced_bt_ble_host_phy_preferences_t   tx_phy;        /* TX PHY selected */

} conn_state_info_t;

extern TaskHandle_t get_throughput_task_handle;
extern TaskHandle_t notif_send_task_handle;

/*******************************************************************************
*        Function Prototypes
*******************************************************************************/

/* Callback function for Bluetooth stack management type events */
wiced_bt_dev_status_t  app_bt_management_callback(wiced_bt_management_evt_t event,
                                     wiced_bt_management_evt_data_t *p_event_data);
void get_throughput_task(void *pvParam);
void send_notification_task(void *pvParam);

