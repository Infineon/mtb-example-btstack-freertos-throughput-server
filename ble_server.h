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
 * (c) 2021-2026, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 *
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
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

