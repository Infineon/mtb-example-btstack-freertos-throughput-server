/******************************************************************************
* File Name: main.c
*
* Description: This is the source code for the BTSTACK based FreeRTOS
*              LE GATT Server Throughput Example for ModusToolbox.
*
* Related Document: See README.md
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
*******************************************************************************/

/*******************************************************************************
*        Header Files
*******************************************************************************/
#include "ble_server.h"
#include "wiced_bt_stack.h"
#include "cy_retarget_io.h"
#include <FreeRTOS.h>
#include <task.h>
#include "cycfg_bt_settings.h"
#include "cybsp_bt_config.h"

/*******************************************************************************
*        Macros
*******************************************************************************/
#define TASK_PRIORITY                   ( configMAX_PRIORITIES - 4 )
#define TASK_STACK_SIZE                 ( configMINIMAL_STACK_SIZE * 4 )
#define THROUGHPUT_TASK_STRING                 "Throughput Task"
#define NOTIFICATION_TASK_STRING               "Notification Task"

/*******************************************************************************
*        Global Variables
*******************************************************************************/
/*Handle for the task*/
TaskHandle_t notif_send_task_handle;
TaskHandle_t get_throughput_task_handle;

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
int main()
{
    cy_rslt_t bsp_result = CY_RSLT_SUCCESS;
    wiced_result_t result = WICED_SUCCESS;
    BaseType_t rtos_result;

    /* Initialize the board support package */
    bsp_result = cybsp_init();
    if (CY_RSLT_SUCCESS != bsp_result)
    {
        CY_ASSERT(0);
    }
    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX,
                        CYBSP_DEBUG_UART_RX,
                        CY_RETARGET_IO_BAUDRATE);
    printf("**** BLE Throughput: GATT Server Application Start ****\n\n");

    /* Initialize bluetooth porting layer by setting HCI configuration */
    cybt_platform_config_init(&cybsp_bt_platform_cfg);

    /* Register call back and configuration with stack */
    result = wiced_bt_stack_init(app_bt_management_callback,
                                    &wiced_bt_cfg_settings);

    /* Check if stack initialization was successful */
    if( WICED_BT_SUCCESS == result)
    {
        printf("Bluetooth Stack Initialization Successful \n");
    }
    else
    {
        printf("Bluetooth Stack Initialization failed!! \n");
        CY_ASSERT(0);
    }

    /* Create a task to calculate throughput*/
    rtos_result = xTaskCreate(get_throughput_task, THROUGHPUT_TASK_STRING,
                              TASK_STACK_SIZE, NULL, TASK_PRIORITY,
                              &get_throughput_task_handle);
    if(pdPASS != rtos_result)
    {
        printf("Throughput Task creation failed\n");
        CY_ASSERT(0);
    }

    /* Create a task to send notifications */
    rtos_result = xTaskCreate(send_notification_task, NOTIFICATION_TASK_STRING,
                              TASK_STACK_SIZE, NULL, TASK_PRIORITY,
                              &notif_send_task_handle);
    if(pdPASS != rtos_result)
    {
        printf("Notification Task creation failed\n");
        CY_ASSERT(0);
    }

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

    /* The application should never reach here */
    CY_ASSERT(0) ;
}

/* [] END OF FILE */
