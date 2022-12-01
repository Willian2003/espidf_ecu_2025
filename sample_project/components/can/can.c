#include "can.h"

void can_preconf()
{

    // Initialize configuration structures using macro initializers
    can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_22, ESP_INTR_FLAG_IRAM);
    can_timing_config_t t_config = CAN_TIMING_CONFIG_1MBITS();
    can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

    // Install CAN driver
    if (can_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
    {
        printf("Driver installed\n");
    }
    else
    {
        printf("Failed to install driver\n");
        return;
    }

    // Start CAN driver
    if (can_start() == ESP_OK)
    {
        printf("Driver started\n");
    }
    else
    {
        printf("Failed to start driver\n");
        return;
    }

    return;
}

void receive_CAN(void *pvParameters)
{

    // Wait for message to be received
    can_message_t message;
    if (can_receive(&message, pdMS_TO_TICKS(10000)) == ESP_OK)
    {
        printf("Message received\n");
    }
    else
    {
        printf("Failed to receive message\n");
        return;
    }

    // Process received message
    if (message.flags & CAN_MSG_FLAG_EXTD)
    {
        printf("Message is in Extended Format\n");
    }
    else
    {
        printf("Message is in Standard Format\n");
    }
    printf("ID is %d\n", message.identifier);
    if (!(message.flags & CAN_MSG_FLAG_RTR))
    {
        for (int i = 0; i < message.data_length_code; i++)
        {
            printf("Data byte %d = %d\n", i, message.data[i]);
        }
    }
}