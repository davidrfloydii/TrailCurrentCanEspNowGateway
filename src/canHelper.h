#pragma once
#include "globals.h"

#define CAN_RX 13
#define CAN_TX 15
// Interval:
#define POLLING_RATE_MS 33
static bool driver_installed = false;

namespace canHelper
{
    void initialize()
    {
        // Initialize configuration structures using macro initializers
        twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NO_ACK);
        twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // Look in the api-reference for other speed sets.
        // Filter to only listen for messages from a single extended can identifier. Will need to be changed once we have
        // two different senders. One for on/off and another for 0 >> 255 values.
        twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        // Install TWAI driver
        if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
        {
            debugln("Driver installed");
        }
        else
        {
            // debugln("Failed to install driver");
            return;
        }

        // Start TWAI driver
        if (twai_start() == ESP_OK)
        {
            // debugln("Driver started");
        }
        else
        {
            // debugln("Failed to start driver");
            return;
        }

        // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
        uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
        if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK)
        {
            // ebugln("CAN Alerts reconfigured");
        }
        else
        {
            // debugln("Failed to reconfigure alerts");
            return;
        }

        // TWAI driver is now successfully installed and started
        driver_installed = true;
    }

    static void handle_rx_message(twai_message_t &message)
    {
        outgoingMessage.identifier = message.identifier;
        outgoingMessage.data_length_code = message.data_length_code;
        if (message.data_length_code = 8)
        {
            outgoingMessage.dataByte7 = message.data[7];
        }
        else
        {
            outgoingMessage.dataByte7 = 0x0;
        }
        if (message.data_length_code = 7)
        {
            outgoingMessage.dataByte6 = message.data[6];
        }
        else
        {
            outgoingMessage.dataByte6 = 0x0;
        }
        if (message.data_length_code = 6)
        {
            outgoingMessage.dataByte5 = message.data[5];
        }
        else
        {
            outgoingMessage.dataByte5 = 0x0;
        }
        if (message.data_length_code = 5)
        {
            outgoingMessage.dataByte4 = message.data[4];
        }
        else
        {
            outgoingMessage.dataByte4 = 0;
        }
        if (message.data_length_code = 4)
        {
            outgoingMessage.dataByte3 = message.data[3];
        }
        else
        {
            outgoingMessage.dataByte3 = 0x0;
        }
        if (message.data_length_code = 3)
        {
            outgoingMessage.dataByte2 = message.data[2];
        }
        else
        {
            outgoingMessage.dataByte2 = 0x0;
        }
        if (message.data_length_code = 2)
        {
            outgoingMessage.dataByte1 = message.data[1];
        }
        else
        {
            outgoingMessage.dataByte1 = 0x0;
        }
        if (message.data_length_code = 1)
        {
            outgoingMessage.dataByte0 = message.data[0];
        }
        else
        {
            outgoingMessage.dataByte0 = 0;
        }
        newDataToSend = true;
    }

    void checkCanBusForMessages()
    {
        // Check if alert happened
        uint32_t alerts_triggered;
        twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
        twai_status_info_t twaistatus;
        twai_get_status_info(&twaistatus);

        // Check if message is received
        if (alerts_triggered & TWAI_ALERT_RX_DATA)
        {
            // One or more messages received. Handle all.
            twai_message_t message;
            while (twai_receive(&message, 0) == ESP_OK)
            {
                handle_rx_message(message);
            }
        }
    }
}