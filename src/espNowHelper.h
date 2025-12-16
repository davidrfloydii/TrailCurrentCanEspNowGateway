#pragma once
#include "globals.h"

esp_now_peer_info_t peerInfo;

esp_now_message_t incomingMessage;

String success;

namespace espNowHelper
{
    // Callback when data is received
    void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
    {
        memcpy(&incomingMessage, incomingData, sizeof(incomingMessage));
    }

    // Callback when data is sent
    void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
    {
        if (status == 0)
        {
            success = "Delivery Success :)";
        }
        else
        {
            success = "Delivery Fail :(";
        }
    }

    uint8_t getMacAddress(uint8_t *mac)
    {
        return esp_wifi_get_mac(WIFI_IF_STA, mac);
    }

    void sendData()
    {
        for (int i = 0; i < outgoingMessage.data_length_code; i++)
        {           
            uint8_t address = *(&outgoingMessage.dataByte0 + i);
        }
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&outgoingMessage, sizeof(outgoingMessage));
        if (result == ESP_OK)
        {
            debugln("Sent with success");
        }
        else
        {
            debugln("Error sending the data");
        }
    }

    void initialize()
    {
        WiFi.mode(WIFI_STA);
        // Init ESP-NOW
        if (esp_now_init() != ESP_OK)
        {
            Serial.println("Error initializing ESP-NOW");
            return;
        }

        // Once ESPNow is successfully Init, we will register for Send CB to
        // get the status of Trasnmitted packet
        esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));

        // Register peer
        memcpy(peerInfo.peer_addr, broadcastAddress, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;

        // Add peer
        if (esp_now_add_peer(&peerInfo) != ESP_OK)
        {
            Serial.println("Failed to add peer");
            return;
        }
        // Register for a callback function that will be called when data is received
        esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
    }
}