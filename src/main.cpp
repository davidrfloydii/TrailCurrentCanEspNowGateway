#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "driver/twai.h"
// Pins used to connect to CAN bus transceiver:
#define RX_PIN 14
#define TX_PIN 27
#define CAN_LISTEN_TO_MESSAGE_ID 0x1B
#define CAN_SEND_MESSAGE_LIGHT_CHANGE_IDENTIFIER 0x15

unsigned long canStartMillis;
unsigned long canCurrentMillis;
const unsigned long canStatusPeriod = 33;

int device01CurrentState = 0;
int device02CurrentState = 0;
int device03CurrentState = 0;
int device04CurrentState = 0;
int device05CurrentState = 0;
int device06CurrentState = 0;
int device07CurrentState = 0;
int device08CurrentState = 0;

typedef struct struct_message
{
  int device01State;
  int device02State;
  int device03State;
  int device04State;
  int device05State;
  int device06State;
  int device07State;
  int device08State;
  int turnAllOffCommand;
  int turnAllOnCommand;
} struct_message;

struct_message trailerStatusData;
struct_message carStateChangeData;
esp_now_peer_info_t peerInfo;
// REPLACE WITH THE MAC Address of your receiver
uint8_t broadcastAddress[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// Variable to store if sending data was successful
String success;

// Interval:
#define POLLING_RATE_MS 33

static bool driver_installed = false;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0)
  {
    success = "Delivery Success :)";
  }
  else
  {
    success = "Delivery Fail :(";
  }
}

static void send_power_message(int btn, int desiredValue)
{
  // Configure message to transmit
  twai_message_t message;
  message.identifier = 0x15;
  message.extd = false; // Using CAN 2.0 extended id allowing up to 536870911 identifiers
  message.rtr = false;
  message.data_length_code = 2;
  message.data[0] = btn;
  message.data[1] = desiredValue;
  Serial.print("Value Sent: ");
  Serial.print(btn);
  Serial.println(" ");
  Serial.println(desiredValue);
  // Queue message for transmission
  if (twai_transmit(&message, pdMS_TO_TICKS(10)) == ESP_OK)
  {
    // Serial.println("Message queued for transmission");
  }
  else
  {
    // Serial.println("Failed to queue message for transmission");
  }
}

static void send_toggle_message(int btn, int desiredValue)
{
  // Configure message to transmit
  twai_message_t message;
  message.identifier = 0x18;
  message.extd = false; // Using CAN 2.0 extended id allowing up to 536870911 identifiers
  message.rtr = false;
  message.data_length_code = 2;
  message.data[0] = btn;
  message.data[1] = desiredValue;
  Serial.print("Value Sent: ");
  Serial.print(btn);
  Serial.println(" ");
  Serial.println(desiredValue);
  // Queue message for transmission
  if (twai_transmit(&message, pdMS_TO_TICKS(10)) == ESP_OK)
  {
    // Serial.println("Message queued for transmission");
  }
  else
  {
    // Serial.println("Failed to queue message for transmission");
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&carStateChangeData, incomingData, sizeof(carStateChangeData));
  Serial.print("Value Received: ");
  Serial.println(carStateChangeData.turnAllOffCommand);

  if (carStateChangeData.device01State != device01CurrentState)
  {
    send_power_message(0, carStateChangeData.device01State);
  }
  else if (carStateChangeData.device02State != device02CurrentState)
  {
    send_power_message(1, carStateChangeData.device02State);
  }
  else if (carStateChangeData.device03State != device03CurrentState)
  {
    send_power_message(2, carStateChangeData.device03State);
  }
  else if (carStateChangeData.device04State != device04CurrentState)
  {
    send_power_message(3, carStateChangeData.device04State);
  }
  else if (carStateChangeData.device04State != device05CurrentState)
  {
    send_power_message(4, carStateChangeData.device05State);
  }
  else if (carStateChangeData.device04State != device06CurrentState)
  {
    send_power_message(5, carStateChangeData.device06State);
  }
  else if (carStateChangeData.device04State != device07CurrentState)
  {
    send_power_message(6, carStateChangeData.device07State);
  }
  else if (carStateChangeData.device04State != device08CurrentState)
  {
    send_power_message(7, carStateChangeData.device08State);
  }
  else if (carStateChangeData.turnAllOffCommand == 1)
  {
    send_toggle_message(8, 0);
  }
  else if (carStateChangeData.turnAllOnCommand == 1)
  {
    Serial.println("Got the message to turn all on");
    send_toggle_message(9, 1);
  }
}

void setup()
{
  // Start Serial:
  Serial.begin(115200);

  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NO_ACK);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // Look in the api-reference for other speed sets.
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
  {
    Serial.println("Driver installed");
  }
  else
  {
    Serial.println("Failed to install driver");
    return;
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK)
  {
    Serial.println("Driver started");
  }
  else
  {
    Serial.println("Failed to start driver");
    return;
  }

  // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK)
  {
    Serial.println("CAN Alerts reconfigured");
  }
  else
  {
    Serial.println("Failed to reconfigure alerts");
    return;
  }

  // TWAI driver is now successfully installed and started
  driver_installed = true;

  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
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

static void handle_rx_message(twai_message_t &message)
{
  // Process received message
  if (message.extd)
  {
    // Serial.println("Message is in Extended Format");
  }
  else
  {
    // Serial.println("Message is in Standard Format");
  }
  if (!(message.rtr))
  {
    if (message.identifier == 27)
    {
      device01CurrentState = message.data[0];
      device02CurrentState = message.data[1];
      device03CurrentState = message.data[2];
      device04CurrentState = message.data[3];
      device05CurrentState = message.data[4];
      device06CurrentState = message.data[5];
      device07CurrentState = message.data[6];
      device08CurrentState = message.data[7];
    }
    trailerStatusData.device01State = device01CurrentState;
    trailerStatusData.device02State = device02CurrentState;
    trailerStatusData.device03State = device03CurrentState;
    trailerStatusData.device04State = device04CurrentState;
    trailerStatusData.device05State = device05CurrentState;
    trailerStatusData.device06State = device06CurrentState;
    trailerStatusData.device07State = device07CurrentState;
    trailerStatusData.device08State = device08CurrentState;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&trailerStatusData, sizeof(trailerStatusData));

    if (result == ESP_OK)
    {
      // Serial.println("Sent with success");
    }
    else
    {
      // Serial.println("Error sending the data");
    }
    // Serial.println("");
  }
}

void loop()
{
  canCurrentMillis = millis();
  if (canCurrentMillis - canStartMillis >= canStatusPeriod)
  {
    // Check if alert happened
    uint32_t alerts_triggered;
    twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
    twai_status_info_t twaistatus;
    twai_get_status_info(&twaistatus);

    // Handle alerts
    if (alerts_triggered & TWAI_ALERT_ERR_PASS)
    {
      Serial.println("Alert: TWAI controller has become error passive.");
    }
    if (alerts_triggered & TWAI_ALERT_BUS_ERROR)
    {
      Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
      Serial.printf("Bus error count: %lu\n", twaistatus.bus_error_count);
    }
    if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL)
    {
      Serial.println("Alert: The RX queue is full causing a received frame to be lost.");
      Serial.printf("RX buffered: %lu\t", twaistatus.msgs_to_rx);
      Serial.printf("RX missed: %lu\t", twaistatus.rx_missed_count);
      Serial.printf("RX overrun %lu\n", twaistatus.rx_overrun_count);
    }

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
    canStartMillis = canCurrentMillis;
  }
}
