#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <esp_mac.h>

// function definitions:

/*~~~~~Pin Mapping~~~~~*/


/*~~~~~Hardware Definitions~~~~~*/

// These are hardware specific to the Heltec WiFi LoRa 32 V4
// Cite: https://resource.heltec.cn/download/WiFi_LoRa_32_V4/Schematic/HTIT-WB32LAF_V4.3.pdf
#define PRG_BUTTON 0
#define LORA_NSS_PIN 8
#define LORA_SCK_PIN 9
#define LORA_MOSI_PIN 10
#define LORA_MISO_PIN 11
#define LORA_RST_PIN 12
#define LORA_BUSY_PIN 13
#define LORA_DIO1_PIN 14

/*~~~~~Radio Configuration~~~~~*/

// Initialize SX1262 radio
// Make a custom SPI device because *of course* Heltec didn't use the default SPI pins
SPIClass spi(FSPI);
SPISettings spiSettings(2000000, MSBFIRST, SPI_MODE0); // Defaults, works fine
SX1262 radio = new Module(LORA_NSS_PIN, LORA_DIO1_PIN, LORA_RST_PIN, LORA_BUSY_PIN, spi, spiSettings);

// make sure basestation is on the same channel
#define LORA_FREQ 915.0
#define LORA_BW 125.0
#define LORA_SF 9

/*~~~~~Global Variables~~~~~*/
volatile bool receivedFlag = false;

/*~~~~~Interrupts~~~~~*/
void IRAM_ATTR receiveISR(void)
{
  // WARNING:  No Flash memory may be accessed from the IRQ handler: https://stackoverflow.com/a/58131720
  //  So don't call any functions or really do anything except change the flag
  receivedFlag = true;
}

/*~~~~~Helper Functions~~~~~*/

void error_message(const char *message, int16_t state)
{
  Serial.printf("ERROR!!! %s with error code %d\n", message, state);
  while (true)
    ; // loop forever
}

void setup()
{
  Serial.begin(115200);
  delay(2000);
  /* Initialize Lora */
  // Set up SPI with our specific pins
  spi.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_NSS_PIN);

  Serial.print("Initializing radio...");
  int16_t state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, 5, 0x12, 15, 8);
  if (state != RADIOLIB_ERR_NONE)
  {
    error_message("Radio initialization failed", state);
  }

  state = radio.setCurrentLimit(140.0);
  if (state != RADIOLIB_ERR_NONE)
  {
    error_message("Current limit initialization failed", state);
  }

  state = radio.setDio2AsRfSwitch(true);
  if (state != RADIOLIB_ERR_NONE)
  {
    error_message("DIO2 RF switch initialization failed", state);
  }

  state = radio.explicitHeader();
  if (state != RADIOLIB_ERR_NONE)
  {
    error_message("Explicit header initialization failed", state);
  }

  state = radio.setCRC(2);
  if (state != RADIOLIB_ERR_NONE)
  {
    error_message("CRC initialization failed", state);
  }

  // set the function that will be called when a new packet is received
  radio.setDio1Action(receiveISR);

  // start continuous reception
  Serial.print("Beginning continuous reception...");
  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE)
  {
    error_message("Starting reception failed", state);
  }
  Serial.println("Complete!");
}

void loop()
{
  // Handle packet receptions
  if (receivedFlag)
  {
    receivedFlag = false;

    // data buffer
    uint8_t data[3];
    int state = radio.readData(data, sizeof(data));

    // parse data as id
    uint32_t id =
        ((uint32_t)data[0] << 16) |
        ((uint32_t)data[1] << 8) |
        data[2];

    if (state == RADIOLIB_ERR_NONE)
    {
      // packet was successfully received
      Serial.println("Received packet!");

      // print the data of the packet which is ID
      Serial.printf("Device:  %06x\n", id);

      // print the RSSI (Received Signal Strength Indicator)
      // of the last received packet
      Serial.print("\tRSSI:\t\t");
      Serial.print(radio.getRSSI());
      Serial.println(" dBm");
    }
    else if (state == RADIOLIB_ERR_RX_TIMEOUT)
    {
      // timeout occurred while waiting for a packet
      Serial.println("timeout!");
    }
    else if (state == RADIOLIB_ERR_CRC_MISMATCH)
    {
      // packet was received, but is malformed
      Serial.println("CRC error!");
    }
    else
    {
      // some other error occurred
      Serial.print("failed, code ");
      Serial.println(state);
    }

    // resume listening
    state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE)
    {
      error_message("Resuming reception failed", state);
    }
  }
}

