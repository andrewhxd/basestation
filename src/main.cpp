#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>
#include <esp_mac.h>
#include <map>
#include <U8g2lib.h>

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

// GC1109 front-end enable (CSD)
#define FEM_EN 2

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

/*~~~~~Screen Configuration~~~~~*/

#define OLED_RESET 21
#define OLED_SDA 17
#define OLED_SCL 18
U8G2_SSD1306_128X64_NONAME_F_SW_I2C display(U8G2_R0, /* clock=*/OLED_SCL, /* data=*/OLED_SDA, /* reset=*/OLED_RESET); // All Boards without Reset of the Display

// Screen drawing locations
#define X_MAX 128
#define Y_MAX 64
static uint8_t iteration_count = 0;
static uint32_t x_coor = 0;
static uint32_t y_coor = 10;
static int8_t x_rate = 4;
static int8_t y_rate = 4;

// String to draw on screen
static char display_str[80] = {0};


/*~~~~~Global Variables~~~~~*/
volatile bool receivedFlag = false;

/*~~~~~Timing State~~~~~*/
// The first ID we ever see becomes the lap marker (start/finish line).
// Subsequent sightings of that same ID close out a lap.
// Other IDs are treated as segments within the current lap.
bool     lap_marker_set = false;
uint32_t lap_marker_id  = 0;
uint32_t lap_count      = 0;
uint32_t lap_start_ms   = 0;
uint32_t last_lap_ms    = 0;   // duration of the most recently completed lap

// Per-lap segment timestamps: id -> millis() when seen this lap.
// Cleared on every lap rollover.
std::map<uint32_t, uint32_t> segment_times;

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

// Draw text horizontally centered on the screen at the given baseline y.
// Uses whatever font is currently set on `display`.
void draw_centered(int16_t y, const char *text)
{
  int16_t w = display.getStrWidth(text);
  int16_t x = (X_MAX - w) / 2;
  if (x < 0)
    x = 0; // string wider than screen — pin to left
  display.drawStr(x, y, text);
}

void logo()
{
  snprintf(display_str, sizeof(display_str), "Initializing");
  display.clearBuffer();
  draw_centered(20, display_str);

  snprintf(display_str, sizeof(display_str), "Basestation");
  draw_centered(40, display_str);

  display.sendBuffer();
}

void setup()
{
  Serial.begin(115200);
  delay(2000);

  /* Initialize Lora */

  pinMode(FEM_EN, OUTPUT);
  digitalWrite(FEM_EN, HIGH); // GC1109 Permanently Enabled

  // Set up SPI with our specific pins
  spi.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_NSS_PIN);

  Serial.print("Initializing radio...");
  int16_t state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, 5, 0x12, 22, 8);
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

  // Initialize the display
  display.begin();
  display.setContrast(200);
  display.setFont(u8g2_font_ncenB10_tr);

  // draw startup logo
  logo();
  delay(3000);
  display.clear();
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

    if (state == RADIOLIB_ERR_NONE)
    {
      // parse data as id
      uint32_t id =
          ((uint32_t)data[0] << 16) |
          ((uint32_t)data[1] << 8) |
          data[2];

      // packet was successfully received
      Serial.println("Received packet!");

      // print the data of the packet which is ID
      Serial.printf("Device:  %06X\n", id);

      // print the RSSI (Received Signal Strength Indicator)
      // of the last received packet
      Serial.print("\tRSSI:\t\t");
      Serial.print(radio.getRSSI());
      Serial.println(" dBm");

      /* Lap and Segment Timing */
      uint32_t now = millis(); // get current time to compare

      if (!lap_marker_set)
      {
        // First ID we've ever seen, set it as the lap marker.
        lap_marker_set = true;
        lap_marker_id  = id;
        lap_start_ms   = now;
        Serial.printf("\tLap marker registered: %06X - Timing Started\n", id);
      }
      else if (id == lap_marker_id)
      {
        uint32_t elapsed = now - lap_start_ms;
        last_lap_ms = elapsed;
        lap_count++;
        Serial.printf("\tLAP %lu complete: %lu ms (%.2f s)\n",
                      lap_count, elapsed, elapsed / 1000.0f);

        // Dump segment splits for this lap, then clear for the next one.
        for (auto &kv : segment_times)
        {
          uint32_t split = kv.second - lap_start_ms;
          Serial.printf("\t  segment %06X: %lu ms\n", kv.first, split);
        }
        segment_times.clear();

        lap_start_ms = now;
      }
      else
      {
        // Segment beacon within the current lap.
        uint32_t split = now - lap_start_ms;
        segment_times[id] = now;
        Serial.printf("\tSegment %06X @ %lu ms into lap %lu\n",
                      id, split, lap_count + 1);
      }
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

  /* Live display update: previous lap + current elapsed */
  static uint32_t last_draw_ms = 0;
  uint32_t now = millis();
  if (now - last_draw_ms >= 100)  // ~10 Hz refresh
  {
    last_draw_ms = now;

    display.clearBuffer();

    // Top: most recently completed lap time in seconds (converted from ms)
    snprintf(display_str, sizeof(display_str),
             "Last: %.2fs", last_lap_ms / 1000.0f);
    draw_centered(25, display_str);

    // Bottom: live elapsed time on current lap
    uint32_t elapsed = lap_marker_set ? (now - lap_start_ms) : 0;
    snprintf(display_str, sizeof(display_str),
             "Now:  %.2fs", elapsed / 1000.0f);
    draw_centered(50, display_str);

    display.sendBuffer();
  }
}

