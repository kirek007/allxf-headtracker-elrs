// https://github.com/kilrah/allxf-headtracker-elrs
// Bridge from AllXF UART Headtracker or any PPM source to ELRS backpack over esp-now using ESP32

// msp.h and some other bits from https://github.com/jlpoltrack/ELRS-Headtracker-to-SBUS/

#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <mbedtls/md5.h>
#include <msp.h>
#include <ESP32_ppm.h>
#include "allxf.h"

// --------- CONFIG ------------
// Common
#define BINDING_PHRASE   "MY_PHRASE"       // Must match your ELRS / Backpack setup
#define LED              8                 // Board LED

// AllXF
#define RX_PIN           4                 // Connects to T on headtracker

// PPM
#define PPM_PIN          3                 // Where the input PPM signal is connected

// -------- GLOBALS ----------
// allxf vars
uint8_t allxf_rxBuf[8];
uint8_t allxf_rxCount;
allxf_htMessage_t allxf_htMsg;
allxf_htState_t allxf_htState = STATE_WAIT_FOOTER;

// elrs uid
uint8_t uid[6];

// ppm vars
ppmReader ppmRx;
int* ppmArray;

// diag counters
volatile uint32_t diag_sendOk, diag_sendFail;
uint32_t diag_uartBytes, diag_htFrames, diag_ppmFrames, diag_lastPrint;

void onEspNowSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) diag_sendOk++; else diag_sendFail++;
}

// -------- TX functions --------
void sendMspEspNow(uint16_t function, const uint8_t *data, uint16_t len) {
    uint8_t frame[64];
    uint8_t frameLen = mspBuildCommand(frame, sizeof(frame), function, data, len);
    if (frameLen) esp_now_send(uid, frame, frameLen);
}

// ELRS treats PTR payload as raw 11-bit CRSF values (same scale the HDZero backpack sends)
#define CRSF_1000  191
#define CRSF_MID   992
#define CRSF_2000  1792

// +-180 deg (+-2048) -> CRSF 1000..2000, centred on 992
int16_t allxfAngleToCrsf(int32_t raw) {
    return constrain(CRSF_MID + raw * (CRSF_2000 - CRSF_1000) / 4096, CRSF_1000, CRSF_2000);
}

void sendAllxfHtPacket() {
    int16_t channels[5];
    channels[0] = allxfAngleToCrsf(allxf_htMsg.pan);
    channels[1] = allxfAngleToCrsf(allxf_htMsg.tilt);
    channels[2] = allxfAngleToCrsf(allxf_htMsg.roll);
    channels[3] = map(allxf_htMsg.mode, 0, 2, CRSF_1000, CRSF_2000);
    channels[4] = map(allxf_htMsg.sensitivity, -15, 15, CRSF_1000, CRSF_2000);
    sendMspEspNow(MSP_ELRS_SET_PTR, (uint8_t*) channels, sizeof(channels));
}

void sendPpmHtPacket() {
    uint8_t channelCount = ppmArray[0];
    int16_t channels[channelCount];

    for(uint8_t i = 0; i < channelCount; i++)
      channels[i] = constrain(map(ppmArray[i+1], 1000, 2000, CRSF_1000, CRSF_2000), CRSF_1000, CRSF_2000);

    sendMspEspNow(MSP_ELRS_SET_PTR, (uint8_t*) channels, sizeof(channels));
    Serial.printf("ppm chans: %d\t", channelCount);
    for(uint8_t i = 0; i < channelCount; i++)
      Serial.printf("%d\t",channels[i]);
    Serial.println();
}

// Replicates ELRS build system: MD5('-DMY_BINDING_PHRASE="<phrase>"')[0:6]
void generateUID(const char *phrase, uint8_t *out) {
  char buf[128];
  snprintf(buf, sizeof(buf), "-DMY_BINDING_PHRASE=\"%s\"", phrase);

  uint8_t hash[16];
  mbedtls_md5_context ctx;
  mbedtls_md5_init(&ctx);
  mbedtls_md5_starts(&ctx);
  mbedtls_md5_update(&ctx, (const uint8_t *)buf, strlen(buf));
  mbedtls_md5_finish(&ctx, hash);
  mbedtls_md5_free(&ctx);

  memcpy(out, hash, 6);
  out[0] &= ~0x01;  // Ensure unicast (clear LSB of first byte)
}

// ----------- AllXF receive handler ---------
bool processAllxfHt(void) {
  if(Serial1.available()) {
    uint8_t data = Serial1.read();
    diag_uartBytes++;

    if(allxf_htState == STATE_RECEIVING) {
      allxf_rxBuf[allxf_rxCount++] = data;
      if(allxf_rxCount == sizeof(allxf_rxBuf)) {
        allxf_htState = STATE_WAIT_FOOTER;
      }
    }

    else if(allxf_htState == STATE_NEW_MESSAGE && data == allxf_ht_message_header) {
      allxf_htState = STATE_RECEIVING;
      allxf_rxCount = 0;
    }

    else if(allxf_htState == STATE_WAIT_FOOTER && data == allxf_ht_message_footer) {
      allxf_htState = STATE_NEW_MESSAGE;
      uint8_t* ptr = (uint8_t*)&allxf_htMsg;
      memcpy(ptr, allxf_rxBuf, sizeof(allxf_rxBuf));
      Serial.printf("allxf mode: %d\tsens: %3d\troll: %6d\ttilt: %6d\tpan: %6d\n", allxf_htMsg.mode, allxf_htMsg.sensitivity, allxf_htMsg.roll, allxf_htMsg.tilt, allxf_htMsg.pan);
      return true;
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  while(!Serial && millis() < 3000);

  // Setup HT UART
  pinMode(RX_PIN, INPUT_PULLUP);
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, -1);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  // Generate ELRS UID from binding phrase
  generateUID(BINDING_PHRASE, uid);

  // Set up Wifi
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, uid);

  // Set channel to 1 (hardcoded in all Backpack modules)
  // WiFi.begin("", "", 1) is a no-op on core 3.x (empty SSID is rejected), so set it directly
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  uint8_t ch; wifi_second_chan_t ch2;
  esp_wifi_get_channel(&ch, &ch2);
  Serial.printf("\nelrsht boot, phrase \"%s\"\nuid/mac %02X:%02X:%02X:%02X:%02X:%02X  sta mac %s  wifi ch %d\n",
    BINDING_PHRASE, uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], WiFi.macAddress().c_str(), ch);

  // Set up ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, uid, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK)
      Serial.print("ESP-NOW add peer failed\n");
  esp_now_register_send_cb(onEspNowSent);

  // Start PPM receiver
  pinMode(PPM_PIN, INPUT_PULLUP);
  ppmArray = ppmRx.begin(PPM_PIN);
  ppmRx.start();
}

void loop() {
  // Read HT
  if(processAllxfHt()) {
    diag_htFrames++;
    sendAllxfHtPacket();
    digitalWrite(LED, !digitalRead(LED));
  }
  if(ppmRx.newFrame()) {
    diag_ppmFrames++;
    sendPpmHtPacket();
    digitalWrite(LED, !digitalRead(LED));
  }

  // diag: uart bytes/frames from HT, and esp-now delivery (ok = backpack with our uid ACKed)
  if(millis() - diag_lastPrint >= 1000) {
    diag_lastPrint = millis();
    Serial.printf("[diag] uart bytes %lu  ht frames %lu  ppm frames %lu  espnow ok %lu fail %lu  crsf pan %d tilt %d roll %d\n",
      diag_uartBytes, diag_htFrames, diag_ppmFrames, diag_sendOk, diag_sendFail,
      allxfAngleToCrsf(allxf_htMsg.pan), allxfAngleToCrsf(allxf_htMsg.tilt), allxfAngleToCrsf(allxf_htMsg.roll));
  }
}

