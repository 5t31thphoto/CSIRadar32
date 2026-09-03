// ═══════════════════════════════════════════════════════════════
//  CSI-Beacon-Extended  —  drop-in replacement for CSI-Beacon.ino
//  that ALSO responds to a small ESP-NOW command protocol from
//  the CSI-Radar-S3 receiver.
//
//  BACKWARD COMPATIBLE.  On boot, this beacon:
//    - Uses fixed MAC 1A:00:00:00:00:ID
//    - Transmits ESP-NOW broadcast at 100 Hz
//    - Payload = uint32_t counter  ← Cardputer & stock RX expect this
//    - Advertises AP "CSI-Beacon-N" on channel 11 HT40
//  Exactly like the stock beacon.  A stock receiver (Cardputer,
//  v0.1 T-Display firmware, etc.) sees no difference.
//
//  NEW: an ESP-NOW receive callback listens for command packets
//  guarded by a 32-bit magic word (BEACON_CMD_MAGIC).  If, and
//  only if, a properly-magiced command arrives, the beacon may:
//    - Change its TX rate (e.g. down to 20 Hz for lower ambient
//      channel utilisation during long stereo runs)
//    - Enable light-sleep between bursts (power save)
//    - Reply to PING with a PONG carrying the extended-firmware
//      marker so a receiver can detect capability
//    - Restore defaults
//
//  Random ESP-NOW traffic that lacks the magic is ignored, so the
//  beacon never accidentally re-configures itself just because
//  someone happens to be broadcasting nearby.
//
//  Serial commands from the stock firmware ("beacon id N",
//  "beacon status", "beacon restart") are preserved verbatim.
//
//  Flash on any ESP32 (C3, C6, S2, S3, classic).  Same pins.
// ═══════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_sleep.h>

// ── Firmware identity ─────────────────────────────────────────
#define BEACON_FW_NAME     "CSI-Beacon-Extended"
#define BEACON_FW_VERSION  "0.2.0"

// ── Command protocol ──────────────────────────────────────────
// Chosen so a stray 4-byte counter (which is what beacons OUTPUT)
// is extremely unlikely to alias these values.  Both values are
// higher than any counter would practically reach in a session
// (2^31 ≈ 68 years @ 100 Hz), but the "top nibble = 0xB or 0xC"
// pattern makes it fully unambiguous even on wraparound.
#define BEACON_CMD_MAGIC   0xBCC1D01AUL   // command from RX → beacon
#define BEACON_PONG_MAGIC  0xBCF0F0AAUL   // reply from beacon → RX

enum : uint8_t {
    BEACON_OP_PING              = 1,   // reply with PONG
    BEACON_OP_SET_RATE          = 2,   // arg_u16 = Hz (1..200)
    BEACON_OP_SET_SLEEP         = 3,   // arg_u16 = 0/1 (light sleep between bursts)
    BEACON_OP_RESTORE_DEFAULTS  = 4,   // back to 100 Hz, no sleep
};

// Wire packet formats.  Kept POD; ESP-NOW passes raw bytes.
struct BeaconCommand {
    uint32_t magic;         // BEACON_CMD_MAGIC
    uint8_t  target_id;     // 0 = broadcast (any beacon), else specific ID
    uint8_t  op;
    uint16_t arg_u16;
    uint32_t arg_u32;
    uint8_t  sender_mac[6]; // where to send the PONG (if op == PING)
    uint8_t  _pad[2];
};

struct BeaconPong {
    uint32_t magic;         // BEACON_PONG_MAGIC
    uint8_t  beacon_id;
    uint8_t  fw_marker;     // 1 = extended firmware
    uint16_t current_rate_hz;
    uint32_t uptime_ms;
    uint32_t total_tx_count;
    uint8_t  sleep_enabled;
    uint8_t  fw_version[7]; // truncated string
    uint8_t  _pad[4];
};

// ── Runtime state (only NEW stuff — original state below) ─────
static uint32_t s_tx_period_ms   = 10;    // 100 Hz default (10 ms period)
static bool     s_sleep_between  = false;
static uint32_t s_cmd_rx_count   = 0;     // diagnostic
static uint32_t s_cmd_ignored    = 0;     // packets without our magic

// ── Original beacon state (verbatim) ──────────────────────────
static uint8_t  beacon_id          = 1;
static uint32_t beacon_pkt_count   = 0;
static char     beacon_ssid[32]    = "CSI-Beacon-1";
static uint8_t  fixed_mac[6]       = {0x1A, 0x00, 0x00, 0x00, 0x00, 0x01};

#if defined(LED_BUILTIN)
  #define BEACON_LED LED_BUILTIN
#elif defined(ARDUINO_M5STACK_NANOC6)
  #define BEACON_LED 7
#else
  #define BEACON_LED -1
#endif

static WiFiUDP udp;
static String  serialBuf;
static const uint8_t broadcast_addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void handleCommand(const String& cmd);

// ── ESP-NOW recv (command listener) ───────────────────────────
// Signature guard — arduino-esp32 v3.x changed to esp_now_recv_info_t.
#ifndef ESP_ARDUINO_VERSION_MAJOR
  #if __has_include(<esp_arduino_version.h>)
    #include <esp_arduino_version.h>
  #endif
#endif
#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 3
#endif

// Add or refresh a unicast peer entry for the sender so we can PONG back.
// The peer table is small (~20 slots on most chips); if we've already
// added this peer, re-adding returns an error we can safely ignore.
static void ensure_peer(const uint8_t mac[6]) {
    esp_now_peer_info_t p = {};
    memcpy(p.peer_addr, mac, 6);
    p.channel = 0;
    p.ifidx   = WIFI_IF_STA;
    p.encrypt = false;
    esp_err_t err = esp_now_add_peer(&p);
    (void)err;
}

static void send_pong_to(const uint8_t dest_mac[6]) {
    BeaconPong pong = {};
    pong.magic           = BEACON_PONG_MAGIC;
    pong.beacon_id       = beacon_id;
    pong.fw_marker       = 1;
    // Guard against overflow when computing current rate
    pong.current_rate_hz = (uint16_t)(s_tx_period_ms > 0 ? 1000 / s_tx_period_ms : 0);
    pong.uptime_ms       = millis();
    pong.total_tx_count  = beacon_pkt_count;
    pong.sleep_enabled   = s_sleep_between ? 1 : 0;
    memcpy(pong.fw_version, BEACON_FW_VERSION, sizeof(pong.fw_version));
    ensure_peer(dest_mac);
    esp_now_send(dest_mac, (const uint8_t*)&pong, sizeof(pong));
}

static void apply_command(const BeaconCommand &c, const uint8_t src[6]) {
    // Target filter: 0 = all beacons, else must match this beacon's ID.
    if (c.target_id != 0 && c.target_id != beacon_id) return;
    s_cmd_rx_count++;

    switch (c.op) {
        case BEACON_OP_PING: {
            // Prefer sender_mac from the packet (RX may want a specific
            // unicast target), fall back to the ESP-NOW src MAC.
            uint8_t dest[6];
            bool nonzero = false;
            for (int i = 0; i < 6; i++) if (c.sender_mac[i]) { nonzero = true; break; }
            memcpy(dest, nonzero ? c.sender_mac : src, 6);
            send_pong_to(dest);
            Serial.printf("[BEACON] PING → PONG to %02X:%02X:%02X:%02X:%02X:%02X\n",
                dest[0], dest[1], dest[2], dest[3], dest[4], dest[5]);
        } break;

        case BEACON_OP_SET_RATE: {
            uint16_t hz = c.arg_u16;
            if (hz < 1)   hz = 1;
            if (hz > 200) hz = 200;
            s_tx_period_ms = 1000 / hz;
            if (s_tx_period_ms == 0) s_tx_period_ms = 1;
            Serial.printf("[BEACON] SET_RATE %u Hz (period=%lu ms)\n",
                hz, (unsigned long)s_tx_period_ms);
        } break;

        case BEACON_OP_SET_SLEEP: {
            s_sleep_between = (c.arg_u16 != 0);
            Serial.printf("[BEACON] SET_SLEEP %s\n", s_sleep_between ? "on" : "off");
        } break;

        case BEACON_OP_RESTORE_DEFAULTS: {
            s_tx_period_ms  = 10;
            s_sleep_between = false;
            Serial.println("[BEACON] RESTORE_DEFAULTS (100 Hz, no sleep)");
        } break;

        default:
            Serial.printf("[BEACON] unknown op %u\n", (unsigned)c.op);
            break;
    }
}

// Common handler for both callback signatures
static void handle_incoming(const uint8_t *src, const uint8_t *data, int len) {
    if (!data || len < (int)sizeof(uint32_t)) { s_cmd_ignored++; return; }
    uint32_t magic;
    memcpy(&magic, data, sizeof(magic));
    if (magic != BEACON_CMD_MAGIC) { s_cmd_ignored++; return; }
    if (len < (int)sizeof(BeaconCommand)) { s_cmd_ignored++; return; }
    BeaconCommand c;
    memcpy(&c, data, sizeof(c));
    apply_command(c, src);
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (!info) return;
    handle_incoming(info->src_addr, data, len);
}
#else
static void espnow_recv_cb(const uint8_t *src, const uint8_t *data, int len) {
    handle_incoming(src, data, len);
}
#endif

// ── Beacon startup (verbatim from stock, + recv cb registration) ─
void startBeacon() {
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    delay(100);

    WiFi.mode(WIFI_AP_STA);

    fixed_mac[5] = beacon_id;
    esp_err_t mac_err = esp_wifi_set_mac(WIFI_IF_STA, fixed_mac);
    if (mac_err != ESP_OK) Serial.printf("[BEACON] WARNING: set_mac failed (%d)\n", mac_err);
    uint8_t check_mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, check_mac);
    Serial.printf("[BEACON] STA MAC set: %02X:%02X:%02X:%02X:%02X:%02X\n",
        check_mac[0], check_mac[1], check_mac[2], check_mac[3], check_mac[4], check_mac[5]);

    snprintf(beacon_ssid, sizeof(beacon_ssid), "CSI-Beacon-%d", beacon_id);
    WiFi.softAP(beacon_ssid, NULL, 11);

    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40);
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_channel(11, WIFI_SECOND_CHAN_BELOW);

    esp_wifi_start();
    delay(100);

    // ESP-NOW init
    esp_now_init();
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcast_addr, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    // NEW: register recv callback for the command protocol
    esp_now_register_recv_cb(espnow_recv_cb);

    esp_now_rate_config_t rate_cfg = {};
    rate_cfg.phymode = WIFI_PHY_MODE_HT40;
    rate_cfg.rate    = WIFI_PHY_RATE_MCS0_LGI;
    esp_err_t rc = esp_now_set_peer_rate_config(broadcast_addr, &rate_cfg);
    if (rc != ESP_OK) Serial.printf("[BEACON] Rate config not supported (%d), using default\n", rc);

    udp.begin(55555);

    beacon_pkt_count = 0;
}

// ── setup / loop ──────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println(F("\n========================================"));
    Serial.printf("  %s v%s\n", BEACON_FW_NAME, BEACON_FW_VERSION);
    Serial.println(F("  100 Hz default, listens for RX commands"));
    Serial.println(F("========================================"));

    if (BEACON_LED >= 0) {
        pinMode(BEACON_LED, OUTPUT);
        digitalWrite(BEACON_LED, LOW);
    }

    startBeacon();

    Serial.printf("[BEACON] ID=%d MAC=%02X:%02X:%02X:%02X:%02X:%02X\n",
        beacon_id, fixed_mac[0], fixed_mac[1], fixed_mac[2],
        fixed_mac[3], fixed_mac[4], fixed_mac[5]);
    Serial.printf("[BEACON] SSID=%s CH=11 HT40 MCS0 (rate=%luHz)\n",
        beacon_ssid, (unsigned long)(1000 / s_tx_period_ms));
    Serial.println(F("[BEACON] Type 'beacon help' for commands"));
}

void loop() {
    static unsigned long last_tx = 0;
    unsigned long now = millis();

    // TX cadence — driven by s_tx_period_ms so commands can slow us down.
    if (now - last_tx >= s_tx_period_ms) {
        last_tx = now;

        uint32_t count = beacon_pkt_count;
        esp_now_send(broadcast_addr, (const uint8_t*)&count, sizeof(count));

        // UDP ping reply path (single-beacon fallback mode)
        int pktSize = udp.parsePacket();
        if (pktSize > 0) {
            uint8_t buf[32];
            udp.read(buf, sizeof(buf));
            uint8_t reply[8] = {0xC5, 0x1B, beacon_id, (uint8_t)(count & 0xFF)};
            udp.beginPacket(udp.remoteIP(), udp.remotePort());
            udp.write(reply, 4);
            udp.endPacket();
        }

        beacon_pkt_count++;

        if (BEACON_LED >= 0 && (beacon_pkt_count % 50) == 0)
            digitalWrite(BEACON_LED, !digitalRead(BEACON_LED));

        if ((beacon_pkt_count % 1000) == 0)
            Serial.printf("[BEACON] TX:%lu cmd_rx:%lu ignored:%lu rate=%luHz sleep=%d\n",
                beacon_pkt_count, s_cmd_rx_count, s_cmd_ignored,
                (unsigned long)(1000 / s_tx_period_ms), s_sleep_between);
    }

    // Serial control (unchanged from stock)
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            serialBuf.trim();
            if (serialBuf.length() > 0) { handleCommand(serialBuf); serialBuf = ""; }
        } else if (serialBuf.length() < 128) serialBuf += c;
    }

    // Optional light sleep between bursts.  Only useful at slower rates —
    // at 100 Hz the 10 ms budget is dominated by TX + turnaround so
    // sleeping is counter-productive.  We only sleep when the effective
    // period is > 30 ms.  Wake source is the timer; commands that arrive
    // during sleep are queued by the Wi-Fi driver and processed on wake.
    if (s_sleep_between && s_tx_period_ms > 30) {
        uint32_t elapsed = millis() - last_tx;
        if (elapsed + 15 < s_tx_period_ms) {
            uint32_t sleep_us = (s_tx_period_ms - elapsed - 5) * 1000UL;
            esp_sleep_enable_timer_wakeup(sleep_us);
            esp_light_sleep_start();
        } else {
            delay(1);
        }
    } else {
        delay(1);
    }
}

// ── Serial commands (verbatim, plus one extra to inspect state) ─
void handleCommand(const String& cmd) {
    if (!cmd.startsWith("beacon ")) { Serial.println(F("Type 'beacon help'")); return; }
    String sub = cmd.substring(7); sub.trim();

    if (sub == "help") {
        Serial.println(F("── CSI-Beacon-Extended Commands ──"));
        Serial.println(F("beacon id N     — set ID 1-8 (MAC=1A:00:00:00:00:N)"));
        Serial.println(F("beacon status   — show config"));
        Serial.println(F("beacon rate N   — set TX rate Hz (1..200)"));
        Serial.println(F("beacon sleep 0|1— light-sleep between bursts"));
        Serial.println(F("beacon reset    — restore stock defaults (100Hz)"));
        Serial.println(F("beacon restart  — restart"));
        Serial.println(F("──────────────────────────────────"));
    } else if (sub.startsWith("id ")) {
        int id = sub.substring(3).toInt();
        if (id >= 1 && id <= 8) {
            beacon_id = (uint8_t)id;
            startBeacon();
            Serial.printf("[BEACON] ID=%d MAC=1A:00:00:00:00:%02X SSID=%s\n",
                beacon_id, beacon_id, beacon_ssid);
        } else Serial.println(F("ERR: id 1-8"));
    } else if (sub.startsWith("rate ")) {
        int hz = sub.substring(5).toInt();
        if (hz < 1 || hz > 200) { Serial.println(F("ERR: rate 1..200")); return; }
        s_tx_period_ms = 1000 / hz;
        if (s_tx_period_ms == 0) s_tx_period_ms = 1;
        Serial.printf("[BEACON] rate=%d Hz (period=%lu ms)\n", hz, (unsigned long)s_tx_period_ms);
    } else if (sub.startsWith("sleep ")) {
        int v = sub.substring(6).toInt();
        s_sleep_between = (v != 0);
        Serial.printf("[BEACON] sleep=%d\n", (int)s_sleep_between);
    } else if (sub == "reset") {
        s_tx_period_ms  = 10;
        s_sleep_between = false;
        Serial.println(F("[BEACON] defaults restored"));
    } else if (sub == "status") {
        Serial.println(F("── CSI-Beacon-Extended Status ──"));
        Serial.printf("FW:      %s v%s\n", BEACON_FW_NAME, BEACON_FW_VERSION);
        Serial.printf("ID:      %d\n", beacon_id);
        Serial.printf("MAC:     %02X:%02X:%02X:%02X:%02X:%02X\n",
            fixed_mac[0], fixed_mac[1], fixed_mac[2],
            fixed_mac[3], fixed_mac[4], fixed_mac[5]);
        Serial.printf("SSID:    %s\n", beacon_ssid);
        Serial.printf("CH:      11 (HT40)\n");
        Serial.printf("TX:      %lu (%luHz)\n",
            beacon_pkt_count, (unsigned long)(1000 / s_tx_period_ms));
        Serial.printf("Sleep:   %s\n", s_sleep_between ? "on" : "off");
        Serial.printf("Cmd RX:  %lu handled, %lu ignored\n", s_cmd_rx_count, s_cmd_ignored);
        Serial.printf("AP:      %d client(s)\n", WiFi.softAPgetStationNum());
    } else if (sub == "restart") {
        ESP.restart();
    } else Serial.println(F("Unknown. Type 'beacon help'"));
}
