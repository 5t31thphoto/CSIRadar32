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
#include <esp_timer.h>   // v0.9: microsecond clock for precise TX cadence

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
// v0.9 COMMANDED MODE ONLY.  Exact period in microseconds, and an
// absolute next-transmit deadline.
//
// The stock path schedules with `last_tx = now` after each send, so
// every period becomes (period + however late the poll was) and the
// error is never corrected -- it random-walks.  Measured effect: slot
// slip exceeds a 5.6 ms slot in under a second at six beacons, which is
// why staggering transmits achieved nothing.
//
// It also truncated: 1000/30 = 33 ms, i.e. 30.30 Hz, not 30.
//
// Precise mode advances an ABSOLUTE deadline (next += period) at
// microsecond resolution, so latency on one cycle does not push the
// next one.  Enabled only once a rate command arrives, so an
// uncommanded beacon behaves EXACTLY as stock for old receivers.
// Number of interleave slots the transmit period is divided into.  Must
// match the receiver's MAX_BEACONS so every possible id gets its own.
#define BEACON_MAX_SLOTS 6
static uint32_t s_tx_period_us   = 10000;
static int64_t  s_next_tx_us     = 0;
static bool     s_precise_timing = false;
static uint8_t  s_sleep_fail_count = 0;
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
            // Exact microsecond period: 1000000/30 = 33333 us is a true
            // 30.000 Hz, where 1000/30 = 33 ms is 30.30 Hz.
            s_tx_period_us   = 1000000UL / hz;
            s_precise_timing = true;       // commanded => precise from now on

            // ── SLOT ANCHORING ──────────────────────────────────────
            // SET_RATE is a BROADCAST, so every beacon receives this same
            // RF event within microseconds of the others.  That gives a
            // shared epoch for free -- no sync protocol, no extra traffic.
            //
            // Each beacon offsets its first transmit by its own id, so
            // the beacons interleave instead of piling up.  With the
            // absolute-deadline scheduling above the offsets HOLD, which
            // they could not do before: the old millis() cadence lost a
            // 5.6 ms slot in under a second, which is why staggering had
            // never been worth doing.
            //
            // Relative crystal drift (~20 ppm) moves neighbours by about
            // 1.2 ms per minute against a 5.6 ms slot, and the receiver
            // re-commands periodically, which re-anchors everyone.
            {
                uint32_t slot = s_tx_period_us / BEACON_MAX_SLOTS;
                uint32_t mine = (beacon_id >= 1 && beacon_id <= BEACON_MAX_SLOTS)
                              ? (uint32_t)(beacon_id - 1) : 0;
                s_next_tx_us = esp_timer_get_time() + (int64_t)mine * slot;
                Serial.printf("[BEACON] slot %u/%u, offset %lu us\n",
                              (unsigned)mine + 1, (unsigned)BEACON_MAX_SLOTS,
                              (unsigned long)(mine * slot));
            }
            Serial.printf("[BEACON] SET_RATE %u Hz (period=%lu ms)\n",
                hz, (unsigned long)s_tx_period_ms);
        } break;

        case BEACON_OP_SET_SLEEP: {
            s_sleep_between    = (c.arg_u16 != 0);
            s_sleep_fail_count = 0;
            // Wi-Fi power save MUST be MIN_MODEM for light sleep to keep
            // the radio coherent across a wake.  Arming sleep while PS
            // was NONE is what made beacons wake with a dead TX path and
            // stay silent until power-cycled.
            esp_wifi_set_ps(s_sleep_between ? WIFI_PS_MIN_MODEM : WIFI_PS_NONE);
            esp_wifi_set_channel(11, WIFI_SECOND_CHAN_BELOW);
            Serial.printf("[BEACON] SET_SLEEP %s (ps=%s)\n",
                          s_sleep_between ? "on" : "off",
                          s_sleep_between ? "MIN_MODEM" : "NONE");
        } break;

        case BEACON_OP_RESTORE_DEFAULTS: {
            s_tx_period_ms     = 10;
            s_tx_period_us     = 10000;
            s_precise_timing   = false;    // back to stock timing exactly
            s_next_tx_us       = 0;
            s_sleep_between    = false;
            s_sleep_fail_count = 0;
            esp_wifi_set_ps(WIFI_PS_NONE);
            esp_wifi_set_channel(11, WIFI_SECOND_CHAN_BELOW);
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

    // TX cadence.
    //
    // PRECISE (commanded) path uses an absolute microsecond deadline so
    // timing error cannot accumulate.  STOCK path is byte-for-byte the
    // original millis() logic, so an uncommanded beacon is unchanged.
    bool due;
    if (s_precise_timing) {
        int64_t now_us = esp_timer_get_time();
        if (s_next_tx_us == 0) s_next_tx_us = now_us;
        due = (now_us >= s_next_tx_us);
        if (due) {
            s_next_tx_us += (int64_t)s_tx_period_us;
            // If we fell more than a whole period behind -- a long sleep
            // overrun or a burst of command handling -- resync instead of
            // firing a catch-up burst that would collide with everyone.
            if (now_us - s_next_tx_us > (int64_t)s_tx_period_us)
                s_next_tx_us = now_us + (int64_t)s_tx_period_us;
            last_tx = now;                 // sleep block still uses millis
        }
    } else {
        due = (now - last_tx >= s_tx_period_ms);
        if (due) last_tx = now;
    }
    if (due) {

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
    // period is > 30 ms.
    //
    // FIX: this used to call esp_light_sleep_start() while Wi-Fi power
    // save was WIFI_PS_NONE.  Light sleep only maintains the Wi-Fi
    // connection when PS is WIFI_PS_MIN_MODEM — with PS_NONE the radio
    // is not kept coherent across the sleep, so the beacon woke with a
    // dead TX path and went silent until it was power-cycled.  That is
    // the "beacons drop out and don't come back" fault.
    //
    // Three things make it survivable now:
    //   1. PS is switched to MIN_MODEM whenever sleep is armed (done in
    //      apply_command), and back to NONE when it is disarmed.
    //   2. The return value is CHECKED.  Consecutive rejections disarm
    //      sleep automatically, so a beacon can never wedge itself.
    //   3. The channel is re-asserted after every wake, because light
    //      sleep can drop the channel configuration.
    if (s_sleep_between && s_tx_period_ms > 30) {
        uint32_t elapsed = millis() - last_tx;
        if (elapsed + 15 < s_tx_period_ms) {
            uint32_t sleep_us = (s_tx_period_ms - elapsed - 5) * 1000UL;
            esp_sleep_enable_timer_wakeup(sleep_us);
            esp_err_t sr = esp_light_sleep_start();
            if (sr != ESP_OK) {
                // Sleep was rejected. Do not keep trying blindly.
                if (++s_sleep_fail_count >= 5) {
                    s_sleep_between = false;
                    s_sleep_fail_count = 0;
                    esp_wifi_set_ps(WIFI_PS_NONE);
                    Serial.println("[beacon] light sleep rejected 5x - "
                                   "sleep disabled, staying awake");
                }
                delay(1);
            } else {
                s_sleep_fail_count = 0;
                // Light sleep can lose the channel; put it back before the
                // next transmit or the frame goes out on the wrong one.
                esp_wifi_set_channel(11, WIFI_SECOND_CHAN_BELOW);
            }
        } else {
            delay(1);
        }
    } else {
        delay(1);
    }

    // Stall watchdog: if sleep is armed but nothing has actually gone out
    // for a long time, the radio did not survive a wake.  Disarm sleep and
    // recover rather than sitting silent until someone pulls the power.
    if (s_sleep_between && (millis() - last_tx) > 2000) {
        s_sleep_between = false;
        s_sleep_fail_count = 0;
        esp_wifi_set_ps(WIFI_PS_NONE);
        esp_wifi_set_channel(11, WIFI_SECOND_CHAN_BELOW);
        Serial.println("[beacon] TX stalled >2s with sleep on - "
                       "sleep disabled, radio restored");
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
