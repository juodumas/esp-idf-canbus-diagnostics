/*
 * canbus-diag  --  minimal ESP32-C3 TWAI diagnostic tool (new esp_twai.h API).
 *
 * Purpose: isolate ACK / electrical problems on a 2-node CAN bus.
 *
 * On boot: installs TWAI but does NOT transmit.
 *
 * All RX/TX-done/state-change/error events come via ISR callbacks and are
 * forwarded to a monitor task through a FreeRTOS queue, because ESP_LOG is
 * not ISR-safe. The error callback exposes ack_err / bit_err / form_err /
 * stuff_err / arb_lost bits.
 *
 * CLI (over USB-Serial-JTAG, single USB cable):
 *   b  cycle bitrate  (125 -> 250 -> 500 -> 1000 kbps), re-create node
 *   m  cycle mode     (normal -> listen-only -> loopback -> self-test)
 *   s  send test frame id=0x00F  data=AB CD
 *   S  send test frame id=0x00F  data= (0 bytes, DLC=0)
 *   i  print status (state, TEC, REC, bus_err_num)
 *   ?  help
 *
 * fail_retry_cnt = 0  -> a no-ACK reports TX-done(success=false) immediately,
 * instead of retransmitting forever.
 */

#include <cstdio>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

static const char *TAG = "cantest";

/* ---- Pins -------------------------------------- */
static constexpr gpio_num_t CAN_TX = GPIO_NUM_21;
static constexpr gpio_num_t CAN_RX = GPIO_NUM_20;

/* ---- Bitrates to cycle with 'b' --------------------------------------- */
static const uint32_t bitrates[] = {10000, 25000, 50000, 125000, 250000, 500000, 1000000};
static const int NUM_BITRATES   = sizeof(bitrates) / sizeof(bitrates[0]);
static int cur_bitrate_idx =  NUM_BITRATES - 1; // start at 1mbps

/* ---- Modes to cycle with 'm' ------------------------------------------ */
enum mode_t { MODE_NORMAL = 0, MODE_LISTEN_ONLY, MODE_LOOPBACK, MODE_SELF_TEST, MODE_COUNT };
static const char *mode_names[] = {"normal", "listen-only", "loopback", "self-test"};
static int cur_mode = MODE_NORMAL;

/* ---- State ------------------------------------------------------------ */
static twai_node_handle_t node = nullptr;

/* ---- Event queue (ISR -> monitor task) ------------------------------- */
struct evt_t {
    enum kind_t { TX, RX, STATE, ERR } kind;
    union {
        struct { bool success; } tx;
        struct { uint32_t id; uint16_t dlc; uint8_t ide; uint8_t rtr; uint8_t data[8]; } rx;
        struct { uint32_t old_s; uint32_t new_s; } st;
        struct { uint32_t flags; } err;
    };
};
static QueueHandle_t evt_queue = nullptr;

/* ---- ISR callbacks ---------------------------------------------------- */
static bool on_tx_done(twai_node_handle_t, const twai_tx_done_event_data_t *e, void *)
{
    evt_t ev{}; ev.kind = evt_t::TX; ev.tx.success = e->is_tx_success;
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(evt_queue, &ev, &woken);
    return woken == pdTRUE;
}

static bool on_rx_done(twai_node_handle_t h, const twai_rx_done_event_data_t *, void *)
{
    uint8_t buf[8] = {};
    twai_frame_t f{}; f.buffer = buf; f.buffer_len = sizeof(buf);
    if (twai_node_receive_from_isr(h, &f) == ESP_OK) {
        evt_t ev{}; ev.kind = evt_t::RX;
        ev.rx.id  = f.header.id;
        ev.rx.dlc = f.header.dlc;
        ev.rx.ide = f.header.ide;
        ev.rx.rtr = f.header.rtr;
        size_t n = f.header.dlc; if (n > 8) n = 8;
        memcpy(ev.rx.data, buf, n);
        BaseType_t woken = pdFALSE;
        xQueueSendFromISR(evt_queue, &ev, &woken);
        return woken == pdTRUE;
    }
    return false;
}

static bool on_state_change(twai_node_handle_t, const twai_state_change_event_data_t *e, void *)
{
    evt_t ev{}; ev.kind = evt_t::STATE; ev.st.old_s = (uint32_t)e->old_sta; ev.st.new_s = (uint32_t)e->new_sta;
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(evt_queue, &ev, &woken);
    return woken == pdTRUE;
}

static bool on_error(twai_node_handle_t, const twai_error_event_data_t *e, void *)
{
    evt_t ev{}; ev.kind = evt_t::ERR; ev.err.flags = e->err_flags.val;
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(evt_queue, &ev, &woken);
    return woken == pdTRUE;
}

/* ---- Node (re)creation ----------------------------------------------- */
static void node_destroy(void)
{
    if (!node) return;
    (void)twai_node_disable(node);
    (void)twai_node_delete(node);
    node = nullptr;
}

static void node_create(void)
{
    node_destroy();

    twai_onchip_node_config_t cfg{};
    cfg.io_cfg.tx = CAN_TX;
    cfg.io_cfg.rx = CAN_RX;
    cfg.io_cfg.quanta_clk_out    = (gpio_num_t)-1;
    cfg.io_cfg.bus_off_indicator = (gpio_num_t)-1;
    cfg.bit_timing.bitrate   = bitrates[cur_bitrate_idx];
    cfg.bit_timing.sp_permill = 0;        // driver default sample point
    cfg.bit_timing.ssp_permill = 0;
    cfg.data_timing = cfg.bit_timing;    // unused for classic, harmless
    cfg.fail_retry_cnt = 0;              // report no-ACK immediately (no infinite retrans)
    cfg.tx_queue_depth = 10;
    cfg.intr_priority  = 0;

    switch (cur_mode) {
        case MODE_LISTEN_ONLY: cfg.flags.enable_listen_only = 1; break;
        case MODE_LOOPBACK:    cfg.flags.enable_loopback    = 1; break;
        case MODE_SELF_TEST:   cfg.flags.enable_self_test   = 1;
                               cfg.flags.enable_loopback    = 1; break;
        default: break;
    }

    esp_err_t err = twai_new_node_onchip(&cfg, &node);
    if (err != ESP_OK) { ESP_LOGE(TAG, "twai_new_node_onchip: %d (%s)", err, esp_err_to_name(err)); node = nullptr; return; }

    twai_event_callbacks_t cbs{};
    cbs.on_tx_done       = on_tx_done;
    cbs.on_rx_done       = on_rx_done;
    cbs.on_state_change  = on_state_change;
    cbs.on_error         = on_error;
    err = twai_node_register_event_callbacks(node, &cbs, nullptr);
    if (err != ESP_OK) { ESP_LOGE(TAG, "register_callbacks: %d (%s)", err, esp_err_to_name(err)); }

    err = twai_node_enable(node);
    if (err != ESP_OK) { ESP_LOGE(TAG, "twai_node_enable: %d (%s)", err, esp_err_to_name(err)); }

    ESP_LOGI(TAG, "TWAI node up @ %lu bps  mode=%s  tx=%d rx=%d  fail_retry=0",
             (unsigned long)bitrates[cur_bitrate_idx], mode_names[cur_mode], (int)CAN_TX, (int)CAN_RX);
}

/* ---- Commands --------------------------------------------------------- */
static void cmd_send(bool with_data)
{
    if (!node) { ESP_LOGE(TAG, "node not up"); return; }
    // NOTE: the esp_twai HAL treats header.dlc==0 as "unset" and derives the
    // on-wire DLC from buffer_len (twai_hal_v1.c: final_dlc = dlc ? dlc :
    // len2dlc(buffer_len)). So to actually send a 0-byte data frame we must
    // also pass buffer_len=0 (and a null/unused buffer); otherwise the stale
    // payload bytes leak onto the wire as a 2-byte frame.
    static uint8_t payload[2] = { 0xAB, 0xCD };  // only read when with_data
    twai_frame_t f{};
    f.header.id  = with_data ? 0xFFF : 0x100;
    f.header.dlc = with_data ? 2 : 0;
    f.buffer     = with_data ? payload : nullptr;
    f.buffer_len = with_data ? sizeof(payload) : 0;
    esp_err_t err = twai_node_transmit(node, &f, 200);
    ESP_LOGI(TAG, "tx id=0x%03X dlc=%u err=%d (%s) -- watch for TX done callback",
             f.header.id, f.header.dlc, err, esp_err_to_name(err));
}

static const char *state_name(uint32_t s)
{
    switch (s) {
        case TWAI_ERROR_ACTIVE:   return "ACTIVE";
        case TWAI_ERROR_WARNING:  return "WARNING(>=96)";
        case TWAI_ERROR_PASSIVE:  return "PASSIVE(>=128)";
        case TWAI_ERROR_BUS_OFF:  return "BUS_OFF(>=256)";
        default:                  return "?";
    }
}

static void cmd_info(void)
{
    if (!node) { ESP_LOGE(TAG, "node not up"); return; }
    twai_node_status_t st{};
    twai_node_record_t rec{};
    esp_err_t err = twai_node_get_info(node, &st, &rec);
    if (err != ESP_OK) { ESP_LOGE(TAG, "get_info: %d (%s)", err, esp_err_to_name(err)); return; }
    ESP_LOGI(TAG, "INFO state=%s TEC=%u REC=%u tx_q_remaining=%lu bus_err_num=%lu",
             state_name((uint32_t)st.state),
             st.tx_error_count, st.rx_error_count,
             (unsigned long)st.tx_queue_remaining, (unsigned long)rec.bus_err_num);
}

/* ---- Monitor task (drains event queue) ------------------------------- */
static void print_err_flags(uint32_t v)
{
    twai_error_flags_t f{}; f.val = v;
    if (f.ack_err)   ESP_LOGE(TAG, "  ack_err   -> NO ACK (other node didn't hear/ACK)");
    if (f.bit_err)   ESP_LOGE(TAG, "  bit_err   -> TX line mismatch (stuck-dominant / TX-RX swap?)");
    if (f.form_err)  ESP_LOGW(TAG, "  form_err  -> bad fixed-form bits on the wire");
    if (f.stuff_err) ESP_LOGW(TAG, "  stuff_err -> stuffing violation on the wire");
    if (f.arb_lost)  ESP_LOGI(TAG, "  arb_lost  -> lost arbitration (normal if collision)");
    if (!v)          ESP_LOGI(TAG, "  (no error flags set)");
}

static void monitor_task(void *)
{
    evt_t ev;
    while (true) {
        if (xQueueReceive(evt_queue, &ev, portMAX_DELAY) != pdPASS) continue;
        switch (ev.kind) {
            case evt_t::TX:
                if (ev.tx.success) ESP_LOGI(TAG, "CB TX done: SUCCESS (frame was ACKed)");
                else               ESP_LOGE(TAG, "CB TX done: FAILED (no ACK / bus-off)");
                break;
            case evt_t::RX:
                ESP_LOGI(TAG, "CB RX id=0x%03X ide=%u rtr=%u dlc=%u  data=",
                         ev.rx.id, ev.rx.ide, ev.rx.rtr, ev.rx.dlc);
                printf("     ");
                for (int i = 0; i < ev.rx.dlc && i < 8; i++) printf("%02X ", ev.rx.data[i]);
                printf("\n");
                break;
            case evt_t::STATE:
                ESP_LOGW(TAG, "CB state: %s -> %s", state_name(ev.st.old_s), state_name(ev.st.new_s));
                break;
            case evt_t::ERR:
                ESP_LOGE(TAG, "CB error flags=0x%02X", ev.err.flags);
                print_err_flags(ev.err.flags);
                break;
        }
    }
}

/* ---- CLI ------------------------------------------------------------- */
static void print_help(void)
{
    printf("\nCAN-bus diagnostic CLI\n");
    printf("  b   cycle bitrate  (now %lu bps, %d/%d)\n",
           (unsigned long)bitrates[cur_bitrate_idx], cur_bitrate_idx + 1, NUM_BITRATES);
    printf("  m   cycle mode     (now %s)\n", mode_names[cur_mode]);
    printf("  s   send frame     id=0x00F data=AB CD   (DLC=2)\n");
    printf("  S   send frame     id=0x00F data=        (DLC=0)\n");
    printf("  i   print status   (state/TEC/REC/bus_err)\n");
    printf("  ?   help\n");
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "=== canbus-diag boot ===");
    evt_queue = xQueueCreate(32, sizeof(evt_t));
    configASSERT(evt_queue);
    xTaskCreate(monitor_task, "mon", 4096, nullptr, 5, nullptr);
    node_create();
    print_help();

    while (true) {
        int c = getchar();
        if (c == EOF) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
        switch (c) {
            case 'b': case 'B':
                cur_bitrate_idx = (cur_bitrate_idx + 1) % NUM_BITRATES;
                node_create();
                print_help();
                break;
            case 'm': case 'M':
                cur_mode = (cur_mode + 1) % MODE_COUNT;
                node_create();
                print_help();
                break;
            case 'S': cmd_send(false);  break;
            case 's': cmd_send(true);  break;
            case 'i': case 'I': cmd_info();  break;
            case '?': case 'h': case 'H': print_help(); break;
            case '\r': case '\n': case ' ': break;
            default:
                ESP_LOGW(TAG, "unknown '%c' (0x%02X) -- press ? for help", c, c);
                break;
        }
    }
}
