#include "drivers/uart_cli.h"
#include "utils/ringbuf.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#ifndef UART_CLI_RX_BUF_SIZE
#define UART_CLI_RX_BUF_SIZE 512
#endif

#ifndef UART_CLI_LINE_MAX
#define UART_CLI_LINE_MAX 128
#endif

static UART_HandleTypeDef *s_huart = NULL;

static uint8_t  s_rx_storage[UART_CLI_RX_BUF_SIZE];
static ringbuf_t s_rb;
static uint8_t s_rx_byte;

static char s_line[UART_CLI_LINE_MAX];
static uint32_t s_line_len = 0;

// implemented in app/cli_app.c
extern void app_cli_handle_line(const char *line);

// ───────────── TX ─────────────
void uart_cli_send(const char *s) {
  if (!s_huart || !s) return;
  HAL_UART_Transmit(s_huart, (uint8_t*)s, (uint16_t)strlen(s), 1000);
}

void uart_cli_sendf(const char *fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  uart_cli_send(buf);
}

// ───────────── INIT ─────────────
void uart_cli_init(UART_HandleTypeDef *huart) {
  s_huart = huart;
  rb_init(&s_rb, s_rx_storage, sizeof(s_rx_storage));
  s_line_len = 0;

  HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);

  uart_cli_send("\r\n[CLI] ready. Type HELP\r\n> ");
}

// ───────────── RX callback glue ─────────────
void uart_cli_on_rx_byte(UART_HandleTypeDef *huart) {
  if (!s_huart || huart != s_huart) return;

  (void)rb_push(&s_rb, s_rx_byte);
  HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
}

// ───────────── line parser ─────────────
static void handle_char(uint8_t c) {
  if (c == '\r') return;

  if (c == '\n') {
    s_line[s_line_len] = '\0';
    uart_cli_send("\r\n");
    if (s_line_len > 0) app_cli_handle_line(s_line);
    s_line_len = 0;
    uart_cli_send("> ");
    return;
  }

  // backspace
  if (c == 0x7F || c == 0x08) {
    if (s_line_len > 0) {
      s_line_len--;
      uart_cli_send("\b \b");
    }
    return;
  }

  // printable
  if (c >= 32 && c <= 126) {
    if (s_line_len < (UART_CLI_LINE_MAX - 1U)) {
      s_line[s_line_len++] = (char)c;
      char e[2] = {(char)c, 0};
      uart_cli_send(e);
    }
  }
}

void uart_cli_poll(void) {
  uint8_t c;
  while (rb_pop(&s_rb, &c) == 0) {
    handle_char(c);
  }
}
