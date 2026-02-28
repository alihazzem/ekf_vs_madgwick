#include "app/cli_app.h"
#include "drivers/uart_cli.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <ctype.h>

static void trim_inplace(char *s) {
  while (*s && isspace((unsigned char)*s)) {
    memmove(s, s + 1, strlen(s));
  }
  size_t n = strlen(s);
  while (n > 0 && isspace((unsigned char)s[n - 1])) {
    s[n - 1] = '\0';
    n--;
  }
}

static void to_upper_inplace(char *s) {
  while (*s) { *s = (char)toupper((unsigned char)*s); s++; }
}

void app_cli_print_banner(void) {
  uart_cli_send("\r\n============================\r\n");
  uart_cli_send(" EKF vs Madgwick (STM32F411)\r\n");
  uart_cli_send(" UART CLI enabled\r\n");
  uart_cli_send("============================\r\n");
  uart_cli_send("Type: HELP\r\n");
}

void app_cli_handle_line(const char *line) {
  char cmd[128];
  strncpy(cmd, line, sizeof(cmd) - 1);
  cmd[sizeof(cmd) - 1] = '\0';
  trim_inplace(cmd);

  if (cmd[0] == '\0') return;

  char up[128];
  strncpy(up, cmd, sizeof(up) - 1);
  up[sizeof(up) - 1] = '\0';
  to_upper_inplace(up);

  if (strcmp(up, "HELP") == 0) {
    uart_cli_send("Commands:\r\n");
    uart_cli_send("  HELP\r\n");
    uart_cli_send("  PING\r\n");
    uart_cli_send("  STATUS\r\n");
    return;
  }

  if (strcmp(up, "PING") == 0) {
    uart_cli_send("PONG\r\n");
    return;
  }

  if (strcmp(up, "STATUS") == 0) {
    uart_cli_sendf("uptime_ms=%lu\r\n", (unsigned long)HAL_GetTick());
    uart_cli_send("uart=USART2 115200\r\n");
    return;
  }

  uart_cli_send("ERR: unknown command. Type HELP\r\n");
}
