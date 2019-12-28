#ifndef UART_RX_TASK_H
#define UART_RX_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

  void uart_rx_task(void const *argument);
  void uart_rx_notify();
  
#ifdef __cplusplus
}
#endif

#endif /* UART_RX_TASK_H */
