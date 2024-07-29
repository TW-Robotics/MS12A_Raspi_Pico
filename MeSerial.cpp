#include "MeSerial.h"


MeSerial::MeSerial(uint8_t receivePin, uint8_t transmitPin)
{
  _RxPin = receivePin;
  _TxPin = transmitPin;
}


void MeSerial::begin(long baudrate)
{
  uart_init(UART_ID, BAUD_RATE);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
  uart_set_hw_flow(UART_ID, false, false);

  //_bitPeriod = 1000000 / BAUD_RATE; //brauchen wir mglw nicht
}

void MeSerial::end(void)
{
  void uart_deinit(UART_ID);
}

size_t MeSerial::write(uint8_t byte)
{
  static void uart_write_blocking (UART_ID,byte,8);	//Könnte auch 7 bit sein
  return 8;
}

int16_t MeSerial::read(void)
{
  uint8_t* data = nullptr;
  static void uart_read_blocking 	(UART_ID,data,8)
	
  return *data;
}

int16_t MeSerial::available(void)
{
  if (static bool uart_is_writable(UART_ID) && static bool uart_is_readable(UART_ID))
  {
    return (int16_t)1;
  }
  else
  {
    return (int16_t)0;
  }
}

bool MeSerial::listen(void)
{
  return (true);
}

bool MeSerial::isListening(void)
{
  return (true);
}

void MeSerial::sendString(char *str)
{
  static void uart_puts(UART_ID,str);
}


