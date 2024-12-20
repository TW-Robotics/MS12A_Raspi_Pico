#include "MeSerial.h"
#include "hardware/uart.h"

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
  uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
  //uart_set_hw_flow(UART_ID, true, true);
  uart_set_fifo_enabled	(UART_ID,false);


  //_bitPeriod = 1000000 / BAUD_RATE; //brauchen wir mglw nicht
}

void MeSerial::end()
{
  uart_deinit(UART_ID);
}

size_t MeSerial::write(uint8_t byte)
{
  
  uart_write_blocking(UART_ID,&byte,1);	//Schreiben von 1 Byte
  return 0;
}

int16_t MeSerial::read()
{
  uint16_t* data = nullptr; //Unsicher was das data sein sollte könnte auch ein uint8_t* sein
  //uart_read_blocking(UART_ID,data,8); Falsch
  data* = uart_getc(UART_ID);
	
  return *data;
}

int16_t MeSerial::available()
{
  bool avilable1 =  uart_is_writable(UART_ID);
  bool avilable2 =  uart_is_readable(UART_ID);
  
  if (avilable1 && avilable2)
  {
    return (int16_t)1;
  }
  else
  {
    return (int16_t)0;
  }
}

bool MeSerial::listen()
{
  return (true);
}

bool MeSerial::isListening()
{
  return (true);
}

void MeSerial::sendString(char *str)
{
  uart_puts(UART_ID,str);
}


