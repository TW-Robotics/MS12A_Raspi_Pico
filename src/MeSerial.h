#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <string>
#pragma once

#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8 //brauchen wir wsl nicht
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

#define UART_TX_PIN 0 //Pico UART 1 siehe Pinout
#define UART_RX_PIN 1

class MeSerial
{
public:

  MeSerial(uint8_t receivePin, uint8_t transmitPin);
  void begin(long baudrate);
  size_t write(uint8_t byte);
  int16_t read();
  int16_t available();
  void end();
  bool listen();
  bool isListening();
  void sendString(char *str);

protected:
  int16_t _bitPeriod;
  int16_t _byte;
  long _lastTime;
  char buffer[64];
  std::string lastLine;
  int bufferIndex;

private:
  volatile uint8_t _RxPin;
  volatile uint8_t _TxPin; 
};

