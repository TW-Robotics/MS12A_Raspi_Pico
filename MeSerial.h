#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"


#define UART_ID uart1
#define BAUD_RATE 115200
//#define DATA_BITS 8 //brauchen wir wsl nicht
//#define STOP_BITS 1
//#define PARITY    UART_PARITY_NONE

#define UART_TX_PIN 6 //Pico UART 1 siehe Pinout
#define UART_RX_PIN 7

class MeSerial
{
public:

  MeSerial(uint8_t receivePin, uint8_t transmitPin);
  void begin(long baudrate);
  size_t write(uint8_t byte);
  int read();
  int available();
  void end(void);
  bool listen(void);
  bool isListening(void);
  void sendString(char *str);

protected:
  int16_t _bitPeriod;
  int16_t _byte;
  long _lastTime;
  char buffer[64];
  String lastLine;
  int bufferIndex;

private:
  volatile uint8_t _RxPin;
  volatile uint8_t _TxPin; 
};

