#include "MeSerial.h"

#define ALL_DEVICE              0xff    // Broadcast command identifies
#define CUSTOM_TYPE             0x00    // 0x00 indicates no external module

#define CTL_ASSIGN_DEV_ID       0x10 // Assignment device ID
#define CTL_SYSTEM_RESET        0x11 // reset from host
#define CTL_READ_DEV_VERSION    0x12 // read the firmware version
#define CTL_SET_BAUD_RATE       0x13 // Set the bandrate
#define CTL_CMD_TEST            0x14 // Just for test
#define CTL_ERROR_CODE          0x15 // error code

#define SMART_SERVO              0x60
  /* Secondary command */
  #define SET_SERVO_PID                          0x10
  #define SET_SERVO_ABSOLUTE_POS                 0x11
  #define SET_SERVO_RELATIVE_POS                 0x12
  #define SET_SERVO_CONTINUOUS_ROTATION          0x13
  #define SET_SERVO_MOTION_COMPENSATION          0x14
  #define CLR_SERVO_MOTION_COMPENSATION          0x15
  #define SET_SERVO_BREAK                        0x16
  #define SET_SERVO_RGB_LED                      0x17
  #define SERVO_SHARKE_HAND                      0x18
  #define SET_SERVO_CMD_MODE                     0x19

  #define GET_SERVO_STATUS                       0x20
  #define GET_SERVO_PID                          0x21
  #define GET_SERVO_CUR_POS                      0x22
  #define GET_SERVO_SPEED                        0x23
  #define GET_SERVO_MOTION_COMPENSATION          0x24
  #define GET_SERVO_TEMPERATURE                  0x25
  #define GET_SERVO_ELECTRIC_CURRENT             0x26
  #define GET_SERVO_VOLTAGE                      0x27

  #define SET_SERVO_CURRENT_ANGLE_ZERO_DEGREES   0x30
  #define SET_SERVO_ABSOLUTE_ANGLE               0x31
  #define SET_SERVO_RELATIVE_ANGLE               0x32
  #define SET_SERVO_ABSOLUTE_ANGLE_LONG          0x33
  #define SET_SERVO_RELATIVE_ANGLE_LONG          0x34
  #define SET_SERVO_PWM_MOVE                     0x35
  #define GET_SERVO_CUR_ANGLE                    0x36
  #define SET_SERVO_INIT_ANGLE                   0x37
  #define REPORT_WHEN_REACH_THE_SET_POSITION     0x40

#define START_SYSEX             0xF0 // start a MIDI Sysex message
#define END_SYSEX               0xF7 // end a MIDI Sysex message

/* report error code */
#define PROCESS_SUC             0x0F
#define PROCESS_BUSY            0x10
#define PROCESS_ERROR           0x11
#define WRONG_TYPE_OF_SERVICE   0x12

#define DEFAULT_UART_BUF_SIZE      64

typedef struct{
  uint8_t dev_id;
  uint8_t srv_id;
  uint8_t value[DEFAULT_UART_BUF_SIZE - 2];
}sysex_message_type;

typedef struct{
  uint8_t service_id;
  void (*request_fun)(void *arg);
  void (*response_fun)(void *arg);
}Cmd_list_tab_type;

union sysex_message{
  uint8_t storedInputData[DEFAULT_UART_BUF_SIZE];
  sysex_message_type val;
};

union{
  uint8_t byteVal[8];
  double doubleVal;
}val8byte;

union{
  uint8_t byteVal[4];
  float floatVal;
  long longVal;
}val4byte;

union{
  uint8_t byteVal[2];
  short shortVal;
}val2byte;

union{
  uint8_t byteVal[1];
  uint8_t charVal;
}val1byte;

typedef struct
{
  long angleValue;
  float servoSpeed;
  float voltage;
  float temperature;
  float current;
}servo_device_type;

typedef void (*smartServoCb)(uint8_t); 


class MeSmartServo : public MeSerial
{
public:

MeSmartServo(uint8_t receivePin, uint8_t transmitPin);

uint8_t readByte(uint8_t *argv,int16_t idx);
short readShort(uint8_t *argv,int16_t idx,boolean ignore_high);
float readFloat(uint8_t *argv,int16_t idx);
long readLong(uint8_t *argv,int idx);
uint8_t sendByte(uint8_t val);
uint8_t sendShort(int16_t val,boolean ignore_high);
uint8_t sendFloat(float val);
uint8_t sendLong(long val);
boolean assignDevIdRequest(void);
boolean moveTo(uint8_t dev_id,long angle_value,float speed,smartServoCb callback = NULL);
boolean move(uint8_t dev_id,long angle_value,float speed,smartServoCb callback = NULL);
boolean setZero(uint8_t dev_id);
boolean setBreak(uint8_t dev_id, uint8_t breakStatus);
boolean setRGBLed(uint8_t dev_id, uint8_t r_value, uint8_t g_value, uint8_t b_value);
boolean handSharke(uint8_t dev_id);
boolean setPwmMove(uint8_t dev_id, int16_t pwm_value);
boolean setInitAngle(uint8_t dev_id,uint8_t = 0,int16_t = 40);
long getAngleRequest(uint8_t devId);
float getSpeedRequest(uint8_t devId);
float getVoltageRequest(uint8_t devId);
float getTempRequest(uint8_t devId);
float getCurrentRequest(uint8_t devId);
void assignDevIdResponse(void *arg);
void errorCodeCheckResponse(void *arg);
void smartServoCmdResponse(void *arg);
void processSysexMessage(void);
void smartServoEventHandle(void);

private:
  union sysex_message sysex;
  volatile int16_t sysexBytesRead;
  volatile uint8_t servo_num_max;
  volatile uint16_t resFlag;
  volatile servo_device_type servo_dev_list[8];
  volatile long cmdTimeOutValue;
  volatile boolean parsingSysex;
  smartServoCb _callback;
};
