#include "radioInterface.h"
/*
  Assuming Serial has already been initialized, send the data
*/
bool initRadio()
{
  RADIO_UART.println("INIT");
  char data[200];
  int bytesRead = RADIO_UART.readBytesUntil('\n', data, 200);
  data[bytesRead] = '\0';

  if (strncmp(data, "OK", bytesRead))
  {
    return true;
  }
  else
  {
    return false;
  }
}
bool transmitData(char *data, uint32_t timestamp)
{
  char buf[128];
  snprintf("%lu: %s", timestamp, data);
  int bytesWritten = RADIO_UART.println(buf);
  if (bytesWritten)
  {
    return true;
  }
  else
  {
    return false;
  }
}
char *recordSerialData()
{
}