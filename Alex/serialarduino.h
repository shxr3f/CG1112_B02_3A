#ifndef __SERIALARDUINO_H__
#define __SERIALARDUINO_H__

// Response types. This goes into the command field



extern char dataSend;
void setupSerial();
void startSerial();
int readSerial(char *buffer);
void writeSerial(const unsigned char *buffer, int len);


#endif
