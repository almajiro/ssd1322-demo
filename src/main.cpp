#include <SPI.h>
#include <Arduino.h>
#include "screen.h"
#include "max7219.h"

#define CS     D8     // Chip Select
#define DC     D4     // Data/Command Control PIN
#define RESET  D1     // Reset PIN

#define MAXCS  D0     // MAX7219 CS
#define MCPCS  D2     // MCP23017 CS
#define MCPW   0x40
#define MCPR   0x41
#define MCPINT D3

#define CONTROL_WORD 0x04
#define COMMAND_WORD 0x02
#define WRITE_START 0xe0
#define WRITE_END 0xf0

#define SERIAL_BUFFER_SIZE 8192

byte readRegisterValueFromSerial();
ICACHE_RAM_ATTR void readMcp();

uint8_t buffer[8192];
uint8_t maxBuffer[8];
bool buttonPressed = false;
uint8_t buttonState = 0x00;
unsigned long prevPressedTime = 0;

SPISettings SSD1322SSPI(10000000, MSBFIRST, SPI_MODE0);
//SPISettings SSD1322SSPI(24000000, MSBFIRST, SPI_MODE0);
SPISettings MAX7219SPI(10000000, MSBFIRST, SPI_MODE0);
SPISettings MCPSPI(10000000, MSBFIRST, SPI_MODE0);

void ssd1322_command(byte command)
{
  SPI.beginTransaction(SSD1322SSPI);
  digitalWrite(CS, HIGH);
  digitalWrite(DC, LOW);
  digitalWrite(CS, LOW);
  SPI.transfer(command);
  digitalWrite(CS, HIGH);
  SPI.endTransaction();
}

void ssd1322_data(byte command)
{
  SPI.beginTransaction(SSD1322SSPI);
  digitalWrite(CS, HIGH);
  digitalWrite(DC, HIGH);
  digitalWrite(CS, LOW);
  SPI.transfer(command);
  digitalWrite(CS, HIGH);
  SPI.endTransaction();
}

void ssd1322_writebytes(uint8_t * data, uint32_t size)
{
  SPI.beginTransaction(SSD1322SSPI);
  digitalWrite(CS, HIGH);
  digitalWrite(DC, HIGH);
  digitalWrite(CS, LOW);
  //SPI.writeBytes(data, size);
  SPI.transfer(data, size);
  digitalWrite(CS, HIGH);
  SPI.endTransaction();
}

void write_mcp(byte address, byte value)
{
  SPI.beginTransaction(MCPSPI);
  digitalWrite(MCPCS, LOW);
  SPI.transfer(MCPW);
  SPI.transfer(address);
  SPI.transfer(value);
  SPI.endTransaction();
  digitalWrite(MCPCS, HIGH);
}

byte read_mcp(byte address)
{
  byte value;
  SPI.beginTransaction(MCPSPI);
  digitalWrite(MCPCS, LOW);
  SPI.transfer(MCPR);
  SPI.transfer(address);
  value = SPI.transfer(0x00);
  SPI.endTransaction();
  digitalWrite(MCPCS, HIGH);

  buttonState = value;
  return value;
}

ICACHE_RAM_ATTR void readMcp()
{
  buttonPressed = true;
}

void maxWrite(int address, byte data)
{
  SPI.beginTransaction(MAX7219SPI);
  digitalWrite(MAXCS, LOW);
  SPI.transfer(address);
  SPI.transfer(data);
  digitalWrite(MAXCS, HIGH);
  SPI.endTransaction();
}

void maxWriteBuffer(int digit, int number)
{
  uint8_t mask = 0x80;
  uint8_t digitMask = 0x80 >> (digit+1);
  for (int i=0; i<8; i++) {
    if (bitmapNumbers[number] & mask) {
      maxBuffer[i] = maxBuffer[i] | digitMask;
    } else {
      maxBuffer[i] = maxBuffer[i] & ~digitMask;
    }
    mask >>= 1;
  }
}

void maxDisplay()
{
  for (int i=0; i<8; i++) {
    maxWrite(i+1, maxBuffer[i]);
  }
}

void setup() {
  pinMode(RESET, OUTPUT);
  pinMode(CS, OUTPUT);
  pinMode(DC, OUTPUT);
  pinMode(MCPINT, INPUT_PULLUP);
  pinMode(MAXCS, OUTPUT);
  pinMode(MCPCS, OUTPUT);

  digitalWrite(MCPCS, HIGH);
  digitalWrite(MAXCS, HIGH);
  digitalWrite(CS, HIGH);
  digitalWrite(DC, LOW);

  // Initialize Display
  digitalWrite(RESET, HIGH);
  delay(100);
  digitalWrite(RESET, LOW);
  delay(400);
  digitalWrite(RESET, HIGH);

  SPI.begin();

  write_mcp(0x0A, 0B00100000);



  // GPIOB ALL INPUT
  write_mcp(0x01, 0xFF);

  // ENABLE PULLUP ON GPIOB
  write_mcp(0x0D, 0xFF);

  // INVERT VALUE ON GPIOB
  write_mcp(0x03, 0xFF);

  // ENABLE INTERRUPT ON GPIOA
  write_mcp(0x05, 0xFF);

  ssd1322_command(SSD1322_SETCOMMANDLOCK);// 0xFD
	ssd1322_data(0x12);// Unlock OLED driver IC

	ssd1322_command(SSD1322_DISPLAYOFF);// 0xAE

	ssd1322_command(SSD1322_SETCLOCKDIVIDER);// 0xB3
	//ssd1322_data(0xA1);// 0xB3
	ssd1322_data(0xB1);// 0xB3

	ssd1322_command(SSD1322_SETMUXRATIO);// 0xCA
	ssd1322_data(0x3F);// duty = 1/64


	ssd1322_command(SSD1322_SETDISPLAYOFFSET);// 0xA2
	ssd1322_data(0x00);

	ssd1322_command(SSD1322_SETSTARTLINE);// 0xA1
	ssd1322_data(0x00);

	ssd1322_command(SSD1322_SETREMAP);// 0xA0
	ssd1322_data(0x14);//Horizontal address increment,Disable Column Address Re-map,Enable Nibble Re-map,Scan from COM[N-1] to COM0,Disable COM Split Odd Even
	ssd1322_data(0x11);//Enable Dual COM mode

	ssd1322_command(SSD1322_SETGPIO);// 0xB5
	ssd1322_data(0x00);// Disable GPIO Pins Input

	ssd1322_command(SSD1322_FUNCTIONSEL);// 0xAB
	ssd1322_data(0x01);// selection external vdd

	ssd1322_command(SSD1322_DISPLAYENHANCE);// 0xB4
	ssd1322_data(0xA0);// enables the external VSL
	//ssd1322_data(0xFD);// 0xfFD,Enhanced low GS display quality;default is 0xb5(normal),

	ssd1322_command(SSD1322_SETCONTRASTCURRENT);// 0xC1
	ssd1322_data(0xFF);// 0xFF - default is 0x7f

	ssd1322_command(SSD1322_MASTERCURRENTCONTROL);// 0xC7
	ssd1322_data(0x0F);// default is 0x0F

	// Set grayscale
	ssd1322_command(SSD1322_SELECTDEFAULTGRAYSCALE); // 0xB9

 	ssd1322_command(SSD1322_SETPHASELENGTH);// 0xB1
	ssd1322_data(0xE2);// default is 0x74

	ssd1322_command(SSD1322_DISPLAYENHANCEB);// 0xD1
	ssd1322_data(0x82);// Reserved;default is 0xa2(normal)
	ssd1322_data(0x20);//

	ssd1322_command(SSD1322_SETPRECHARGEVOLTAGE);// 0xBB
	ssd1322_data(0x1F);// 0.6xVcc

	ssd1322_command(SSD1322_SETSECONDPRECHARGEPERIOD);// 0xB6
	ssd1322_data(0x08);// default

	ssd1322_command(SSD1322_SETVCOMH);// 0xBE
	ssd1322_data(0x07);// 0.86xVcc;default is 0x04

	ssd1322_command(SSD1322_NORMALDISPLAY);// 0xA6

	ssd1322_command(SSD1322_EXITPARTIALDISPLAY);// 0xA9


  ssd1322_command(SSD1322_SETCOLUMNADDR);
  ssd1322_data(MIN_SEG);
  ssd1322_data(MAX_SEG);

  ssd1322_command(SSD1322_SETROWADDR);
  ssd1322_data(0);
  ssd1322_data(63);

  ssd1322_command(SSD1322_WRITERAM);

  ssd1322_writebytes(screen1, 8192);

	ssd1322_command(SSD1322_DISPLAYON);// 0xAF

  //Serial.begin(1000000);
  //Serial.begin(2000000);
  //Serial.begin(2000000, SERIAL_8E1);
  Serial.begin(1209600, SERIAL_8E1);
  //Serial.begin(9600);


  digitalWrite(CS, LOW);

  SPI.setFrequency(800000);
  // turn on
  digitalWrite(MAXCS, LOW);
  SPI.transfer(0x0C);
  SPI.transfer(0x01);
  digitalWrite(MAXCS, HIGH);

  // intensity max
  digitalWrite(MAXCS, LOW);
  SPI.transfer(0x0A);
  SPI.transfer(0x0F);
  digitalWrite(MAXCS, HIGH);

  // scan limit 8
  digitalWrite(MAXCS, LOW);
  SPI.transfer(0x0B);
  SPI.transfer(0x07);
  digitalWrite(MAXCS, HIGH);

  // Set Raw data Mode
  digitalWrite(MAXCS, LOW);
  SPI.transfer(0x09);
  SPI.transfer(0x00);
  digitalWrite(MAXCS, HIGH);

  for (int i=0; i<8; i++) {
    maxBuffer[i] = 0x00;
  }

  maxWriteBuffer(0, 1);
  maxWriteBuffer(1, 2);
  maxWriteBuffer(2, 3);
  maxWriteBuffer(3, 4);

  maxDisplay();

  // GPIOA ALL OUTPUT
  write_mcp(0x00, 0x00);

  // WRITE GPIOA
  for(int j=0; j<2; j++) {
    byte led = 0x01;
    for (int i=0; i<8; i++) {
      write_mcp(0x12, led);
      led <<= 1;
      delay(20);
    }

    led = 0x80;
    for (int i=0; i<8; i++) {
      write_mcp(0x12, led);
      led >>= 1;
      delay(20);
    }
  }

  //attachInterrupt(digitalPinToInterrupt(MCPINT), readMcp, FALLING);
}

void loop() {
  if (((millis() - prevPressedTime) > 10) && buttonPressed == true) {
    Serial.write(read_mcp(0x11));
    buttonPressed = false;
    prevPressedTime = millis();
  } else if(buttonPressed == true) {
    read_mcp(0x11);
    buttonPressed = false;
  }

  int operation = readRegisterValueFromSerial();

  switch (operation) {
    case 0xE4:
      for (uint16_t i=0; i<8192; i++) {
        buffer[i] = readRegisterValueFromSerial();
      }
      ssd1322_command(SSD1322_WRITERAM);
      ssd1322_writebytes(buffer, 8192);
      break;
    case 0xE5:
      for (int i=0; i<8; i++) {
        maxWriteBuffer(i, readRegisterValueFromSerial());
      }

      maxDisplay();
      break;

    case 0xE6:
      write_mcp(0x12, readRegisterValueFromSerial());
      break;
  }
}

byte readRegisterValueFromSerial()
{
  while (Serial.available() <= 0);
  return Serial.read();
}
