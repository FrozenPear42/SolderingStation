#ifndef OLED_SSD1306_H
#define OLED_SSD1306_H

#define SSD1306_FONTS

#include <stdint.h>
#include "stm32f1xx_hal.h"

#define SSD1306_DEFAULT_ADDRESS     0x78
#define SSD1306_SETCONTRAST         0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON        0xA5
#define SSD1306_NORMALDISPLAY       0xA6
#define SSD1306_INVERTDISPLAY       0xA7
#define SSD1306_DISPLAYOFF          0xAE
#define SSD1306_DISPLAYON           0xAF
#define SSD1306_SETDISPLAYOFFSET    0xD3
#define SSD1306_SETCOMPINS          0xDA
#define SSD1306_SETVCOMDETECT       0xDB
#define SSD1306_SETDISPLAYCLOCKDIV  0xD5
#define SSD1306_SETPRECHARGE        0xD9
#define SSD1306_SETMULTIPLEX        0xA8
#define SSD1306_SETLOWCOLUMN        0x00
#define SSD1306_SETHIGHCOLUMN       0x10
#define SSD1306_SETSTARTLINE        0x40
#define SSD1306_MEMORYMODE          0x20
#define SSD1306_COLUMNADDR          0x21
#define SSD1306_PAGEADDR            0x22
#define SSD1306_COMSCANINC          0xC0
#define SSD1306_COMSCANDEC          0xC8
#define SSD1306_SEGREMAP            0xA0
#define SSD1306_CHARGEPUMP          0x8D
#define SSD1306_SWITCHCAPVCC        0x02
#define SSD1306_NOP                 0xE3

void SSD1306_init(I2C_HandleTypeDef* pI2C);
void SSD1306_enable(I2C_HandleTypeDef* pI2C, uint8_t pEnable);
void SSD1306_invert(I2C_HandleTypeDef* pI2C, uint8_t pInvert);
void SSD1306_sendFrameBuffer(I2C_HandleTypeDef* pI2C, uint8_t* pBuffer);

void SSD1306_sendCommand(I2C_HandleTypeDef* pI2C, uint8_t pCommand);

void SSD1306_clear(I2C_HandleTypeDef* pI2C);
void SSD1306_drawGlyph(I2C_HandleTypeDef* pI2C, uint8_t pX, uint8_t pY, uint8_t* pGlyph);
void SSD1306_drawChar(I2C_HandleTypeDef* pI2C, uint8_t pX, uint8_t pY, char pChar);
void SSD1306_drawString(I2C_HandleTypeDef* pI2C, uint8_t pX, uint8_t pY, char* pString, uint8_t pLen);
void SSD1306_drawStringRight(I2C_HandleTypeDef* pI2C, uint8_t pX, uint8_t pY, char* pString, uint8_t pLen);

uint8_t* getGlyph(char pGlyph);

void FrameBuffer_drawPixel(uint8_t* pBuffer, uint8_t pX, uint8_t pY, uint8_t pOn);
void FrameBuffer_drawVLine(uint8_t* pBuffer, uint8_t pX, uint8_t pY, uint8_t pLength);
void FrameBuffer_drawHLine(uint8_t* pBuffer, uint8_t pX, uint8_t pY, uint8_t pLength);
void FrameBuffer_drawGlyph(uint8_t* pBuffer, uint8_t pX, uint8_t pY, uint8_t* pGlyph);
void FrameBuffer_drawChar(uint8_t* pBuffer, uint8_t pX, uint8_t pY, char pChar);
void FrameBuffer_drawString(uint8_t* pBuffer, uint8_t pX, uint8_t pY, char* pString, uint8_t pLen);
#endif //OLED_SSD1306_H
