// Filename: lcd_EduBoost.h
// Description: Module to control the color 128x128 TFT LCD display on Educational BoosterPack MKII
//
// Last Modified:  March 21, 2017 by Chris Siu

#ifndef LCD_H_
#define LCD_H_

// dimensions of the LCD
#define LCD_MAX_X 127  // x coordinate 0 to 127
#define LCD_MAX_Y 127  // y coordinate 0 to 127

// A few basic colour options to output to the LCD
// the LCD is configured in 16 bit colour mode (64K possible colours)
// bits 0 to 4 control blue intensity
// bits 5 to 10 control green instensisty
// bits 11 to 15 control red intensity
#define BLACK    (0x0000)
#define WHITE    (0xFFFF)
#define RED      (0xF800)
#define GREEN    (0x07E0)
#define BLUE     (0x001F)
#define YELLOW   (RED | GREEN)
#define CYAN     (GREEN | BLUE)
#define MAGENTA  (RED | BLUE)

// Parameters for 6x8 fonts
#define WIDTH_6x8 6
#define HEIGHT_6x8 8
#define MAX_COLUMNS_6x8 (LCD_MAX_X + 1)/WIDTH_6x8   //Maximum string length for LCD display
#define MAX_ROWS_6x8    (LCD_MAX_Y + 1)/HEIGHT_6x8

// Bitmap for 6x8 fonts
static const unsigned char Terminal6x8e[224][6] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x06, 0x5F, 0x06, 0x00,
    0x00, 0x07, 0x03, 0x00, 0x07, 0x03,
    0x00, 0x24, 0x7E, 0x24, 0x7E, 0x24,
    0x00, 0x24, 0x2B, 0x6A, 0x12, 0x00,
    0x00, 0x63, 0x13, 0x08, 0x64, 0x63,
    0x00, 0x36, 0x49, 0x56, 0x20, 0x50,
    0x00, 0x00, 0x07, 0x03, 0x00, 0x00,
    0x00, 0x00, 0x3E, 0x41, 0x00, 0x00,
    0x00, 0x00, 0x41, 0x3E, 0x00, 0x00,
    0x00, 0x08, 0x3E, 0x1C, 0x3E, 0x08,
    0x00, 0x08, 0x08, 0x3E, 0x08, 0x08,
    0x00, 0x00, 0xE0, 0x60, 0x00, 0x00,
    0x00, 0x08, 0x08, 0x08, 0x08, 0x08,
    0x00, 0x00, 0x60, 0x60, 0x00, 0x00,
    0x00, 0x20, 0x10, 0x08, 0x04, 0x02,
    0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E,
    0x00, 0x00, 0x42, 0x7F, 0x40, 0x00,
    0x00, 0x62, 0x51, 0x49, 0x49, 0x46,
    0x00, 0x22, 0x49, 0x49, 0x49, 0x36,
    0x00, 0x18, 0x14, 0x12, 0x7F, 0x10,
    0x00, 0x2F, 0x49, 0x49, 0x49, 0x31,
    0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30,
    0x00, 0x01, 0x71, 0x09, 0x05, 0x03,
    0x00, 0x36, 0x49, 0x49, 0x49, 0x36,
    0x00, 0x06, 0x49, 0x49, 0x29, 0x1E,
    0x00, 0x00, 0x6C, 0x6C, 0x00, 0x00,
    0x00, 0x00, 0xEC, 0x6C, 0x00, 0x00,
    0x00, 0x08, 0x14, 0x22, 0x41, 0x00,
    0x00, 0x24, 0x24, 0x24, 0x24, 0x24,
    0x00, 0x00, 0x41, 0x22, 0x14, 0x08,
    0x00, 0x02, 0x01, 0x59, 0x09, 0x06,
    0x00, 0x3E, 0x41, 0x5D, 0x55, 0x1E,
    0x00, 0x7E, 0x11, 0x11, 0x11, 0x7E,
    0x00, 0x7F, 0x49, 0x49, 0x49, 0x36,
    0x00, 0x3E, 0x41, 0x41, 0x41, 0x22,
    0x00, 0x7F, 0x41, 0x41, 0x41, 0x3E,
    0x00, 0x7F, 0x49, 0x49, 0x49, 0x41,
    0x00, 0x7F, 0x09, 0x09, 0x09, 0x01,
    0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A,
    0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F,
    0x00, 0x00, 0x41, 0x7F, 0x41, 0x00,
    0x00, 0x30, 0x40, 0x40, 0x40, 0x3F,
    0x00, 0x7F, 0x08, 0x14, 0x22, 0x41,
    0x00, 0x7F, 0x40, 0x40, 0x40, 0x40,
    0x00, 0x7F, 0x02, 0x04, 0x02, 0x7F,
    0x00, 0x7F, 0x02, 0x04, 0x08, 0x7F,
    0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E,
    0x00, 0x7F, 0x09, 0x09, 0x09, 0x06,
    0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E,
    0x00, 0x7F, 0x09, 0x09, 0x19, 0x66,
    0x00, 0x26, 0x49, 0x49, 0x49, 0x32,
    0x00, 0x01, 0x01, 0x7F, 0x01, 0x01,
    0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F,
    0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F,
    0x00, 0x3F, 0x40, 0x3C, 0x40, 0x3F,
    0x00, 0x63, 0x14, 0x08, 0x14, 0x63,
    0x00, 0x07, 0x08, 0x70, 0x08, 0x07,
    0x00, 0x71, 0x49, 0x45, 0x43, 0x00,
    0x00, 0x00, 0x7F, 0x41, 0x41, 0x00,
    0x00, 0x02, 0x04, 0x08, 0x10, 0x20,
    0x00, 0x00, 0x41, 0x41, 0x7F, 0x00,
    0x00, 0x04, 0x02, 0x01, 0x02, 0x04,
    0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
    0x00, 0x00, 0x03, 0x07, 0x00, 0x00,
    0x00, 0x20, 0x54, 0x54, 0x54, 0x78,
    0x00, 0x7F, 0x44, 0x44, 0x44, 0x38,
    0x00, 0x38, 0x44, 0x44, 0x44, 0x28,
    0x00, 0x38, 0x44, 0x44, 0x44, 0x7F,
    0x00, 0x38, 0x54, 0x54, 0x54, 0x08,
    0x00, 0x08, 0x7E, 0x09, 0x09, 0x00,
    0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C,
    0x00, 0x7F, 0x04, 0x04, 0x78, 0x00,
    0x00, 0x00, 0x00, 0x7D, 0x40, 0x00,
    0x00, 0x40, 0x80, 0x84, 0x7D, 0x00,
    0x00, 0x7F, 0x10, 0x28, 0x44, 0x00,
    0x00, 0x00, 0x00, 0x7F, 0x40, 0x00,
    0x00, 0x7C, 0x04, 0x18, 0x04, 0x78,
    0x00, 0x7C, 0x04, 0x04, 0x78, 0x00,
    0x00, 0x38, 0x44, 0x44, 0x44, 0x38,
    0x00, 0xFC, 0x44, 0x44, 0x44, 0x38,
    0x00, 0x38, 0x44, 0x44, 0x44, 0xFC,
    0x00, 0x44, 0x78, 0x44, 0x04, 0x08,
    0x00, 0x08, 0x54, 0x54, 0x54, 0x20,
    0x00, 0x04, 0x3E, 0x44, 0x24, 0x00,
    0x00, 0x3C, 0x40, 0x20, 0x7C, 0x00,
    0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C,
    0x00, 0x3C, 0x60, 0x30, 0x60, 0x3C,
    0x00, 0x6C, 0x10, 0x10, 0x6C, 0x00,
    0x00, 0x9C, 0xA0, 0x60, 0x3C, 0x00,
    0x00, 0x64, 0x54, 0x54, 0x4C, 0x00,
    0x00, 0x08, 0x3E, 0x41, 0x41, 0x00,
    0x00, 0x00, 0x00, 0x77, 0x00, 0x00,
    0x00, 0x00, 0x41, 0x41, 0x3E, 0x08,
    0x00, 0x02, 0x01, 0x02, 0x01, 0x00,
    0x00, 0x3C, 0x26, 0x23, 0x26, 0x3C,
    0x00, 0x14, 0x3E, 0x55, 0x41, 0x21,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xE0, 0x60, 0x00, 0x00,
    0x00, 0x40, 0x34, 0x0E, 0x05, 0x00,
    0x00, 0xE0, 0x60, 0x00, 0xE0, 0x60,
    0x00, 0x40, 0x00, 0x40, 0x00, 0x40,
    0x00, 0x00, 0x02, 0x3F, 0x02, 0x00,
    0x00, 0x00, 0x12, 0x3F, 0x12, 0x00,
    0x00, 0x00, 0x02, 0x01, 0x02, 0x00,
    0x00, 0x63, 0x13, 0x48, 0x04, 0x43,
    0x00, 0x4C, 0x55, 0x56, 0x55, 0x64,
    0x00, 0x00, 0x08, 0x14, 0x00, 0x00,
    0x00, 0x3E, 0x41, 0x7F, 0x49, 0x41,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x64, 0x55, 0x56, 0x4D, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x06, 0x07, 0x00, 0x00,
    0x00, 0x00, 0x07, 0x03, 0x00, 0x00,
    0x00, 0x07, 0x06, 0x00, 0x07, 0x06,
    0x00, 0x03, 0x07, 0x00, 0x03, 0x07,
    0x00, 0x00, 0x18, 0x18, 0x00, 0x00,
    0x00, 0x08, 0x08, 0x08, 0x00, 0x00,
    0x00, 0x08, 0x08, 0x08, 0x08, 0x08,
    0x00, 0x10, 0x08, 0x10, 0x08, 0x00,
    0x01, 0x07, 0x01, 0x07, 0x01, 0x07,
    0x00, 0x08, 0x55, 0x56, 0x55, 0x20,
    0x00, 0x00, 0x14, 0x08, 0x00, 0x00,
    0x00, 0x38, 0x44, 0x38, 0x54, 0x48,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x64, 0x55, 0x56, 0x4D, 0x00,
    0x00, 0x04, 0x09, 0x70, 0x09, 0x04,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x60, 0xFA, 0x60, 0x00,
    0x00, 0x18, 0x24, 0x66, 0x24, 0x00,
    0x00, 0x48, 0x3E, 0x49, 0x41, 0x22,
    0x00, 0x5D, 0x22, 0x22, 0x22, 0x5D,
    0x00, 0x29, 0x2A, 0x7C, 0x2A, 0x29,
    0x00, 0x00, 0x00, 0x77, 0x00, 0x00,
    0x00, 0x26, 0x4D, 0x55, 0x59, 0x32,
    0x00, 0x00, 0x02, 0x00, 0x02, 0x00,
    0x3E, 0x41, 0x5D, 0x55, 0x41, 0x3E,
    0x00, 0x08, 0x55, 0x55, 0x55, 0x5E,
    0x00, 0x08, 0x14, 0x00, 0x08, 0x14,
    0x00, 0x04, 0x04, 0x04, 0x04, 0x1C,
    0x00, 0x00, 0x30, 0x7D, 0x30, 0x00,
    0x3E, 0x41, 0x5D, 0x4B, 0x55, 0x3E,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x4E, 0x51, 0x51, 0x4E, 0x00,
    0x00, 0x00, 0x24, 0x2E, 0x24, 0x00,
    0x00, 0x09, 0x0D, 0x0A, 0x00, 0x00,
    0x00, 0x09, 0x0F, 0x05, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x01, 0x00,
    0x00, 0xFC, 0x20, 0x20, 0x1C, 0x00,
    0x00, 0x06, 0x09, 0x7F, 0x01, 0x7F,
    0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x80, 0xC0, 0x40, 0x00,
    0x00, 0x02, 0x0F, 0x00, 0x00, 0x00,
    0x00, 0x06, 0x09, 0x09, 0x06, 0x00,
    0x00, 0x14, 0x08, 0x00, 0x14, 0x08,
    0x00, 0x17, 0x08, 0x34, 0x2A, 0x78,
    0x00, 0x17, 0x08, 0x4C, 0x6A, 0x50,
    0x05, 0x17, 0x0A, 0x34, 0x2A, 0x79,
    0x00, 0x30, 0x48, 0x4D, 0x40, 0x20,
    0x00, 0x70, 0x29, 0x25, 0x28, 0x70,
    0x00, 0x70, 0x28, 0x25, 0x29, 0x70,
    0x00, 0x70, 0x29, 0x25, 0x29, 0x70,
    0x00, 0x70, 0x2A, 0x25, 0x2A, 0x71,
    0x00, 0x70, 0x29, 0x24, 0x29, 0x70,
    0x00, 0x70, 0x2A, 0x2D, 0x2A, 0x70,
    0x00, 0x7E, 0x11, 0x7F, 0x49, 0x41,
    0x00, 0x1E, 0xA1, 0xE1, 0x61, 0x12,
    0x00, 0x7C, 0x55, 0x55, 0x54, 0x44,
    0x00, 0x7C, 0x54, 0x55, 0x55, 0x44,
    0x00, 0x7C, 0x55, 0x55, 0x55, 0x44,
    0x00, 0x7C, 0x55, 0x54, 0x55, 0x44,
    0x00, 0x00, 0x45, 0x7D, 0x44, 0x00,
    0x00, 0x00, 0x44, 0x7D, 0x45, 0x00,
    0x00, 0x00, 0x45, 0x7D, 0x45, 0x00,
    0x00, 0x00, 0x45, 0x7C, 0x45, 0x00,
    0x00, 0x08, 0x7F, 0x49, 0x41, 0x3E,
    0x00, 0x7A, 0x11, 0x22, 0x79, 0x00,
    0x00, 0x3D, 0x43, 0x42, 0x3C, 0x00,
    0x00, 0x3C, 0x42, 0x43, 0x3D, 0x00,
    0x00, 0x3C, 0x43, 0x43, 0x3D, 0x00,
    0x00, 0x3E, 0x45, 0x42, 0x3D, 0x00,
    0x00, 0x3C, 0x42, 0x43, 0x3D, 0x00,
    0x00, 0x00, 0x28, 0x10, 0x28, 0x00,
    0x00, 0x7E, 0x61, 0x5D, 0x43, 0x3F,
    0x00, 0x3C, 0x40, 0x41, 0x3D, 0x00,
    0x00, 0x3D, 0x41, 0x40, 0x3C, 0x00,
    0x00, 0x3C, 0x41, 0x41, 0x3D, 0x00,
    0x00, 0x3C, 0x41, 0x40, 0x3D, 0x00,
    0x00, 0x04, 0x08, 0x71, 0x09, 0x04,
    0x00, 0xFE, 0xAA, 0x28, 0x10, 0x00,
    0x00, 0xFE, 0x4A, 0x4A, 0x34, 0x00,
    0x00, 0x20, 0x55, 0x55, 0x54, 0x78,
    0x00, 0x20, 0x54, 0x54, 0x55, 0x79,
    0x00, 0x20, 0x55, 0x55, 0x55, 0x78,
    0x00, 0x20, 0x56, 0x55, 0x56, 0x7D,
    0x00, 0x20, 0x55, 0x54, 0x55, 0x78,
    0x00, 0x20, 0x57, 0x55, 0x57, 0x7C,
    0x00, 0xFC, 0x20, 0x20, 0x1C, 0x00,
    0x00, 0x38, 0xC4, 0xC4, 0x44, 0x28,
    0x00, 0x39, 0x55, 0x54, 0x08, 0x00,
    0x00, 0x38, 0x54, 0x55, 0x09, 0x00,
    0x00, 0x38, 0x55, 0x55, 0x09, 0x00,
    0x00, 0x38, 0x55, 0x54, 0x09, 0x00,
    0x00, 0x00, 0x01, 0x7D, 0x40, 0x00,
    0x00, 0x00, 0x00, 0x7D, 0x41, 0x00,
    0x00, 0x00, 0x01, 0x7D, 0x41, 0x00,
    0x00, 0x00, 0x01, 0x7C, 0x41, 0x00,
    0x00, 0x39, 0x47, 0x45, 0x46, 0x3D,
    0x00, 0x7A, 0x09, 0x0A, 0x71, 0x00,
    0x00, 0x39, 0x45, 0x44, 0x38, 0x00,
    0x00, 0x38, 0x44, 0x45, 0x39, 0x00,
    0x00, 0x38, 0x45, 0x45, 0x39, 0x00,
    0x00, 0x3A, 0x45, 0x46, 0x39, 0x00,
    0x00, 0x38, 0x45, 0x44, 0x39, 0x00,
    0x00, 0x08, 0x08, 0x2A, 0x08, 0x08,
    0x00, 0x58, 0x24, 0x54, 0x48, 0x34,
    0x00, 0x3D, 0x41, 0x20, 0x7C, 0x00,
    0x00, 0x3C, 0x40, 0x21, 0x7D, 0x00,
    0x00, 0x3C, 0x41, 0x21, 0x7D, 0x00,
    0x00, 0x3C, 0x41, 0x20, 0x7D, 0x00,
    0x00, 0x9C, 0xA0, 0x61, 0x3D, 0x00,
    0x00, 0xFF, 0xA5, 0x24, 0x18, 0x00,
    0x00, 0x9C, 0xA1, 0x60, 0x3D, 0x00
};


//Function Prototypes

void lcdClear(int colour);

void lcdInit();

void lcdSetPixel(int x, int y, int colour);

void lcdFont (int x, int y, char ascii_char, int text_color, int backgnd_color);

void lcdWrite6x8 (int row, int col, char *str, int text_color, int backgnd_color);

#endif /* LCD_H_ */
