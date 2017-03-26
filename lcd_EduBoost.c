// Filename: lcd_EduBoost.c
// Description: Module to control the color 128x128 TFT LCD display on Educational BoosterPack MKII
//
// Last Modified:  March 21, 2017 by Chris Siu
//
// Connection to boosterpack LCD:
// P1.5 for UCB0 SCK connects to LCD SCK
// P1.6 for UCB0 MOSI connects to LCD SDA
// P5.7 connects to LCD RST pin
// P5.0 connects to LCD CS pin
// P3.7 connects to LCD DC pin

#include "msp.h"
#include "lcd_EduBoost.h"


// ST7735 LCD controller Command Set (copied from TI sample code)
#define CM_NOP             0x00
#define CM_SWRESET         0x01
#define CM_RDDID           0x04
#define CM_RDDST           0x09
#define CM_SLPIN           0x10
#define CM_SLPOUT          0x11
#define CM_PTLON           0x12
#define CM_NORON           0x13
#define CM_INVOFF          0x20
#define CM_INVON           0x21
#define CM_GAMSET          0x26
#define CM_DISPOFF         0x28
#define CM_DISPON          0x29
#define CM_CASET           0x2A
#define CM_RASET           0x2B
#define CM_RAMWR           0x2C
#define CM_RGBSET          0x2d
#define CM_RAMRD           0x2E
#define CM_PTLAR           0x30
#define CM_MADCTL          0x36
#define CM_COLMOD          0x3A
#define CM_SETPWCTR        0xB1
#define CM_SETDISPL        0xB2
#define CM_FRMCTR3         0xB3
#define CM_SETCYC          0xB4
#define CM_SETBGP          0xb5
#define CM_SETVCOM         0xB6
#define CM_SETSTBA         0xC0
#define CM_SETID           0xC3
#define CM_GETHID          0xd0
#define CM_SETGAMMA        0xE0
#define CM_MADCTL_MY       0x80
#define CM_MADCTL_MX       0x40
#define CM_MADCTL_MV       0x20
#define CM_MADCTL_ML       0x10
#define CM_MADCTL_BGR      0x08
#define CM_MADCTL_MH       0x04

// Correction factors for display offsets (deduced from TI sample code, but not completely sure why)
#define X_CORRECTION_OFFSET 2
#define Y_CORRECTION_OFFSET 1

//  LCD pins
#define LCD_SCK           BIT5  // P1.5 for UCB0 SCK connects to LCD SCK
#define LCD_MOSI          BIT6  // P1.6 for UCB0 MOSI connects to LCD SDA
#define LCD_RST           BIT7  // P5.7 connects to LCD RST pin
#define LCD_CS            BIT0  // P5.0 connects to LCD CS pin
#define LCD_DC            BIT7  // P3.7 connects to LCD DC pin

// define # CPU cycles for delays
#define _1ms (3018)
#define _10ms (10 * _1ms)
#define _50ms (50 * _1ms)
#define _120ms (120 * _1ms)
#define _200ms (200 * _1ms)

// Argument for isData in lcdWrite function
#define DATA 1
#define CMD 0


void lcdWrite(char byte, int isData);
void lcdMapPixel (int *x, int *y);


///////////////////////////////////////////////////////////////////////
// lcdInit - Initializes ports, SPI settings and LCD.
//           This function assumes CPU clock is less than 15MHz.
// Arguments: none
// Return Value: none
///////////////////////////////////////////////////////////////////////
void lcdInit()
{
    // enable pins for UCB0 SPI MOSI/SCK 
    P1SEL0 |= (LCD_SCK | LCD_MOSI);

    // enable CS, RST and DC outputs
    P5DIR |= (LCD_RST | LCD_CS);
    P3DIR |= (LCD_DC);

    // activate CS - it is ACTIVE LOW
    P5OUT &= ~LCD_CS;

    // ensure UCB0 SPI module in software reset
    UCB0CTLW0 |= EUSCI_B_CTLW0_SWRST;

	// configure SPI as master, MSB first, SMCLK as clock source,
    //	data captured on first clock edge, low clock inactive state
    UCB0CTLW0 |= (EUSCI_B_CTLW0_MST | EUSCI_B_CTLW0_MSB | EUSCI_B_CTLW0_SSEL__SMCLK);
    UCB0CTLW0 |= EUSCI_B_CTLW0_CKPH;
    UCB0BRW = 0x01;

    // take UCB0 SPI module out of software reset
    UCB0CTLW0 &= ~EUSCI_B_CTLW0_SWRST;

    // Reset the display with hardware reset
    P5OUT &= ~LCD_RST;
    __delay_cycles(_50ms);
    P5OUT |= LCD_RST;
    __delay_cycles(_120ms);

    // Sleep Out
    lcdWrite(CM_SLPOUT, CMD);
    __delay_cycles(_120ms);

    // Gamma Set
    lcdWrite(CM_GAMSET, CMD);
    lcdWrite(0x04, DATA);

    // Frame Rate Control
    lcdWrite(CM_SETPWCTR, CMD);
    lcdWrite(0x0A, DATA);
    lcdWrite(0x14, DATA);

    // Power Control 1
    lcdWrite(CM_SETSTBA, CMD);
    lcdWrite(0x0A, DATA);
    lcdWrite(0x00, DATA);

    // Interface Pixel Format
    lcdWrite(CM_COLMOD, CMD);
    lcdWrite(0x05, DATA);
    __delay_cycles(_10ms);

    // Memory Data Access Control
    lcdWrite(CM_MADCTL, CMD);
    lcdWrite(CM_MADCTL_BGR, DATA);

    // Normal Display Mode On
    lcdWrite(CM_NORON, CMD);

    // clear the display
    lcdClear(BLACK);

    // Display On
     __delay_cycles(_10ms);
    lcdWrite(CM_DISPON, CMD);
}


///////////////////////////////////////////////////////////////////////
// lcdClear - Clears the entire LCD display
// Arguments: none
// Return Value: colour - A 16 bit value to represent the RGB color to be
//                        clear all pixels to
///////////////////////////////////////////////////////////////////////
void lcdClear(int colour) {
	int row, col;  // variables used to track current pixel while clearing

	// set x range (Column Address Set)
	lcdWrite(CM_CASET, CMD);
	lcdWrite(0, DATA);
	lcdWrite(X_CORRECTION_OFFSET, DATA);
	lcdWrite(0, DATA);
	lcdWrite(LCD_MAX_X + X_CORRECTION_OFFSET, DATA);

	// set y range (Row Address Set)
    lcdWrite(CM_RASET, CMD);
	lcdWrite(0, DATA);
	lcdWrite(Y_CORRECTION_OFFSET, DATA);
	lcdWrite(0, DATA);
	lcdWrite(LCD_MAX_Y + Y_CORRECTION_OFFSET, DATA);

	// clear all pixels in range
    lcdWrite(CM_RAMWR, CMD);        //Memory Write
    for (row = 0; row <= LCD_MAX_Y; row++) {
        for (col = 0; col <= LCD_MAX_X; col++) {
        	// clear to desired colour
	        lcdWrite(colour >> 8, DATA);
            lcdWrite(colour, DATA);
        }
    }

}


///////////////////////////////////////////////////////////////////////
// lcdSetPixel - Sets the pixel colour at the specified display location.
//               The bottom left corner of the LCD is (0,0)
// Arguments: x - The horizontal location of the pixel
//            y - The vertical location of the pixel
//            colour - A 16 bit value to represent the RGB colour to be
//                     output at the pixel location
// Return Value: none
///////////////////////////////////////////////////////////////////////
void lcdSetPixel(int x, int y, int colour) {

	// set x co-ordinate for the pixel
	lcdWrite(CM_CASET, CMD);
	lcdWrite(0, DATA);
	lcdWrite(x + X_CORRECTION_OFFSET, DATA);
	lcdWrite(0, DATA);
	lcdWrite(x + X_CORRECTION_OFFSET, DATA);

	// set y co-ordinate for the pixel
    lcdWrite(CM_RASET, CMD);
	lcdWrite(0, DATA);
	lcdWrite(y + Y_CORRECTION_OFFSET, DATA);
	lcdWrite(0, DATA);
	lcdWrite(y + Y_CORRECTION_OFFSET, DATA);

	// output colour to the pixel location
    lcdWrite(CM_RAMWR, CMD);
    lcdWrite(colour >> 8, DATA);
    lcdWrite(colour, DATA);
}


///////////////////////////////////////////////////////////////////////
// lcdWrite - Sends a command/data byte to the LCD.
// Arguments: byte - value to be sent to the LCD
//            isData - true if byte is data, false if byte is a command
// Return Value: none
///////////////////////////////////////////////////////////////////////
void lcdWrite(char byte, int isData)
{
    // wait until SPI finished transmitting previous byte
    while ((UCB0STATW & EUSCI_B_STATW_BUSY) != 0)  ;
	
    // if byte is data
    if (isData == DATA)
        P3OUT |= LCD_DC;    // set data/command pin
    else
		P3OUT &= ~LCD_DC;   // clear data/command pin

    // Transmit byte
    UCB0TXBUF = byte;
}



///////////////////////////////////////////////////////////////////////
// lcdFont - writes a character to the LCD display, starting at any
//      pixel on the display
// Arguments:   x,y - pixel coordinates to start displaying text
//              ascii_char - character to be display
//              text_color - color of character
//              backgnd_color - background color
// Return Value: none
///////////////////////////////////////////////////////////////////////
void lcdFont (int x, int y, char ascii_char, int text_color, int backgnd_color)
{
    int c, i, j, x1, y1;
    unsigned char bitmap_byte;

    c = (int)(ascii_char - ' ');

    for (i=0; i < WIDTH_6x8; i++)  {
        bitmap_byte = Terminal6x8e[c][i];

        for (j=0; j < HEIGHT_6x8; j++)  {
            x1 = x+i;
            y1 = y+j;
            lcdMapPixel(&x1, &y1);
            if (bitmap_byte & 0x01)
                lcdSetPixel(x1, y1, text_color);
            else
                lcdSetPixel(x1, y1, backgnd_color);

            bitmap_byte = (bitmap_byte >> 1);
        }
    }

}


///////////////////////////////////////////////////////////////////////
// lcdWrite6x8 - writes a string to the LCD display, restricted to the
//               allowable rows and columns for 6x8 characters
//      Row range = 0 to MAX_ROWS_6x8-1 (0 to 20)
//      Column range = 0 to MAX_COLUMNS_6x8-1 (0 to 15)
// Arguments:   x,y - pixel coordinates to start displaying text
//              ascii_char - character to be display
//              text_color - color of character
//              backgnd_color - background color
// Return Value: none
///////////////////////////////////////////////////////////////////////
void lcdWrite6x8 (int row, int col, char *str, int text_color, int backgnd_color)  {
    int i, x0, y0;

    //Check for and fix out of range conditions
    if (row < 0)
        row = 0;
    else if (row >= MAX_ROWS_6x8)
        row = MAX_ROWS_6x8 - 1;
    else if (col < 0)
        col = 0;
    else if (col >= MAX_COLUMNS_6x8)
        col = MAX_COLUMNS_6x8 - 1;

    x0 = row*WIDTH_6x8;
    y0 = col*HEIGHT_6x8;

    for (i=0; i < strlen(str); i++)  {
        lcdFont (x0+i*WIDTH_6x8, y0, str[i], text_color, backgnd_color);
    }
}

///////////////////////////////////////////////////////////////////////
// lcdMapPixel - maps the origin (0,0) from the lower right hand corner
//      of the display to the upper left hand corner
// Arguments: pointer to (x,y) coordinates to the transformed
// Return Value: variables x and y are modified to new values
///////////////////////////////////////////////////////////////////////
void lcdMapPixel (int *x, int *y)  {
    *x = LCD_MAX_X - (*x);
    *y = LCD_MAX_Y - (*y);
}
