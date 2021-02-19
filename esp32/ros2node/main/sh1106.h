#ifndef MAIN_SH1106_H_
#define MAIN_SH1106_H_

// Following definitions are bollowed from 
// https://www.elecrow.com/download/SH1106%20datasheet.pdf

// SLA (0x3C) + WRITE_MODE (0x00) =  0x78 (0b01111000)
#define OLED_I2C_ADDRESS   0x3C

// Control byte
#define OLED_CONTROL_BYTE_CMD_SINGLE    0x80
#define OLED_CONTROL_BYTE_CMD_STREAM    0x00
#define OLED_CONTROL_BYTE_DATA_STREAM   0x40

// Fundamental commands (pg.29)

// NOP
#define OLED_CMD_NOP                    0xE3

// follow with 0x00 to 0xFF to set contrast value to 0-255.
#define OLED_CMD_SET_CONTRAST           0x81 

// I don't know what the difference between this and
// OLED_CMD_DISPLAY_ON. This one is described by the manual as
// "Selects normal display or Entire Display ON", and the
// OLED_CMD_DISPLAY_ON is described as "Turns on OLED panel or turns
// off". I have so many questions.
#define OLED_CMD_ENTIRE_DISPLAY_ON      0xA5
#define OLED_CMD_ENTIRE_DISPLAY_OFF     0xA4

// Described as "Normal indication when low, but reverse indication
// when high". I don't know.
#define OLED_CMD_DISPLAY_INVERT_NORMAL   0xA6
#define OLED_CMD_DISPLAY_INVERT_INVERTED 0xA7

// "Turns on OLED panel or turns off", according to manual.
#define OLED_CMD_DISPLAY_OFF             0xAE
#define OLED_CMD_DISPLAY_ON              0xAF




// *********************
// *                   *
// *  Hardware Config  *
// *                   *
// *********************

// Follow with 0x40-0x7F for start line 0 to 63.
#define OLED_CMD_SET_DISPLAY_START_LINE 0x40

// Something something maps ram to display differently - in
// practice, flips the bitmapping vertically - inverting this
// will write right-to-left instead of left-to-right.
#define OLED_CMD_SET_SEGMENT_REMAP_INVERSE 0xA1
#define OLED_CMD_SET_SEGMENT_REMAP_NORMAL 0xA0

// Manual: "Scan from COM0 to COM[n-1] or scan from COM[n-1] to COM0."
#define OLED_CMD_SET_COM_SCAN_MODE_REVERSE 0xC8    
#define OLED_CMD_SET_COM_SCAN_MODE_NORMAL  0xC0


// Shifts the "first line" upwards by n lines. Follow this command
// with 0x00-0x3f for an upwards shift by 0 to 63 lines.
#define OLED_CMD_SET_DISPLAY_OFFSET     0xD3


// Timing and Driving Scheme
#define OLED_CMD_SET_DISPLAY_CLK_DIV    0xD5    // follow with 0x80
#define OLED_CMD_SET_PRECHARGE          0xD9    // follow with 0xF1
#define OLED_CMD_SET_VCOMH_DESELCT      0xDB    // follow with 0x30

// DC-DC Control. Follow this with 0x8A for off and 0x8B for on.
#define OLED_CMD_SET_CHARGE_PUMP_CTRL        0xAD
#define OLED_CMD_SET_CHARGE_PUMP_ON 0x0B
#define OLED_CMD_SET_CHARGE_PUMP_OFF 0x0A


#endif /* MAIN_SH1106_H_ */
