/**
 * Copyright (c) 2018, Łukasz Marcin Podkalicki <lpodkalicki@gmail.com>
 *
 * This is ATtiny13/25/45/85 library for TM1638 chip - LED driver controller with key-scan interface.
 *
 * Features:
 * - display digits & dots
 * - display raw segments
 * - display LEDs
 * - display on/off
 * - brightness control
 * - scan keys
 *
 * References:
 * - library: https://github.com/lpodkalicki/attiny-tm1638-library
 * - documentation: https://github.com/lpodkalicki/attiny-tm1638-library/README.md
 * - TM1638 datasheet: https://github.com/lpodkalicki/attiny-tm1638-library/blob/master/docs/tm1638.pdf
 */

#ifndef	_TM1638_H_
#define	_TM1638_H_

#include <linux/ioctl.h>

//#define IOCTL_DISPLAY_SEG 0xFF112200
//#define IOCTL_CLEAR_SEG 0xFF112211
//#define IOCTL_SET_LED 0xFF112222
//#define IOCTL_CLEAR_LED 0xFF112233
//#define IOCTL_SCAN_KEY 0xFF112244
//#define IOCTL_SET_CONFIG 0xFF112255

struct display_digit
{
    unsigned char position;
    unsigned char digit;
    unsigned char dot;
    unsigned char dummy;
};

struct display_led
{
    unsigned char position;
    unsigned char value;
};

#define IOC_MAGIC 's'
#define IOCTL_DISPLAY_SEG _IOW(IOC_MAGIC,0, struct display_digit)
#define IOCTL_CLEAR_SEG _IOW(IOC_MAGIC, 1, struct display_digit)
#define IOCTL_CLEAR_LED _IOW(IOC_MAGIC, 2, struct display_digit)
#define IOCTL_SET_LED _IOW(IOC_MAGIC, 3, struct display_digit)
#define IOCTL_SCAN_KEY _IOW(IOC_MAGIC, 4, struct display_digit)
#define IOCTL_SET_CONFIG _IOW(IOC_MAGIC, 5, struct display_digit)


#ifdef __KERNEL__
// Main Settings
#define	TM1638_DIO_PIN			PB0
#define	TM1638_CLK_PIN			PB1
#define	TM1638_STB_PIN			PB2
#define	TM1638_DELAY_US			2
#define	TM1638_MSLEEP			1
#define	TM1638_USLEEP_MAX			20
#define	TM1638_USLEEP_MIN			10

#define	TM1638_0_BRIGHTNESS 0x00
#define	TM1638_1_BRIGHTNESS 0x01
#define	TM1638_2_BRIGHTNESS 0x02
#define	TM1638_3_BRIGHTNESS 0x03
#define	TM1638_4_BRIGHTNESS 0x03
#define	TM1638_5_BRIGHTNESS 0x05
#define	TM1638_6_BRIGHTNESS 0x06
#define	TM1638_MAX_BRIGHTNESS 0x07


// TM1638 commands
#define	TM1638_CMD_SET_DATA		0x40
#define	TM1638_CMD_SET_ADDR		0xC0
#define	TM1638_CMD_SET_DSIPLAY		0x80

// TM1638 data settings (use bitwise OR to contruct complete command)
#define	TM1638_SET_DATA_WRITE		0x00 // write data to the display register
#define	TM1638_SET_DATA_READ		0x02 // read the key scan data
#define	TM1638_SET_DATA_A_ADDR		0x00 // automatic address increment
#define	TM1638_SET_DATA_F_ADDR		0x04 // fixed address
#define	TM1638_SET_DATA_M_NORM		0x00 // normal mode
#define	TM1638_SET_DATA_M_TEST		0x10 // test mode

// TM1638 display control command set (use bitwise OR to consruct complete command)
#define	TM1638_SET_DISPLAY_OFF		0x00 // off
#define	TM1638_SET_DISPLAY_ON		0x08 // on


/**
 * Initialize TM1638 LED controller.
 * Clock pin (TM1638_CLK_PIN), strobe pin (TM1638_STB_PIN)
 * and data pin (TM1638_DIO_PIN) are defined at the top of this file.
 */
void TM1638_init(const uint8_t enable, const uint8_t brightness);

/**
 * Set display brightness.
 * Min value: 0
 * Max value: 7
 */
void TM1638_set_brightness(const uint8_t value);

/**
 * Turn display on/off.
 * value: 1 - on, 0 - off
 */
void TM1638_enable(const uint8_t value);

/**
 * Display raw segments at position (0x00..0x07)
 *
 *      bits:
 *        -- 0 --
 *       |       |
 *       5       1
 *       |       |
 *        -- 6 --
 *       |       |
 *       4       2
 *       |       |
 *        -- 3 -- *7
 *
 * Example segment configurations:
 * - for character 'H', segments=0b01110110
 * - for character '-', segments=0b01000000
 * - etc.
 */
void TM1638_display_segments(const uint8_t position, const uint8_t segments);

/**
 * Clear all display segments (including dots).
 */
void TM1638_clear_segments(void);

/**
 * Display digit ('0'..'9') with optional dot at position (0x00..0x07)
 */
void TM1638_display_digit(const uint8_t position, const uint8_t digit, const uint8_t dot);

/**
 * Light LED at position (0x00..0x07)
 * value: 1 - on, 0 - off
 */
void TM1638_set_led(const uint8_t position, const uint8_t value);

/**
 * Clear all leds.
 */
void TM1638_clear_leds(void);

/**
 * Scan keys.
 */
uint8_t TM1638_scan_keys(void);

#endif	/* _TM1638_H_ */
#endif
