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

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/fcntl.h>
#include <linux/mm.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <mach/gpio.h>
#include <plat/gpio-cfg.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include "tm1638.h"


struct display_digit
{
    uint8_t position;
    uint8_t digit;
    uint8_t dot;
    uint8_t dummy;
};

struct display_led
{
    uint8_t position;
    uint8_t value;
};

struct tm1638_display
{
    struct display_digit dis_dig;
    struct display_led dis_led;
};

struct tm1638_data
{
	int gpio_dio;
	int gpio_sck;
    int gpio_stb;
    struct device *dev;
};

struct tm1638_data *data;

static void tm1638_send_config(const uint8_t enable, const uint8_t brightness);
static void tm1638_write_byte(uint8_t value);
static uint8_t tm1638_read_byte(void);
static void tm1638_send_data(const uint8_t address, const uint8_t data);

static uint8_t _config = TM1638_SET_DISPLAY_ON | TM1638_MAX_BRIGHTNESS;
const uint8_t _digit2segments[10] = {
	0x3F, // 0
	0x06, // 1
	0x5B, // 2
	0x4F, // 3
	0x66, // 4
	0x6D, // 5
	0x7D, // 6
	0x07, // 7
	0x7F, // 8
	0x6F  // 9
};


void tm1638_dio_output(void)
{
    s3c_gpio_cfgpin(data->gpio_dio, S3C_GPIO_OUTPUT);
}

void tm1638_dio_input(void)
{
    s3c_gpio_cfgpin(data->gpio_dio, S3C_GPIO_INPUT);
}

void tm1638_dio_high(void)
{
    gpio_set_value(data->gpio_dio, 1);
    usleep_range(TM1638_USLEEP_MIN, TM1638_USLEEP_MAX);
}

void tm1638_dio_low(void)
{
    gpio_set_value(data->gpio_dio, 0);
    usleep_range(TM1638_USLEEP_MIN, TM1638_USLEEP_MAX);
}

unsigned char tm1638_dio_read(void)
{
    return gpio_get_value(data->gpio_dio);
}

void tm1638_clk_high(void)
{
    gpio_set_value(data->gpio_sck, 1);
    usleep_range(TM1638_USLEEP_MIN, TM1638_USLEEP_MAX);
}

void tm1638_clk_low(void)
{
    gpio_set_value(data->gpio_sck, 0);
    usleep_range(TM1638_USLEEP_MIN, TM1638_USLEEP_MAX);
}

void tm1638_stb_high(void)
{
    gpio_set_value(data->gpio_stb, 1);
    usleep_range(TM1638_USLEEP_MIN, TM1638_USLEEP_MAX);
}

void tm1638_stb_low(void)
{
    gpio_set_value(data->gpio_stb, 0);
    usleep_range(TM1638_USLEEP_MIN, TM1638_USLEEP_MAX);
}

uint8_t tm1638_read_byte(void)
{
	uint8_t i, result = 0;

	tm1638_dio_input();
	tm1638_dio_high();

	for (i = 0; i < 8; i++) {
		tm1638_clk_low();
		if (tm1638_dio_read()) {
			result |= 0x80;
		}
		result >>= 1;
		tm1638_clk_high();
	}

	tm1638_dio_output();
	tm1638_dio_low();

	return result;
}

void tm1638_write_byte(uint8_t value)
{
	uint8_t i;

	for (i = 0; i < 8; ++i, value >>= 1) {
		tm1638_clk_low();
        usleep_range(TM1638_USLEEP_MIN, TM1638_USLEEP_MAX);

		if (value & 0x01)
			tm1638_dio_high();
		else
			tm1638_dio_low();

		tm1638_clk_high();
		usleep_range(TM1638_USLEEP_MIN, TM1638_USLEEP_MAX);
	}
}

void tm1638_send_cmd(const uint8_t value)
{
	tm1638_write_byte(value);
}

void tm1638_data_cmd(void)
{
    tm1638_stb_low();
    tm1638_send_cmd(TM1638_CMD_SET_DATA | TM1638_SET_DATA_F_ADDR);
	tm1638_stb_high();
    usleep_range(TM1638_USLEEP_MIN, TM1638_USLEEP_MAX);
}

void tm1638_display_cmd(const uint8_t config)
{
    tm1638_stb_low();
    tm1638_send_cmd(TM1638_CMD_SET_DSIPLAY | config);
	tm1638_stb_high();
    usleep_range(TM1638_USLEEP_MIN, TM1638_USLEEP_MAX);
}

void tm1638_send_data(const uint8_t address, const uint8_t data)
{
    tm1638_data_cmd();
    tm1638_stb_low();
    tm1638_send_cmd(TM1638_CMD_SET_ADDR | address);
    tm1638_write_byte(data);
    tm1638_stb_high();
    usleep_range(TM1638_USLEEP_MIN, TM1638_USLEEP_MAX);
}

void tm1638_enable(const uint8_t value)
{
	tm1638_send_config(value, _config & TM1638_MAX_BRIGHTNESS);
}

void tm1638_set_brightness(const uint8_t value)
{

	tm1638_send_config(_config & TM1638_SET_DISPLAY_ON,
		value & TM1638_MAX_BRIGHTNESS);
}

void tm1638_display_segments(const uint8_t position, const uint8_t segments)
{
	tm1638_send_data(position, segments);
}

void tm1638_clear_segments(void)
{
	uint8_t i;

	for (i = 0; i < 16; ++i)
		tm1638_display_segments(i, 0x00);
}

void tm1638_display_digit(const uint8_t position, const uint8_t digit,
    const uint8_t dot)
{
    uint8_t segment;

    if (digit < 10)
        segment = _digit2segments[digit];
    else
        segment = 0;

    if (dot)
        segment = segment | 0x80;

    tm1638_display_segments(position, segment);

}

void tm1638_set_led(const uint8_t position, const uint8_t value)
{
	tm1638_send_data((position << 1) - 1, !!value);
}

void tm1638_clear_leds(void)
{
	uint8_t i;

	for (i = 0; i < 8; ++i) {
		tm1638_set_led(i, 0x00);
	}
}

uint8_t tm1638_scan_keys(void)
{
	uint8_t i, keys = 0;

  	tm1638_stb_low();
  	tm1638_write_byte(TM1638_CMD_SET_DATA|TM1638_SET_DATA_READ);
	for (i = 0; i < 4; i++) {
		keys |= tm1638_read_byte() << i;
	}
	tm1638_stb_high();

  	return keys;
}

void tm1638_send_config(const uint8_t enable, const uint8_t brightness)
{

    _config = (enable ? TM1638_SET_DISPLAY_ON : TM1638_SET_DISPLAY_OFF) |
        (brightness > TM1638_MAX_BRIGHTNESS ? TM1638_MAX_BRIGHTNESS : brightness);

    tm1638_data_cmd();
    tm1638_display_cmd(_config);
}

void tm1638_hw_init(const uint8_t enable, const uint8_t brightness)
{
    tm1638_send_config(enable, brightness);
    tm1638_clear_segments();
}

static int tm1638_open(struct inode *inode, struct file *file)
{
    pr_err("%s\r\n", __func__);
	return 0;
}

static int tm1638_release(struct inode *inode, struct file *file)
{
    pr_err("%s\r\n", __func__);
	return 0;
}


static long tm1638_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct tm1638_display _tm1638_display;
    void __user *argp = (void __user *) arg;

    switch (cmd) {
        case IOCTL_DISPLAY_SEG:
            pr_err("%s:IOCTL_DISPLAY_SEG\r\n", __func__);
            if (copy_from_user(&_tm1638_display, argp, sizeof(_tm1638_display)))
                return -EFAULT;

            tm1638_display_digit(_tm1638_display.dis_dig.position,
                _tm1638_display.dis_dig.digit, _tm1638_display.dis_dig.dot);
            break;

        case IOCTL_CLEAR_SEG:
            pr_err("%s:IOCTL_CLEAR_SEG\r\n", __func__);
            tm1638_clear_segments();
            break;

        case IOCTL_SET_LED:
            pr_err("%s:IOCTL_SET_LED\r\n", __func__);
            /* tm1638_set_led(const uint8_t position, const uint8_t value) */
            break;

        case IOCTL_CLEAR_LED:
            pr_err("%s:IOCTL_CLEAR_LED\r\n", __func__);
            break;

        case IOCTL_SCAN_KEY:
            pr_err("%s:IOCTL_SCAN_KEY\r\n", __func__);
            break;

        case IOCTL_SET_CONFIG:
            pr_err("%s:IOCTL_SET_CONFIG\r\n", __func__);
            break;

        default:
            pr_err("%s:nothing\r\n", __func__);
            break;
    }

    return 0;
}

static const struct file_operations tm1638_fops = {
	.owner = THIS_MODULE,
	.open = tm1638_open,
	.release = tm1638_release,
	.unlocked_ioctl = tm1638_ioctl,
};

static struct miscdevice tm1638_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "tm1638_dev",
	.fops = &tm1638_fops,
};

/* 
 * data->gpio_dio -> EXYNOS4_GPB(4) --> XspiCSn1/IEM_DAT --> EVB GPIO5
 * data->gpio_sck -> EXYNOS4_GPB(5) --> XspiCLK1/IEM_CLK --> EVB GPIO6
 * data->gpio_stb -> EXYNOS4_GPB(6) --> XspiMISO1/i2cSDA5 --> EVB GPIO7
 * 
 */
 
static int tm1638_probe(struct platform_device *pdev)
{
    int err;
    int ret;
    int i;

    pr_err("%s\r\n", __func__);

    data = kzalloc(sizeof(*data), GFP_KERNEL);

	err = misc_register(&tm1638_device);
	if (err) {
		pr_err("%s: tm1638_device register failed\n", __func__);
		goto misc_register_failed;
	}

    platform_set_drvdata(pdev, data);
    data->dev = &pdev->dev;

    data->gpio_dio = EXYNOS4_GPB(4);
	ret = gpio_request(data->gpio_dio, "tm1638 dio");
	if (ret) {
		dev_err(&pdev->dev, "gpio dio request failed\n");
		goto err_gpio;
	}
    gpio_direction_output(data->gpio_dio, 1);

    data->gpio_sck = EXYNOS4_GPB(5);
	/* Try requesting the GPIOs */
	ret = gpio_request(data->gpio_sck, "tm1638 sck");
	if (ret) {
		dev_err(&pdev->dev, "gpio sck request failed\n");
		goto err_gpio;
	}
	gpio_direction_output(data->gpio_sck, 1);


    data->gpio_stb = EXYNOS4_GPB(6);
	ret = gpio_request(data->gpio_stb, "tm1638 stb");
	if (ret) {
		dev_err(&pdev->dev, "gpio stb request failed\n");
		goto err_gpio;
	}
    gpio_direction_output(data->gpio_stb, 1);

    tm1638_hw_init(TM1638_SET_DISPLAY_ON, TM1638_MAX_BRIGHTNESS);

    for (i = 0; i < 8; i++)
        tm1638_display_digit(i * 2, 0, 1);

    return 0;

err_gpio:
misc_register_failed:

    kfree(data);
    return 0;
}

static int tm1638_remove(struct platform_device *pdev)
{
    pr_err("%s\r\n", __func__);

    tm1638_clear_segments();
    gpio_free(data->gpio_sck);
    gpio_free(data->gpio_dio);
    gpio_free(data->gpio_stb);
    misc_deregister(&tm1638_device);
    kfree(data);
    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id tm1638_match[] = {
	{ .compatible = "test,tm1638" },
	{},
};
MODULE_DEVICE_TABLE(of, tm1638_match);
#endif

static struct platform_driver tm1638_driver = {
	.driver = {
		.name = "tm1638",
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(tm1638_match),
#endif
	},
	.probe = tm1638_probe,
	.remove = tm1638_remove,
	/* .suspend = tm1638_temp_suspend, */
	/* .resume  = tm1638_temp_resume,  */
};

module_platform_driver(tm1638_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Xlloss <xlloss@gmail.com>");
