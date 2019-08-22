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
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux//uaccess.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include "tm1638.h"

#define PIN_NUM 3
#define PIN_DIO_DTS_NAME "gpio_dio"
#define PIN_SCK_DTS_NAME "gpio_sck"
#define PIN_STB_DTS_NAME "gpio_stb"

#define GPIO_OUT_HI 1
#define GPIO_OUT_LO 0

#define TM1638_SEG_0 0x3F
#define TM1638_SEG_1 0x06
#define TM1638_SEG_2 0x5B
#define TM1638_SEG_3 0x4F
#define TM1638_SEG_4 0x66
#define TM1638_SEG_5 0x6D
#define TM1638_SEG_6 0x7D
#define TM1638_SEG_7 0x07
#define TM1638_SEG_8 0x7F
#define TM1638_SEG_9 0x6F
#define TM1638_SEG_DOT 0x80

#define TM1638_LED_NUM 8
#define TM1638_LED_CLR 0

#define TM1638_SEG_NUM 10

#define TM1638_MISCDEV_NAME "tm1638_dev"

struct tm1638_display
{
    struct display_digit dis_dig;
    struct display_led dis_led;
};

struct tm1638_data
{
    struct gpio_desc *gpio_dio;
    struct gpio_desc *gpio_sck;
    struct gpio_desc *gpio_stb;
    struct device *dev;
};

struct tm1638_data *data;

static uint8_t _config = TM1638_SET_DISPLAY_ON | TM1638_MAX_BRIGHTNESS;

const uint8_t _digit2segments[TM1638_SEG_NUM] = {
	TM1638_SEG_0,
	TM1638_SEG_1,
	TM1638_SEG_2,
	TM1638_SEG_3,
	TM1638_SEG_4,
	TM1638_SEG_5,
	TM1638_SEG_6,
	TM1638_SEG_7,
	TM1638_SEG_8,
	TM1638_SEG_9 
};

void tm1638_dio_output(void)
{
	int ret;

	ret = gpiod_direction_output(data->gpio_dio, GPIO_OUT_HI);
	if (ret)
		pr_err("tm1638_dio_output change fail\r\n");
}

void tm1638_dio_input(void)
{
	int ret;

	ret = gpiod_direction_input(data->gpio_dio);
	if (ret)
		pr_err("tm1638_dio_input change fail\r\n");
}

void tm1638_dio_high(void)
{
    gpiod_set_value(data->gpio_dio, GPIO_OUT_HI);
    usleep_range(TM1638_USLEEP_MIN, TM1638_USLEEP_MAX);
}

void tm1638_dio_low(void)
{
    gpiod_set_value(data->gpio_dio, GPIO_OUT_LO);
    usleep_range(TM1638_USLEEP_MIN, TM1638_USLEEP_MAX);
}

unsigned char tm1638_dio_read(void)
{
    return gpiod_get_value(data->gpio_dio);
}

void tm1638_clk_high(void)
{
    gpiod_set_value(data->gpio_dio, GPIO_OUT_HI);
    usleep_range(TM1638_USLEEP_MIN, TM1638_USLEEP_MAX);
}

void tm1638_clk_low(void)
{
    gpiod_set_value(data->gpio_sck, GPIO_OUT_LO);
    usleep_range(TM1638_USLEEP_MIN, TM1638_USLEEP_MAX);
}

void tm1638_stb_high(void)
{
    gpiod_set_value(data->gpio_sck, GPIO_OUT_HI);
    usleep_range(TM1638_USLEEP_MIN, TM1638_USLEEP_MAX);
}

void tm1638_stb_low(void)
{
    gpiod_set_value(data->gpio_stb, GPIO_OUT_LO);
    usleep_range(TM1638_USLEEP_MIN, TM1638_USLEEP_MAX);
}

uint8_t tm1638_read_byte(void)
{
	uint8_t i, result = 0;

    tm1638_dio_high();
	tm1638_dio_input();
    usleep_range(TM1638_USLEEP_MIN, TM1638_USLEEP_MAX);

	for (i = 0; i < 8; i++) {
		tm1638_clk_low();
        usleep_range(TM1638_USLEEP_MIN, TM1638_USLEEP_MAX);
		if (tm1638_dio_read())
			result |= 0x80;

        if (i != 0)
            result = result >> 1;

		tm1638_clk_high();
        usleep_range(TM1638_USLEEP_MIN, TM1638_USLEEP_MAX);
	}


    usleep_range(TM1638_USLEEP_MIN, TM1638_USLEEP_MAX);
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

void tm1638_send_config(const uint8_t enable, const uint8_t brightness)
{

    _config = (enable ? TM1638_SET_DISPLAY_ON : TM1638_SET_DISPLAY_OFF) |
        (brightness > TM1638_MAX_BRIGHTNESS ? TM1638_MAX_BRIGHTNESS : brightness);

    tm1638_data_cmd();
    tm1638_display_cmd(_config);
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

    if (digit < TM1638_SEG_NUM)
        segment = _digit2segments[digit];
    else
        segment = 0;

    if (dot)
        segment = segment | TM1638_SEG_DOT;

    tm1638_display_segments(position, segment);

}

void tm1638_set_led(const uint8_t position, const uint8_t value)
{
	tm1638_send_data((position << 1) - 1, !!value);
}

void tm1638_clear_leds(void)
{
	uint8_t i;

	for (i = 0; i < TM1638_LED_NUM; ++i)
		tm1638_set_led(i, TM1638_LED_CLR);

}

uint8_t tm1638_scan_keys(void)
{
	uint8_t i;
	uint8_t keys = 0;
//    uint8_t mykey[8] = {0};

    tm1638_stb_low();
    usleep_range(TM1638_USLEEP_MIN, TM1638_USLEEP_MAX);
    tm1638_write_byte(TM1638_CMD_SET_DATA | TM1638_SET_DATA_READ);
    usleep_range(TM1638_USLEEP_MIN, TM1638_USLEEP_MAX);
    for (i = 0; i < 4; i++) {
        usleep_range(TM1638_USLEEP_MIN, TM1638_USLEEP_MAX);
//		keys |= tm1638_read_byte() << i;
        pr_err("key %d : read 0x%x\r\n", i, tm1638_read_byte());
//        mykey[keys] = 1;
	}
	tm1638_stb_high();

  	return keys;
}

void tm1638_hw_init(const uint8_t enable, const uint8_t brightness)
{
    tm1638_send_config(enable, brightness);
    tm1638_clear_segments();
}

static int tm1638_open(struct inode *inode, struct file *file)
{
    pr_info("%s\r\n", __func__);
	return 0;
}

static int tm1638_release(struct inode *inode, struct file *file)
{
    pr_info("%s\r\n", __func__);
	return 0;
}

static long tm1638_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct tm1638_display _tm1638_dis;
    struct display_led __tm1638_led;
    void __user *argp = (void __user *) arg;


    switch (cmd) {
        case IOCTL_DISPLAY_SEG:
            pr_info("%s:IOCTL_DISPLAY_SEG\r\n", __func__);

            if (copy_from_user(&_tm1638_dis, argp, sizeof(_tm1638_dis)))
                return -EFAULT;

            tm1638_display_digit(_tm1638_dis.dis_dig.position * 2,
                _tm1638_dis.dis_dig.digit,
                _tm1638_dis.dis_dig.dot);
            break;

        case IOCTL_CLEAR_SEG:
            pr_info("%s:IOCTL_CLEAR_SEG\r\n", __func__);
            tm1638_clear_segments();
            break;

        case IOCTL_SET_LED:
            pr_info("%s:IOCTL_SET_LED\r\n", __func__);
            if (copy_from_user(&__tm1638_led, argp, sizeof(__tm1638_led)))
                return -EFAULT;

            tm1638_set_led(__tm1638_led.position, __tm1638_led.value);
            break;

        case IOCTL_CLEAR_LED:
            tm1638_clear_leds();
            pr_info("%s:IOCTL_CLEAR_LED\r\n", __func__);
            break;

        case IOCTL_SCAN_KEY:
            pr_info("%s:IOCTL_SCAN_KEY\r\n", __func__);
            break;

        case IOCTL_SET_CONFIG:
            pr_info("%s:IOCTL_SET_CONFIG\r\n", __func__);
            break;

        default:
            pr_info("%s:nothing\r\n", __func__);
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
	.name = TM1638_MISCDEV_NAME,
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
    int i;
	char *pin_name[PIN_NUM] = {PIN_DIO_DTS_NAME,
            PIN_SCK_DTS_NAME, PIN_STB_DTS_NAME};

    pr_info("%s\r\n", __func__);

    //data = kzalloc(sizeof(*data), GFP_KERNEL);

	err = misc_register(&tm1638_device);
	if (err) {
		pr_err("%s: tm1638_device register failed\n", __func__);
		goto misc_register_failed;
	}

    //platform_set_drvdata(pdev, data);
    //data->dev = &pdev->dev;

    /* Try requesting the GPIOs */
	for (i = 0; i < PIN_NUM; i++) {
		switch (i) {
			case 0:
				data->gpio_dio = devm_gpiod_get_optional(&pdev->dev,
					pin_name[i], GPIOD_OUT_HIGH);
				if (IS_ERR(data->gpio_dio))
					goto gpio_init_error;
				break;

			case 1:
				data->gpio_sck = devm_gpiod_get_optional(&pdev->dev,
					pin_name[i], GPIOD_OUT_HIGH);
				if (IS_ERR(data->gpio_sck))
					goto gpio_init_error;
				break;

			case 2:
				data->gpio_stb = devm_gpiod_get_optional(&pdev->dev,
					pin_name[i], GPIOD_OUT_HIGH);
				if (IS_ERR(data->gpio_stb))
					goto gpio_init_error;
				break;
		}
	}

    tm1638_hw_init(TM1638_SET_DISPLAY_ON, TM1638_MAX_BRIGHTNESS);

    for (i = 0; i < 8; i++) {
        tm1638_display_digit(i * 2, 0, 1);
        tm1638_set_led(i, 1);
    }
    /* pr_err("get key %d\r\n", tm1638_scan_keys()); */

    return 0;

gpio_init_error:
	dev_err(&pdev->dev, "%s gpio init failed index %d\r\n", __func__, i);

misc_register_failed:
    dev_err(&pdev->dev, "%s misc_register_failed\r\n", __func__);
//    kfree(data);
    return 0;
}

static int tm1638_remove(struct platform_device *pdev)
{
    pr_err("%s\r\n", __func__);

    tm1638_clear_segments();
    misc_deregister(&tm1638_device);
//    kfree(data);
    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id tm1638_match[] = {
	{ .compatible = "test, tm1638" },
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
MODULE_AUTHOR("Slash <slash.linux.c@gmail.com>");
