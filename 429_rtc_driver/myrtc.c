/*
 * Dallas DS1302 RTC Support
 *
 *  Copyright (C) 2002 David McCullough
 *  Copyright (C) 2003 - 2007 Paul Mundt
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License version 2. See the file "COPYING" in the main directory of
 * this archive for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/bcd.h>

#define DEBUG    1

#define DRV_NAME	"zynq_rtc"
#define DRV_VERSION	"1.00.a"

#define	RTC_CMD_READ	0x81		/* Read command */
#define	RTC_CMD_WRITE	0x80		/* Write command */

#define	RTC_CMD_WRITE_ENABLE	0x00		/* Write enable */
#define	RTC_CMD_WRITE_DISABLE	0x80		/* Write disable */

#define RTC_ADDR_RAM0	0x20		/* Address of RAM0 */
#define RTC_ADDR_TCR	0x08		/* Address of trickle charge register */
#define	RTC_ADDR_CTRL	0x07		/* Address of control register */

#define	RTC_ADDR_YEAR	0x06		/* Address of year register */
#define	RTC_ADDR_MON	0x05		/* Address of month register */
#define	RTC_ADDR_DATE	0x04		/* Address of day of month register */

#define	RTC_ADDR_DAY	0x03		/* Address of day of week register */
#define	RTC_ADDR_HOUR	0x02		/* Address of hour register */
#define	RTC_ADDR_MIN	0x01		/* Address of minute register */
#define	RTC_ADDR_SEC	0x00		/* Address of second register */


struct xlnx_rtc_dev {
	struct rtc_device	*rtc;
	void __iomem		*reg_base;
	int			alarm_irq;
	int			sec_irq;
};


static unsigned char ds1302_readbyte(unsigned long addr)
{
	unsigned char val;
	val = *(volatile unsigned char *)(addr);
	return val;
}

static void ds1302_writebyte(unsigned long addr, unsigned int val)
{
	*(volatile unsigned char *)(addr) = (unsigned char)val;
	printk(KERN_DEBUG"%s,%x\n",__func__,val);
}

static int ds1302_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct xlnx_rtc_dev *xrtcdev = dev_get_drvdata(dev);

	tm->tm_sec	= bcd2bin(ds1302_readbyte(xrtcdev->reg_base + RTC_ADDR_SEC));
	tm->tm_min	= bcd2bin(ds1302_readbyte(xrtcdev->reg_base + RTC_ADDR_MIN));
	tm->tm_hour	= (bcd2bin(ds1302_readbyte(xrtcdev->reg_base + RTC_ADDR_HOUR)) + 16) % 24;//中国处于UTC+8时区
	//tm->tm_hour	= bcd2bin(ds1302_readbyte(xrtcdev->reg_base + RTC_ADDR_HOUR));
	tm->tm_wday	= bcd2bin(ds1302_readbyte(xrtcdev->reg_base + RTC_ADDR_DAY)) ;
	tm->tm_mday	= bcd2bin(ds1302_readbyte(xrtcdev->reg_base + RTC_ADDR_DATE));
	tm->tm_mon	= bcd2bin(ds1302_readbyte(xrtcdev->reg_base + RTC_ADDR_MON)) - 1;
	tm->tm_year	= bcd2bin(ds1302_readbyte(xrtcdev->reg_base + RTC_ADDR_YEAR));

	//printk(KERN_DEBUG"(2)%s,%ld\n",__func__,xrtcdev->reg_base);

	if (tm->tm_year < 70)
		tm->tm_year += 100;

	printk(KERN_DEBUG"%s: tm is secs=%d, mins=%d, hours=%d, "
		"mday=%d, mon=%d, year=%d, wday=%d\n",
		__func__,
		tm->tm_sec, tm->tm_min, tm->tm_hour,
		tm->tm_mday, tm->tm_mon + 1, tm->tm_year, tm->tm_wday);

	return rtc_valid_tm(tm);
}

static int ds1302_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct xlnx_rtc_dev *xrtcdev = dev_get_drvdata(dev);
/*
	printk(KERN_DEBUG"(3)%s,%ld\n",__func__,xrtcdev->reg_base);
	printk(KERN_DEBUG"%s: tm is secs=%d, mins=%d, hours=%d, "
		"mday=%d, mon=%d, year=%d, wday=%d\n",
		__func__,
		tm->tm_sec, tm->tm_min, tm->tm_hour,
		tm->tm_mday, tm->tm_mon + 1, tm->tm_year % 100, tm->tm_wday);
*/
	ds1302_writebyte(xrtcdev->reg_base + 8 + RTC_ADDR_CTRL, RTC_CMD_WRITE_ENABLE);

	ds1302_writebyte(xrtcdev->reg_base + 8 + RTC_ADDR_SEC, bin2bcd(tm->tm_sec));
	ds1302_writebyte(xrtcdev->reg_base + 8 + RTC_ADDR_MIN, bin2bcd(tm->tm_min));
	ds1302_writebyte(xrtcdev->reg_base + 8 + RTC_ADDR_HOUR, bin2bcd((tm->tm_hour + 8) % 24));//中国处于UTC+8时区
	ds1302_writebyte(xrtcdev->reg_base + 8 + RTC_ADDR_DAY, bin2bcd(tm->tm_wday));
	ds1302_writebyte(xrtcdev->reg_base + 8 + RTC_ADDR_DATE, bin2bcd(tm->tm_mday));
	ds1302_writebyte(xrtcdev->reg_base + 8 + RTC_ADDR_MON, bin2bcd(tm->tm_mon + 1));
	ds1302_writebyte(xrtcdev->reg_base + 8 + RTC_ADDR_YEAR, bin2bcd(tm->tm_year % 100));

	ds1302_writebyte(xrtcdev->reg_base + 8 + RTC_ADDR_CTRL, RTC_CMD_WRITE_DISABLE);
	mdelay(100);
	ds1302_writebyte(xrtcdev->reg_base + 8 + RTC_ADDR_CTRL, RTC_CMD_WRITE_ENABLE);
/*	
	*(volatile unsigned long *)(xrtcdev->reg_base + 12) = 0x00700101;
	*(volatile unsigned long *)(xrtcdev->reg_base + 8) = 0x000A2324;
	*(volatile unsigned long *)(xrtcdev->reg_base + 12) = 0x80700101;
	mdelay(100);
	*(volatile unsigned long *)(xrtcdev->reg_base + 12) = 0x00700101;
*/
	/* Start RTC */
	//ds1302_writebyte(xrtcdev->reg_base + RTC_ADDR_SEC, ds1302_readbyte(xrtcdev->reg_base + RTC_ADDR_SEC) & ~0x80);

	//ds1302_writebyte(xrtcdev->reg_base + RTC_ADDR_CTRL, RTC_CMD_WRITE_DISABLE);	
	

	return 0;
}

static int ds1302_rtc_ioctl(struct device *dev, unsigned int cmd,
			    unsigned long arg)
{
	//int err;
	//struct xlnx_rtc_dev *xrtcdev = dev_get_drvdata(dev);
	switch (cmd) {

		//case RTC_RD_TIME:
		//{
		//	err = ds1302_rtc_read_time(xrtcdev->rtc, &tm)
		
		//}
#ifdef RTC_SET_CHARGE
		case RTC_SET_CHARGE:
		{
			int tcs_val;

			if (copy_from_user(&tcs_val, (int __user *)arg, sizeof(int)))
				return -EFAULT;

			ds1302_writebyte(RTC_ADDR_TCR, (0xa0 | tcs_val * 0xf));
			return 0;
		}
#endif

	}

	return -ENOIOCTLCMD;
}

static struct rtc_class_ops ds1302_rtc_ops = {
	.read_time	= ds1302_rtc_read_time,
	.set_time	= ds1302_rtc_set_time,
	.ioctl		= ds1302_rtc_ioctl,
};

static int xlnx_rtc_probe(struct platform_device *pdev)
{
	struct xlnx_rtc_dev *xrtcdev;
	struct resource *res;
	unsigned long remap_size;   /* Device Memory Size */
	int ret;

	//printk(KERN_DEBUG"rtc-ds1302.c test\n");	
	xrtcdev = devm_kzalloc(&pdev->dev, sizeof(*xrtcdev), GFP_KERNEL);
	if (!xrtcdev)
		return -ENOMEM;

	platform_set_drvdata(pdev, xrtcdev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
        	dev_err(&pdev->dev, "No memory resource\n");
        	return -ENODEV;
     	}
	
	xrtcdev->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(xrtcdev->reg_base))
		return PTR_ERR(xrtcdev->reg_base);

	//printk(KERN_DEBUG"(1)%s,%ld\n",__func__,xrtcdev->reg_base);

	*(volatile unsigned long *)(xrtcdev->reg_base + 12) = 0x00160A0A;
	*(volatile unsigned long *)(xrtcdev->reg_base + 8) = 0x00050000;
	*(volatile unsigned long *)(xrtcdev->reg_base + 12) = 0x80160A0A;
	mdelay(100);
	*(volatile unsigned long *)(xrtcdev->reg_base + 12) = 0x00160A0A;

/*
	remap_size = res->end - res->start + 1;
     	if (!request_mem_region(res->start, remap_size, pdev->name)) {
        	dev_err(&pdev->dev, "Cannot request IO\n");
        	return -ENXIO;
     	}
	
	xrtcdev->reg_base = ioremap(res->start, remap_size);
     	if (xrtcdev->reg_base == NULL) {
        	dev_err(&pdev->dev, "Couldn't ioremap memory at 0x%08lx\n",(unsigned long)res->start);
       		ret = -ENOMEM;
     	}

	
	
	printk(KERN_DEBUG"(2)%s,%ld\n",__func__,xrtcdev->reg_base);


	

	rtc_reg1 = *(volatile unsigned int *)(xrtcdev->reg_base + 4);
	rtc_reg0 = *(volatile unsigned int *)(xrtcdev->reg_base + 0);

		time_temp[5] = (unsigned char)(rtc_reg1 >> 16);        //RTC year
		time_temp[4] = (unsigned char)(rtc_reg1 >> 8);         //RTC month
		time_temp[3] = (unsigned char)(rtc_reg1);              //RTC date


		time_temp[2] = (unsigned char)(rtc_reg0 >> 16);        //RTC Hour
		time_temp[1] = (unsigned char)(rtc_reg0 >> 8);         //RTC Munite
		time_temp[0] = (unsigned char)(rtc_reg0);              //RTC Second

		for (i=0;i<6;i++){

			time_v[i] = time_temp[i]/16*10+time_temp[i]%16;//格式为: 秒 分 时 日 月 星期 年
		}

		printk("current time is:20%d-%d-%d %d:%d:%d\n",time_v[5],time_v[4],time_v[3],time_v[2],time_v[1],time_v[0]);  //convert string

	time_sw[6] = *(volatile unsigned char *)(xrtcdev->reg_base + 6);
	time_sw[5] = *(volatile unsigned char *)(xrtcdev->reg_base + 5);
	time_sw[4] = *(volatile unsigned char *)(xrtcdev->reg_base + 4);
	time_sw[3] = *(volatile unsigned char *)(xrtcdev->reg_base + 3);
	time_sw[2] = *(volatile unsigned char *)(xrtcdev->reg_base + 2);
	time_sw[1] = *(volatile unsigned char *)(xrtcdev->reg_base + 1);
	time_sw[0] = *(volatile unsigned char *)(xrtcdev->reg_base + 0);


		printk("current time is:20%d-%d-%d %d %d:%d:%d\n",time_sw[6],time_sw[5],time_sw[4],time_sw[3],time_sw[2],time_sw[1],time_sw[0]);
*/		

	xrtcdev->rtc = devm_rtc_device_register(&pdev->dev, "ds1302", &ds1302_rtc_ops, THIS_MODULE);
	if (IS_ERR(xrtcdev->rtc))
		return PTR_ERR(xrtcdev->rtc);


	printk(KERN_DEBUG"rtc-ds1302.c test!!\n");
	



	return 0;
}

 static int xlnx_rtc_remove(struct platform_device *pdev)
 {

     return 0;
 }

static int __maybe_unused xlnx_rtc_suspend(struct device *dev)
{
	

	return 0;
}

static int __maybe_unused xlnx_rtc_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(xlnx_rtc_pm_ops, xlnx_rtc_suspend, xlnx_rtc_resume);

static const struct of_device_id xlnx_rtc_of_match[] = {
	{.compatible = "xlnx,rtcip-1.00.a" },
	{ }
};

MODULE_DEVICE_TABLE(of, xlnx_rtc_of_match);

static struct platform_driver xlnx_rtc_driver = {
	.probe		= xlnx_rtc_probe,
	.remove		= xlnx_rtc_remove,
	.driver		= {
		.name	= DRV_NAME,
		.pm	= &xlnx_rtc_pm_ops,
		.of_match_table	= xlnx_rtc_of_match,
	},
};

module_platform_driver(xlnx_rtc_driver);

MODULE_DESCRIPTION("ZYNQ RTC driver");
MODULE_VERSION(DRV_VERSION);
MODULE_AUTHOR("ZZQ");
MODULE_LICENSE("GPL v2");
