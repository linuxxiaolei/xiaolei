#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <asm/irq.h>
#include <mach/regs-gpio.h> //定义s3c2410的GPIO
#include <mach/hardware.h> //定义操作s3c2410的GPIO的函数 
#include <linux/device.h> //自动创建设备文件应该包含的头文件
#include <mach/gpio.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>  
#include <linux/poll.h>  
#include <linux/irq.h>  
#include <linux/wait.h>
#include <linux/semaphore.h>
#include <linux/signal.h>

MODULE_LICENSE("GPL");

struct spi_gpio_rtd_irq 
{
    unsigned    gpio;
    char        type;
    char        *pDesc;
};

struct spi_gpio_rtd_platform_data
{
    unsigned                    sck;
    unsigned                    mosi;
    unsigned                    miso;
    unsigned                    cs;
    char                        flag;
    struct spi_gpio_rtd_irq     irq[2];
   
};

static struct spi_gpio_rtd_platform_data spi_cross_rtd_data1 = {
    .sck  =  S3C2410_GPG(9),
    .mosi =  S3C2410_GPG(10),
    .miso =  S3C2410_GPG(11),
    .cs   =  S3C2410_GPG(12),
    .flag =  75,
    .irq  = {
       [0]= {
          .gpio = S3C2410_GPG(7),
          .type = 0,
          .pDesc= "dydr1"
       }, 

       [1]= {
          .gpio = S3C2410_GPG(8),
          .type = 1,
          .pDesc= "dydr2"
       }, 
    },
};

static struct spi_gpio_rtd_platform_data spi_cross_rtd_data2 = {
    .sck  =  S3C2410_GPE(11),
    .mosi =  S3C2410_GPE(12),
    .miso =  S3C2410_GPE(13),
    .cs   =  S3C2410_GPL(13),
    .flag =  76,
    .irq  = {
       [0]= {
          .gpio = S3C2410_GPF(4),
          .type = 0,
          .pDesc= "dydr1"
       }, 

       [1]= {
          .gpio = S3C2410_GPF(5),
          .type = 1,
          .pDesc= "dydr2"
       }, 
    },
};



static struct resource spi_gpi_rtd_resource[] = {
    [0] = {
        .start = IRQ_EINT15,
        .end   = IRQ_EINT15,
        .flags = IORESOURCE_IRQ,
    },
    [1] = {
        .start = IRQ_EINT16,
        .end   = IRQ_EINT16,
        .flags = IORESOURCE_IRQ,
    },
};
static struct resource spi_gpi_rtd_resource2[] = {
    [0] = {
        .start = IRQ_EINT4,
        .end   = IRQ_EINT4,
        .flags = IORESOURCE_IRQ,
    },
    [1] = {
        .start = IRQ_EINT5,
        .end   = IRQ_EINT5,
        .flags = IORESOURCE_IRQ,
    },
};



static struct   platform_device spi_gpio_rtd_devices1 = {
    .name   =   "spi_gpio_rtd1",
    .id         =   0,
    .dev      =
    {
                .platform_data = &spi_cross_rtd_data1,
    },
};

static struct   platform_device spi_gpio_rtd_devices2 = {
    .name   =   "spi_gpio_rtd2",
    .id         =   0,
    .dev      =
    {
                .platform_data = &spi_cross_rtd_data2,
    },
};


static int __init spi_rtd_dev_init()
{
    int ret;
    platform_device_add_resources(&spi_gpio_rtd_devices1,&spi_gpi_rtd_resource,2);
    
    platform_device_add_resources(&spi_gpio_rtd_devices2,&spi_gpi_rtd_resource2,2);
    
    ret = platform_device_register(&spi_gpio_rtd_devices1);
    if(ret < 0)
    {
        printk("====== %s platform device register1 failed \n", __func__);
        
    }
    ret = platform_device_register(&spi_gpio_rtd_devices2);

    if(ret < 0)
    {
        printk("====== %s platform device register2 failed \n", __func__);
        
    }  
    printk("======%s,platform_device_register22 sucess... \n", __func__);
    return ret ;
}

static void __exit spi_rtd_dev_exit()
{

    platform_device_unregister(&spi_gpio_rtd_devices1);
}

module_init(spi_rtd_dev_init);
module_exit(spi_rtd_dev_exit);





