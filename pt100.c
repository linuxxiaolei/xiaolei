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

#define RTD_IOC_MAGIC           'r'
#define RTD_IOCSQSETCONF        _IOW(RTD_IOC_MAGIC,1,char)
#define RTD_IOCGQSETCONF        _IOR(RTD_IOC_MAGIC,2,char)
#define RTD_IOCSQWRITEBYTE      _IOW(RTD_IOC_MAGIC,3,char)  
#define RTD_IOCTQAUNTUM         _IO(RTD_IOC_MAGIC,4)
#define RTD_IOCXQREAD           _IOWR(RTD_IOC_MAGIC,5,int)
#define RTD_IOCFULLREAD         _IO(RTD_IOC_MAGIC,6)
#define RTD_IOCGFULLREAD        _IOR(RTD_IOC_MAGIC,7,float)
#define RTD_IOCXREADTEST        _IOWR(RTD_IOC_MAGIC,8,int)

#define RTD_MAXNR               8

#define SPI_RTD_MAJOR           156
#define SPI_RTD_MINORS          32


#define SPI_RTD_MAJOR2           156+2
#define SPI_RTD_MINORS2          32+2


#define CMD_RTD_WRITE                           0x00
#define CMD_RTD_READ                            0xFF
#define CMD_RTD_CONFIG                          0x80
#define CMD_RTD_READ_CONFIG                     0x00
#define CMD_WRITE_HIGH_FAULTTHRESHOLD_MSB       0x83
#define CMD_WRITE_HIGH_FAULTTHRESHOLD_LSB       0x84
#define CMD_READ_HIGH_FAULTTHRESHOLD_MSB        0x03
#define CMD_READ_HIGH_FAULTTHRESHOLD_LSB        0x04
#define CMD_WRITE_LOW_FAULTTHRESHOLD_MSB        0x85
#define CMD_WRITE_LOW_FAULTTHRESHOLD_LSB        0x86
#define CMD_READ_LOW_FAULTTHRESHOLD_MSB         0x05
#define CMD_READ_LOW_FAULTTHRESHOLD_LSB         0x06
#define CMD_FAULT_STATUS                        0x07
#define CMD_ENABLE_VBIAS                        0xD0

#define CONF_VBIAS_ON                           0x80 //1000
#define CONF_CONVERSION_MODE_AUTO               0x40 //0100
#define CONF_SHOT_OFF                           0x20
#define CONF_3WRIE_SELECT_ON                    0x10
#define CONF_FAULT_DELECTION_CYCLE_ON           0x0C
#define CONF_FAULT_STATUS_CLEAR_ON              0x02
#define CONF_50HZFILTER_SELECT_ON               0x01


#define SPI_RTD_MISO_GPIO   ((pdata)->miso)
#define SPI_RTD_MOSI_GPIO   ((pdata)->mosi)
#define SPI_RTD_SCK_GPIO    ((pdata)->sck)
#define SPI_RTD_CS_GPIO     ((pdata)->cs)
/*
static float a = 0.00390830;
static float b = -0.0000005775;
static float c = -0.00000000000418301;
*/

//#define MAX_31865_1        //屏蔽编译驱动2



static DECLARE_BITMAP(minors, SPI_RTD_MINORS);
//static DECLARE_BITMAP(minors, SPI_RTD_MINORS2);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);



struct class     *spi_rtd_class;
struct class     *spi_rtd_class2;


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

struct spi_rtd_data
{
    float       reference_res;  //参考电阻
    float       resistance_rtd;//rtd电阻值
    double      temperature ;
    u8          fault_error;//故障登记
    u8          rtd_config;//配置
};



struct spi_gpio_rtd_data
{
    struct list_head                               device_entry;
    struct  platform_device                        *pdev;
    struct  spi_rtd_data                           rtd_phy;   
    struct  spi_gpio_rtd_platform_data             *pdata;
    struct  semaphore                              sem;
    struct  fasync_struct                          *pFasync;
    dev_t                                          devt;
    u8                                             drdy;
    u8                                             reg;
    u16                                            irq;
    wait_queue_head_t                              waitq;
};

struct spi_gpio_rtd_data                *g_rtd_dev;
struct spi_gpio_rtd_data                *g_rtd_dev2;

void spi_delay(int num)
{
    ndelay(500*num);
}

static inline void spi_set_rtd_mosi(struct spi_gpio_rtd_platform_data *pdata,int is_on)
{
    gpio_set_value(pdata->mosi,is_on);
}

static inline int  spi_get_rtd_miso(struct spi_gpio_rtd_platform_data *pdata)
{
    return gpio_get_value(pdata->miso);
}

static inline void spi_set_rtd_clk(struct spi_gpio_rtd_platform_data *pdata,int is_on)
{
    gpio_set_value(pdata->sck,is_on);
}

static inline void spi_set_rtd_cs(struct spi_gpio_rtd_platform_data *pdata,int is_on)
{
    gpio_set_value(pdata->cs,is_on);
}

static void spi_send_start(struct spi_gpio_rtd_platform_data *pdata,unsigned int mode )
{
    switch(mode)
    {
        case SPI_MODE_0:
        break;
        case SPI_MODE_1:
        break;
        case SPI_MODE_2:
        spi_set_rtd_cs(pdata,0);
        spi_delay(1);
        spi_set_rtd_clk(pdata,0);
        break;
        case SPI_MODE_3:
        break;
    }
}

static void spi_send_end(struct spi_gpio_rtd_platform_data *pdata)
{
    spi_set_rtd_clk(pdata,1);
    spi_set_rtd_cs(pdata,1);
}

static void spi_send_cmd(struct spi_gpio_rtd_platform_data *pdata,unsigned char cmd)
{
    unsigned char i =0 ;
    for(i=0;i<2;i++)
    {
        if(cmd & 0x02)
            spi_set_rtd_mosi(pdata,1);
        else
            spi_set_rtd_mosi(pdata,0);
        spi_delay(1);  
        spi_set_rtd_clk(pdata,0);
        spi_delay(1);  
        cmd = (cmd << 1);
    }
    spi_set_rtd_clk(pdata,1);
}

static void spi_send_byte(struct spi_gpio_rtd_platform_data *pdata,unsigned char data)
{
    int i = 0;
    for(i=0;i<8;i++)
    {
        spi_set_rtd_clk(pdata,0);
        if(data & 0x80)
            spi_set_rtd_mosi(pdata,1);
        else
            spi_set_rtd_mosi(pdata,0);
        spi_delay(1);  
        spi_set_rtd_clk(pdata,1);
        spi_delay(1);  
        data = (data << 1 );
    }
}

static unsigned char spi_revc_byte(struct spi_gpio_rtd_platform_data *pdata)
{
    int i,tmp;
    unsigned char in =0 ;
    for(i=0;i<8;i++)
    {
        in = (in << 1);
        spi_set_rtd_clk(pdata,0);
        spi_delay(1);  
        tmp = spi_get_rtd_miso(pdata);
        if(tmp == 1)
            in = in|0x01;
        spi_set_rtd_clk(pdata,1);     
        spi_delay(1);
    }

    return in;
}

static void spi_write_rtd(struct spi_gpio_rtd_platform_data *pdata,unsigned char reg, unsigned char data)
{
    unsigned char out;
    out = data;
    spi_send_start(pdata, SPI_MODE_2);
    //spi_send_cmd(pdata,CMD_RTD_WRITE);
    spi_send_byte(pdata,reg);
    spi_send_byte(pdata,data);
    spi_send_end(pdata);
}

static void spi_write_burst_rtd(struct spi_gpio_rtd_platform_data *pdata,unsigned char reg,unsigned char *data, unsigned int len)
{
    int i;
    spi_send_start(pdata, SPI_MODE_2);
    spi_send_byte(pdata,reg);
    for(i=0;i<len;i++)
    {
        spi_send_byte(pdata,data[i]);
    }
    spi_send_end(pdata);
}

static unsigned int spi_read_rtd(struct spi_gpio_rtd_platform_data *pdata,unsigned char reg)
{
    unsigned int tmp = 0;
    spi_send_start(pdata,SPI_MODE_2);
    spi_send_byte(pdata,reg);
    spi_revc_byte(pdata);
    tmp = spi_revc_byte(pdata);
    tmp<<=8;
    tmp |= spi_revc_byte(pdata);
    spi_send_end(pdata);
    return tmp;
}

static void spi_rtd_setup(struct spi_gpio_rtd_data *rtd)
{
    int ret;
    unsigned char data[8];
    struct spi_gpio_rtd_platform_data *pdata = rtd->pdata;
    #if 0
    spi_write_rtd(pdata,CMD_RTD_CONFIG,CONF_VBIAS_ON|CONF_CONVERSION_MODE_AUTO|CONF_FAULT_STATUS_CLEAR_ON);
    #endif
    spi_write_rtd(pdata,CMD_RTD_CONFIG,rtd->rtd_phy.rtd_config);
    ret = spi_read_rtd(pdata,CMD_RTD_READ_CONFIG);
    //if(ret == 0)
    {
        printk(KERN_INFO "Reading2:%d contents of Configuration register to verify communication with max31865 is done properly\n",ret);
        #if 0
        spi_write_rtd(pdata,CMD_WRITE_HIGH_FAULTTHRESHOLD_MSB,0xFF);//Writing High Fault Threshold MSB
        spi_write_rtd(pdata,CMD_WRITE_HIGH_FAULTTHRESHOLD_LSB,0xFF);
        spi_write_rtd(pdata,CMD_WRITE_LOW_FAULTTHRESHOLD_MSB,0x00);
        spi_write_rtd(pdata,CMD_WRITE_LOW_FAULTTHRESHOLD_LSB,0x00);
        #endif
        data[0] = 0xFF;
        data[1] = 0xFF;
        data[2] = 0x00;
        data[3] = 0x00;
        spi_write_burst_rtd(pdata,CMD_WRITE_HIGH_FAULTTHRESHOLD_MSB,data,4);
        rtd->rtd_phy.reference_res = 400;
        rtd->rtd_phy.resistance_rtd = 100;
        
    }
}

static void spi_rtd_full_read(struct spi_gpio_rtd_data *rtd,int *val)
{
    u8 temp;
    u16 rtd_res;
    u16 tmep_u16;
    spi_set_rtd_cs(rtd->pdata,0);
    spi_send_byte(rtd->pdata,rtd->reg);
    temp = spi_revc_byte(rtd->pdata);
    rtd_res =  spi_revc_byte(rtd->pdata);
    rtd_res<<=8;
    rtd_res |= spi_revc_byte(rtd->pdata);
    *val = rtd_res; //注意

    tmep_u16 =  spi_revc_byte(rtd->pdata);
    tmep_u16<<=8;
    tmep_u16 |= spi_revc_byte(rtd->pdata);
    tmep_u16 = tmep_u16>>1;

    tmep_u16 =  spi_revc_byte(rtd->pdata);
    tmep_u16<<=8;
    tmep_u16 |= spi_revc_byte(rtd->pdata);
    spi_set_rtd_cs(rtd->pdata,1);
}


static irqreturn_t  spi_rtd_interrupt(int irq,void *dev_id)
{
    struct spi_gpio_rtd_data *rtd = (struct spi_gpio_rtd_data *)dev_id;
    

    if(gpio_get_value(rtd->pdata->irq[1].gpio) == 1)
    {
        printk(KERN_INFO "spi_rtd_interrupt %s,GPG8 is High... \n",rtd->pdata->irq[1].pDesc);
    }
    else if(gpio_get_value(rtd->pdata->irq[1].gpio) == 0)
    {
        rtd->drdy = 1;
        wake_up_interruptible(&rtd->waitq);
        kill_fasync(&rtd->pFasync,SIGIO,POLL_IN);
        printk(KERN_INFO "spi_rtd_interrupt %s,GPG8 is low... \n",rtd->pdata->irq[1].pDesc);
    }
    return IRQ_RETVAL(IRQ_HANDLED);  
}

static int spi_gpio_rtd_open(struct inode *inode,struct file *filp )
{
    int ret = -ENXIO;

    struct spi_gpio_rtd_data    *rtd;
    mutex_lock(&device_list_lock);
    printk("open is execu333331111...\n");
    printk("inode rdev:%d,arry1:%d,arry2:%d...\n",inode->i_rdev,g_rtd_dev->devt,g_rtd_dev2->devt);
    list_for_each_entry(rtd, &device_list, device_entry){
         if(rtd->devt == inode->i_rdev)
         {
            break;
         }
    }
    //rtd = container_of(inode->i_rdev,struct spi_gpio_rtd_data,devt);
    printk("open is execu22:::%d...\n",rtd->devt);
    if((!rtd))
    {
        printk("kzalloc g_rtd_dev is failed .......\n");
        return -1;
    }
    if(rtd->devt == inode->i_rdev)
    {
        printk("open is execu33...\n");
        ret = 0;
        filp->private_data = rtd;
        nonseekable_open(inode, filp);
    }

    printk("open rtd4 :%d,irq:%s.......\n",rtd->pdata->flag,rtd->pdata->irq[0].pDesc);


    ret = request_irq(rtd->irq,spi_rtd_interrupt,IRQ_TYPE_EDGE_BOTH,rtd->pdata->irq[1].pDesc,(void *)rtd);

    if(ret)
    {
        printk(KERN_INFO "request_irq failed... \n");
    }

    mutex_unlock(&device_list_lock);
    
    return 0;
}

static int spi_gpio_rtd_release(struct inode *inode,struct file *filp)
{
    int ret = 0;
    struct spi_gpio_rtd_data    *pData ;
    pData = filp->private_data;
    filp->private_data = NULL;
    disable_irq(pData->irq);
    free_irq(pData->irq,(void *)pData);
    return ret;
}

static ssize_t spi_gpio_rtd_write(struct file *filp,char __user *buf, size_t count, loff_t *f_pos)
{
    ssize_t ret;
    unsigned long len;
    struct spi_gpio_rtd_data    *spi_gpio_rtd ;
    unsigned char data[3] = {0};
    spi_gpio_rtd = filp->private_data;
    if(count < 3)
    {
        printk("%s write data bytes too litte\n", __func__);
        return -1;
    }
    len = copy_from_user(data,buf,3);

    switch(data[0])
    {
        case CMD_RTD_WRITE:
        spi_write_rtd(spi_gpio_rtd->pdata,data[1],data[2]);
        ret = 3;
        break;
        case CMD_RTD_READ:
        spi_gpio_rtd->reg = data[1];
        printk(KERN_INFO "max31685 reg:%d....\n",spi_gpio_rtd->reg);
        break;
    }

    return ret ;
}

static ssize_t spi_gpio_rtd_read(struct file *filp,char __user *buf, size_t count, loff_t *f_pos)
{
    int ret = 0;
    unsigned int data;
    struct spi_gpio_rtd_data    *spi_gpio_rtd ;
    spi_gpio_rtd = filp->private_data;
    if(count == 0)
    return ret ;
    if(filp->f_flags & O_NONBLOCK)//非阻塞的读取
    {
        if(!spi_gpio_rtd->drdy)
        {
            return -EAGAIN;
        }
    }
    else
    {
        if(!spi_gpio_rtd->drdy)
            wait_event_interruptible(spi_gpio_rtd->waitq,spi_gpio_rtd->drdy);
    }
    spi_gpio_rtd->drdy = 0;
    down_interruptible(&spi_gpio_rtd->sem);
    data = spi_read_rtd(spi_gpio_rtd->pdata,spi_gpio_rtd->reg); 
    printk(KERN_INFO "wake up to read4:%02x...\n",data);
    ret = copy_to_user(buf,&data,2);
    up(&spi_gpio_rtd->sem);
    
    if(ret == 1)
        ret = -EFAULT;

     return ret ;
}

static int spi_gpio_rtd_fasync(int fd,struct file *filp,int mode)
{

    return fasync_helper(fd,filp,mode,&g_rtd_dev2->pFasync);

}

static int spi_gpio_rtd_poll(struct file *filp,poll_table *wait)
{
    struct spi_gpio_rtd_data *spi_gpio_rtd = filp->private_data;
    unsigned int mask = 0;
    poll_wait(filp,&spi_gpio_rtd->waitq,wait);
    if(spi_gpio_rtd->drdy)
    {
        mask |=POLLIN|POLLRDNORM;
    }

    return mask;
}
static int spi_gpio_rtd_test(struct spi_gpio_rtd_data *rtd,int cnt)
{  
       int i;
       int rtd_res;
       spi_set_rtd_cs(rtd->pdata,0);
       spi_send_byte(rtd->pdata,0x00);
       for(i=0;i<cnt;i++)
       {
                rtd_res =  spi_revc_byte(rtd->pdata);
                rtd_res<<=8;
                rtd_res |= spi_revc_byte(rtd->pdata);
                printk(KERN_INFO "%d read :%02x\n",__LINE__,rtd_res);
       }
       spi_set_rtd_cs(rtd->pdata,1);

       return rtd_res;
}
static int spi_gpio_rtd_ioctl(struct file *filp,unsigned int cmd,unsigned long arg)
{
    int i;
    int ret;
    int err = 0;
    int temp;
    struct spi_gpio_rtd_data    *spi_gpio_rtd ;
    spi_gpio_rtd = filp->private_data;
 
    if(_IOC_TYPE(cmd) != RTD_IOC_MAGIC) /* 检测命令的有效性 */
        return -EINVAL;
    if(_IOC_NR(cmd) > RTD_MAXNR)
        return -EINVAL;  
    if(_IOC_DIR(cmd) & _IOC_READ) /* 根据命令类型，检测参数空间是否可以访问 */
        err = !access_ok(VERIFY_WRITE,(void *)arg,_IOC_SIZE(cmd)); 
    if(_IOC_DIR(cmd) & _IOC_WRITE)
        err = !access_ok(VERIFY_READ,(void *)arg,_IOC_SIZE(cmd));  
    if(err)
    {
        printk(KERN_INFO "erron....\n");
        return -EINVAL;
    }

    switch(cmd)
    {
        case RTD_IOCSQSETCONF:
            ret = __get_user(spi_gpio_rtd->rtd_phy.rtd_config,(char __user *)arg);
            spi_rtd_setup(spi_gpio_rtd);
        break;
            
        case RTD_IOCGQSETCONF:
            ret = __put_user(spi_gpio_rtd->rtd_phy.rtd_config,(char __user*)arg);
        break;
        
        case RTD_IOCXQREAD:
            ret = __get_user(err,(int __user *)arg);
            ret = __put_user(spi_gpio_rtd_test(spi_gpio_rtd,err),(int __user*)arg);
        break;
        
        case RTD_IOCSQWRITEBYTE:
        
        break;
        case RTD_IOCTQAUNTUM:
        break;
        case RTD_IOCFULLREAD:
            spi_rtd_full_read(spi_gpio_rtd,&temp);
        break;
        case RTD_IOCGFULLREAD:

        break;
        
        case RTD_IOCXREADTEST:   
            ret = __get_user(err,(int __user *)arg);
            printk(KERN_INFO,"ERR:%d....\n",err);
            for(i=0;i<1;i++)
                    spi_rtd_full_read(spi_gpio_rtd,&temp);
            printk(KERN_INFO "temp val4:%02x\n",temp); 
            ret = __put_user(temp,(int __user*)arg);
        break;
        default:
        
        break;
    }

    return ret;
}

static int __devinit   spi_gpio_rtd_alloc(unsigned pin ,const char *label,bool is_in)
{
    int    value;
    value = gpio_request(pin,  label);
    if(value == 0)
    {   
        if(is_in)
            value = gpio_direction_input(pin);
        else
            value = gpio_direction_output(pin, 0);
    }
    return value;
}

static int __init spi_gpio_rtd_request(struct spi_gpio_rtd_platform_data *pdata,const char *label)
{
    int value;
    value = spi_gpio_rtd_alloc(SPI_RTD_MOSI_GPIO,label,false);
    if(value)
        goto done;
    value = spi_gpio_rtd_alloc(SPI_RTD_MISO_GPIO,label,true);
    if(value)
        goto free_mosi;
    value = spi_gpio_rtd_alloc(SPI_RTD_SCK_GPIO,label,false);
    if(value)
        goto free_miso;    

    value = spi_gpio_rtd_alloc(SPI_RTD_CS_GPIO,label,false);
    if(value)
        goto free_cs;    

    goto done;

    free_cs:
    gpio_free(SPI_RTD_CS_GPIO);
    free_miso:
    gpio_free(SPI_RTD_MISO_GPIO);
    free_mosi:
    gpio_free(SPI_RTD_MOSI_GPIO);
    done:
    return value;
}



static const struct file_operations spi_gpio_rtd_fops = {
    .owner = THIS_MODULE,
    .write = spi_gpio_rtd_write,
    .read = spi_gpio_rtd_read,
    .open = spi_gpio_rtd_open,
    .unlocked_ioctl = spi_gpio_rtd_ioctl,
    .fasync = spi_gpio_rtd_fasync,
    .poll   = spi_gpio_rtd_poll,
    .release = spi_gpio_rtd_release,
};


static int __init spi_gpio_rtd_probe(struct platform_device *pdev)
{
    int ret;
    long minor;
    struct resource *spi_plat_resource;
    struct spi_gpio_rtd_data    *spi_gpio_rtd ;
    struct spi_gpio_rtd_platform_data *pdata =  pdev->dev.platform_data;
    
    ret = spi_gpio_rtd_request(pdata,dev_name(&pdev->dev));
    spi_gpio_rtd= (struct spi_gpio_rtd_data  *)kzalloc(sizeof(struct spi_gpio_rtd_data),GFP_KERNEL);
    if(!spi_gpio_rtd)
    {
        printk("====== malloc spi_gpio_rtd drvier data failed\n");
        ret = -ENOMEM;
        goto free_gpio;
    }

    if(pdata)
    {
        spi_gpio_rtd->pdata = pdata;
    }

    INIT_LIST_HEAD(&spi_gpio_rtd->device_entry);
    
    spi_gpio_rtd->pdev = pdev;
    spi_gpio_rtd->drdy = 0;
    init_waitqueue_head(&spi_gpio_rtd->waitq);
    sema_init(&spi_gpio_rtd->sem,1);

    BUILD_BUG_ON( SPI_RTD_MINORS > 256);
    ret = register_chrdev(SPI_RTD_MAJOR,"spi_rtd",&spi_gpio_rtd_fops);
    if(ret < 0)
    {
        printk("====== %s register chrdev failed, status %d\n", __func__, ret);
        return ret;
    }
    spi_rtd_class = class_create(THIS_MODULE,"spi_rtd");
    if(IS_ERR(spi_rtd_class )){
        printk("====== %s class_create failed \n", __func__);
        unregister_chrdev(SPI_RTD_MAJOR ,"spi_rtd");
        return PTR_ERR(spi_rtd_class );
    }

    mutex_lock(&device_list_lock);
    minor = find_first_zero_bit(minors,SPI_RTD_MINORS);

    if(minor < SPI_RTD_MINORS)
    {
        struct device *dev;
        printk("rtd device_create111111 ... \n");
        spi_gpio_rtd->devt = MKDEV(SPI_RTD_MAJOR,minor);
        dev = device_create(spi_rtd_class,&pdev->dev,spi_gpio_rtd->devt,
        spi_gpio_rtd,"spi_rtd");
        ret = IS_ERR(dev)?PTR_ERR(dev):0;
    }
    else
    {
        dev_dbg(&pdev->dev, "spi_gpio_rtd no minor number available!\n");
        ret  = -ENODEV;
    }    

    if(ret == 0){
        set_bit(minor,minors);
    }

    list_add(&spi_gpio_rtd->device_entry, &device_list);
    mutex_unlock(&device_list_lock);
    
    g_rtd_dev = spi_gpio_rtd;

    spi_plat_resource = platform_get_resource(spi_gpio_rtd->pdev,IORESOURCE_IRQ,0);
    spi_plat_resource = platform_get_resource(spi_gpio_rtd->pdev,IORESOURCE_IRQ,1);
    spi_gpio_rtd->irq = spi_plat_resource->start;

    printk("%d,%d,%d is getresource ...\n",spi_plat_resource->start,spi_plat_resource->end,spi_plat_resource->flags);
    
    if(spi_plat_resource == NULL)
    {
        printk(KERN_INFO "platform_get_resource failed... \n");
        return -EINVAL;
    }

    
    platform_set_drvdata(pdev,spi_gpio_rtd);
    printk("probe111111 is finished2 %d ... \n",pdata->flag);
    if(ret<0)
    free_gpio:
    {
        printk(KERN_INFO "free gpio...............\n");
        gpio_free(pdata->sck);
        gpio_free(pdata->miso);
        gpio_free(pdata->mosi);
        gpio_free(pdata->cs);
    }

    return ret;
}
 
static int __devexit spi_gpio_rtd_remove(struct platform_device *pdev)
{

    struct spi_gpio_rtd_data    *spi_gpio_rtd ;
    struct spi_gpio_rtd_platform_data *pdata ;
    spi_gpio_rtd = platform_get_drvdata(pdev);
    pdata = pdev->dev.platform_data;
    mutex_lock(&device_list_lock);
    
    platform_set_drvdata(pdev,NULL);
    list_del(&spi_gpio_rtd->device_entry);
    
    mutex_unlock(&device_list_lock);
    gpio_free(pdata->sck);
    gpio_free(pdata->miso);
    gpio_free(pdata->mosi);
    gpio_free(pdata->cs);
    
    return 0;
}
static int __init spi_gpio_rtd_probe2(struct platform_device *pdev)
{
    int ret;
    long minor;
    struct resource *spi_plat_resource;
    struct spi_gpio_rtd_data    *spi_gpio_rtd ;
    struct spi_gpio_rtd_platform_data *pdata =  pdev->dev.platform_data;
    
    ret = spi_gpio_rtd_request(pdata,dev_name(&pdev->dev));
    spi_gpio_rtd= (struct spi_gpio_rtd_data  *)kzalloc(sizeof(struct spi_gpio_rtd_data),GFP_KERNEL);
    if(!spi_gpio_rtd)
    {
        printk("====== malloc spi_gpio_rtd drvier data failed\n");
        ret = -ENOMEM;
        goto free_gpio;
    }
    
    INIT_LIST_HEAD(&spi_gpio_rtd->device_entry);
    
    if(pdata)
    {
        spi_gpio_rtd->pdata = pdata;
    }
    spi_gpio_rtd->pdev = pdev;
    spi_gpio_rtd->drdy = 0;
    init_waitqueue_head(&spi_gpio_rtd->waitq);
    sema_init(&spi_gpio_rtd->sem,1);

    BUILD_BUG_ON( SPI_RTD_MINORS2 > 256);
    ret = register_chrdev(SPI_RTD_MAJOR2,"spi_rtd2",&spi_gpio_rtd_fops);
    if(ret < 0)
    {
        printk("====== %s register chrdev failed, status %d\n", __func__, ret);
        return ret;
    }
    spi_rtd_class2 = class_create(THIS_MODULE,"spi_rtd2");
    if(IS_ERR(spi_rtd_class2 )){
        printk("====== %s class_create failed \n", __func__);
        unregister_chrdev(SPI_RTD_MAJOR2 ,"spi_rtd2");
        return PTR_ERR(spi_rtd_class2 );
    }

    mutex_lock(&device_list_lock);
    minor = find_first_zero_bit(minors,SPI_RTD_MINORS2);

    if(minor < SPI_RTD_MINORS2)
    {
        struct device *dev;
        printk("rtd device_create222222 ... \n");
        spi_gpio_rtd->devt = MKDEV(SPI_RTD_MAJOR2,minor);
        dev = device_create(spi_rtd_class2,&pdev->dev,spi_gpio_rtd->devt,
        spi_gpio_rtd,"spi_rtd2");
        ret = IS_ERR(dev)?PTR_ERR(dev):0;
    }
    else
    {
        dev_dbg(&pdev->dev, "spi_gpio_rtd no minor number available!\n");
        ret  = -ENODEV;
    }    

    if(ret == 0){
        set_bit(minor,minors);
    }

    list_add(&spi_gpio_rtd->device_entry, &device_list);
    mutex_unlock(&device_list_lock);
    
    g_rtd_dev2 = spi_gpio_rtd;

    spi_plat_resource = platform_get_resource(spi_gpio_rtd->pdev,IORESOURCE_IRQ,0);
    spi_plat_resource = platform_get_resource(spi_gpio_rtd->pdev,IORESOURCE_IRQ,1);
    spi_gpio_rtd->irq = spi_plat_resource->start;

    printk("%d,%d,%d is getresource ...\n",spi_plat_resource->start,spi_plat_resource->end,spi_plat_resource->flags);
    
    if(spi_plat_resource == NULL)
    {
        printk(KERN_INFO "platform_get_resource failed... \n");
        return -EINVAL;
    }

    
    platform_set_drvdata(pdev,spi_gpio_rtd);
    printk("probe222222 is finished2 %d ... \n",pdata->flag);
    if(ret<0)
    free_gpio:
    {
        printk(KERN_INFO "free gpio...............\n");
        gpio_free(pdata->sck);
        gpio_free(pdata->miso);
        gpio_free(pdata->mosi);
        gpio_free(pdata->cs);
    }

    return ret;
}

static int __devexit spi_gpio_rtd_remove2(struct platform_device *pdev)
{

    struct spi_gpio_rtd_data    *spi_gpio_rtd ;
    struct spi_gpio_rtd_platform_data *pdata ;
    spi_gpio_rtd = platform_get_drvdata(pdev);
    pdata = pdev->dev.platform_data;
    mutex_lock(&device_list_lock);
    
    platform_set_drvdata(pdev,NULL);
    list_del(&spi_gpio_rtd->device_entry);
    
    mutex_unlock(&device_list_lock);
    gpio_free(pdata->sck);
    gpio_free(pdata->miso);
    gpio_free(pdata->mosi);
    gpio_free(pdata->cs);
    
    return 0;
}


static struct platform_driver   spi_gpio_rtd_driver1 = {
    .driver =   {
        .name   =   "spi_gpio_rtd1",
        .owner  =    THIS_MODULE,
    }   ,
    .probe = spi_gpio_rtd_probe,
    .remove=__devexit_p(spi_gpio_rtd_remove),
};


static struct platform_driver   spi_gpio_rtd_driver2 = {
    .driver =   {
        .name   =   "spi_gpio_rtd2",
        .owner  =    THIS_MODULE,
    }   ,
    .probe = spi_gpio_rtd_probe2,
    .remove=__devexit_p(spi_gpio_rtd_remove2),
};



static int __init   spi_rtd_init()
{
    int ret;

    ret = platform_driver_register(&spi_gpio_rtd_driver1);
    printk("platform_driver_register1...\n", __func__);
    if(ret < 0 )
    {
        if(!IS_ERR(spi_rtd_class ))
        {
            printk("====== %s platform driver register1 failed \n", __func__);
            class_destroy(spi_rtd_class);
            unregister_chrdev(SPI_RTD_MAJOR ,spi_gpio_rtd_driver1.driver.name);
        }
    }
 
    ret = platform_driver_register(&spi_gpio_rtd_driver2);
    printk("platform_driver_register2...\n", __func__);
    if(ret < 0 )
    {
        if(!IS_ERR(spi_rtd_class ))
        {
            printk("====== %s platform driver register2 failed \n", __func__);
            class_destroy(spi_rtd_class2);
            unregister_chrdev(SPI_RTD_MAJOR2 ,spi_gpio_rtd_driver2.driver.name);
        }
    }
    return ret;
}

static void __exit  spi_rtd_exit()
{
 
    platform_driver_unregister(&spi_gpio_rtd_driver1);
    if(!IS_ERR(spi_rtd_class ))
    {
        class_destroy(spi_rtd_class); 
        unregister_chrdev(SPI_RTD_MAJOR ,spi_gpio_rtd_driver1.driver.name);
    }

    platform_driver_unregister(&spi_gpio_rtd_driver2);  
    if(!IS_ERR(spi_rtd_class2 ))
    {
        class_destroy(spi_rtd_class2);
        unregister_chrdev(SPI_RTD_MAJOR ,spi_gpio_rtd_driver2.driver.name);
    }
}

module_init(spi_rtd_init);
module_exit(spi_rtd_exit);




