#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <signal.h>  
#include <poll.h> 
#include <math.h>

/* SPI Commands */

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
#define Rref                    4000         //if using PT100, change this from 4000 to 400

int  g_drdy = 0;

void read_signal_handler(int num)
{
    printf("intterrput is coming ...\n");
    g_drdy = 1;
}

void configure(int fd)
{ 
        char arg;
        {
            arg = 0xC2|0x20;
            if(ioctl(fd,RTD_IOCSQSETCONF,&arg)<0)
            {
                printf("%d error\n",__LINE__);
            }
            arg = 0;
            if(ioctl(fd,RTD_IOCGQSETCONF,&arg)<0)
            {
                printf("%d error\n",__LINE__);
            }
            printf("get34 config:%02x\n",arg);
        }
}


int main(int argc,char **argv)
{
    int cmd;
    int flag;
    char arg;
    float AD_Value;                   //store the value of RTD MSB and RTD LSB register
	float RTD_Resistor;            //RTD resistor value based on RTD_MSB and RTD_LSB register
	float RTD_Temperature;         // Temperature value based on direct read out data
	float Corrected_Temperature;   //Temperature after correction
   
    int fd = open("/dev/spi_rtd",O_RDWR);
    if(fd < 0)
        printf("open failed\n");
        
    #if 0
    signal(SIGIO,read_signal_handler);
    fcntl(fd, F_SETOWN, getpid());  
    flag = fcntl(fd,F_GETFL);  
    fcntl(fd,F_SETFL,flag | FASYNC);  
    #endif
    if(atoi(argv[1]) == 1)
    {
        //sscanf(argv[2],"%02x",arg);
        arg = 0x91;
        if(ioctl(fd,RTD_IOCSQSETCONF,&arg)<0)
        {
            printf("%d error\n",__LINE__);
        }
        arg = 0;
        if(ioctl(fd,RTD_IOCGQSETCONF,&arg)<0)
        {
            printf("%d error\n",__LINE__);
        }

        printf("get config:%02x\n",arg);
        
        int arg1 = 5;
        if(ioctl(fd,RTD_IOCXQREAD,&arg1)<0)
        {
            printf("%d error\n",__LINE__);
        }

        printf("get value:%02x\n",arg1);
    }

    else if(atoi(argv[1]) == 2)
    {
        configure(fd);
        configure(fd);
        configure(fd);
    }
    char buf[3] = {0xFF,0x01,0x00};

    if(write(fd,buf,3) < 0)
    {
         printf("%d error\n",__LINE__);
    }
    
    while(1)
    { 
        int arg2;
        unsigned char buf2[3];

        #if 1
        if(ioctl(fd,RTD_IOCXREADTEST,&arg2)<0)
        {
             printf("%d error\n",__LINE__);
        }
        buf2[0] = (arg2>>8)&0x00FF;
        buf2[1] =  arg2&0x00FF;
        #else
        arg2 = read(fd,buf2,2);
        printf("read:%02x%02x\n",buf2[0],buf2[1]);
        #endif

        AD_Value = (((buf2[0]<<8)|buf2[1])&0xFFFE)>>1;
        RTD_Resistor=((AD_Value*4000.0000)/32768.00);
		RTD_Temperature=(AD_Value/32.00)-256;
		printf("rdt res2 %f,%d\n",RTD_Resistor,RTD_Resistor);
		if(RTD_Resistor>=(Rref/4))      //temperature>0
            Corrected_Temperature=((sqrt(pow(0.0039083,2)+4*0.0000005775*(1-(RTD_Resistor/Rref)*4))-0.0039083)/(2*(0-0.0000005775)));
        else
        {
            
			if(Rref==400)
		         Corrected_Temperature = 0-241.96+2.2163*RTD_Resistor+0.0028541*pow(RTD_Resistor,2)-0.000009912*pow(RTD_Resistor,3)+0.000000017052*pow(RTD_Resistor,4);
			if(Rref==4000)
				 Corrected_Temperature = 0-241.96+2.2163*RTD_Resistor*0.1+0.0028541*pow((RTD_Resistor*0.1),2)-0.000009912*pow((RTD_Resistor*0.1),3)+0.000000017052*pow((RTD_Resistor*0.1),4);	
        }

        printf("current temp4:%fn",Corrected_Temperature);
            
        sleep(1);
    }
}

