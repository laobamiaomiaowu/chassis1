#include <iostream>
#include <arpa/inet.h>
#include <pthread.h>
#include <string.h>
#include "../include/base_serial.h"

pthread_mutex_t waitMute=PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond=PTHREAD_COND_INITIALIZER;

BaseSerial::BaseSerial()
{

    serialKey = ftok(".",6);

    if(serialKey >= 0)
    {
       serialMsgid = msgget(serialKey,IPC_CREAT | 0x0666);
    }
    pthread_rwlockattr_t attr;
    pthread_rwlockattr_init(&attr);
    pthread_rwlockattr_setkind_np(&attr,PTHREAD_RWLOCK_PREFER_WRITER_NP); //WRITE FIRST
    pthread_rwlockattr_setpshared(&attr,PTHREAD_PROCESS_SHARED);
    pthread_rwlock_init(&rwMute,&attr);
    pthread_rwlockattr_destroy(&attr);

    //串口开启，设置
    fd = open_port(fd,"/dev/chassis_port");/*串口号/dev/ttySn,USB口号/dev/ttyUSBn*/
    if(fd == -1)
    {
        fprintf(stderr,"uart_open error\n");
        exit(EXIT_FAILURE);
    }

    if(set_port(fd,115200,0,8,'N',1) == -1)
    {
        fprintf(stderr,"uart set failed!\n");
        exit(EXIT_FAILURE);
    }

    //创建收发线程
    pthread_create(&threadSerial,NULL,recv_port,(void*)this);
    //pthread_create(&threadSend,NULL,InputMotSpeed,(void*)this);

}

BaseSerial::~BaseSerial()
{
    pthread_rwlock_destroy(&rwMute);
    msgctl(serialMsgid,IPC_RMID,NULL);
    close_port(fd);
}

//打开串口
int BaseSerial::open_port(int fd,const char *pathname)
{
    fd = open(pathname,O_RDWR|O_NOCTTY| O_NONBLOCK); //NO DELAY
//    fd = open(pathname,O_RDWR|O_NOCTTY); // DELAY
    if(fd == -1)
    {
        perror("Open UART failed!");
        return -1;
    }
    return fd;
}

int BaseSerial::set_port(int fd,int baude,int c_flow,int bits,char parity,int stop)
{

    struct termios options;
    /*设置输入输出波特率，两者保持一致*/
    switch(baude)
    {
        case 4800:
            cfsetispeed(&options,B4800);
            cfsetospeed(&options,B4800);
            break;
        case 9600:
            cfsetispeed(&options,B9600);
            cfsetospeed(&options,B9600);
            break;
        case 19200:
            cfsetispeed(&options,B19200);
            cfsetospeed(&options,B19200);
            break;
        case 38400:
            cfsetispeed(&options,B38400);
            cfsetospeed(&options,B38400);
            break;
        case 115200:
            cfsetispeed(&options,B115200);
            cfsetospeed(&options,B115200);
            break;
        default:
            fprintf(stderr,"Unkown baude!\n");
            return -1;
    }

    /*设置控制模式*/
    options.c_cflag |= CLOCAL;//保证程序不占用串口
    options.c_cflag |= CREAD;//保证程序可以从串口中读取数据

    /*设置数据流控制*/
    switch(c_flow)
    {
        case 0://不进行流控制
            options.c_cflag &= ~CRTSCTS;
            break;
        case 1://进行硬件流控制
            options.c_cflag |= CRTSCTS;
            break;
        case 2://进行软件流控制
            options.c_cflag |= IXON|IXOFF|IXANY;
            break;
        default:
            fprintf(stderr,"Unkown c_flow!\n");
            return -1;
    }

    /*设置数据位*/
    switch(bits)
    {
        case 5:
            options.c_cflag &= ~CSIZE;//屏蔽其它标志位
            options.c_cflag |= CS5;
            break;
        case 6:
            options.c_cflag &= ~CSIZE;//屏蔽其它标志位
            options.c_cflag |= CS6;
            break;
        case 7:
            options.c_cflag &= ~CSIZE;//屏蔽其它标志位
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag &= ~CSIZE;//屏蔽其它标志位
            options.c_cflag |= CS8;
            break;
        default:
            fprintf(stderr,"Unkown bits!\n");
            return -1;
    }

    /*设置校验位*/
    switch(parity)
    {
        /*无奇偶校验位*/
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_iflag &= ~INPCK;//INPCK：使奇偶校验起作用
            break;
        /*设为空格,即停止位为2位*/
        case 's':
        case 'S':
            options.c_cflag &= ~PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag &= ~CSTOPB;//CSTOPB：使用两位停止位
            break;
        /*设置奇校验*/
        case 'o':
        case 'O':
            options.c_cflag |= PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag |= PARODD;//PARODD：若设置则为奇校验,否则为偶校验
            options.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
            options.c_cflag |= ISTRIP;//ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
            break;
        /*设置偶校验*/
        case 'e':
        case 'E':
            options.c_cflag |= PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag &= ~PARODD;//PARODD：若设置则为奇校验,否则为偶校验
            options.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
            options.c_cflag |= ISTRIP;//ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
            break;
        default:
            fprintf(stderr,"Unkown parity!\n");
            return -1;
    }

    /*设置停止位*/
    switch(stop)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;//CSTOPB：使用两位停止位
            break;
        case 2:
            options.c_cflag |= CSTOPB;//CSTOPB：使用两位停止位
            break;
        default:
            fprintf(stderr,"Unkown stop!\n");
            return -1;
    }

    options.c_iflag &=~(INLCR | ICRNL);
    options.c_iflag &=~(IXON | IXOFF | IXANY);
    /*设置输出模式为原始输出*/
    options.c_oflag &= ~OPOST;//OPOST：若设置则按定义的输出处理，否则所有c_oflag失效
    options.c_oflag &= ~(ONLCR | OCRNL);

    /*设置本地模式为原始模式*/
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    /*
     *ICANON：允许规范模式进行输入处理
     *ECHO：允许输入字符的本地回显
     *ECHOE：在接收EPASE时执行Backspace,Space,Backspace组合
     *ISIG：允许信号
     */

    /*设置等待时间和最小接受字符*/ //DELAY STA WORK
    options.c_cc[VTIME] = 1;//可以在select中设置（单位为100ms）
    options.c_cc[VMIN] = 23;//最少读取一个字符


    /*如果发生数据溢出，只接受数据，但是不进行读操作*/
    tcflush(fd,TCIFLUSH);

    /*激活配置*/
    if(tcsetattr(fd,TCSANOW,&options) < 0)
    {
        perror("tcsetattr failed");
        return -1;
    }

    return 0;

}

int BaseSerial::close_port(int fd)
{
    close(fd);
    return 0;
}


//写串口
int BaseSerial::write_port(int fd,const char *w_buf,size_t len)
{

    int cnt = 0;
    pthread_rwlock_wrlock(&rwMute);
    pthread_mutex_lock(&waitMute);
    cnt = write(fd,w_buf,len);
    tcflush(fd,TCOFLUSH);
    if(cnt == -1)
    {
        fprintf(stderr,"write error!\n");
        return -1;
    }
    pthread_cond_wait(&cond,&waitMute);
    pthread_mutex_unlock(&waitMute);
    pthread_rwlock_unlock(&rwMute);

    return cnt;
}

//接收串口
void* BaseSerial::recv_port(void* args)
{
    BaseSerial *pCser = (BaseSerial*)args;
    char buff[30];
    fd_set rfds;
    int nread = 0;
    struct timeval tv;
    tv.tv_sec=0;
    tv.tv_usec=1000;

    while (1)
    {
        FD_ZERO(&rfds);
        FD_SET(pCser->fd, &rfds);
        if (select(1+pCser->fd, &rfds, NULL, NULL, &tv)>0)
        {
            if (FD_ISSET(pCser->fd, &rfds))
            {
                usleep(10000);
                pthread_mutex_lock(&waitMute);
                nread=read(pCser->fd, buff, sizeof(buff));
                pthread_cond_signal(&cond);
                pthread_mutex_unlock(&waitMute);
    		    printf("readLen = %d\n",nread);

                pCser->DisposeData(buff,nread);
            }
        }
    }
}

//处理接收的数据
void BaseSerial::DisposeData(char *unpData,int dataLen)
{
    char checkSum=0x00;
    for(int i=0;i<dataLen-1;i++){
        checkSum+=unpData[i];
    }
    //检查校验位
    if(checkSum!=unpData[dataLen-1]){
        printf("buffer check failed\n");
        return;
    }else{
        //根据协议数据处理
        MotoRev motoRev;
        int tmp1=0;
        memcpy(&tmp1,unpData+3,4);
        tmp1 = BigLittleSwpa32(tmp1);
        motoRev.leftFrontMoto=tmp1; // 左前电机编码器计数值
        int tmp2=0;
        memcpy(&tmp2,unpData+7,4);
        tmp2 = BigLittleSwpa32(tmp2);
        motoRev.rightFrontMoto=tmp2; // 右前电机编码器计数值
        int tmp3=0;
        memcpy(&tmp3,unpData+11,4);
        tmp3 = BigLittleSwpa32(tmp3);
        motoRev.leftBehindMoto=tmp3; // 左后电机编码器计数值
        int tmp4=0;
        memcpy(&tmp4,unpData+15,4);
        tmp4 = BigLittleSwpa32(tmp4);
        motoRev.rightBehindMoto=tmp4; // 右后电机编码器计数值
        motoRev.batteryLevel=((unsigned int)unpData[19]); // 电池电量(单位:%)
        int tmp5=0;
        memcpy(&tmp5,unpData+20,2);
        tmp5 = BigLittleSwap16(tmp5);
        motoRev.batteryVoltage=tmp5*100; // 电池电压(单位:mv)

        // printf("左前%d\t",motoRev.leftFrontMoto);
        // printf("右前%d\t",motoRev.rightFrontMoto);
        // printf("左后%d\t",motoRev.leftBehindMoto);
        // printf("右后%d\t",motoRev.rightBehindMoto);
        // printf("电量%d\t",motoRev.batteryLevel);
        // printf("电压%d\n",motoRev.batteryVoltage);

        //存入容器
        buffer_rev.push_back(motoRev);
     }
}


//设置电机速度
void BaseSerial::InputMotSpeed(short speedLeft,short speedRight)
{
    //根据协议转将左右轮速度转换为指令
    //不能超过最大速度
    if(speedLeft>100){
        speedLeft=100;
    }
    if(speedLeft<-100){
        speedLeft=-100;
    }
    if(speedRight>100){
        speedRight=100;
    }
    if(speedRight<-100){
        speedRight=-100;
    }
    //存放buffer的数组
    unsigned char sendMotoBuf[12]={};
    sendMotoBuf[0]=0xfe;
    sendMotoBuf[1]=0xef;
    sendMotoBuf[2]=0x08;
    sendMotoBuf[3]=(char)(speedLeft>>8);
    sendMotoBuf[4]=(char)speedLeft;
    sendMotoBuf[5]=(char)(speedRight>>8);
    sendMotoBuf[6]=(char)speedRight;
    sendMotoBuf[7]=(char)(speedLeft>>8);
    sendMotoBuf[8]=(char)speedLeft;
    sendMotoBuf[9]=(char)(speedRight>>8);
    sendMotoBuf[10]=(char)speedRight;
    sendMotoBuf[11]=0x00;
    for(int i=0;i<sizeof(sendMotoBuf)-1;i++){
        sendMotoBuf[11]+=sendMotoBuf[i];
    }
    write_port(fd,(char*)sendMotoBuf,sizeof(sendMotoBuf));
}

