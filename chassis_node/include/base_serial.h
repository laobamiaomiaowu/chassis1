#ifndef BASE_SERIAL_H
#define BASE_SERIAL_H
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <time.h>
#include <thread>
#include <mutex>
#include <list>
#include <signal.h>
#include <pthread.h>
#include <string.h>
#include <deque> 

#define BigLittleSwap16(A) (((short)(A)&0xff00)>>8) | (((short)(A)&0x00ff)<<8)
#define BigLittleSwpa32(A) ((((int)(A)&0xff000000)>>24)|(((int)(A)&0x00ff0000)>>8)|(((int)(A)&0x0000ff00)<<8)|(((int)(A)&0x000000ff)<<24))

using namespace std;

// 底盘串口接受数据结构体
struct MotoRev
{
    int leftFrontMoto; // 左前电机编码器计数值
    int rightFrontMoto; // 右前电机编码器计数值
    int leftBehindMoto; // 左后电机编码器计数值
    int rightBehindMoto; // 右后电机编码器计数值
    uint8_t batteryLevel; // 电池电量
    unsigned short batteryVoltage; // 电池电压
};

//消息队列结构体
struct msbuf
{
    long int    mtype;     // 消息类型
    MotoRev    mtext;  // 消息内容
};

class BaseSerial
{
public:
	BaseSerial();
	~BaseSerial();
	// 测试函数
	void test();
	// 打开串口
	int open_port(int fd,const char *pathname);
	//设置串口
	int set_port(int fd,int baude,int c_flow,int bits,char parity,int stop);
	//关闭串口
	int close_port(int fd);
	//写串口
	int write_port(int fd,const char *w_buf,size_t len);
	//接收串口
	static void* recv_port(void* args);
	//接受串口后数据处理
    void DisposeData(char * unpData,int dataLen);
	//设置电机速度
    void InputMotSpeed(short speedLeft, short speedRight);
    

    pthread_mutex_t msgMute;
	deque<MotoRev> buffer_rev; //双段队列存放接收到的底盘串口接受数据结构体
	
private:
	key_t serialKey;
    int   serialMsgid;

	pthread_rwlock_t rwMute;
    int fd; //串口设置
    pthread_t threadSend; //发送线程
    pthread_t threadSerial; //接收线程
};

#endif // BASE_SERIAL_H
