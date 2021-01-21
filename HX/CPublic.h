#pragma once
#include <string>
#include "CSerialPort/SerialPort.h"
#include "CSerialPort/SerialPortInfo.h"

using namespace itas109;


class CPublic
{
public:
	CPublic();
	virtual ~CPublic();

public:
	


	//内存泄漏
	//static CSerialPort m_SerialPort;//About CSerialPort
public:
	////创建CRC16的全局函数
	//static unsigned short CRC16(unsigned char* puchMsg, unsigned short usDataLen);
	//
	////发送函数
	//static void SendData(int CommTypeIn, WORD DownAdd, DWORD DownData);

};



