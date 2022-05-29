#ifndef _SOCKETS_H_
#define _SOCKETS_H_

#include<stdio.h>
#include<windows.h>

#pragma comment(lib,"WS2_32.lib")
#pragma warning(disable:4996)

//打开网络通信 
bool OpenSockets(SOCKET& sock, const char IP[], const unsigned short Port)
{
	WSADATA wsaData;
	SOCKADDR_IN addrSrv;

	if (!WSAStartup(MAKEWORD(1, 1), &wsaData))
	{
		if ((sock = socket(AF_INET, SOCK_STREAM, 0)) != INVALID_SOCKET)
		{
			addrSrv.sin_addr.S_un.S_addr = inet_addr(IP);
			addrSrv.sin_family = AF_INET;
			addrSrv.sin_port = htons(Port);
			connect(sock, (SOCKADDR*)&addrSrv, sizeof(SOCKADDR));
			return true;
		}
	}
	return false;
}

void CloseSockets(SOCKET& sock)
{
	closesocket(sock);
	WSACleanup();
}

#endif
