// tcp客户端端的代码
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "protocol.hpp"
#include <iostream>

char buff[4 * 138] = {0};

void createData()
{
    float my_data1[138];
    for (int i{0}; i < 138; i++)
    {
        my_data1[i] = i + 0.002;
    }
    // float转char

    for (int i{0}; i < 138 * 4; i++)
    {
        char *pc = (char *)(&my_data1);
        buff[i] = *(pc + i);
    }
}

void sendData(char *buf)
{
    ComunicationData a;
    int cnt{0};
    char *pc = nullptr;
    a.write_data = {0};

    pc = (char *)(&a.write_data);
    for (int i{0}; i < sizeof(WriteData); i++, cnt++)
    {
        buf[cnt] = *(pc + i);
    }
}

int main()
{
    // 1、创建socket套接字
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
    {
        perror("socket error");
        return -1;
    }
    struct sockaddr_in ser;
    ser.sin_family = AF_INET;
    ser.sin_port = htons(atoi("9000"));
    ser.sin_addr.s_addr = inet_addr("127.0.0.1");
    // 2、连接
    socklen_t len = sizeof(struct sockaddr_in);
    if (connect(sockfd, (struct sockaddr *)&ser, len) < 0)
    {
        perror("connect error");
        return -1;
    }
    // 3、发送数据

    float my_datas[138];
    // sendData(buff);
    while (1)
    {
        scanf("%s", buff);
        sendData(buff);
        send(sockfd, buff, sizeof(buff), 0);
        memset(buff, 0x00, 4 * 138);

        int ret = recv(sockfd, buff, 4 * 138, 0);

        for (int i{0}; i < 138; i++)
        {
            char *pa;
            pa = buff;
            float b = *(float *)&pa[i * 4];
            my_datas[i] = b;
        }

        for(int i{0}; i < 138; i++)
        {
            std::cout<<my_datas[i]<<std::endl;
        }

        if (ret < 0)
        {
            perror("recv error");
            continue;
        }
        printf("client %s[%d]say:%s", inet_ntoa(ser.sin_addr), ntohs(ser.sin_port), buff);
    }
    close(sockfd);
}
