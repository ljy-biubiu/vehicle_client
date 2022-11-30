// tcp服务端的代码
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <iostream>
#include "protocol.hpp"

char my_data[4 * 138];
float my_receive_data[138];

void sendData(char *buf)
{
    ComunicationData a;
    int cnt{0};
    char *pc = nullptr;
    a.read_data = {0};

    a.read_data.amplitude_position = 9999.999;

    pc = (char *)(&a.read_data);
    for (int i{0}; i < sizeof(ReadData); i++, cnt++)
    {
        buf[cnt] = *(pc + i);
    }
}


void createData()
{
    float my_data1[138];
    for (int i{0}; i < 138; i++)
    {
        my_data1[i] = i + 0.002;
    }
    // float转char

    // for (int i{0}; i < 138; i++)
    // {
    //     // std::cout<<my_data1[i]<<std::endl;
    //     char *pc = (char *)(&my_data1[i]);
    //     for (int y = 0; y < 4; y++)
    //     {
    //         my_data[y + i*4] = *( pc + y );
    //     }
    // }

    for (int i{0}; i < 138 * 4; i++)
    {
        char *pc = (char *)(&my_data1);
        my_data[i] = *(pc + i);
    }

    // for (int i{0}; i < 138; i++)
    // {
    //     char *pa;
    //     pa = my_data;
    //     float b = *(float *)&pa[i*4];
    //     std::cout << b << std::endl;
    // }
}

int main()
{

    //createData();

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

    // 2、绑定地址
    socklen_t len = sizeof(struct sockaddr_in);
    if (bind(sockfd, (struct sockaddr *)&ser, len) < 0)
    {
        perror("bind error");
        close(sockfd);
        return -1;
    }

    // 3、监听
    if (listen(sockfd, 5) < 0)
    {
        perror("listen error");
        close(sockfd);
        return -1;
    }
    while (1)
    {
        int n_sockfd;
        struct sockaddr_in cli;
        // 4、accept
        n_sockfd = accept(sockfd, (struct sockaddr *)&cli, &len);
        if (n_sockfd < 0)
        {
            perror("accept error");
            continue;
        }
        // 5、recv
        while (1)
        {
            // // 6、send
            char buff[4 * 138] = {0};
            int ret = recv(n_sockfd, buff, 4 * 138, 0);
            if (ret < 0)
            {
                perror("recv error");
                continue;
            }

            for (int i{0}; i < 138; i++)
            {
                char *pa;
                pa = buff;
                float b = *(float *)&pa[i * 4];
                my_receive_data[i] = b;
            }

            for(int i{0}; i < 138; i++)
            {
                std::cout<<my_receive_data[i]<<std::endl;
            }

            printf("client %s[%d]say:%s", inet_ntoa(cli.sin_addr), ntohs(cli.sin_port), buff);

            // memset(buff, 0x00, 1024);
            // scanf("%s", buff);
            // sendData(buff);
            // std::cout<<"waitto send"<<std::endl;
            // send(n_sockfd, buff, sizeof(buff), 0);
            // std::cout<<"has send"<<std::endl;
            
            // 7、close(sockfd)
        }
        close(n_sockfd);
    }
}
