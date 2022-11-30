#include "chat_plc.h"
#include <functional>
#include "qdebug.h"

void ChatPlc::init()
{
    if (!this->tcp_init())
    {
        return;
    };

    this->init_thread();
}

void ChatPlc::init_thread()
{
    std::thread dowork_thread(std::bind(&ChatPlc::run, this));
    dowork_thread.detach();
}

bool ChatPlc::tcp_init()
{
    qDebug()<<"尝试连接下位机";

    // 1、创建socket套接字
    this->my_sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (this->my_sockfd < 0)
    {
        perror("socket error");
        qCritical()<<"创建socket失败";
        return false;
    }

    qDebug()<<"创建连接下位机的socket成功";

    myaddr.sin_family = AF_INET;
    myaddr.sin_port = htons(atoi(std::to_string(this->my_port).c_str()));
    myaddr.sin_addr.s_addr = inet_addr(this->my_ip.c_str());
    // 2、连接
    socklen_t len = sizeof(struct sockaddr_in);
    if (connect(this->my_sockfd, (struct sockaddr *)&myaddr, len) < 0)
    {
        //        perror("connect error");
        qCritical()<<"连接下位机失败";
        return false;
    }
    qDebug()<<"连接下位机成功";
    return true;
}

void ChatPlc::UnInitialize()
{
    close(this->my_sockfd);
}

//用std::function+lamda 代替 抽象出协议层
void ChatPlc::run()
{
    // sendData(buff);
    while (1)
    {
        // send datas
        // send(sockfd, buff, sizeof(buff), 0);
        memset(buff, 0x00, RECEIVE_BUFFER_SIZE);

        int ret = recv(this->my_sockfd, buff, RECEIVE_BUFFER_SIZE, 0);

        anlysisReceiveData();

        // prinf test
        // for (int i{0}; i < RECEIVE_DATA_SIZE; i++)
        // {
        //     std::cout << my_datas[i] << std::endl;
        // }

        if (ret < 0)
        {
            //perror("recv error");
            qCritical()<<"recv error";
            break;
            ;
        }
    }
}

//bool send_data(ComunicationData &msg);

bool ChatPlc::send_data(WriteData &msg)
{
    int cnt{0};
    char *pc = nullptr;
    char send_buf[RECEIVE_BUFFER_SIZE] ={0};

    pc = (char *)(&msg);
    for (int i{0}; i < (int)sizeof(WriteData); i++, cnt++)
    {
        send_buf[cnt] = *(pc + i);
    }
    if(send(this->my_sockfd, send_buf, sizeof(send_buf), 0) != -1)
    {
        return true;
    }
    else {
        return false;
    }

}

void ChatPlc::anlysisReceiveData()
{

    int cnt{0};
    char *pc = nullptr;
    ReadData msg ;
    memset(&msg, 0, sizeof(msg));

    pc = (char *)(&msg);
    for (int i{0}; i < (int)sizeof(ReadData); i++, cnt++)
    {
        *(pc + i) = buff[cnt];
    }

    msg = *(ReadData *)pc;

    std::lock_guard<std::mutex> lck(m_lock);
    recv_data_queue_.push(msg);


    if (recv_data_queue_.size() > 3)
    {
        // recv_data_queue_.pop();
        qCritical()<<"警告：接收到PCL数据队列超过3！";
    }
}
