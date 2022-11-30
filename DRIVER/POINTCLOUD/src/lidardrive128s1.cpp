#include "lidardrive128s1.h"
#include <functional>
#include "qdebug.h"



void LidarDrive128S1::init()
{
    this->udp_init(&this->my_sock, &this->myClientAddr, this->my_port, this->my_ip);
    this->init_thread();
}


void LidarDrive128S1::init_thread()
{
    std::thread dowork_thread(std::bind(&LidarDriveInterface::run, this));
    dowork_thread.detach();
}

void LidarDrive128S1::udp_init(int *sock_num, sockaddr_in *sockaddr, int server_port, const std::string ip)
{
    UnInitialize(sock_num);
    sockaddr->sin_family = AF_INET;
    sockaddr->sin_addr.s_addr = inet_addr(ip.c_str());
    sockaddr->sin_port = htons(server_port);
    *sock_num = socket(AF_INET, SOCK_DGRAM, 0);
    bind(*sock_num, (struct sockaddr *)sockaddr, sizeof(*sockaddr));
}

void LidarDrive128S1::UnInitialize(int *sock_num)
{
    shutdown(*sock_num, SHUT_RDWR);
}


void LidarDrive128S1::run()
{
    unsigned int len = sizeof(sockaddr_in);
    myClientAddr = myaddr;
    //接收数据
    unsigned char recvBuf[1206] = {0};
    int recvLen;

    lidardata.angle_h.resize(10);
    lidardata.angle_v.resize(10);
    lidardata.distance.resize(10);
    lidardata.intensity.resize(10);

    std::vector<std::vector<float>> angle_h; //水平角度储存
    std::vector<std::vector<float>> angle_v; //垂直角度储存
    std::vector<std::vector<float>> distance;
    std::vector<std::vector<int>> intensity;

    angle_h.resize(10);
    angle_v.resize(10);
    distance.resize(10);
    intensity.resize(10);

    while (true)
    {
        //获取套接字接收内容
        recvLen = recvfrom(my_sock, recvBuf, sizeof(recvBuf), 0, (struct sockaddr *)&myClientAddr, &len);
        if (recvLen > 0)
        {

            for (int i = 0; i < 1192; i += 8)
            {

                if (recvBuf[i] == 0xff && recvBuf[i + 1] == 0xaa && recvBuf[i + 2] == 0xbb && recvBuf[i + 3] == 0xcc && recvBuf[i + 4] == 0xdd && recvBuf[i + 5] == 0xee)
                {
                    lidardata.distance.clear();
                    lidardata.intensity.clear();
                    lidardata.angle_h.clear();
                    lidardata.angle_v.clear();
                    if (lidardata.angle_h.empty())
                    {
                        lidardata.angle_h = angle_h;
                        lidardata.angle_v = angle_v;
                        lidardata.distance = distance;
                        lidardata.intensity = intensity;
                        // emit SendData(lidardata);

                        generatePclCould();
                    }
                    angle_h.clear();
                    angle_h.resize(10);
                    angle_v.clear();
                    angle_v.resize(10);
                    distance.clear();
                    distance.resize(10);
                    intensity.clear();
                    intensity.resize(10);
                }
                else
                {
                    //计算新的垂直角度和水平角度
                    int iTempAngle = recvBuf[i + 2];

                    int lineNum = iTempAngle >> 6; //光学通道只有(0，1，2，3）
                    float angle_h1 = (recvBuf[i + 1] + recvBuf[i] * 256) / 100.f;
                    angle_h1 = angle_h1 > 327.68 ? angle_h1 - 655.36 : angle_h1;

                    angle_h[lineNum].push_back(angle_h1);

                    int IsNum = (iTempAngle >> 5) & 0x01; // 符号位 0：正数 1：负数
                    int angle_v1;
                    if (1 == IsNum)
                    {
                        int iAngle_V = (recvBuf[i + 3] + recvBuf[i + 2] * 256);
                        angle_v1 = (iAngle_V | 0xc000);
                        angle_v1 = angle_v1 > 32768 ? angle_v1 - 65536 : angle_v1;
                    }
                    else
                    {
                        angle_v1 = (recvBuf[i + 3] + (iTempAngle % 64) * 256);
                    }
                    float angle_v2 = angle_v1 / 100.f;
                    angle_v[lineNum].push_back(angle_v2);
                    distance[lineNum].push_back((recvBuf[i + 6] + recvBuf[i + 5] * 256 + recvBuf[i + 4] * 65536) / 1000.f);
                    intensity[lineNum].push_back(recvBuf[i + 7]);
                }
            }
        }
    }
}

void LidarDrive128S1::generatePclCould()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    float cos_x_angle = cos(XAngle * PI / 180);
    float sin_x_angle = sin(XAngle * PI / 180);
    float cos_y_angle = cos(YAngle * PI / 180);
    float sin_y_angle = sin(YAngle * PI / 180);
    float cos_z_angle = cos(ZAngle * PI / 180);
    float sin_z_angle = sin(ZAngle * PI / 180);

    float g_fOffAngleValue[4] = {1.6, -0.4, 0.6, -1.4}; //振镜偏移值

    for (int i = 0; i < 4; i++)
    {
        float fOffAngleValue = 0;
        fOffAngleValue = g_fOffAngleValue[i];
        {
            for (size_t j = 0; j < lidardata.angle_h[i].size(); j++)
            {
                float fOffAngle = lidardata.angle_v[i][j] + 5.3;
                ;
                float fAngle_R0 = cos(30 * PI_ / 180) * cos(fOffAngleValue * PI_ / 180) * cos(fOffAngle * PI_ / 180) - sin(fOffAngle * PI_ / 180) * sin(fOffAngleValue * PI / 180);

                float fSinV_angle = 2 * fAngle_R0 * sin(fOffAngle * PI_ / 180) + sin(fOffAngleValue * PI_ / 180);
                float fCosV_angle = sqrt(1 - pow(fSinV_angle, 2));

                float fSinCite = (2 * fAngle_R0 * cos(fOffAngle * PI_ / 180) * sin(30 * PI_ / 180) - cos(fOffAngleValue * PI_ / 180) * sin(60 * PI_ / 180)) / fCosV_angle;
                float fCosCite = sqrt(1 - pow(fSinCite, 2));

                float fSinCite_H = sin(lidardata.angle_h[i][j] * PI_ / 180) * fCosCite + cos(lidardata.angle_h[i][j] * PI_ / 180) * fSinCite;
                float fCosCite_H = cos(lidardata.angle_h[i][j] * PI_ / 180) * fCosCite - sin(lidardata.angle_h[i][j] * PI_ / 180) * fSinCite;
                if (lidardata.distance[i][j] < 0.3)
                {
                    continue;
                }
                pcl::PointXYZRGB PointTemp1;
                PointTemp1.x = (lidardata.distance[i][j] * fCosV_angle * fSinCite_H);
                PointTemp1.y = (lidardata.distance[i][j] * fCosV_angle * fCosCite_H);
                PointTemp1.z = (lidardata.distance[i][j] * fSinV_angle);

                //坐标轴方向转换
                float transformed_x = PointTemp1.x * cos_z_angle * cos_y_angle + PointTemp1.y * (-sin_z_angle * cos_x_angle + cos_z_angle * sin_y_angle * sin_x_angle) + PointTemp1.z * (sin_z_angle * sin_x_angle + cos_z_angle * sin_y_angle * cos_x_angle);
                float transformed_y = PointTemp1.x * sin_z_angle * cos_y_angle + PointTemp1.y * (cos_z_angle * cos_x_angle + sin_z_angle * sin_y_angle * sin_x_angle) + PointTemp1.z * (sin_z_angle * sin_y_angle * cos_x_angle - cos_z_angle * sin_x_angle);
                float transformed_z = (-sin_y_angle * PointTemp1.x + cos_y_angle * sin_x_angle * PointTemp1.y + cos_y_angle * cos_x_angle * PointTemp1.z);

                //点转换
                PointTemp1.x = transformed_x + Base_X;
                PointTemp1.y = transformed_y + Base_Y;
                PointTemp1.z = transformed_z + Base_Z;

                if (lidardata.intensity[i][j] <= 63)
                {
                    PointTemp1.r = 0;
                    PointTemp1.g = 254 - 4 * lidardata.intensity[i][j];
                    PointTemp1.b = 255;
                }
                else if (lidardata.intensity[i][j] > 63 && lidardata.intensity[i][j] <= 127)
                {
                    PointTemp1.r = 0;
                    PointTemp1.g = 4 * lidardata.intensity[i][j] - 254;
                    PointTemp1.b = 510 - 4 * lidardata.intensity[i][j];
                }
                else if (lidardata.intensity[i][j] > 127 && lidardata.intensity[i][j] <= 191)
                {
                    PointTemp1.r = 4 * lidardata.intensity[i][j] - 510;
                    PointTemp1.g = 255;
                    PointTemp1.b = 0;
                }
                else if (lidardata.intensity[i][j] > 191 && lidardata.intensity[i][j] <= 255)
                {
                    PointTemp1.r = 255;
                    PointTemp1.g = 1022 - 4 * lidardata.intensity[i][j];
                    PointTemp1.b = 0;
                }

                tCloud->points.push_back(PointTemp1);
            }
        }
    }

    std::lock_guard<std::mutex> lck(m_lock);
    recv_lidar_queue_.push(tCloud);


    if (recv_lidar_queue_.size() > 3)
    {
        recv_lidar_queue_.pop();

        qWarning()<<"警告：接收到lidar驱动队列超过3！";
    }
    // SendData(tCloud);
}
