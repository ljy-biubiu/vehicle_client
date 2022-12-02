#include "ptz.h"
#include "qdebug.h"

#include <functional>
#include <cstdlib>
#include <iostream>

#include "qdebug.h"

int i=0;


PTZ::PTZ()
{
    PTZPreset_num =21;
    currentnum =0;
    isRun = true;

}


void PTZ::globalParamsCallback(TotalParams msg,bool save_flag)
{
    this->cam_IP =  QString(msg.cameraBase.ip.c_str()) ;
    this->cam_ID = QString(msg.cameraBase.name.c_str()) ;
    this->cam_Pass = QString(msg.cameraBase.code.c_str()) ;

}


void PTZ::start()
{
    std::thread dowork_thread(std::bind(&PTZ::run, this));
    dowork_thread.detach();
}


void PTZ::PTZ_init(HWND hwnd_)
{
    qDebug()<<"cam_IP : "<< cam_IP;
    qDebug()<<"cam_ID : "<< cam_ID;
    qDebug()<<"cam_Pass : "<< cam_Pass;

    NET_DVR_Init();
    NET_DVR_SetConnectTime(2000, 1);
    NET_DVR_SetReconnect(120000, true);
    NET_DVR_DEVICEINFO_V30 struDeviceInfo;

    char* camera_ip ;
    char* camera_userid;
    char* camera_pass ;
    QByteArray ba1 = cam_IP.toLatin1(); // must
    QByteArray ba2 = cam_ID.toLatin1(); // must
    QByteArray ba3 = cam_Pass.toLatin1(); // must
    camera_ip = ba1.data();
    camera_userid = ba2.data();
    camera_pass = ba3.data();

    lUserID = NET_DVR_Login_V30(camera_ip, 8000,camera_userid, camera_pass, &struDeviceInfo);

    if (lUserID < 0)
    {
        error = QString("Login failed, error code: %d\n").arg(NET_DVR_GetLastError());
        qDebug("Login failed, error code: %d\n", NET_DVR_GetLastError());
        NET_DVR_Cleanup();
        return ;
    }
    else
    {
        mas = "PTZ connect success";
    }
    NET_DVR_PTZPreset_Other(lUserID,1,GOTO_PRESET,6);//回到预置点6
    PTZ_Cruise_Open(); //开始巡航



    NET_DVR_PREVIEWINFO struPlayInfo;
    struPlayInfo.hPlayWnd = hwnd_;
    struPlayInfo.lChannel     = 1;       //预览通道号
    struPlayInfo.dwStreamType = 0;       //0-主码流，1-子码流，2-码流3，3-码流4，以此类推
    struPlayInfo.dwLinkMode   = 0;       //0- TCP方式，1- UDP方式，2- 多播方式，3- RTP方式，4-RTP/RTSP，5-RSTP/HTTP
    struPlayInfo.bBlocked     = 0;       //0- 非阻塞取流，1- 阻塞取流

    static LONG lRealPlayHandle = NET_DVR_RealPlay_V40(this->lUserID, &struPlayInfo, NULL, NULL);
    //    QString  message1=QString("lrealpal %1").arg(lRealPlayHandle);
    //    QString  message2=QString("error = %1").arg(NET_DVR_GetLastError());

    if (lRealPlayHandle < 0)
    {
        NET_DVR_Logout(this->lUserID);
        NET_DVR_Cleanup();
    }

    this->start();

}


/************************************************************************************/

//void PTZ::OnButtonUpChanged()
//{
//    PTZ_Up(7,0);
//}

//void PTZ::OffButtonUpChanged()
//{
//    PTZ_Up(7,1);
//}

//void PTZ::OnButtonDownChanged()
//{
//    PTZ_Down(7,0);
//}
//void PTZ::OffButtonDownChanged()
//{
//    PTZ_Down(7,1);
//}

//void PTZ::OnButtonLeftChanged()
//{
//    PTZ_Left(7,0);
//}
//void PTZ::OffButtonLeftChanged()
//{
//    PTZ_Left(7,1);
//}
//void PTZ::OnButtonRightChanged()
//{
//    PTZ_Right(7,0);
//}
//void PTZ::OffButtonRightChanged()
//{
//    PTZ_Right(7,1);
//}

////调焦
//void PTZ::OnButtonInChanged()
//{
//    PTZ_Zoom_In(1,0);
//}
//void PTZ::OffButtonInChanged()
//{
//    PTZ_Zoom_In(1,1);
//}

//void PTZ::OnButtonOutChanged()
//{
//    PTZ_Zoom_Out(1,0);
//}
//void PTZ::OffButtonOutChanged()
//{
//    PTZ_Zoom_Out(1,1);
//}

////聚焦
//void PTZ::OnButtonNearChanged()
//{
//    PTZ_Focus_Near(1,0);
//}
//void PTZ::OffButtonNearChanged()
//{
//    PTZ_Focus_Near(1,1);
//}
//void PTZ::OnButtonFarChanged()
//{
//    PTZ_Focus_Far(1,0);
//}
//void PTZ::OffButtonFarChanged()
//{
//    PTZ_Focus_Far(1,1);
//}

////光圈
//void PTZ::OnButtonOpenChanged()
//{
//    PTZ_Iris_Open(1,0);
//}
//void PTZ::OffButtonOpenChanged()
//{
//    PTZ_Iris_Open(1,1);
//}

//void PTZ::OnButtonCloseChanged()
//{
//    PTZ_Iris_Close(1,0);
//}
//void PTZ::OffButtonCloseChanged()
//{
//    PTZ_Iris_Close(1,1);
//}


///************************************************************************/
//void PTZ::PTZ_Up(int speed , int flag)/*上移*/
//{
//    NET_DVR_PTZControlWithSpeed_Other(lUserID , 1 , TILT_UP , flag , speed);
//}
//void PTZ::PTZ_Down(int speed , int flag)/*下移*/
//{
//    NET_DVR_PTZControlWithSpeed_Other(lUserID , 1 , TILT_DOWN , flag , speed);
//}
//void PTZ::PTZ_Left(int speed , int flag)/*左移*/
//{
//    NET_DVR_PTZControlWithSpeed_Other(lUserID , 1 , PAN_LEFT , flag , speed);
//}
//void PTZ::PTZ_Right(int speed , int flag)/*右移*/
//{
//    NET_DVR_PTZControlWithSpeed_Other(lUserID , 1 , PAN_RIGHT , flag , speed);
//}

////调焦
//void PTZ::PTZ_Zoom_In(int speed , int flag)/*焦距变大*/
//{
//    NET_DVR_PTZControlWithSpeed_Other(lUserID , 1 , ZOOM_IN , flag , speed);
//}
//void PTZ::PTZ_Zoom_Out(int speed , int flag)/*焦距变小*/
//{
//    NET_DVR_PTZControlWithSpeed_Other(lUserID , 1 , ZOOM_OUT , flag , speed);
//}

////聚焦
//void PTZ::PTZ_Focus_Near(int speed , int flag)/*焦点前调*/
//{
//    NET_DVR_PTZControlWithSpeed_Other(lUserID , 1 , FOCUS_NEAR , flag , speed);
//}
//void PTZ::PTZ_Focus_Far(int speed , int flag)/*焦点后调*/
//{
//    NET_DVR_PTZControlWithSpeed_Other(lUserID , 1 , FOCUS_FAR , flag , speed);
//}
////光圈
//void PTZ::PTZ_Iris_Open(int speed , int flag)/*光圈扩大*/
//{
//    NET_DVR_PTZControlWithSpeed_Other(lUserID , 1 , IRIS_OPEN , flag , speed);
//}
//void PTZ::PTZ_Iris_Close(int speed , int flag)/*光圈缩小*/
//{
//    NET_DVR_PTZControlWithSpeed_Other(lUserID , 1 , IRIS_CLOSE , flag , speed);
//}

//巡航
void PTZ::PTZ_Cruise_Open()/*巡航开始*/
{
    NET_DVR_PTZCruise_Other(lUserID , 1, RUN_SEQ , 1,0,10 );
}
void PTZ::PTZ_Cruise_Close()/*巡航结束*/
{
    NET_DVR_PTZCruise_Other(lUserID , 1, STOP_SEQ , 1,0,10 );
}

/*****************************************************************************************/


bool PTZ::PTZ_GOTO_Preset(int num)
{
    return NET_DVR_PTZPreset(lUserID,GOTO_PRESET,num);
}


void PTZ::FollowTarget(float Angle,float Dist)//追踪目标
{
    // 根据障碍物中心点的水平角度设置相机转动到相应的预置点
    // 预置点的序号所代表球机位置可以通过浏览器登录海康相机ip设置
    // 相机转动 预置点6 时对应雷达视野正前方
    // Angle是相对于雷达Y轴偏转计算的;
    // 向左偏转的角度判断
    if (Angle >= -10 && Angle < 10)
        num = 6;
    else if (Angle >= -30 && Angle < -10 )
        num = 5;
    else if (Angle >= -50 && Angle < -30)
        num = 4;
    else if (Angle >= -70 && Angle < -50) // 雷达水平视野范围为120度，所以向左向右偏转60度就行，预置点1、2无需用到
        num = 3;

    // 向右偏转
    else if (Angle >= 10 && Angle < 30)
        num = 7;
    else if (Angle >= 30 && Angle < 50)
        num = 8;
    else if (Angle >= 50 && Angle < 70)
        num = 9;

    if(num!=currentnum)
    {
        if(!PTZ_GOTO_Preset(num))
        {
            printf("PTZ_GOTO_Preset failed, error code: %d\n", NET_DVR_GetLastError());
        }else
        {
            qDebug()<<"转动预置点: " << num << "Success";
        }
        getPara();
        currentnum = num;
    }

}

void PTZ::run()
{
    while(1)
    {
        if(isRun)
        {
            FollowTarget(war_horizontal,war_dist);
        }

        usleep(100*1000);
    }
}


void PTZ::getPara()
{
    int iRet;
    int P=0;
    int T=0;
    NET_DVR_PTZPOS  PTZParams;//球机参数
    DWORD paramsLen;
    iRet = NET_DVR_GetDVRConfig(lUserID , NET_DVR_GET_PTZPOS, 1 , &PTZParams , sizeof(NET_DVR_PTZPOS) , &paramsLen);
    if(iRet == 0)
    {
        qDebug("2error:%d\n" , iRet);
    }
    P=ToHex(PTZParams.wPanPos);
    T=ToHex(PTZParams.wTiltPos);
    //qDebug("PTZParamsw.wpanpos=  %d,TiltPos=%d\n"  ,PTZParams.wPanPos,(PTZParams.wTiltPos));//wPanPos水平参数,,wTiltPos垂直
}


/**********十进制转换为十六进制************/

int PTZ::ToHex(int n)
{
    int i=0,m=0,r[100];
    int p=0;
    while(n>0){
        r[i++]=n%16;
        n=n/16;
    }
    for(i=i-1;i>=0;i--){
        m=r[i];
        p=p+m*pow(10,i);
    }
    return p;
}

