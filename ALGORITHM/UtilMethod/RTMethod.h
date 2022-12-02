#pragma once

#include <iostream>
#include <string>
#include <cstring>
#include <iomanip>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;

namespace rtmethod {
    struct RTParams {
        double x;//x方向 配准平移距离，米
        double y;
        double z;
        double roll;// 绕x轴 配准旋转角度，度，右手定则
        double pitch;
        double heading;

        RTParams() {
            x = 0;
            y = 0;
            z = 0;
            roll = 0;
            pitch = 0;
            heading = 0;
        }

        RTParams(double _roll, double _pitch, double _heading, double _x, double _y, double _z) {
            x = _x;
            y = _y;
            z = _z;
            roll = _roll;
            pitch = _pitch;
            heading = _heading;
        }
    };

    struct RPYAngles {
        double xRoll;
        double yPitch;
        double zYaw;
    };

    class RTMethod {
    public:
        RTMethod();

        ~RTMethod();

        /**
         * 公开成员方法
         */
    public:
        void printRTStruct(RTParams rt);

        void printRTMat(Eigen::Matrix4d rtMat);

        void rtMat2RTStruct_zyx(Eigen::Matrix4d rtMat, RTParams &rtStruct);

        void rtStruct2RTMat_zyx(RTParams rtStruct, Eigen::Matrix4d &rtMat);

        void rtMat2RTStruct_yxz(Eigen::Matrix4d rtMat, RTParams &rtStruct);

        void rtStruct2RTMat_yxz(RTParams rtStruct, Eigen::Matrix4d &rtMat);

        /**
         * 私有成员变量 、getter、setter
         */
    private:
        RTParams initStruct_zyx;
    public:
        const RTParams &getInitStructZyx() const;

        void setInitStructZyx(const RTParams &initStructZyx);

    private:
        Eigen::Matrix4d initMat_zyx;
    public:
        const Eigen::Matrix4d &getInitMatZyx() const;

        void setInitMatZyx(const Eigen::Matrix4d &_initMatZyx);

    private:
        RTParams rtStruct_zyx;
    public:
        const RTParams &getRtStructZyx() const;

        void setRtStructZyx(const RTParams &_rtStruct);

    private:
        Eigen::Matrix4d rtMat_zyx;
    public:
        const Eigen::Matrix4d &getRtMatZyx() const;

        void setRtMatZyx(const Eigen::Matrix4d &rtMatZyx);

    private:
        RTParams rtStruct_yxz;
    public:
        const RTParams &getRtStructYxz() const;

        void setRtStructYxz(const RTParams &rtStructYxz);

    private:
        Eigen::Matrix4d rtMat_yxz;
    public:
        const Eigen::Matrix4d &getRtMatYxz() const;

        void setRtMatYxz(const Eigen::Matrix4d &rtMatYxz);

        /**
         * 私有变量
         */
    private:
        Eigen::Matrix<double, 3, 3> Rx_roll;
        Eigen::Matrix<double, 3, 3> Ry_pitch;
        Eigen::Matrix<double, 3, 3> Rz_yaw;
        Eigen::Matrix<double, 3, 3> R_zyx;
        Eigen::Matrix<double, 3, 3> R_yxz;
        Eigen::Matrix<double, 3, 3> R_xyz;

        /**
         * 私有成员方法
         */
    private:
        void setRxRyRz(RTParams rtStruct);

        void rotMatrixToAngles_yxz(Eigen::Matrix3d rotationMatrix, RPYAngles &rpyAngles);
    };
}


