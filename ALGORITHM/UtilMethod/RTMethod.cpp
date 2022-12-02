

#include "RTMethod.h"

namespace rtmethod {
    RTMethod::RTMethod() {
        rtStruct_zyx = {};
        rtMat_zyx = Eigen::Matrix4d::Identity();

        rtStruct_yxz = {};
        rtMat_yxz = Eigen::Matrix4d::Identity();

        Rx_roll = Eigen::Matrix<double, 3, 3>::Identity();
        Ry_pitch = Eigen::Matrix<double, 3, 3>::Identity();
        Rz_yaw = Eigen::Matrix<double, 3, 3>::Identity();
        R_zyx = Eigen::Matrix<double, 3, 3>::Identity();
        R_yxz = Eigen::Matrix<double, 3, 3>::Identity();
        R_xyz = Eigen::Matrix<double, 3, 3>::Identity();
    }

    RTMethod::~RTMethod() {

    }

    void RTMethod::rtMat2RTStruct_zyx(Eigen::Matrix4d rtMat, RTParams &rtStruct) {
        this->printRTMat(rtMat);
        Eigen::Matrix3d outMat3d = rtMat.block<3, 3>(0, 0);
        Eigen::Vector3d eulerAngles = outMat3d.eulerAngles(2, 1, 0);

        rtStruct.roll = eulerAngles(2) * 180.0 / M_PI;
        rtStruct.pitch = eulerAngles(1) * 180.0 / M_PI;
        rtStruct.heading = eulerAngles(0) * 180.0 / M_PI;
        /*Eigen::Vector3d eulerAngles = outMat3d.eulerAngles(1, 0, 2);
        rtStruct.roll = eulerAngles(1) * 180.0 / M_PI;
        rtStruct.pitch = eulerAngles(0) * 180.0 / M_PI;
        rtStruct.heading = eulerAngles(2) * 180.0 / M_PI;*/
        rtStruct.x = rtMat(0, 3);
        rtStruct.y = rtMat(1, 3);
        rtStruct.z = rtMat(2, 3);
        this->printRTStruct(rtStruct);
    }

    void RTMethod::rtStruct2RTMat_zyx(RTParams rtStruct, Eigen::Matrix4d &rtMat) {
        rtMat = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d mat3d1 = Eigen::Matrix3d::Identity();
        // 欧拉角转换为旋转向量，分别计算三个分量，然后将其相乘起来
        Eigen::AngleAxisd roll_x(rtStruct.roll * M_PI / 180, Eigen::Vector3d(1, 0, 0));
        Eigen::AngleAxisd pitch_y(rtStruct.pitch * M_PI / 180, Eigen::Vector3d(0, 1, 0));
        Eigen::AngleAxisd yaw_z(rtStruct.heading * M_PI / 180, Eigen::Vector3d(0, 0, 1));
        /*cout << "roll_x = " << endl << roll_x.matrix() << endl;
        cout << "pitch_y = " << endl << pitch_y.matrix() << endl;
        cout << "yaw_z = " << endl << yaw_z.matrix() << endl;*/
        Eigen::AngleAxisd rotationVectorX;
        rotationVectorX = yaw_z * pitch_y * roll_x;
        //rotationVectorX = pitch_y * roll_x * yaw_z;
        mat3d1 = rotationVectorX.matrix();

        rtMat.block<3, 3>(0, 0) = mat3d1;
        rtMat(0, 3) = rtStruct.x;
        rtMat(1, 3) = rtStruct.y;
        rtMat(2, 3) = rtStruct.z;
        cout << "------ RT_zyx : ------" << endl;
        this->printRTMat(rtMat);
    }

    void RTMethod::printRTStruct(RTParams rt) {
        cout << setprecision(8);
        cout << "[ roll = " << rt.roll
             << " ] , [ pitch = " << rt.pitch
             << " ] , [ heading = " << rt.heading
             << " ] , [ x = " << rt.x
             << " ] , [ y = " << rt.y
             << " ] , [ z = " << rt.z
             << " ] " << endl;
    }

    void RTMethod::printRTMat(Eigen::Matrix4d rtMat) {
        //cout << "------ rtMat : ------" << endl;
        cout << rtMat << endl;
        //cout << "---------------------" << endl;
    }

    const RTParams &RTMethod::getRtStructZyx() const {
        return rtStruct_zyx;
    }

    void RTMethod::setRtStructZyx(const RTParams &_rtStruct) {
        RTMethod::rtStruct_zyx = _rtStruct;
        this->printRTStruct(rtStruct_zyx);
        this->setRxRyRz(rtStruct_zyx);
        this->rtStruct2RTMat_zyx(rtStruct_zyx, rtMat_zyx);
        rtMat_zyx = initMat_zyx * rtMat_zyx;
        this->printRTMat(rtMat_zyx);
        cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
        this->rtMat2RTStruct_yxz(rtMat_zyx, rtStruct_yxz);
        this->rtStruct2RTMat_yxz(rtStruct_yxz, rtMat_yxz);
    }

    void RTMethod::setRxRyRz(RTParams rtStruct) {
        /*Rx_roll = Eigen::Matrix<double, 3, 3>::Identity();
        Ry_pitch = Eigen::Matrix<double, 3, 3>::Identity();
        Rz_yaw = Eigen::Matrix<double, 3, 3>::Identity();*/

        Rx_roll(1, 1) = cos(rtStruct.roll * M_PI / 180);
        Rx_roll(1, 2) = -sin(rtStruct.roll * M_PI / 180);
        Rx_roll(2, 1) = sin(rtStruct.roll * M_PI / 180);
        Rx_roll(2, 2) = cos(rtStruct.roll * M_PI / 180);

        Ry_pitch(0, 0) = cos(rtStruct.pitch * M_PI / 180);
        Ry_pitch(0, 2) = sin(rtStruct.pitch * M_PI / 180);
        Ry_pitch(2, 0) = -sin(rtStruct.pitch * M_PI / 180);
        Ry_pitch(2, 2) = cos(rtStruct.pitch * M_PI / 180);

        Rz_yaw(0, 0) = cos(rtStruct.heading * M_PI / 180);
        Rz_yaw(0, 1) = -sin(rtStruct.heading * M_PI / 180);
        Rz_yaw(1, 0) = sin(rtStruct.heading * M_PI / 180);
        Rz_yaw(1, 1) = cos(rtStruct.heading * M_PI / 180);
        /*cout << "Rx_roll = " << endl << Rx_roll << endl;
        cout << "Ry_pitch = " << endl << Ry_pitch << endl;
        cout << "Rz_yaw = " << endl << Rz_yaw << endl;*/

        /*R_zyx = Rz_yaw * Ry_pitch * Rx_roll;
        R_yxz = Ry_pitch * Rx_roll * Rz_yaw;
        R_xyz = Rx_roll * Ry_pitch * Rz_yaw;

        cout << "------ R_zyx : ------" << endl << R_zyx << endl;
        cout << "------ R_yxz : ------" << endl << R_yxz << endl;
        cout << "------ R_xyz : ------" << endl << R_xyz << endl;*/
    }

    void RTMethod::rtMat2RTStruct_yxz(Eigen::Matrix4d rtMat, RTParams &rtStruct) {
        Eigen::Matrix3d outMat3d = rtMat.block<3, 3>(0, 0);
        /*Eigen::Vector3d eulerAngles = outMat3d.eulerAngles(2, 1, 0);
        rtStruct.roll = eulerAngles(2) * 180.0 / M_PI;
        rtStruct.pitch = eulerAngles(1) * 180.0 / M_PI;
        rtStruct.heading = eulerAngles(0) * 180.0 / M_PI;*/
        RPYAngles rpyAngles{};
        this->rotMatrixToAngles_yxz(outMat3d, rpyAngles);
        rtStruct.roll = rpyAngles.xRoll * 180.0 / M_PI;
        rtStruct.pitch = rpyAngles.yPitch * 180.0 / M_PI;
        rtStruct.heading = rpyAngles.zYaw * 180.0 / M_PI;
        /*Eigen::Vector3d eulerAngles = outMat3d.eulerAngles(1, 0, 2);
        rtStruct.roll = eulerAngles(1) * 180.0 / M_PI;
        rtStruct.pitch = eulerAngles(0) * 180.0 / M_PI;
        rtStruct.heading = eulerAngles(2) * 180.0 / M_PI;*/
        rtStruct.x = rtMat(0, 3);
        rtStruct.y = rtMat(1, 3);
        rtStruct.z = rtMat(2, 3);
//        this->printRTStruct(rtStruct);
    }

    void RTMethod::rtStruct2RTMat_yxz(RTParams rtStruct, Eigen::Matrix4d &rtMat) {
        rtMat = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d mat3d1 = Eigen::Matrix3d::Identity();
        // 欧拉角转换为旋转向量，分别计算三个分量，然后将其相乘起来
        Eigen::AngleAxisd roll_x(rtStruct.roll * M_PI / 180, Eigen::Vector3d(1, 0, 0));
        Eigen::AngleAxisd pitch_y(rtStruct.pitch * M_PI / 180, Eigen::Vector3d(0, 1, 0));
        Eigen::AngleAxisd yaw_z(rtStruct.heading * M_PI / 180, Eigen::Vector3d(0, 0, 1));
        /*cout << "roll_x = " << endl << roll_x.angle() * 180 / M_PI << endl;
        cout << "pitch_y = " << endl << pitch_y.angle() * 180 / M_PI << endl;
        cout << "yaw_z = " << endl << yaw_z.angle() * 180 / M_PI << endl;*/
        Eigen::AngleAxisd rotationVectorX;
        rotationVectorX = pitch_y * roll_x * yaw_z;
        mat3d1 = rotationVectorX.matrix();

        rtMat.block<3, 3>(0, 0) = mat3d1;
        rtMat(0, 3) = rtStruct.x;
        rtMat(1, 3) = rtStruct.y;
        rtMat(2, 3) = rtStruct.z;
        //cout << "------ RT_yxz : ------" << endl;
        //this->printRTMat(rtMat);
    }

    const RTParams &RTMethod::getInitStructZyx() const {
        return initStruct_zyx;
    }

    void RTMethod::setInitStructZyx(const RTParams &initStructZyx) {
        initStruct_zyx = initStructZyx;
        this->rtStruct2RTMat_zyx(initStruct_zyx, initMat_zyx);
    }

    const Eigen::Matrix4d &RTMethod::getInitMatZyx() const {
        return initMat_zyx;
    }

    void RTMethod::setInitMatZyx(const Eigen::Matrix4d &_initMatZyx) {
        initMat_zyx = _initMatZyx;
    }

    const Eigen::Matrix4d &RTMethod::getRtMatZyx() const {
        return rtMat_zyx;
    }

    void RTMethod::setRtMatZyx(const Eigen::Matrix4d &rtMatZyx) {
        rtMat_zyx = rtMatZyx;
        this->rtMat2RTStruct_zyx(rtMat_zyx, rtStruct_zyx);
    }

    const Eigen::Matrix4d &RTMethod::getRtMatYxz() const {
        return rtMat_yxz;
    }

    void RTMethod::setRtMatYxz(const Eigen::Matrix4d &rtMatYxz) {
        rtMat_yxz = rtMatYxz;
    }

    const RTParams &RTMethod::getRtStructYxz() const {
        return rtStruct_yxz;
    }

    void RTMethod::setRtStructYxz(const RTParams &rtStructYxz) {
        rtStruct_yxz = rtStructYxz;
    }

    void RTMethod::rotMatrixToAngles_yxz(Eigen::Matrix3d rotationMatrix, RPYAngles &rpyAngles) {
        double r00 = rotationMatrix(0, 0), r01 = rotationMatrix(0, 1), r02 = rotationMatrix(0, 2);
        double r10 = rotationMatrix(1, 0), r11 = rotationMatrix(1, 1), r12 = rotationMatrix(1, 2);
        double r20 = rotationMatrix(2, 0), r21 = rotationMatrix(2, 1), r22 = rotationMatrix(2, 2);
        double cosRoll = sqrt(r10 * r10 + r11 * r11);

        rpyAngles.zYaw = atan2(r10, r11);
        rpyAngles.yPitch = atan2(r02, r22);
        rpyAngles.xRoll = atan2(-r12, cosRoll);

    }
}

