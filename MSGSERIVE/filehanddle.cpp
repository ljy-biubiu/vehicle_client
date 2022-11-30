#include "filehanddle.h"
#include "qdebug.h"

FileHanddle::FileHanddle()
{

}

void FileHanddle::init(TotalParams &total_params)
{
    read_file_path = "/home/ljy/my_paramss.json";
    readDataLocal(total_params);
}


int FileHanddle::writeDataLocal(TotalParams &total_params)
{

    //根节点
    Json::Value root;
    //子节点
    Json::Value ComunicationData;
    //子节点属性
    ComunicationData["angle"] = Json::Value(total_params.protocol_data.read_data.angle);
    ComunicationData["amplitude_position"] = Json::Value(total_params.protocol_data.read_data.amplitude_position);
    ComunicationData["height"] = Json::Value(total_params.protocol_data.read_data.height);
    ComunicationData["safe_distance"] = Json::Value(total_params.protocol_data.read_data.safe_distance);
    ComunicationData["resolution"] = Json::Value(total_params.protocol_data.read_data.resolution);
    ComunicationData["build_map_signal"] = Json::Value(total_params.protocol_data.read_data.build_map_signal);
    ComunicationData["comnunication_flag"] = Json::Value(total_params.protocol_data.read_data.comnunication_flag);
    ComunicationData["device_status"] = Json::Value(total_params.protocol_data.read_data.device_status);
    ComunicationData["material_area_x"] = Json::Value(total_params.protocol_data.read_data.material_area_x);
    ComunicationData["material_area_y"] = Json::Value(total_params.protocol_data.read_data.material_area_y);
    ComunicationData["meterial_area_length"] = Json::Value(total_params.protocol_data.read_data.meterial_area_length);
    ComunicationData["meterial_area_width"] = Json::Value(total_params.protocol_data.read_data.meterial_area_width);
    ComunicationData["technology"] = Json::Value(total_params.protocol_data.read_data.technology);
    //子节点挂到根节点上
    root["ComunicationData"] = Json::Value(ComunicationData);

    //子节点
    Json::Value LidarPreset;
    //子节点属性
    LidarPreset["paintarea_Radius"] = Json::Value(total_params.lidarPreset.paintarea_Radius);
    LidarPreset["XAngle"] = Json::Value(total_params.lidarPreset.XAngle);
    LidarPreset["YAngle"] = Json::Value(total_params.lidarPreset.YAngle);
    LidarPreset["ZAngle"] = Json::Value(total_params.lidarPreset.ZAngle);
    LidarPreset["Base_X"] = Json::Value(total_params.lidarPreset.Base_X);
    LidarPreset["Base_Y"] = Json::Value(total_params.lidarPreset.Base_Y);
    LidarPreset["Base_Z"] = Json::Value(total_params.lidarPreset.Base_Z);
    //子节点挂到根节点上
    root["LidarPreset"] = Json::Value(LidarPreset);

    //子节点
    Json::Value LidarCluster;
    //子节点属性
    LidarCluster["ClusterTolerance"] = Json::Value(total_params.lidarCluster.ClusterTolerance);
    LidarCluster["MinClusterSize"] = Json::Value(total_params.lidarCluster.MinClusterSize);
    LidarCluster["MaxClusterSize"] = Json::Value(total_params.lidarCluster.MaxClusterSize);
    LidarCluster["Resolution"] = Json::Value(total_params.lidarCluster.Resolution);
    LidarCluster["Difference_threshold"] = Json::Value(total_params.lidarCluster.Difference_threshold);
    LidarCluster["Kvalue"] = Json::Value(total_params.lidarCluster.Kvalue);
    LidarCluster["EuclideanDistance"] = Json::Value(total_params.lidarCluster.EuclideanDistance);
    //子节点挂到根节点上
    root["LidarCluster"] = Json::Value(LidarCluster);

    //子节点
    Json::Value LidarBase;
    //子节点属性
    LidarBase["lidarPort"] = Json::Value(total_params.lidarBase.lidarPort);
    LidarBase["LidarAddress"] = Json::Value(total_params.lidarBase.LidarAddress);
    LidarBase["ServerPort"] = Json::Value(total_params.lidarBase.ServerPort);
    LidarBase["ServerAddress"] = Json::Value(total_params.lidarBase.ServerAddress);
    LidarBase["DataAddress"] = Json::Value(total_params.lidarBase.DataAddress);
    LidarBase["DeviceId"] = Json::Value(total_params.lidarBase.DeviceId);
    LidarBase["LidarModel"] = Json::Value(total_params.lidarBase.LidarModel);
    //子节点挂到根节点上
    root["LidarBase"] = Json::Value(LidarBase);


    //子节点
    Json::Value LidarCameraBase;
    //子节点属性
    LidarCameraBase["Pos_X"] = Json::Value(total_params.lidarCameraBase.Pos_X);
    LidarCameraBase["Pos_Y"] = Json::Value(total_params.lidarCameraBase.Pos_Y);
    LidarCameraBase["Pos_Z"] = Json::Value(total_params.lidarCameraBase.Pos_Z);
    LidarCameraBase["View_X"] = Json::Value(total_params.lidarCameraBase.View_X);
    LidarCameraBase["View_Y"] = Json::Value(total_params.lidarCameraBase.View_Y);
    LidarCameraBase["View_Z"] = Json::Value(total_params.lidarCameraBase.View_Z);
    LidarCameraBase["Up_X"] = Json::Value(total_params.lidarCameraBase.Up_X);
    LidarCameraBase["Up_Y"] = Json::Value(total_params.lidarCameraBase.Up_Y);
    LidarCameraBase["Up_Z"] = Json::Value(total_params.lidarCameraBase.Up_Z);
    //子节点挂到根节点上
    root["LidarCameraBase"] = Json::Value(LidarCameraBase);


    //子节点
    Json::Value CameraBase;
    //子节点属性
    CameraBase["ip"] = Json::Value(total_params.cameraBase.ip);
    CameraBase["code"] = Json::Value(total_params.cameraBase.code);
    CameraBase["name"] = Json::Value(total_params.cameraBase.name);
    //子节点挂到根节点上
    root["CameraBase"] = Json::Value(CameraBase);

    //子节点
    Json::Value LowerMachineBase;
    //子节点属性
    LowerMachineBase["ip"] = Json::Value(total_params.lowerMachineBase.ip);
    LowerMachineBase["port"] = Json::Value(total_params.lowerMachineBase.port);
    //子节点挂到根节点上
    root["LowerMachineBase"] = Json::Value(LowerMachineBase);

    std::ofstream ofs;
    ofs.open(read_file_path);
    if(!ofs.is_open())
    {
        qCritical()<<"写入json文件失败";
        return false;
    }
    ofs<<root.toStyledString();
    ofs.close();
    return  true;
}


void FileHanddle::readDataLocal(TotalParams &total_params)
{
    Json::Reader reader;
    Json::Value json_value;
    std::ifstream ifs(read_file_path, std::ios::binary);

    if(!ifs.is_open())
    {
        qCritical()<<"读取参数文件失败，创建参数文件";
        writeDataLocal(total_params);
        ifs.open(read_file_path);
    }


    if(reader.parse(ifs, json_value))
    {
        //PLC协议设置内容
        total_params.protocol_data.read_data.angle= json_value["ComunicationData"]["angle"].asFloat();
        total_params.protocol_data.read_data.amplitude_position= json_value["ComunicationData"]["amplitude_position"].asFloat();
        total_params.protocol_data.read_data.height= json_value["ComunicationData"]["height"].asFloat();
        total_params.protocol_data.read_data.safe_distance= json_value["ComunicationData"]["safe_distance"].asFloat();
        total_params.protocol_data.read_data.resolution= json_value["ComunicationData"]["resolution"].asFloat();
        total_params.protocol_data.read_data.build_map_signal= json_value["ComunicationData"]["build_map_signal"].asFloat();
        total_params.protocol_data.read_data.comnunication_flag= json_value["ComunicationData"]["comnunication_flag"].asFloat();
        total_params.protocol_data.read_data.device_status= json_value["ComunicationData"]["device_status"].asFloat();
        total_params.protocol_data.read_data.material_area_x= json_value["ComunicationData"]["material_area_x"].asFloat();
        total_params.protocol_data.read_data.material_area_y= json_value["ComunicationData"]["material_area_y"].asFloat();
        total_params.protocol_data.read_data.meterial_area_length= json_value["ComunicationData"]["meterial_area_length"].asFloat();
        total_params.protocol_data.read_data.meterial_area_width= json_value["ComunicationData"]["meterial_area_width"].asFloat();
        total_params.protocol_data.read_data.technology= json_value["ComunicationData"]["technology"].asFloat();

        //
        total_params.lidarPreset.paintarea_Radius = json_value["LidarPreset"]["paintarea_Radius"].asDouble();
        total_params.lidarPreset.XAngle = json_value["LidarPreset"]["XAngle"].asDouble();
        total_params.lidarPreset.YAngle = json_value["LidarPreset"]["YAngle"].asDouble();
        total_params.lidarPreset.ZAngle = json_value["LidarPreset"]["ZAngle"].asDouble();
        total_params.lidarPreset.Base_X = json_value["LidarPreset"]["Base_X"].asDouble();
        total_params.lidarPreset.Base_Y = json_value["LidarPreset"]["Base_Y"].asDouble();
        total_params.lidarPreset.Base_Z = json_value["LidarPreset"]["Base_Z"].asDouble();

        //
        total_params.lidarCluster.ClusterTolerance = json_value["LidarCluster"]["ClusterTolerance"].asDouble();
        total_params.lidarCluster.MinClusterSize = json_value["LidarCluster"]["MinClusterSize"].asDouble();
        total_params.lidarCluster.MaxClusterSize = json_value["LidarCluster"]["MaxClusterSize"].asDouble();
        total_params.lidarCluster.Resolution = json_value["LidarCluster"]["Resolution"].asDouble();
        total_params.lidarCluster.Difference_threshold = json_value["LidarCluster"]["Difference_threshold"].asDouble();
        total_params.lidarCluster.Kvalue = json_value["LidarCluster"]["Kvalue"].asDouble();
        total_params.lidarCluster.EuclideanDistance = json_value["LidarCluster"]["EuclideanDistance"].asDouble();


        //
        total_params.lidarBase.lidarPort = json_value["LidarBase"]["lidarPort"].asDouble();
        total_params.lidarBase.LidarAddress = json_value["LidarBase"]["LidarAddress"].asString();
        total_params.lidarBase.ServerPort = json_value["LidarBase"]["ServerPort"].asDouble();
        total_params.lidarBase.ServerAddress = json_value["LidarBase"]["ServerAddress"].asString();
        total_params.lidarBase.DataAddress = json_value["LidarBase"]["DataAddress"].asString();
        total_params.lidarBase.DeviceId = json_value["LidarBase"]["DeviceId"].asInt();
        total_params.lidarBase.LidarModel = json_value["LidarBase"]["LidarModel"].asInt();

        //
        total_params.lidarCameraBase.Pos_X = json_value["LidarCameraBase"]["Pos_X"].asDouble();
        total_params.lidarCameraBase.Pos_Y = json_value["LidarCameraBase"]["Pos_Y"].asDouble();
        total_params.lidarCameraBase.Pos_Z = json_value["LidarCameraBase"]["Pos_Z"].asDouble();
        total_params.lidarCameraBase.View_X = json_value["LidarCameraBase"]["View_X"].asDouble();
        total_params.lidarCameraBase.View_Y = json_value["LidarCameraBase"]["View_Y"].asDouble();
        total_params.lidarCameraBase.View_Z = json_value["LidarCameraBase"]["View_Z"].asDouble();
        total_params.lidarCameraBase.Up_X = json_value["LidarCameraBase"]["Up_X"].asDouble();
        total_params.lidarCameraBase.Up_Y = json_value["LidarCameraBase"]["Up_Y"].asDouble();
        total_params.lidarCameraBase.Up_Z = json_value["LidarCameraBase"]["Up_Z"].asDouble();

        //
        total_params.cameraBase.ip = json_value["CameraBase"]["ip"].asString();
        total_params.cameraBase.code = json_value["CameraBase"]["code"].asString();
        total_params.cameraBase.name = json_value["CameraBase"]["name"].asString();

        //
        total_params.lowerMachineBase.ip = json_value["LowerMachineBase"]["ip"].asString();
        total_params.lowerMachineBase.port = json_value["LowerMachineBase"]["port"].asInt();

    }
    else
    {
        qCritical()<<"Can not parse Json file!";
    }

    ifs.close();
    //total_params.printData();
}
