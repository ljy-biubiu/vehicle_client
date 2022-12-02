
#define DLL_IMPLEMENT

#include "CSF.h"
#include "XYZReader.h"
#include "Vec3.h"
#include "Cloth.h"
#include "Rasterization.h"
#include "c2cdist.h"
#include <fstream>

CSF::CSF(int index) {
    params.bSloopSmooth = true;
    params.time_step = 1;
    params.class_threshold = 0.5;
    params.cloth_resolution = 0.5;
    params.rigidness = 3;
    params.iterations = 5000;

    this->index = index;
}

CSF::CSF() {
    params.bSloopSmooth = true;
    params.time_step = 1;
    params.class_threshold = 0.5;
    params.cloth_resolution = 0.5;
    params.rigidness = 3;
    params.iterations = 5000;
    this->index = 0;
}

CSF::~CSF() {}

void CSF::setPointCloud(std::vector<csf::Point> points) {
    point_cloud.resize(points.size());
    int pointCount = static_cast<int>(points.size());
#pragma omp parallel for
    for (int i = 0; i < pointCount; i++) {
        csf::Point las;
        las.x = points[i].x;
        las.y = -points[i].z;
        las.z = points[i].y;
        point_cloud[i] = las;
    }
}

void CSF::setPointCloud(double *points, int rows) {
#define A(i, j) points[i + j * rows]

    for (int i = 0; i < rows; i++) {
        csf::Point p;
        p.x = A(i, 0);
        p.y = -A(i, 2);
        p.z = A(i, 1);
        point_cloud.push_back(p);
    }
}

void CSF::setPointCloud(csf::PointCloud &pc) {
    point_cloud.resize(pc.size());
    int pointCount = static_cast<int>(pc.size());
#pragma omp parallel for
    //点云倒置
    for (int i = 0; i < pointCount; i++) {
        csf::Point las;
        las.x = pc[i].x;
        las.y = -pc[i].z;
        las.z = pc[i].y;
        point_cloud[i] = las;
    }
}

void CSF::setPointCloud(std::vector<std::vector<float> > points) {
    point_cloud.resize(points.size());
    int pointCount = static_cast<int>(points.size());
#pragma omp parallel for
    for (int i = 0; i < pointCount; i++) {
        csf::Point las;
        las.x = points[i][0];
        las.y = -points[i][2];
        las.z = points[i][1];
        point_cloud[i] = las;
    }
}

void CSF::readPointsFromFile(std::string filename) {
    this->point_cloud.resize(0);
    read_xyz(filename, this->point_cloud);
}

//执行过滤函数
void CSF::do_filtering(std::vector<int> &groundIndexes, std::vector<int> &offGroundIndexes, bool exportCloth) {
    // Terrain
    //std::cout << "[" << this->index << "] Configuring terrain..." << std::endl;
    csf::Point bbMin, bbMax;
    //获取最大最小边界
    point_cloud.computeBoundingBox(bbMin, bbMax);
    double cloth_y_height = 0.05;

    int clothbuffer_d = 2;
    Vec3 origin_pos(bbMin.x - clothbuffer_d * params.cloth_resolution, bbMax.y + cloth_y_height, bbMin.z - clothbuffer_d * params.cloth_resolution);

    int width_num = static_cast<int>(std::floor((bbMax.x - bbMin.x) / params.cloth_resolution)) + 2 * clothbuffer_d;

    int height_num = static_cast<int>(std::floor((bbMax.z - bbMin.z) / params.cloth_resolution)) + 2 * clothbuffer_d;

    //std::cout << "[" << this->index << "] Configuring cloth..." << std::endl;
    //std::cout << "[" << this->index << "]  - width: " << width_num << " " << "height: " << height_num << std::endl;

    Cloth cloth(
            origin_pos,
            width_num,
            height_num,
            params.cloth_resolution,
            params.cloth_resolution,
            0.3,
            9999,
            params.rigidness,
            params.time_step
    );

    //std::cout << "[" << this->index << "] Rasterizing..." << std::endl;
    Rasterization::RasterTerrian(cloth, point_cloud, cloth.getHeightvals());

    double time_step2 = params.time_step * params.time_step;
    double gravity = 0.2;

    //std::cout << "[" << this->index << "] Simulating..." << std::endl;
    cloth.addForce(Vec3(0, -gravity, 0) * time_step2);


    for (int i = 0; i < params.iterations; i++) {
        double maxDiff = cloth.timeStep();
        cloth.terrCollision();
        //params.class_threshold / 100
        if ((maxDiff != 0) && (maxDiff < 0.005)) {
            // early stop
            break;
        }
        // pd++;
    }

    if (params.bSloopSmooth) {
        //std::cout << "[" << this->index << "]  - post handle..." << std::endl;
        cloth.movableFilter();
    }

    if (exportCloth)
        cloth.saveToFile();

    c2cdist c2c(params.class_threshold);
    c2c.calCloud2CloudDist(cloth, point_cloud, groundIndexes, offGroundIndexes);
}

