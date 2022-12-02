
#ifndef _CSF_H_
#define _CSF_H_

#include <vector>
#include <string>
#include "point_cloud.h"


struct Params {
    bool bSloopSmooth;
    double time_step;
    double class_threshold;
    double cloth_resolution;
    int rigidness;
    int iterations;
};

#ifdef _CSF_DLL_EXPORT_
# ifdef DLL_IMPLEMENT
#  define DLL_API    __declspec(dllexport)
# else // ifdef DLL_IMPLEMENT
#  define DLL_API    __declspec(dllimport)
# endif // ifdef DLL_IMPLEMENT
#endif // ifdef _CSF_DLL_EXPORT_

#ifdef _CSF_DLL_EXPORT_
class DLL_API CSF
#else // ifdef _CSF_DLL_EXPORT_

class CSF
#endif // ifdef _CSF_DLL_EXPORT_
{
public:

    CSF(int index);

    CSF();

    ~CSF();

    // set pointcloud from vector
    void setPointCloud(std::vector<csf::Point> points);

    // set point cloud from a one-dimentional array. it defines a N*3 point cloud by the given rows.
    void setPointCloud(double *points, int rows);

    // set point cloud for python
    void setPointCloud(std::vector<std::vector<float> > points);

    // read pointcloud from txt file: (X Y Z) for each line
    void readPointsFromFile(std::string filename);

    inline csf::PointCloud &getPointCloud() {
        return point_cloud;
    }

    inline const csf::PointCloud &getPointCloud() const {
        return point_cloud;
    }

    std::size_t size() {
        return point_cloud.size();
    }

    void setPointCloud(csf::PointCloud &pc);

    void do_filtering(std::vector<int> &groundIndexes, std::vector<int> &offGroundIndexes, bool exportCloth = false);

private:

#ifdef _CSF_DLL_EXPORT_
    class __declspec (dllexport)csf::PointCloud point_cloud;
#else
    csf::PointCloud point_cloud;
#endif

public:
    Params params;
    int index;
};

#endif // ifndef _CSF_H_
