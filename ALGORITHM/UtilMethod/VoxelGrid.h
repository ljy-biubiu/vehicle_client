#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>   // TicToc
//#include <boost/sort/spreadsort/integer_sort.hpp>

#include <omp.h>

using namespace std;

namespace voxelgrid {
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef pcl::PointCloud<PointT>::Ptr PointCloudTPtr;
    typedef unsigned long long int uInt64;
    typedef long long int Int64;

    //存放计算点云的idx和点云编号的cloud_point_index的结构头
    struct cloud_point_index_idx {
        unsigned int idx;
        unsigned int cloud_point_index;

        cloud_point_index_idx() = default;

        cloud_point_index_idx(unsigned int idx_, unsigned int cloud_point_index_) : idx(idx_), cloud_point_index(cloud_point_index_) {}

        bool operator<(const cloud_point_index_idx &p) const {
            return (idx < p.idx);
        }
    };

    struct Array4usInt64 {
        uInt64 x;
        uInt64 y;
        uInt64 z;
        uInt64 C;
    };
    struct Array4Int64 {
        Int64 x;
        Int64 y;
        Int64 z;
        Int64 C;
    };

    class VoxelGrid {
    public:
        VoxelGrid();

        ~VoxelGrid();

        void filter(const PointCloudTPtr& inputCloud, float xVoxel, float yVoxel, float zVoxel, PointCloudTPtr &outputCloud);

    private:

        void getMaxMinEx(PointCloudTPtr inputCloud, PointT &minP, PointT &maxP);
    };

} // voxelgrid
