
#include "VoxelGrid.h"

namespace voxelgrid {
    VoxelGrid::VoxelGrid() {

    }

    VoxelGrid::~VoxelGrid() {

    }

    void VoxelGrid::filter(const PointCloudTPtr& inputCloud, float xVoxel, float yVoxel, float zVoxel, PointCloudTPtr &outputCloud) {
        outputCloud->clear();
        //先判断输入的点云是否为空
        if (inputCloud->empty()) {
            cout << "输入点云为空！" << endl;
            return;
        }
        //存放输入点云的最大与最小坐标
        PointT min_p{}, max_p{};
        getMaxMinEx(inputCloud, min_p, max_p);

        PointT inverse_leaf_size_{};
        inverse_leaf_size_.x = 1 / xVoxel;
        inverse_leaf_size_.y = 1 / yVoxel;
        inverse_leaf_size_.z = 1 / zVoxel;

        //计算最小和最大边界框值
        Array4Int64 min_b_{}, max_b_{};
        min_b_.x = (Int64) (floor(min_p.x * inverse_leaf_size_.x));
        max_b_.x = (Int64) (floor(max_p.x * inverse_leaf_size_.x));
        min_b_.y = (Int64) (floor(min_p.y * inverse_leaf_size_.y));
        max_b_.y = (Int64) (floor(max_p.y * inverse_leaf_size_.y));
        min_b_.z = (Int64) (floor(min_p.z * inverse_leaf_size_.z));
        max_b_.z = (Int64) (floor(max_p.z * inverse_leaf_size_.z));

        Array4usInt64 div_b_{}, divb_mul_{};

        //计算沿所有轴所需的分割数
        div_b_.x = max_b_.x - min_b_.x + 1;
        div_b_.y = max_b_.y - min_b_.y + 1;
        div_b_.z = max_b_.z - min_b_.z + 1;
        div_b_.C = 0;

        //设置除法乘数
        divb_mul_.x = 1;
        divb_mul_.y = div_b_.x;
        divb_mul_.z = div_b_.x * div_b_.y;
        divb_mul_.C = 0;

        //用于计算idx和pointcloud索引的存储
        std::vector<cloud_point_index_idx> index_vector;
        index_vector.reserve(inputCloud->size());

        //第一步：遍历所有点并将它们插入到具有计算idx的index_vector向量中;具有相同idx值的点将有助于产生CloudPoint的相同点
        for (int i = 0; i < inputCloud->size(); i++) {
            auto ijk0 = (uInt64) ((Int64) floor(inputCloud->at(i).x * inverse_leaf_size_.x) - min_b_.x);
            auto ijk1 = (uInt64) ((Int64) floor(inputCloud->at(i).y * inverse_leaf_size_.y) - min_b_.y);
            auto ijk2 = (uInt64) ((Int64) floor(inputCloud->at(i).z * inverse_leaf_size_.z) - min_b_.z);

            //计算质心叶索引
            uInt64 idx = ijk0 * divb_mul_.x + ijk1 * divb_mul_.y + ijk2 * divb_mul_.z;
            index_vector.emplace_back(idx, i);
        }

        //第二步：使用表示目标单元格的值作为索引对index_vector向量进行排序;实际上属于同一输出单元格的所有点都将彼此相邻
        std::sort(index_vector.begin(), index_vector.end(), std::less<cloud_point_index_idx>());

        /*auto rightshift_func = [](const cloud_point_index_idx &x, const uInt64 offset) { return x.idx >> offset; };
        boost::sort::spreadsort::integer_sort(index_vector.begin(), index_vector.end(), rightshift_func);*/

        //第三步：计数输出单元格，我们需要跳过所有相同的，相邻的idx值
        uInt64 total = 0;
        uInt64 index = 0;
        uInt64 min_points_per_voxel_ = 0;
        //first_and_last_indices_vector [i]表示属于对应于第i个输出点的体素的index_vector中的第一个点的index_vector中的索引，以及不属于第一个点的索引
        std::vector<std::pair<uInt64, uInt64> > first_and_last_indices_vector;
        first_and_last_indices_vector.reserve(index_vector.size());                              //分配内存空间

        while (index < index_vector.size()) {
            uInt64 i = index + 1;
            while (i < index_vector.size() && index_vector[i].idx == index_vector[index].idx)
                ++i;
            if (i - index >= min_points_per_voxel_) {
                ++total;
                first_and_last_indices_vector.push_back(std::pair<uInt64, uInt64>(index, i));
            }
            index = i;
        }

        //第四步：计算质心，将它们插入最终位置
        uInt64 first_and_last_indices_vector_size = first_and_last_indices_vector.size();
        outputCloud->resize(total);      //给输出点云分配内存空间
        for (uInt64 cp = 0; cp < first_and_last_indices_vector_size; ++cp) {
            // 计算质心 - 来自所有输入点的和值，这些值在index_vector数组中具有相同的idx值
            PointT PointCloud{};
            uInt64 first_index, last_index;
            first_index = first_and_last_indices_vector[cp].first;
            last_index = first_and_last_indices_vector[cp].second;
            double x_Sum = 0, y_Sum = 0, z_Sum = 0, i_Sum = 0;
            for (uInt64 li = first_index; li < last_index; ++li) {
                x_Sum += inputCloud->at(index_vector[li].cloud_point_index).x;
                y_Sum += inputCloud->at(index_vector[li].cloud_point_index).y;
                z_Sum += inputCloud->at(index_vector[li].cloud_point_index).z;
            }
            double deltaIndex = last_index - first_index;
            PointCloud = inputCloud->at(first_index);
            PointCloud.x = x_Sum / deltaIndex;
            PointCloud.y = y_Sum / deltaIndex;
            PointCloud.z = z_Sum / deltaIndex;
            outputCloud->at(cp) = PointCloud;
        }
    }

    void VoxelGrid::getMaxMinEx(PointCloudTPtr inputCloud, PointT &minP, PointT &maxP) {
        minP = {};
        maxP = {};
        if (inputCloud->empty()) {
            cout << "输入点云为空" << endl;
            return;
        }
        minP = inputCloud->at(0);
        maxP = inputCloud->at(0);
        for (PointT &p: *inputCloud) {
            if (minP.x > p.x) {
                minP.x = p.x;
            }
            if (minP.y > p.y) {
                minP.y = p.y;
            }
            if (minP.z > p.z) {
                minP.z = p.z;
            }
            if (maxP.x < p.x) {
                maxP.x = p.x;
            }
            if (maxP.y < p.y) {
                maxP.y = p.y;
            }
            if (maxP.z < p.z) {
                maxP.z = p.z;
            }
        }
    }
} // voxelgrid