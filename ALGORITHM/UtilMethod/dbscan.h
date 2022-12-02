#pragma once

#include <vector>
#include <cmath>
#include <omp.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace std;

namespace dbscan {

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3

    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef pcl::PointCloud<PointT>::Ptr PointCloudTPtr;

    typedef struct Point_ {
        float x, y, z;  // X, Y, Z position
        int clusterID;  // clustered ID
    } Point;

    class DBSCAN {
    public:
        DBSCAN(unsigned int minPts, float eps, PointCloudTPtr _cloud) {
            cloud.reset(new PointCloudT);
            *cloud = *_cloud;
            m_minPoints = minPts;
            m_epsilon = eps * eps;
            clusterID = 1;
            m_pointSize = cloud->size();
            for (int i = 0; i < m_pointSize; ++i) {
                Point point{cloud->at(i).x, cloud->at(i).y, cloud->at(i).z, UNCLASSIFIED};
                m_points.push_back(point);
            }
            //cout << m_pointSize << "  " << m_points.size() << endl;
        }

        ~DBSCAN() {
            cloud->clear();
            m_points.clear();
            clusterIndex.clear();
        }

        void runCluster();

        void runCluster(vector<vector<int>> &_clusterIndex);

    private:
        void statisticIndex();

        vector<int> calculateCluster(Point point);

        int expandCluster(Point point, int clusterID);

        inline double calculateDistance(const Point &pointCore, const Point &pointTarget);

    private:
        vector<Point> m_points;
        PointCloudTPtr cloud;
        vector<vector<int>> clusterIndex;
        vector<PointCloudTPtr> clusteredCloud;
    public:
        const vector<PointCloudTPtr> &getClusteredCloud() const;

    private:

        unsigned int m_pointSize;
        unsigned int m_minPoints;
        float m_epsilon;
        int clusterID;
    };
}

