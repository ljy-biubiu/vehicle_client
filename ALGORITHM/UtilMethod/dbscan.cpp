#include "dbscan.h"

using namespace dbscan;

void DBSCAN::runCluster() {
    clusterID = 1;
    vector<Point>::iterator iter;
    for (iter = m_points.begin(); iter != m_points.end(); ++iter) {
        if (iter->clusterID == UNCLASSIFIED) {
            if (expandCluster(*iter, clusterID) != FAILURE) {
                clusterID += 1;
            }
        }
    }
    clusterID--;
    //cout << "clusterNum = " << clusterID << endl;
    this->statisticIndex();
}

void DBSCAN::runCluster(vector<vector<int>> &_clusterIndex) {
    this->runCluster();
    _clusterIndex = clusterIndex;
}

int DBSCAN::expandCluster(Point point, int _clusterID) {
    // 计算近邻点
    vector<int> clusterSeeds = calculateCluster(point);

    if (clusterSeeds.size() < m_minPoints) {
        point.clusterID = NOISE;// 噪声点
        return FAILURE;
    } else {
        int index = 0, indexCorePoint = -1;
        vector<int>::iterator iterSeeds;
        // 判断核心点所在的索引位置
        for (iterSeeds = clusterSeeds.begin(); iterSeeds != clusterSeeds.end(); ++iterSeeds) {
            m_points.at(*iterSeeds).clusterID = _clusterID;
            if (m_points.at(*iterSeeds).x == point.x && m_points.at(*iterSeeds).y == point.y &&
                m_points.at(*iterSeeds).z == point.z) {
                indexCorePoint = index;
            }
            ++index;
        }
        // 擦除核心点索引之前的数据点
        if (indexCorePoint >= 0) {
            clusterSeeds.erase(clusterSeeds.begin() + indexCorePoint);
        }

        for (vector<int>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i) {
            vector<int> clusterNeighors = calculateCluster(m_points.at(clusterSeeds[i]));

            if (clusterNeighors.size() >= m_minPoints) {
                vector<int>::iterator iterNeighors;
                for (iterNeighors = clusterNeighors.begin(); iterNeighors != clusterNeighors.end(); ++iterNeighors) {
                    if (m_points.at(*iterNeighors).clusterID == UNCLASSIFIED ||
                        m_points.at(*iterNeighors).clusterID == NOISE) {
                        if (m_points.at(*iterNeighors).clusterID == UNCLASSIFIED) {
                            clusterSeeds.push_back(*iterNeighors);
                            n = clusterSeeds.size();
                        }
                        m_points.at(*iterNeighors).clusterID = _clusterID;
                    }
                }
            }
        }
        return SUCCESS;
    }
}

vector<int> DBSCAN::calculateCluster(Point point) {
    int index = 0;
    vector<Point>::iterator iter;
    vector<int> clusterIndex;
    /*for (iter = m_points.begin(); iter != m_points.end(); ++iter) {
        if (calculateDistance(point, *iter) <= m_epsilon) {
            clusterIndex.push_back(index);
        }
        index++;
    }*/

    for (auto &p: m_points) {
        if (calculateDistance(point, p) <= m_epsilon) {
            clusterIndex.push_back(index);
        }
        index++;
    }
    /*for (int i = 0; i < m_pointSize; ++i) {
        if (calculateDistance(point, m_points.at(i)) <= m_epsilon){
            clusterIndex.push_back(i);
        }
    }*/
    return clusterIndex;
}

inline double DBSCAN::calculateDistance(const Point &pointCore, const Point &pointTarget) {
    return pow(pointCore.x - pointTarget.x, 2) + pow(pointCore.y - pointTarget.y, 2) +
           pow(pointCore.z - pointTarget.z, 2);
}

void DBSCAN::statisticIndex() {
    clusterIndex.resize(clusterID);
    clusteredCloud.resize(clusterID);
    for (int i = 0; i < clusterID; ++i) {
        clusteredCloud.at(i) = boost::shared_ptr<PointCloudT >(new PointCloudT);
    }
    for (int i = 0; i < m_pointSize; ++i) {
        int clusterIdTmp = m_points.at(i).clusterID;
        if (clusterIdTmp > 0) {
            clusterIndex.at(clusterIdTmp - 1).push_back(i);
            clusteredCloud.at(clusterIdTmp - 1)->push_back(cloud->at(i));
        }
    }
}

const vector<PointCloudTPtr> &DBSCAN::getClusteredCloud() const {
    return clusteredCloud;
}




