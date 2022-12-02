//
// Created by niujy on 2022/11/23.
//

#include "ROIFilter.h"

namespace roifilter {
    ROIFilter::ROIFilter() {

    }

    ROIFilter::~ROIFilter() {

    }

    void ROIFilter::roiFilter(PointCloudTPtr cloud_in, std::string pointsPath, double cell_size_, PointCloudTPtr &cloud_out) {
        std::vector<cv::Point2d> polygon;
        this->readRegionPoints(pointsPath, polygon);
        this->roiFilter(cloud_in, polygon, cell_size_, cloud_out);
    }

    void ROIFilter::roiFilter(PointCloudTPtr cloud_in, std::vector<cv::Point2d> polygon, double cell_size_, PointCloudTPtr &cloud_out) {
//        double st = Util::timeMs();
        cv::Point2d centerPoly(0, 0);
        cv::Point2d minP = polygon.at(0);
        cv::Point2d maxP = polygon.at(0);
        for (int i = 0; i < polygon.size(); ++i) {
            /*centerPoly.x += polygon.at(i).x;
            centerPoly.y += polygon.at(i).y;*/
            if (minP.x > polygon.at(i).x) {
                minP.x = polygon.at(i).x;
            }
            if (minP.y > polygon.at(i).y) {
                minP.y = polygon.at(i).y;
            }
            if (maxP.x < polygon.at(i).x) {
                maxP.x = polygon.at(i).x;
            }
            if (maxP.y < polygon.at(i).y) {
                maxP.y = polygon.at(i).y;
            }
        }
        /*centerPoly.x /= polygon.size();
        centerPoly.y /= polygon.size();*/
        centerPoly.x = (minP.x + maxP.x) * 0.5;
        centerPoly.y = (minP.y + maxP.y) * 0.5;

        double lx = maxP.x - minP.x;
        double ly = maxP.y - minP.y;
        double range = sqrt(lx * lx + ly * ly);

        if (range < cell_size_ * 3) {
            *cloud_out = *cloud_in;
            return;
        }

        std::vector<cv::Point2d> polygonNew;
        for (int i = 0; i < polygon.size(); ++i) {
            polygonNew.push_back(cv::Point2d(polygon.at(i).x - centerPoly.x, polygon.at(i).y - centerPoly.y));
        }

//        cout << "polygonNew = " << Util::timeMs() - st << " ms " << endl;
//        st = Util::timeMs();

        HdmapROIFilter hdmapRoiFilter;
        hdmapRoiFilter.DrawPolygon(polygonNew, range * 0.5, cell_size_);
//        cout << "DrawPolygon = " << Util::timeMs() - st << " ms " << endl;
//        st = Util::timeMs();
        //cout<<"range = "<<range * 0.5<<endl;
        size_t cloudSize = cloud_in->size();
        for (size_t i = 0; i < cloudSize; ++i) {
            cv::Point2d point = cv::Point2d(cloud_in->points[i].x - centerPoly.x, cloud_in->points[i].y - centerPoly.y);
            if (hdmapRoiFilter.HdmapRoiFilter(point)) {
                cloud_out->push_back(cloud_in->points[i]);
            }
        }
//        cout << "HdmapRoiFilter = " << Util::timeMs() - st << " ms " << endl;
//        st = Util::timeMs();
    }

    void ROIFilter::readRegionPoints(std::string pointsPath, vector<cv::Point2d> &polygon) {
        std::ifstream polygon_data(pointsPath, std::ifstream::in);
        double x = 0, y = 0;
        while (polygon_data >> x >> y) {
            polygon.push_back(cv::Point2d(x, y));
        }
        polygon_data.close();
    }
} // roifilter