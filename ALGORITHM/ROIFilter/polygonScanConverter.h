#pragma once

#include "opencv2/opencv.hpp"
#include <iostream>

template<typename T>
bool isfinite(T arg) {
    return arg == arg &&
           arg != std::numeric_limits<T>::infinity() &&
           arg != -std::numeric_limits<T>::infinity();
}

namespace apollo {
    namespace perception {

/**
 * @class PolygonScanConverter
 * @brief: This is a converter from polygon to scan lines, by which we can build
 * bitmap. Assume major direction as x direction, we scan polygons in x
 * ascending
 * order.
 */
        class PolygonScanConverter {
        public:
            typedef cv::Point2d Point;
            typedef std::pair<double, double> Interval;
            typedef std::pair<Point, Point> Segment;

            struct Edge {
                bool operator<(const Edge &other) const {
                    return y < other.y;
                }

                bool MoveUp(const double delta_x);

                double max_x;
                double max_y;

                double x;
                double y;
                double k;
            };

            enum MajorDirection {
                XMAJOR = 0, YMAJOR = 1
            };

            void Init(const MajorDirection major_dir, const Interval &valid_range, const std::vector<cv::Point2d> &polygon, const double step);

            void ConvertScans(std::vector<std::vector<Interval> > *scans_intervals);

        private:
            static const double kEpsilon;
            static const double kInf;
            std::vector<cv::Point2d> polygon_;
            std::vector<Segment> segments_;

            std::vector<double> slope_;

            std::vector<std::vector<Edge> > edge_table_;

            std::vector<Edge> active_edge_table_;

            double min_x_;
            double step_;
            size_t scans_size_;
            MajorDirection major_dir_;
            MajorDirection op_major_dir_;

            /**
             * @brief: If some point of polygon happens to be around the scan line,
             * lightly modify the x coordinate to avoid situation of singular point.
             */
            void DisturbPolygon();

            void BuildEdgeTable();

            void UpdateActiveEdgeTable(const size_t x_id, std::vector<Interval> *scan_intervals);

            void ConvertPolygonToSegments();

            bool ConvertSegmentToEdge(const size_t seg_id, std::pair<int, Edge> *out_edge);
        };

    }  // namespace perception
}  // namespace apollo

