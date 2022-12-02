#include "hdmapRoiFilter.h"

void HdmapROIFilter::DrawPolygon(const std::vector<cv::Point2d> polygon_, double range_, double cell_size_)
{
	// 1. Get Major Direction as X direction and convert map_polygons to raw polygons
	range = range_;
	cell_size = cell_size_;
	extend_dist = 0;
	polygon = polygon_;
	//MajorDirection major_dir = GetMajorDirection();
	major_dir = MajorDirection::XMAJOR;

	// 2. Get valid x range
	Interval valid_x_range;
	GetValidXRange(&valid_x_range);

	// 3. Convert polygon to scan intervals(Most important)
	std::vector< std::vector<Interval> > scans_intervals;
	apollo::perception::PolygonScanConverter polygon_scan_converter;
	polygon_scan_converter.Init(major_dir, valid_x_range, polygon, cell_size);
	polygon_scan_converter.ConvertScans(&scans_intervals);

	cv::Mat polygonPoint(cvRound(range * 2 / cell_size), cvRound(range * 2 / cell_size), CV_8UC1, cv::Scalar(0));
	for(int i = 0; i < (int)polygon.size(); i++)
	{
		int col, row;
		if(major_dir == MajorDirection::XMAJOR)
		{
			col = cvRound((range + polygon[i].x) / cell_size);
			row = cvRound((range + polygon[i].y) / cell_size);
		}
		else
		{
			col = cvRound((range + polygon[i].y) / cell_size);
			row = cvRound((range + polygon[i].x) / cell_size);
		}
		if(row < 0 || row > polygonPoint.rows || col < 0 || col > polygonPoint.cols) continue;
		polygonPoint.at<uchar>(row, col) = 255;
	}

	double x = valid_x_range.first;
	binary = cv::Mat(cvRound(range * 2 / cell_size), cvRound(range * 2 / cell_size), CV_8UC1, cv::Scalar(0));
	for (size_t i = 0; i < scans_intervals.size(); x += cell_size, i++)
	{
		for(std::vector<Interval>::iterator scan_interval = scans_intervals[i].begin(); scan_interval < scans_intervals[i].end(); scan_interval++)
		{
			if (scan_interval->first > scan_interval->second)
				std::cout << "scan interval is not valid: " << "scan_interval.first = " << scan_interval->first << ", " << "scan_interval.second = " << scan_interval->second << ".";
			Interval valid_y_range;
			valid_y_range.first = std::max(-range, scan_interval->first - extend_dist);
			valid_y_range.second = std::min(range, scan_interval->second + extend_dist);

			if (valid_y_range.first > valid_y_range.second)
				continue;
			size_t x_id = static_cast<size_t>((x + range) / cell_size);
			size_t min_y_id = static_cast<size_t>((valid_y_range.first + range) / cell_size);
			size_t max_y_id = static_cast<size_t>((valid_y_range.second + range) / cell_size);
			for(size_t k = min_y_id; k < max_y_id; k++)
				binary.at<uchar>(k, x_id) = 255;
		}
	}

	//cv::imshow("polygonPoint", polygonPoint);
	//cv::imshow("binary", binary);
}

bool HdmapROIFilter::HdmapRoiFilter(cv::Point2d point)
{
	int col, row;
	if(major_dir == MajorDirection::XMAJOR)
	{
		col = cvRound((range + point.x) / cell_size);
		row = cvRound((range + point.y) / cell_size);
	}
	else
	{
		col = cvRound((range + point.y) / cell_size);
		row = cvRound((range + point.x) / cell_size);
	}
	if(row < 0 || row >= binary.rows || col < 0 || col >= binary.cols)
		return false;
	if(binary.at<uchar>(row, col) == 255)
		return true;
	else
		return false;
}

MajorDirection HdmapROIFilter::GetMajorDirection()
{
	double min_x = range, min_y = range;
	double max_x = -range, max_y = -range;

	// Get available x_range and y_range, then set the direction with small range as major direction.
	for (size_t j = 0; j < polygon.size(); j++)
	{
		min_x = std::min(min_x, polygon[j].x);
		max_x = std::max(max_x, polygon[j].x);
		min_y = std::min(min_y, polygon[j].y);
		max_y = std::max(max_y, polygon[j].y);
	}

	min_x = std::max(min_x, -range);
	max_x = std::min(max_x, range);
	min_y = std::max(min_y, -range);
	max_y = std::min(max_y, range);

	return (max_x - min_x) < (max_y - min_y) ? MajorDirection::XMAJOR : MajorDirection::YMAJOR;
}

void HdmapROIFilter::GetValidXRange(Interval* valid_x_range)
{
	double polygon_min_pt[2] = {range, range};
	double polygon_max_pt[2] = {-range, -range};

	for(std::vector<cv::Point2d>::iterator point = polygon.begin(); point < polygon.end(); point++)
	{
		polygon_min_pt[0] = std::min(polygon_min_pt[0], point->x);
		polygon_min_pt[1] = std::min(polygon_min_pt[1], point->y);

		polygon_max_pt[0] = std::max(polygon_max_pt[0], point->x);
		polygon_max_pt[1] = std::max(polygon_max_pt[1], point->y);
	}

	valid_x_range->first = std::max(polygon_min_pt[major_dir], -range);
	valid_x_range->second = std::min(polygon_max_pt[major_dir], range);

	// For numerical stability
	valid_x_range->first = (static_cast<int>((valid_x_range->first + range) / cell_size) + 0.5) * cell_size - range;
}
