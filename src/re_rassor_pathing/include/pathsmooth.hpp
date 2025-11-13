#pragma once
#include <vector>
#include <string>

struct Pt {double x, y;};

struct SmoothParams {
    int smoothing_passes = 4;
    int num_points = 50;
    double xmin = -50, xmax = 800, ymin = -50, ymax = 800; 
};

std::vector<Pt> SmoothPathBSpline(const std::vector<Pt>& waypoints, const SmoothParams& params);
std::vector<Pt> LoadWaypointsCsv(const std::string& path, bool has_header=true);
void SaveCsvXY(const std::string& path, const std::vector<Pt>& pts);