#pragma once
#include <RVC/RVC.h>
#include <vector>

struct XYZRange {
    XYZRange() {
        xmin = -5.0f;
        xmax = 5.0f;
        ymin = -5.0f;
        ymax = 5.0f;
        zmin = -5.0f;
        zmax = 5.0f;
    }

    float xmin, xmax;
    float ymin, ymax;
    float zmin, zmax;
};

int Truncate(RVC::PointMap &pm, const XYZRange &range, std::vector<double> &truncate_pts);