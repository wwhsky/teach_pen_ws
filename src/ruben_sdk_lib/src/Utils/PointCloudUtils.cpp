#include "ruben_sdk_lib/Utils/PointCloudUtils.h"

#include <iostream>

#define INRANGE(v, l, h) ((v) > (l) && (v) < (h))

int Truncate(RVC::PointMap &pm, const XYZRange &range, std::vector<double> &truncate_pts) {
    if (!pm.IsValid()) {
        std::cout << "point map is invalid!" << std::endl;
        return 1;
    }

    RVC::Size pm_size = pm.GetSize();
    size_t pm_num = pm_size.rows * pm_size.cols;

    if (pm_num == 0) {
        std::cout << "invalid point num: " << pm_num << std::endl;
        return 2;
    }

    truncate_pts.clear();
    truncate_pts.resize(pm_num * 3);

    const double xmin = range.xmin;
    const double xmax = range.xmax;
    const double ymin = range.ymin;
    const double ymax = range.ymax;
    const double zmin = range.zmin;
    const double zmax = range.zmax;

    double *points = pm.GetPointDataPtr();
    size_t actual_size = 0;

    for (size_t i = 0; i < pm_num; i++) {
        double x = *points;
        double y = *(points + 1);
        double z = *(points + 2);

        if (INRANGE(x, xmin, xmax) && INRANGE(y, ymin, ymax) && INRANGE(z, zmin, zmax)) {
            truncate_pts[actual_size] = x;
            truncate_pts[actual_size + 1] = y;
            truncate_pts[actual_size + 2] = z;
            actual_size += 3;
        }
        points += 3;
    }

    truncate_pts.resize(actual_size);
    return 0;
}