#include <string>
#include <sstream>
#include <array>
#include <cmath>
#include <stdexcept>

#define private public
#define protected public

#include "src/Kinematics/RotatingTableFiveAxis.h"

#define private private
#define protected protected

extern "C" {
void cartesian_to_closest_furthest_c(
    float* cx, float* cy, float* cz, float* ca, float* cb,
    int n,
    float* closest, float* furthest,
    float center_x, float center_y, float z_height,
    float rotation_offset, float focus_offset,
    float laser_head_z_angle, float max_z
) {
    Kinematics::RotatingTableFiveAxis k("wrapper");
    k._center_x           = center_x;
    k._center_y           = center_y;
    k._z_height           = z_height;
    k._rotation_offset    = rotation_offset;
    k._focus_offset       = focus_offset;
    k._laser_head_z_angle = laser_head_z_angle;
    k._max_z              = max_z;
    k._last_mpos          = Kinematics::Mpos(0, 0, 0, 0, 0);

    for (auto i=0; i < n; ++i) {
        Kinematics::Cpos cartesian(cx[i], cy[i], cz[i], ca[i], cb[i]);
        Kinematics::Mpos c, f;
        k.cartesian_to_closest_furthest(cartesian, c, f);
        c.copyToFloatArray(closest + i*6);  
        f.copyToFloatArray(furthest + i*6);   
    }
}
}