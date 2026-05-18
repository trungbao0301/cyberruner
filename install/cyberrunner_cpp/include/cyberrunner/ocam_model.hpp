#pragma once
#include <array>
#include <cmath>
#include <opencv2/opencv.hpp>

// ── OCamCalib camera parameters (from ocam_params.py) ────────────────────────
struct OcamModel {
    // Forward polynomial: maps sensor (rho) → angle t  (used for calibration)
    std::array<double, 5> pol = {
        -2.019736e+03, 0.0, 1.721818e-03, -2.128158e-06, 1.185387e-09
    };
    // Inverse polynomial: maps angle t → sensor radius r  (used for rectification)
    std::array<double, 11> invpol = {
         6527.896707,    27857.075672,   18825.582643, -214449.985674,
        -792381.859202, -1374262.433263, -1419615.741175,
        -917497.821432,  -364331.873758,  -81330.665119, -7808.821317
    };
    double cy     = 598.837773;
    double cx     = 958.882643;
    double c      = 0.999326;
    double d      = 0.000071;
    double e      = 0.000066;
    int    height = 1200;
    int    width  = 1920;
};

// Output rectified image dimensions
constexpr int    OCAM_OUT_W    = 960;
constexpr int    OCAM_OUT_H    = 720;
constexpr double OCAM_DEFAULT_FX = 850.0;
constexpr double OCAM_DEFAULT_FY = 750.0;

// ── Build OCamCalib undistortion maps ─────────────────────────────────────────
// Equivalent to build_rectify_map() in camera_node.py.
// Produces two CV_32F maps for use with cv::remap().
inline void buildOcamRectifyMap(
    const OcamModel& ocam,
    int out_w, int out_h,
    double fx, double fy,
    cv::Mat& map_x, cv::Mat& map_y)
{
    map_x.create(out_h, out_w, CV_32F);
    map_y.create(out_h, out_w, CV_32F);

    for (int row = 0; row < out_h; ++row) {
        for (int col = 0; col < out_w; ++col) {
            // Back-project pixel (col, row) through pinhole model
            double xn = (col - out_w * 0.5) / fx;
            double yn = (row - out_h * 0.5) / fy;
            double zn = 1.0;
            double norm = std::sqrt(xn*xn + yn*yn + zn*zn);
            double X = xn / norm;
            double Y = yn / norm;
            double Z = zn / norm;

            // OCamCalib world→cam projection
            double rho = std::sqrt(X*X + Y*Y) + 1e-12;
            double t   = std::atan2(-Z, rho);

            // Evaluate inverse polynomial: r = sum(invpol[i] * t^i)
            double r  = 0.0;
            double tt = 1.0;
            for (double coeff : ocam.invpol) {
                r  += coeff * tt;
                tt *= t;
            }

            double xn2 = X / rho;
            double yn2 = Y / rho;

            // Apply affine distortion model (c, d, e parameters)
            double u = ocam.cx + (xn2 * r) * ocam.c + (yn2 * r) * ocam.d;
            double v = ocam.cy + (xn2 * r) * ocam.e + (yn2 * r);

            map_x.at<float>(row, col) = static_cast<float>(u);
            map_y.at<float>(row, col) = static_cast<float>(v);
        }
    }
}
