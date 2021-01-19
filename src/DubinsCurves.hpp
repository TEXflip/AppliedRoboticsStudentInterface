#include <cstdint>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>

enum curve_type
{
    LSL,
    RSR,
    LSR,
    RSL,
    RLR,
    LRL,
    N_OF_POSSIBLE_CURVES
};

struct ScaledParams
{
    double sc_th0, sc_thf, sc_k_max, sc_k_max_inv, lambda;
};

struct ScaledCurveSegments
{
    double s1, s2, s3;
    bool ok = false;
};

struct DubinsLine
{
    double s, x, y, th, k;
};

struct DubinsArc
{
    double k, L, x0, y0, th0, xf, yf, thf; // L = length, k = curvature
};

struct DubinsCurve
{
    DubinsArc arcs[3];
    double L;
    curve_type type;
};

class DubinsCurvesHandler
{
private:
    double k_max = 10;

    int8_t curves_arguments[6][3] = {
        {1, 0, 1},
        {-1, 0, -1},
        {1, 0, -1},
        {-1, 0, 1},
        {-1, 1 - 1},
        {1, -1, 1}};

    DubinsLine computeDubinsLine(double L, double x0, double y0, double th0, double k);
    DubinsArc computeDubinsArc(double x0, double y0, double th0, double k, double L);
    DubinsCurve computeDubinsCurve(double x0, double y0, double th0, double s1, double s2, double s3, double k1, double k2, double k3);
    bool check(double s1, double s2, double s3, double k0, double k1, double k2, double th0, double thf);

    double sinc(double t);
    double mod2pi(double angle);
    double rangeSymm(double angle);
    ScaledParams scaleToStandard(double x0, double y0, double th0, double xf, double yf, double thf);
    ScaledCurveSegments scaleFromStandard(ScaledCurveSegments sc_curve_segments, double lambda);
    ScaledCurveSegments LSL(double sc_th0, double sc_thf, double sc_k_max, double sc_k_max_inv);
    ScaledCurveSegments RSR(double sc_th0, double sc_thf, double sc_k_max, double sc_k_max_inv);
    ScaledCurveSegments LSR(double sc_th0, double sc_thf, double sc_k_max, double sc_k_max_inv);
    ScaledCurveSegments RSL(double sc_th0, double sc_thf, double sc_k_max, double sc_k_max_inv);
    ScaledCurveSegments RLR(double sc_th0, double sc_thf, double sc_k_max, double sc_k_max_inv);
    ScaledCurveSegments LRL(double sc_th0, double sc_thf, double sc_k_max, double sc_k_max_inv);

public:
    DubinsCurvesHandler() = default;
    explicit DubinsCurvesHandler(double k_max);
    DubinsCurve findShortestPath(double x0, double y0, double th0, double x1, double y1, double th1);
    std::vector<DubinsLine> discretizeDubinsCurve(DubinsCurve curve, float minLength);
};