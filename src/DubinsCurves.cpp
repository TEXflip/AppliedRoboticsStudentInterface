#include "DubinsCurves.hpp"

#include <algorithm>
#include <cmath>

DubinsCurvesHandler::DubinsCurvesHandler(double k_max)
{
    this->k_max = k_max;
}

DubinsCurvesHandler::DubinsCurvesHandler(double k_max, double discretizer_size)
{
    this->k_max = k_max;
    this->discretizer_size = discretizer_size;
}

DubinsLine DubinsCurvesHandler::computeDubinsLine(double s, double x0, double y0, double th0, double k)
{
    DubinsLine dubins_line{};

    dubins_line.s = s;
    dubins_line.x = x0 + s * sinc(k * s / 2.0) * cos(th0 + k * s / 2.0);
    dubins_line.y = y0 + s * sinc(k * s / 2.0) * cos(th0 + k * s / 2.0);
    dubins_line.th = mod2pi(th0 + k * s);
    dubins_line.k = k;

    return dubins_line;
}

DubinsArc DubinsCurvesHandler::computeDubinsArc(double x0, double y0, double th0, double k, double L)
{
    DubinsArc dubins_arc{};

    dubins_arc.k = k;
    dubins_arc.L = L;

    DubinsLine corr_dubins_line = computeDubinsLine(L, x0, y0, th0, k);
    dubins_arc.xf = corr_dubins_line.x;
    dubins_arc.yf = corr_dubins_line.y;
    dubins_arc.thf = corr_dubins_line.th;

    return dubins_arc;
}

DubinsCurve DubinsCurvesHandler::computeDubinsCurve(double x0, double y0, double th0, double s1, double s2, double s3, double k1, double k2, double k3)
{
    DubinsCurve d_curve{};

    d_curve.arcs[0] = computeDubinsArc(x0, y0, th0, k1, s1);
    d_curve.arcs[1] = computeDubinsArc(d_curve.arcs[0].xf, d_curve.arcs[0].yf, d_curve.arcs[0].thf, k1, s2);
    d_curve.arcs[2] = computeDubinsArc(d_curve.arcs[1].xf, d_curve.arcs[1].yf, d_curve.arcs[1].thf, k2, s3);

    d_curve.L = d_curve.arcs[0].L + d_curve.arcs[1].L + d_curve.arcs[2].L;

    return d_curve;
}

bool DubinsCurvesHandler::check(double s1, double s2, double s3, double k0, double k1, double k2, double th0, double thf)
{
    double x0 = -1;
    double y0 = 0;
    double xf = 0;
    double yf = 0;

    double eq1 = x0 + s1 * sinc(0.5 * k0 * s1) * cos(th0 + 0.5 * k0 * s1) 
                    + s2 * sinc(0.5 * k1 * s2) * cos(th0 + k0 * s1 + 0.5 * k1 * s2) 
                    + s3 * sinc(0.5 * k2 * s3) * cos(th0 + k0 * s1 + k1 * s2 + 0.5 * k2 * s3) - xf;

    double eq2 = y0 + s1 * sinc(0.5 * k0 * s1) * sin(th0 + 0.5 * k0 * s1) 
                    + s2 * sinc(0.5 * k1 * s2) * sin(th0 + k0 * s1 + 0.5 * k1 * s2) 
                    + s3 * sinc(0.5 * k2 * s3) * sin(th0 + k0 * s1 + k1 * s2 + 0.5 * k2 * s3) - yf;

    double eq3 = rangeSymm(k0 * s1 + k1 * s2 + k2 * s3 + th0 - thf);

    bool Lpos = (s1 > 0) || (s2 > 0) || (s3 > 0);
    return (sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3) < 1.e-10) && Lpos;
}

double DubinsCurvesHandler::sinc(double t)
{
    if (abs(t) < 0.002)
        return 1.0 - (t * t) / 6.0 * (1.0 - (t * t) / 20.0);
    else
        return sin(t) / t;
}

double DubinsCurvesHandler::mod2pi(double ang)
{
    double out = ang;
    while (out < 0)
        out += 2 * M_PI;
    while (out >= 2 * M_PI)
        out -= 2 * M_PI;
    return out;
}

double DubinsCurvesHandler::rangeSymm(double ang)
{
    double out = ang;
    while (out <= -M_PI)
        out += 2 * M_PI;
    while (out > M_PI)
        out -= 2 * M_PI;
    return out;
}

ScaledParams DubinsCurvesHandler::scaleToStandard(double x0, double y0, double th0, double xf, double yf, double thf)
{
    ScaledParams params{};

    double dx = xf - x0;
	double dy = yf - y0;
    double phi = atan2(dy, dx);
	params.lambda = hypot(dx, dy)/2.0;

	params.sc_k_max = this->k_max * params.lambda;
	params.sc_th0 = mod2pi(th0 - phi);
	params.sc_thf = mod2pi(thf - phi);

    return params;
}

ScaledCurveSegments DubinsCurvesHandler::scaleFromStandard(ScaledCurveSegments sc_curve_segments, double lambda)
{
    ScaledCurveSegments out{};
    out.s1 = sc_curve_segments.s1 * lambda;
    out.s2 = sc_curve_segments.s2 * lambda;
    out.s3 = sc_curve_segments.s3 * lambda;
    return out;
}

ScaledCurveSegments DubinsCurvesHandler::LSL(double sc_th0, double sc_thf, double sc_k_max, double sc_k_max_inv)
{
    ScaledCurveSegments out{};

    double C = cos(sc_thf) - cos(sc_th0);
    double S = 2.0 * sc_k_max + sin(sc_th0) - sin(sc_thf);
    double temp1 = atan2(C, S);
	out.sc_s1 = sc_k_max_inv * mod2pi(temp1 - sc_th0);
    double temp2 = 2. + 4. * (sc_k_max*sc_k_max) - 2. * cos(sc_th0 - sc_thf) + 4. * sc_k_max * (sin(sc_th0) - sin(sc_thf));
    if (temp2 < 0){
        out.sc_s1 = 0;
        out.sc_s2 = 0;
        out.sc_s3 = 0;
        return out;
    }
	out.sc_s2 = sc_k_max_inv * sqrt(temp2);
    out.sc_s3 = sc_k_max_inv * mod2pi(sc_thf - temp1);
    out.ok = true;
    //check(sc_s1, sc_k_max, sc_s2, 0, sc_s3, sc_k_max, sc_th0, sc_thf);
    return out;
}
ScaledCurveSegments DubinsCurvesHandler::RSR(double sc_th0, double sc_thf, double sc_k_max, double sc_k_max_inv)
{
    ScaledCurveSegments out{};

    double C = cos(sc_th0) - cos(sc_thf);
    double S = 2.0 * sc_k_max - sin(sc_th0) + sin(sc_thf);
    double temp1 = atan2(C, S);
	out.sc_s1 = sc_k_max_inv * mod2pi(sc_th0-temp1);
    double temp2 = 2. + 4. * (sc_k_max*sc_k_max) - 2. * cos(sc_th0 - sc_thf) - 4. * sc_k_max * (sin(sc_th0) - sin(sc_thf));
    if (temp2 < 0){
        out.sc_s1 = 0;
        out.sc_s2 = 0;
        out.sc_s3 = 0;
        return out;
    }
	out.sc_s2 = sc_k_max_inv * sqrt(temp2);
    out.sc_s3 = sc_k_max_inv * mod2pi(temp1-sc_thf);
    out.ok = true;
    //check(sc_s1, -sc_k_max, sc_s2, 0, sc_s3, -sc_k_max, sc_th0, sc_thf);
    return out;
}
ScaledCurveSegments DubinsCurvesHandler::LSR(double sc_th0, double sc_thf, double sc_k_max, double sc_k_max_inv)
{
    ScaledCurveSegments out{};

    double C = cos(sc_th0) + cos(sc_thf);
    double S = 2.0 * sc_k_max + sin(sc_th0) + sin(sc_thf);
    double temp1 = atan2(-C, S);
    double temp3 = 4. * (sc_k_max*sc_k_max) - 2. + 2. * cos(sc_th0 - sc_thf) + 4. * sc_k_max * (sin(sc_th0) + sin(sc_thf));
    if (temp3 < 0){
        out.sc_s1 = 0;
        out.sc_s2 = 0;
        out.sc_s3 = 0;
        return out;
    }
	out.sc_s2 = sc_k_max_inv * sqrt(temp3);
    double temp2 = -atan2(-2, out.sc_s2 * sc_k_max);
    out.sc_s1 = sc_k_max_inv * mod2pi(temp1+temp2-sc_th0);
    out.sc_s3 = sc_k_max_inv * mod2pi(temp1+temp2-sc_thf);
    out.ok = true;
    //check(sc_s1, sc_k_max, sc_s2, 0, sc_s3, -sc_k_max, sc_th0, sc_thf);
    return out;
}
ScaledCurveSegments DubinsCurvesHandler::RSL(double sc_th0, double sc_thf, double sc_k_max, double sc_k_max_inv)
{
    ScaledCurveSegments out{};

    double C = cos(sc_th0) + cos(sc_thf);
    double S = 2.0 * sc_k_max - sin(sc_th0) - sin(sc_thf);
    double temp1 = atan2(C, S);
    double temp3 = 4. * (sc_k_max*sc_k_max) - 2. + 2. * cos(sc_th0 - sc_thf) - 4. * sc_k_max * (sin(sc_th0) + sin(sc_thf));
    if (temp3 < 0){
        out.sc_s1 = 0;
        out.sc_s2 = 0;
        out.sc_s3 = 0;
        return out;
    }
	out.sc_s2 = sc_k_max_inv * sqrt(temp3);
    double temp2 = atan2(2, out.sc_s2 * sc_k_max);
    out.sc_s1 = sc_k_max_inv * mod2pi(sc_th0-temp1+temp2);
    out.sc_s3 = sc_k_max_inv * mod2pi(sc_thf-temp1+temp2);
    out.ok = true;
    //check(sc_s1, -sc_k_max, sc_s2, 0, sc_s3, sc_k_max, sc_th0, sc_thf);
    return out;
}
ScaledCurveSegments DubinsCurvesHandler::RLR(double sc_th0, double sc_thf, double sc_k_max, double sc_k_max_inv)
{
    ScaledCurveSegments out{};

    double C = cos(sc_th0) - cos(sc_thf);
    double S = 2.0 * sc_k_max - sin(sc_th0) + sin(sc_thf);
    double temp1 = atan2(C, S);
    double temp2 = 0.125 * ( 6. - 4. * (sc_k_max*sc_k_max)  + 2. * cos(sc_th0 - sc_thf) + 4. * sc_k_max * (sin(sc_th0) - sin(sc_thf)));
    if (abs(temp2) > 1){
        out.sc_s1 = 0;
        out.sc_s2 = 0;
        out.sc_s3 = 0;
        return out;
    }
	out.sc_s2 = sc_k_max_inv * mod2pi(2 * M_PI - acos(temp2));
    out.sc_s1 = sc_k_max_inv * mod2pi(sc_th0-temp1+0.5*sc_s2*sc_k_max);
    out.sc_s3 = sc_k_max_inv * mod2pi(sc_th0-sc_thf+sc_k_max*(sc_s2-sc_s1));
    out.ok = true;
    //check(sc_s1, -sc_k_max, sc_s2, sc_k_max, sc_s3, -sc_k_max, sc_th0, sc_thf);
    return out;
}
ScaledCurveSegments DubinsCurvesHandler::LRL(double sc_th0, double sc_thf, double sc_k_max, double sc_k_max_inv)
{
    ScaledCurveSegments out{};

    double C = cos(sc_thf) - cos(sc_th0);
    double S = 2.0 * sc_k_max + sin(sc_th0) - sin(sc_thf);
    double temp1 = atan2(C, S);
    double temp2 = 0.125 * ( 6. - 4. * (sc_k_max*sc_k_max)  + 2. * cos(sc_th0 - sc_thf) - 4. * sc_k_max * (sin(sc_th0) - sin(sc_thf)));
    if (abs(temp2) > 1){
        out.sc_s1 = 0;
        out.sc_s2 = 0;
        out.sc_s3 = 0;
        return out;
    }
	out.sc_s2 = sc_k_max_inv * mod2pi(2 * M_PI - acos(temp2));
    out.sc_s1 = sc_k_max_inv * mod2pi(temp1-sc_th0+0.5*sc_s2*sc_k_max);
    out.sc_s3 = sc_k_max_inv * mod2pi(sc_thf-sc_th0+sc_k_max*(sc_s2-sc_s1));
    out.ok = true;
    //check(sc_s1, sc_k_max, sc_s2, -sc_k_max, sc_s3, sc_k_max, sc_th0, sc_thf);
    return out;
}

DubinsCurve DubinsCurvesHandler::findShortestPath(double x0, double y0, double th0, double xf, double yf, double thf)
{
    ScaledParams s = scaleToStandard(x0, y0, th0, xf, yf, thf);

    ScaledCurveSegments (*)(double sc_th0, double sc_thf, double sc_k_max, double sc_k_max_inv)[6] primitives = {LSL, RSR, LSR, RSL, RLR, LRL}

    pidx = -1;
    double L = INFINITY, Lcur, cur_s1, cur_s2, cur_s3;
    for (int i = 0; i < 6; i++)
    {
        ScaledCurveSegments params = primitives[i](s.sc_th0, s.sc_thf, s.sc_k_max, s.sc_k_max_inv);
        Lcur = params.s1 + params.s2 + params.s3;
        if (params.ok && Lcur < L){
            L = Lcur;
            cur_s1 = params.s1;
            cur_s2 = params.s2;
            cur_s3 = params.s3;
            pidx = i;
        }
    }

    if (pidx > 0){
        ScaledCurveSegments p;
        p.s1 = cur_s1;
        p.s2 = cur_s2;
        p.s3 = cur_s3;
        ScaledCurveSegments segment = scaleFromStandard(p, s.lambda);

        return computeDubinsCurve(x0, y0, th0, segment.s1, segment.s2, segment.s3, curves_arguments[pidx][0]*s.sc_k_max, curves_arguments[pidx][1]*s.sc_k_max, curves_arguments[pidx][2]*s.sc_k_max);
    }
    return NULL;
}