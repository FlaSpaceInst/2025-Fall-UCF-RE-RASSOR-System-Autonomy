#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "pathsmooth.hpp"

static double dist(const Pt& a, const Pt& b) {
    double dx=a.x-b.x, dy=a.y-b.y; return std::sqrt(dx*dx+dy*dy);
}
static double totalLen(const std::vector<Pt>& v){
    double L=0; for(size_t i=1;i<v.size();++i) L+=dist(v[i],v[i-1]); return L;
}

TEST(PathSmooth, CountAndBounds) {
    std::vector<Pt> wps = {{0,0},{5,1},{10,0},{15,-1},{20,0},{25,2},{30,0},{35,-2},{40,0}};
    SmoothParams p; p.num_points=300; p.smoothing_passes=4;
    auto out = SmoothPathBSpline(wps, p);
    ASSERT_GE(out.size(), 270u);  // ~0.9*num_points
    for (auto& q: out) {
        EXPECT_GE(q.x, p.xmin); EXPECT_LE(q.x, p.xmax);
        EXPECT_GE(q.y, p.ymin); EXPECT_LE(q.y, p.ymax);
    }
}

TEST(PathSmooth, DerivativeBoundedAndSmooth) {
    std::vector<Pt> wps = {{0,0},{5,1},{10,0},{15,-1},{20,0},{25,2},{30,0},{35,-2},{40,0}};
    SmoothParams p; p.num_points=300; p.smoothing_passes=4;
    auto out = SmoothPathBSpline(wps, p);
    ASSERT_GE(out.size(), 3u);

    double L = totalLen(out);
    ASSERT_GT(L, 1e-8);
    double avg = L/(out.size()-1);

    const double kMaxStepMult = 1.8;
    const double kMaxAccelMult = 3.0;

    std::vector<double> step(out.size()-1);
    int zeroes=0;
    for(size_t i=1;i<out.size();++i){
        step[i-1] = dist(out[i], out[i-1]);
        if (step[i-1] < 1e-12) ++zeroes;
        ASSERT_LE(step[i-1], kMaxStepMult*avg) << "spike at i="<<i;
    }
    ASSERT_LT(zeroes, static_cast<int>(out.size()*0.02));

    for(size_t i=1;i<step.size();++i){
        double accel = std::fabs(step[i]-step[i-1]);
        ASSERT_LE(accel, kMaxAccelMult*avg) << "jerk at i="<<i;
    }
}

TEST(PathSmooth, EndpointsNearOriginal) {
    std::vector<Pt> wps = {{0,0},{10,0},{20,5},{30,0},{40,0}};
    SmoothParams p; p.num_points=200; p.smoothing_passes=2;
    auto out = SmoothPathBSpline(wps, p);
    ASSERT_GE(out.size(), 2u);
    double eps=0.8; // tune as needed
    EXPECT_LE(dist(out.front(), wps.front()), eps);
    EXPECT_LE(dist(out.back(),  wps.back()),  eps);
}
