#include "pathsmooth.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace {

inline double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

inline double dist(double x1, double y1, double x2, double y2) {
    const double dx = x1 - x2, dy = y1 - y2;
    return std::sqrt(dx*dx + dy*dy);
}

inline double segDist(const Pt& a, const Pt& b) {
    return dist(a.x, a.y, b.x, b.y);
}

// thomas algorithm for tridiagonal system
void solve_tridiagonal(const std::vector<double>& a,
                       const std::vector<double>& b,
                       const std::vector<double>& c,
                       std::vector<double>& d) {
    const size_t n = b.size();
    if (n == 0) return;

    std::vector<double> cp(n, 0.0), dp(n, 0.0);
    cp[0] = (n > 1) ? (c[0] / b[0]) : 0.0;
    dp[0] = d[0] / b[0];

    for (size_t i = 1; i < n; ++i) {
        double denom = b[i] - a[i] * cp[i - 1];
        cp[i] = (i < n - 1) ? (c[i] / denom) : 0.0;
        dp[i] = (d[i] - a[i] * dp[i - 1]) / denom;
    }

    d[n - 1] = dp[n - 1];
    for (size_t i = n - 1; i-- > 0; ) {
        d[i] = dp[i] - cp[i] * d[i + 1];
    }
}

// compute 2nd derivs
std::vector<double> natural_cubic_second_derivs(const std::vector<double>& t,
                                                const std::vector<double>& y) {
    const size_t n = y.size();
    std::vector<double> m(n, 0.0);
    if (n <= 2) return m;

    std::vector<double> a(n, 0.0), b(n, 0.0), c(n, 0.0), d(n, 0.0);
    
    b[0] = 1.0; b[n-1] = 1.0; d[0] = 0.0; d[n-1] = 0.0;

    for (size_t i = 1; i + 1 < n; ++i) {
        const double h_i   = t[i]   - t[i-1];
        const double h_ip1 = t[i+1] - t[i];
        a[i] = h_i;
        b[i] = 2.0 * (h_i + h_ip1);
        c[i] = h_ip1;
        d[i] = 6.0 * ((y[i+1] - y[i]) / h_ip1 - (y[i] - y[i-1]) / h_i);
    }

    solve_tridiagonal(a, b, c, d); 
    m = d;
    return m;
}

inline double spline_eval(double u,
                          double t_i, double t_ip1,
                          double y_i, double y_ip1,
                          double m_i, double m_ip1) {
    const double h = t_ip1 - t_i;
    const double A = (t_ip1 - u) / h;
    const double B = (u - t_i) / h;
    return A * y_i + B * y_ip1
         + ((A*A*A - A) * m_i + (B*B*B - B) * m_ip1) * (h*h) / 6.0;
}

void smooth_pass(std::vector<Pt>& pts) {
    if (pts.size() < 3) return;
    std::vector<Pt> out = pts;
    for (size_t i = 1; i + 1 < pts.size(); ++i) {
        out[i].x = 0.25 * pts[i-1].x + 0.5 * pts[i].x + 0.25 * pts[i+1].x;
        out[i].y = 0.25 * pts[i-1].y + 0.5 * pts[i].y + 0.25 * pts[i+1].y;
    }
    pts.swap(out);
}

} 

// public api

std::vector<Pt> LoadWaypointsCsv(const std::string& path, bool has_header) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Failed to open input CSV: " + path);

    std::vector<Pt> pts;
    std::string line;

    if (has_header && std::getline(in, line)) {
        // skip header
    }
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string xs, ys;
        if (!std::getline(ss, xs, ',')) continue;
        if (!std::getline(ss, ys, ',')) continue;
        Pt p{std::stod(xs), std::stod(ys)};
        pts.push_back(p);
    }
    return pts;
}

void SaveCsvXY(const std::string& path, const std::vector<Pt>& pts) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("Failed to open output CSV: " + path);
    out << "x,y\n";
    for (const auto& p : pts) out << p.x << "," << p.y << "\n";
}

std::vector<Pt> SmoothPathBSpline(const std::vector<Pt>& waypoints,
                                  const SmoothParams& params) {
    if (waypoints.size() < 2) return waypoints;

    std::vector<Pt> wps = waypoints;
    for (int k = 0; k < std::max(0, params.smoothing_passes); ++k) {
        smooth_pass(wps);
    }

    const size_t n = wps.size();
    std::vector<double> t(n, 0.0);
    double total = 0.0;
    for (size_t i = 1; i < n; ++i) {
        total += segDist(wps[i], wps[i-1]);
        t[i] = total;
    }
    if (total <= 0.0) {
        return std::vector<Pt>(std::max(2, params.num_points), wps.front());
    }
    for (double& v : t) v /= total;

    std::vector<double> x(n), y(n);
    for (size_t i = 0; i < n; ++i) { x[i] = wps[i].x; y[i] = wps[i].y; }
    const std::vector<double> mx = natural_cubic_second_derivs(t, x);
    const std::vector<double> my = natural_cubic_second_derivs(t, y);

    // sample uniformly
    const int M = std::max(2, params.num_points);
    std::vector<Pt> out;
    out.reserve(M);

    for (int j = 0; j < M; ++j) {
        const double u = (M == 1) ? 0.0 : static_cast<double>(j) / static_cast<double>(M - 1);

        size_t i = static_cast<size_t>(std::lower_bound(t.begin(), t.end(), u) - t.begin());
        if (i == 0) i = 1;
        if (i >= n) i = n - 1;

        double xu = spline_eval(u, t[i-1], t[i], x[i-1], x[i], mx[i-1], mx[i]);
        double yu = spline_eval(u, t[i-1], t[i], y[i-1], y[i], my[i-1], my[i]);

        xu = clamp(xu, params.xmin, params.xmax);
        yu = clamp(yu, params.ymin, params.ymax);

        out.push_back({xu, yu});
    }

    return out;
}