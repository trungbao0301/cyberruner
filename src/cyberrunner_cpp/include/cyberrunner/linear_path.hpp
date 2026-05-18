#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace cyberrunner
{

struct PathPoint
{
    double x = 0.0;
    double y = 0.0;
};

class LinearPath
{
public:
    explicit LinearPath(double distance_px = 2.0)
    : distance_px_(std::max(distance_px, 1e-6))
    {
    }

    void setDistance(double distance_px)
    {
        distance_px_ = std::max(distance_px, 1e-6);
    }

    void build(const std::vector<PathPoint>& waypoints)
    {
        orig_waypoints_ = waypoints;
        points_.clear();

        if (waypoints.size() < 2) return;

        for (size_t i = 1; i < waypoints.size(); ++i) {
            const auto& a = waypoints[i - 1];
            const auto& b = waypoints[i];
            const double dx = b.x - a.x;
            const double dy = b.y - a.y;
            const double len = std::hypot(dx, dy);
            const int count = std::max(1, static_cast<int>(std::floor(len / distance_px_)) + 1);

            for (int j = 0; j < count; ++j) {
                const double t = static_cast<double>(j) / static_cast<double>(count);
                points_.push_back({a.x + t * dx, a.y + t * dy});
            }
        }

        points_.push_back(waypoints.back());
    }

    bool valid() const
    {
        return !points_.empty();
    }

    size_t size() const
    {
        return points_.size();
    }

    double distance() const
    {
        return distance_px_;
    }

    const std::vector<PathPoint>& points() const
    {
        return points_;
    }

    std::pair<int, PathPoint> closestPoint(double x, double y) const
    {
        if (points_.empty()) return {-1, {}};

        int best_idx = -1;
        double best_d2 = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < points_.size(); ++i) {
            const double dx = points_[i].x - x;
            const double dy = points_[i].y - y;
            const double d2 = dx * dx + dy * dy;
            if (d2 < best_d2) {
                best_d2 = d2;
                best_idx = static_cast<int>(i);
            }
        }

        return {best_idx, points_[best_idx]};
    }

    std::vector<PathPoint> relativePath(double x, double y, int count, int step) const
    {
        std::vector<PathPoint> rel;
        rel.reserve(std::max(0, count));

        if (points_.empty() || count <= 0) return rel;

        const auto [idx, unused] = closestPoint(x, y);
        (void)unused;
        if (idx < 0) {
            for (int i = 0; i < count; ++i) rel.push_back({});
            return rel;
        }

        step = std::max(step, 1);
        for (int i = 0; i < count; ++i) {
            const int target_idx = std::clamp(
                idx + i * step,
                0,
                static_cast<int>(points_.size()) - 1);
            rel.push_back({points_[target_idx].x - x, points_[target_idx].y - y});
        }
        return rel;
    }

private:
    double distance_px_ = 2.0;
    std::vector<PathPoint> orig_waypoints_;
    std::vector<PathPoint> points_;
};

}  // namespace cyberrunner
