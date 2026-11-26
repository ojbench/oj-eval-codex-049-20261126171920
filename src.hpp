#ifndef PPCA_SRC_HPP
#define PPCA_SRC_HPP
#include "math.h"
#include <vector>
#include <algorithm>

class Controller {
public:
    Controller(const Vec &_pos_tar, double _v_max, double _r, int _id, Monitor *_monitor) {
        pos_tar = _pos_tar;
        v_max = _v_max;
        r = _r;
        id = _id;
        monitor = _monitor;
    }
    void set_pos_cur(const Vec &_pos_cur) { pos_cur = _pos_cur; }
    void set_v_cur(const Vec &_v_cur) { v_cur = _v_cur; }

private:
    int id;
    Vec pos_tar, pos_cur, v_cur;
    double v_max, r;
    Monitor *monitor;
    Vec progress_pos;
    int ticks = 0, still_ticks = 0, escape_left = 0, escape_cooldown = 0;
    std::vector<Vec> route;
    Vec yield_target;
    Vec escape_target;
    int yield_left = 0, yield_cooldown = 0;

    void plan_route() {
        struct Disk { Vec c; double radius; };
        std::vector<Disk> disks;
        std::vector<Vec> nodes{pos_cur, pos_tar};
        for (int j = 0; j < monitor->get_robot_number(); ++j) {
            if (j == id) continue;
            Disk disk{monitor->get_pos_cur(j), r + monitor->get_r(j)};
            disks.push_back(disk);
            for (int k = 0; k < 20; ++k) {
                double a = 2 * PI * k / 20;
                nodes.push_back(disk.c + Vec(std::cos(a), std::sin(a)) * (disk.radius * 1.025 + 0.001));
            }
        }
        auto clear = [&](Vec a, Vec b) {
            Vec delta = b - a;
            for (auto disk : disks) {
                double t = std::max(0.0, std::min(1.0, (disk.c - a).dot(delta) / std::max(sq(delta), 1e-15)));
                if (sq(a + delta * t - disk.c) < disk.radius * disk.radius - 1e-7) return false;
            }
            return true;
        };
        int size = nodes.size();
        std::vector<double> dist(size, 1e100);
        std::vector<int> prev(size, -1);
        std::vector<bool> used(size, false);
        dist[0] = 0;
        for (int step = 0; step < size; ++step) {
            int u = -1;
            for (int i = 0; i < size; ++i)
                if (!used[i] && (u == -1 || dist[i] < dist[u])) u = i;
            if (u == -1 || dist[u] > 1e99 || u == 1) break;
            used[u] = true;
            for (int v = 0; v < size; ++v) {
                if (used[v]) continue;
                double next = dist[u] + (nodes[u] - nodes[v]).norm();
                if (next < dist[v] && clear(nodes[u], nodes[v])) {
                    dist[v] = next;
                    prev[v] = u;
                }
            }
        }
        route.clear();
        if (prev[1] != -1)
            for (int u = 1; u != 0; u = prev[u]) route.push_back(nodes[u]);
    }

    struct Plane { Vec n; double b; }; // Feasible velocities satisfy n dot v <= b.
    double sq(const Vec &v) const { return v.dot(v); }
    Vec limit(Vec v) const {
        double length = std::sqrt(sq(v));
        return length > v_max ? v * (v_max / length) : v;
    }

    // The optimum over a disk clipped by half-planes lies at the preferred
    // point, a projected edge point, or an intersection of boundaries.
    bool solve(const std::vector<Plane> &planes, Vec preferred, Vec &answer) const {
        double best = 1e100;
        auto consider = [&](Vec v) {
            if (sq(v) > v_max * v_max + 1e-8) return;
            for (const auto &p : planes)
                if (p.n.dot(v) > p.b + 1e-8) return;
            double cost = sq(v - preferred);
            if (cost < best) { best = cost; answer = v; }
        };
        consider(limit(preferred));
        consider(Vec());
        for (std::size_t i = 0; i < planes.size(); ++i) {
            const auto &p = planes[i];
            consider(preferred - p.n * (p.n.dot(preferred) - p.b));
            double disc = v_max * v_max - p.b * p.b;
            if (disc >= 0) {
                Vec side(-p.n.y, p.n.x);
                consider(p.n * p.b + side * std::sqrt(disc));
                consider(p.n * p.b - side * std::sqrt(disc));
            }
            for (std::size_t j = 0; j < i; ++j) {
                const auto &q = planes[j];
                double det = p.n.cross(q.n);
                if (std::abs(det) > 1e-10)
                    consider(Vec((p.b * q.n.y - q.b * p.n.y) / det,
                                 (p.n.x * q.b - q.n.x * p.b) / det));
            }
        }
        return best < 1e99;
    }

public:
    Vec get_v_next() {
        Vec goal = pos_tar - pos_cur;
        if (ticks++ % 40 == 0) {
            if (ticks > 1 && goal.norm() > 0.02 &&
                (pos_cur - progress_pos).norm() < v_max * TIME_INTERVAL * 4)
                plan_route();
            progress_pos = pos_cur;
        }
        while (!route.empty() && (route.back() - pos_cur).norm() < std::max(0.02, v_max * TIME_INTERVAL))
            route.pop_back();
        if (!route.empty()) goal = route.back() - pos_cur;
        if (yield_cooldown > 0) --yield_cooldown;
        if (yield_left > 0) {
            --yield_left;
            goal = yield_target - pos_cur;
        } else if (yield_cooldown == 0) {
            for (int j = 0; j < monitor->get_robot_number(); ++j) {
                if (j == id || monitor->get_r(j) < r * 1.2) continue;
                Vec away = pos_cur - monitor->get_pos_cur(j);
                double radius = r + monitor->get_r(j);
                if (away.norm() < radius * 1.08 && monitor->get_v_cur(j).norm() < v_max * 0.1) {
                    yield_target = pos_cur + away.normalize() * radius * 0.6;
                    yield_left = std::max(80, int(radius * 0.6 / std::max(v_max * TIME_INTERVAL, 0.001)));
                    yield_cooldown = yield_left + 80;
                    route.clear();
                    goal = yield_target - pos_cur;
                    break;
                }
            }
        }
        // All controllers observe the same public snapshot. A prolonged global
        // standstill triggers a brief outward expansion, opening escape gaps.
        Vec center;
        double motion = 0, scale = 0;
        int count = monitor->get_robot_number();
        for (int j = 0; j < count; ++j) {
            center += monitor->get_pos_cur(j);
            motion += monitor->get_v_cur(j).norm();
            scale += monitor->get_r(j);
        }
        center = center / count;
        if (escape_cooldown > 0) --escape_cooldown;
        if (motion < std::max(0.0001, scale * 0.0001)) ++still_ticks;
        else still_ticks = 0;
        if (still_ticks >= 15 && ticks > 60 && escape_cooldown == 0) {
            Vec outward = pos_cur - center;
            if (outward.norm() < 0.001) {
                double angle = 2 * PI * id / count;
                outward = Vec(std::cos(angle), std::sin(angle));
            }
            escape_target = pos_cur + outward.normalize() * std::max(scale, 1.0);
            escape_left = 100;
            escape_cooldown = 500;
            route.clear();
        }
        if (escape_left > 0) {
            --escape_left;
            goal = escape_target - pos_cur;
        }
        Vec preferred = limit(goal / TIME_INTERVAL);
        Vec velocity = monitor->get_v_cur(id);
        std::vector<Plane> safe, planes;
        double urgency = 0;
        for (int j = 0; j < monitor->get_robot_number(); ++j) {
            if (j == id) continue;
            Vec p = monitor->get_pos_cur(j) - pos_cur;
            double d = std::sqrt(sq(p));
            if (d < 1e-12) continue;
            double radius = r + monitor->get_r(j);
            Vec n = p / d;
            // Both robots reserve half the current separating gap. This
            // guarantees separation throughout the complete next time step.
            safe.push_back({n, std::max(0.0, d - radius) / (2 * TIME_INTERVAL)});
            Vec relative = velocity - monitor->get_v_cur(j);
            double horizon = std::max(1.0, 2.0 * radius / std::max(v_max, 0.01));
            double inv = 1 / horizon;
            Vec w = relative - p * inv;
            double w2 = sq(w), dot = w.dot(p);
            Vec direction, correction;
            if (dot < 0 && dot * dot > radius * radius * w2) {
                double length = std::sqrt(w2);
                Vec unit = length > 1e-12 ? w / length : -n;
                direction = Vec(unit.y, -unit.x);
                correction = unit * (radius * inv - length);
            } else {
                double leg = std::sqrt(std::max(0.0, d * d - radius * radius));
                if (p.cross(w) > 0)
                    direction = Vec(p.x * leg - p.y * radius, p.x * radius + p.y * leg) / (d * d);
                else
                    direction = -Vec(p.x * leg + p.y * radius, -p.x * radius + p.y * leg) / (d * d);
                correction = direction * relative.dot(direction) - relative;
            }
            Vec normal(direction.y, -direction.x);
            planes.push_back({normal, normal.dot(velocity + correction * 0.5)});
            Vec approach = preferred - monitor->get_v_cur(j);
            double t = std::max(0.0, std::min(horizon, p.dot(approach) / std::max(sq(approach), 1e-12)));
            if (sq(p - approach * t) < radius * radius * 1.3 && p.dot(goal) > 0)
                urgency = std::max(urgency, std::min(1.0, radius * 2 / d));
        }
        preferred = limit(preferred + Vec(preferred.y, -preferred.x) * (0.25 * urgency));
        planes.insert(planes.end(), safe.begin(), safe.end());
        Vec result;
        if (!solve(planes, preferred, result)) solve(safe, preferred, result);
        return limit(result);
    }
};
#endif
