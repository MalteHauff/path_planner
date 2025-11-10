/********************************************************************************
 * hybrid_A_star.h -- Two-stage planning (strict then relaxed) + Node extras
 *  - Stage 1: rectangle footprint collision, multi-turn primitives
 *  - Stage 2: if no path, switch to center-only collision & retry
 *  - Returns adore::TrajectoryVector (vector<Node_Lite>) as GraphSearchNode expects
 ********************************************************************************/
#pragma once

#include <cmath>
#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include <cstdint>
#include <functional>
#include <algorithm>
#include <eigen3/Eigen/Dense>

#include "../env/path.h"
#include "../env/node.h"
#include "../env/occupancy_grid.h"
#include "../env/search_grid.h"
#include "collision_check_offline.h"

namespace adore {
namespace fun {

class Hybrid_A_Star
{
public:
    using TrajectoryVector = adore::TrajectoryVector; // vector<Node_Lite>

    explicit Hybrid_A_Star(bool /*enable_smoothing*/ = false) {}
    void setSize(std::uint32_t /*H*/, std::uint32_t /*W*/) {}

    TrajectoryVector plan(
        GRID<Node<3,double>>* /*grid*/,
        env::OccupanyGrid* og,
        adore::fun::CollisionCheckOffline* /*cco*/,
        Node<3,double>* Start,
        Node<3,double>* End,
        int HeadingResolutionDeg,
        int maxIterations,
        double vehicleWidth_m, double vehicleLength_m)
    {
        // inner lambda that runs one planning stage:
        // relaxed=false → full rectangular collision
        // relaxed=true  → center-only collision (failsafe)
        auto run = [&](bool relaxed)->TrajectoryVector {
            // --- configure discretization and node static settings ---
            heading_res_rad_ = (static_cast<double>(HeadingResolutionDeg) * M_PI) / 180.0;
            Node<3,double>::setFootprintMeters(vehicleWidth_m, vehicleLength_m);
            Node<3,double>::setCenterOnlyCollision(relaxed);
            Node<3,double>::enableExtraTurns(true);
            Node<3,double>::allowReverse(false); // set true if reversing should be allowed

            // Choose a curvature radius so that a full heading bin corresponds to ~1 cell arc
            const double dA = std::max(1e-6, heading_res_rad_);
            const double R  = 1.0 / dA;

            // Copy start/goal so stage 2 can restart cleanly
            Node<3,double> S = *Start;
            Node<3,double> G = *End;
            S.set_R(R); S.set_dA(dA); S.C = 0.0;

            // --- open set (min-heap on f = g + h) ---
            struct Cmp {
                bool operator()(const Node<3,double>* a, const Node<3,double>* b) const {
                    return a->C > b->C;
                }
            };
            std::priority_queue<Node<3,double>*, std::vector<Node<3,double>*>, Cmp> open;

            // --- A* bookkeeping ---
            std::unordered_map<std::uint64_t, Node<3,double>*> registry;  // key -> node*
            std::unordered_map<std::uint64_t, std::uint64_t>    parent;    // child key -> parent key
            std::unordered_map<std::uint64_t, double>           gscore;    // key -> g

            const auto key_of = [this](const Node<3,double>* n) -> std::uint64_t {
                const int xi = static_cast<int>(std::floor(n->x));
                const int yi = static_cast<int>(std::floor(n->y));
                const int hi = heading_index(n->psi);
                return  (static_cast<std::uint64_t>(xi & 0xFFFF) << 32)
                      | (static_cast<std::uint64_t>(yi & 0xFFFF) << 16)
                      | (static_cast<std::uint64_t>(hi & 0xFFFF));
            };

            const auto inside = [&](const Node<3,double>* n)->bool {
                const int row = static_cast<int>(std::floor(n->y));
                const int col = static_cast<int>(std::floor(n->x));
                // Use OccupanyGrid's validity (unknown policy is managed by the caller)
                return og->check_valid_position(row, col);
            };

            const auto hfun = [&](const Node<3,double>* n)->double {
                const double dx = static_cast<double>(n->x) - static_cast<double>(G.x);
                const double dy = static_cast<double>(n->y) - static_cast<double>(G.y);
                const double pos  = std::hypot(dx, dy);
                const double dpsi = angle_diff(n->psi, G.psi);
                return pos + 0.1 * std::abs(dpsi);
            };

            // --- seed ---
            Node<3,double>* S_heap = new Node<3,double>(S);
            const std::uint64_t start_key = key_of(S_heap);
            registry[start_key] = S_heap;
            gscore[start_key]   = 0.0;
            S_heap->C = hfun(S_heap);
            open.push(S_heap);

            int iters = 0;
            while (!open.empty() && iters++ < maxIterations)
            {
                Node<3,double>* cur = open.top();
                open.pop();

                if (is_goal(cur, &G)) {
                    // reconstruct path (world in cell coords + yaw)
                    TrajectoryVector out;
                    auto rec = reconstruct(parent, registry, key_of, cur, S_heap);
                    out.reserve(rec.size());
                    for (const auto& v : rec) {
                        adore::Node_Lite n;
                        n.x = v.x(); n.y = v.y(); n.psi = v.z(); n.s = 0.0;
                        out.push_back(n);
                    }
                    for (auto &kv : registry) if (kv.second) delete kv.second;
                    return out;
                }

                // expand neighbors from motion primitives
                std::vector<Node<3,double>*> succs = cur->updateSuccessorsNH();
                for (Node<3,double>* s : succs) {
                    s->set_R(R);
                    s->set_dA(dA);

                    // bounds / feasibility
                    if (!inside(s)) { delete s; continue; }

                    // edge + state collision (rectangle or center depending on 'relaxed')
                    if (!cur->isEdgeCollisionFreeRectTo(og, s) || !s->isCollisionFreeRect(og)) {
                        delete s; continue;
                    }

                    const std::uint64_t sk = key_of(s);
                    const auto curk = key_of(cur);
                    const double tentative_g = gscore[curk] + step_cost(cur, s);

                    auto itg = gscore.find(sk);
                    if (itg != gscore.end() && tentative_g >= itg->second) {
                        delete s; continue;
                    }

                    registry[sk] = s;
                    parent[sk]   = curk;
                    gscore[sk]   = tentative_g;
                    s->C         = tentative_g + hfun(s);
                    open.push(s);
                }
            }

            for (auto &kv : registry) if (kv.second) delete kv.second;
            return TrajectoryVector{};
        };

        // Stage 1: strict rectangular collision
        auto res = run(false);
        if (!res.empty()) return res;

        // Stage 2: relaxed center-only collision (GraphSearchNode will still do a final hard check)
        return run(true);
    }

private:
    double heading_res_rad_{M_PI/12.0};

    int heading_index(double psi) const
    {
        // normalize to [0, 2π)
        while (psi < 0.0) psi += 2.0 * M_PI;
        while (psi >= 2.0 * M_PI) psi -= 2.0 * M_PI;

        const double bin  = std::max(1e-6, heading_res_rad_);
        const int    bins = std::max(1, static_cast<int>(std::round((2.0 * M_PI) / bin)));
        int idx = static_cast<int>(std::round(psi / bin));
        if (idx >= bins) idx = bins - 1;
        if (idx < 0)     idx = 0;
        return idx;
    }

    static double angle_diff(double a, double b)
    {
        double d = a - b;
        while (d >  M_PI) d -= 2.0*M_PI;
        while (d < -M_PI) d += 2.0*M_PI;
        return d;
    }

    bool is_goal(const Node<3,double>* a, const Node<3,double>* b) const
    {
        const double dx = static_cast<double>(a->x) - static_cast<double>(b->x);
        const double dy = static_cast<double>(a->y) - static_cast<double>(b->y);
#ifndef GOAL_TOLERANCE_CELLS
#define GOAL_TOLERANCE_CELLS 3.0
#endif
        return (dx*dx + dy*dy) <= (GOAL_TOLERANCE_CELLS * GOAL_TOLERANCE_CELLS);
    }

    static double step_cost(const Node<3,double>* a, const Node<3,double>* b)
    {
        const double dx   = static_cast<double>(b->x) - static_cast<double>(a->x);
        const double dy   = static_cast<double>(b->y) - static_cast<double>(a->y);
        const double dpsi = angle_diff(b->psi, a->psi);
        return std::hypot(dx, dy) + 0.03 * std::abs(dpsi);
    }

    template<typename KeyOf>
    static std::vector<Eigen::Vector3d> reconstruct(
        const std::unordered_map<std::uint64_t, std::uint64_t>& parent,
        const std::unordered_map<std::uint64_t, Node<3,double>*>& registry,
        KeyOf key_of,
        Node<3,double>* goal,
        Node<3,double>* start)
    {
        std::vector<Eigen::Vector3d> out;
        out.reserve(1024);

        std::uint64_t cur = key_of(goal);
        const std::uint64_t s  = key_of(start);

        while (true) {
            auto itn = registry.find(cur);
            if (itn == registry.end()) break;
            Node<3,double>* n = itn->second;
            out.emplace_back(static_cast<double>(n->x), static_cast<double>(n->y), n->psi);
            if (cur == s) break;
            auto itp = parent.find(cur);
            if (itp == parent.end()) break;
            cur = itp->second;
        }

        std::reverse(out.begin(), out.end());
        return out;
    }
};

} // namespace fun
} // namespace adore
