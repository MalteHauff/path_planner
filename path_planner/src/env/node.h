/********************************************************************************
 * node.h  -- Non-holonomic node with multi-action primitives and runtime
 *            collision policy toggles (rectangle vs center-only).
 *
 *  - Successors: straight, ±dA arcs, optional ±0.5*dA (gentle) and ±1.5*dA (sharper)
 *  - Optional reverse straight (off by default)
 *  - Runtime toggle: center-only collision (fallback stage in planner)
 *  - API kept compatible with GraphSearchNode & Hybrid_A_Star
 ********************************************************************************/
#pragma once

#include "../algo/collision_check_offline.h"  // only for signature parity in updateSuccessors3D
#include "occupancy_grid.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <type_traits>
#include <vector>

namespace adore {
namespace fun {

    template <int TYPE, typename T>
    class Node
    {
    private:
        // Keep a small 2D holonomic set for API parity (unused by 3D pipeline)
        static constexpr int N_suc = 3;
        double dx_H[N_suc]{1.0, 1.0, 1.0};
        double dy_H[N_suc]{ 1.0, 0.0,-1.0};
        double dpsi_H[N_suc]{ M_PI/4.0, 0.0, -M_PI/4.0 };

        // Non-holonomic parameters (cells/rad)
        double R{2.0};     // radius proxy used with dA to produce ~1 cell forward
        double dA{M_PI/4.0}; // primary heading increment (rad)

        // Costs (optional storage)
        double G{0.0}, H{0.0};

        int type{TYPE};

        static constexpr double kPI   = 3.14159265358979323846;
        static constexpr double kTWO_PI = 2.0 * kPI;

        static double wrapToPi(double a)
        {
            while (a >  kPI) a -= kTWO_PI;
            while (a < -kPI) a += kTWO_PI;
            return a;
        }
        static double lerpAngle(double a, double b, double t)
        {
            const double da = wrapToPi(b - a);
            return wrapToPi(a + t * da);
        }
        static double angle_to_pipi(double a) { return wrapToPi(a); }

        // --- rectangular footprint raster check in CELL coordinates ---
        static bool poseFootprintIsFreeCells(const adore::env::OccupanyGrid* og,
                                             double cx, double cy, double psi,
                                             double width_cells, double length_cells)
        {
            const double hw = 0.5 * std::max(1.0, width_cells);
            const double hl = 0.5 * std::max(1.0, length_cells);

            const int xmin = static_cast<int>(std::floor(-hl));
            const int xmax = static_cast<int>(std::ceil( hl));
            const int ymin = static_cast<int>(std::floor(-hw));
            const int ymax = static_cast<int>(std::ceil( hw));

            const double c = std::cos(psi);
            const double s = std::sin(psi);

            for (int ix = xmin; ix <= xmax; ++ix) {
                for (int iy = ymin; iy <= ymax; ++iy) {
                    // rotate body offset -> grid offset
                    const double rx = ix * c - iy * s;
                    const double ry = ix * s + iy * c;

                    const double gx = cx + rx;
                    const double gy = cy + ry;

                    const int row = static_cast<int>(std::floor(gy));
                    const int col = static_cast<int>(std::floor(gx));

                    if (!og->check_valid_position(row, col)) {
                        return false;
                    }
                }
            }
            return true;
        }

        // Edge collision by sampling between two poses (cell space)
        bool edgeCollisionFreeRect(const adore::env::OccupanyGrid* og,
                                   double x0, double y0, double p0,
                                   double x1, double y1, double p1) const
        {
            // Fallback: center-only validity at endpoints
            if (s_center_only_collision)
            {
                const int r0 = static_cast<int>(std::floor(y0));
                const int c0 = static_cast<int>(std::floor(x0));
                const int r1 = static_cast<int>(std::floor(y1));
                const int c1 = static_cast<int>(std::floor(x1));
                return og->check_valid_position(r0, c0) && og->check_valid_position(r1, c1);
            }

            // Convert robot footprint from meters to cells
            const double res = std::max(1e-9, og->resolution);
            const double width_cells_raw  = s_footprint_w_m / res;
            const double length_cells_raw = s_footprint_l_m / res;
#ifndef COLLISION_SHRINK_CELLS
#define COLLISION_SHRINK_CELLS 1.0
#endif
            const double width_cells  = std::max(1.0, width_cells_raw  - 2.0*COLLISION_SHRINK_CELLS);
            const double length_cells = std::max(1.0, length_cells_raw - 2.0*COLLISION_SHRINK_CELLS);

            const double dx = x1 - x0;
            const double dy = y1 - y0;

#ifndef COLLISION_STEP_CELLS
#define COLLISION_STEP_CELLS 1.5
#endif
            const double step_cells = COLLISION_STEP_CELLS;
            const int steps = std::max(1, static_cast<int>(std::ceil(std::max(std::fabs(dx), std::fabs(dy)) / step_cells)));

            for (int s = 0; s <= steps; ++s)
            {
                const double t   = static_cast<double>(s) / static_cast<double>(steps);
                const double xi  = x0 + t * dx;
                const double yi  = y0 + t * dy;
                const double psi = lerpAngle(p0, p1, t);

                if (!poseFootprintIsFreeCells(og, xi, yi, psi, width_cells, length_cells)) {
                    return false; // any sample collides -> block the edge
                }
            }
            return true;
        }

        // helper to push a successor with a custom delta heading (d) and derived radius (R=1/d)
        void push_arc(std::vector<Node<3,double>*>& out, double d) const
        {
            const double cp = std::cos(psi);
            const double sp = std::sin(psi);
            const double d_abs = std::max(1e-6, std::fabs(d));
            const double Rloc = 1.0 / d_abs;
            const double sign = (d >= 0.0) ? 1.0 : -1.0;
            const double dpsi = sign * d_abs;

            Node<3,double>* tmp = new Node<3,double>;
            tmp->x   = x + Rloc * (std::sin(psi + dpsi) - sp);
            tmp->y   = y + Rloc * (-std::cos(psi + dpsi) + cp);
            tmp->psi = angle_to_pipi(psi + dpsi);
            out.push_back(tmp);
        }

    public:
        // --- Runtime-settable global knobs (per template instantiation) ---
        inline static double s_footprint_w_m = 0.30;
        inline static double s_footprint_l_m = 0.40;
        inline static bool   s_enable_extra_turns      = true;  // adds ±0.5*dA and ±1.5*dA arcs
        inline static bool   s_allow_reverse           = false; // adds backward straight step
        inline static bool   s_center_only_collision   = false; // collision fallback

        static void setFootprintMeters(double w, double l) {
            s_footprint_w_m = std::max(0.05, w);
            s_footprint_l_m = std::max(0.05, l);
        }
        static void enableExtraTurns(bool v){ s_enable_extra_turns = v; }
        static void allowReverse(bool v){ s_allow_reverse = v; }
        static void setCenterOnlyCollision(bool v){ s_center_only_collision = v; }

        // Public state (kept for compatibility with existing code)
        double C{0.0};
        T x{0};
        T y{0};
        double psi{0.0};
        bool isClosed{false};
        bool isOpen{false};
        bool isVisited{false};
        bool isGloballyVisited{false};
        int index_depth{0};
        int index_width{0};
        int index_length{0};

        Node() = default;

        // Set position helpers (for GraphSearchNode call sites)
        void setPosition(double xx, double yy, double psi_) { x = static_cast<T>(xx); y = static_cast<T>(yy); psi = angle_to_pipi(psi_); }
        void setPosition(int col, int row, double psi_)     { x = static_cast<T>(col); y = static_cast<T>(row); psi = angle_to_pipi(psi_); }

        // 7-arg overload used in GraphSearchNode
        void setPosition(double xx, double yy, double psi_,
                         int Width, int Length, int Depth, double HeadingResolutionRad)
        {
            (void)Width; (void)Length;
            x = static_cast<T>(xx);
            y = static_cast<T>(yy);
            psi = angle_to_pipi(psi_);
            update_index(Width, Length, Depth, HeadingResolutionRad);
        }

        // Motion primitive parameters
        void set_R(double R_)  { R = R_; }
        void set_dA(double dA_){ dA = dA_; }
        void set_G(double G_)  { G = G_; }
        void set_H(double H_)  { H = H_; }

        void update_index(int /*Width*/, int /*Length*/, int Depth, double HeadingResolutionRad)
        {
            int d = static_cast<int>(std::floor(psi / std::max(1e-9, HeadingResolutionRad)));
            d %= Depth;
            if (d < 0) d += Depth;

            index_depth = d;
            index_width  = static_cast<int>(std::floor(x));
            index_length = static_cast<int>(std::floor(y));
        }

        double get_R() const { return R; }
        double get_dA() const { return dA; }
        double get_G() const { return G; }
        double get_H() const { return H; }
        double get_C() const { return C; }

        // ---------- successor generation (holonomic 2D grid, API parity) ----------
        std::vector<Node<2,int>*> updateSuccessorsH()
        {
            std::vector<Node<2,int>*> successors;
            successors.reserve(N_suc);
            for (int i = 0; i < N_suc; ++i) {
                Node<2,int>* tmp = new Node<2,int>;
                tmp->x = static_cast<int>(std::floor(this->x + dx_H[i]));
                tmp->y = static_cast<int>(std::floor(this->y + dy_H[i]));
                tmp->psi = 0.0;
                successors.push_back(tmp);
            }
            return successors;
        }

        // ---------- non-holonomic successors (multi-action) ----------
        std::vector<Node<3,double>*> updateSuccessorsNH() const
        {
            std::vector<Node<3,double>*> successors_nh;
            successors_nh.reserve(9);

            const double cp = std::cos(psi);
            const double sp = std::sin(psi);

            auto push_straight = [&](){
                Node<3,double>* tmp = new Node<3,double>;
                // choose R so that R * dA ≈ 1 cell forward (planner sets R=1/dA)
                tmp->x   = x + R * (cp * dA);
                tmp->y   = y + R * (sp * dA);
                tmp->psi = psi;
                successors_nh.push_back(tmp);
            };

            // forward straight
            push_straight();

            // primary ±dA arcs
            push_arc(successors_nh,  dA);
            push_arc(successors_nh, -dA);

            if (s_enable_extra_turns) {
                // gentler ±0.5*dA arcs (bigger radius)
                push_arc(successors_nh,  0.5*dA);
                push_arc(successors_nh, -0.5*dA);
                // sharper ±1.5*dA arcs (smaller radius)
                push_arc(successors_nh,  1.5*dA);
                push_arc(successors_nh, -1.5*dA);
            }

            if (s_allow_reverse) {
                // straight backward ~1 cell
                Node<3,double>* tmp = new Node<3,double>;
                tmp->x   = x - R * (cp * dA);
                tmp->y   = y - R * (sp * dA);
                tmp->psi = psi;
                successors_nh.push_back(tmp);
            }

            return successors_nh;
        }

        // ---------- 3D pool-fill overload (kept for API parity) ----------
        void updateSuccessors3D(adore::env::OccupanyGrid* /*og*/,
                                adore::fun::CollisionCheckOffline* /*cco*/,
                                double /*HeadingResolutionRad*/,
                                std::vector<Node<3,double>*>& out_successors)
        {
            // Keep minimal behavior for legacy callers; not used by new planner
            if (out_successors.size() < 3) return;
            const double cp = std::cos(psi);
            const double sp = std::sin(psi);
            auto assign = [&](Node<3,double>* dst, double nx, double ny, double npsi)
            { dst->x = nx; dst->y = ny; dst->psi = angle_to_pipi(npsi); };
            assign(out_successors[0], x + R*(cp*dA),                 y + R*(sp*dA),                 psi);
            assign(out_successors[1], x + R*(std::sin(psi+dA)-sp),   y + R*(-std::cos(psi+dA)+cp),  psi+dA);
            assign(out_successors[2], x + R*(-std::sin(psi-dA)+sp),  y + R*( std::cos(psi-dA)-cp),  psi-dA);
        }

        // ---------- collision queries ----------
        bool isCollisionFreeCenter(const adore::env::OccupanyGrid* og) const
        {
            const int row = static_cast<int>(std::floor(y));
            const int col = static_cast<int>(std::floor(x));
            return og->check_valid_position(row, col);
        }

        bool isCollisionFreeRect(const adore::env::OccupanyGrid* og) const
        {
            if (s_center_only_collision) return isCollisionFreeCenter(og);

            const double res = std::max(1e-9, og->resolution);
            const double width_cells_raw  = s_footprint_w_m / res;
            const double length_cells_raw = s_footprint_l_m / res;
#ifndef COLLISION_SHRINK_CELLS
#define COLLISION_SHRINK_CELLS 1.0
#endif
            const double width_cells  = std::max(1.0, width_cells_raw  - 2.0*COLLISION_SHRINK_CELLS);
            const double length_cells = std::max(1.0, length_cells_raw - 2.0*COLLISION_SHRINK_CELLS);

            return poseFootprintIsFreeCells(og, static_cast<double>(x), static_cast<double>(y), psi,
                                            width_cells, length_cells);
        }

        bool isEdgeCollisionFreeRectTo(const adore::env::OccupanyGrid* og,
                                       const Node<3,double>* target) const
        {
            return edgeCollisionFreeRect(og,
                                         static_cast<double>(x), static_cast<double>(y), psi,
                                         static_cast<double>(target->x), static_cast<double>(target->y), target->psi);
        }

        // ---------- helpers ----------
        Node<2,int>* nH2H() const
        {
            Node<2,int>* tmp = new Node<2,int>;
            tmp->x = static_cast<int>(std::floor(this->x));
            tmp->y = static_cast<int>(std::floor(this->y));
            tmp->psi = this->psi;
            return tmp;
        }

        void print() const
        {
            std::cout << "\nnode: " << std::fixed << std::setprecision(6)
                      << static_cast<double>(x) << "\t"
                      << static_cast<double>(y) << "\t"
                      << psi << "\t"
                      << G << "\t"
                      << H << "\t"
                      << C << "\n";
        }
    };

} // namespace fun
} // namespace adore
