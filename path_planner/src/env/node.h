/********************************************************************************
 * node.h  -- cleaned and API-compatible version
 *
 * Keep this file at: src/env/node.h
 *
 * Notes:
 *  - Provides Node<2,int> and Node<3,double> behaviour used across the codebase.
 *  - Adds a pool-friendly overload:
 *        void updateSuccessors3D(OccupanyGrid*, CollisionCheckOffline*, double, std::vector<Node<3,double>*>&)
 *    which fills the provided pointers in-place and marks invalid candidates with x=y=-1.0.
 ********************************************************************************/

#pragma once

#include "../algo/collision_check_offline.h"
#include "../mad/coordinateconversion.h"
#include "../mad/arraymatrixtools.h"
#include "occupancy_grid.h"

#include <cmath>
#include <algorithm>
#include <iostream>
#include <type_traits>
#include <iomanip>
#include <vector>
#include <sstream>

namespace adore {
namespace fun {

    namespace bg = boost::geometry;

    template <int TYPE, typename T>
    class Node
    {
    private:
        static const int N_suc = 3;

        // Holonomic successor offsets
        double dx_H[N_suc];
        double dy_H[N_suc];
        double dpsi_H[N_suc];

        // Non-holonomic parameters
        double R;   // Maximum turning radius
        double dA;  // Dubins Curve Angle

        // Stored costs
        double G;   // cost so far (stored)
        double H;   // cost to go (stored)

        int type;
        double cp; // cos(psi)
        double sp; // sin(psi)
        long double pi, TWO_PI;

        // helper: normalize angle into [0, TWO_PI)
        static inline double normalizeAngle2Pi(double ang, double TWO_PI_local)
        {
            double a = std::fmod(ang, TWO_PI_local);
            if (a < 0.0) a += TWO_PI_local;
            return a;
        }

    public:
        double C;  // total cost G + H
        T x;
        T y;
        double psi;
        bool isClosed;
        bool isOpen;
        bool isVisited;
        bool isGloballyVisited;
        int index_depth;
        int index_width;
        int index_length;

        Node()
        {
            type = TYPE;
            G = 0.0;
            H = 0.0;
            isClosed = false;
            isOpen = false;
            isVisited = false;
            isGloballyVisited = false;

            R = 2.0;
            dA = adore::mad::CoordinateConversion::DegToRad(45.0);

            dx_H[0] = 1.0;
            dx_H[1] = 1.0;
            dx_H[2] = 1.0;

            dy_H[0] = 1.0;
            dy_H[1] = 0.0;
            dy_H[2] = -1.0;

            dpsi_H[0] = adore::mad::CoordinateConversion::DegToRad(45.0);
            dpsi_H[1] = 0.0;
            dpsi_H[2] = adore::mad::CoordinateConversion::DegToRad(-45.0);

            pi = 3.141592653589793;
            TWO_PI = 2.0 * pi;

            index_depth = 0;
            index_width = 0;
            index_length = 0;
            psi = 0.0;
            x = static_cast<T>(0);
            y = static_cast<T>(0);
            C = 0.0;
        }

        void set_R(double R_) { this->R = R_; }
        void set_dA(double dA_) { this->dA = dA_; }
        void set_G(double G_) { this->G = G_; }
        void set_H(double H_) { this->H = H_; }

        // normalize and set indices (clamps to grid bounds)
        void update_index(int Width, int Length, int Depth, double HeadingResolutionRad)
        {
            double psi_norm = normalizeAngle2Pi(this->psi, static_cast<double>(TWO_PI));
            int idx_depth = static_cast<int>(std::floor(psi_norm / HeadingResolutionRad));
            if (idx_depth < 0) idx_depth = 0;
            if (Depth > 0 && idx_depth >= Depth) idx_depth = Depth - 1;
            index_depth = idx_depth;

            // compute integer grid indices via floor (conservative)
            int ix = static_cast<int>(std::floor(static_cast<double>(this->x)));
            int iy = static_cast<int>(std::floor(static_cast<double>(this->y)));

            if (Length > 0) {
                ix = std::max(0, std::min(ix, Length - 1));
            } else {
                ix = 0;
            }
            if (Width > 0) {
                iy = std::max(0, std::min(iy, Width - 1));
            } else {
                iy = 0;
            }

            index_length = ix;
            index_width = iy;
        }

        // safer setPosition with bounds check and psi normalization
        bool setPosition(double x_in, double y_in, double psi_in, int Width, int Length, int Depth, double HeadingResolutionRad)
        {
            const double eps = 1e-9;
            if (!(x_in + eps >= 0.0 && x_in < static_cast<double>(Length) - eps &&
                  y_in + eps >= 0.0 && y_in < static_cast<double>(Width) - eps))
            {
                std::cout << "\nWRONG COORDINATES: x=" << x_in << " y=" << y_in
                          << " (expected x in [0," << Length << "), y in [0," << Width << "))\n";
                return false;
            }

            if constexpr (std::is_integral<T>::value) {
                this->x = static_cast<T>(std::floor(x_in));
                this->y = static_cast<T>(std::floor(y_in));
            } else {
                this->x = static_cast<T>(x_in);
                this->y = static_cast<T>(y_in);
            }

            this->psi = normalizeAngle2Pi(psi_in, static_cast<double>(TWO_PI));
            this->update_index(Width, Length, Depth, HeadingResolutionRad);

            return true;
        }

        bool hasEqualIndex(Node* node, double headingResolution) const
        {
            double psi_a = normalizeAngle2Pi(node->psi, static_cast<double>(TWO_PI));
            double psi_b = normalizeAngle2Pi(this->psi, static_cast<double>(TWO_PI));
            int idx_a = static_cast<int>(std::floor(psi_a / headingResolution));
            int idx_b = static_cast<int>(std::floor(psi_b / headingResolution));

            int ix_a = static_cast<int>(std::floor(static_cast<double>(node->x)));
            int iy_a = static_cast<int>(std::floor(static_cast<double>(node->y)));
            int ix_b = static_cast<int>(std::floor(static_cast<double>(this->x)));
            int iy_b = static_cast<int>(std::floor(static_cast<double>(this->y)));

            return (idx_a == idx_b) && (iy_a == iy_b) && (ix_a == ix_b);
        }

        bool isEqual(Node* node, double headingResolution = 5.0, double tolerance = 0.1) const
        {
            if (this->type == 2) {
                return (this->x == node->x && this->y == node->y);
            }
            if (this->type == 3) {
                double dx = std::abs(static_cast<double>(this->x) - static_cast<double>(node->x));
                double dy = std::abs(static_cast<double>(this->y) - static_cast<double>(node->y));
                double dpsi = std::abs(this->psi - node->psi);
                if (dpsi > TWO_PI - headingResolution) dpsi = TWO_PI - dpsi;
                return (dx < tolerance && dy < tolerance && dpsi <= headingResolution);
            }
            return false;
        }

        bool isCloseTo(Node* node, double tolerance = 10.0) const
        {
            double dx = static_cast<double>(this->x) - static_cast<double>(node->x);
            double dy = static_cast<double>(this->y) - static_cast<double>(node->y);
            return (dx*dx + dy*dy) < (tolerance * tolerance);
        }

        // tentative G (does NOT modify stored G) - returns predecessor->G + step cost
        double get_G(Node* predecessor) const
        {
            double dx = static_cast<double>(this->x) - static_cast<double>(predecessor->x);
            double dy = static_cast<double>(this->y) - static_cast<double>(predecessor->y);
            double EucDist = std::sqrt(dx*dx + dy*dy);
            return predecessor->G + EucDist;
        }

        // stored G getter
        double get_G() const { return this->G; }

        // H computation: Euclidean distance (keeps same units as G)
        double get_H(Node* goal)
        {
            double dx = static_cast<double>(this->x) - static_cast<double>(goal->x);
            double dy = static_cast<double>(this->y) - static_cast<double>(goal->y);
            this->H = std::sqrt(dx*dx + dy*dy);
            return this->H;
        }

        void update_C() { this->C = this->G + this->H; }

        // ---------- 2D succ (allocates) ----------
        std::vector<Node<2,int>*> updateSuccessors2D(adore::env::OccupanyGrid* og)
        {
            int gridWidth = og->Grid.rows();
            int gridLength = og->Grid.cols();

            std::vector<Node<2,int>*> successors_h;
            for (int i = 0; i < N_suc; ++i)
            {
                Node<2,int>* tmp = new Node<2,int>;
                double new_x = static_cast<double>(this->x) + dx_H[i];
                double new_y = static_cast<double>(this->y) + dy_H[i];

                if constexpr (std::is_integral<T>::value) {
                    tmp->x = static_cast<int>(std::floor(new_x));
                    tmp->y = static_cast<int>(std::floor(new_y));
                } else {
                    tmp->x = static_cast<int>(new_x);
                    tmp->y = static_cast<int>(new_y);
                }

                tmp->psi = dpsi_H[i];
                tmp->set_G(this->G);

                if (tmp->x >= 0 && tmp->x < gridLength && tmp->y >= 0 && tmp->y < gridWidth && og->Grid(tmp->y, tmp->x) < 1.0) {
                    successors_h.push_back(tmp);
                } else {
                    // invalid -> free memory to avoid leak
                    delete tmp;
                }
            }
            return successors_h;
        }

        // ---------- 3D succ (original return-by-vector) ----------
        std::vector<Node<3,double>*> updateSuccessors3D(adore::env::OccupanyGrid* og,
                                                       adore::fun::CollisionCheckOffline* cco,
                                                       long double HeadingResolutionRad)
        {
            std::vector<Node<3,double>*> successors_nh;
            this->R = 5.0;

            int gridWidth = og->Grid.rows();
            int gridLength = og->Grid.cols();
            int gridDepth = static_cast<int>(cco->offlineCollisionTable.size());

            cp = std::cos(this->psi);
            sp = std::sin(this->psi);

            // candidate 0
            {
                Node<3,double>* tmp = new Node<3,double>;
                tmp->x = this->x + this->R * (cp * this->dA);
                tmp->y = this->y + this->R * (sp * this->dA);
                tmp->psi = this->psi;
                tmp->set_G(this->G);
                tmp->update_index(gridWidth, gridLength, gridDepth, static_cast<double>(HeadingResolutionRad));
                if (tmp->x >= 0.0 && tmp->x < gridLength && tmp->y >= 0.0 && tmp->y < gridWidth &&
                    isCollisionFree(tmp, og, cco, HeadingResolutionRad))
                {
                    successors_nh.push_back(tmp);
                } else {
                    delete tmp;
                }
            }

            // candidate 1
            {
                Node<3,double>* tmp1 = new Node<3,double>;
                tmp1->x = this->x + this->R * (std::sin(this->psi + this->dA) - sp);
                tmp1->y = this->y + this->R * (-std::cos(this->psi + this->dA) + cp);
                tmp1->psi = this->psi + this->dA;
                tmp1->set_G(this->G);
                tmp1->update_index(gridWidth, gridLength, gridDepth, static_cast<double>(HeadingResolutionRad));
                if (tmp1->x >= 0.0 && tmp1->x < gridLength && tmp1->y >= 0.0 && tmp1->y < gridWidth &&
                    isCollisionFree(tmp1, og, cco, HeadingResolutionRad))
                {
                    successors_nh.push_back(tmp1);
                } else {
                    delete tmp1;
                }
            }

            // candidate 2
            {
                Node<3,double>* tmp2 = new Node<3,double>;
                tmp2->x = this->x + this->R * (-std::sin(this->psi - this->dA) + sp);
                tmp2->y = this->y + this->R * (std::cos(this->psi - this->dA) - cp);
                tmp2->psi = this->psi - this->dA;
                tmp2->set_G(this->G);
                tmp2->update_index(gridWidth, gridLength, gridDepth, static_cast<double>(HeadingResolutionRad));
                if (tmp2->x >= 0.0 && tmp2->x < gridLength && tmp2->y >= 0.0 && tmp2->y < gridWidth &&
                    isCollisionFree(tmp2, og, cco, HeadingResolutionRad))
                {
                    successors_nh.push_back(tmp2);
                } else {
                    delete tmp2;
                }
            }

            return successors_nh;
        }

        // ---------- 3D pool-fill overload (fills provided pointers, marks invalid with x=y=-1.0) ----------
        // out_successors must contain at least 3 pointers to preallocated Node<3,double> objects.
        void updateSuccessors3D(adore::env::OccupanyGrid* og,
                                adore::fun::CollisionCheckOffline* cco,
                                double HeadingResolutionRad,
                                std::vector<Node<3,double>*>& out_successors)
        {
            if (out_successors.size() < 3) {
                std::cerr << "[Node] updateSuccessors3D: out_successors.size() < 3\n";
                return;
            }

            this->R = 5.0;
            int gridWidth = og->Grid.rows();
            int gridLength = og->Grid.cols();
            int gridDepth = static_cast<int>(std::max<int>(1, static_cast<int>(cco->offlineCollisionTable.size())));

            cp = std::cos(this->psi);
            sp = std::sin(this->psi);

            // candidate 0
            {
                Node<3,double>* tmp = out_successors[0];
                tmp->x = this->x + this->R * (cp * this->dA);
                tmp->y = this->y + this->R * (sp * this->dA);
                tmp->psi = this->psi;
                tmp->set_G(this->G);
                tmp->update_index(gridWidth, gridLength, gridDepth, HeadingResolutionRad);

                if (!(tmp->x >= 0.0 && tmp->x < gridLength && tmp->y >= 0.0 && tmp->y < gridWidth &&
                      isCollisionFree(tmp, og, cco, HeadingResolutionRad)))
                {
                    tmp->x = -1.0; tmp->y = -1.0;
                }
            }

            // candidate 1
            {
                Node<3,double>* tmp1 = out_successors[1];
                tmp1->x = this->x + this->R * (std::sin(this->psi + this->dA) - sp);
                tmp1->y = this->y + this->R * (-std::cos(this->psi + this->dA) + cp);
                tmp1->psi = this->psi + this->dA;
                tmp1->set_G(this->G);
                tmp1->update_index(gridWidth, gridLength, gridDepth, HeadingResolutionRad);

                if (!(tmp1->x >= 0.0 && tmp1->x < gridLength && tmp1->y >= 0.0 && tmp1->y < gridWidth &&
                      isCollisionFree(tmp1, og, cco, HeadingResolutionRad)))
                {
                    tmp1->x = -1.0; tmp1->y = -1.0;
                }
            }

            // candidate 2
            {
                Node<3,double>* tmp2 = out_successors[2];
                tmp2->x = this->x + this->R * (-std::sin(this->psi - this->dA) + sp);
                tmp2->y = this->y + this->R * (std::cos(this->psi - this->dA) - cp);
                tmp2->psi = this->psi - this->dA;
                tmp2->set_G(this->G);
                tmp2->update_index(gridWidth, gridLength, gridDepth, HeadingResolutionRad);

                if (!(tmp2->x >= 0.0 && tmp2->x < gridLength && tmp2->y >= 0.0 && tmp2->y < gridWidth &&
                      isCollisionFree(tmp2, og, cco, HeadingResolutionRad)))
                {
                    tmp2->x = -1.0; tmp2->y = -1.0;
                }
            }
        }

        // checks collisions by consulting the offlineCollisionTable from cco
        bool isCollisionFree(Node<3,double>* node,
                             adore::env::OccupanyGrid* og,
                             adore::fun::CollisionCheckOffline* cco,
                             long double HeadingResolutionRad)
        {
            int gridWidth = og->Grid.rows();
            int gridLength = og->Grid.cols();
            int Depth = static_cast<int>(cco->offlineCollisionTable.size());

            // compute sub-cell indices (clamped)
            int index_x = static_cast<int>(std::floor((node->x - std::floor(node->x)) * cco->NumberOfPointsPerAxis));
            int index_y = static_cast<int>(std::floor((node->y - std::floor(node->y)) * cco->NumberOfPointsPerAxis));
            index_x = std::max(0, std::min(index_x, cco->NumberOfPointsPerAxis - 1));
            index_y = std::max(0, std::min(index_y, cco->NumberOfPointsPerAxis - 1));

            // use node's depth index (safer)
            int index_psi = node->index_depth;
            if (index_psi < 0) index_psi = 0;
            if (index_psi >= Depth) index_psi = Depth - 1;

            // iterate offline table cells (table returns vector of offsets)
            for (size_t i = 0; i < cco->offlineCollisionTable[index_psi](index_x, index_y).size(); ++i)
            {
                int X = int(std::floor(node->x)) + bg::get<0>(cco->offlineCollisionTable[index_psi](index_x, index_y)[i]);
                int Y = int(std::floor(node->y)) + bg::get<1>(cco->offlineCollisionTable[index_psi](index_x, index_y)[i]);

                if (X >= 0 && X < gridLength && Y >= 0 && Y < gridWidth)
                {
                    if (og->Grid(Y, X) > 0.900 || og->Grid(Y, X) < 0.0)
                    {
                        return false;
                    }
                }
            }

            return true;
        }

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
                      << static_cast<double>(this->x) << "\t"
                      << static_cast<double>(this->y) << "\t"
                      << this->psi << "\t"
                      << this->G << "\t"
                      << this->H << "\t"
                      << this->C << "\n";
        }

    };

} // namespace fun
} // namespace adore
