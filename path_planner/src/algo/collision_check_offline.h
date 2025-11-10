/********************************************************************************
 * Copyright ...
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/
#pragma once

#include <boost/geometry.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/container/vector.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>

namespace adore
{
    namespace fun
    {
        namespace bg = boost::geometry;

        inline constexpr double deg2rad(double deg) noexcept {
            return deg * 3.14159265358979323846 / 180.0;
        }

        /**
         * Offline collision lookup for a rectangular footprint on a unit-cell grid.
         *
         * Units:
         *  - vehicleWidth / vehicleLength are in **cells** (not meters).
         *  - HeadingResolutionDeg is the discretization step in degrees (e.g., 20 -> 18 bins).
         *  - NumberOfPointsPerAxis is the number of sub-cell samples per axis, i.e. offsets in [-0.5, 0.5).
         *
         * For each (subcell_i, subcell_j, heading_k), we precompute the list of unit grid-cells
         * within a local bounding box that intersect the rotated footprint when the robot center
         * is at the bounding box center **plus** the sub-cell offset.
         */
        class CollisionCheckOffline
        {
            using Point   = bg::model::point<double,2,bg::cs::cartesian>;
            using polygon = bg::model::polygon<Point>;

            // footprint (cells) and discretizations
            double w_{}, l_{}, hs_deg_{};
            int N_{}, Depth_{};

            // local bounding box (cells)
            int    boundingBoxSize_{};
            double cellSize_{1.0}; // unit grid

            // temporary buffers / structures
            boost::numeric::ublas::matrix<std::vector<Point>> matrix_2d_;
            Eigen::ArrayXXi configurationSpace_; // 0/1 marks for intersecting unit cells
            boost::numeric::ublas::matrix<Point> subcell_offsets_; // offsets in [-0.5,0.5)
            polygon rectangularBox_;

        public:
            int NumberOfPointsPerAxis{};
            // offlineCollisionTable[heading_k](i,j) -> vector of unit-cell indices (as Points with integer coords)
            boost::container::vector<boost::numeric::ublas::matrix<std::vector<Point>>> offlineCollisionTable;

            CollisionCheckOffline(double vehicleWidth_cells,
                                  double vehicleLength_cells,
                                  double HeadingResolutionDeg,
                                  int    NumberOfPointsPerAxis)
            {
                // --- store params ---
                this->NumberOfPointsPerAxis = std::max(1, NumberOfPointsPerAxis);
                N_       = this->NumberOfPointsPerAxis * this->NumberOfPointsPerAxis;
                w_       = vehicleWidth_cells;
                l_       = vehicleLength_cells;
                hs_deg_  = HeadingResolutionDeg;
                Depth_   = static_cast<int>(std::round(360.0 / HeadingResolutionDeg));

                // Make sure the bounding box can contain the rotated rectangle in any orientation.
                // Diagonal of rectangle (cells) + small safety margin.
                const double diag = std::sqrt(w_ * w_ + l_ * l_);
                boundingBoxSize_  = static_cast<int>(std::ceil(diag / cellSize_)) + 2; // +2 margin
                if (boundingBoxSize_ < 1) boundingBoxSize_ = 1;

                // Allocate temporary configuration space (0/1)
                configurationSpace_.resize(boundingBoxSize_, boundingBoxSize_);
                configurationSpace_.setZero();

                // Prepare the offline table container: Depth_ matrices of vectors
                offlineCollisionTable.clear();
                matrix_2d_.resize(this->NumberOfPointsPerAxis, this->NumberOfPointsPerAxis);
                offlineCollisionTable.reserve(Depth_);
                for (int k = 0; k < Depth_; ++k)
                    offlineCollisionTable.push_back(matrix_2d_);

                // Subcell offsets in [-0.5, 0.5) (centered sampling inside a cell)
                subcell_offsets_.resize(this->NumberOfPointsPerAxis, this->NumberOfPointsPerAxis);
                const double step = 1.0 / static_cast<double>(this->NumberOfPointsPerAxis);
                for (int i = 0; i < this->NumberOfPointsPerAxis; ++i)
                {
                    for (int j = 0; j < this->NumberOfPointsPerAxis; ++j)
                    {
                        const double ox = ( (i + 0.5) * step ) - 0.5; // [-0.5, 0.5)
                        const double oy = ( (j + 0.5) * step ) - 0.5; // [-0.5, 0.5)
                        subcell_offsets_(i,j) = Point(ox, oy);
                    }
                }

                rectangularBoundingBox_();
                create_();

                std::cout << "\n*** Collision offline lookup generated "
                          << "(bbox=" << boundingBoxSize_
                          << ", depth=" << Depth_
                          << ", subcell=" << this->NumberOfPointsPerAxis << "x"
                          << this->NumberOfPointsPerAxis << ")\n";
            }

        private:
            void create_()
            {
                const double dtheta = deg2rad(hs_deg_);
                for (int i = 0; i < NumberOfPointsPerAxis; ++i)
                {
                    for (int j = 0; j < NumberOfPointsPerAxis; ++j)
                    {
                        double angle = 0.0;
                        for (int k = 0; k < Depth_; ++k)
                        {
                            const polygon rotated = rotation_(subcell_offsets_(i,j), angle);
                            configurationSpace_.setZero();  // reuse buffer (no reallocation)
                            markIntersections_(rotated);
                            writeLookup_(i, j, k);
                            angle += dtheta;
                        }
                    }
                }
            }

            // Fill configurationSpace_(x,y) = 1 if the rotated footprint intersects unit cell [x,x+1]x[y,y+1]
            void markIntersections_(const polygon& p)
            {
                // Pre-allocate corners container to avoid reallocations in inner loop
                // (we still need to reassign points each time).
                for (int x = 0; x < boundingBoxSize_; ++x)
                {
                    for (int y = 0; y < boundingBoxSize_; ++y)
                    {
                        polygon cell;
                        std::vector<Point> corners;
                        corners.reserve(5);
                        corners.emplace_back(static_cast<double>(x),     static_cast<double>(y));
                        corners.emplace_back(static_cast<double>(x + 1), static_cast<double>(y));
                        corners.emplace_back(static_cast<double>(x + 1), static_cast<double>(y + 1));
                        corners.emplace_back(static_cast<double>(x),     static_cast<double>(y + 1));
                        corners.emplace_back(static_cast<double>(x),     static_cast<double>(y));
                        bg::assign_points(cell, corners);
                        bg::correct(cell);

                        configurationSpace_(x, y) = bg::intersects(p, cell) ? 1 : 0;
                    }
                }
            }

            // Transfer 1-cells from configurationSpace_ to offlineCollisionTable[k](i,j)
            void writeLookup_(int i, int j, int k)
            {
                auto& vec = offlineCollisionTable[k](i, j);
                vec.clear();
                // Reserve a reasonable amount: area of bbox / 2 (heuristic)
                vec.reserve(static_cast<std::size_t>(boundingBoxSize_ * boundingBoxSize_ / 2));

                for (int x = 0; x < boundingBoxSize_; ++x)
                {
                    for (int y = 0; y < boundingBoxSize_; ++y)
                    {
                        if (configurationSpace_(x, y) == 1)
                        {
                            // Store as integer grid indices using a Point (double container)
                            vec.emplace_back(static_cast<double>(x), static_cast<double>(y));
                        }
                    }
                }
            }

            // Define axis-aligned rectangular footprint centered at (0,0) before rotation/translation
            void rectangularBoundingBox_()
            {
                std::vector<Point> tmp_p;
                tmp_p.reserve(5);
                tmp_p.emplace_back(-l_ * 0.5, -w_ * 0.5);
                tmp_p.emplace_back( l_ * 0.5, -w_ * 0.5);
                tmp_p.emplace_back( l_ * 0.5,  w_ * 0.5);
                tmp_p.emplace_back(-l_ * 0.5,  w_ * 0.5);
                tmp_p.emplace_back(-l_ * 0.5, -w_ * 0.5);
                bg::assign_points(rectangularBox_, tmp_p);
                bg::correct(rectangularBox_);
            }

            // Rotate the rectangular footprint by 'angle' (rad) and translate it so that its center
            // lies at the middle of the local bbox plus the given sub-cell offset in [-0.5, 0.5).
            polygon rotation_(const Point& subcell_offset, double angle) const
            {
                polygon rbb;

                // Center of local bbox (in unit-cell coordinates), plus subcell offset
                const double cx = (static_cast<double>(boundingBoxSize_) * 0.5) + bg::get<0>(subcell_offset);
                const double cy = (static_cast<double>(boundingBoxSize_) * 0.5) + bg::get<1>(subcell_offset);

                const double ca = std::cos(angle);
                const double sa = std::sin(angle);

                std::vector<Point> tmp_p;
                tmp_p.reserve(rectangularBox_.outer().size());
                for (const auto& p : rectangularBox_.outer())
                {
                    const double px = bg::get<0>(p);
                    const double py = bg::get<1>(p);
                    tmp_p.emplace_back(px * ca - py * sa + cx,
                                       px * sa + py * ca + cy);
                }
                bg::assign_points(rbb, tmp_p);
                bg::correct(rbb);
                return rbb;
            }
        };
    }
}
