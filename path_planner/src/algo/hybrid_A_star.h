#pragma once
/********************************************************************************
 * Hybrid A* header - memory-optimized version (node pool)
 ********************************************************************************/

#include "../env/search_grid.h"
#include "A_star.h"
#include "../env/node.h"
#include "../env/dubins_curve.h"
#include <boost/heap/fibonacci_heap.hpp>
#include "../env/tree_builder.h"
#include "../env/trajectory_smoothing.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

namespace adore {
namespace fun {

class Hybrid_A_Star {
private:
    long double pi;
    TrajectorySmoothing* smoothing;
    std::string RED = "LineStyle=none;PointSize=4;LineColor=1,0,0";

    static const int nH_Type = 3; // non-holonomic node type (Node<3,double>)
    static const int H_Type = 2;  // holonomic node type (Node<2,int>)

    GRID<adore::fun::Node<H_Type,int>> H_GRID;
    adore::fun::TreeBuilder<nH_Type,double> Tree;
    DubinsCurve dubins;
    A_Star a_star;
    long int node_counter = 0;
    long double HeadingResolutionRad;
    int iteration;

    // Node pool (holds Node<3,double> objects to avoid new/delete at runtime)
    std::vector<Node<nH_Type,double>> nodePool;
    size_t poolIndex = 0;

    struct compare {
        bool operator()(Node<nH_Type,double>* n1, Node<nH_Type,double>* n2) const {
            return n1->C > n2->C;
        }
    };

    // Get pointer to next preallocated node (grows pool in chunks)
    Node<nH_Type,double>* getNodeFromPool() {
        if (poolIndex >= nodePool.size()) {
            // grow by chunks to avoid frequent reallocs
            nodePool.resize(nodePool.size() + 1024);
        }
        return &nodePool[poolIndex++];
    }

public:
    Hybrid_A_Star(TrajectorySmoothing* smoothing)
    : smoothing(smoothing), pi(3.141592653589793), poolIndex(0)
    {
        nodePool.reserve(4096);
    }

    void setSize(int Width, int Length) {
        H_GRID.resize(Width, Length);
    }

    TrajectoryVector plan(GRID<Node<nH_Type,double>>* grid,
                          adore::env::OccupanyGrid* og,
                          adore::fun::CollisionCheckOffline* cco,
                          Node<nH_Type,double>* Start,
                          Node<nH_Type,double>* End,
                          int HeadingResolution,
                          int MAX_ITERATION,
                          double vehicleWidth,
                          double vehicleLength)
    {
        std::cout<<"start init algo"<<std::endl;
        Tree.init();
        iteration = 0;
        HeadingResolutionRad = double(HeadingResolution) * pi/180.0;
        grid->initialize();

        double Hval = cost2go(Start, End, &H_GRID, og);
        Start->set_H(Hval);
        Start->set_G(0.0);
        Start->update_C();
        Start->isOpen = true;
        Start->isClosed = false;

        boost::heap::fibonacci_heap<Node<nH_Type,double>*, boost::heap::compare<compare>> heap;
        heap.push(Start);
        grid->replace(Start, HeadingResolutionRad);

        TrajectoryVector path;
        int expansions = 0;

        while(!heap.empty()) {
            Node<nH_Type,double>* current = heap.top();
            heap.pop();
            if (grid->isClosed(current, HeadingResolutionRad)) {
                continue;
            }

            grid->set_closed(current, HeadingResolutionRad);

            if (current->isEqual(End, HeadingResolutionRad)) {
                std::cout<<"Found path to goal, starting reconstructing of path"<<std::endl;
                Tree.push_p(current);
                path = Tree.build(Start, End, vehicleWidth, vehicleLength);
                return path;
            }

            if (current->isCloseTo(End, 5.0)) {
                double length = dubins.plan(current, End, og, cco, HeadingResolutionRad, true);
                if (dubins.isCollisionFree) {
                    Tree.push_p(current);
                    path = Tree.build(Start, current, vehicleWidth, vehicleLength);
                    smoothing->get_pre_trajectory(og, &Tree.tree, &dubins.optPath.curve, vehicleWidth, vehicleLength);
                    return path;
                }
            }

            evaluateSuccessors(current, grid, &H_GRID, og, cco, End, &heap, HeadingResolutionRad);

            if (++expansions > MAX_ITERATION) {
                std::cerr << "[A*] reached MAX_ITERATION -> abort\n";
                break;
            }
        }

        std::cout << "\nGOAL IS UNREACHABLE\n";
        return path;
    }

private:
    void evaluateSuccessors(Node<nH_Type,double>* node,
                            GRID<Node<nH_Type,double>>* grid,
                            GRID<Node<H_Type,int>>* h_grid,
                            adore::env::OccupanyGrid* og,
                            adore::fun::CollisionCheckOffline* cco,
                            Node<nH_Type,double>* End,
                            boost::heap::fibonacci_heap<Node<nH_Type,double>*, boost::heap::compare<compare>>* heap,
                            double HeadingResolutionRad)
    {
        // allocate three successor objects from pool
        Node<nH_Type,double>* s0 = getNodeFromPool();
        Node<nH_Type,double>* s1 = getNodeFromPool();
        Node<nH_Type,double>* s2 = getNodeFromPool();

        std::vector<Node<3,double>*> successors = { s0, s1, s2 };

        // CALL the Node overload that fills the provided successor pointers.
        node->updateSuccessors3D(og, cco, HeadingResolutionRad, successors);

        for (auto succPtr : successors) {
            // invalid marker (set by Node::updateSuccessors3D when candidate rejected)
            if (succPtr->x < 0.0 || succPtr->y < 0.0) continue;

            if (grid->isClosed(succPtr, HeadingResolutionRad)) continue;

            double tentative_g = succPtr->get_G(node);
            double stored_g = grid->get_G(succPtr, HeadingResolutionRad);

            bool not_in_open = !grid->isOpen(succPtr, HeadingResolutionRad);
            bool better_path = tentative_g + 1e-9 < stored_g;
            bool same_index = node->hasEqualIndex(succPtr, HeadingResolutionRad);

            if (not_in_open || better_path || same_index) {
                succPtr->set_H(cost2go(succPtr, End, h_grid, og));
                succPtr->set_G(tentative_g);
                succPtr->update_C();

                if (same_index && succPtr->C > node->C + 0.01) {
                    continue;
                }

                Tree.push_p(node);
                Tree.push_s(succPtr);

                succPtr->isOpen = true;
                succPtr->isClosed = false;

                grid->replace(succPtr, HeadingResolutionRad);
                heap->push(succPtr);
            }
        }
    }

    double cost2go (Node<nH_Type,double>* Current, Node<nH_Type,double>* End, GRID<Node<H_Type,int>> * grid, adore::env::OccupanyGrid* og) {
        double dx = static_cast<double>(Current->x) - static_cast<double>(End->x);
        double dy = static_cast<double>(Current->y) - static_cast<double>(End->y);
        double euclid = std::sqrt(dx*dx + dy*dy);

        double dubinsPathLength = 0.0;
        try { dubinsPathLength = dubins.plan(Current, End, og); } catch(...) { dubinsPathLength = -1.0; }

        if (!std::isfinite(dubinsPathLength) || dubinsPathLength <= 0.0) {
            return euclid;
        }
        return std::min(dubinsPathLength, euclid);
    }

}; // class Hybrid_A_Star

} // namespace fun
} // namespace adore
