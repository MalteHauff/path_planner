/********************************************************************************
 * Copyright (C) 2017-2020 German Aerospace Center (DLR). 
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the 
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0 
 *
 * Contributors: 
 *   Reza Dariani - initial API and implementation
 ********************************************************************************/

#pragma once
#include <boost/container/vector.hpp>
//#include <adore/apps/if_plotlab/plot_shape.h>
#include "path.h"
#include <iostream>
#include <algorithm>

namespace adore
{
    namespace fun
    {
       /**
       *  ----------------------
       */
       template <int TYPE, typename T>
        class TreeBuilder
        {
        private:
            std::string RED= "LineColor=1.,0.,0.;LineWidth=3";
            std::string GRAY= "LineColor=0.7,0.7,0.7;LineWidth=1";

        public:
            // p: parent nodes (corresponds 1:1 with s)
            // s: successor nodes (the nodes discovered)
            // tree: reconstructed ordered path (Node_Lite vector)
            adore::TrajectoryVector p;
            adore::TrajectoryVector s;
            adore::TrajectoryVector tree;

            TreeBuilder()
            {
            }

            void init()
            {
                p.clear();
                s.clear();
                tree.clear();
            }

            // push parent for the successor(s) that will be pushed next
            void push_p(Node<TYPE,T>* node )
            {
                Node_Lite tmp;
                tmp.x = node->x;
                tmp.y = node->y;
                tmp.psi = node->psi;
                p.push_back(tmp);
            }

            void push_s(Node<TYPE,T>* node )
            {
                Node_Lite tmp;
                tmp.x = node->x;
                tmp.y = node->y;
                tmp.psi = node->psi;
                s.push_back(tmp);
            }

            // build reconstructs a path from Start -> End using the parallel arrays p/s.
            // If no matching successor is found during backtracking, the function
            // emits a warning and returns the partial path found so far (reversed).
            TrajectoryVector build(Node<TYPE,T>* Start, Node<TYPE,T>* End,double vehicleWidth, double vehicleLength)
            {
                tree.clear();

                Node_Lite search;
                search.x = End->x;
                search.y = End->y;
                search.psi = End->psi;

                // seed tree with the End point (we will backtrack)
                tree.push_back(search);

                int starting_index = static_cast<int>(s.size()) - 1;
                std::cout<<"size : "<< starting_index <<"  size tree: "<< tree.size() << std::endl;
                double tolerance = 0.92;

                std::cout << "End point : " << search.x << "/ "<<search.y<<std::endl;

                // safety counter to avoid infinite loops
                int safety_counter = 0;
                const int SAFETY_LIMIT = static_cast<int>(s.size()) + 1000;

                while(true)
                {
                    bool found = false;

                    // backtrack: search s[] for the current 'search' (starting from latest)
                    for(int i = starting_index; i >= 0; --i)
                    {
                        // exact match is used in original code; also accept near-equality if desired
                        if (s[i].x == search.x && s[i].y == search.y)
                        {
                            // found the successor that matches `search` -> follow its parent p[i]
                            found = true;
                            starting_index = i - 1; // next time only search earlier indices
                            // update search to parent
                            search.x = p[i].x;
                            search.y = p[i].y;
                            search.psi = p[i].psi;
                            tree.push_back(search);
                            break;
                        }
                    }

                    // if we found nothing, warn and break (return partial tree reversed)
                    if (!found)
                    {
                        std::cerr << "[TreeBuilder] Warning: failed to find matching successor for search=("
                                  << tree.back().x << "," << tree.back().y << "). Aborting backtrack and returning partial tree.\n";
                        break;
                    }

                    // check termination: reached Start
                    if (search.x == Start->x && search.y == Start->y)
                    {
                        break;
                    }

                    // safety guard
                    safety_counter++;
                    if (safety_counter > SAFETY_LIMIT)
                    {
                        std::cerr << "[TreeBuilder] Safety abort in build(): exceeded iterations (" << SAFETY_LIMIT << ").\n";
                        break;
                    }
                } // while

                // reverse the collected points to get Start -> End order
                std::reverse(tree.begin(), tree.end());
                std::cout<<"\nTree size: "<<tree.size()<<"\n";

                return tree;
            }
        };
    }
}