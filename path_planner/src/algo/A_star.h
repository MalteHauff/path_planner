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
#include <adore/fun/search_grid.h>
#include <adore/fun/node.h>
#include <adore/fun/tree_builder.h>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/heap/binomial_heap.hpp>
#include <ctime>
#include <chrono>
namespace adore
{
	namespace fun
	{
	   /**
	   *  ----------------------
	   */
       	class A_Star
		{
            private:                       
            static const int H_Type = 2;  //holonomic
            adore::fun::GRID<adore::fun::Node<H_Type,int>> H_GRID;
            adore::fun::TreeBuilder<H_Type,int> Tree;
            double BIG_NUMBER;
            //https://www.boost.org/doc/libs/1_53_0/doc/html/boost/heap/fibonacci_heap.html
            // https://stackoverflow.com/questions/16705894/defining-compare-function-for-fibonacci-heap-in-boost
            struct compare
            {
                bool operator()(Node<H_Type,int>* n1, Node<H_Type,int>* n2) const
                {
                    return n1->C > n2->C;
                }
            };          

            public:

            A_Star()
            {
                BIG_NUMBER = 1000.0;
                               
            
            }

            double plan(GRID<Node<H_Type,int>>* grid, adore::env::OccupanyGrid* og, Node<H_Type,int>* Start, Node<H_Type,int>* End) //
            {

                //Tree.init();
                grid->initialize();                
                Start->get_H(End); 
                Start->update_C();               
                Start->isOpen = true;  Start->isClosed = false;
                grid->replace(Start);
                //https://www.boost.org/doc/libs/1_53_0/doc/html/boost/heap/fibonacci_heap.html
                boost::heap::fibonacci_heap<Node<H_Type,int>*, boost::heap::compare<compare>> heap;
                heap.push(Start);
                std::cout<<"a_star not hybrid"<<std::endl;
                while(!heap.empty())
                {
                    std::cout<<"a_star not hybrid in loop"<<std::endl;
                    Node<H_Type,int>* predecessor_node = heap.top();
                    if(grid->isClosed(predecessor_node))
                    {
                        heap.pop();
                        continue;
                    }
                    else if(grid->isOpen(predecessor_node))
                    {
                        grid->set_visited(predecessor_node);
                        heap.pop();
                       if(predecessor_node->isEqual(End))
                        {
                            //Tree.push_p(predecessor_node);
                            //Tree.build(Start,End,figure);
                            //std::cout<<"\nGOAL IS REACHED";
                            //std::cout<<"\nG: "<<predecessor_node->get_G()<<"\n";
                          
                            return predecessor_node->get_G();
                        }
                        else
                        {   
                           evaluateSuccessors(predecessor_node, grid, og, End,&heap);
                        }
                    }
                }
                return BIG_NUMBER;
                

            }

            void evaluateSuccessors(Node<2,int>* node, GRID<Node<H_Type,int>>* grid, adore::env::OccupanyGrid* og,Node<H_Type,int>* End,boost::heap::fibonacci_heap<Node<H_Type,int>*, boost::heap::compare<compare>>* heap)
            {
                double G ;
                std::vector<Node< 2,  int>*>  successors_h;
                successors_h = node->updateSuccessors2D(og);
                for (int i=0; i<successors_h.size(); i++)
                {
                   if(!successors_h[i]->isClosed )//&& og->Grid(successors_h[i]->y, successors_h[i]->x) < 1.0)
                   {
                       G = successors_h[i]->get_G(node);
                       if(!grid->isOpen(successors_h[i]) || G <  grid->get_G(successors_h[i]))
                       {
                           //Tree.push_p(node);
                           //Tree.push_s(node->successors[i]);
                           successors_h[i]->get_H(End);
                           successors_h[i]->isOpen = true;
                           successors_h[i]->update_C();
                           grid->replace(successors_h[i]);
                           heap->push(successors_h[i]);
                       }

                   }
                }
            }


        };
    }
}