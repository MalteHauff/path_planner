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
        /*struct Node_Lite
        {
            double x;
            double y;
            double psi;
            double s;   //used only for smoothing
        };*/
        //typedef boost::container::vector<Node_Lite> TrajectoryVector;
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
        TrajectoryVector build(Node<TYPE,T>* Start, Node<TYPE,T>* End,double vehicleWidth, double vehicleLength)
        {
            int index = 0;
            Node_Lite search;
            search.x = End->x;
            search.y = End->y;
            search.psi = End->psi;
            tree.push_back(search);
            int starting_index = s.size()-1;
            std::cout<<"size : "<<starting_index<<"  size tree: "<<tree.size()<<std::endl;
            int counter = 0;
            double tolerance = 0.92;
            std::cout << "End point : " << search.x << "/ "<<search.y<<std::endl;
            while(true)
            {
                //std::cout<<"while interation"<<std::endl;
                for(int i=starting_index; i>=0; --i)
                {
                    //std::cout << "Punkte : " << s[i].x << "/ "<<s[i].y<<std::endl;
                    if(s[i].x == search.x && s[i].y == search.y)//(std::abs(s[i].x - search.x) < tolerance && std::abs(s[i].y - search.y) < tolerance)//(s[i].x == search.x && s[i].y == search.y)//TODO Vergleich
                    {
                        //std::cout<<"equal point"<<std::endl;
                        index = i;
                        starting_index = i;
                        search.x = p[i].x;
                        search.y = p[i].y;
                        search.psi = p[i].psi;
                        tree.push_back(search);
                        break;
                    }
                }
                //std::cout<<"out of forLoop in build"<<std::endl;
                if(search.x == Start->x && search.y == Start->y)
                {
                    break;
                }
         
            }//while
            std::reverse(tree.begin(), tree.end());
            std::cout<<"\nTree size: "<<tree.size();
            
            /*
            if(figure!=nullptr)
            {
                std::vector<double> x,y;
                x.clear(); y.clear();
                for(int i=0; i<tree.size(); i++)
                {
                    x.push_back(tree[i].x);
                    y.push_back(tree[i].y);
                    std::stringstream ss;
                    ss.str("");
                    ss << "f"<<i;                    
                    PLOT::plotRectangle(ss.str(), tree[i].x, tree[i].y, vehicleLength, vehicleWidth, figure, GRAY, tree[i].psi);
                }
                figure->plot("#Tree",&x[0],&y[0],1.1,x.size(),RED);
            }*/
            return tree;
        }     

        };
    }
}