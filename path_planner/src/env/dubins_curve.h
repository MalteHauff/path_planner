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
#include "node.h"
#include "../mad/arraymatrixtools.h"
//#include <adore/apps/if_plotlab/plot_shape.h>
#include <boost/container/vector.hpp>
#include "tree_builder.h"
#include <complex>
#include "path.h"
namespace adore
{
	namespace fun
	{
	   /**
       https://gieseanw.wordpress.com/2012/10/21/a-comprehensive-step-by-step-tutorial-to-computing-dubins-paths/
       https://github.com/AndrewWalker/Dubins-Curves
	   *  ----------------------
	   */
       	class DubinsCurve
		{
            private:
            std::string GRAY= "LineColor=0.7,0.7,0.7;LineWidth=1";
            //std::string RED= "LineColor=1.,0.,0.;LineWidth=3";
            std::string C1="LineStyle=none;PointSize=2;LineColor=1,0,1";
            std::string C2="LineStyle=none;PointSize=2;LineColor=0.5,0,1";
            std::string C3="LineStyle=none;PointSize=2;LineColor=0,0.,1.0";
            std::string C4="LineStyle=none;PointSize=2;LineColor=0,0.,1";
            std::string C5="LineStyle=none;PointSize=2;LineColor=0,0,0.0";
            std::string C6="LineStyle=none;PointSize=2;LineColor=0.5,0.5,0.5";
            std::string CO="LineStyle=none;PointSize=4;LineColor=0,0.5,0..0";
            std::vector<std::string> color;
            double pi;
            double TWO_PI;
            double BIG_NUMBER;
            double maximumTurningRadius;
            double min_cost;
            int optIndex;
            static inline double mod (double x, double y)
            {
                return x-y*std::floor(x/y);
            }

            public:
            static const int N = 6;   //possible path
            enum possiblePath  {LSL, RSR, LSR, RSL, RLR, LRL};
            enum possibleSegment  {L, S, R};
            bool isCollisionFree;
            static const int nH_Type = 3;  //non holonomic
            std::vector<Node< nH_Type,  double>*> dubinsNodes; 
            struct Path 
            {
                int type;
                char segment[3];
                double cost;
                double length;
                bool status;
                double p;
                double q;
                double t;
                TrajectoryVector curve;//TreeBuilder<nH_Type,double>::TrajectoryVector curve;
            } path[N], optPath;
            typedef Node_Lite Point;//typedef TreeBuilder<nH_Type,double>::Node_Lite Point;
            DubinsCurve(double maximumTurningRadius=6.0)
            {
                dubinsNodes.clear();
                isCollisionFree = true;
                pi = 3.141592653589793;
                TWO_PI = 2.0000 * pi;
                BIG_NUMBER = 1000.0;
                min_cost = BIG_NUMBER;
                this->maximumTurningRadius = maximumTurningRadius;
                path[0].type = possiblePath::LSL;
                path[1].type = possiblePath::RSR;
                path[2].type = possiblePath::LSR;
                path[3].type = possiblePath::RSL;
                path[4].type = possiblePath::RLR;
                path[5].type = possiblePath::LRL;
                path[LSL].segment[0] = 'L'; path[LSL].segment[1] = 'S'; path[LSL].segment[2] = 'L';
                path[RSR].segment[0] = 'R'; path[RSR].segment[1] = 'S'; path[RSR].segment[2] = 'R';
                path[LSR].segment[0] = 'L'; path[LSR].segment[1] = 'S'; path[LSR].segment[2] = 'R';
                path[RSL].segment[0] = 'R'; path[RSL].segment[1] = 'S'; path[RSL].segment[2] = 'L';
                path[RLR].segment[0] = 'R'; path[RLR].segment[1] = 'L'; path[RLR].segment[2] = 'R';
                path[LRL].segment[0] = 'L'; path[LRL].segment[1] = 'R'; path[LRL].segment[2] = 'L';
                color.push_back(C1);color.push_back(C2);color.push_back(C3);color.push_back(C4);color.push_back(C5);color.push_back(C6);
            }
            void init()
            {
                path[LSL].status = false;
                path[LSL].cost = BIG_NUMBER;
                path[LSL].length = BIG_NUMBER;
                path[RSR].status = false;
                path[RSR].cost = BIG_NUMBER;
                path[RSR].length = BIG_NUMBER;  
                path[LSR].status = false;
                path[LSR].cost = BIG_NUMBER;
                path[LSR].length = BIG_NUMBER; 
                path[RSL].status = false;
                path[RSL].cost = BIG_NUMBER;
                path[RSL].length = BIG_NUMBER;  
                path[RLR].status = false;
                path[RLR].cost = BIG_NUMBER;
                path[RLR].length = BIG_NUMBER; 
                path[LRL].status = false;
                path[LRL].cost = BIG_NUMBER;
                path[LRL].length = BIG_NUMBER;  
                optPath.curve.clear();   
                isCollisionFree = true;
            }
            double plan(Node<3,double>* Start, Node<3,double>* End,adore::env::OccupanyGrid* og, adore::fun::CollisionCheckOffline* cco = nullptr, double HeadingResolutionRad = 0.0,bool PRINT =false)
            {
                init();
                double dx = End->x - Start->x;
                double dy = End->y - Start->y;
                double D = std::sqrt(dx*dx + dy*dy);
                double d = D / maximumTurningRadius;
                double theta = mod(std::atan2(dy,dx),TWO_PI);
                double alpha = mod(Start->psi - theta, TWO_PI);
                double beta = mod(End->psi - theta, TWO_PI);
                double OptPathLength = curves(alpha,beta,d,PRINT);
                //std::cout<<"dubin plan"<<std::endl;
                if(cco != nullptr)
                {
                allCurvesToNodes(Start,End,og,cco, HeadingResolutionRad);
                }

                return OptPathLength;

            }
            private:            
            double curves(double alpha, double beta, double d, bool PRINT)
            {
                double bestPathLength = BIG_NUMBER;
                double ca = std::cos(alpha);
                double sa = std::sin(alpha);
                double cb = std::cos(beta);
                double sb = std::sin(beta); 
                double cab = std::cos(alpha - beta);  
                double p_squared = 2 + d*d - 2*cab + (2*d*(sa-sb));
                if(p_squared > 0.0)
                {
                    path[LSL].status = true; 
                    double tmp = std::atan2(cb-ca,d+sa-sb);
                    path[LSL].t = mod(-alpha + tmp ,TWO_PI);
                    path[LSL].p = std::sqrt(p_squared);
                    path[LSL].q = mod(beta - tmp ,TWO_PI);
                    path[LSL].cost = path[LSL].t + path[LSL].p + path[LSL].q;
                    path[LSL].length = path[LSL].cost * maximumTurningRadius;
                    
                }
                p_squared = 2 + (d*d)-(2*cab)+(2*d*(sb-sa));
                if( p_squared > 0.0)
                {
                    path[RSR].status = true;
                    double tmp = atan2(ca-cb,d-sa+sb);
                    path[RSR].t = mod(alpha - tmp ,TWO_PI);
                    path[RSR].p = std::sqrt(p_squared);
                    path[RSR].q = mod(-beta + tmp ,TWO_PI);
                    path[RSR].cost = path[RSR].t + path[RSR].p + path[RSR].q;
                    path[RSR].length = path[RSR].cost * maximumTurningRadius;
                    
                }
                p_squared = -2 + (d*d)+(2*cab)+(2*d*(sb+sa));
                if( p_squared > 0.0)
                {
                    path[LSR].status = true;
                    path[LSR].p = std::sqrt(p_squared);
                    double tmp = std::atan2(-ca-cb,d+sa+sb)- std::atan2(-2.0,path[LSR].p);
                    path[LSR].t = mod(-alpha + tmp ,TWO_PI);                
                    path[LSR].q = mod(-mod(beta,TWO_PI) + tmp ,TWO_PI);
                    path[LSR].cost = path[LSR].t + path[LSR].p + path[LSR].q;
                    path[LSR].length = path[LSR].cost * maximumTurningRadius;
                    
                }
                p_squared = -2 + (d*d)+(2*cab)-(2*d*(sb+sa));
                if( p_squared > 0.0)
                {
                    path[RSL].status = true;
                    path[RSL].p = std::sqrt(p_squared);
                    double tmp = std::atan2(ca+cb,d-sa-sb)- std::atan2(2.0,path[RSL].p);
                    path[RSL].t = mod(alpha - tmp ,TWO_PI);                
                    path[RSL].q = mod(beta - tmp ,TWO_PI);
                    path[RSL].cost = path[RSL].t + path[RSL].p + path[RSL].q;
                    path[RSL].length = path[RSL].cost * maximumTurningRadius;
                    
                }  
                double tmp = (6.0 - d*d + 2*cab + 2*d*(sa-sb))/8.0;
                if( tmp < 1.0 && tmp>=-1.0)
                {

                    path[RLR].status = true;
                    path[RLR].p = mod(TWO_PI - ((acos(tmp))),TWO_PI); 
                    path[RLR].t = mod(alpha - std::atan2(ca-cb,d-sa+sb) ,TWO_PI) + mod(0.5*path[RLR].p,TWO_PI);                
                    path[RLR].q = mod(alpha - beta - path[RLR].t + path[RLR].p ,TWO_PI);
                    path[RLR].cost = path[RLR].t + path[RLR].p + path[RLR].q;
                    path[RLR].length = path[RLR].cost * maximumTurningRadius;
                    
                }    
                tmp = (6.0 - d*d + 2*cab + 2*d*(-sa+sb))/8.0;
                if( std::abs(tmp) < 1.0)
                {

                    path[LRL].status = true;
                    path[LRL].p = mod(TWO_PI - (std::acos(tmp)),TWO_PI);
                    path[LRL].t = mod(-alpha - std::atan2(ca-cb,d+sa-sb) ,TWO_PI) + mod(0.5*path[LRL].p,TWO_PI);                
                    path[LRL].q = mod(mod( beta,TWO_PI)-alpha - path[LRL].t + path[LRL].p ,TWO_PI);
                    path[LRL].cost = path[LRL].t + path[LRL].p + path[LRL].q;
                    path[LRL].length = path[LRL].cost * maximumTurningRadius;
                    
                } 
                double costs[N];
                double sortedCosts[N];
                int sortedIndex[N];
                for(int i=0; i<N; ++i)
                {
                    costs[i] = path[i].cost;
                }
                adore::mad::ArrayMatrixTools::sort(&sortedCosts[0], &sortedIndex[0],&costs[0],N);
                optPath = path[sortedIndex[0]]; 
                optIndex =  sortedIndex[0];
                if(PRINT) print(); 
                return   optPath.length;                                                                                                                              
            }

            void throughDubin(double progress, Point* current_q, Path* path, adore::env::OccupanyGrid* og,adore::fun::CollisionCheckOffline* cco, double HeadingResolutionRad)
            {
                double pt = (progress/maximumTurningRadius);
                Point tmp_q;
                tmp_q.x = 0.0;
                tmp_q.y = 0.0;
                tmp_q.psi = current_q->psi;
                Point tmp1,tmp2, next;
                tmp1 = throughSegment (path->t, &tmp_q,path->segment[0]);
                tmp2 = throughSegment (path->p, &tmp1,path->segment[1]);
                if(pt < path->t)
                {
                    next = throughSegment (pt , &tmp_q,path->segment[0]);
                }
                else if (pt < path->t + path->p)
                { 
                    next = throughSegment ( pt - path->t, &tmp1,path->segment[1]);
                }
                else
                {
                    next = throughSegment (pt - path->t - path->p, &tmp2,path->segment[2]);
                }
                next.x = next.x * maximumTurningRadius + current_q->x;
                next.y = next.y * maximumTurningRadius + current_q->y;
                next.psi = next.psi - mod(next.psi/(TWO_PI),TWO_PI); 
                next.psi = mod(next.psi,TWO_PI); 

                path->curve.push_back(next);

              if(!chckCollision(&next, og,  cco,  HeadingResolutionRad))
               {
                   isCollisionFree = false;
               }
            }
           Point throughSegment (double progress,Point* current, char segment)
           {
            Point next;
            double pt = (progress);
            if(segment == 'L')
            {
                next.x = current->x + sin(current->psi + pt) - sin(current->psi);
                next.y = current->y - cos(current->psi + pt) + cos(current->psi);
                next.psi = current->psi + pt;
            }
            if(segment == 'R')
            {
                next.x = current->x - sin(current->psi - pt) + sin(current->psi);
                next.y = current->y + cos(current->psi - pt) - cos(current->psi);
                next.psi = current->psi - pt;
            }  
            if(segment == 'S')
            {
                next.x = current->x + cos(current->psi) * pt;
                next.y = current->y + sin(current->psi) * pt;
                next.psi = current->psi ;
            }  
            return next;           
            
        } 
        void allCurvesToNodes(Node<3,double>* Start, Node<3,double>* End,adore::env::OccupanyGrid* og,adore::fun::CollisionCheckOffline* cco, double HeadingResolutionRad)
        {
            
            Point q;
            q.x = Start->x;
            q.y = Start->y;
            q.psi = Start->psi;
            std::stringstream ss;
            int i= optIndex;
            path[i].curve.clear();
            std::cout<<"befor loop"<<std::endl;
           // for (int i=0; i<N; ++i)
            {
                std::cout<<"in loop"<<std::endl;
                ss<<i;
                if(path[i].status)
                {
                    double length = 0.0;
                    while (length < path[i].length && isCollisionFree)
                    {
                        throughDubin(length, &q, &path[i], og,cco, HeadingResolutionRad);
                        length += 0.1; 
                    }
                }
                if(isCollisionFree) 
                {
                    //plot_dubin(figure);
                }
            }
            optPath = path[optIndex];

        } 
         bool chckCollision(Point* point, adore::env::OccupanyGrid* og, adore::fun::CollisionCheckOffline* cco, long double HeadingResolutionRad)
         {
            int X, Y;
            int gridWidth = og->Grid.rows();
            int gridLength = og->Grid.cols();  
            int Depth = cco->offlineCollisionTable.size();      
            int index_x = std::max(int((point->x - long(point->x))* cco->NumberOfPointsPerAxis), -100);
            int index_y = std::max(int((point->y - long(point->y))* cco->NumberOfPointsPerAxis), -100);
            int index_psi = int( mad::ArrayMatrixTools::mod(point->psi,TWO_PI) /HeadingResolutionRad);
            index_psi = std::min(index_psi, Depth-1);
            for(int i=0; i<cco->offlineCollisionTable[index_psi](index_x,index_y).size() ; ++i)
            {
                X = int(point->x) + bg::get<0>(cco->offlineCollisionTable[index_psi](index_x,index_y)[i]);
                Y = int(point->y) + bg::get<1>(cco->offlineCollisionTable[index_psi](index_x,index_y)[i]);
                if(X >= 0 && X < gridLength && Y >= 0 && Y < gridWidth)
                {
                    if(og->Grid(Y, X)>0.900 ||  og->Grid(Y, X)<0.0)
                    {                      
                        return false;
                    }
                }
                
            }
            return true;
         }
        void print()
        {
            for(int i=0; i<N; ++i)
            {
                std::cout<<std::fixed<<"\n"<<i<<"\t"<<path[i].segment[0]<<path[i].segment[1]<<path[i].segment[2]<<"\t"<<path[i].cost<<"\t"<<path[i].length;
            }
            std::cout<<"\n";
        }   
        void plot_dubin()
        {

            /*
            int size = path [optIndex].curve.size();
            std::vector<double> x, y, psi;
            for (int i=0; i< size; ++i)
            {
                std::stringstream ss;
                ss.str("");
                ss << "fff"<<i*2;  
                PLOT::plotRectangle(ss.str(), path [optIndex].curve[i].x, path [optIndex].curve[i].y, 2.0, 2.0, figure, GRAY, path [optIndex].curve[i].psi);                
                x.push_back(path [optIndex].curve[i].x);
                y.push_back(path [optIndex].curve[i].y);
                psi.push_back(path [optIndex].curve[i].psi);
            }
            figure->plot("#d_c",x.data(),y.data(), 1.2, size, color[1]); 
            */                  
        }     

             

       

        };
    }
}