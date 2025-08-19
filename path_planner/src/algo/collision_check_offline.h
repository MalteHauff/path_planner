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
#include <boost/geometry.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include "../mad/coordinateconversion.h"
#include <boost/numeric/ublas/matrix.hpp>
#include <Eigen/Dense> 
namespace adore
{
	namespace fun
	{
	   /**
	   *  ----------------------
	   */
        namespace bg = boost::geometry; 
        namespace bgm = bg::model;
        using point_xy = bgm::d2::point_xy<double>; 
        using polygon = bgm::polygon<point_xy>;
        using namespace bg::strategy::transform;
       	class CollisionCheckOffline
		{
            private:
            int N;
            
            double w, l, hs;
            int Depth;
            int boundingBoxSize;
            double cellSize;
            typedef bg::model::point<double,2,bg::cs::cartesian> Point;
            
            boost::numeric::ublas::matrix<std::vector<Point>> matrix_2d;
            Eigen::MatrixXd configurationSpace; 
            
            typedef bg::model::box<Point> box;
            boost::numeric::ublas::matrix<Point> points;
            polygon rectangularBox;
            boost::numeric::ublas::matrix<polygon> configurationSpaceBoxes;

            public:
            int NumberOfPointsPerAxis;
            boost::container::vector<boost::numeric::ublas::matrix<std::vector<Point>>> offlineCollisionTable;
            CollisionCheckOffline(double vehicleWidth, double vehicleLength, double HeadingResolution, int NumberOfPointsPerAxis)
            {
                cellSize = 1.00;
                this->NumberOfPointsPerAxis = NumberOfPointsPerAxis;
                N = NumberOfPointsPerAxis * NumberOfPointsPerAxis;
                w = vehicleWidth;
                l = vehicleLength;
                hs = HeadingResolution;
                Depth = 360 / HeadingResolution;
                boundingBoxSize = std::ceil(std::sqrt(w*w + l*l) / cellSize);
                configurationSpaceBoxes.resize(boundingBoxSize,boundingBoxSize);
                offlineCollisionTable.clear();
                matrix_2d.resize(NumberOfPointsPerAxis,NumberOfPointsPerAxis);
                for (int i=0; i<Depth; i++)
                {
                    offlineCollisionTable.push_back(matrix_2d);
                }  
                points.resize(NumberOfPointsPerAxis,NumberOfPointsPerAxis);
                Point tmp_point;
                for (int i=0; i<NumberOfPointsPerAxis; ++i)
                {
                    for(int j=0; j<NumberOfPointsPerAxis; ++j)
                    {
                        tmp_point.set<0>(1.0 / NumberOfPointsPerAxis * j );
                        tmp_point.set<1>(1.0 / NumberOfPointsPerAxis * i );
                        points(i,j) = tmp_point;
                        //std::cout<<"\n"<<i<<"\t"<<j<<"\t"<<bg::get<0>(points(i,j))<<"\t"<<bg::get<1>(points(i,j));
                    }
                } 
                configurationSpace = Eigen::MatrixXd::Zero(boundingBoxSize,boundingBoxSize); 
                rectangularBoundingBox();
                create(); 
                std::cout<<"\n***Lookup is generated\n"  ;         

            }
            void create()
            {
                for(int i=0; i<NumberOfPointsPerAxis;++i)
                {
                    for(int j=0; j<NumberOfPointsPerAxis; ++j)
                    {
                        
                        double angle = 0.0;
                        for (int k=0; k<Depth; ++k)
                        {
                            //std::cout<<"\n"<<hs<<"\t"<<angle<<"\t"<<bg::get<0>(points(i,j))<<"\t"<<bg::get<1>(points(i,j));
                            auto rotatedRectangularBox = rotation(points(i,j), angle);
                            configurationSpace = Eigen::MatrixXd::Zero(boundingBoxSize,boundingBoxSize); 
                            checkIntersection(rotatedRectangularBox);
                            lookup( i,j,k);                            
                            angle += adore::mad::CoordinateConversion::DegToRad(hs);
                            //int a; std::cin>>a;

                        }
                    }
                }
            }
            void checkIntersection(polygon p)
            {
                for(int i=0; i<boundingBoxSize;++i)
                {
                    //std::cout<<"\n----------";
                    for (int j=0; j<boundingBoxSize; ++j)
                    {  
                        bool b = bg::intersects(configurationSpaceBoxes(i,j),p);
                        if(b) configurationSpace(i,j) = true;
                        //std::cout<<"\n"<<i<<"\t"<<j<<"\t"<<(b ? "1": "0");

                    }
                }        
            }
            void lookup(int i, int j, int k)
            {
                for(int l=0; l<boundingBoxSize;++l)
                {
                    for (int m=0; m<boundingBoxSize; ++m)
                    {                 
                        if(configurationSpace(l,m))
                        {
                            //std::cout<<"\n"<<m - int(boundingBoxSize*0.5 + bg::get<0>(points(i,j)))<<"\t"<<l - int(boundingBoxSize*0.5 + bg::get<1>(points(i,j)));
                            offlineCollisionTable[k](i,j).push_back(Point(m - int(boundingBoxSize*0.5 + bg::get<0>(points(i,j))), l - int(boundingBoxSize*0.5 + bg::get<1>(points(i,j)))));
                        }
                    }
                }
                //std::cout<<"\n"<<i<<"\t"<<j<<"\t"<<k<<"\t"<<offlineCollisionTable[k](i,j).size()<<"\t"<<boundingBoxSize;
            }
            void rectangularBoundingBox()
            {
                box rectangle(Point(-l/2.0/cellSize,-w/2.0/cellSize),Point(l/2.0/cellSize,w/2.0/cellSize));
                bg::assign(rectangularBox,rectangle);
                for(int i=0; i<boundingBoxSize;++i)
                {
                    for (int j=0; j<boundingBoxSize; ++j)
                    {
                        box rectangularBox(Point(j,i),Point(j+1,i+1));
                        //std::cout<<"\n"<<j<<"\t"<<i<<"\t"<<j+1<<"\t"<<i+1;
                        bg::assign(configurationSpaceBoxes(i,j), rectangularBox);
                        //std::cout<<"\n"<<bg::dsv(configurationSpaceBoxes(i,j));

                    }
                }


            }
            polygon rotation(Point point, double angle)
            {
                polygon rbb;
                double x = boundingBoxSize*0.5 + bg::get<0>(point);
                double y = boundingBoxSize*0.5 + bg::get<1>(point);
                std::vector<Point> tmp_p;
                for(const auto&p: rectangularBox.outer())
                {
                    tmp_p.push_back(Point(p.x()*cos(angle)-p.y()*sin(angle)+x,p.x()*sin(angle)+p.y()*cos(angle)+y));                    
                }
                bg::assign_points(rbb,tmp_p);
                bg::correct(rbb);
                return rbb;

            }

            


        };
    }
}