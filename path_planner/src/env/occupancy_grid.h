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
//#include <plotlablib/afigurestub.h>
//#include <plotlablib/figurestubfactory.h>
//#include <adore/apps/if_plotlab/plot_shape.h>
#include <boost/geometry.hpp>
#include <boost/container/vector.hpp>
#include <eigen3/Eigen/Dense>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <stdint.h>

//#include <adore/fun
namespace adore
{
	namespace env
	{
        namespace bg = boost::geometry; 
        namespace bgm = bg::model;
        using point_xy = bgm::d2::point_xy<double>; 
        using polygon = bgm::polygon<point_xy>;
       	class OccupanyGrid
		{
            public:
                typedef bg::model::point<double,2,bg::cs::cartesian> Point;
                typedef bg::model::box<Point> box;
                polygon rectangularBox;
                struct circle
                {
                    double x, y, r;
                };            
                struct _Obstacle
                {
                    double x;
                    double y;
                    double width;
                    double length;
                    double alpha;
                    std::vector<double> vertices_x,vertices_y;
                    std::vector<circle> circles;
                    polygon poly;
                    int ID;
                };
                Eigen::MatrixXd Grid; 
                typedef boost::container::vector<_Obstacle> obstacleList;
                obstacleList obstacles;
                double pi;
                uint32_t width;
                uint32_t height;
                // Option A (keeps using msg->data inside function)
                void init(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &msg, uint32_t h, uint32_t w)
                    {
                    std::cout<<"start og init"<<std::endl;  
                    height = h;
                    width = w; 
                    pi = 3.141592653589793;
                    std::cout<< height << width <<std::endl;
                    Grid = Eigen::MatrixXd::Zero(height,width); //
                    std::cout << "matrix init finished" << std::endl;
                    
                    for (int r=0; r<Grid.rows(); r++)
                    
                    {                    
                        for(int c=0; c<Grid.cols(); c++)
                        {
                            if(msg->data[r*Grid.cols()+c]==0){
                                Grid(r,c) = 1;//msg->data[r*Grid.cols()+c];
                            }
                            else if(msg->data[r*Grid.cols()+c]==-2){
                                Grid(r,c) = 0;//msg->data[r*Grid.cols()+c];
                            }
                            else{
                                Grid(r,c) = -1;//msg->data[r*Grid.cols()+c];
                            }
                            //std::cout<<Grid(r,c)<<std::endl;
                        }
                    }
                }
                
                boost::container::vector<_Obstacle> get_obstacles()
                {
                    return obstacles;
                }
                void resize(int Width, int Length)
                {
                    //figure->clear();  
                    //Grid = Eigen::MatrixXd::Zero(Width,Length);
                    /*
                    _Obstacle obst_0;
                    obst_0.ID = 0;
                    obst_0.x = 16.;
                    obst_0.y = 10.;
                    obst_0.length = 10.;
                    obst_0.width = 3.;
                    obst_0.alpha = 0.17;
                    obst_0.poly = rotation(&obst_0,figure); 
                    obstacles.push_back(obst_0);
                    _Obstacle obst_1;
                    obst_1.ID = 1;
                    obst_1.x = 25.;
                    obst_1.y = 2.;
                    obst_1.length = 3.;
                    obst_1.width = 2.;
                    obst_1.alpha = 0.37;
                    obst_1.poly = rotation(&obst_1,figure); 
                    obstacles.push_back(obst_1);     */           
                    obstacle();
                //if(figure != nullptr)
                //{
                  //  PLOT(figure);
                //}


                                
                }
                void calculateCircles(_Obstacle* obst)
                {
                    obst->circles.clear();
                    double r = std::min(obst->length, obst->width) ;
                    double l = std::max(obst->length, obst->width);
                    double x,y, tmp_x, tmp_y;
                    std::stringstream tag;
                    circle tmp_circle;
                    for(double  i=0.0; i<l - r/4.0; i=i+r*0.5)
                    {
                        tmp_x = -l/2.0 + i + r/4.0;
                        tmp_y = 0.0;;
                        transformation(x,y,obst->alpha, tmp_x , tmp_y);
                        x += obst->x ;
                        y += obst->y ;
                        tag << "circle"<<obst->ID<<"-"<<i;
                        tmp_circle.x = x;
                        tmp_circle.y = y;
                        tmp_circle.r = r/2.;
                        obst->circles.push_back(tmp_circle);
                        //PLOT::plotCircle(tag.str(),x,y,0.10, r/2., figure,"LineColor=0.8,0.8,0.8;LineWidth=0;FillColor=1,0,0");

                    }
                    std::cout<<"\nNum of circles: "<<obst->circles.size();


                }
                void plotSoftRectangle(_Obstacle* obst,std::string tag)
                {
                    std::vector<double> x_v, y_v;
                    double x,y,xt,yt ;
                    for (double beta = 0.0; beta < 2*pi ; beta = beta + 0.1745)
                    {
                        polar2Cartesian(x, y, get_softRectangle_r (beta, obst->length, obst->width), beta);
                        transformation(xt, yt,obst->alpha, x, y);
                        x_v.push_back(obst->x + xt);
                        y_v.push_back(obst->y + yt);
                        //std::cout<<"\n"<<x<<"\t"<<y;
                    }
                    //figure->plot(tag,&x_v[0],&y_v[0],2.5,x_v.size(), GREEN);
                }
                void plotEllipse(_Obstacle* obst,std::string tag)
                {
                    std::vector<double> x_v, y_v;
                    double x,y,xt,yt ;
                    for (double beta = 0.0; beta < 2*pi ; beta = beta + 0.1745)
                    {
                        polar2Cartesian(x, y, get_ellipse_r (beta, obst->length, obst->width), beta);
                        transformation(xt, yt,obst->alpha, x, y);
                        x_v.push_back(obst->x + xt);
                        y_v.push_back(obst->y + yt);
                        //std::cout<<"\n"<<x<<"\t"<<y;
                    }
                    //figure->plot(tag,&x_v[0],&y_v[0],2.5,x_v.size(), GREEN);
                }            
                void plotPolygon(_Obstacle* obst,std::string tag)
                {
                //std::cout<<"\n****** "<<obst->vertices_x.size();
                    //figure->plot(tag,&obst->vertices_x[0],&obst->vertices_y[0],2.5,obst->vertices_x.size(), GREEN);

                }
                double get_ellipse_r(double beta, double length, double width)
                {
                    length = length/1.4142;
                    width = width/1.4142;
                    return (length*width)/( (sqrt(  ((width*cos(beta))*(width*cos(beta))) + ((length*sin(beta))*(length*sin(beta)))  )    )  );                
                }
                double get_softRectangle_r (double beta, double length, double width)
                { 
                    
                    double softness = 0.01;
                    double cos_2 = cos(beta)*cos(beta);
                    double cos_4 = cos_2 * cos_2;
                    double soft_rectangle=( 2*cos_4 -  (2* cos_2 )+1 );
                    double soft_rectangle_rr= 1/(pow(soft_rectangle,softness));
                    return sqrt((length/1.4142)*soft_rectangle_rr*cos((beta))*(length/1.4142)*soft_rectangle_rr*cos((beta))  +((width/1.4142)*soft_rectangle_rr*sin((beta)))*((width/2.)*soft_rectangle_rr*sin((beta))));                
                }
                static void transformation(double &x, double &y,double theta, double x_o, double y_o)
                {
                    x = x_o  *cos(theta) - y_o  * sin(theta) ;
                    y = x_o  *sin(theta) + y_o  * cos(theta) ;
                } 
                bool check_valid_position(int row, int col){
                    //std::cout<<"start check "<<std::endl;
                    if(! point_inside(row, col)) {
                        //std::cout<<"Point is not inside the recorded area"<<std::endl;
                        return false;
                    }
                    //std::cout<<" row col: " <<Grid(row,col)<<std::endl;
                    //std::cout<<" col row: " <<Grid(col, row)<<std::endl;
                    return Grid(row,col) == 0;
                }
                bool point_inside(int row, int col){
                    //return row >= 0 && row < width && col >= 0 && col <= height;
                    //std::cout<< "point inside: row "<< row << "  height  " << height << "  col  " << col << " width  " << width<<std::endl;
                    return row >= 0 && row < height && col >= 0 && col <= width;
                }          
            private:
                std::string GREEN= "LineColor=0.,1.,0.;LineWidth=5";
                std::string RED= "LineColor=1.,0.,0.;LineWidth=5";
                void polar2Cartesian(double &x, double &y, double r, double beta)
                {
                    x = r* cos(beta);
                    y = r* sin(beta);
                }

                void obstacle ()
                {
                    for (int r=0; r<Grid.rows(); ++r)
                    {                    
                        for(int c=0; c<Grid.cols(); ++c)
                        {
                            for(int i=0; i<obstacles.size(); ++i)
                            {
                                if(bg::within(Point(c,r),obstacles[i].poly))
                            //if((r<15 && r>6 && c<18 && c> 14) || (r<12 && r>-1 && c<38 && c> 33) || (r<12 && r>8 && c<63 && c> 59) || (r<9 && r>5 && c<51 && c> 47))
                                {
                                    Grid(r,c) = 1;
                                }
                            }

                        }
                    }
                }

                void PLOT()
                {
                    
                    /*            
                    std::stringstream ss;
                    for (int r=0; r<Grid.rows(); ++r)
                    {                    
                        for(int c=0; c<Grid.cols(); ++c)
                        {
                            ss.clear();
                            ss.str("");
                            ss << "f"<<r*Grid.cols()+c;
                            if(Grid(r,c)==1) {
                                PLOT::plotPosition(ss.str(),c,r,figure,RED,0.1);
                            }
                            //std::cout<<"\n"<<r<<"\t"<<c<<"\t"<<r*Grid.cols()+c;
                            else if (Grid(r,c)==0) {
                                PLOT::plotPosition(ss.str(),c,r,figure,GREEN,0.1);
                            }
                        }

                    }*/
                    
                }

                polygon rotation(_Obstacle* obst)
            {
                obst->vertices_x.clear();
                obst->vertices_y.clear();
                double l= obst->length;
                double w= obst->width;
                double angle = obst->alpha;
                box rectangle(Point(-l/2.0,-w/2.0),Point(l/2.0,w/2.0));
                bg::assign(rectangularBox,rectangle);                
                polygon rbb;
                double x = obst->x;
                double y = obst->y;
                std::vector<Point> tmp_p;
                for(const auto&p: rectangularBox.outer())
                {
                    tmp_p.push_back(Point(p.x()*cos(angle)-p.y()*sin(angle)+x,p.x()*sin(angle)+p.y()*cos(angle)+y));
                    obst->vertices_x.push_back(p.x()*cos(angle)-p.y()*sin(angle)+x);   
                    obst->vertices_y.push_back(p.x()*sin(angle)+p.y()*cos(angle)+y);                    
                }
                bg::assign_points(rbb,tmp_p);
                bg::correct(rbb);
                //std::cout<<"\n"<<bg::dsv(rbb);
                //figure->plot("#obst54654",&obst->vertices_x[0],&obst->vertices_y[0],2.5,obst->vertices_x.size(), GREEN);
                
                //std::cout<<"\n"<<bg::dsv(configurationSpaceBoxes(i,j));
                return rbb;

            } 
            
                


        };
    }
}