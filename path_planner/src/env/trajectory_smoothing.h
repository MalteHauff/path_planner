#pragma once
#include <boost/container/vector.hpp>
#include "tree_builder.h"
#include "../mad/cubicpiecewisefunction.h"
#include "../mad/coordinateconversion.h"
#include "csaps.h"
#include "path.h"

namespace adore
{
    namespace fun
    {
       /**
       * ----------------------
       */
        class TrajectorySmoothing
        {
            private:
            double pi;
            env::OccupanyGrid::obstacleList obstacles;
            enum STATES {X , Y, PSI,  S, L} ;
            enum INPUTS {DELTA} ;
            adore::mad::CubicPiecewiseFunction::PieceweisePolynomial pp_d, pp_x, pp_y, pp_psi;
            std::string RED= "LineColor=1.,0.,0.;LineWidth=1";  
            std::string BLUE= "LineColor=0.,0.,1.;LineWidth=2";  
            std::string L_BLUE= "LineColor=0.7,0.7,1.;LineWidth=2"; 
            std::string GREEN= "LineStyle=none;PointSize=4;LineColor=0,1,0";
            std::string BLACK= "LineColor=0.,0.,0.;LineWidth=1"; 
            inline static const int OptimizationPoints = 30;
            double const_velocity;

            double coef1[OptimizationPoints];
            double coef2[OptimizationPoints];
            double coef3[OptimizationPoints];
            double coef4[OptimizationPoints];
            double breaks[OptimizationPoints+1]; 
            boost::container::vector<double>  v_Delta;
            boost::container::vector<boost::container::vector<double>> parallel_control;
            boost::container::vector<double> control;
            boost::container::vector<double> control_back;
            boost::container::vector<double> obj_F;
            boost::container::vector<double> d_obj_F;
            double obj_F_back;
            double eps;
            double improvement;
            double Lagrange_mem;
            int iteration;
            boost::container::vector<boost::container::vector<double>> states;
            std::vector<double> state_s;
            static const int nH_Type = 3;
            static const int nX = 5;
            double vehicleLength;
            double vehicleWidth;
            double dt, dth;
            TrajectoryVector pre_trajectory;
            struct Circle
            {
                double x, y, r;
            };  
            boost::container::vector<Circle> egoCircles;

            // Helper function to evaluate spline derivatives
            csaps::DoubleArray eval_derivative_spline(const csaps::UnivariateCubicSmoothingSpline& spline, const csaps::DoubleArray& xidata, int deriv_order)
            {
                csaps::DoubleArray y_deriv(xidata.size());
                const auto& coeffs = spline.GetCoeffs();
                const auto& breaks = spline.GetBreaks();

                for (int i = 0; i < xidata.size(); ++i) {
                    double x_point = xidata(i);
                    // Find the correct spline piece index
                    int piece_idx = 0;
                    for (int j = 0; j < breaks.size() - 1; ++j) {
                        if (x_point >= breaks(j) && x_point <= breaks(j+1)) {
                            piece_idx = j;
                            break;
                        }
                    }

                    double a = coeffs(piece_idx, 0);
                    double b = coeffs(piece_idx, 1);
                    double c = coeffs(piece_idx, 2);
                    double d = coeffs(piece_idx, 3);
                    double dx = x_point - breaks(piece_idx);

                    // Manually calculate the derivatives based on the cubic polynomial
                    // S(x) = a*(x-xi)^3 + b*(x-xi)^2 + c*(x-xi) + d
                    switch (deriv_order) {
                        case 0: // Original value
                            y_deriv(i) = a*dx*dx*dx + b*dx*dx + c*dx + d;
                            break;
                        case 1: // First derivative: S'(x) = 3a*(x-xi)^2 + 2b*(x-xi) + c
                            y_deriv(i) = 3*a*dx*dx + 2*b*dx + c;
                            break;
                        case 2: // Second derivative: S''(x) = 6a*(x-xi) + 2b
                            y_deriv(i) = 6*a*dx + 2*b;
                            break;
                        case 3: // Third derivative: S'''(x) = 6a
                            y_deriv(i) = 6*a;
                            break;
                        default:
                            y_deriv(i) = 0.0;
                            break;
                    }
                }
                return y_deriv;
            }

            public:
            TrajectorySmoothing()
            {
                const_velocity = 1.;
                dt = 0.250;
                states.resize(nX);
                for(int i=0; i<nX; ++i) states[i].resize(OptimizationPoints+1);
                eps = 1e-8;
                Lagrange_mem = 0.0;
                improvement = 1e3;
                pi = 3.141592653589793;
            }

            void get_pre_trajectory(adore::env::OccupanyGrid* og, TrajectoryVector* tree, TrajectoryVector* curve, double vehicleWidth, double vehicleLength)
            {
                this->vehicleLength = vehicleLength;
                this->vehicleWidth  = vehicleWidth;
                iteration = 0;
                obstacles = og->get_obstacles();

                std::cout<<"\nNum of Obstacles: "<<obstacles.size();
                std::stringstream poly, s_rect, s_ellipse;
                for(int i=0; i<obstacles.size(); ++i)
                {
                    poly << "obstPoly"<<i;
                    s_rect<< "obst_s_rect"<<i;
                    s_ellipse<< "obst_s_ellipse"<<i;
                }
                
                pre_trajectory.insert(pre_trajectory.end(),tree->begin(), tree->end());
                for(int i=1; i<curve->size(); ++i)
                {
                    pre_trajectory.push_back((*curve)[i]);
                }

                int p_size = pre_trajectory.size();
                int N = std::min(OptimizationPoints, p_size);
                std::cout<<"\nTrajectory size: "<<p_size<<"\toptimization vector size: "<<N;
                csaps::DoubleArray _s(N);
                csaps::DoubleArray _x(N);
                csaps::DoubleArray _y(N);
                csaps::DoubleArray _psi(N);  
                csaps::DoubleArray _k(N); 
                csaps::DoubleArray _delta(N);  

                pre_trajectory[0].s = 0.0;
                _s(0) = pre_trajectory[0].s ;
                _x(0) = pre_trajectory[0].x ;
                _y(0) = pre_trajectory[0].y ;
                          
                for(int i=1; i<N; ++i)
                {
                    pre_trajectory[i].s = pre_trajectory[i-1].s + sqrt( (pre_trajectory[i].x - pre_trajectory[i-1].x)*(pre_trajectory[i].x - pre_trajectory[i-1].x) + (pre_trajectory[i].y - pre_trajectory[i-1].y)*(pre_trajectory[i].y - pre_trajectory[i-1].y) );  
                    _s(i) = pre_trajectory[i].s ;
                    _x(i) = pre_trajectory[i].x ;
                    _y(i) = pre_trajectory[i].y ;
                }
                
                double max_speed = 2.0;
                const_velocity = pre_trajectory[N-1].s / (N*dt);
                const_velocity = std::min(const_velocity, max_speed);
                std::cout<<"\nconst_velocity "<<const_velocity;
                
                csaps::UnivariateCubicSmoothingSpline splineX(_s, _x, 0.9);
                csaps::UnivariateCubicSmoothingSpline splineY(_s, _y, 0.9);
                
                csaps::DoubleArray x, dx, ddx, dddx;
                csaps::DoubleArray y, dy, ddy, dddy ;  
                csaps::DoubleArray delta, d_delta, dd_delta, ddd_delta ;                       

                std::tie(x,dx,ddx,dddx) = splineX(N, _s);     
                std::tie(y,dy,ddy,dddy) = splineY(N, _s);  
                          
                for(int i=0; i<N; ++i) 
                {
                    _psi(i)=adore::mad::CoordinateConversion::twoPIrange((std::atan2(dy(i),dx(i))));
                }  
                csaps::UnivariateCubicSmoothingSpline splinePSI(_s, _psi, 0.9);                 
                for(int i=0; i<N; ++i) 
                {
                    _k(i)= (dx(i)*ddy(i) - dy(i)*ddx(i)) / std::pow((dx(i)*dx(i) + dy(i)*dy(i)),1.50);
                }  
                for(int i=0; i<N; ++i) 
                {                
                    _delta(i)= std::atan((vehicleLength)  * _k(i)); 
                }  
                csaps::UnivariateCubicSmoothingSpline splineD(_s, _delta, 0.95);  

                std::tie(delta,d_delta,dd_delta,ddd_delta) = splineD(N, _s); 
                
                auto d_coefs = splineD.GetCoeffs();
                auto d_breaks = splineD.GetBreaks();                
                toPolynom(N,&pp_d,d_breaks, d_coefs);
                auto x_coefs = splineX.GetCoeffs();
                auto x_breaks = splineX.GetBreaks(); 
                toPolynom(N,&pp_x,x_breaks, x_coefs);
                auto y_coefs = splineY.GetCoeffs();
                auto y_breaks = splineY.GetBreaks();                
                toPolynom(N,&pp_y,y_breaks, y_coefs);
                auto p_coefs = splinePSI.GetCoeffs();
                auto p_breaks = splinePSI.GetBreaks();                
                toPolynom(N,&pp_psi,p_breaks, p_coefs);
                std::cout<<"\nPre trajectory is loaded\n";  
               
                initHorizon(N,og); 
                integrate(N,og,&control,L_BLUE,true); 
                createParallelSystems(N,&control);
                while(improvement > 1e-5)
                {
                    optimize(N,og);
                    iteration++; 
                    if(iteration > 150) break;
                }  

                std::cout<<std::fixed<<"\n"<<iteration<<"\t"<<states[L].back() <<"\t"<<improvement<<"\n"; 
                integrate(N,og,&control,BLUE,true); 
            }

            private:
            void optimize(int N, adore::env::OccupanyGrid* og)
            {
                   // createParallelSystems(N,&control);
                    evaluate(N,og);
                    addEpsilon();
                    evaluate(N,og);
                    gradient(N);
                    update_control(N);
                    integrate(N,og,&control,BLUE,false); 
            }
            void initHorizon(int N, adore::env::OccupanyGrid* og)
            {
                obj_F.clear();
                obj_F.resize(N);
                d_obj_F.clear();
                d_obj_F.resize(N);
                parallel_control.clear();
                parallel_control.resize(N);
                control_back.clear();
                control_back.resize(N);
                control.clear();
                states[X].clear();
                states[Y].clear();
                states[PSI].clear();
                states[S].clear();
                states[L].clear();
                states[X].push_back(pre_trajectory[0].x);
                states[Y].push_back(pre_trajectory[0].y);
                states[PSI].push_back(pre_trajectory[0].psi);
                states[S].push_back(0.0);
                states[L].push_back(0.0); 
                double tmp_x[nX], x[nX];
                for(int i=0; i<nX; ++i) tmp_x[i] = states[i][0];
               // std::memcpy(&tmp_x[0],&x[0],nX*sizeof(double));                
                for(int i=0; i<N; ++i)        
                {
                    double s = states[S].back();
                    int index = adore::mad::CubicPiecewiseFunction::findIndex(s,pp_d);
                    double input = adore::mad::CubicPiecewiseFunction::splineEvaluation(index,s,pp_d);
                    control.push_back(input);
                    ralston(og, &tmp_x[0],input,&x[0],dt);
                    for(int j=0; j<nX; ++j) {states[j].push_back(x[j]);}
                    std::memcpy(&tmp_x[0],&x[0],nX*sizeof(double)); 
                }  
                
               // std::cout<<"\n"<<states[Y].size();                           

                

            }
            void integrate(int N, adore::env::OccupanyGrid* og, boost::container::vector<double>*  u, std::string color, bool plot= false)
            {

                double temp_x0[nX];
                double rkXn[nX];
                for(int i=0; i<nX; ++i) temp_x0[i] = states[i][0];
                //while(temp_x0[S]<pre_trajectory[NumOfPoints-1].s )  
                for(int i=1; i<=N; ++i)        
                {
                    ralston(og, &temp_x0[0],(*u)[i-1],&rkXn[0],dt);
                    (std::memcpy)(&temp_x0[0],&rkXn[0],nX*sizeof(double));
                    for(int j=0; j<nX; ++j) {states[j][i] = (rkXn[j]);}
                }
                improvement = std::abs(states[L][N-1] - Lagrange_mem);
                Lagrange_mem = states[L][N-1] ;
                //std::cout<<"\nsize: "<<state_x.size();
                //std::cout<<"\n"<<states[Y].size();
               // if(figure != nullptr)figure->plot("#st",&states[X].data()[0],&states[Y].data()[0], 1.0, states[Y].size(), RED);
                //if(figure != nullptr)figure->plot("#rt",&pre_trajectory_x.data()[0],&pre_trajectory_y.data()[0], 1.1, pre_trajectory_x.size(), GREEN);  
              //if(plot)
              //{
                //  figure->plot(color,&states[X].data()[0],&states[Y].data()[0], 1.0, states[Y].size(), color);                
              //}

            }   
           double r_psi, t_psi, r_x, r_y, l_dist, penalty_delta;
            double obj_fun(adore::env::OccupanyGrid* og,double s, double psi, double x, double y, double u)
            {
                penalty_delta = 0.0;
                if(u > 0.4 || u < -0.4) penalty_delta = (u-0.4)*(u-0.4)*0.05;
                //r := reference, t:= trajectory (to be optimized)
                int index = adore::mad::CubicPiecewiseFunction::findIndex(s,pp_psi);
                r_psi = adore::mad::CubicPiecewiseFunction::splineEvaluation(index,s,pp_psi);
                //r_psi = adore::mad::CoordinateConversion::twoPIrange(r_psi);
                //t_psi = adore::mad::CoordinateConversion::twoPIrange(psi);
                r_x = adore::mad::CubicPiecewiseFunction::splineEvaluation(index,s,pp_x);
                r_y = adore::mad::CubicPiecewiseFunction::splineEvaluation(index,s,pp_y);
                l_dist = -sin(r_psi)* (r_x-x)+ cos(r_psi)* (r_y-y);    
                //std::cout<<std::fixed<<"\n"<<l_dist<<"\t"<<r_psi<<"\t"<<r_x<<"\t"<<x<<"\t"<<r_y<<"\t"<<y<<"\t"<<s<<"\t"<<index;
                //--------------
                //TO DO: Vehicle size must be considered
                ego2circles(x, y, psi);
                double  toleance, penalty_obstacle;
                toleance = 0.50;
                penalty_obstacle = 0.0;
                for(int i=0; i<obstacles.size(); i++)
                {
                    //penalty_obstacle += ellipsePenalty(og,&obstacles[i], x, y, toleance);
                    penalty_obstacle += circlePenalty(&obstacles[i],  toleance);
                    
                }



                //--------
               // std::cout<<"\n"<<penalty_obstacle;
                return (l_dist*l_dist*2e-3+ penalty_delta + penalty_obstacle*0.1);            
            }
            double ellipsePenalty(adore::env::OccupanyGrid* og,env::OccupanyGrid::_Obstacle* obst, double x, double y, double toleance)
            {
                double p = 0.0;
                double theta    = atan2((y-obst->y),(x-obst->x));
                double distance = sqrt   ( ((x-obst->x)*(x-obst->x))+ ((y-obst->y)*(y-obst->y)));
                double distance2Obst = distance - og->get_ellipse_r(obst->alpha + pi, obst->length, obst->width);
                if(distance2Obst > toleance) p += 0.0;
                else p += (distance2Obst - toleance)*(distance2Obst - toleance);
                return p;
            }
            double circlePenalty(env::OccupanyGrid::_Obstacle* obst, double toleance)
            {
                double p = 0.0;
                double distance ;
                double distance2Obst;
                for(int i=0; i<egoCircles.size(); i++)
                {
                    for(int j=0; j<obst->circles.size(); j++)
                    {
                        distance = sqrt   ( ((egoCircles[i].x - obst->circles[j].x)*(egoCircles[i].x- obst->circles[j].x))+ ((egoCircles[i].y- obst->circles[j].y)*(egoCircles[i].y- obst->circles[j].y)));
                        distance2Obst = distance - egoCircles[i].r - obst->circles[j].r;
                        if(distance2Obst > toleance) p += 0.0;
                        else p += (distance2Obst - toleance)*(distance2Obst - toleance);

                    }
                }
                return p;

            }
            void ego2circles(double x_o, double y_o, double theta)
            {
                egoCircles.clear();
                //egoCircles.resize(std::ceil(vehicleLength/0.5));
                double x,y;
                //std::stringstream tag;
                Circle tmp_circle;
                //std::cout<<"\n"<<vehicleLength<<"\t"<<std::ceil(vehicleLength/0.5);
                for(double  i=0.0; i<vehicleLength - vehicleWidth/4.; i=i+vehicleWidth*0.5)
                {
                    env::OccupanyGrid::transformation(x,y,theta, -vehicleLength/2.0 + i +  vehicleWidth/4., 0.0);
                    x += x_o ;
                    y += y_o ;
                    //tag << "circle"<<"-"<<i;
                    tmp_circle.x = x;
                    tmp_circle.y = y;
                    tmp_circle.r = vehicleWidth/2.;
                    egoCircles.push_back(tmp_circle);
                  //  egoCircles[i] = (tmp_circle);
                   // PLOT::plotCircle(tag.str(),x,y,0.10, r/2., figure,"LineColor=0.8,0.8,0.8;LineWidth=0;FillColor=1,0,0");
                }
                //std::cout<<"\nEgo circle size: "<<egoCircles.size();
            }
            void evaluate(int N, adore::env::OccupanyGrid* og)
            {
                int n = parallel_control.size();
                bool plot = false;
               for(int i=0 ; i<n ; ++i)        
               {
                   //plot = false;
                   //if(i==25) plot = true;
                integrate(N,og, &parallel_control[i],RED, plot); 
                obj_F[i] = (states[L].back() );
               } 
               /*
               double s = states[S].back(); 
               int index = adore::mad::CubicPiecewiseFunction::findIndex(s ,pp_x);
               double r_psi = adore::mad::CubicPiecewiseFunction::splineEvaluation(index,s,pp_psi);
               double r_x = adore::mad::CubicPiecewiseFunction::splineEvaluation(index,s,pp_x);
               double r_y = adore::mad::CubicPiecewiseFunction::splineEvaluation(index,s,pp_y); 
               double l_dist = -sin(r_psi)* (r_x-states[X].back())+ cos(r_psi)* (r_y-states[Y].back());
               std::cout<<"\n"<<  r_x <<"\t"<<r_y <<"\t"<<r_psi;
               std::cout<<"\n"<<  states[X].back() <<"\t"<<states[Y].back() <<"\t"<<s<<"\t"<<l_dist;
               obj_F[n-1] +=   l_dist;  
               */           
                            
            }
           
 

            void createParallelSystems(int N, boost::container::vector<double>*  delta)
            {
                for(int i=0; i<N; ++i)
                {
                    parallel_control[i] = (*delta);
                }
            }  
            void addEpsilon()
            {
                //control_back.clear();
                //control_back.insert(control_back.end(),parallel_control[0].begin(), parallel_control[0].end());
                control_back = parallel_control[0];
                obj_F_back = obj_F[0];
                for(int i=1; i<parallel_control.size(); ++i)
                {
                    parallel_control[i] = control_back;
                    parallel_control[i-1][i-1] += eps;
                }
                parallel_control[parallel_control.size()-1][parallel_control.size()-1] += eps;
            }  
            void gradient(int N)
            {
                parallel_control[0] = control_back;
                for(int i=0; i<N; ++i)
                {   
                    d_obj_F[i] = ( ((obj_F[i]) - (obj_F_back))/(eps));  
                    //std::cout<<"\n"<<i<<"\t"<<  d_obj_F[i];           
                }  
                obj_F[0] =  obj_F_back;          
            } 
            void update_control(int N)
            {
                for(int i=0; i<N; ++i)
                {
                    control[i] = control_back[i] - 2e-2*d_obj_F[i];
                    //std::cout<<"\n"<<i<<"\t"<<control_back[i]<<"\t"<<control[i];
                }
                control_back = control;
                parallel_control[0] = control;
            }

            void kinematicModel(adore::env::OccupanyGrid* og,double u, double *x, double *xN)
            {
                xN[X] = const_velocity*cos(x[PSI]);  //x
                xN[Y] = const_velocity*sin(x[PSI]);  //y
                xN[PSI] = const_velocity*tan(u)/vehicleLength;  //psi
                xN[S] = const_velocity;           //s
                xN[L] = obj_fun(og,x[S],x[PSI],x[X],x[Y],u);
            }
                double q1[nX];
                double q2[nX];
                double q3[nX];
                double q4[nX];
                double tmp_x[nX];
            void ralston(adore::env::OccupanyGrid* og,double *x, double u, double *xN, double dt)
            {
                dth = dt * 0.66666;
                kinematicModel(og, u,&x[0],&q1[0]);
                tmp_x[0]=x[0]+q1[0]*dth;
                tmp_x[1]=x[1]+q1[1]*dth;
                tmp_x[2]=x[2]+q1[2]*dth;
                tmp_x[3]=x[3]+q1[3]*dth;
                tmp_x[4]=x[4]+q1[4]*dth;
                kinematicModel(og,u,&tmp_x[0],&q2[0]); 
                xN[0]=x[0]+(dt*(0.250*q1[0]+ q2[0]*0.66666));
                xN[1]=x[1]+(dt*(0.250*q1[1]+ q2[1]*0.66666));
                xN[2]=x[2]+(dt*(0.250*q1[2]+ q2[2]*0.66666));
                xN[3]=x[3]+(dt*(0.250*q1[3]+ q2[3]*0.66666));
                xN[4]=x[4]+(dt*(0.250*q1[4]+ q2[4]*0.66666));
            }                 
            void rk4(double *x, double u, double *xN, double dt)
            {
                /*
                dth = dt * 0.500;
                kinematicModel(u,&x[0],&q1[0]);
                tmp_x[0]=x[0]+q1[0]*dth;
                tmp_x[1]=x[1]+q1[1]*dth;
                tmp_x[2]=x[2]+q1[2]*dth;
                tmp_x[3]=x[3]+q1[3]*dth;
                tmp_x[4]=x[4]+q1[4]*dth;
                kinematicModel(u,&tmp_x[0],&q2[0]);
                tmp_x[0]=x[0]+q2[0]*dth;
                tmp_x[1]=x[1]+q2[1]*dth;
                tmp_x[2]=x[2]+q2[2]*dth;
                tmp_x[3]=x[3]+q2[3]*dth;  
                tmp_x[4]=x[4]+q2[4]*dth;            
                kinematicModel(u,&tmp_x[0],&q3[0]);
                tmp_x[0]=x[0]+q3[0]*dt;
                tmp_x[1]=x[1]+q3[1]*dt;
                tmp_x[2]=x[2]+q3[2]*dt;
                tmp_x[3]=x[3]+q3[3]*dt;
                tmp_x[4]=x[4]+q3[4]*dt;
                kinematicModel(u,&tmp_x[0],&q4[0]);     
                xN[0]=x[0]+(dt*(q1[0]+ q2[0]*2.0 + q3[0]*2.0 +q4[0]))/6.0;
                xN[1]=x[1]+(dt*(q1[1]+ q2[1]*2.0 + q3[1]*2.0 +q4[1]))/6.0;
                xN[2]=x[2]+(dt*(q1[2]+ q2[2]*2.0 + q3[2]*2.0 +q4[2]))/6.0;
                xN[3]=x[3]+(dt*(q1[3]+ q2[3]*2.0 + q3[3]*2.0 +q4[3]))/6.0;
                xN[4]=x[4]+(dt*(q1[4]+ q2[4]*2.0 + q3[4]*2.0 +q4[4]))/6.0;
            */
            } 
            void toPolynom(int N, adore::mad::CubicPiecewiseFunction::PieceweisePolynomial* pp,Eigen::ArrayXd &breaks, Eigen::ArrayXXd &coefs)
            {
                for(int i=0; i<N-1; i++)
                {
                    coef1[i] = coefs(i,0);
                    coef2[i] = coefs(i,1);
                    coef3[i] = coefs(i,2);
                    coef4[i] = coefs(i,3);
                    breaks[i]=breaks(i);
                    //std::cout<<"\n"<<breaks[i];
                }
                breaks[N-1] = breaks(N-1);
                adore::mad::CubicPiecewiseFunction::toPolynomialFrom(pp,&breaks[0],&coef1[0],&coef2[0],&coef3[0],&coef4[0],N-1);           
            }                   
        };
    }
}