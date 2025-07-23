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
#include <algo/collision_check_offline.h>
//#include <plotlablib/afigurestub.h>
//#include <plotlablib/figurestubfactory.h>
//#include <adore/apps/if_plotlab/plot_shape.h>
#include <mad/coordinateconversion.h>
#include <mad/arraymatrixtools.h>



namespace adore
{
	namespace fun
	{
	   /**
	   *  ----------------------
	   */
       namespace bg = boost::geometry; 
       template <int TYPE, typename T>
       	class Node
		{
            private:
            std::string RED= "LineColor=1.,0.,0.;LineWidth=3";
            static const int N_suc = 3;
            // H := Holonomic,   nH := non Holonomic
            double dx_H[N_suc];
            double dy_H[N_suc];
            double dpsi_H[N_suc]; 
            double R;   //Maximum Turning radius
            double dA;  // Dubins Curve Angle
            double G;   //cost so far
            double H;   // cost to go        
            int type;    
            double cp; //cos(psi)
            double sp; //sin(psi)
            long double pi, TWO_PI;



            

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
            isOpen   = false;
            isVisited = false;
            isGloballyVisited = false;
            R = 2.0;        //Maximum turning radius
            dA = adore::mad::CoordinateConversion::DegToRad(45);//(2*6.75);   //Dubins angle 6.75
            dx_H[0] = 1.0; //1.0;
            dx_H[1] = 1.0; //1.0;
            dx_H[2] = 1.0; //1.0;
            dy_H[0] = 1.0;//1.0;
            dy_H[1] = 0.0;
            dy_H[2] = -1.0; //-1.0;    
            dpsi_H[0] =  adore::mad::CoordinateConversion::DegToRad(45.0);
            dpsi_H[1] = 0.0;        
            dpsi_H[2] = adore::mad::CoordinateConversion::DegToRad(-45.0);
            pi = 3.141592653589793;
            TWO_PI = 2.00000000 * pi;  

        }
        void set_R (double R) {this->R = R;}
        void set_dA (double dA) {this->dA = dA;}
        void set_G(double G) {this->G = G;}
        void set_H(double H) {this->H = H;}
        void update_index(int Width, int Length, int Depth, double HeadingResolutionRad)
        {
            index_depth = int( mad::ArrayMatrixTools::mod(this->psi,TWO_PI) /HeadingResolutionRad);
            //std::cout<<"index depth : "<<index_depth<<std::endl;
            index_width = int(this->y);
            index_length = int(this->x);
            index_depth = std::min(index_depth, Depth-1);
            //std::cout<<"min depth : "<<index_depth<<std::endl;
        }
        bool setPosition(double x, double y, double psi ,int Width, int Length, int Depth, double HeadingResolutionRad)
        {
            if(x >= 0.0 && x <Length && y>=0.0 && y< Width)
            {
                this->x = x;
                this->y = y;
                this->psi = psi;
                this->update_index(Width, Length, Depth, HeadingResolutionRad);
                /*if(figure != nullptr)
                {
                    std::stringstream ss;
                    ss.str("");
                    ss << "f"<<x+y;
                    PLOT::plotArrow(ss.str(),x,y,1.0,psi,1.0,0.50,RED,figure);
                }*/
                return true;
            }
            else
            {
                 std::cout<<"\nWRONG COORDINATES";
                 return false;
            }
        }
        bool hasEqualIndex(Node* node, double headingResolution)
        {
            return(((std::fmod(node->psi,2.0*PI) /headingResolution) == (std::fmod(this->psi,2.0*PI) /headingResolution)) 
                    && (int(node->y) == int(this->y)) && (int(node->x) == int(this->x)) ); 
        }
        bool isEqual (Node* node, double headingResolution = 5.0, double tolerance = 0.1) //tolerance = 0.1
        {
            bool result = false;
            if(this->type == 2 && this->x == node->x && this->y == node->y )
            {
                result = true;
            }
            if(this->type == 3 && std::abs(this->x - node->x) < tolerance && 
                std::abs(this->y - node->y) < tolerance &&
               ( std::abs(this->psi - node->psi) <= headingResolution || std::abs(this->psi - node->psi) >= 2*PI - headingResolution) ) 
            {
                result = true;
            }  
            return result;          
        }
        bool isCloseTo (Node* node, double tolerance = 10.0)
        {
            return ( ( (this->x - node->x)*(this->x - node->x) + (this->y - node->y)*(this->y - node->y))< tolerance*tolerance );

        }
        double get_G (Node* predecessor)
        {
            double EucDist_sq = (this->x - predecessor->x)*(this->x - predecessor->x) + (this->y - predecessor->y)*(this->y - predecessor->y);
            this->isVisited = true;
            if(this->type == 2)   {this->G += EucDist_sq;} 
            if(this->type == 3)   {this->G += std::sqrt(EucDist_sq);} 
            return this->G;
        }
        double get_G() 
        {
            return this->G;
        }
        


        double get_H(Node* goal) //only for type 2, Holonomic
        {
          this->H = (this->x - goal->x)*(this->x - goal->x) + (this->y - goal->y)*(this->y - goal->y);
          return this->H ;
        }  
        void update_C()
        {
            this-> C = this->G + this->H;
        }

        std::vector<Node< 2,  int>*>  updateSuccessors2D(adore::env::OccupanyGrid* og)
        {
                std::cout<< "expanding 2D"<<std::endl;
                int gridWidth = og->Grid.rows();
                int gridLength = og->Grid.cols();
                std::vector<Node< 2,  int>*> successors_h; 
                for(int i=0; i<N_suc; i++)
                {
                    Node<2,int>* tmp = new Node<2,int>;
                    tmp->x = this->x + dx_H[i];
                    tmp->y = this->y + dy_H[i];
                    tmp->psi = dpsi_H[i];
                    tmp->set_G(this->G);
                    //std::cout<<"\n"<<tmp->x<<"\t"<<tmp->y;
                    if(tmp->x >= 0.0 && tmp->x <gridLength && tmp->y>=0.0 && tmp->y< gridWidth && og->Grid(tmp->y, tmp->x)<1.00)
                    {
                       // std::cout<<"\tis added";
                        successors_h.push_back(tmp);
                    }
                }
                return successors_h;
        }

        std::vector<Node< 3,  double>*>  updateSuccessors3D(adore::env::OccupanyGrid* og,adore::fun::CollisionCheckOffline* cco, long double HeadingResolutionRad)
        {
                //this->print();
                std::cout<<"expanding..."<<std::endl;
                this->R = 5;
                int gridWidth = og->Grid.rows();
                int gridLength = og->Grid.cols();
                int gridDepth = cco->offlineCollisionTable.size();                       
                std::vector<Node< 3,  double>*> successors_nh; 
                cp = std::cos(this->psi);
                sp = std::sin(this->psi);
                Node<3,double>* tmp = new Node<3,double>;
                Node<3,double>* tmp1 = new Node<3,double>;
                Node<3,double>* tmp2 = new Node<3,double>;
                tmp->x = this->x + this->R*(cp*this->dA);
                tmp->y = this->y + this->R*(sp*this->dA);
                tmp->psi = this->psi;
                tmp->set_G(this->G);

                tmp->update_index(gridWidth, gridLength, gridDepth,HeadingResolutionRad);
                if(tmp->x >= 0.0 && tmp->x <gridLength && tmp->y>=0.0 && tmp->y< gridWidth && isCollisionFree(tmp,og, cco, HeadingResolutionRad))
                {
                    //std::cout<<"node 1   : ";
                    //tmp->print();
                    successors_nh.push_back(tmp); 
                }
                else{
                    std::cout<<"node 1 not accepted  : ";
                    tmp->print();
                }
                //---                
                tmp1->x = this->x + this->R*(std::sin(this->psi+this->dA)-sp);
                tmp1->y = this->y + this->R*(-std::cos(this->psi+this->dA)+cp);
                tmp1->psi = this->psi + this->dA;
                tmp1->set_G(this->G);
                tmp1->update_index(gridWidth, gridLength, gridDepth,HeadingResolutionRad);
                if(tmp1->x >= 0.0 && tmp1->x <gridLength && tmp1->y>=0.0 && tmp1->y< gridWidth && isCollisionFree(tmp1,og, cco, HeadingResolutionRad))
                {
                    //std::cout<<"node 2   : ";
                    //tmp1->print();
                    successors_nh.push_back(tmp1);
                }
                else{
                    std::cout<<"node 2 not accepted  : ";
                    tmp1->print();
                }
                //---                
                tmp2->x = this->x + this->R*(-std::sin(this->psi-this->dA)+sp);
                tmp2->y = this->y + this->R*(std::cos(this->psi -this->dA)-cp);
                tmp2->psi = this->psi - this->dA;
                tmp2->set_G(this->G);
                tmp2->update_index(gridWidth, gridLength, gridDepth,HeadingResolutionRad);
                if(tmp2->x >= 0.0 && tmp2->x <gridLength && tmp2->y>=0.0 && tmp2->y< gridWidth && isCollisionFree(tmp2,og, cco, HeadingResolutionRad))
                {
                    //std::cout<<"node 3   : ";
                    //tmp2->print();
                    successors_nh.push_back(tmp2);
                    //successors_nh.back()->print();
                }  
                else{
                    std::cout<<"node 3 not accepted  : ";
                    tmp2->print();
                }
                return successors_nh;      
            }
            
        bool isCollisionFree(Node<3,double>* node,adore::env::OccupanyGrid* og, adore::fun::CollisionCheckOffline* cco, long double HeadingResolutionRad)
        {
            int X, Y;
            int gridWidth = og->Grid.rows();
            int gridLength = og->Grid.cols();        
            int index_x = std::max(int((node->x - long(node->x))* cco->NumberOfPointsPerAxis), -100);
            int index_y = std::max(int((node->y - long(node->y))* cco->NumberOfPointsPerAxis), -100);
            int index_psi = this->index_depth;
            for(int i=0; i<cco->offlineCollisionTable[index_psi](index_x,index_y).size() ; ++i)
            {
                X = int(node->x) + bg::get<0>(cco->offlineCollisionTable[index_psi](index_x,index_y)[i]);
                Y = int(node->y) + bg::get<1>(cco->offlineCollisionTable[index_psi](index_x,index_y)[i]);
                if(X >= 0 && X < gridLength && Y >= 0 && Y < gridWidth)
                {
                    if(og->Grid(Y, X)>0.900 || og->Grid(Y, X) < 0.0)
                    {                      
                        return false;
                    }
                }
                
            }
               // std::cout<<"\nreturn TRUE";std::cout<<"\nreturn TRUE";std::cout<<"\nreturn TRUE";
            return true;
       }
        
        Node<2,int>* nH2H()
        {
            Node<2,int>* tmp = new Node<2,int>;
            tmp->x = std::floor(this->x);
            tmp->y = std::floor(this->y);
            tmp->psi = this->psi;
           // std::cout<<"\n"<<tmp->x <<"\t"<<tmp->y;
            return tmp;
        }
        void print()
        {
            std::cout<<"\nnode: ";
            std::cout<<std::fixed<<this->x<<"\t"<<this->y<<"\t"<<this->psi<<"\t"<<this->G<<"\t"<<this->H<<"\t"<<this->C<<"\n";
        }





        };
    }
}
