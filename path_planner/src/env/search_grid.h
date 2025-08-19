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
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/container/vector.hpp>
#include "../mad/arraymatrixtools.h"

#include <vector>
namespace adore
{
	namespace fun
	{
	   /**
	   *  ----------------------
	   */
       
       template <typename T>
        class GRID
		{
            private:
            static const int nH_Type = 3;  //non holonomic
            static const int H_Type = 2;  //holonomic
            public:
            //std::vector<boost::numeric::ublas::matrix<T>> grid;
            
            boost::container::vector<boost::numeric::ublas::matrix<T>> grid;
            boost::numeric::ublas::matrix<T> matrix_2d;
            int Length;
            int Width;
            int Depth;
            double BIG_NUMBER;
            double pi, TWO_PI;
            GRID()
            {
                pi = 3.141592653589793;
                TWO_PI = 2.0000 * pi;                
                this->Length = 0;
                this->Width = 0;
                this->Depth = 0; 
                BIG_NUMBER = 1e3;               
            }
            double get_Length() {return Length;}
            double get_Width() {return Width;}
            double get_Depth() {return Depth;}
            void resize(int Width, int Length, int Depth=1)
            {
                //Eigen::Matrix<T,Width,Length> test;
                grid.clear();      
                std ::cout<<"nh clear init"<<std::endl;          
                matrix_2d.resize(Width,Length);
                this->Length = Length;
                this->Width = Width;
                this->Depth = Depth;//Depth;
                std ::cout<<"nh befor loop "<<std::endl;
                std ::cout<<Depth<<std::endl;
                for (int i=0; i<this->Depth; i++)
                {
                    std ::cout<<i<<std::endl;
                    grid.push_back(matrix_2d);
                }
                std ::cout<<"nh after loop "<<std::endl;
            }
   
            
            void initialize(bool PRINT=false)
            {
                if(PRINT)
                {
                    std::cout<<"\nLength: "<<Length<<"\tWidth: "<<Width<<"\tDepth: "<<Depth;
                    std::cout<<"\n"<<grid.size()<<"\t"<<grid[0].size1()<<"\t"<<grid[0].size2();
                }
                for (std::size_t d = 0; d< Depth; ++d)
                {
                    for(std::size_t  r = 0; r< Width; ++r)
                    {
                        for(std::size_t  c=0; c<Length; ++c)
                        {
                            grid[d](r,c).isClosed = false; 
                            grid[d](r,c).isOpen = false; 
                            grid[d](r,c).set_G(BIG_NUMBER); 
                        }
                    }
                }
 
                
                
            } 
            void replace(Node<nH_Type,double>* node, double HeadingResolutionRad)
            {
                
                grid[node->index_depth](node->index_width, node->index_length) = *node;
            }            
            void replace(Node<H_Type,int>* node)
            {
                grid[0](node->y, node->x) =*node;
            }
            bool isClosed(Node<nH_Type,double>* node,double HeadingResolutionRad)
            {
                //std::cout<<"\nisCLosed "<<mad::ArrayMatrixTools::mod(node->psi,TWO_PI) <<"\t"<<HeadingResolutionRad<<"\t"<<mad::ArrayMatrixTools::mod(node->psi,TWO_PI) /HeadingResolutionRad<<"\t"<<std::floor( mad::ArrayMatrixTools::mod(node->psi,TWO_PI) /HeadingResolutionRad)<<"\t"<<int(node->y)<<"\t"<<int(node->x)<<"\n";
               return (grid[node->index_depth](node->index_width, node->index_length).isClosed);
            }             
            bool isClosed(Node<H_Type,int>* node)
            {
               return (grid[0](node->y, node->x).isClosed);
            } 
            bool isOpen(Node<nH_Type,double>* node,double HeadingResolutionRad)
            {
               return (grid[node->index_depth](node->index_width, node->index_length).isOpen);
            }             
            bool isOpen(Node<H_Type,int>* node)
            {
               return (grid[0](node->y, node->x).isOpen);
            }
            double get_G(Node<nH_Type,double>* node,double HeadingResolutionRad)
            {
               return (grid[node->index_depth](node->index_width, node->index_length).get_G());
            }             
            double get_G(Node<H_Type,int>* node)
            {
               return (grid[0](node->y, node->x).get_G());
            } 
            void set_closed(Node<nH_Type,double>* node,double HeadingResolutionRad)
            {
                 // std::cout<<"\nSetCLosed "<<int( mad::ArrayMatrixTools::mod(node->psi,TWO_PI) /HeadingResolutionRad)<<"\t"<<int(node->y)<<"\t"<<int(node->x);
              grid[node->index_depth](node->index_width, node->index_length) .isClosed = true;
              grid[node->index_depth](node->index_width, node->index_length) .isOpen = false;
            }                           
            void set_visited(Node<H_Type,int>* node)
            {
              grid[0](node->y, node->x).isVisited = true;  
              grid[0](node->y, node->x).isClosed = true;
              grid[0](node->y, node->x).isOpen = false;
            }                                            

        };
        
        template <typename T>
        class ArrayFormGrid
        {
             public:
            T* grid; 
            int Length;
            int Width;
            int Depth;
            double BIG_NUMBER;
            ArrayFormGrid()
            {
                this->Length = 0;
                this->Width = 0;
                this->Depth = 0; 
                BIG_NUMBER = 1e3;               
            }
            double get_Length() {return Length;}
            double get_Width() {return Width;}
            double get_Depth() {return Depth;}
            void resize(int Width, int Length, int Depth=1)
            {
                grid = new T[Width*Length*Depth]();
                this->Length = Length;
                this->Width = Width;
                this->Depth = Depth;
            }
            void clear()
            {
                //delete[] grid;
            }
   
            
            void initialize(bool PRINT=false)
            {
                if(PRINT)
                {
                    std::cout<<"\nLength: "<<Length<<"\tWidth: "<<Width<<"\tDepth: "<<Depth;
                }
                if(Depth == 1)
                {
                    for(int r = 0; r< Width; r++)
                    {
                        for(int c=0; c<Length; c++)
                        {
                            //std::cout<<"\n"<<r*Length+c;
                            grid[r*Length+c].isClosed = false; 
                            grid[r*Length+c].isOpen = false; 
                            grid[r*Length+c].set_G(BIG_NUMBER); 
                            
                        }
                    }
                }
                else
                {
                    /*
                    for(int d=0; d<Depth; d++)
                    {
                     for(int r = 0; r< Width; r++)
                    {
                        for(int c=0; c<Length; c++)
                        {
                            //std::cout<<"\n"<<d*Length*Depth + r*Length+c;
                            grid[d*Length*Depth + r*Length+c].isClosed = false; 
                            grid[d*Length*Depth + r*Length+c].isOpen = false; 
                            grid[d*Length*Depth + r*Length+c].set_G(BIG_NUMBER); 
                            
                        }
                    }                       
                    }
                    */
                    for (std::size_t  c=0; c<Width*Length*Depth; ++c)
                    {
                        grid[c].isClosed = false; 
                        grid[c].isOpen = false; 
                        grid[c].set_G(BIG_NUMBER);                      
                    }
                }
             
                
            } 
            void replace(Node<2,int>* node)
            {
                grid[node->y*Length + node->x] = *node;
            }
            bool isClosed(Node<2,int>* node)
            {
               return (grid[node->y*Length + node->x].isClosed);
            }  
            bool isOpen(Node<2,int>* node)
            {              
               return (grid[node->y*Length + node->x].isOpen);
            }
            double get_G(Node<2,int>* node)
            {               
               return (grid[node->y*Length + node->x].get_G());
            }               
            void set_visited(Node<2,int>* node)
            {               
              grid[node->y*Length + node->x].isVisited = true;  
              grid[node->y*Length + node->x].isClosed = true;
              grid[node->y*Length + node->x].isOpen = false;
            }                                            
           

        };
    }
}