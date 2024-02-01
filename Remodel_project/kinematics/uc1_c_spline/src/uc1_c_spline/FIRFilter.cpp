/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Agent.cpp
 * Author: Davide Chiaravalli, Federico Califano
 * 
 * Created on Sep 8, 2016, 2:05 PM
 */

#include "FIRFilter.h"


namespace fir_filter {
	
    FIRFilter::FIRFilter(double period,double Ts) 
    {
        this->period=period;						//time period of the spline
        this->sampling_time=Ts;				//sampling time of the filters used to evaluate the spline
        this->cell_number=(int)(this->period/this->sampling_time);			//number of cells of the filters
        this->next_input_position=0;
        this->state.resize(this->cell_number);
        
        Vector6d inizializer;
        inizializer.setZero();
        this->initialiseFilter(inizializer);
        this->sum << 0,0,0,0,0,0;
    }

    FIRFilter::~FIRFilter() 
    {
    }

    void FIRFilter::initialiseFilter(Vector6d initial_value)
    {

        for(int i=0;i<this->cell_number;i++)
                this->state[i]=initial_value;
        this->resetInputPosition();
        
        for (int i=0;i<this->cell_number;i++)
            this->sum = this->sum + this->state[i];
    }

    void FIRFilter::setFilterParameters(double period, double Ts)
    {
        this->period=period;						
        this->sampling_time=Ts;
        // std::cout<<period<<std::endl;
        // std::cout<<Ts<<std::endl;
        this->cell_number=(int)(this->period/this->sampling_time);
        // std::cout<<"celle"<<this->cell_number<<std::endl;
        std::vector<Vector6d> temp;
        temp.resize(this->cell_number);
        this->state=temp;
        //this->initialiseFilter(Eigen::Vector3d(0,0,0));
    }
    
    void FIRFilter::resetInputPosition()
    {
        this->next_input_position=0; 
    }

    void FIRFilter::addNewInput(Vector6d input)
    {
        this->sum = this->sum - this->state[this->next_input_position] + input;
        this->state[next_input_position] = input;
        this->next_input_position = (this->next_input_position+1)%this->cell_number;
    }

    Vector6d FIRFilter::evaluateFilterOutput()
    {
        return this->sum/this->cell_number;
    }

    int FIRFilter::getFilterCellNumber()
    {
        return this->cell_number;
    }
}
