// Filename:  kukaModelFromFri-rtnetcomponent.hpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description: Orocos component using RTNET to compute the kuka model
//              from fri data

#ifndef KUKA_MODEL_FROM_FRI_RTNET_COMPONENT_HPP
#define KUKA_MODEL_FROM_FRI_RTNET_COMPONENT_HPP

#include <friRTNetExampleAbstract.hpp>
#include <Eigen/Dense>

class KukaMingXingControllerRTNET : public FriRTNetExampleAbstract{
    public:
        KukaMingXingControllerRTNET(std::string const& name);

        RTT::InputPort<double> port_i;

        RTT::OutputPort<double> port_o;

        void updateHook();
};

#endif
