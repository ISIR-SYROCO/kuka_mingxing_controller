// Filename:  kukaMingXingController-rtnetcomponent.cpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  

#include "kukaMingXingController-rtnetcomponent.hpp"
#include <rtt/Component.hpp>
#include <iostream>

KukaMingXingControllerRTNET::KukaMingXingControllerRTNET(std::string const& name) : FriRTNetExampleAbstract(name){

}

void KukaMingXingControllerRTNET::updateHook(){
   fri_frm_krl = m_fromFRI.get(); 
   if(fri_frm_krl.intData[0] == 1){ //command mode

       std::vector<double> JState(LWRDOF);
       std::vector<double> JVel(LWRDOF);
       RTT::FlowStatus joint_state_fs = iport_msr_joint_pos.read(JState);
       RTT::FlowStatus joint_vel_fs = iport_msr_joint_vel.read(JVel)

       if(joint_state_fs == RTT::NewData){
       }

       //Update Model
       //Set task projector
       //Update projector
       //Compute tau
       //Send tau

   }

}


/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(kuka_ming_xing_controller)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(KukaMingXingControllerRTNET)
