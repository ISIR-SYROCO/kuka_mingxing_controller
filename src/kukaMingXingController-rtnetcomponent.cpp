// Filename:  kukaMingXingController-rtnetcomponent.cpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  

#include "kukaMingXingController-rtnetcomponent.hpp"
#include <rtt/Component.hpp>
#include <iostream>

KukaMingXingControllerRTNET::KukaMingXingControllerRTNET(std::string const& name) : FriRTNetExampleAbstract(name){
    model = new kukafixed("kuka");
    ctrl = new orcisir::GHCJTController("myCtrl", *model, solver, true, false);
    FMS = new orc::FullModelState("torqueTask.FModelState", *model, orc::FullState::INTERNAL);
    FTS = new orc::FullTargetState("torqueTask.FTargetState", *model, orc::FullState::INTERNAL);
    feat = new orc::FullStateFeature("torqueTask", *FMS);
    featDes = new orc::FullStateFeature("torqueTask.Des", *FTS);
	
	Eigen::VectorXd qdes_task1(7);
	qdes_task1<<0.25,0.25,0.25,0.25,0.25,0.25,0.25;
	FTS->set_q(qdes_task1);
	
	accTask = &(ctrl->createGHCJTTask("accTask", *feat, *featDes));

	ctrl->addTask(*accTask);
	accTask->activateAsObjective();
	accTask->setStiffness(0.05);
	accTask->setDamping(0.02);

    ctrl->setActiveTaskVector();

    // SET TASK PRIORITIES, COMPUTE TASK PROJECTORS
    int nt = ctrl->getNbActiveTask();

    param_priority.resize(nt,nt);
    param_priority<<1;
    ctrl->setTaskProjectors(param_priority);

    ctrl->setGSHCConstraint();
}

void KukaMingXingControllerRTNET::updateHook(){
   fri_frm_krl = m_fromFRI.get(); 
   if(fri_frm_krl.intData[0] == 1){ //command mode

       std::vector<double> JState(LWRDOF);
       std::vector<double> JVel(LWRDOF);
       RTT::FlowStatus joint_state_fs = iport_msr_joint_pos.read(JState);
       RTT::FlowStatus joint_vel_fs = iport_msr_joint_vel.read(JVel);

       if(joint_state_fs == RTT::NewData){
           Eigen::VectorXd joint_pos(JState.data());
       }
       if(joint_vel_fs == RTT::NewData){
           Eigen::VectorXd joint_vel(JVel.data());
       }

       //Update Model
       model->setJointPositions(joint_pos);
       model->setJointVelocities(joint_vel);

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
