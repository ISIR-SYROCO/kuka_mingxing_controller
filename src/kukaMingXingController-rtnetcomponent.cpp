// Filename:  kukaMingXingController-rtnetcomponent.cpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  

#include "kukaMingXingController-rtnetcomponent.hpp"
#include <rtt/Component.hpp>
#include <iostream>

KukaMingXingControllerRTNET::KukaMingXingControllerRTNET(std::string const& name) : FriRTNetExampleAbstract(name){
    this->addOperation("setQ1des", &KukaMingXingControllerRTNET::setQdesTask1, this, RTT::OwnThread);
    this->addOperation("setParamPriority", &KukaMingXingControllerRTNET::setParamPriority, this, RTT::OwnThread);
    this->addOperation("setT1Stiffness", &KukaMingXingControllerRTNET::setStiffnessTask1, this, RTT::OwnThread);
    this->addOperation("setT1Damping", &KukaMingXingControllerRTNET::setDampingTask1, this, RTT::OwnThread);
    model = new kukafixed("kuka");
    ctrl = new orcisir::GHCJTController("myCtrl", *model, solver, true, false);
    FMS = new orc::FullModelState("torqueTask.FModelState", *model, orc::FullState::INTERNAL);
    FTS = new orc::FullTargetState("torqueTask.FTargetState", *model, orc::FullState::INTERNAL);
    feat = new orc::FullStateFeature("torqueTask", *FMS);
    featDes = new orc::FullStateFeature("torqueTask.Des", *FTS);

	joint_position_command.assign(LWRDOF, 0.0);

    qdes_task1.resize(7);
    qdes_task1<<0.0,-0.05,0.0,1.5,0.0,-1.2,0.0;
    tau.resize(7);
    tau.setConstant(0);
    FTS->set_q(qdes_task1);
	
    accTask = &(ctrl->createGHCJTTask("accTask", *feat, *featDes));

    ctrl->addTask(*accTask);
    accTask->activateAsObjective();
    accTask->setStiffness(0.001);
    accTask->setDamping(0.000);

    ctrl->setActiveTaskVector();

    // SET TASK PRIORITIES, COMPUTE TASK PROJECTORS
    int nt = ctrl->getNbActiveTask();

    param_priority.resize(nt,nt);
    param_priority<<0;
    ctrl->setTaskProjectors(param_priority);

    //ctrl->setGSHCConstraint();
}

void KukaMingXingControllerRTNET::updateHook(){

    std::string fri_mode("e_fri_unkown_mode");
    bool fri_cmd_mode = false;
    RTT::FlowStatus fs_event = iport_events.read(fri_mode);
    if (fri_mode == "e_fri_cmd_mode")
        fri_cmd_mode = true;
    else if (fri_mode == "e_fri_mon_mode")
        fri_cmd_mode = false;
        
    std::vector<double> JState(LWRDOF);
    std::vector<double> JVel(LWRDOF);
    RTT::FlowStatus joint_state_fs = iport_msr_joint_pos.read(JState);
    RTT::FlowStatus joint_vel_fs = iport_msr_joint_vel.read(JVel);

    if(joint_state_fs == RTT::NewData){        
        Eigen::VectorXd joint_pos(7);
        
        for(unsigned int i = 0; i < LWRDOF; i++){
            joint_pos[i] = JState[i];
            joint_position_command[i] = JState[i];
        }
        
        model->setJointPositions(joint_pos);
	//std::cout << "Jpos " << joint_pos.transpose() << std::endl;
    }
    
    if(joint_vel_fs == RTT::NewData){
        Eigen::VectorXd joint_vel(7);
        for(unsigned int i = 0; i < LWRDOF; i++){
            joint_vel[i] = JVel[i];
        }
        model->setJointVelocities(joint_vel);
    }

    //Set task projector
    ctrl->setTaskProjectors(param_priority);
    //Update projector
    ctrl->doUpdateProjector();
    //Compute tau
    ctrl->computeOutput(tau); 
    //std::cout << tau.transpose() << std::endl;
    //Send tau
    if (fri_cmd_mode){
        if(requiresControlMode(30)){
            std::vector<double> joint_eff_command;
            joint_eff_command.assign(LWRDOF, 0.0);
            for(unsigned int i=0; i<LWRDOF; ++i){
                joint_eff_command[i] = tau[i];
            }
            oport_add_joint_trq.write(joint_eff_command);
        }
        oport_joint_position.write(joint_position_command);
    }
}

void KukaMingXingControllerRTNET::setParamPriority(std::vector<double> &param){
    int nt = ctrl->getNbActiveTask();
    if(param.size() != nt*nt){
        std::cout << "Incorrect size" << std::endl;
        return;
    }    
    else{
        //Row Major
        for(int i=0; i<nt; ++i){
            for(int j=0; j<nt; ++j){
                param_priority(i,j) = param[i*nt+j];
            }
        }
        ctrl->setTaskProjectors(param_priority);
        ctrl->doUpdateProjector();
    }    
}

void KukaMingXingControllerRTNET::setQdesTask1(std::vector<double> &qdes){
    for(unsigned int i = 0; i < LWRDOF; i++){
        qdes_task1[i] = qdes[i];
    }
    FTS->set_q(qdes_task1);
}

void KukaMingXingControllerRTNET::setStiffnessTask1(double &stiffness){
    accTask->setStiffness(stiffness);
}

void KukaMingXingControllerRTNET::setDampingTask1(double &damping){
    accTask->setDamping(damping);
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
