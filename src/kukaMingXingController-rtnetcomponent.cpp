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
    this->addOperation("setStiffness", &KukaMingXingControllerRTNET::setStiffnessTask, this, RTT::OwnThread);
    this->addOperation("setDamping", &KukaMingXingControllerRTNET::setDampingTask, this, RTT::OwnThread);
    this->addOperation("setEEdes", &KukaMingXingControllerRTNET::setEEdesTask2, this, RTT::OwnThread);
    

    eq.resize(LWRDOF);
    deq.resize(LWRDOF);

    model = new kukafixed("kuka");
    ctrl = new orcisir::GHCJTController("myCtrl", *model, solver, true, false);
    
    FMS = new orc::FullModelState("torqueTask.FModelState", *model, orc::FullState::INTERNAL);
    FTS = new orc::FullTargetState("torqueTask.FTargetState", *model, orc::FullState::INTERNAL);
    feat = new orc::FullStateFeature("torqueTask", *FMS);
    featDes = new orc::FullStateFeature("torqueTask.Des", *FTS);
    qdes_task1.resize(7);
    qdes_task1<<0.0,-0.05,0.0,1.5,0.0,-1.2,0.0;
    FTS->set_q(qdes_task1);
    
    accTask = &(ctrl->createGHCJTTask("accTask", *feat, *featDes));

    ctrl->addTask(*accTask);
    accTask->activateAsObjective();
    accTask->setStiffness(0.001);
    accTask->setDamping(0.000);
    
    SF = new orc::SegmentFrame("frame.SFrame", *model, "kuka.07", Eigen::Displacementd());
    TF = new orc::TargetFrame("frame.TFrame", *model);
    feat2 = new orc::PositionFeature("frame", *SF, orc::XYZ);
    featDes2 = new orc::PositionFeature("frame.Des", *TF, orc::XYZ);
    posdes_task2 = Eigen::Displacementd(0.3,0.0,0.5);
    TF->setPosition(posdes_task2);//0.3,0.0,0.5
    TF->setVelocity(Eigen::Twistd());
    TF->setAcceleration(Eigen::Twistd());

    accTask2 = &(ctrl->createGHCJTTask("accTask2", *feat2, *featDes2));
    
    ctrl->addTask(*accTask2);
    accTask2->activateAsObjective();
    accTask2->setStiffness(16);//30,18 ; 16,12 ; 5,7
    accTask2->setDamping(12);

	joint_position_command.assign(LWRDOF, 0.0);

    tau.resize(7);
    tau.setConstant(0);

    ctrl->setActiveTaskVector();

    // SET TASK PRIORITIES, COMPUTE TASK PROJECTORS
    int nt = ctrl->getNbActiveTask();

    param_priority.resize(nt,nt);
    param_priority<<0, 1, 0, 0;
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
    
    Eigen::VectorXd joint_pos(7);
    if(joint_state_fs == RTT::NewData){        
        
        
        for(unsigned int i = 0; i < LWRDOF; i++){
            joint_pos[i] = JState[i];
            joint_position_command[i] = JState[i];
        }
        
        model->setJointPositions(joint_pos);
	//std::cout << "Jpos " << joint_pos.transpose() << std::endl;
    }
    
    Eigen::VectorXd joint_vel(7);
    if(joint_vel_fs == RTT::NewData){
        
        for(unsigned int i = 0; i < LWRDOF; i++){
            joint_vel[i] = JVel[i];
        }
        model->setJointVelocities(joint_vel);
    }
    
    Eigen::MatrixXd Jac(3,7);
    Eigen::Displacementd posEndEffMes;
    RTT::FlowStatus jacobian_fs = jacobianPort.read(J);
    if(jacobian_fs==RTT::NewData){   
        Eigen::MatrixXd JacFull(6,7); 
		JacFull.noalias() = J.data;
		Jac = Jac.block(3,0, 3,7);
    }
    RTT::FlowStatus cartPos_fs =  iport_cart_pos.read(X);
    if(cartPos_fs==RTT::NewData){
        double x = (double)X.position.x;
        double y = (double)X.position.y;
        double z = (double)X.position.z;
        posEndEffMes.x()=x;
        posEndEffMes.y()=y;
        posEndEffMes.z()=z;
    }
        
    //Set task projector
    ctrl->setTaskProjectors(param_priority);

    // UpdateAugmentedJacobian
    Eigen::MatrixXd augmentedJacobian = MatrixXd::Zero(feat->getDimension()+feat2->getDimension(), LWRDOF);
    int rowindex = 0;
    int taskdim;

    taskdim = feat->getDimension();
    augmentedJacobian.block(rowindex,0, taskdim, LWRDOF)=accTask->getJacobian();
    rowindex += taskdim;

    taskdim = feat2->getDimension();
    augmentedJacobian.block(rowindex,0, taskdim, LWRDOF)=Jac;
    
    //Update projector
    Eigen::MatrixXd proj(LWRDOF,LWRDOF);
    computeProjector(accTask->getPriority(), augmentedJacobian, proj);
    accTask->setProjector(proj);
    Eigen::MatrixXd proj2(LWRDOF,LWRDOF);
    computeProjector(accTask2->getPriority(), augmentedJacobian, proj2);
    accTask2->setProjector(proj2);
    
    //Compute tau
    ctrl->computeOutput(tau); 

    
    eq = joint_pos - FTS->q();//qmes - qdes
    deq = joint_vel - FTS->qdot();//qdotmes - qdotdes
    spaceTransform = posEndEffMes.getRotation().adjoint();//SF->getPosition()=end effector position mesured

    error = posEndEffMes.getTranslation() - TF->getPosition().getTranslation();//SF->getPosition()=end effector position mesured

    Rdes_in_r = posEndEffMes.getRotation().inverse() * TF->getPosition().getRotation();

    Eigen::Vector3d linearVel = Jac*joint_vel;
    errDot = linearVel - Rdes_in_r.adjoint() * TF->getVelocity().getLinearVelocity();
    errorDot = spaceTransform * errDot;

    const Eigen::VectorXd f1 = -accTask->getStiffness() * eq
                               - accTask->getDamping() * deq;
    const Eigen::VectorXd f2 = -accTask2->getStiffness() * error
                               - accTask2->getDamping() * errorDot;


    tau = proj.transpose()*accTask->getJacobian().transpose() * f1
          + proj2.transpose()*Jac.transpose() * f2;//accTask2->getJacobian()=mesured jacobian
    
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

std::pair<Eigen::VectorXd, Eigen::MatrixXd> KukaMingXingControllerRTNET::sortRows(const Eigen::MatrixXd &C, const Eigen::MatrixXd &J){
    int totalTaskDim = J.rows();
    int nDof = J.cols();
    Eigen::MatrixXd Cii_J = MatrixXd::Zero(totalTaskDim, 1+nDof);

    Cii_J.col(0) = C.diagonal();
    Cii_J.block(0,1,totalTaskDim,nDof) = J;

    int rdim = Cii_J.rows();
    Eigen::VectorXd tmp(1+nDof);
    for (int rmin=0; rmin<rdim-1;++rmin)
    {
        for(int i=rdim-1; i>rmin; --i)
        {
            if (Cii_J(i,0)>Cii_J(i-1,0))
            {
                tmp = Cii_J.row(i-1);
                Cii_J.row(i-1) = Cii_J.row(i);
                Cii_J.row(i) = tmp;
            }
        }
    }

    return std::pair<Eigen::VectorXd, Eigen::MatrixXd>(Cii_J.col(0),Cii_J.block(0,1,totalTaskDim,nDof));
}

void KukaMingXingControllerRTNET::computeProjector(const Eigen::MatrixXd &C, const Eigen::MatrixXd &J, Eigen::MatrixXd& projector){
    int totalTaskDim = J.rows();
    int nDof = J.cols();

    Eigen::VectorXd Cs(nDof);
    Eigen::MatrixXd Js(totalTaskDim, nDof);

    std::pair<Eigen::VectorXd, Eigen::MatrixXd> sortedMatrix = sortRows(C,J);
    Cs = sortedMatrix.first;
    Js = sortedMatrix.second;
    orcisir::OrthonormalFamily onfamily(Js, 1e-9);
    onfamily.computeOrthonormalFamily();
    Eigen::MatrixXd onb_Js = onfamily.getOnf();
    Eigen::VectorXi origin = onfamily.getOrigin();

    int k = onfamily.getIndex();
    Eigen::VectorXd Chat(k);

    for (int j=0; j<k; ++j)
    {
        Chat(j)=Cs(origin(j));
    }


    Eigen::MatrixXd alpha = Eigen::MatrixXd(Chat.asDiagonal());
    projector = Eigen::MatrixXd::Identity(nDof,nDof) - onb_Js.transpose()*alpha*onb_Js;
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
void KukaMingXingControllerRTNET::setEEdesTask2(std::vector<double> &eedes){
    posdes_task2 = Eigen::Displacementd(eedes[0], eedes[1], eedes[2]);
    TF->setPosition(posdes_task2);
}

void KukaMingXingControllerRTNET::setStiffnessTask(double &stiffness1, double &stiffness2){
    accTask->setStiffness(stiffness1);
    accTask2->setStiffness(stiffness2);
}

void KukaMingXingControllerRTNET::setDampingTask(double &damping1, double &damping2){
    accTask->setDamping(damping1);
    accTask2->setDamping(damping2);
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
