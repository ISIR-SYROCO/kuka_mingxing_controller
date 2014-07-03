// Filename:  kukaMingXingController-rtnetcomponent.cpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description:  

#include "kukaMingXingController-rtnetcomponent.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <fstream>

KukaMingXingControllerRTNET::KukaMingXingControllerRTNET(std::string const& name) : FriRTNetExampleAbstract(name){
    this->addOperation("setQ1des", &KukaMingXingControllerRTNET::setQdesTask1, this, RTT::OwnThread);
    this->addOperation("setParamPriority", &KukaMingXingControllerRTNET::setParamPriority, this, RTT::OwnThread);
    this->addOperation("setStiffness", &KukaMingXingControllerRTNET::setStiffnessTask, this, RTT::OwnThread);
    this->addOperation("setIntegrator", &KukaMingXingControllerRTNET::setIntegratorTask, this, RTT::OwnThread);
    this->addOperation("setDamping", &KukaMingXingControllerRTNET::setDampingTask, this, RTT::OwnThread);
    this->addOperation("setEEdes", &KukaMingXingControllerRTNET::setEEdesTask2, this, RTT::OwnThread);
    this->addOperation("setLemniscate", &KukaMingXingControllerRTNET::setLemniscate, this, RTT::OwnThread);
    this->addOperation("setEldes", &KukaMingXingControllerRTNET::setEldesTask3, this, RTT::OwnThread);
    this->addOperation("getPoseEE", &KukaMingXingControllerRTNET::getPoseEE, this, RTT::OwnThread);
    this->addOperation("getErrorEE", &KukaMingXingControllerRTNET::getErrorEE, this, RTT::OwnThread);
    this->addOperation("getTau", &KukaMingXingControllerRTNET::getTau, this, RTT::OwnThread);
    this->addOperation("getQ", &KukaMingXingControllerRTNET::getQ, this, RTT::OwnThread);
    this->addOperation("useInterp", &KukaMingXingControllerRTNET::useInterpolation, this, RTT::OwnThread);
    this->addOperation("setMode", &KukaMingXingControllerRTNET::setControlMode, this, RTT::OwnThread);//0:posture, 1:ee, 2 GHC
    this->addOperation("useLemniscate", &KukaMingXingControllerRTNET::useLemniscate, this, RTT::OwnThread);
  
    //plot data
    errEE.resize(60000);
    errEl.resize(60000);
    errQ.resize(60000);
    vecT.resize(60000);
    eex.resize(60000);
    eey.resize(60000);
    eez.resize(60000);
    eerefx.resize(60000);
    eerefy.resize(60000);
    eerefz.resize(60000);
    doPlot = true;
  
    tau_max.resize(7);
    tau_max<<200,200,100,100,100,30,30;

    
    eq.resize(LWRDOF);
    deq.resize(LWRDOF);
    eqInt.resize(LWRDOF);
    eqInt.setConstant(0);
    errorInt.setConstant(0);
    errorInt3.setConstant(0);
    ki1 = 0.0;
    ki2 = 0.0;
    ki3 = 0.0;

    model = new kukafixed("kuka");
    ctrl = new orcisir::GHCJTController("myCtrl", *model, solver, true, false);
    counter = 0;
    interpCounter = 0;
    interpCounterEl = 0;
    controlMode = 1;
    initPosSet = false;
    initPosSetEl = false;
    interpolation = false;
    interpolationEl = false;
    lemniscate = false;
    
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
    accTask->setStiffness(9);
    accTask->setDamping(1);
    proj=Eigen::MatrixXd::Zero(LWRDOF,LWRDOF);
    
    lemniCenter<<0.6,-0.1,0.49;
    lemniLenY = 0.8;
    lemniLenZ = 0.6;
    lemniFreq = 1.0;
    SF = new orc::SegmentFrame("frame.SFrame", *model, "kuka.07", Eigen::Displacementd(0.0,0.0,0.06));
    TF = new orc::TargetFrame("frame.TFrame", *model);
    feat2 = new orc::PositionFeature("frame", *SF, orc::XYZ);
    featDes2 = new orc::PositionFeature("frame.Des", *TF, orc::XYZ);
    posdes_task2 = Eigen::Displacementd(0.0,0.49,0.59);//0.3,0.0,0.5
    TF->setPosition(posdes_task2);
    TF->setVelocity(Eigen::Twistd());
    TF->setAcceleration(Eigen::Twistd());

    accTask2 = &(ctrl->createGHCJTTask("accTask2", *feat2, *featDes2));
    
    ctrl->addTask(*accTask2);
    accTask2->activateAsObjective();
    accTask2->setStiffness(20);
    accTask2->setDamping(6);
    proj2=Eigen::MatrixXd::Zero(LWRDOF,LWRDOF);

    SF3 = new orc::SegmentFrame("frame3.SFrame", *model, "kuka.04", Eigen::Displacementd());
    TF3 = new orc::TargetFrame("frame3.TFrame", *model);
    feat3 = new orc::PositionFeature("frame3", *SF3, orc::XYZ);
    featDes3 = new orc::PositionFeature("frame3.Des", *TF3, orc::XYZ);
    posdes_task3 = Eigen::Displacementd(-0.3,-0.2,0.5);
    TF3->setPosition(posdes_task3);
    TF3->setVelocity(Eigen::Twistd());
    TF3->setAcceleration(Eigen::Twistd());

    accTask3 = &(ctrl->createGHCJTTask("accTask3", *feat3, *featDes3));
    
    ctrl->addTask(*accTask3);
    accTask3->activateAsObjective();
    accTask3->setStiffness(20);
    accTask3->setDamping(6);
    proj3=Eigen::MatrixXd::Zero(LWRDOF,LWRDOF);

    joint_position_command.assign(LWRDOF, 0.0);

    tau.resize(LWRDOF);
    tau.setConstant(0);

    ctrl->setActiveTaskVector();

    // SET TASK PRIORITIES, COMPUTE TASK PROJECTORS
    int nt = ctrl->getNbActiveTask();

    param_priority.resize(nt,nt);
    param_priority<<0, 1, 1, 
		    0, 0, 0,
		    0, 1, 0;
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
    std::vector<double> JTrq(LWRDOF);
    RTT::FlowStatus joint_state_fs = iport_msr_joint_pos.read(JState);
    RTT::FlowStatus joint_vel_fs = iport_msr_joint_vel.read(JVel);
    RTT::FlowStatus joint_trq_fs = iport_msr_joint_trq.read(JTrq);

    Eigen::VectorXd joint_pos(LWRDOF);
    Eigen::VectorXd joint_vel(LWRDOF);
    Eigen::VectorXd joint_trq(LWRDOF);
    Eigen::MatrixXd Jac = Eigen::MatrixXd::Zero(3,LWRDOF);
    Eigen::MatrixXd Jac3 = Eigen::MatrixXd::Zero(3,LWRDOF);


    if(joint_state_fs == RTT::NewData){        
        
        
        for(unsigned int i = 0; i < LWRDOF; i++){
            joint_pos[i] = JState[i];
            joint_position_command[i] = JState[i];
        }
        model->setJointPositions(joint_pos);
	if (counter==0)
	    FTS->set_q(joint_pos);
	//std::cout << "Jpos " << joint_pos.transpose() << std::endl;
    }
    

    if(joint_vel_fs == RTT::NewData){
        
        for(unsigned int i = 0; i < LWRDOF; i++){
            joint_vel[i] = JVel[i];
        }
        model->setJointVelocities(joint_vel);
	//std::cout << "joint vel " << joint_vel.transpose() << std::endl;
    }
    

    if(joint_trq_fs == RTT::NewData){        
        
        
        for(unsigned int i = 0; i < LWRDOF; i++){
            joint_trq[i] = JTrq[i];
        }

	
    }


//    RTT::FlowStatus jacobian_fs = jacobianPort.read(J);
//    if(jacobian_fs==RTT::NewData){   
//        Eigen::MatrixXd JacFull(6,7); 
//		JacFull.noalias() = J.data;
//		Jac = JacFull.block(0,0, 3,7);
//    }
    
    Eigen::Displacementd posEndEffMes;
    RTT::FlowStatus cartPos_fs =  iport_cart_pos.read(X);
    if(cartPos_fs==RTT::NewData){
        posEndEffMes.x()=(double)X.position.x;
        posEndEffMes.y()=(double)X.position.y;
        posEndEffMes.z()=(double)X.position.z;
        posEndEffMes.qx()=(double)X.orientation.x;
        posEndEffMes.qy()=(double)X.orientation.y;
        posEndEffMes.qz()=(double)X.orientation.z;
        posEndEffMes.qw()=(double)X.orientation.w;
	if (!initPosSet)
        {
            EExinit = (double)X.position.x;
            EEyinit = (double)X.position.y;
            EEzinit = (double)X.position.z;
            initPosSet = true;
        }
	//std::cout<<"posEndEffMes="<<posEndEffMes.getTranslation().transpose()<<std::endl;

    }
  

    Jac = accTask2->getJacobian();
    Jac3 = accTask3->getJacobian();

    //std::cout<<"Jac3="<<Jac3<<std::endl;
    /*Eigen::MatrixXd JacP = Eigen::MatrixXd::Zero(3,LWRDOF);
    RTT::FlowStatus jacobian_fs = jacobianPort.read(J);
    Eigen::MatrixXd JacFull(6,LWRDOF);
    if(jacobian_fs==RTT::NewData){            
	JacFull.noalias() = J.data;
	JacP = JacFull.block(0,0, 3,LWRDOF);
        Jac = posEndEffMes.getRotation().adjoint().transpose()*JacP;
        

    }*/

    double dt = 0.001;
    double coe = 0.0;
    double pi = 3.1415926;
    double switch_duration = 100.0;
    double oneToZero = 0.0;
    double zeroToOne = 0.0;
    if (controlMode==2)
    {
	if (counter==0)
	{
            param_priority<<0, 0, 0, 
		    	    1, 0, 1,
		    	    1, 0, 0;//pos>el>ee
	    std::cout <<"s1=pos>el>ee"<<std::endl;
	}

        else if (counter==250)
        {
	    getErrorEE();
	    getErrorEl();
	    getErrorQ();

            /*param_priority<<0, 1, 1, 
		    	    0, 0, 0,
		    	    0, 1, 0;//ee>el>pos*/
	    initPosSet = false;
//	    initPosSetEl = false;
//	    interpCounterEl = 0;
	    std::cout <<"s2=ee>el>pos"<<std::endl;
	}
	  else if ((counter > 250) && (counter <= (250+int(switch_duration))))
	    {
 	        coe = counter - 250;
                oneToZero = (cos(coe * pi/switch_duration) + 1.0)/2.0; //1 to 0
	        zeroToOne = 1.0 - oneToZero;
	        param_priority(1,0) = oneToZero;
	        param_priority(1,2) = oneToZero;
  		param_priority(2,0) = oneToZero;
	        param_priority(0,1) = zeroToOne;
	        param_priority(0,2) = zeroToOne;
	        param_priority(2,1) = zeroToOne;
	    }
        else if (counter==900)
	{
	    getErrorEE();
	    getErrorEl();
	    getErrorQ();
            /*param_priority<<0, 1, 1, 
		    	    0, 0, 1,
		    	    0, 0, 0;//el>ee>pos */
            initPosSet = false;
//	    initPosSetEl = false;
//	    interpCounterEl = 0;
	    std::cout <<"s3=el>ee>pos"<<std::endl;
	}
	  else if ((counter > 900) && (counter <= (900+int(switch_duration))))
	    {
 	        coe = counter - 900;
                oneToZero = (cos(coe * pi/switch_duration) + 1.0)/2.0; //1 to 0
	        zeroToOne = 1.0 - oneToZero;

	        param_priority(1,2) = zeroToOne;
	        param_priority(2,1) = oneToZero;
	    }
        else if (counter==1600)
	{
	    getErrorEE();
	    getErrorEl();
	    getErrorQ();
            /*param_priority<<0, 1, 0, 
		    	    0, 0, 0,
		    	    0, 0, 1;//ee>pos*/
            initPosSet = false;
//            interpolation = true;
//	    interpCounter = 0;
//	    initPosSetEl = false;
//	    interpCounterEl = 0;
	    std::cout <<"s4=ee>pos"<<std::endl;
	}
	  else if ((counter > 1600) && (counter <= (1600+int(switch_duration))))
	    {
 	        coe = counter - 1600;
                oneToZero = (cos(coe * pi/switch_duration) + 1.0)/2.0; //1 to 0
	        zeroToOne = 1.0 - oneToZero;
	        param_priority(0,2) = oneToZero;
	        param_priority(1,2) = oneToZero;
	        param_priority(2,2) = zeroToOne;

	    }
        else if (counter==2300)
	{
	    getErrorEE();
	    getErrorEl();
	    getErrorQ();
            param_priority<<0, 1, 1, 
		    	    0, 0, 0,
		    	    0, 1, 0;//ee>el>pos
 //           initPosSet = false;
//	    initPosSetEl = false;
//	    interpCounterEl = 0;
	    std::cout <<"s5=ee>el>pos"<<std::endl;
	}
	  else if ((counter > 2300) && (counter <= (2300+int(switch_duration))))
	    {
 	        coe = counter - 2300;
                oneToZero = (cos(coe * pi/switch_duration) + 1.0)/2.0; //1 to 0
	        zeroToOne = 1.0 - oneToZero;
	        param_priority(0,2) = zeroToOne;
	        param_priority(2,1) = zeroToOne;
	        param_priority(2,2) = oneToZero;

	    }
        else if (counter==3000)
	{
	    getErrorEE();
	    getErrorEl();
	    getErrorQ();
	}
    }  
    if (!initPosSet)
    {
        EExinit = (double)X.position.x;
        EEyinit = (double)X.position.y;
        EEzinit = (double)X.position.z;
        initPosSet = true;
    }
    if (!initPosSetEl)
    {
        Elxinit = SF3->getPosition().getTranslation()[0];
        Elyinit = SF3->getPosition().getTranslation()[1];
        Elzinit = SF3->getPosition().getTranslation()[2];
        initPosSetEl = true;
    }
    if (controlMode==2&&lemniscate)
    {
        
        double time = dt*counter;
        posdes_task2 = Eigen::Displacementd(lemniCenter[0],lemniCenter[1]+lemniLenY*cos(lemniFreq*time),lemniCenter[2]+lemniLenZ*sin(lemniFreq*2.0*time));
        TF->setPosition(posdes_task2);
    } 

    if (interpCounter <= 100 && interpolation)
    {
        double coef2 = 0.01*interpCounter;
        double coef = 1.0 - coef2;
        double tx = posdes_task2.getTranslation()[0];
        double ty = posdes_task2.getTranslation()[1];
        double tz = posdes_task2.getTranslation()[2];
        Eigen::Displacementd posdes = Eigen::Displacementd(coef*EExinit+coef2*tx, coef*EEyinit+coef2*ty, coef*EEzinit+coef2*tz);
    	TF->setPosition(posdes);
    }
    if (interpCounterEl <= 10 && interpolationEl)
    {
        double coefEl2 = 0.1*interpCounterEl;
        double coefEl = 1.0 - coefEl2;
        double txEl = posdes_task3.getTranslation()[0];
        double tyEl = posdes_task3.getTranslation()[1];
        double tzEl = posdes_task3.getTranslation()[2];
        Eigen::Displacementd posdes3 = Eigen::Displacementd(coefEl*Elxinit+coefEl2*txEl, coefEl*Elyinit+coefEl2*tyEl, coefEl*Elzinit+coefEl2*tzEl);
    	TF3->setPosition(posdes3);
    }

    //compute task error
    eq = FTS->q() - joint_pos;
    deq = - joint_vel;
    eqInt += eq*dt;

    Eigen::Displacementd posdes_task2p2 = Eigen::Displacementd(lemniCenter[0],lemniCenter[1]+lemniLenY*cos(lemniFreq*dt*(counter+2)),lemniCenter[2]+lemniLenZ*sin(lemniFreq*2.0*dt*(counter+2)));
    Eigen::Displacementd posdes_task2p1 = Eigen::Displacementd(lemniCenter[0],lemniCenter[1]+lemniLenY*cos(lemniFreq*dt*(counter+1)),lemniCenter[2]+lemniLenZ*sin(lemniFreq*2.0*dt*(counter+1)));
    error = TF->getPosition().getTranslation() - posEndEffMes.getTranslation();
    Eigen::Vector3d linearVel = Jac*joint_vel;
    errorDot = -linearVel;
    errorInt += error*dt;

    error3 = TF3->getPosition().getTranslation() - SF3->getPosition().getTranslation();
    Eigen::Vector3d linearVel3 = Jac3*joint_vel;
    errorDot3 = -linearVel3;
    errorInt3 += error3*dt;

    const Eigen::VectorXd f1 = accTask->getStiffness() * eq
                               + accTask->getDamping() * deq
			       + ki1*eqInt;
    const Eigen::VectorXd f2 = accTask2->getStiffness() * error
                               + accTask2->getDamping() * errorDot
			       + ki2 *errorInt;
    if (lemniscate)
	f2 += (posdes_task2p2.getTranslation()  + TF->getPosition().getTranslation() - 2.*posdes_task2p1.getTranslation())/dt/dt;
			       
    const Eigen::VectorXd f3 = accTask3->getStiffness() * error3
                               + accTask3->getDamping() * errorDot3
			       + ki3 *errorInt3;
    //std::cout << "f3 " << f3.transpose() << std::endl;
    //std::cout << "eq " << eq.transpose() << std::endl;
    //std::cout << "deq " << deq.transpose() << std::endl;
    //std::cout << "error " << error.transpose() << std::endl;
    //std::cout << "errorDot " << errorDot.transpose() << std::endl;
    //if (counter>=1500&&counter<2500)
    //    std::cout << "el pos " << SF3->getPosition().getTranslation().transpose() << std::endl;

    //Set task projector
    ctrl->setTaskProjectors(param_priority);

    // UpdateAugmentedJacobian
    Eigen::MatrixXd augmentedJacobian = MatrixXd::Zero(feat->getDimension()+feat2->getDimension()+feat3->getDimension(), LWRDOF);
    int rowindex = 0;
    int taskdim;

    taskdim = feat->getDimension();
    augmentedJacobian.block(rowindex,0, taskdim, LWRDOF)=accTask->getJacobian();
    rowindex += taskdim;

    taskdim = feat2->getDimension();
    augmentedJacobian.block(rowindex,0, taskdim, LWRDOF)=Jac;
    rowindex += taskdim;
 
    taskdim = feat3->getDimension();
    augmentedJacobian.block(rowindex,0, taskdim, LWRDOF)=Jac3;
   
    //Update projector
    computeProjector(accTask->getPriority(), augmentedJacobian, proj);
    accTask->setProjector(proj);

    computeProjector(accTask2->getPriority(), augmentedJacobian, proj2);
    accTask2->setProjector(proj2);
 
    computeProjector(accTask3->getPriority(), augmentedJacobian, proj3);
    accTask3->setProjector(proj3);
   
    //Compute tau
    ctrl->computeOutput(tau); 
    //std::cout<<"posModel="<<SF->getPosition().getTranslation().transpose()<<std::endl;
    //std::cout<<"JacModel="<<accTask2->getJacobian()<<std::endl;

    if (controlMode==0) 
    {
        tau = f1;
//	std::cout << "errorq="<<eq.transpose() << std::endl;
    }
    else if (controlMode==1)
    {
	tau = Jac.transpose() * f2;
//	std::cout << "errorEE="<<(posdes_task2.getTranslation() - posEndEffMes.getTranslation()).transpose() << std::endl;
    }
    else if (controlMode==2)
    {
            param_priority<<0, 1, 1, 
		    	    0, 0, 0,
		    	    0, 1, 0;//ee>el>pos
        tau = proj.transpose() * f1 + proj2.transpose()*Jac.transpose() * f2 + proj3.transpose()*Jac3.transpose() * f3;
        //tau = Jac3.transpose() * f3;
    }
          
//    for(int i = 0; i < tau.size(); ++i)
//    {
//      if(tau(i) > tau_max(i)) tau(i) = tau_max(i);
//      else if(tau(i) < -tau_max(i)) tau(i) = -tau_max(i);
//    }

      

    //std::cout << "tau="<<tau.transpose() << std::endl;

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

    //plot data
    if (doPlot)
    {
	std::ofstream myfile;
	myfile.open ("kukadata.txt");
	std::ofstream myfile2;
	myfile2.open ("kuka_task_error.txt");	
	int start,end,endindex;
	start = 2000;
	end = 50000;
	endindex = end-start-1;
	if (counter>=start &&counter <= end)
    	{
	    eex[counter-start] = posEndEffMes.getTranslation()[0];
	    eey[counter-start] = posEndEffMes.getTranslation()[1];
	    eez[counter-start] = posEndEffMes.getTranslation()[2];
	    eerefx[counter-start] = TF->getPosition().getTranslation()[0];
	    eerefy[counter-start] = TF->getPosition().getTranslation()[1];
	    eerefz[counter-start] = TF->getPosition().getTranslation()[2];
	    if (counter == end)
	    {		
		for (unsigned int i=0;i<endindex;++i)
		    myfile <<eex[i]<<" ";
		myfile <<eex[endindex]<<"\n";

		for (unsigned int i=0;i<endindex;++i)
		    myfile <<eey[i]<<" ";
		myfile <<eey[endindex]<<"\n";

		for (unsigned int i=0;i<endindex;++i)
		    myfile <<eez[i]<<" ";
		myfile <<eez[endindex]<<"\n";

		for (unsigned int i=0;i<endindex;++i)
		    myfile <<eerefx[i]<<" ";
		myfile <<eerefx[endindex]<<"\n";

		for (unsigned int i=0;i<endindex;++i)
		    myfile <<eerefy[i]<<" ";
		myfile <<eerefy[endindex]<<"\n";

		for (unsigned int i=0;i<endindex;++i)
		    myfile <<eerefz[i]<<" ";
		myfile <<eerefz[endindex]<<"\n";
	    }
	    myfile.close();
	}
	if (counter <= end)
    	{
            errEE[counter] = error.norm();
            errEl[counter] = error3.norm();
            errQ[counter] = eq.norm();
            vecT[counter] = counter*dt;
	    if (counter == end)
	    {
		endindex = end-1;
		for (unsigned int i=0;i<endindex;++i)
		    myfile2 <<errEE[i]<<" ";
		myfile2 <<errEE[endindex]<<"\n";	  


		for (unsigned int i=0;i<endindex;++i)
		    myfile2 <<errEl[i]<<" ";
		myfile2 <<errEl[endindex]<<"\n";	


		for (unsigned int i=0;i<endindex;++i)
		    myfile2 <<errQ[i]<<" ";
		myfile2 <<errQ[endindex]<<"\n";	


		for (unsigned int i=0;i<endindex;++i)
		    myfile2 <<vecT[i]<<" ";
		myfile2 <<vecT[endindex]<<"\n";	      
	    }
	    myfile2.close();
	}
		/*std::cout<<"eex=[";
		for (unsigned int i=0;i<1499;++i)
		    std::cout<<eex[i]<<",";
		std::cout<<eex[1499]<<"]"<<std::endl<<std::endl;

		std::cout<<"eey=[";
		for (unsigned int i=0;i<1499;++i)
	    	    std::cout<<eey[i]<<",";
		std::cout<<eey[1499]<<"]"<<std::endl<<std::endl;

		std::cout<<"eez=[";
		for (unsigned int i=0;i<1499;++i)
		    std::cout<<eez[i]<<",";
		std::cout<<eez[1499]<<"]"<<std::endl<<std::endl;

		std::cout<<"eerefx=[";
		for (unsigned int i=0;i<1499;++i)
 		    std::cout<<eerefx[i]<<",";
		std::cout<<eerefx[1499]<<"]"<<std::endl<<std::endl;

		std::cout<<"eerefy=[";
		for (unsigned int i=0;i<1499;++i)
  		    std::cout<<eerefy[i]<<",";
		std::cout<<eerefy[1499]<<"]"<<std::endl<<std::endl;

		std::cout<<"eerefz=[";
		for (unsigned int i=0;i<1499;++i)
		    std::cout<<eerefz[i]<<",";
		std::cout<<eerefz[1499]<<"]"<<std::endl<<std::endl;*/

	    /*if (counter == 1000)
	    {
	    std::cout<<"errEE=[";
	    for (unsigned int i=0;i<499;++i)
	    	std::cout<<errEE[i]<<",";
	    std::cout<<errEE[499]<<"]"<<std::endl<<std::endl;

	    std::cout<<"errEl=[";
	    for (unsigned int i=0;i<499;++i)
	    	std::cout<<errEl[i]<<",";
	    std::cout<<errEl[499]<<"]"<<std::endl<<std::endl;

	    std::cout<<"errQ=[";
	    for (unsigned int i=0;i<499;++i)
	    	std::cout<<errQ[i]<<",";
	    std::cout<<errQ[499]<<"]"<<std::endl<<std::endl;

	    std::cout<<"time=[";
	    for (unsigned int i=0;i<499;++i)
	    	std::cout<<vecT[i]<<",";
	    std::cout<<vecT[499]<<"]"<<std::endl;

	    }*/


    }


    if (counter <100000)
        counter += 1;
    if (interpCounter <100&&interpolation)
        interpCounter += 1;
    if (interpCounterEl <10&&interpolationEl)
        interpCounterEl += 1;
}

std::pair<Eigen::VectorXd, Eigen::MatrixXd> KukaMingXingControllerRTNET::sortRows(const Eigen::MatrixXd &C, const Eigen::MatrixXd &J){
    int totalTaskDim = J.rows();
    int nDof = J.cols();
    Eigen::MatrixXd Cii_J = MatrixXd::Zero(totalTaskDim, 1+nDof);

    Cii_J.col(0) = C.diagonal();
    Cii_J.block(0,1,totalTaskDim,nDof) = J;

    int rdim = Cii_J.rows();
    Eigen::VectorXd tmp(1+nDof);
    for (unsigned int rmin=0; rmin<rdim-1;++rmin)
    {
        for(unsigned int i=rdim-1; i>rmin; --i)
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

    for (unsigned int j=0; j<k; ++j)
    {
        Chat(j)=Cs(origin(j));
    }


    Eigen::MatrixXd alpha = Eigen::MatrixXd(Chat.asDiagonal());
    projector = Eigen::MatrixXd::Identity(nDof,nDof) - onb_Js.transpose()*alpha*onb_Js;
}

void KukaMingXingControllerRTNET::setParamPriority(std::vector<double> &param){
    unsigned int nt = ctrl->getNbActiveTask();
    if(param.size() != nt*nt){
        std::cout << "Incorrect size" << std::endl;
        return;
    }    
    else{
        //Row Major
        for(unsigned int i=0; i<nt; ++i){
            for(unsigned int j=0; j<nt; ++j){
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
    initPosSet = false;
    interpCounter = 0;
}

void KukaMingXingControllerRTNET::setLemniscate(std::vector<double> &center, double lenY, double lenZ, double freq){
	lemniCenter<<center[0],center[1],center[2];
	lemniLenY = lenY;
	lemniLenZ = lenZ;
	lemniFreq = freq;
}

void KukaMingXingControllerRTNET::setEldesTask3(std::vector<double> &eldes){
    posdes_task3 = Eigen::Displacementd(eldes[0], eldes[1], eldes[2]);
    TF3->setPosition(posdes_task3);
    initPosSetEl = false;
    interpCounterEl = 0;
}

void KukaMingXingControllerRTNET::setStiffnessTask(double stiffness1, double stiffness2, double stiffness3){
    accTask->setStiffness(stiffness1);
    accTask2->setStiffness(stiffness2);
    accTask3->setStiffness(stiffness3);
}

void KukaMingXingControllerRTNET::setDampingTask(double damping1, double damping2, double damping3){
    accTask->setDamping(damping1);
    accTask2->setDamping(damping2);
    accTask3->setDamping(damping3);
}

void KukaMingXingControllerRTNET::setIntegratorTask(double integrator1, double integrator2, double integrator3){
    ki1=integrator1;
    ki2=integrator2;
    ki3=integrator3;
}

void KukaMingXingControllerRTNET::getPoseEE(){
	std::cout << X.position.x << " " 
                  << X.position.y << " "
                  << X.position.z << std::endl;
}

void KukaMingXingControllerRTNET::getErrorEE(){
	std::cout << "errorEE="<<error.transpose() << std::endl;
}

void KukaMingXingControllerRTNET::getErrorEl(){
	std::cout << "errorEl="<< error3.transpose() << std::endl;
}

void KukaMingXingControllerRTNET::getErrorQ(){
	std::cout << "errorQ="<< eq.transpose() << std::endl;
}

void KukaMingXingControllerRTNET::getTau(){
	std::cout << tau.transpose() << std::endl;
}

void KukaMingXingControllerRTNET::getQ(){
	for(unsigned int i=0; i<LWRDOF; ++i){
		std::cout << joint_position_command[i] << " ";
	}
	std::cout << std::endl;
}

void KukaMingXingControllerRTNET::useInterpolation(bool interp){
	interpolation = interp;
}

void KukaMingXingControllerRTNET::setControlMode(int mode){
	controlMode = mode;
}

void KukaMingXingControllerRTNET::useLemniscate(bool lemni){
	lemniscate = lemni;
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
