// Filename:  kukaModelFromFri-rtnetcomponent.hpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description: Orocos component using RTNET to compute the kuka model
//              from fri data

#ifndef KUKA_MODEL_FROM_FRI_RTNET_COMPONENT_HPP
#define KUKA_MODEL_FROM_FRI_RTNET_COMPONENT_HPP

#include <friRTNetExampleAbstract.hpp>

#include <orcisir/GHCJTController.h>
#include <orcisir/Solvers/OneLevelSolver.h>
#include "kukafixed.h"
#include <kukakdl/kukakdl.hpp>
#include <kukakdl/orckukakdl.hpp>

#include <orc/control/Feature.h>
#include <orc/control/FullState.h>
#include <orc/control/ControlFrame.h>
#include <orc/control/ControlEnum.h>

#include <orcisir/OrthonormalFamily.h>

#include <Eigen/Dense>
#include <fstream>

class KukaMingXingControllerRTNET : public FriRTNetExampleAbstract{
    public:
        KukaMingXingControllerRTNET(std::string const& name);
		//kukafixed* model;
		OrcKukaKDL* model;
        KukaKDL modelKDL;
        orcisir::OneLevelSolverWithQuadProg solver;
        orcisir::GHCJTController* ctrl;
	int counter;
        int controlMode;
        //Task1
        orc::FullModelState* FMS;
        orc::FullTargetState* FTS;
        orc::FullStateFeature* feat;
        orc::FullStateFeature* featDes;
        orcisir::GHCJTTask* accTask;
        
        Eigen::VectorXd qdes_task1;
        
        Eigen::MatrixXd proj;
        
        //Task2
        orc::SegmentFrame* SF;
        orc::TargetFrame* TF;
        orc::PositionFeature* feat2;
        orc::PositionFeature* featDes2;
        orcisir::GHCJTTask* accTask2;
        
        Eigen::Displacementd posdes_task2;
        Eigen::Twistd veldes_task2;
        int interpCounterEE;
        double EExinit;
	double EEyinit;
	double EEzinit;
        bool initPosSet;
        bool interpolationEE;
        bool lemniscate;
        
        Eigen::MatrixXd proj2;
            
        //Task3
        orc::SegmentFrame* SF3;
        orc::TargetFrame* TF3;
        orc::PositionFeature* feat3;
        orc::PositionFeature* featDes3;
        orcisir::GHCJTTask* accTask3;
        
        Eigen::Displacementd posdes_task3;
        Eigen::Twistd veldes_task3;
        int interpCounterEl;
        double Elxinit;
	double Elyinit;
	double Elzinit;
        bool initPosSetEl;
        bool interpolationEl;
	Eigen::Vector3d lemniCenter;
	double lemniLenY, lemniLenZ, lemniFreq;

        
        Eigen::MatrixXd proj3;

        Eigen::Vector3d error,errDot,errorDot,errorInt;
        Eigen::Vector3d error3,errDot3,errorDot3,errorInt3;
        Eigen::Displacementd::Rotation3D Rdes_in_r;
        Eigen::Matrix3d spaceTransform;
        Eigen::VectorXd eq,deq,eqInt;
	double ki1,ki2,ki3;
        
        Eigen::VectorXd tau;
		Eigen::VectorXd tauJT;
        Eigen::VectorXd tau_max;
        std::vector<double> joint_position_command;

        Eigen::MatrixXd param_priority;
        
		Eigen::Displacementd posEndEffMes;
		Eigen::Displacementd posElbowMes;
        KDL::Jacobian J;
      	geometry_msgs::Pose X;

	//plot data
	Eigen::VectorXd errEE,errEl,errQ,vecT;
	Eigen::VectorXd eex,eey,eez,eerefx,eerefy,eerefz;
	bool doPlot;
	std::ofstream myfile;
	//myfile.open ("kukadata.txt", std::ofstream::app);
	std::ofstream myfile2;
	std::ofstream torqueCtrl;
	std::ofstream torqueJT;
	//myfile2.open ("kuka_task_error.txt", std::ofstream::app);	

        void updateHook();
        
        void computeProjector(const Eigen::MatrixXd &C, const Eigen::MatrixXd &J, Eigen::MatrixXd& projector);
        std::pair<Eigen::VectorXd, Eigen::MatrixXd> sortRows(const Eigen::MatrixXd &C, const Eigen::MatrixXd &J);
	
	void getPoseEE();
	void getPoseEl();
	void getErrorEE();
	void getErrorEl();
	void getErrorQ();
	void getTau();
	void getQ();
        
        void setParamPriority(std::vector<double> &param);
        void setQdesTask1(std::vector<double> &qdes);
	void setLemniscate(std::vector<double> &center, double lenY, double lenZ, double freq);
        void setEEdesTask2(std::vector<double> &eedes);
        void setEldesTask3(std::vector<double> &eldes);
	void setStiffnessTask(double stiffness1, double stiffness2, double stiffness3);
	void setDampingTask(double damping1, double damping2, double damping3);
  	void setIntegratorTask(double integrator1, double integrator2, double integrator3);
	void setControlMode(int mode);
        void useInterpolationEE(bool interp);
	void useInterpolationEl(bool interp);
	void useLemniscate(bool lemni);
};

#endif
