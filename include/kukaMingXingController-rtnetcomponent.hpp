// Filename:  kukaModelFromFri-rtnetcomponent.hpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak (hak@isir.upmc.fr) 
// Description: Orocos component using RTNET to compute the kuka model
//              from fri data

#ifndef KUKA_MODEL_FROM_FRI_RTNET_COMPONENT_HPP
#define KUKA_MODEL_FROM_FRI_RTNET_COMPONENT_HPP

#include <friRTNetExampleAbstract.hpp>
#include "kukafixed.h"
#include <orcisir/GHCJTController.h>
#include <orcisir/Solvers/OneLevelSolver.h>

#include <orc/control/Feature.h>
#include <orc/control/FullState.h>
#include <orc/control/ControlFrame.h>
#include <orc/control/ControlEnum.h>

#include <orcisir/OrthonormalFamily.h>

#include <Eigen/Dense>

class KukaMingXingControllerRTNET : public FriRTNetExampleAbstract{
    public:
        KukaMingXingControllerRTNET(std::string const& name);
        kukafixed* model;
        orcisir::OneLevelSolverWithQuadProg solver;
        orcisir::GHCJTController* ctrl;
        //Task1
        orc::FullModelState* FMS;
        orc::FullTargetState* FTS;
        orc::FullStateFeature* feat;
        orc::FullStateFeature* featDes;
        orcisir::GHCJTTask* accTask;
        
        Eigen::VectorXd qdes_task1;
        
        //Task2
        orc::SegmentFrame* SF;
        orc::TargetFrame* TF;
        orc::PositionFeature* feat2;
        orc::PositionFeature* featDes2;
        orcisir::GHCJTTask* accTask2;
        
        Eigen::Displacementd posdes_task2;
        Eigen::Twistd veldes_task2;
        
            
        Eigen::Vector3d error,errDot,errorDot;
        Eigen::Displacementd::Rotation3D Rdes_in_r;
        Eigen::Matrix3d spaceTransform;
        Eigen::VectorXd eq,deq;
        
        Eigen::VectorXd tau;
        std::vector<double> joint_position_command;

        Eigen::MatrixXd param_priority;
        
        KDL::Jacobian J;
      	geometry_msgs::Pose X;

        void updateHook();
        
        void computeProjector(const Eigen::MatrixXd &C, const Eigen::MatrixXd &J, Eigen::MatrixXd& projector);
        std::pair<Eigen::VectorXd, Eigen::MatrixXd> sortRows(const Eigen::MatrixXd &C, const Eigen::MatrixXd &J);
        
        void setParamPriority(std::vector<double> &param);
        void setQdesTask1(std::vector<double> &qdes);
        void setEEdesTask2(std::vector<double> &eedes);
	void setStiffnessTask(double &stiffness1, double &stiffness2);
	void setDampingTask(double &damping1, double &damping2);
};

#endif
