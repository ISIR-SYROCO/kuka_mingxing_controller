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

#include <Eigen/Dense>

class KukaMingXingControllerRTNET : public FriRTNetExampleAbstract{
    public:
        KukaMingXingControllerRTNET(std::string const& name);
        kukafixed* model;
        orcisir::OneLevelSolverWithQuadProg solver;
        orcisir::GHCJTController* ctrl;
        orc::FullModelState* FMS;
        orc::FullTargetState* FTS;
        orc::FullStateFeature* feat;
        orc::FullStateFeature* featDes;
        orcisir::GHCJTTask* accTask;

        Eigen::VectorXd qdes_task1;
        Eigen::VectorXd tau;
        std::vector<double> joint_position_command;

        Eigen::MatrixXd param_priority;

        void updateHook();
        void setParamPriority(std::vector<double> &param);
        void setQdesTask1(std::vector<double> &qdes);
};

#endif
