//
// Created by arclab on 1/7/20.
//

#ifndef SAWINTUITIVERESEARCHKIT_MTSTELEOPERATIONPSMNETINTERFACE_H
#define SAWINTUITIVERESEARCHKIT_MTSTELEOPERATIONPSMNETINTERFACE_H

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmPositionJointSet.h>

#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

// always include last
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

class CISST_EXPORT mtsTeleOperationPSMNetInterface: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

    public:
        mtsTeleOperationPSMNetInterface(const std::string &componentName, const double periodInSeconds);
        mtsTeleOperationPSMNetInterface(const mtsTaskPeriodicConstructorArg &arg);

    ~mtsTeleOperationPSMNetInterface();

    void Configure(const std::string &filename = "");

    void Startup(void);

    void Run(void);

    void Cleanup(void);

protected:

    void Init(void);

    mtsInterfaceProvided * interfaceProvided;

    //Values being set by tele-op
    prmPositionCartesianSet CartesianDesired;
    prmPositionJointSet JawDesired;
    std::string StateDesired_by_Teleop;

    void SetDesiredState_teleop(const std::string & state);
    void ReadCurrentPositionJaw(const prmPositionJointSet &newJaw);

    void Freeze(void);

    void GetCurrentState(std::string & state) const;
    void GetDesiredState(std::string & state) const;


    mtsInterfaceProvided * interfaceProvided_ROS;

    //Actual values being updated by PSM via ROS interface
    std::string StateCurrent;
    std::string StateDesired;
    prmPositionCartesianGet CartesianCurrent;
    prmStateJoint JawCurrent;

    //These functions will be interfaced via ROS and are reading values from PSM
    void ReadCurrentPositionCartesian(const prmPositionCartesianSet &newCartesianSet);
    void ReadCurrentState(const std::string &state);
    void ReadDesiredState(const std::string &state);

    //These functions will be interfaced via ROS and are setting values for PSM
    void SetDesiredState(std::string &state) const;
    void SetCurrentPositionCartesian(prmPositionCartesianGet &newCommandToBeSent) const;
    void SetCurrentPositionJaw(prmStateJoint &newJawToBeSent) const;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsTeleOperationPSMNetInterface);

#endif //SAWINTUITIVERESEARCHKIT_MTSTELEOPERATIONPSMNETINTERFACE_H
