//
// Created by arclab on 1/7/20.
//

#include <sawIntuitiveResearchKit/mtsTeleOperationPSMNetInterface.h>
#include <stdio.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsTeleOperationPSMNetInterface, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

mtsTeleOperationPSMNetInterface::mtsTeleOperationPSMNetInterface(const std::string & componentName, const double periodInSeconds):
        mtsTaskPeriodic(componentName, periodInSeconds)
{
    Init();
}

mtsTeleOperationPSMNetInterface::mtsTeleOperationPSMNetInterface(const mtsTaskPeriodicConstructorArg & arg):
        mtsTaskPeriodic(arg)
{
    Init();
}

mtsTeleOperationPSMNetInterface::~mtsTeleOperationPSMNetInterface(){
}

void mtsTeleOperationPSMNetInterface::Configure(std::string const & filename){
}

void mtsTeleOperationPSMNetInterface::Startup(void){
}


void mtsTeleOperationPSMNetInterface::Init(void){

    StateDesired_by_Teleop = "";

    //Values that are received from PSM
    this->StateTable.AddData(CartesianCurrent,  "CartesianPosition");
    this->StateTable.AddData(JawCurrent ,  "JawCurrent");

    this->StateTable.AddData(CartesianDesired,  "CartesianDesired");
    this->StateTable.AddData(JawDesired ,  "JawDesired");

    JawCurrent.Position().resize(1);
    JawDesired.Goal().resize(1);

    //Set up the interfaces required by tele-op PSM
    interfaceProvided = AddInterfaceProvided("TeleOp");
    if(interfaceProvided){
        interfaceProvided->AddCommandReadState(this->StateTable, CartesianCurrent,  "GetPositionCartesian");
        interfaceProvided->AddCommandReadState(this->StateTable, JawCurrent, "GetStateJaw");

        interfaceProvided->AddCommandRead(&mtsTeleOperationPSMNetInterface::GetCurrentState, this, "GetCurrentState");
        interfaceProvided->AddCommandRead(&mtsTeleOperationPSMNetInterface::GetDesiredState, this, "GetDesiredState");

        interfaceProvided->AddCommandWriteState(this->StateTable, CartesianDesired, "SetPositionCartesian");
        interfaceProvided->AddCommandWriteState(this->StateTable, JawDesired, "SetPositionJaw");

        interfaceProvided->AddCommandWrite(&mtsTeleOperationPSMNetInterface::SetDesiredState_teleop, this, "SetDesiredState");
        interfaceProvided->AddCommandVoid(&mtsTeleOperationPSMNetInterface::Freeze, this, "Freeze");

        interfaceProvided->AddMessageEvents();
    }

    interfaceProvided_ROS = AddInterfaceProvided("ROS");
    if (interfaceProvided_ROS) {

        //Exposed states to ROS interface that should be written to.
        interfaceProvided_ROS->AddCommandWrite(&mtsTeleOperationPSMNetInterface::ReadCurrentPositionCartesian,
                                               this,"ReadCurrentPositionCartesian");
        interfaceProvided_ROS->AddCommandWrite(&mtsTeleOperationPSMNetInterface::ReadCurrentPositionJaw,
                                               this, "ReadPositionJaw");
        interfaceProvided_ROS->AddCommandWrite(&mtsTeleOperationPSMNetInterface::ReadCurrentState,
                                               this,"ReadCurrentState");
        interfaceProvided_ROS->AddCommandWrite(&mtsTeleOperationPSMNetInterface::ReadDesiredState,
                                               this, "ReadDesiredState");

        //Exposes states that should be read from
        interfaceProvided_ROS->AddCommandRead(&mtsTeleOperationPSMNetInterface::SetCurrentPositionCartesian,
                                              this, "SetPositionCartesian");
        interfaceProvided_ROS->AddCommandRead(&mtsTeleOperationPSMNetInterface::SetCurrentPositionJaw,
                                              this, "SetPositionJaw");
//        interfaceProvided_ROS->AddCommandVoid("Freeze", mPSM.Freeze);
        interfaceProvided_ROS->AddCommandRead(&mtsTeleOperationPSMNetInterface::SetDesiredState,
                                              this, "SetDesiredState");
    }
}

void mtsTeleOperationPSMNetInterface::Run(void){
    ProcessQueuedEvents();
    ProcessQueuedCommands();
}

void mtsTeleOperationPSMNetInterface::Cleanup(void){}


void mtsTeleOperationPSMNetInterface::SetDesiredState_teleop(const std::string & state){
    StateDesired_by_Teleop = state;
}

void mtsTeleOperationPSMNetInterface::Freeze(void){
}


void mtsTeleOperationPSMNetInterface::GetCurrentState(std::string &state) const{
    state = StateCurrent;
}

void mtsTeleOperationPSMNetInterface::GetDesiredState(std::string &state) const{
    state = StateDesired;
}

void mtsTeleOperationPSMNetInterface::ReadCurrentPositionCartesian(const prmPositionCartesianSet &newCartesianSet){
    CartesianCurrent.Position() = newCartesianSet.Goal();
    //Hack for now. This needs to be solved in dvrk_add_topics_functions from dvrk-ros
    //Will be solved by adding other info like coordinate frame.
    CartesianCurrent.Valid() = true;
}

void mtsTeleOperationPSMNetInterface::ReadCurrentPositionJaw(const prmPositionJointSet &newJaw){
    if(newJaw.Goal().size() == 1 && newJaw.Goal().size() == 1) {
        JawCurrent.Position()[0] = newJaw.Goal()[0];
    }
}

void mtsTeleOperationPSMNetInterface::ReadCurrentState(const std::string &state){
    StateCurrent = state;
}

void mtsTeleOperationPSMNetInterface::ReadDesiredState(const std::string &state){
    StateDesired = state;
}

void mtsTeleOperationPSMNetInterface::SetCurrentPositionCartesian(prmPositionCartesianGet &newCommandToBeSent) const{
    newCommandToBeSent.Position() = CartesianDesired.Goal();
}

void mtsTeleOperationPSMNetInterface::SetCurrentPositionJaw(prmStateJoint &newJawToBeSent) const{
    //Another hack, needs to be solved in dvrk_add_topics_functions from dvrk-ros
    newJawToBeSent.Valid() = true;
    newJawToBeSent.Name().resize(1);
    newJawToBeSent.Position().resize(1);
    newJawToBeSent.Velocity().resize(1);
    newJawToBeSent.Effort().resize(1);

    if(JawDesired.Goal().size() == 1 && newJawToBeSent.Position().size() == 1){
        newJawToBeSent.Position()[0] = JawDesired.Goal()[0];
    }
}

void mtsTeleOperationPSMNetInterface::SetDesiredState(std::string &state) const{
    //If no new command has been given by teleop, then send current command
    if(StateDesired_by_Teleop == ""){
        state = StateCurrent;
    }
    else{
        state = StateDesired_by_Teleop;
    }
}

