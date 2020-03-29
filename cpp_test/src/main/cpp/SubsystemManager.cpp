/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "SubsystemManager.h"

SubsystemManager::SubsystemManager(std::vector<Subsystem*> & subsystems) {
    //create the notifiers as special pointers to be deleted when the parent class is deleted
    enabledLoopNoti = std::make_unique<frc::Notifier>([=] { enabledLoop(); });
    disabledLoopNoti = std::make_unique<frc::Notifier>([=] { disabledLoop(); });

    //register all of the subsystems for management
    for(int i = 0; (unsigned)i < subsystems.size(); i++){
        subsystemList.push_back(subsystems.at(i));
    }

    enabledLoopNoti -> SetName("Enabled Loop");
    disabledLoopNoti -> SetName("Disabled Loop");
}

void SubsystemManager::startEnabledLoop(){
    enabledLoopNoti -> StartPeriodic(units::second_t(deltaT));
}

void SubsystemManager::startDisabledLoop(){
    disabledLoopNoti -> StartPeriodic(units::second_t(deltaT));
}

void SubsystemManager::stopEnabledLoop(){
    enabledLoopNoti -> Stop();
}

void SubsystemManager::stopDisabledLoop(){
    disabledLoopNoti -> Stop();
}
