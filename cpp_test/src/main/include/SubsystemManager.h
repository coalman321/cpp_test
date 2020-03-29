/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <frc/Notifier.h>
#include <units/units.h>
#include <vector>
#include <memory>
#include <subsystems/Subsystem.h>

class SubsystemManager {
  std::vector<Subsystem*> subsystemList;
  std::unique_ptr<frc::Notifier> enabledLoopNoti, disabledLoopNoti;
  double deltaT = 0.010; // 100 hz update rate

  void enabledLoop(){
    for(int i = 0; (unsigned)i < subsystemList.size(); i++){
      subsystemList.at(i) -> readPeriodicInputs();
    }
    for(int i = 0; (unsigned)i < subsystemList.size(); i++){
      subsystemList.at(i) -> onLoop();
    }
    for(int i = 0; (unsigned)i < subsystemList.size(); i++){
      subsystemList.at(i) -> writePeriodicOutputs();
    }
  }

  void disabledLoop(){
      for(int i = 0; (unsigned)i < subsystemList.size(); i++){
      subsystemList.at(i) -> readPeriodicInputs();
    }
    for(int i = 0; (unsigned)i < subsystemList.size(); i++){
      subsystemList.at(i) -> writePeriodicOutputs();
    }
  }

 public:
  SubsystemManager(std::vector<Subsystem*> & subsystems);

  void startEnabledLoop();
  void stopEnabledLoop();

  void startDisabledLoop();
  void stopDisabledLoop();

};
