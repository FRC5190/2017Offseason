#pragma once

#include <frc/controller/StateSpaceControllerCoeffs.h>
#include <frc/controller/StateSpaceLoop.h>
#include <frc/controller/StateSpaceObserverCoeffs.h>
#include <frc/controller/StateSpacePlantCoeffs.h>

frc::StateSpacePlantCoeffs<2, 1, 1> MakeFlywheelUErrorPlantCoeffs();
frc::StateSpaceControllerCoeffs<2, 1, 1> MakeFlywheelUErrorControllerCoeffs();
frc::StateSpaceObserverCoeffs<2, 1, 1> MakeFlywheelUErrorObserverCoeffs();
frc::StateSpaceLoop<2, 1, 1> MakeFlywheelUErrorLoop();
