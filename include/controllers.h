#pragma once

#include <tuple>
#include "matrix_math.h"

class PIDController {

private:
    std::tuple<float, float> rollPitchController(float altitudeOutput, float lateralPositionOutputX, float lateralPositionOutputY, Matrix3X3 rotationMatrix);
    float yawController(float yawTarget, float yawActual);
    std::tuple<float, float, float> bodyRateController(float rollPitchOutputP, float rollPitchOutputQ, float yawOutput, float pActual, float qActual, float rActual);

public:
    PIDController(float kpAltitude, float kdAltitude,
        float kpLateralPositionX, float kdLateralPositionX,
        float kpLateralPositionY, float kdLateralPositionY,
        float kpRollPitch,
        float kpYaw,
        float kpBodyRateP, float kpBodyRateQ, float kpBodyRateR);

    float kpAltitude;
    float kdAltitude;
    float kpLateralPositionX;
    float kdLateralPositionX;
    float kpLateralPositionY;
    float kdLateralPositionY;
    float kpRollPitch;
    float kpYaw;
    float kpBodyRateP;
    float kpBodyRateQ;
    float kpBodyRateR;

    float altitudeController(float zTarget, float zDotTarget, float zActual, float zDotActual, float feedForwardTerm);
    std::tuple<float, float> lateralPositionController(float xTarget, float yTarget, float xDotTarget, float yDotTarget, float xActual, float yActual, float xDotActual, float yDotActual);
    std::tuple<float, float, float> attitudeController(float lateralPositionOutputX, float lateralPositionOutputY, float altitudeOutput, float yawActual, Matrix3X3 rotationMatrix, float pActual, float qActual, float rActual);

    std::tuple<float, float, float> resolveAngularVelocities(float rollPitchOutputP, float rollPitchOutputQ, float yawOutput, Matrix3X3 rotationMatrix);
};
