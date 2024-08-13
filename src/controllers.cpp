#include "../include/matrix_math.h"
#include "../include/controllers.h"
#include <Eigen/Dense>

PIDController::PIDController(float kpAltitude, float kdAltitude,
    float kpLateralPositionX, float kdLateralPositionX,
    float kpLateralPositionY, float kdLateralPositionY,
    float kpRollPitch,
    float kpYaw,
    float kpBodyRateP, float kpBodyRateQ, float kpBodyRateR)
    : kpAltitude(kpAltitude),
    kdAltitude(kdAltitude),
    kpLateralPositionX(kpLateralPositionX),
    kdLateralPositionX(kdLateralPositionX),
    kpLateralPositionY(kpLateralPositionY),
    kdLateralPositionY(kdLateralPositionY),
    kpRollPitch(kpRollPitch),
    kpYaw(kpYaw),
    kpBodyRateP(kpBodyRateP),
    kpBodyRateQ(kpBodyRateQ),
    kpBodyRateR(kpBodyRateR)
{}

std::tuple<float, float> PIDController::lateralPositionController(float xTarget, float yTarget, float xDotTarget, float yDotTarget, float xActual, float yActual, float xDotActual, float yDotActual) {

    float xError = xTarget - xActual;
	float yError = yTarget - yActual;

	float xDotError = xDotTarget - xDotActual;
	float yDotError = yDotTarget - yDotActual;

	float lateralPositionOutputX = kpLateralPositionX * xError + kdLateralPositionX * xDotError;
	float lateralPositionOutputY = kpLateralPositionY * yError + kdLateralPositionY * yDotError;

	return std::make_tuple(lateralPositionOutputX, lateralPositionOutputY);

}

float PIDController::altitudeController(float zTarget, float zDotTarget, float zActual, float zDotActual, float feedForwardTerm) {

    float zError = zTarget - zActual;
    float zDotError = zDotTarget - zDotActual;

    float altitudeOutput = kpAltitude * zError + kdAltitude * zDotError + feedForwardTerm;

    return altitudeOutput;

    
} 

std::tuple<float, float> PIDController::rollPitchController(float altitudeOutput, float lateralPositionOutputX, float lateralPositionOutputY, Matrix3X3 rotationMatrix) {

    float rollPitchOutputP = 1 / rotationMatrix[2][2] * (rotationMatrix[1][0] * kpRollPitch * (lateralPositionOutputX - rotationMatrix[0][2]) - rotationMatrix[0][0] * kpRollPitch * (lateralPositionOutputY - rotationMatrix[1][2]));
    float rollPitchOutputQ = 1 / rotationMatrix[2][2] * (rotationMatrix[1][1] * kpRollPitch * (lateralPositionOutputX - rotationMatrix[0][2]) - rotationMatrix[0][1] * kpRollPitch * (lateralPositionOutputY - rotationMatrix[1][2]));

    return std::make_tuple(rollPitchOutputP, rollPitchOutputQ);
}

float PIDController::yawController(float yawTarget, float yawActual) {

	float yawError = yawTarget - yawActual;
	float yawOutput = kpYaw * yawError;

	return yawOutput;
}

std::tuple<float, float, float> PIDController::bodyRateController(float rollPitchOutputP, float rollPitchOutputQ, float yawOutput, float pActual, float qActual, float rActual) {

    float pError = rollPitchOutputP - pActual;
	float qError = rollPitchOutputQ - qActual;
	float rError = yawOutput - rActual;

	float bodyRateOutputP = kpBodyRateP * pError;
	float bodyRateOutputQ = kpBodyRateQ * qError;
	float bodyRateOutputR = kpBodyRateR * rError;

	return std::make_tuple(bodyRateOutputP, bodyRateOutputQ, bodyRateOutputR);
}


std::tuple<float, float, float> PIDController::attitudeController(float lateralPositionOutputX, float lateralPositionOutputY, float altitudeOutput, float yawActual, Matrix3X3 rotationMatrix, float pActual, float qActual, float rActual) {

    std::tuple<float, float> rollPitchOutput = rollPitchController(altitudeOutput, lateralPositionOutputX, lateralPositionOutputY, rotationMatrix);
    float rollPitchOutputP = std::get<0>(rollPitchOutput);
    float rollPitchOutputQ = std::get<1>(rollPitchOutput);
    float yawOutput = yawController(yawActual, yawActual);
    std::tuple<float, float, float> bodyRateOutput = bodyRateController(rollPitchOutputP, rollPitchOutputQ, yawOutput, pActual, qActual, rActual);

    return bodyRateOutput;
} 