#include "../include/dynamics.h"
#include "../include/matrix_math.h"
#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <iostream>

using namespace std;

/**
 * @brief Constructs a Drone3D object.
 * @param dt The time step used 
 * @param kf The thrust coefficient.
 */
Drone3D::Drone3D(
    const double dt,
    const double kf,
    const double km,
    const double ix,
    const double iy,
    const double iz,
    const double l
) : dt(dt), kf(kf), km(km), Ix(ix), Iy(iy), Iz(iz), l(l), omega1(0), omega2(0), omega3(0), omega4(0),
    x(0), y(0), z(0), phi(0), theta(0), psi(0), x_dot(0), y_dot(0), z_dot(0), p(0), q(0), r(0), m(1), g(9.81) {
		this->clockTime = 0.0;
	}

/*
I'm going to assume an XYZ coordinate system with the following features:
- X is forward
- Y is right
- Z is up (not down, as is common in applications like this)

I'm also going to assume a motor configuration like this:
- Motor 1 is at the front right and spins clockwise
- Motor 2 is at the back right and spins counterclockwise
- Motor 3 is at the back left and spins clockwise
- Motor 4 is at the front left and spins counterclockwise
*/

double Drone3D::F() const {
    return F1() + F2() + F3() + F4();
}

double Drone3D::F1() const {
    return kf * omega1 * omega1;
}

double Drone3D::F2() const {
    return kf * omega2 * omega2;
}

double Drone3D::F3() const {
    return kf * omega3 * omega3;
}

double Drone3D::F4() const {
    return kf * omega4 * omega4;
}

double Drone3D::M1() const {
    return km * omega1 * omega1;
}

double Drone3D::M2() const {
    return -km * omega2 * omega2;
}

double Drone3D::M3() const {
    return km * omega3 * omega3;
}

double Drone3D::M4() const {
    return -km * omega4 * omega4;
}

double Drone3D::Mx() const {
    return l * (-F1() - F2() + F3() + F4());
}

double Drone3D::My() const {
    return l * (F1() - F2() - F3() + F4());
}

double Drone3D::Mz() const {
    return M1() + M2() + M3() + M4();
}

Matrix3X3 Drone3D::R() const {
    vector<vector<double>> Rx = {
        {1, 0, 0},
        {0, cos(phi), -sin(phi)},
        {0, sin(phi), cos(phi)}
    };

    vector<vector<double>> Ry = {
        {cos(theta), 0, sin(theta)},
        {0, 1, 0},
        {-sin(theta), 0, cos(theta)}
    };

    vector<vector<double>> Rz = {
        {cos(psi), -sin(psi), 0},
        {sin(psi), cos(psi), 0},
        {0, 0, 1}
    };

    return Matrix3X3(Rz) * (Matrix3X3(Ry) * Matrix3X3(Rx));
}

void Drone3D::advanceAttitude() {
    double p_dot = Mx() / Ix;
    double q_dot = My() / Iy;
    double r_dot = Mz() / Iz;

    p += p_dot * dt;
    q += q_dot * dt;
    r += r_dot * dt;

    // This transformation matrix has a specific name
    // and can be found in the literature.
    Matrix3X3 transformationMatrix({
        {1, sin(phi) * tan(theta), cos(phi) * tan(theta)},
        {0, cos(phi), -sin(phi)},
        {0, sin(phi) / cos(theta), cos(phi) / cos(theta)}
    });

    vector<double> attitudeRates = {p, q, r};
    vector<double> attitudeRatesInBodyFrame = transformationMatrix * attitudeRates;
    phi += attitudeRatesInBodyFrame[0] * dt;
    theta += attitudeRatesInBodyFrame[1] * dt;
    psi += attitudeRatesInBodyFrame[2] * dt;
}

void Drone3D::advancePosition() {
    vector<double> appliedBodyForces = {0, 0, F()};
    vector<double> appliedBodyForcesInWorldFrame = R() * appliedBodyForces;
    // Subtract the force of gravity
    appliedBodyForcesInWorldFrame[2] -= m * g;
    // Divide each force component by mass to get acceleration
    vector<double> acceleration = appliedBodyForcesInWorldFrame;
    acceleration[0] /= m;
    acceleration[1] /= m;
    acceleration[2] /= m;

    // Update velocity
    x_dot += acceleration[0] * dt;
    y_dot += acceleration[1] * dt;
    z_dot += acceleration[2] * dt;

    // Update position
    x += x_dot * dt;
    y += y_dot * dt;
    z += z_dot * dt;
}

tuple<double, double, double, double> Drone3D::controlSignalsToAngularVelocities(const double u1, const double u2, const double u3, const double u4) {
    // u1 corresponds to collective thrust
    // u2 corresponds to roll moment
    // u3 corresponds to pitch moment
    // u4 corresponds to yaw moment
    // Formulate a system of equations to solve for omega1, omega2, omega3, omega4
    // The equation has the form A * omega^2 = b where A is a 4x4 matrix, omega is a 4x1 vector, and b is a 4x1 vector.
    // The A matrix is composed of the coefficients of the omega^2 terms.

    Eigen::Matrix4d A;
    A << kf, kf, kf, kf,
        -kf * l, -kf * l, kf* l, kf* l,
        kf*l, -kf*l, -kf*l, kf*l,
        km, -km, km, -km;

    Eigen::Vector4d b;
    b << u1, u2, u3, u4;

    Eigen::Vector4d omegaSquared = A.inverse() * b;

    return { sqrt(omegaSquared(0)), sqrt(omegaSquared(1)), sqrt(omegaSquared(2)), sqrt(omegaSquared(3)) };
}

void Drone3D::advanceState(const double u1, const double u2, const double u3, const double u4) {
    auto [omega1, omega2, omega3, omega4] = controlSignalsToAngularVelocities(u1, u2, u3, u4);
    this->omega1 = omega1;
    this->omega2 = omega2;
    this->omega3 = omega3;
    this->omega4 = omega4;

    advanceAttitude();
    advancePosition();
	updateClockTime();

}

void Drone3D::updateClockTime() {
	clockTime += dt;
}

void Drone3D::updateLog() {
	std::ofstream logFile("../data/log.txt", std::ios_base::app);
	if (logFile.is_open()) {
		logFile << std::fixed << std::setprecision(2);
		logFile << clockTime << ",";
		logFile << x << "," << y << "," << z << ",";
		logFile << phi << "," << theta << "," << psi << ",";
		logFile << x_dot << "," << y_dot << "," << z_dot << ",";
		logFile << p << "," << q << "," << r << ",";
		logFile << omega1 << "," << omega2 << "," << omega3 << "," << omega4 << std::endl;
		logFile.close();
	}
	else {
		std::cerr << "Unable to open file for writing" << std::endl;
	}
}
