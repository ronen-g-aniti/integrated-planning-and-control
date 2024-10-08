#pragma once

#include <vector>
#include <tuple>
#include "matrix_math.h"
#include <string>

/**
 * @class Drone3D
 * @brief Represents a 3D drone model for simulation purposes.
 *
 * This class encapsulates the physical and dynamic properties of a 3D drone,
 * allowing for state advancement based on rotor speeds and physical parameters.
 */
class Drone3D {
private:
    double dt; ///< Time step for simulation
    double kf; ///< Thrust coefficient
    double km; ///< Drag coefficient
    double l; ///< Distance from the center of the drone to each rotor
    double Ix, Iy, Iz; ///< Moments of inertia around the x, y, and z axes
    double omega1, omega2, omega3, omega4; ///< Angular velocities of the four rotors
    double x, y, z; ///< Position coordinates in 3D space
    double phi, theta, psi; ///< Orientation angles (roll, pitch, yaw)
    double x_dot, y_dot, z_dot; ///< Linear velocities in 3D space
    double p, q, r; ///< Angular velocities around the x, y, and z axes
    double m; ///< Mass of the drone
    double g; ///< Gravitational acceleration
    double clockTime; ///< Current time in the simulation

    /**
     * @brief Advances the attitude (orientation) of the drone based on current state and inputs.
     */
    void advanceAttitude();

    /**
     * @brief Advances the position of the drone based on current state and inputs.
     */
    void advancePosition();

    /**
     * @brief Updates the current time in the simulation.
     */
    void updateClockTime();

    /**
     * @brief Updates the log file with the current state of the drone and the control action just taken. 
     */
    void updateLog();

public:
    /**
     * @brief Constructs a new Drone3D object with specified parameters.
     *
     * @param dt Time step for the advance state method -- take 0.01 and divide by the speedup factor of the inner loop, which will be 10 for now.
     * @param kf Thrust coefficient (default is 0.0000015)
     * @param km Drag coefficient (default is 0.00000015)
     * @param ix Moment of inertia around the x-axis (default is 0.01)
     * @param iy Moment of inertia around the y-axis (default is 0.01)
     * @param iz Moment of inertia around the z-axis (default is 0.01)
     * @param l Distance from the center of the drone to each rotor (default is 0.25)
     */
    Drone3D(
        const double dt = 0.01 / 10.0, // Take 0.01 and divide by the speedup factor of the inner loop, which will be 10 for now
        const double kf = 0.0000015,
        const double km = 0.00000015,
        const double ix = 0.01,
        const double iy = 0.01,
        const double iz = 0.01,
        const double l = 0.25
    );

    /**
     * @brief Computes the total thrust generated by the drone.
     * @return Total thrust.
     */
    double F() const;

    /**
     * @brief Computes the thrust generated by rotor 1.
     * @return Thrust from rotor 1.
     */
    double F1() const;

    /**
     * @brief Computes the thrust generated by rotor 2.
     * @return Thrust from rotor 2.
     */
    double F2() const;

    /**
     * @brief Computes the thrust generated by rotor 3.
     * @return Thrust from rotor 3.
     */
    double F3() const;

    /**
     * @brief Computes the thrust generated by rotor 4.
     * @return Thrust from rotor 4.
     */
    double F4() const;

    /**
     * @brief Computes the moment generated by rotor 1.
     * @return Moment from rotor 1.
     */
    double M1() const;

    /**
     * @brief Computes the moment generated by rotor 2.
     * @return Moment from rotor 2.
     */
    double M2() const;

    /**
     * @brief Computes the moment generated by rotor 3.
     * @return Moment from rotor 3.
     */
    double M3() const;

    /**
     * @brief Computes the moment generated by rotor 4.
     * @return Moment from rotor 4.
     */
    double M4() const;

    /**
     * @brief Computes the moment around the x-axis.
     * @return Moment around the x-axis.
     */
    double Mx() const;

    /**
     * @brief Computes the moment around the y-axis.
     * @return Moment around the y-axis.
     */
    double My() const;

    /**
     * @brief Computes the moment around the z-axis.
     * @return Moment around the z-axis.
     */
    double Mz() const;

    /**
     * @brief Computes the body frame to world frame transformation matrix.
     * @return Transformation matrix.
     */
    Matrix3X3 R() const;

    /**
     * @brief Advances the state of the drone based on the given rotor speeds.
     *
     * @param omega1 Angular velocity of rotor 1.
     * @param omega2 Angular velocity of rotor 2.
     * @param omega3 Angular velocity of rotor 3.
     * @param omega4 Angular velocity of rotor 4.
     */
    void advanceState(const double u1, const double u2, const double u3, const double u4);

    /**
     * @brief Converts control signals to angular velocities for the rotors.
     *
     * @param u1 Control signal for rotor 1.
     * @param u2 Control signal for rotor 2.
     * @param u3 Control signal for rotor 3.
     * @param u4 Control signal for rotor 4.
     * @return Tuple containing the angular velocities for the four rotors.
     */
    tuple<double, double, double, double> controlSignalsToAngularVelocities(const double u1, const double u2, const double u3, const double u4);



    // Getter methods for state variables
    double getX() const { return x; }
    double getY() const { return y; }
    double getZ() const { return z; }
    double getPhi() const { return phi; }
    double getTheta() const { return theta; }
    double getPsi() const { return psi; }
    double getXDot() const { return x_dot; }
    double getYDot() const { return y_dot; }
    double getZDot() const { return z_dot; }
    double getP() const { return p; }
    double getQ() const { return q; }
    double getR() const { return r; }
    double getClockTime() const { return clockTime; }
    double getDt() const { return dt; }
    Matrix3X3 getRotationMatrix() const {
        return R();
    }
    // Set propeller speeds
    void setPropellerSpeeds(const double omega1, const double omega2, const double omega3, const double omega4) {
        this->omega1 = omega1;
        this->omega2 = omega2;
        this->omega3 = omega3;
        this->omega4 = omega4;
    }

};
