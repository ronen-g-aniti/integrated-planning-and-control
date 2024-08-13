import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random 
from scipy.interpolate import CubicSpline

# Genetic algorithm parameters
POPULATION_SIZE = 20
GENERATIONS = 50
MUTATION_RATE = 0.1

class ControllerConfig:
    def __init__(self, kpAltitude, kdAltitude, kpLateralPositionX, kdLateralPositionX,
                 kpLateralPositionY, kdLateralPositionY, kpRollPitch, kpYaw,
                 kpBodyRateP, kpBodyRateQ, kpBodyRateR):
        self.kpAltitude = kpAltitude
        self.kdAltitude = kdAltitude
        self.kpLateralPositionX = kpLateralPositionX
        self.kdLateralPositionX = kdLateralPositionX
        self.kpLateralPositionY = kpLateralPositionY
        self.kdLateralPositionY = kdLateralPositionY
        self.kpRollPitch = kpRollPitch
        self.kpYaw = kpYaw
        self.kpBodyRateP = kpBodyRateP
        self.kpBodyRateQ = kpBodyRateQ
        self.kpBodyRateR = kpBodyRateR

class Drone3D:
    def __init__(self, dt=0.01, kf=0.0000015, km=0.00000015, ix=0.01, iy=0.01, iz=0.01, l=0.25):
        self.dt = dt
        self.kf = kf
        self.km = km
        self.Ix = ix
        self.Iy = iy
        self.Iz = iz
        self.l = l
        
        self.omega1 = 0
        self.omega2 = 0
        self.omega3 = 0
        self.omega4 = 0
        
        self.x = 0
        self.y = 0
        self.z = 0
        self.phi = 0
        self.theta = 0
        self.psi = 0
        
        self.x_dot = 0
        self.y_dot = 0
        self.z_dot = 0
        self.p = 0
        self.q = 0
        self.r = 0
        
        self.m = 1  # mass of the drone
        self.g = 9.81  # acceleration due to gravity
        
        self.clockTime = 0.0

    def F(self):
        return self.F1() + self.F2() + self.F3() + self.F4()

    def F1(self):
        return self.kf * self.omega1 ** 2

    def F2(self):
        return self.kf * self.omega2 ** 2

    def F3(self):
        return self.kf * self.omega3 ** 2

    def F4(self):
        return self.kf * self.omega4 ** 2

    def M1(self):
        return self.km * self.omega1 ** 2

    def M2(self):
        return -self.km * self.omega2 ** 2

    def M3(self):
        return self.km * self.omega3 ** 2

    def M4(self):
        return -self.km * self.omega4 ** 2

    def Mx(self):
        return self.l * (-self.F1() - self.F2() + self.F3() + self.F4())

    def My(self):
        return self.l * (self.F1() - self.F2() - self.F3() + self.F4())

    def Mz(self):
        return self.M1() + self.M2() + self.M3() + self.M4()

    def R(self):
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(self.phi), -np.sin(self.phi)],
            [0, np.sin(self.phi), np.cos(self.phi)]
        ])

        Ry = np.array([
            [np.cos(self.theta), 0, np.sin(self.theta)],
            [0, 1, 0],
            [-np.sin(self.theta), 0, np.cos(self.theta)]
        ])

        Rz = np.array([
            [np.cos(self.psi), -np.sin(self.psi), 0],
            [np.sin(self.psi), np.cos(self.psi), 0],
            [0, 0, 1]
        ])

        return Rz @ (Ry @ Rx)

    def advanceAttitude(self):
        p_dot = self.Mx() / self.Ix
        q_dot = self.My() / self.Iy
        r_dot = self.Mz() / self.Iz

        self.p += p_dot * self.dt
        self.q += q_dot * self.dt
        self.r += r_dot * self.dt

        transformationMatrix = np.array([
            [1, np.sin(self.phi) * np.tan(self.theta), np.cos(self.phi) * np.tan(self.theta)],
            [0, np.cos(self.phi), -np.sin(self.phi)],
            [0, np.sin(self.phi) / np.cos(self.theta), np.cos(self.phi) / np.cos(self.theta)]
        ])

        attitudeRates = np.array([self.p, self.q, self.r])
        attitudeRatesInBodyFrame = transformationMatrix @ attitudeRates
        self.phi += attitudeRatesInBodyFrame[0] * self.dt
        self.theta += attitudeRatesInBodyFrame[1] * self.dt
        self.psi += attitudeRatesInBodyFrame[2] * self.dt

    def advancePosition(self):
        appliedBodyForces = np.array([0, 0, self.F()])
        appliedBodyForcesInWorldFrame = self.R() @ appliedBodyForces
        appliedBodyForcesInWorldFrame[2] -= self.m * self.g

        acceleration = appliedBodyForcesInWorldFrame / self.m

        self.x_dot += acceleration[0] * self.dt
        self.y_dot += acceleration[1] * self.dt
        self.z_dot += acceleration[2] * self.dt

        self.x += self.x_dot * self.dt
        self.y += self.y_dot * self.dt
        self.z += self.z_dot * self.dt

    def controlSignalsToAngularVelocities(self, u1, u2, u3, u4):
        A = np.array([
            [self.kf, self.kf, self.kf, self.kf],
            [-self.kf * self.l, -self.kf * self.l, self.kf * self.l, self.kf * self.l],
            [self.kf * self.l, -self.kf * self.l, -self.kf * self.l, self.kf * self.l],
            [self.km, -self.km, self.km, -self.km]
        ])

        b = np.array([u1, u2, u3, u4])
        omegaSquared = np.linalg.inv(A) @ b

        # Handling negative values in omegaSquared
        omegaSquared = np.where(omegaSquared < 0, 0, omegaSquared)

        return tuple(np.sqrt(omegaSquared))

    def advanceState(self, u1, u2, u3, u4):
        self.omega1, self.omega2, self.omega3, self.omega4 = self.controlSignalsToAngularVelocities(u1, u2, u3, u4)
        self.advanceAttitude()
        self.advancePosition()
        self.updateClockTime()

    def updateClockTime(self):
        self.clockTime += self.dt

    def updateLog(self, log_filename="test_log.txt"):
        with open(log_filename, "a") as logFile:
            logFile.write(f"{self.clockTime:.2f},{self.x:.2f},{self.y:.2f},{self.z:.2f},")
            logFile.write(f"{self.phi:.2f},{self.theta:.2f},{self.psi:.2f},")
            logFile.write(f"{self.x_dot:.2f},{self.y_dot:.2f},{self.z_dot:.2f},")
            logFile.write(f"{self.p:.2f},{self.q:.2f},{self.r:.2f},")
            logFile.write(f"{self.omega1:.2f},{self.omega2:.2f},{self.omega3:.2f},{self.omega4:.2f}\n")


class PIDController:
    def __init__(self, config):
        self.config = config

    def lateralPositionController(self, xTarget, yTarget, xDotTarget, yDotTarget, xActual, yActual, xDotActual, yDotActual):
        xError = xTarget - xActual
        yError = yTarget - yActual

        xDotError = xDotTarget - xDotActual
        yDotError = yDotTarget - yDotActual

        lateralPositionOutputX = self.config.kpLateralPositionX * xError + self.config.kdLateralPositionX * xDotError
        lateralPositionOutputY = self.config.kpLateralPositionY * yError + self.config.kdLateralPositionY * yDotError

        return lateralPositionOutputX, lateralPositionOutputY

    def altitudeController(self, zTarget, zDotTarget, zActual, zDotActual, feedForwardTerm=9.81):
        zError = zTarget - zActual
        zDotError = zDotTarget - zDotActual

        altitudeOutput = self.config.kpAltitude * zError + self.config.kdAltitude * zDotError + feedForwardTerm

        return altitudeOutput

    def rollPitchController(self, altitudeOutput, lateralPositionOutputX, lateralPositionOutputY, rotationMatrix):
        rollPitchOutputP = (1 / rotationMatrix[2, 2]) * (
            rotationMatrix[1, 0] * self.config.kpRollPitch * (lateralPositionOutputX - rotationMatrix[0, 2])
            - rotationMatrix[0, 0] * self.config.kpRollPitch * (lateralPositionOutputY - rotationMatrix[1, 2])
        )

        rollPitchOutputQ = (1 / rotationMatrix[2, 2]) * (
            rotationMatrix[1, 1] * self.config.kpRollPitch * (lateralPositionOutputX - rotationMatrix[0, 2])
            - rotationMatrix[0, 1] * self.config.kpRollPitch * (lateralPositionOutputY - rotationMatrix[1, 2])
        )

        return rollPitchOutputP, rollPitchOutputQ

    def yawController(self, yawTarget, yawActual):
        yawError = yawTarget - yawActual
        yawOutput = self.config.kpYaw * yawError

        return yawOutput

    def bodyRateController(self, rollPitchOutputP, rollPitchOutputQ, yawOutput, pActual, qActual, rActual):
        pError = rollPitchOutputP - pActual
        qError = rollPitchOutputQ - qActual
        rError = yawOutput - rActual

        bodyRateOutputP = self.config.kpBodyRateP * pError
        bodyRateOutputQ = self.config.kpBodyRateQ * qError
        bodyRateOutputR = self.config.kpBodyRateR * rError

        return bodyRateOutputP, bodyRateOutputQ, bodyRateOutputR

    def attitudeController(self, lateralPositionOutputX, lateralPositionOutputY, altitudeOutput, yawTarget, rotationMatrix, pActual, qActual, rActual):
        rollPitchOutputP, rollPitchOutputQ = self.rollPitchController(altitudeOutput, lateralPositionOutputX, lateralPositionOutputY, rotationMatrix)
        yawOutput = self.yawController(yawTarget, yawTarget)
        bodyRateOutputP, bodyRateOutputQ, bodyRateOutputR = self.bodyRateController(rollPitchOutputP, rollPitchOutputQ, yawOutput, pActual, qActual, rActual)

        return bodyRateOutputP, bodyRateOutputQ, bodyRateOutputR


def generate_trajectory(duration, time_step):
    # Define key waypoints and their corresponding times
    waypoint_times = np.array([0, duration / 4, duration / 2, 3 * duration / 4, duration])
    waypoints = np.array([
        [0, 0, 1, 0],      # [x, y, z, yaw] at time 0
        [1, 1, 1.5, np.pi/4],  # At 1/4 duration
        [2, 0, 2, np.pi/2],    # At 1/2 duration
        [1, -1, 1.5, 3*np.pi/4], # At 3/4 duration
        [0, 0, 1, np.pi]       # At final time
    ])

    # Generate cubic splines for each dimension
    cs_x = CubicSpline(waypoint_times, waypoints[:, 0])
    cs_y = CubicSpline(waypoint_times, waypoints[:, 1])
    cs_z = CubicSpline(waypoint_times, waypoints[:, 2])
    cs_yaw = CubicSpline(waypoint_times, waypoints[:, 3])

    # Generate time points
    time_points = np.arange(0, duration + time_step, time_step)

    # Evaluate splines at each time point for position
    x_trajectory = cs_x(time_points)
    y_trajectory = cs_y(time_points)
    z_trajectory = cs_z(time_points)
    yaw_trajectory = cs_yaw(time_points)

    # Evaluate splines at each time point for velocity (first derivative)
    x_dot_trajectory = cs_x.derivative()(time_points)
    y_dot_trajectory = cs_y.derivative()(time_points)
    z_dot_trajectory = cs_z.derivative()(time_points)
    yaw_dot_trajectory = cs_yaw.derivative()(time_points)

    # Combine into a single trajectory array (with velocity included)
    trajectory = np.vstack((
        time_points, 
        x_trajectory, y_trajectory, z_trajectory, yaw_trajectory, 
        x_dot_trajectory, y_dot_trajectory, z_dot_trajectory, yaw_dot_trajectory
    )).T

    return trajectory

def save_trajectory_to_file(trajectory, filename):
    np.savetxt(filename, trajectory, fmt="%.2f", delimiter=",",
               header="t,x,y,z,yaw,x_dot,y_dot,z_dot,yaw_dot", comments='')
    
def evaluate_controller_performance(trajectory, log_filename):
    # Read the log file
    log_data = np.genfromtxt(log_filename, delimiter=',', skip_header=1)

    # Extract the relevant columns from the trajectory and log data
    trajectory_subset = trajectory[:, [1, 2, 3, 4, 5, 6, 7, 8]]  # x, y, z, yaw, x_dot, y_dot, z_dot, yaw_dot
    log_data_subset = np.hstack((log_data[:, [1, 2, 3, 6]], log_data[:, [7, 8, 9, 10]]))  # x, y, z, yaw, x_dot, y_dot, z_dot, yaw_dot

    # Calculate the mean squared error for each state variable
    mse = np.mean((trajectory_subset - log_data_subset) ** 2, axis=0)

    return mse

def initialize_population():
    population = []
    for _ in range(POPULATION_SIZE):
        config = ControllerConfig(
            kpAltitude=np.random.uniform(0.0, 100.0),
            kdAltitude=np.random.uniform(0.0, 100.0),
            kpLateralPositionX=np.random.uniform(0.0, 100.0),
            kdLateralPositionX=np.random.uniform(0.0, 100.0),
            kpLateralPositionY=np.random.uniform(0.0, 100.0),
            kdLateralPositionY=np.random.uniform(0.0, 100.0),
            kpRollPitch=np.random.uniform(0.0, 100.0),
            kpYaw=np.random.uniform(0.0, 100.0),
            kpBodyRateP=np.random.uniform(0.0, 100.0),
            kpBodyRateQ=np.random.uniform(0.0, 100.0),
            kpBodyRateR=np.random.uniform(0.0, 100.0)
        )
        population.append(config)
    return population

def fitness_function(config):
    drone = Drone3D()
    controller = PIDController(config)
    duration = 10.0
    time_step = 0.01
    trajectory = generate_trajectory(duration, time_step)
    log_filename = "test_log.txt"
    
    with open(log_filename, "w") as logFile:
        logFile.write("time,x,y,z,phi,theta,psi,x_dot,y_dot,z_dot,p,q,r,omega1,omega2,omega3,omega4\n")
    
    for i in range(len(trajectory)):
        t, x_target, y_target, z_target, yaw_target, x_dot_target, y_dot_target, z_dot_target, yaw_dot_target = trajectory[i]
        x_actual, y_actual, z_actual = drone.x, drone.y, drone.z
        yaw_actual = drone.psi
        p_actual, q_actual, r_actual = drone.p, drone.q, drone.r
        rotation_matrix = drone.R()

        # Lateral position controller
        lateral_position_output_x, lateral_position_output_y = controller.lateralPositionController(
            x_target, y_target, x_dot_target, y_dot_target, x_actual, y_actual, drone.x_dot, drone.y_dot)

        # Altitude controller
        altitude_output = controller.altitudeController(
            z_target, z_dot_target, z_actual, drone.z_dot)

        # Attitude controller
        body_rate_output_p, body_rate_output_q, body_rate_output_r = controller.attitudeController(
            lateral_position_output_x, lateral_position_output_y, altitude_output, yaw_target, rotation_matrix, p_actual, q_actual, r_actual)

        # Update the state of the drone
        drone.advanceState(altitude_output, lateral_position_output_x, lateral_position_output_y, yaw_target)

        # Log the state of the drone
        drone.updateLog(log_filename)
    
    mse = evaluate_controller_performance(trajectory, log_filename)
    return np.mean(mse)


def select_parents(population, fitnesses):
    parents = []
    for _ in range(2):
        idx = random.choices(range(len(population)), weights=fitnesses, k=1)[0]
        parents.append(population[idx])
    return parents

def crossover(parent1, parent2):
    child = ControllerConfig(
        kpAltitude=(parent1.kpAltitude + parent2.kpAltitude) / 2,
        kdAltitude=(parent1.kdAltitude + parent2.kdAltitude) / 2,
        kpLateralPositionX=(parent1.kpLateralPositionX + parent2.kpLateralPositionX) / 2,
        kdLateralPositionX=(parent1.kdLateralPositionX + parent2.kdLateralPositionX) / 2,
        kpLateralPositionY=(parent1.kpLateralPositionY + parent2.kpLateralPositionY) / 2,
        kdLateralPositionY=(parent1.kdLateralPositionY + parent2.kdLateralPositionY) / 2,
        kpRollPitch=(parent1.kpRollPitch + parent2.kpRollPitch) / 2,
        kpYaw=(parent1.kpYaw + parent2.kpYaw) / 2,
        kpBodyRateP=(parent1.kpBodyRateP + parent2.kpBodyRateP) / 2,
        kpBodyRateQ=(parent1.kpBodyRateQ + parent2.kpBodyRateQ) / 2,
        kpBodyRateR=(parent1.kpBodyRateR + parent2.kpBodyRateR) / 2,
    )
    return child

def mutate(child):
    if np.random.rand() < MUTATION_RATE:
        child.kpAltitude *= np.random.uniform(0.8, 1.2)
    if np.random.rand() < MUTATION_RATE:
        child.kdAltitude *= np.random.uniform(0.8, 1.2)
    if np.random.rand() < MUTATION_RATE:
        child.kpLateralPositionX *= np.random.uniform(0.8, 1.2)
    if np.random.rand() < MUTATION_RATE:
        child.kdLateralPositionX *= np.random.uniform(0.8, 1.2)
    if np.random.rand() < MUTATION_RATE:
        child.kpLateralPositionY *= np.random.uniform(0.8, 1.2)
    if np.random.rand() < MUTATION_RATE:
        child.kdLateralPositionY *= np.random.uniform(0.8, 1.2)
    if np.random.rand() < MUTATION_RATE:
        child.kpRollPitch *= np.random.uniform(0.8, 1.2)
    if np.random.rand() < MUTATION_RATE:
        child.kpYaw *= np.random.uniform(0.8, 1.2)
    if np.random.rand() < MUTATION_RATE:
        child.kpBodyRateP *= np.random.uniform(0.8, 1.2)
    if np.random.rand() < MUTATION_RATE:
        child.kpBodyRateQ *= np.random.uniform(0.8, 1.2)
    if np.random.rand() < MUTATION_RATE:
        child.kpBodyRateR *= np.random.uniform(0.8, 1.2)
    return child

def genetic_algorithm():
    population = initialize_population()
    for generation in range(GENERATIONS):
        fitnesses = [1 / fitness_function(config) for config in population]
        
        new_population = []
        for _ in range(POPULATION_SIZE):
            parent1, parent2 = select_parents(population, fitnesses)
            child = crossover(parent1, parent2)
            child = mutate(child)
            new_population.append(child)
        
        population = new_population
        best_fitness = max(fitnesses)
        print(f"Generation {generation + 1}: Best fitness = {best_fitness:.6f}")

    best_idx = np.argmax(fitnesses)
    return population[best_idx]

def main():
    duration = 10.0  # seconds
    time_step = 0.01  # seconds

    # Generate the trajectory
    trajectory = generate_trajectory(duration, time_step)
    save_trajectory_to_file(trajectory, "test_trajectory.txt")
    print("Trajectory saved to test_trajectory.txt")

    # Run genetic algorithm to find the best PID configuration
    best_config = genetic_algorithm()
    print("Best PID configuration found:")
    print(f"kpAltitude: {best_config.kpAltitude}, kdAltitude: {best_config.kdAltitude}")
    print(f"kpLateralPositionX: {best_config.kpLateralPositionX}, kdLateralPositionX: {best_config.kdLateralPositionX}")
    print(f"kpLateralPositionY: {best_config.kpLateralPositionY}, kdLateralPositionY: {best_config.kdLateralPositionY}")
    print(f"kpRollPitch: {best_config.kpRollPitch}, kpYaw: {best_config.kpYaw}")
    print(f"kpBodyRateP: {best_config.kpBodyRateP}, kpBodyRateQ: {best_config.kpBodyRateQ}, kpBodyRateR: {best_config.kpBodyRateR}")

    # Use the best PID configuration to run the final simulation
    drone = Drone3D()
    controller = PIDController(best_config)

    log_filename = "test_log.txt"
    with open(log_filename, "w") as logFile:
        logFile.write("time,x,y,z,phi,theta,psi,x_dot,y_dot,z_dot,p,q,r,omega1,omega2,omega3,omega4\n")
    
    for i in range(len(trajectory)):
        t, x_target, y_target, z_target, yaw_target, x_dot_target, y_dot_target, z_dot_target, yaw_dot_target = trajectory[i]
        x_actual, y_actual, z_actual = drone.x, drone.y, drone.z
        yaw_actual = drone.psi
        p_actual, q_actual, r_actual = drone.p, drone.q, drone.r
        rotation_matrix = drone.R()

        lateral_position_output_x, lateral_position_output_y = controller.lateralPositionController(
            x_target, y_target, x_dot_target, y_dot_target, x_actual, y_actual, drone.x_dot, drone.y_dot)
        altitude_output = controller.altitudeController(
            z_target, z_dot_target, z_actual, drone.z_dot)
        body_rate_output_p, body_rate_output_q, body_rate_output_r = controller.attitudeController(
            lateral_position_output_x, lateral_position_output_y, altitude_output, yaw_target, rotation_matrix, p_actual, q_actual, r_actual)

        drone.advanceState(altitude_output, lateral_position_output_x, lateral_position_output_y, yaw_target)
        drone.updateLog(log_filename)

    print("Simulation completed. Log file saved to test_log.txt")

    mse = evaluate_controller_performance(trajectory, log_filename)
    print("Mean squared error for each state variable:")
    print(f"X: {mse[0]:.2f}")
    print(f"Y: {mse[1]:.2f}")
    print(f"Z: {mse[2]:.2f}")
    print(f"Yaw: {mse[3]:.2f}")

    position_mse = np.mean(mse[:3])
    print(f"Position MSE: {position_mse:.2f}")

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(trajectory[:, 1], trajectory[:, 2], trajectory[:, 3], c='m', label='Desired Trajectory')
    ax.scatter(trajectory[:, 1], trajectory[:, 2], trajectory[:, 3], c='r', marker='o', s=10)
    ax.plot(np.genfromtxt(log_filename, delimiter=',', skip_header=1)[:, 1], np.genfromtxt(log_filename, delimiter=',', skip_header=1)[:, 2], np.genfromtxt(log_filename, delimiter=',', skip_header=1)[:, 3], c='b', label='Simulated Trajectory')
    ax.scatter(np.genfromtxt(log_filename, delimiter=',', skip_header=1)[:, 1], np.genfromtxt(log_filename, delimiter=',', skip_header=1)[:, 2], np.genfromtxt(log_filename, delimiter=',', skip_header=1)[:, 3], c='g', marker='o', s=10)
    plt.show()

if __name__ == "__main__":
    main()
