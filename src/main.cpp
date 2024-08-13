#include <iostream>
#include "../include/obstacles.h"
#include "../include/controllers.h"
#include "../include/dynamics.h"
#include "../include/trajectory.h"
#include "../include/matrix_math.h"
#include "../include/lattice.h"
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <tuple>
#include <fstream>
#include <cmath>

using namespace std;

/**
 * @brief Trims leading and trailing whitespace from a string.
 * @param str The string to trim.
 * @return The trimmed string.
 */
string trim(const string& str) {
    size_t first = str.find_first_not_of(' ');
    if (first == string::npos) return "";
    size_t last = str.find_last_not_of(' ');
    return str.substr(first, last - first + 1);
}

/**
 * @brief Parses a JSON file and returns a mapping of key-value pairs.
 * @param filename The name of the JSON file.
 * @return A mapping of key-value pairs.
 */
map<string, float> parseJson(const string& filename) {
    ifstream file(filename);
    map<string, float> mappingOfData;
    string line;

    if (file.is_open()) {
        while (getline(file, line)) {
            line = trim(line);
            if (line.empty() || line[0] == '{' || line[0] == '}') continue;

            size_t colon = line.find(':');
            if (colon == string::npos) continue;

            string key = trim(line.substr(0, colon));
            key.erase(remove(key.begin(), key.end(), '"'), key.end());

            string valueStr = trim(line.substr(colon + 1));
            valueStr.erase(remove(valueStr.begin(), valueStr.end(), ','), valueStr.end());
            float value = stof(valueStr);

            mappingOfData[key] = value;
        }
        file.close();
    }
    return mappingOfData;
}

/**
 * @brief Writes the start and goal points to a file.
 * @param start The start point.
 * @param goal The goal point.
 */
void writePointsToFile(const Eigen::Vector3f& start, const Eigen::Vector3f& goal) {
    std::ofstream outFile("data/points.txt");
    if (outFile.is_open()) {
        outFile << start.x() << "," << start.y() << "," << start.z() << std::endl;
        outFile << goal.x() << "," << goal.y() << "," << goal.z() << std::endl;
        outFile.close();
    }
    else {
        std::cerr << "Unable to open file for writing" << std::endl;
    }
}

/**
 * @brief Writes the path to a file.
 * @param path The path to write.
 */
void writePathToFile(const vector<Eigen::Vector3f>& path) {
    std::ofstream outFile("data/path.txt");
    if (outFile.is_open()) {
        for (const auto& point : path) {
            outFile << point.x() << "," << point.y() << "," << point.z() << std::endl;
        }
        outFile.close();
    }
    else {
        std::cerr << "Unable to open file for writing" << std::endl;
    }
}

/**
 * @brief Writes the trajectory to a file.
 * @param timestamps The timestamps of the trajectory.
 * @param points The points of the trajectory.
 */
void writeTrajectoryToFile(const vector<float>& timestamps, const vector<Eigen::Vector3f>& points) {
    std::ofstream outFile("data/trajectory.txt");
    if (outFile.is_open()) {
        for (size_t i = 0; i < timestamps.size(); i++) {
            outFile << timestamps[i] << "," << points[i].x() << "," << points[i].y() << "," << points[i].z() << std::endl;
        }
        outFile.close();
    }
    else {
        std::cerr << "Unable to open file for writing" << std::endl; // cerr is used to output errors to the console
    }
}

/**
 * @brief From the trajectory points and timing data, computes a more detailed representation of 
 * the trajectory that includes the velocity and yaw of the drone at each point.
 * @param timestamps The timestamps of the trajectory.
 * @param points The points of the trajectory.
 * @param detailedPoints The detailed points of the trajectory. This is an empty vector that will be populated by the function.
 */
void computeTrajectoryDetails(const vector<float>& timestamps, const vector<Eigen::Vector3f>& points, vector<vector<float>>& detailedPoints) {
    for (size_t i = 0; i < points.size(); ++i) {
        Eigen::Vector3f currentPoint = points[i];

        float xDot = 0.0f, yDot = 0.0f, zDot = 0.0f;
        float yaw = 0.0f;

        if (i < points.size() - 1) {
            Eigen::Vector3f nextPoint = points[i + 1];
            float dt = timestamps[i + 1] - timestamps[i];

            xDot = (nextPoint.x() - currentPoint.x()) / dt;
            yDot = (nextPoint.y() - currentPoint.y()) / dt;
            zDot = (nextPoint.z() - currentPoint.z()) / dt;

            yaw = std::atan2(nextPoint.y() - currentPoint.y(), nextPoint.x() - currentPoint.x());
        }
        else if (i > 0) {
            Eigen::Vector3f prevPoint = points[i - 1];
            float dt = timestamps[i] - timestamps[i - 1];

            xDot = (currentPoint.x() - prevPoint.x()) / dt;
            yDot = (currentPoint.y() - prevPoint.y()) / dt;
            zDot = (currentPoint.z() - prevPoint.z()) / dt;

            yaw = std::atan2(currentPoint.y() - prevPoint.y(), currentPoint.x() - prevPoint.x());
        }

        // Override by making yaw always 0 for simplicity as I incrementally build this simulation
        detailedPoints.push_back({ currentPoint.x(), currentPoint.y(), currentPoint.z(), xDot, yDot, zDot, 0.0 });
    }
}

/**
 * @brief Writes the detailed trajectory to a file.
 * @param timestamps The timestamps of the trajectory.
 * @param detailedPoints The detailed trajectory data having form [x, y, z, xDot, yDot, zDot, yaw].
 */
void writeDetailedTrajectoryToFile(const vector<float>& timestamps, const vector<vector<float>>& detailedPoints) {
    std::ofstream outFile("data/detailed_trajectory.txt");
    if (outFile.is_open()) {
        for (size_t i = 0; i < timestamps.size(); i++) {
            outFile << timestamps[i] << ",";
            for (size_t j = 0; j < detailedPoints[i].size(); ++j) {
                outFile << detailedPoints[i][j];
                if (j < detailedPoints[i].size() - 1) {
                    outFile << ",";
                }
            }
            outFile << std::endl;
        }
        outFile.close();
    }
    else {
        std::cerr << "Unable to open file for writing" << std::endl;
    }
}

/**
 * @brief Generates a random pair of free space points.
 * @param droneMap The lattice representing the drone's environment.
 * @return A tuple containing the start and goal points.
 */
tuple<Eigen::Vector3f, Eigen::Vector3f> randomFreeSpacePointPair(const Lattice& droneMap) {
    int randomStartIndex = rand() % droneMap.getFreeSpacePoints().size();
    int randomGoalIndex = rand() % droneMap.getFreeSpacePoints().size();
    Eigen::Vector3f start = droneMap.getFreeSpacePoints()[randomStartIndex];
    Eigen::Vector3f goal = droneMap.getFreeSpacePoints()[randomGoalIndex];
    return make_tuple(start, goal);
}

/**
 * @brief Applies a quintic polynomial algorithm to solve for a smooth trajectory that passes through
 * each point of the given path. Saves the trajectory to a file. The function returns the trajectory
 * in a format [timestamp, x, y, z]. The spacing between timestamps is always 0.01 seconds. An average
 * velocity heuristic value is always used to generate the timestamps for the trajectory. In future
 * implementations, this functionality will be refined. For not, it is all that is needed. 
 * @param path The path to generate a trajectory for.
 * @param timestamps The timestamps of the trajectory.
 * @param points The points of the trajectory.
 */
void generateTrajectory(const vector<Eigen::Vector3f>& path, vector<float>& timestamps, vector<Eigen::Vector3f>& points) {
    float averageVelocity = 1.0f; // Heuristic used in generating a smooth trajectory
    vector<double> startTimes = assignSegmentStartTimes(path, averageVelocity);

    Eigen::VectorXd coeffsX = solveCoefficients(path, startTimes, 'x');
    Eigen::VectorXd coeffsY = solveCoefficients(path, startTimes, 'y');
    Eigen::VectorXd coeffsZ = solveCoefficients(path, startTimes, 'z');

    for (float t = 0; t < startTimes.back(); t += 0.01f) {
        Eigen::Vector3f point = evaluateTrajectory(coeffsX, coeffsY, coeffsZ, startTimes, t);
        points.push_back(point);
        timestamps.push_back(t);
    }

    writeTrajectoryToFile(timestamps, points);
}

/**
 * @brief Logs the results of the simulation to a file.
 * @param drone The drone to log.
 * @param altitudeOutput The output of the altitude controller.
 * @param rollPitchOutputP The output of the roll/pitch controller.
 * @param rollPitchOutputQ The output of the roll/pitch controller.
 * @param yawOutput The output of the yaw controller.
 * @param omega1 The angular velocity of motor 1.
 * @param omega2 The angular velocity of motor 2.
 * @param omega3 The angular velocity of motor 3.
 * @param omega4 The angular velocity of motor 4.
 */
void logResults(const Drone3D& drone, float altitudeOutput, float rollPitchOutputP, float rollPitchOutputQ, float yawOutput, float omega1, float omega2, float omega3, float omega4) {
    std::ofstream logFile("/data/log.txt", std::ios_base::app);
    if (logFile.is_open()) {
        logFile << drone.getClockTime() << ",";
        logFile << drone.getX() << "," << drone.getY() << "," << drone.getZ() << ",";
        logFile << drone.getPhi() << "," << drone.getTheta() << "," << drone.getPsi() << ",";
        logFile << drone.getXDot() << "," << drone.getYDot() << "," << drone.getZDot() << ",";
        logFile << drone.getP() << "," << drone.getQ() << "," << drone.getR() << ",";
        logFile << altitudeOutput << "," << rollPitchOutputP << "," << rollPitchOutputQ << "," << yawOutput << ",";
        logFile << omega1 << "," << omega2 << "," << omega3 << "," << omega4 << std::endl;
        logFile.close();
    }
    else {
        std::cerr << "Unable to open file for writing" << std::endl;
    }
}

// Use the data stored in detailed_trajectory (desired trajectory) and
// log.txt (actual trajectory) to evaluate the performance of the controller
// I will use mean squared error in the position and velocity of the drone
// as the performance metric.
void evaluatePerformance() {
    // Read the detailed trajectory from the file
    ifstream detailedTrajectoryFile("data/detailed_trajectory.txt");
    vector<vector<float>> detailedTrajectory;
    string line;
    while (getline(detailedTrajectoryFile, line)){
        vector<float> point;
        stringstream ss(line);
        string value;
        while (getline(ss, value, ',')){
            point.push_back(stof(value));
        }
        detailedTrajectory.push_back(point);
    }
    detailedTrajectoryFile.close();

    // Read the actual trajectory from the file
    ifstream actualTrajectoryFile("data/log.txt");
    vector<vector<float>> actualTrajectory;
    while (getline(actualTrajectoryFile, line)){
        vector<float> point;
        stringstream ss(line);
        string value;
        while (getline(ss, value, ',')){
            point.push_back(stof(value));
        }
        actualTrajectory.push_back(point);
    }
    actualTrajectoryFile.close();

    // Compute the mean squared error in position and velocity
    float positionError = 0.0f;
    float velocityError = 0.0f;
    for (size_t i = 0; i < detailedTrajectory.size(); i++){
        positionError += pow(detailedTrajectory[i][0] - actualTrajectory[i][1], 2) + pow(detailedTrajectory[i][1] - actualTrajectory[i][2], 2) + pow(detailedTrajectory[i][2] - actualTrajectory[i][3], 2);
        velocityError += pow(detailedTrajectory[i][3] - actualTrajectory[i][7], 2) + pow(detailedTrajectory[i][4] - actualTrajectory[i][8], 2) + pow(detailedTrajectory[i][5] - actualTrajectory[i][9], 2);
    }
    positionError /= detailedTrajectory.size();
    velocityError /= detailedTrajectory.size();

    cout << "Mean squared error in position: " << positionError << endl;
    cout << "Mean squared error in velocity: " << velocityError << endl;
}

int main() {
    // Initialize obstacle parser
    cout << "Generating map and path" << endl;
    string filename = "data/obstacles.csv";
    ObstacleParser obstacleParser(filename);

    // Generate a Lattice map representation of the environment
    cout << "Generating drone map" << endl;
    float resolution = 25.0f;
    Lattice droneMap = Lattice(obstacleParser, resolution);
    cout << "Drone map generated" << endl;

    // Generate a random pair of free space points for the start and goal
    cout << "Generating start and goal points" << endl;
    tuple<Eigen::Vector3f, Eigen::Vector3f> startGoalPair = randomFreeSpacePointPair(droneMap);
    Eigen::Vector3f start = get<0>(startGoalPair);
    Eigen::Vector3f goal = get<1>(startGoalPair);
    cout << "Start and goal points generated" << endl;

    // Write the start and goal points to a file
    cout << "Writing start and goal points to file" << endl;
    writePointsToFile(start, goal);
    cout << "Start and goal points written to file" << endl;

    // Perform an A* search to find a path between the start and goal points
    cout << "Performing A* search" << endl;
    vector<Eigen::Vector3f> path = droneMap.aStarSearch(start, goal);
    cout << "A* search completed" << endl;

    // Write the path to a file
    cout << "Writing path to file" << endl;
    writePathToFile(path);
    cout << "Path written to file" << endl;

    // Generate trajectory
    cout << "Generating trajectory" << endl;
    vector<float> timestamps;
    vector<Eigen::Vector3f> points;
    generateTrajectory(path, timestamps, points);
    cout << "Trajectory generated" << endl;

    // Compute detailed trajectory
    cout << "Computing detailed trajectory" << endl;
    vector<vector<float>> detailedPoints;
    computeTrajectoryDetails(timestamps, points, detailedPoints);
    writeDetailedTrajectoryToFile(timestamps, detailedPoints);
    cout << "Detailed trajectory computed and written to file" << endl;

    // Store detailed trajectory in a hashmap for easy access
    cout << "Storing detailed trajectory in hashmap" << endl;
    unordered_map<float, vector<float>> detailedTrajectory;
    for (size_t i = 0; i < timestamps.size(); i++) {
        detailedTrajectory[timestamps[i]] = detailedPoints[i];
    }

    // Show the contents of a portion of the hash map for debugging
    cout << "Displaying contents of detailed trajectory hashmap" << endl;
    for (size_t i = 0; i < 10; i++) {
        cout << "Timestamp: " << timestamps[i] << " ";
        for (size_t j = 0; j < detailedTrajectory[timestamps[i]].size(); j++) {
            cout << detailedTrajectory[timestamps[i]][j] << " ";
        }
        cout << endl;
    }

    // For now, assume that the simulation will run at the same rate as the trajectory
    // The target state will always be 0.01 seconds ahead of the drone's "clockTime"
        
    // Initialize the 3D drone with default parameters
    cout << "Initializing drone" << endl;
    Drone3D drone;
    cout << "Drone initialized" << endl;

    // For reference, the drone is initialized with a clockTime of 0.0 seconds and a dt of 0.01 seconds.

    // Initialize the controller with the PID gains specified in the JSON file
    cout << "Initializing controller" << endl;
    string pidFilename = "pid_gains.json";
    map<string, float> pidGains = parseJson(pidFilename);
    PIDController controller(pidGains["kpAltitude"], pidGains["kdAltitude"],
        pidGains["kpLateralPositionX"], pidGains["kdLateralPositionX"],
        pidGains["kpLateralPositionY"], pidGains["kdLateralPositionY"],
        pidGains["kpRollPitch"], pidGains["kpYaw"],
        pidGains["kpBodyRateP"], pidGains["kpBodyRateQ"], pidGains["kpBodyRateR"]);
    cout << "Controller initialized" << endl;

    // Begin simulation logic. 
    // Retrieve the target state by querying the detailed trajectory hashmap
    // for the drone's clockTime + dt seconds.

    // Target states comprise 7 elements:
    // x, y, z, xDot, yDot, zDot, yaw

    // Set a feedForwardTerm for the altitude controller
    float feedForwardTerm = 9.81f; // To counteract gravity

    // The simulation will run until the drone reaches the final target state.
    bool isSimulationOver = false; 

    while (!isSimulationOver){

        // Check if the drone has reached the final target state
        if (drone.getClockTime() >= timestamps.back()){
            isSimulationOver = true;
            break;
        }

        // Retrieve the target state for the current time
        std::vector<float> targetState = detailedTrajectory[drone.getClockTime() + drone.getDt() * 10]; // 10x faster rate of the inner loop

        // Extract the target state elements
        float xTarget = targetState[0];
        float yTarget = targetState[1];
        float zTarget = targetState[2];
        float xDotTarget = targetState[3];
        float yDotTarget = targetState[4];
        float zDotTarget = targetState[5];
        float yawTarget = targetState[6];

        // Retrieve the actual state elements from the drone
        float xActual = drone.getX();
        float yActual = drone.getY();
        float zActual = drone.getZ();
        float xDotActual = drone.getXDot();
        float yDotActual = drone.getYDot();
        float zDotActual = drone.getZDot();
        float yawActual = drone.getPsi();
        float pActual = drone.getP();
        float qActual = drone.getQ();
        float rActual = drone.getR();

        // Evaluate the altitude controller
        float altitudeOutput = controller.altitudeController(zTarget, zDotTarget, zActual, zDotActual, feedForwardTerm);

        // Evaluate the lateral position controller
        std::tuple<float, float> lateralPositionOutput = controller.lateralPositionController(xTarget, yTarget, xDotTarget, yDotTarget, xActual, yActual, xDotActual, yDotActual);

        // Assign a name to each of the lateral position outputs
        float lateralPositionOutputX = std::get<0>(lateralPositionOutput);
        float lateralPositionOutputY = std::get<1>(lateralPositionOutput);

        // Evaluate the attitude controller 10 times to represent the 10x faster rate of the inner loop
        for (int i = 0; i < 10; i++){

            // Evaluate the attitude controller
            std::tuple<float, float, float> attitudeOutput = controller.attitudeController(lateralPositionOutputX, lateralPositionOutputY, altitudeOutput, yawActual, drone.getRotationMatrix(), pActual, qActual, rActual);

            // Assign a name to each of the attitude outputs
            float rollPitchOutputP = std::get<0>(attitudeOutput);
            float rollPitchOutputQ = std::get<1>(attitudeOutput);
            float yawOutput = std::get<2>(attitudeOutput);

            // Convert the control signals to angular velocities
            tuple<double, double, double, double> angularVelocities = drone.controlSignalsToAngularVelocities(altitudeOutput, rollPitchOutputP, rollPitchOutputQ, yawOutput);

            // Name each of the angular velocities
            double omega1 = get<0>(angularVelocities);
            double omega2 = get<1>(angularVelocities);
            double omega3 = get<2>(angularVelocities);
            double omega4 = get<3>(angularVelocities);

            // Advance the state of the drone with the computed angular velocities
            // The drone advance state method is set to advance the state of the 
            // drone by 0.01/10 seconds to account for the 10x faster rate of the inner loop
            drone.advanceState(omega1, omega2, omega3, omega4);
            
            // print the control signals to the console
            cout << "u1: " << altitudeOutput << " u2: " << rollPitchOutputP << " u3: " << rollPitchOutputQ << " u4: " << yawOutput << endl;

            // print the four angular speeds to the console
            cout << "omega1: " << omega1 << " omega2: " << omega2 << " omega3: " << omega3 << " omega4: " << omega4 << endl;

            // print the clock time to the console
            cout << "Clock time: " << drone.getClockTime() << endl;

            // print the drone's position to the console
            cout << "Drone position: " << drone.getX() << " " << drone.getY() << " " << drone.getZ() << endl;

            // Log the results of the simulation in a file
            logResults(drone, altitudeOutput, rollPitchOutputP, rollPitchOutputQ, yawOutput, omega1, omega2, omega3, omega4);

        }

        // Evaluate the PID controlled trajectory against the desired trajectory using
        // a custom cost function.
        evaluatePerformance();

    }


    





    return 0;
}