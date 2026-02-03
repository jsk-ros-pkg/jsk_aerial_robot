/*
 * MINCO Trajectory Test
 * 
 * This file demonstrates basic usage of the MINCO trajectory generation library.
 * It generates a simple minimum jerk trajectory and evaluates velocity at a specific time.
 */

#include <iostream>
#include <chrono>
#include <Eigen/Eigen>
#include "aerial_robot_control/minco_trajectory/minco.hpp"
#include "aerial_robot_control/minco_trajectory/trajectory.hpp"

int main(int argc, char** argv)
{
    std::cout << "========================================" << std::endl;
    std::cout << "  MINCO Trajectory Generation Test" << std::endl;
    std::cout << "========================================" << std::endl << std::endl;

    // Create a MINCO_S3NU instance for minimum jerk trajectory (s=3)
    minco::MINCO_S3NU minco;

    // Define the number of trajectory pieces
    int pieceNum = 2;

    // Set initial state: position, velocity, acceleration (PVA)
    Eigen::Matrix3d headState;
    headState.col(0) = Eigen::Vector3d(0.0, 0.0, 0.0);  // Initial position
    headState.col(1) = Eigen::Vector3d(0.0, 0.0, 0.0);  // Initial velocity
    headState.col(2) = Eigen::Vector3d(0.0, 0.0, 0.0);  // Initial acceleration

    // Set final state: position, velocity, acceleration (PVA)
    Eigen::Matrix3d tailState;
    tailState.col(0) = Eigen::Vector3d(5.0, 3.0, 2.0);  // Final position
    tailState.col(1) = Eigen::Vector3d(0.0, 0.0, 0.0);  // Final velocity
    tailState.col(2) = Eigen::Vector3d(0.0, 0.0, 0.0);  // Final acceleration

    // Define intermediate waypoints (for 2 pieces, we need 1 intermediate point)
    Eigen::Matrix3Xd waypoints(3, pieceNum - 1);
    waypoints.col(0) = Eigen::Vector3d(2.5, 2.0, 1.0);

    // Define time allocation for each piece (in seconds)
    Eigen::VectorXd timeAllocation(pieceNum);
    timeAllocation(0) = 2.0;  // First piece duration
    timeAllocation(1) = 2.0;  // Second piece duration

    // Start timing for trajectory generation
    auto start_time = std::chrono::high_resolution_clock::now();

    // Set conditions for MINCO
    minco.setConditions(headState, tailState, pieceNum);

    // Generate trajectory by setting parameters
    minco.setParameters(waypoints, timeAllocation);

    // Get the generated trajectory
    Trajectory<5> trajectory;
    minco.getTrajectory(trajectory);

    // Stop timing
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    double generation_time_ms = duration.count() / 1000.0;

    // Print trajectory generation time
    std::cout << "Trajectory Generation Time: " << generation_time_ms << " ms" << std::endl;
    std::cout << "                            " << duration.count() << " microseconds" << std::endl;
    std::cout << std::endl;

    // Print trajectory information
    std::cout << "Trajectory Information:" << std::endl;
    std::cout << "  Number of pieces: " << trajectory.getPieceNum() << std::endl;
    std::cout << "  Total duration: " << trajectory.getTotalDuration() << " seconds" << std::endl;
    std::cout << std::endl;

    // Print start and end positions
    std::cout << "Start position: [" 
              << trajectory.getPos(0.0).transpose() << "]" << std::endl;
    std::cout << "End position: [" 
              << trajectory.getPos(trajectory.getTotalDuration()).transpose() << "]" << std::endl;
    std::cout << std::endl;

    // Evaluate velocity at different time points
    std::cout << "Velocity Evaluation:" << std::endl;
    double queryTime[] = {0.0, 1.0, 2.0, 3.0, 4.0};
    
    for (int i = 0; i < 5; i++)
    {
        double t = queryTime[i];
        if (t <= trajectory.getTotalDuration())
        {
            Eigen::Vector3d velocity = trajectory.getVel(t);
            std::cout << "  t = " << t << "s, velocity = [" 
                      << velocity.transpose() << "]" << std::endl;
            std::cout << "              velocity norm = " 
                      << velocity.norm() << " m/s" << std::endl;
        }
    }
    std::cout << std::endl;

    // Evaluate position at different time points
    std::cout << "Position Evaluation:" << std::endl;
    for (int i = 0; i < 5; i++)
    {
        double t = queryTime[i];
        if (t <= trajectory.getTotalDuration())
        {
            Eigen::Vector3d position = trajectory.getPos(t);
            std::cout << "  t = " << t << "s, position = [" 
                      << position.transpose() << "]" << std::endl;
        }
    }
    std::cout << std::endl;

    // Evaluate acceleration at specific time
    double testTime = 2.0;
    Eigen::Vector3d acceleration = trajectory.getAcc(testTime);
    std::cout << "Acceleration at t = " << testTime << "s:" << std::endl;
    std::cout << "  acceleration = [" << acceleration.transpose() << "]" << std::endl;
    std::cout << "  acceleration norm = " << acceleration.norm() << " m/s^2" << std::endl;
    std::cout << std::endl;

    // Compute trajectory energy (optimization objective for min-jerk)
    double energy;
    minco.getEnergy(energy);
    std::cout << "Trajectory Energy (jerk integral): " << energy << std::endl;
    std::cout << std::endl;

    // Find maximum velocity and acceleration rates
    std::cout << "Trajectory Limits:" << std::endl;
    std::cout << "  Max velocity rate: " << trajectory.getMaxVelRate() << " m/s" << std::endl;
    std::cout << "  Max acceleration rate: " << trajectory.getMaxAccRate() << " m/s^2" << std::endl;
    std::cout << std::endl;

    std::cout << "========================================" << std::endl;
    std::cout << "  Test completed successfully!" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
