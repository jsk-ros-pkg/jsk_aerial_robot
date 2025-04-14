#pragma once

#include <Eigen/Dense>

namespace aerial_robot_dynamics
{
    void addNoise(Eigen::VectorXd& vec, double sigma)
    {
        std::default_random_engine generator(std::random_device{}());
        std::normal_distribution<double> distribution(0.0, sigma);
        for (int i = 0; i < vec.size(); ++i)
        {
            vec(i) += distribution(generator);
        }
    }
}

