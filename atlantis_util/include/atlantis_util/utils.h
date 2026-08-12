#ifndef ATLANTIS__UTIL_UTILS_H_
#define ATLANTIS__UTIL_UTILS_H_

#include <string>
#include <vector>
#include <Eigen/Geometry>

namespace atlantis {
namespace util {

struct MetricData {
    std::string name;
    double value;
};

double generateRandomValue(double mean, double stddev);

Eigen::Quaterniond rpyToQuaternion(double roll, double pitch, double yaw);

Eigen::Vector3d quaternionToEulerAngles(const Eigen::Quaterniond & q);

std::vector<std::vector<float>> parseVVF(const std::string & input, std::string & error_return);

}
}

#endif