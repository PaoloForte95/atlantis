#include "atlantis_util/utils.h"

#include <random>
#include <sstream>
#include <cmath>

namespace atlantis {
namespace util {

double generateRandomValue(double mean, double stddev)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<> d(mean, stddev);
  return d(gen);
}

Eigen::Quaterniond rpyToQuaternion(double roll, double pitch, double yaw)
{
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
  return yawAngle * pitchAngle * rollAngle;
}

Eigen::Vector3d quaternionToEulerAngles(const Eigen::Quaterniond & q)
{
  double roll = std::atan2(
    2.0 * (q.w() * q.x() + q.y() * q.z()),
    1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));

  double pitch = std::asin(2.0 * (q.w() * q.y() - q.z() * q.x()));

  double yaw = std::atan2(
    2.0 * (q.w() * q.z() + q.x() * q.y()),
    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

  return Eigen::Vector3d(roll, pitch, yaw);
}

std::vector<std::vector<float>> parseVVF(const std::string & input, std::string & error_return)
{
  std::vector<std::vector<float>> result;

  std::stringstream input_ss(input);
  int depth = 0;
  std::vector<float> current_vector;
  while (!!input_ss && !input_ss.eof()) {
    switch (input_ss.peek()) {
      case EOF:
        break;
      case '[':
        depth++;
        if (depth > 2) {
          error_return = "Array depth greater than 2";
          return result;
        }
        input_ss.get();
        current_vector.clear();
        break;
      case ']':
        depth--;
        if (depth < 0) {
          error_return = "More close ] than open [";
          return result;
        }
        input_ss.get();
        if (depth == 1) {
          result.push_back(current_vector);
        }
        break;
      case ',':
      case ' ':
      case '\t':
        input_ss.get();
        break;
      default:
        if (depth != 2) {
          std::stringstream err_ss;
          err_ss << "Numbers at depth other than 2. Char was '" << char(input_ss.peek()) << "'.";
          error_return = err_ss.str();
          return result;
        }
        float value;
        input_ss >> value;
        if (!!input_ss) {
          current_vector.push_back(value);
        }
        break;
    }
  }

  if (depth != 0) {
    error_return = "Unterminated vector string.";
  } else {
    error_return = "";
  }

  return result;
}




}
}