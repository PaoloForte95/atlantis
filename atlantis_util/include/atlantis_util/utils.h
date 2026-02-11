#define ATLANTIS__UTIL_UTILS_H_

#include <string>
#include <Eigen/Geometry>
#include <random>

namespace atlantis {
namespace util {

struct MetricData {
    std::string name;
    double value;
};

struct Waypoint {
    double x;
    double y;
    double theta;
    std::string name;

    bool operator==(const Waypoint& other) const {
        double epsilon = 0.01; // Small tolerance for floating-point comparison
        return std::abs(x - other.x) < epsilon &&
               std::abs(y - other.y) < epsilon &&
               std::abs(theta - other.theta) < epsilon;
    }
};


struct Material {
    std::string name_;
    int id;
    std::map<std::string, double> amounts;


    Material(std::string name){
        name_ = name;
    }

    std::vector<std::string> getLocations(){
        std::vector<std::string> locations;
        //std::cout << "Material " << name_.c_str() << " info"  << std::endl;
        for(auto it = amounts.begin(); it != amounts.end(); ++it) {
            locations.push_back(it->first);
            //std::cout << "Location: " << it->first.c_str() << " ... Amount: " << it->second  << std::endl;
        }
        return locations;
    }

    void addLocation(const std::string& location, double amount) {
        amounts[location] = amount;
    }

    void setAmount(const std::string& location, double amount){
      amounts[location] = amount;
    }

    double getAmount(std::string location){
        auto it = amounts.find(location);
        if (it != amounts.end()) {
          return it->second;
        }
        return 0;
    }
};

double generateRandomValue(double mean, double stddev) {

    // Create a random number generator
    std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    
    // Create a normal (Gaussian) distribution with given mean and standard deviation
    std::normal_distribution<> d(mean, stddev);

    // Generate a random number from the normal distribution
    double random_number = d(gen);

    // Output the random number
    return random_number;
}

Eigen::Quaterniond rpyToQuaternion(double roll, double pitch, double yaw) {
      Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
      Eigen::Quaterniond quaternion = yawAngle * pitchAngle * rollAngle;
    return quaternion;
}

Eigen::Vector3d quaternionToEulerAngles(const Eigen::Quaterniond& q) {
    // Roll (x-axis rotation)
    double roll = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()), 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));

    // Pitch (y-axis rotation)
    double pitch = asin(2.0 * (q.w() * q.y() - q.z() * q.x()));

    // Yaw (z-axis rotation)
    double yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

    return Eigen::Vector3d(roll, pitch, yaw);
}

Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw) {
    Eigen::Quaterniond quaternion;
    quaternion = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                 * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                 * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    return quaternion;
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
      default:  // All other characters should be part of the numbers.
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