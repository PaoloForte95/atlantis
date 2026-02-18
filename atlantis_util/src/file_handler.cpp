#include "atlantis_util/file_handler.h"
#include <fstream>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace atlantis {
namespace util {

// Function to get all keys from a map
template<typename K, typename V>
std::vector<K> getKeys(const std::map<K, V>& m) {
    std::vector<K> keys;
    for (const auto& kv : m) {
        keys.push_back(kv.first);
    }
    return keys;
}


// Function to save metrics to a CSV file
void saveMetricsToCSV(const std::string& filename, 
                      const std::vector<std::string>& columns,
                      const std::map<std::string, std::vector<MetricData>>& data) {
    std::ofstream file;
    file.open(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    // Get the algorithms from the keys of the data map
    std::vector<std::string> algorithms = getKeys(data);

    // Write the first row (metrics)
    file << ",";
    for (const auto& metric : columns) {
        file << metric << ",";
    }
    file << "\n";

    // Write each algorithm and its corresponding data
    for (const auto& alg : algorithms) {
        file << alg << ",";
        for (const auto& metric : columns) {
            auto it = data.find(alg);
            if (it != data.end()) {
                const auto& metricData = it->second;
                auto metricIt = std::find_if(metricData.begin(), metricData.end(), 
                                             [&metric](const MetricData& md) { return md.name == metric; });
                if (metricIt != metricData.end() ) {
                  if(metricIt->value != -1){
                      file << metricIt->value;
                  }
                }
            }
            file << ",";
        }
        file << "\n";
    }

    file.close();
    std::cout << "Metrics saved to " << filename << std::endl;
}




void appendRowToCSV(const std::string& filename, 
                    const std::string& algorithm, 
                    const std::vector<atlantis::util::MetricData>& metricData,
                    const std::vector<std::string>& metrics) {
    std::ofstream file(filename, std::ios_base::app);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    // Append the new algorithm and its metrics
    file << algorithm << ",";
    for (const auto& metric : metrics) {
        auto metricIt = std::find_if(metricData.begin(), metricData.end(),
                                     [&metric](const atlantis::util::MetricData& md) { return md.name == metric; });
        if (metricIt != metricData.end()) {
            file << metricIt->value;
        }
        file << ",";
    }
    file << "\n";

    file.close();
    std::cout << "New metric appended to " << filename << std::endl;
}



std::vector<Waypoint> parseWaypoints(std::string file_path) {
    std::vector<Waypoint> waypoints;
    std::ifstream file(file_path);
    
    if (!file.is_open()) {
        std::cerr << "Could not open file " << file_path << std::endl;
        return waypoints;
    }
    
    std::cout << "Loading waypoints from file " << file_path << std::endl;
    
    std::string line;
    Waypoint waypoint;
    bool readingWaypoint = false;
    
    while (std::getline(file, line)) {
        // Remove leading/trailing spaces
        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t") + 1);
        
        if (line.empty()) continue;
        
        if (line.back() == ':') { // Start of a new waypoint
            if (readingWaypoint) {
                waypoints.push_back(waypoint);
            }
            waypoint = Waypoint(); // Reset waypoint struct
            waypoint.name = line.substr(0, line.size() - 1);
            readingWaypoint = true;
        } else if (readingWaypoint) {
            std::istringstream iss(line);
            std::string key;
            double value;
            
            if (std::getline(iss, key, ':')) {
                key.erase(0, key.find_first_not_of(" \t"));
                key.erase(key.find_last_not_of(" \t") + 1);
                
                if (iss >> value) {
                    if (key == "x") waypoint.x = value;
                    else if (key == "y") waypoint.y = value;
                    //else if (key == "z") waypoint.z = value;
                    else if (key == "yaw") waypoint.theta = value;
                }
            }
        }
    }
    
    if (readingWaypoint) {
        waypoints.push_back(waypoint);
    }
    
    file.close();
    
    for (const auto& wp : waypoints) {
        std::cout << "Loaded waypoint " << wp.name << " : (" 
                  << wp.x << ", " << wp.y << ", " << wp.theta << ")" << std::endl;
    }
    
    return waypoints;
}


std::string resolve_pkg_uri(const std::string& uri)
{
    const std::string prefix = "package://";
    if (uri.rfind(prefix, 0) != 0)
        return uri;

    std::string rest = uri.substr(prefix.size());
    auto pos = rest.find('/');
    std::string pkg = rest.substr(0, pos);
    std::string rel = (pos == std::string::npos) ? "" : rest.substr(pos + 1);

    std::string share = ament_index_cpp::get_package_share_directory(pkg);
    return share + "/" + rel;
}

// std::vector<Waypoint> parseWaypoints(std::string file_path){
//   std::vector<Waypoint> waypoints;
//     std::ifstream file(file_path);
//     if (!file.is_open()) {
//       std::cerr << "Could not open file " << file_path.c_str() << std::endl;
//       return waypoints;
//     }
//     std::cout << "Loading waypoints from file " << file_path.c_str() << std::endl;
//     std::string line;
//     // Parse each line for waypoints in the format "name: (x, y, theta)"
//     while (std::getline(file, line)) {
//       std::string name, point;
//       std::string delimiter = ":";
//       name = line.substr(0,line.find(delimiter));
//       name.erase(std::remove_if(name.begin(), name.end(), ::isspace), name.end());
//       point = line.substr(line.find(delimiter)+1);
//       point.erase(std::remove_if(point.begin(), point.end(), [](char c) {
//         return c == '(' || c == ')' || std::isspace(c);
//       }), point.end());

//       double components[3];
//       size_t pos = 0;
//       for (int i = 0; i < 3; ++i) {
//         // Find the position of the comma from the current 'pos'
//         size_t commaPos = point.find(',', pos);

//         if (commaPos != std::string::npos) {
//             std::string value = point.substr(pos, commaPos - pos);
//             components[i] = std::stod(value);
//             pos = commaPos + 1; // Move 'pos' to the character after the comma
//         } else {
//             // For the last component (theta), use the remaining string
//             std::string value = point.substr(pos);
//             components[i] = std::stod(value);
//         }
//     }
//       Waypoint waypoint;
//       waypoint.x = components[0];
//       waypoint.y = components[1];
//       waypoint.theta = components[2];
//       waypoint.name = name;
//       waypoints.push_back(waypoint); 
//       std::cout << "Loaded waypoint " << name.c_str() << " :(" << components[0] << ", " << components[1] << ", " << components[2] << ")" <<  std::endl;
//     }
//     // Close the input file
//     file.close();
//     return waypoints;
// }
}
}