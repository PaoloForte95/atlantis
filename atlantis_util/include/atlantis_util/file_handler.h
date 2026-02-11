#ifndef ATLANTIS__UTIL_FILE_HANDLER_H_
#define ATLANTIS__UTIL_FILE_HANDLER_H_


#include <iostream>
#include <vector>
#include <map>
#include "atlantis_util/utils.h"

namespace atlantis {
namespace util {

// Function to get all keys from a map
template<typename K, typename V>std::vector<K> getKeys(const std::map<K, V>& m);


// Function to save metrics to a CSV file
void saveMetricsToCSV(const std::string& filename, 
                      const std::vector<std::string>& columns,
                      const std::map<std::string, std::vector<MetricData>>& data);

// Function to save a metric to a CSV file
void appendRowToCSV(const std::string& filename, 
                    const std::string& algorithm, 
                    const std::vector<atlantis::util::MetricData>& metricData,
                    const std::vector<std::string>& metrics);


std::vector<Waypoint> parseWaypoints(std::string file_path);

}
}


#endif