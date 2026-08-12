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

}
}