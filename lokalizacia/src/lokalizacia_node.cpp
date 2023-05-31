#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

#include "geometry_msgs/Vector3.h"
#include <vector>
#include <numeric>
#include <array>
#include "fstream"

class PoseAverager {
private:
    std::array<std::vector<double>, 3> positions;
    const size_t max_positions = 5;
    ros::Time last_received;

    ros::Subscriber subscriber;
    ros::Publisher publisher;
    ros::NodeHandle n;

    size_t total_inputs = 0;
    size_t correct_inputs = 0;

public:
    PoseAverager() {
        subscriber = n.subscribe("/aruco_single/pose", 1000, &PoseAverager::poseCallback, this);
        publisher = n.advertise<geometry_msgs::Vector3>("average_pose", 1000);
        last_received = ros::Time::now();
    }

    void writeToFile() {
        std::ofstream outfile_x, outfile_z, outfile_percent;

        outfile_x.open("average_x.txt", std::ios_base::app);
        double average_x = computeAverage(positions[0]);
        outfile_x << average_x << ", ";
        outfile_x.close();

        outfile_z.open("average_z.txt", std::ios_base::app);
        double average_z = computeAverage(positions[2]);
        outfile_z << average_z << ", ";
        outfile_z.close();

        outfile_percent.open("detections.txt", std::ios_base::app);
        outfile_percent << "correct: " << correct_inputs << "     total: " << total_inputs << std::endl;
        outfile_percent.close();
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        total_inputs++;

        if (!last_received.isZero() && (ros::Time::now() - last_received).toSec() > 0.2) {
            ROS_WARN_STREAM("Preslo viac ako 200ms od poslednej uspesnej deteckcie. Array pozicii bol resetovany.");
            for(auto& position : positions) {
                position.clear();
            }
        }

        std::array<double, 3> new_positions = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};

        for(size_t i = 0; i < positions.size(); i++) {
            if (!positions[i].empty()) {
                double average = computeAverage(positions[i]);
                double stdev = computeStandardDeviation(positions[i], average);
                if ((std::abs(new_positions[i] - average) >  10.0 * std::max(stdev, 0.8)) || new_positions[2] < 0.0) {
                    ROS_WARN_STREAM("Bola detekovana pozicia, ktora je signifikantne mimo priemeru. Tato pozicia bude ignorovana.");
                    return;
                }
            }
        }

        last_received = ros::Time::now();

        if(positions[0].size() == max_positions) {
            for(auto& position : positions) {
                position.erase(position.begin());
            }
        }

        positions[0].push_back(msg->pose.position.x);
        positions[1].push_back(msg->pose.position.y);
        positions[2].push_back(msg->pose.position.z);

        geometry_msgs::Vector3 avg_pose;
        for(size_t i = 0; i < positions.size(); i++) {
            double average = computeAverage(positions[i]);
            if(i == 0) {
                ROS_INFO_STREAM("Poslednych 5 'x' pozicii: " << printVector(positions[i]) << "\n Priemerne 'x': " << average);
                avg_pose.x = average;
            } else if(i == 1) {
                ROS_INFO_STREAM("Poslednych 5 'y' pozicii: " << printVector(positions[i]) << "\n Priemerne 'y': " << average);
                avg_pose.y = average;
            } else if(i == 2) {
                ROS_INFO_STREAM("Poslednych 5 'z' pozicii: " << printVector(positions[i]) << "\n Priemerne 'z': " << average);
                avg_pose.z = average;
            }
        }

        correct_inputs++;
        publisher.publish(avg_pose);
        writeToFile();
    }

    double computeStandardDeviation(const std::vector<double>& values, double mean) {
        double variance = 0.0;
        for (const auto& value : values) {
            variance += std::pow(value - mean, 2);
        }
        variance /= values.size();
        return std::sqrt(variance);
    }

    std::string printVector(const std::vector<double>& vec) {
        std::stringstream ss;
        for(const auto& v : vec) {
            ss << v << ' ';
        }
        return ss.str();
    }

    double computeAverage(const std::vector<double>& position_values) {
        double sum = std::accumulate(position_values.begin(), position_values.end(), 0.0);
        return position_values.empty() ? 0.0 : sum / position_values.size();
    }

    void spin() {
        ros::spin();
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "avg_lokalizacia");

    PoseAverager poseAverager;
    poseAverager.spin();

    return 0;
}