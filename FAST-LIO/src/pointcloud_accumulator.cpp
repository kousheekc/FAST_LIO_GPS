#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>

#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/SetBool.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/distances.h>

#include <chrono>
#include <iostream>
#include <boost/filesystem.hpp>

class PointCloudCollector
{
public:
    PointCloudCollector(const ros::NodeHandle &nh)
        : nh_(nh), tfListener_(tfBuffer_)
    {
        // Get parameters from the parameter server
        nh_.param<std::string>("/save_directory", save_directory_, "");
        nh_.param<std::string>("/start_time", start_time_, "");
        nh_.param<std::string>("fixed_frame", fixed_frame_, "world");
        nh_.param("voxel_size", voxel_size_, 0.01); // voxel filter leaf size (pointcloud resolution)
        nh_.param("max_intensity", max_intensity_, 500.0);
        nh_.param("publish_rate", publish_rate_, 1.0);             // in Hz
        nh_.param("auto_start_record", auto_start_record_, false); // in Hz
        nh_.param("min_fitness_score", min_fitness_score_, 0.2);   // minimum fitness score required to add a cloud to full_cloud_
        nh_.param("max_overlap", max_overlap_, 0.5);               // max distance (overlap) between new cloud and full_cloud_ to check icp fitness

        // Advertise the PointCloud2 topic for publishing the accumulated cloud
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/accumulated_cloud", 1, true);
        metrics_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("metrics", 1, true);

        // Subscribe to the PointCloud2 topic
        sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/input_cloud", 100, &PointCloudCollector::pc_callback, this);

        // Define the service to start and stop collecting point clouds
        service_ = nh_.advertiseService("record_pointcloud", &PointCloudCollector::record_pointcloud, this);

        // Create a client to this same service
        client_ = nh_.serviceClient<std_srvs::SetBool>("record_pointcloud");

        is_collecting_ = false;
        publish_counter_ = 0;

        // Setup voxel filter resolution
        voxel_filter_.setLeafSize(voxel_size_, voxel_size_, voxel_size_);

        
        if (auto_start_record_)
        {
            // Call the service to start recording point clouds
            std_srvs::SetBool record_cloud;
            record_cloud.request.data = true;
            record_pointcloud(record_cloud.request, record_cloud.response);
            ROS_INFO("%s", record_cloud.response.message.c_str());
        }
    }

    void spin()
    {
        ros::Rate rate(publish_rate_);
        while (ros::ok())
        {
            if (is_collecting_)
                publish_cloud();

            ros::spinOnce();
            rate.sleep();
        }

        // Save the final point cloud to a file
        if (is_collecting_)
            save_cloud("accumulated_pointcloud.pcd");
    }

private:
    ros::NodeHandle nh_;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_list_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr full_cloud_;
    ros::Publisher cloud_pub_, metrics_pub_;
    ros::Subscriber sub_;
    ros::ServiceServer service_;
    ros::ServiceClient client_;
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter_;

    tf2_ros::TransformListener tfListener_;
    tf2_ros::Buffer tfBuffer_;

    // Parameters
    std::string save_directory_, start_time_, fixed_frame_;
    double voxel_size_, max_intensity_;
    double publish_rate_;
    bool is_collecting_, auto_start_record_;
    double min_fitness_score_, max_overlap_;

    // Variables
    int publish_counter_, new_cloud_counter_;

    void pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        if (!is_collecting_)
            return;

        // Look up the transform from the sensor frame to the world frame
        geometry_msgs::TransformStamped transform;
        try
        {
            transform = tfBuffer_.lookupTransform(fixed_frame_, msg->header.frame_id, msg->header.stamp);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN_STREAM("Failed to transform point cloud: " << ex.what());
            return;
        }

        // Transform the point cloud to the world frame
        sensor_msgs::PointCloud2 transformed_cloud;
        pcl_ros::transformPointCloud(fixed_frame_, transform.transform, *msg, transformed_cloud);

        // Convert the PointCloud2 message to a pcl::PointCloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(transformed_cloud, *cloud);

        // Add the new point cloud to the list
        cloud_list_.push_back(cloud);

        ROS_DEBUG("Cloud list size: %ld", cloud_list_.size());
    }

    void publish_cloud()
    {
        // Record the start time
        auto start_time = std::chrono::high_resolution_clock::now();

        new_cloud_counter_ += cloud_list_.size();
        for (const auto &cloud : cloud_list_)
        {
            *full_cloud_ += *cloud;
        }

        // Clear the list of point clouds
        cloud_list_.clear();
        ROS_DEBUG("Clear list");

        // Iterate through each point in the cloud and modify its intensity
        for (pcl::PointCloud<pcl::PointXYZI>::iterator it = full_cloud_->begin(); it != full_cloud_->end(); ++it)
        {
            if (it->intensity > max_intensity_)
            {
                it->intensity = max_intensity_;
            }
        }

        // Filter cloud every 10 new scans
        if (new_cloud_counter_ > 10)
        {
            filter_cloud(full_cloud_);
            new_cloud_counter_ = 0;
        }
        publish_counter_++;

        // Convert the filtered point cloud to a PointCloud2 message and publish it
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*full_cloud_, output);
        output.header.frame_id = fixed_frame_; // Adjust the frame ID as needed
        cloud_pub_.publish(output);

        // Record the end time and calculate the elapsed time
        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        // Print the elapsed time to the console
        ROS_DEBUG("Published point cloud in %ld ms", elapsed_time.count());
    }

    bool record_pointcloud(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        // Process the service request

        is_collecting_ = req.data;

        if (is_collecting_)
        {
            res.message = "Started collecting point clouds";
            res.success = true;

            // Init fullcloud variable
            full_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        }
        else
        {
            res.message = "Stopped collecting point clouds, accumulated pointcloud is saved to " + save_directory_ + "/" + start_time_;
            save_cloud("accumulated_pointcloud.pcd");
            res.success = true;

            // Publish last cloud with all the filters, this will stay as topic is latched
            publish_cloud();
        }
        return true;
    }

    void save_cloud(const std::string &file_name)
    {
        // Concatenate all the point clouds in the list
        for (const auto &cloud : cloud_list_)
            *full_cloud_ += *cloud;

        filter_cloud(full_cloud_);

        // Iterate through each point in the cloud and modify its intensity
        for (pcl::PointCloud<pcl::PointXYZI>::iterator it = full_cloud_->begin(); it != full_cloud_->end(); ++it)
        {
            if (it->intensity > max_intensity_)
            {
                it->intensity = max_intensity_;
            }
        }

        // Apply an outlier removal filter
        // pcl::StatisticalOutlierRemoval<pcl::PointXYZI> SOR_filter;
        // SOR_filter.setInputCloud(full_cloud_);
        // SOR_filter.setMeanK(50);            // Set the number of neighbors to use for mean distance calculation
        // SOR_filter.setStddevMulThresh(2.0); // Set the standard deviation multiplier for the distance threshold
        // SOR_filter.filter(*full_cloud_);

        // Save the filtered point cloud to a .pcd file
        std::string file_path = save_directory_ + "/" + start_time_ + "/" + file_name;
        boost::filesystem::path directory_path = boost::filesystem::path(file_path).parent_path();

        // Check if the directory exists
        if (!(boost::filesystem::exists(directory_path) && boost::filesystem::is_directory(directory_path)))
        {
            // Create the directory
            if (!boost::filesystem::create_directory(directory_path))
            {
                ROS_ERROR("Failed to create directory: %s", directory_path.c_str());
            }
        }
        pcl::io::savePCDFile(file_path, *full_cloud_);
    }

    void filter_cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        // Record the start time
        auto start_time = std::chrono::high_resolution_clock::now();

        // Apply filters to the pointcloud passed through the pointer

        // Apply a voxel grid filter to the full point cloud
        voxel_filter_.setInputCloud(cloud);
        voxel_filter_.filter(*cloud);

        // Record the end time and calculate the elapsed time
        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        // Print the elapsed time to the console
        ROS_DEBUG("Filtered point cloud in %ld ms", elapsed_time.count());
    }

    float compute_overlap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &target_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr &source_cloud)
    {
        // Compute overlap between 2 clouds with the mean nearest neighbor distance for each point of the cloud
        // A low score means that there is a high overlap. The result is a mean distance in meters.

        // Record the start time
        auto start_time = std::chrono::high_resolution_clock::now();

        pcl::search::KdTree<pcl::PointXYZI> kdtree;
        kdtree.setInputCloud(target_cloud);

        float total_distance = 0.0;
        int num_distances = 0;

        for (const auto &point : source_cloud->points)
        {
            std::vector<int> indices;
            std::vector<float> distances;
            kdtree.nearestKSearch(point, 1, indices, distances);
            total_distance += distances[0];
            num_distances++;
        }

        // Record the end time and calculate the elapsed time
        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        // Print the elapsed time to the console
        ROS_INFO("Compute overlap in %ld ms", elapsed_time.count());

        if (num_distances > 0)
        {
            return total_distance / num_distances; // return the mean nearest distance
        }
        else
        {
            return 0.0; // return 0 if source_cloud is empty
        }
    }

    float compute_fitness(const pcl::PointCloud<pcl::PointXYZI>::Ptr &target_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr &source_cloud)
    {
        // Compute icp fitness between 2 clouds (how well the source_cloud fits to the target_cloud)
        // A low score means that there is a high fitness

        // Record the start time
        auto start_time = std::chrono::high_resolution_clock::now();

        // create a correspondence estimation object
        pcl::registration::CorrespondenceEstimation<pcl::PointXYZI, pcl::PointXYZI> ce;

        // set the input point clouds
        ce.setInputSource(source_cloud);
        ce.setInputTarget(target_cloud);

        // compute the correspondences between the two clouds
        pcl::Correspondences correspondences;
        ce.determineCorrespondences(correspondences);

        // compute the fitness score between the clouds
        double fitness_score = 0.0;
        for (pcl::Correspondences::const_iterator it = correspondences.begin(); it != correspondences.end(); ++it)
        {
            // compute the squared distance between the corresponding points
            double distance = pcl::squaredEuclideanDistance(source_cloud->points[it->index_query], target_cloud->points[it->index_match]);
            fitness_score += distance;
        }

        // Record the end time and calculate the elapsed time
        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        // Print the elapsed time to the console
        ROS_INFO("Compute fitness in %ld ms", elapsed_time.count());

        return fitness_score;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_collector");
    ros::NodeHandle nh("~");

    // Initialize the point cloud collector
    PointCloudCollector collector(nh);

    // Spin until interrupted
    collector.spin();

    return 0;
}