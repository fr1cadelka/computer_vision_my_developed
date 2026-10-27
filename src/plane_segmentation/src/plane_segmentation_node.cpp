#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

class PlaneDetector : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_inliers;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_marker;

    double x_min, x_max, y_min, y_max, z_min, z_max, ransac_threshold;

public:
    PlaneDetector() : Node("plane_segmentation_node") {
        // 1. Читаем параметры из launch-файла
        x_min = this->declare_parameter("x_min", -15.0);
        x_max = this->declare_parameter("x_max",  15.0);
        y_min = this->declare_parameter("y_min", -10.0);
        y_max = this->declare_parameter("y_max",  10.0);
        z_min = this->declare_parameter("z_min", -2.0);
        z_max = this->declare_parameter("z_max",  2.0);
        ransac_threshold = this->declare_parameter("ransac_threshold", 0.1);

        RCLCPP_INFO(this->get_logger(), "Параметры обрезки: X[%.1f, %.1f] Y[%.1f, %.1f] Z[%.1f, %.1f]",
                    x_min, x_max, y_min, y_max, z_min, z_max);

        // 2. Подписки и публикации
        subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_filtered", 10, std::bind(&PlaneDetector::cloud_callback, this, std::placeholders::_1));
        publisher_inliers = this->create_publisher<sensor_msgs::msg::PointCloud2>("/plane_inliers", 10);
        publisher_marker = this->create_publisher<visualization_msgs::msg::Marker>("/detected_plane", 10);
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) return;

        // ПУНКТ 2: PassThrough (Обрезка)
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PassThrough<pcl::PointXYZI> pass;

        pass.setInputCloud(cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(x_min, x_max);
        pass.filter(*cloud_filtered);

        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(y_min, y_max);
        pass.filter(*cloud_filtered);

        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(z_min, z_max);
        pass.filter(*cloud_filtered);

        // ДЕБАГ: Выводим размер облака ДО и ПОСЛЕ фильтра
        RCLCPP_INFO(this->get_logger(), "Точек до обрезки: %ld, после обрезки: %ld",
                    cloud->size(), cloud_filtered->size());

        if (cloud_filtered->empty()) {
            RCLCPP_WARN(this->get_logger(), "После обрезки не осталось точек! Проверь параметры!");
            return;
        }

        // ПУНКТ 3: RANSAC
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(ransac_threshold);
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            RCLCPP_WARN(this->get_logger(), "RANSAC не смог найти плоскость.");
            return;
        }

        // ПУНКТ 5: Извлечение и покраска (Зеленые точки)
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZI>());
        extract.filter(*cloud_inliers);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::copyPointCloud(*cloud_inliers, *cloud_colored);
        for (auto& pt : cloud_colored->points) {
            pt.r = 0; pt.g = 255; pt.b = 0;
        }

        sensor_msgs::msg::PointCloud2 inliers_msg;
        pcl::toROSMsg(*cloud_colored, inliers_msg);
        inliers_msg.header.frame_id = msg->header.frame_id;
        inliers_msg.header.stamp = this->now();
        publisher_inliers->publish(inliers_msg);

        // ПУНКТ 4: Маркер (Центр плоскости)
        double cx = 0, cy = 0, cz = 0;
        for (const auto& pt : cloud_inliers->points) {
            cx += pt.x; cy += pt.y; cz += pt.z;
        }
        cx /= cloud_inliers->size(); cy /= cloud_inliers->size(); cz /= cloud_inliers->size();

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = msg->header.frame_id;
        marker.header.stamp = this->now();
        marker.ns = "plane";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = cx;
        marker.pose.position.y = cy;
        marker.pose.position.z = cz;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 5.0;
        marker.scale.y = 5.0;
        marker.scale.z = 0.05;

        marker.color.a = 0.5;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        publisher_marker->publish(marker);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlaneDetector>());
    rclcpp::shutdown();
    return 0;
}
