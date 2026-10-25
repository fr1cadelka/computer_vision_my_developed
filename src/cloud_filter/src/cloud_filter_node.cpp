#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

class CloudFilter: public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
        subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        publisher;
public:
    CloudFilter() : Node ("cloud_loader_node") {
        publisher =
            this->create_publisher<sensor_msgs::msg::PointCloud2>
            ("/cloud_filtered", 2);

        subscriber =
            this->create_subscription<sensor_msgs::msg::PointCloud2>
                     ("cloud",
                      1,
                      std::bind(&CloudFilter::cloud_callback,
                                this,
                                std::placeholders::_1)
                       );


    }
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

        //Создаём указатель на облако точек
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud
            (new pcl::PointCloud<pcl::PointXYZI>());

        //Преобразуем из сообщения в облако
        pcl::fromROSMsg(*msg, *cloud);
        //Проверяем, чтобы облако точек было не пустое
        if(cloud->empty())
            return;

        //1. Получение доступа к значениям отдельных точек
        //Для примера получим значение первых 5 точек
        for(unsigned int idx = 0; idx < cloud->points.size(); idx++) {
            if(idx < 5) {
                pcl::PointXYZI point = cloud->points.at(idx);
                RCLCPP_INFO_STREAM(this->get_logger(),
                                   "Данные точки "<<idx<<" x: "<<point.x
                                                   <<" y: "<<point.y
                                                   <<" z: "<<point.z
                                                   <<" i: "<<point.intensity);
            }
            else {
                break;
            }
        }
        //2. Воксельный фильтр
        pcl::VoxelGrid<pcl::PointXYZI>::Ptr voxel_filter (
            new pcl::VoxelGrid<pcl::PointXYZI>());
        voxel_filter->setLeafSize(0.1f, 0.1f, 0.1f);
        voxel_filter->setInputCloud(cloud);
        voxel_filter->filter(*cloud);

        //3. Статистический фильтр
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI>::Ptr stat_filter
            (new pcl::StatisticalOutlierRemoval<pcl::PointXYZI>());
        stat_filter->setMeanK(50); //количество ближайших соседей
        stat_filter->setStddevMulThresh(1.0); //множитель при среднем отклонении
        //Играет роль порога для отбрасывания точек
        stat_filter->setInputCloud(cloud);
        stat_filter->filter(*cloud);

        //4. Обратное преобразование в сообщение
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = "cloud";
        cloud_msg.header.stamp = this->now();
        publisher->publish(cloud_msg);
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudFilter>());
    rclcpp::shutdown();

    return 0;
}
