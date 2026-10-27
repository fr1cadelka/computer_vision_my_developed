#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
class CloudLoader : public rclcpp::Node {
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        publisher;
    rclcpp::TimerBase::SharedPtr timer;
    sensor_msgs::msg::PointCloud2 cloud;
public:
    CloudLoader() : Node ("cloud_loader_node") {
        publisher =
            this->create_publisher<sensor_msgs::msg::PointCloud2>
                    ("/cloud", 2);
        loadCloud();

        timer =
            this->create_wall_timer(std::chrono::milliseconds(2000),
                                std::bind(&CloudLoader::publish_cloud,
                                                  this));

    }
    void loadCloud() {
        //Путь к файлу
        std::string path =
            "/home/dmitry/Downloads/Cloud_1_ds_SOR.ply";
        //Создаём указатель на облако точек
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZI>());
        //Загружаем облако точек из файла
        int result =
            pcl::io::loadPLYFile(path, *cloud);
        if(result == -1) {
            RCLCPP_ERROR(this->get_logger(), "Ошибка при загрузке облака");
            return;
        }
        //Преобразуем загруженное облако в сообщение
        pcl::toROSMsg(*cloud, this->cloud);
        this->cloud.header.frame_id = "cloud";
    }
    void publish_cloud() {
        cloud.header.stamp = this->now();
        publisher->publish(cloud);
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudLoader>());
    rclcpp::shutdown();

    return 0;
}
