#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <random>

class PlaneDetector : public rclcpp::Node {
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        publisher_cloud; //публикатор облака
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
        publisher_marker; //публикатор маркера плоскости
    rclcpp::TimerBase::SharedPtr timer;
public:
    PlaneDetector() : Node ("cloud_loader_node") {
        publisher_cloud =
            this->create_publisher<sensor_msgs::msg::PointCloud2>
            ("/cloud", 2);

        publisher_marker =
            this->create_publisher<visualization_msgs::msg::Marker>
            ("/detected_plane", 2);

        timer =
            this->create_wall_timer(std::chrono::milliseconds(2000),
                                    std::bind(&PlaneDetector::publish_cloud,
                                              this));

    }

    void publish_cloud() {
        //1. Создание искусственного облака точек для теста
        //Облако
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZI>);
        //Генератор случайных чисел
        std::default_random_engine generator;
        std::uniform_real_distribution<float> distribution(-1.0, 1.0);
        std::normal_distribution<float> noise(0.0, 0.05);

        //Генеририруем случайные точки в рамках плоскости
        for(int i =0; i < 500; i++) {
            pcl::PointXYZI p;
            p.x = distribution(generator);
            p.y = distribution(generator);
            p.z = 0.5 + noise(generator);
            p.intensity = 1.0;
            cloud->push_back(p);
        }
        //Генерируем точки вне плоскости
        for(int i =0; i < 100; i++) {
            pcl::PointXYZI p;
            p.x = distribution(generator);
            p.y = distribution(generator);
            p.z = distribution(generator);
            p.intensity = 0.5;
            cloud->push_back(p);
        }

        //2. Поиск плоскости
        //Коэффициенты модели
        pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
        //Инлаеры
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        //Алгоритм сегментации
        pcl::SACSegmentation<pcl::PointXYZI> seg;

        //Устанавлиаем параметры алгоритма
        //Флаг оптимизации коэффициентов
        seg.setOptimizeCoefficients(true);
        //Тип модели - плоскость
        seg.setModelType(pcl::SACMODEL_PLANE);

        //Метод
        seg.setMethodType(pcl::SAC_RANSAC);
        //Пороговое расстояние от модели
        seg.setDistanceThreshold(0.1);
        //Подаём облако на вход
        seg.setInputCloud(cloud);
        //Получаем результат
        seg.segment(*inliers, *coeff);

        //Проверяем, удалось ли найти плоскость
        if(inliers->indices.empty()) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "плоскость не найдена");
        }

        //3. Публикуем облако точек
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = "cloud";

        cloud_msg.header.stamp = this->now();
        publisher_cloud->publish(cloud_msg);

        //4. Визуализация распознанной плоскости с помощью маркера
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "cloud";
        marker.header.stamp = this->now();
        //пространство имён маркера
        marker.ns = "plane";
        //идентификатор маркера
        marker.id = 0;
        //тип маркера
        marker.type = visualization_msgs::msg::Marker::CUBE;
        //метод добавления маркера
        marker.action = visualization_msgs::msg::Marker::ADD;

        //Параметры расположения маркера
        //Для упрощения (поскольку плоскость параллельна XOY)
        //зададим только положение по Z и без изменения ориентации
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        //Алгоритм распознавания плоскости вернёт коэффициенты
        //плоскости A, B, C, D
        marker.pose.position.z = -coeff->values[3] /
                                 coeff->values[2];

        marker.pose.orientation.w = 1.0; //необходимо вычислить
        //кватернион ориентации по параметрам модели распознанной плоскости

        //Устанавливаем параметры модели
        marker.scale.x = 2.0;
        marker.scale.y = 2.0;
        marker.scale.z = 0.01;

        marker.color.a = 1.0; //прозрачность
        marker.color.r = 0.0; //диапазон значений от 0 до 1
        marker.color.g = 1.0;
        marker.color.b = 0.5;

        publisher_marker->publish(marker);

    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlaneDetector>());
    rclcpp::shutdown();

    return 0;
}

//Задание
//1. Создать пакет, узел, который принимает отфильтрованное облако точек
//2. На облаке выделить область точек путём установки порогов по 3
//кооодинатам, которая соответствует заданной реальной поверхности
//(стена или земля, будет указано в таблице по вариантам)
//3. Полученное подоблако точек подать на вход алгоритма RANSAC по
//сегментации данных и получить модель плоскости
//4. Согласно полученной модели сформировать маркер и опубликовать его
//5. Точки, на основе которых получена модель также опубликовать
//с отличительным цветом
//Продемонстрировать результат в rviz

