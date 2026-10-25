#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImagePublisher: public rclcpp::Node {
public:
    ImagePublisher() : Node("image_source_node") {
        //Определение параметров
        this->declare_parameter<std::string>("source","video");
        this->declare_parameter<std::string>("path","/home/dmitry/Downloads/16198281792142 (1).mp4");
        this->declare_parameter<int>("fps",10);

        //Инициализация публикатора
        publisher =
            this->create_publisher<sensor_msgs::msg::Image>("raw_image",
                                                                    10);
        //Выбор режима
        if(this->get_parameter("source").as_string() == "image") {
            //В этом случае считываем статическое изображение из
            //файла
            image = cv::imread(this->get_parameter("path").as_string());
            if(image.empty()) {
                //Если пустое изображение, то выводим ошибку
                RCLCPP_ERROR(this->get_logger(),
                             "Ошибка при открытии файла изображения");
            }
        }
        else if(this->get_parameter("source").as_string() == "video") {
            //Проверяем путь
            std::string path = this->get_parameter("path").as_string();
            if(isdigit(path[0])) {
                cap.open(std::stoi(path));
            }
            else {
                cap.open(path);
            }
        }
        //Инициализация таймера
        int time_interval = 1000 / this->get_parameter("fps").as_int();
        timer =
            this->create_wall_timer(std::chrono::milliseconds(time_interval),
                                    std::bind(&ImagePublisher::timer_callback,
                                                  this));
    }
private:
    //Публикатор изображения
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
        publisher;
    rclcpp::TimerBase::SharedPtr timer;
    cv::VideoCapture cap; //для видео
    cv::Mat image; // для статического изображения

    void timer_callback() {
        cv::Mat frame;

        if(this->get_parameter("source").as_string() == "video") {
            cap >> frame;
            if(frame.empty()) {
                cap.set(cv::CAP_PROP_POS_FRAMES, 0);
                return;
            }
        }
        else {
            frame = image.clone();
        }

        if(!frame.empty()) {
            sensor_msgs::msg::Image::SharedPtr msg =
                cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).
                                                     toImageMsg();
            msg->header.stamp = this->now();
            msg->header.frame_id = "camera_link";
            publisher->publish(*msg);
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();

    return 0;
}
