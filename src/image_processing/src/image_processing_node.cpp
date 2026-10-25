#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageProcessor: public rclcpp::Node {
public:
    ImageProcessor() : Node("image_processor_node") {
        //Храним только последний кадр
        auto QoS = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

        subscriber =
            this->create_subscription<sensor_msgs::msg::Image>("raw_image",
                                                               QoS,
                                                               std::bind(&ImageProcessor::image_callback,this,std::placeholders::_1));

        //Инициализация публикатора
        publisher =
            this->create_publisher<sensor_msgs::msg::Image>("processed_image",
                                                            QoS);

    }
private:
    //Публикатор изображения
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
        publisher;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {


        cv::Mat result, hsv,frame,yelow_mask;

        frame=cv_bridge::toCvCopy(msg,"bgr8")->image;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        //Два порога - нижний и верхний
        cv::Scalar lower = cv::Scalar(20, 100, 100);
        cv::Scalar upper = cv::Scalar(35, 255, 255);

        //Фильтруем
         cv::inRange(hsv,lower,upper,yelow_mask);

        cv::Mat  kernel_filter= cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5));

        cv::morphologyEx(yelow_mask,yelow_mask,cv::MORPH_OPEN,kernel_filter);

        cv::morphologyEx(yelow_mask,yelow_mask,cv::MORPH_CLOSE,kernel_filter);


        std::vector<std::vector<cv::Point>> countours;

        cv::findContours(yelow_mask,countours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);


        double max_area=0;

        int max_countours_index=-1;

        for(size_t i=0; i<countours.size();i++){

            double area = cv::contourArea(countours[i]);

            if(area>max_area){

                max_area=area;

                max_countours_index=i;
            }


        }

        if(max_countours_index!=-1 && max_area>1000){


            cv::Rect rect=cv::boundingRect(countours[max_countours_index]);

            cv::rectangle(frame,rect,cv::Scalar(255,0,255),3);

        }


        sensor_msgs::msg::Image::SharedPtr out_msg=
            cv_bridge::CvImage(msg->header,"bgr8",frame).toImageMsg();

        publisher->publish(*out_msg);



        // //1. Преобразование в cv::Mat
        // cv::Mat input_frame =
        //     cv_bridge::toCvCopy(msg, "bgr8")->image;

        // //2. Обработка изображения
        // cv::Mat output_frame = process_image_contours(input_frame);

        // //3. Отправка изображения в топик
        // //Определяем тип кодировки
        // std::string encoding;
        // if(output_frame.channels() == 1) {
        // }
        // else {
        //     encoding = "bgr8";
        // }
        // //Формируем сообщение
        // sensor_msgs::msg::Image::SharedPtr out_msg =
        //     cv_bridge::CvImage(msg->header,encoding,output_frame).toImageMsg();
        // //Публикуем
        // publisher->publish(*out_msg);

    }
    //Преобразование цветовых пространств
    // cv::Mat process_image_color(cv::Mat &frame) {
    //     cv::Mat result;
    //     //В HSV
    //     // cv::cvtColor(frame,result,cv::COLOR_BGR2HSV);
    //     //Grayscale
    //     cv::cvtColor(frame,result,cv::COLOR_BGR2GRAY);
    //     return result;
    // }
    // //Фильтрация
    // cv::Mat process_image_filter(cv::Mat &frame) {
    //     cv::Mat result;
    //     //Размер ядра
    //     int kernel_size = 15;
    //     //Размытие по Гауссу
    //     // cv::GaussianBlur(frame, //источник
    //     //                  result, //результат
    //     //                  cv::Size(kernel_size,kernel_size), //размер ядра
    //     //                  0); //сигма функции

    //     //Медианный фильтр
    //     // cv::medianBlur(frame,result,kernel_size);
    //     //Билатеральный фильтр
    //     cv::bilateralFilter(frame,
    //                         result,
    //                         9, //Размер
    //                         75, //отклонение по цвету
    //                         75); //отклонение по расстоянию
    //     return result;
    // }
    //Пороги и бинаризация
    // cv::Mat process_image_threshold(cv::Mat &frame) {
    //     cv::Mat result, gray;
    //     cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    //     //Порог
    //     int threshold = 100;
    //     //Простой с указанием порога
    //     // cv::threshold(gray,result,
    //     //               threshold, //порог
    //     //               255, //максимальное значение пикселей
    //     //               cv::THRESH_BINARY); //режим

    //     //Метод Оцу - бинаризация с адаптивным порогом
    //     cv::threshold(gray,result,
    //                   0,
    //                   255,
    //                   cv::THRESH_BINARY | cv::THRESH_OTSU);

    //     return result;
    // }
    // cv::Mat process_image_threshold_color(cv::Mat &frame) {
    //     cv::Mat result, hsv;
    //     cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    //     //Два порога - нижний и верхний
    //     cv::Scalar lower = cv::Scalar(0, 150, 50);
    //     cv::Scalar upper = cv::Scalar(20, 255, 255);

    //     //Фильтруем
    //     // cv::inRange(hsv,lower,upper,result);

    //     //Для красных цветов
    //     cv::Mat mask1, mask2;
    //     cv::inRange(hsv,lower,upper,mask1);

    //     lower = cv::Scalar(160, 150, 50);
    //     upper = cv::Scalar(180, 255, 255);

    //     cv::inRange(hsv,lower,upper,mask2);

    //     result = mask1 + mask2;

    //     return result;
    // }

    // cv::Mat process_image_morph(cv::Mat &frame) {
    //     cv::Mat result;

    //     //Ядро преобразования
    //     cv::Mat kernel =
    //         cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
    //     //Дилатация - расширение светлых областей
    //     // cv::morphologyEx(frame,result,cv::MORPH_DILATE,kernel);
    //     //Эрозия - расширение тёмных областей
    //     // cv::morphologyEx(frame,result,cv::MORPH_ERODE,kernel);
    //     //Открытие - это дилатация после эрозии - хорошо удаляет шум
    //     // cv::morphologyEx(frame,result,cv::MORPH_OPEN,kernel);
    //     //Закрытие - это эрозия после дилатации - удаляет малые отверстия внутри
    //     //объектов, полезна при выделении объектов
    //     cv::morphologyEx(frame,result,cv::MORPH_CLOSE,kernel);

    //     return result;
    // }
    // cv::Mat process_image_contours(cv::Mat &frame) {
    //     cv::Mat result;

    //     //Подготовка изображения
    //     cv::Mat gray;
    //     cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    //     cv::GaussianBlur(gray, gray, cv::Size(5,5),0);

    //     //Выделение границ
    //     cv::Mat edges;
    //     cv::Canny(gray, edges, 50, 100);

    //     //Поиск контуров
    //     std::vector<std::vector<cv::Point>> contours; //точки контура
    //     std::vector<cv::Vec4i> hierarchy; //иерархия (вложенность) контуров
    //     cv::findContours(edges, //источник
    //                      contours, //результат с контурами
    //                      hierarchy, //иерархия
    //                      cv::RETR_TREE, //метод обнаружения контуров  (внешние и вложенные)
    //                      // cv::RETR_EXTERNAL, //метод обнаружения контуров (только внешние)
    //                      cv::CHAIN_APPROX_SIMPLE); //метод соединения и замыкания констуров
    //     //Анализ обнаруженных контуров
    //     result = frame.clone();
    //     for(unsigned int i = 0; i < contours.size(); i++) {
    //         //Анализируем по площади
    //         double area = cv::contourArea(contours[i]);
    //         //Отбрасываем мелкие контуры
    //         if(area > 1000) {
    //             cv::drawContours(result, //где рисуем
    //                              contours, //вектор с контурами
    //                              i, //номер контура
    //                              cv::Scalar(0,255,0), //цвет
    //                              3); //толщина
    //             //Определение центра масс
    //             cv::Moments m = cv::moments(contours[i]);
    //             int cx = m.m10 / m.m00;
    //             int cy = m.m01 / m.m00;
    //             //Для отладки выводим точку центра и текст с данными площади
    //             cv::circle(result, //где рисуем
    //                        cv::Point(cx,cy), //точка центра
    //                        5, //радиус
    //                        cv::Scalar(0,255,0), //цвет
    //                        -1); //толщина (заливка)
    //             cv::putText(result, //где рисуем
    //                         "Area: " + std::to_string((int)area), //текст
    //                         cv::Point(cx + 10, cy), //координаты текста
    //                         cv::FONT_HERSHEY_SIMPLEX, //шрифт
    //                         1, //размер шрифта
    //                         cv::Scalar(0,255,0), //цвет
    //                         2); //толщина

    //             //Аппроксимация контура
    //             double perimeter = cv::arcLength(contours[i], true);
    //             //Параметр удаления аппроксимирующего контура
    //             double epsilon = 0.01 * perimeter;
    //             std::vector<cv::Point> approx;

    //             cv::approxPolyDP(contours[i],
    //                              approx,
    //                              epsilon,
    //                              true);
    //             //Рисуем аппроксимированный контур
    //             //Рисуем только контуры с определённым числом
    //             //вершин
    //             if(approx.size() == 4) {
    //                 std::vector<std::vector<cv::Point>> approx_contour =
    //                     {approx};
    //                 cv::drawContours(result,
    //                                  approx_contour,
    //                                  0,
    //                                  cv::Scalar(255,0,0),
    //                                  3);
    //             }
    //             //Описанный прямоугольник
    //             cv::Rect rect = cv::boundingRect(approx);
    //             cv::rectangle(result,
    //                           rect,
    //                           cv::Scalar(255,255,0),
    //                           3);
    //         }
    //     }




    //     return result;
    // }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProcessor>());
    rclcpp::shutdown();

    return 0;
}
