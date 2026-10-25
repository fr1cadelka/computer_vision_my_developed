#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageKNNProcessor: public rclcpp::Node {
public:
    ImageKNNProcessor() : Node("image_processor_node") {
        //Храним только последний кадр
        // auto QoS = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

        subscriber =
            this->create_subscription<sensor_msgs::msg::Image>("raw_image",
                                                               10,
                                                               std::bind(&ImageKNNProcessor::image_callback,this,std::placeholders::_1));

        //Инициализация публикатора
        publisher =
            this->create_publisher<sensor_msgs::msg::Image>("processed_image",
                                                            10);

        this->initialize_knn();

    }
private:
    //Публикатор изображения
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
        publisher;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber;

    //Объект классификатора KNN
    cv::Ptr<cv::ml::KNearest> knn;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {

        //1. Преобразование в cv::Mat
        cv::Mat input_frame =
            cv_bridge::toCvCopy(msg, "bgr8")->image;

        //2. Обработка изображения
        cv::Mat output_frame = recognize_digit(input_frame);

        //3. Отправка изображения в топик
        //Определяем тип кодировки
        std::string encoding;
        if(output_frame.channels() == 1) {
            encoding = "mono8";
        }
        else {
            encoding = "bgr8";
        }
        //Формируем сообщение
        sensor_msgs::msg::Image::SharedPtr out_msg =
            cv_bridge::CvImage(msg->header,encoding,output_frame).toImageMsg();
        //Публикуем
        publisher->publish(*out_msg);

    }
    cv::Mat recognize_digit(cv::Mat &frame) {

        cv::Mat result, hsv, mask, red_mask1, red_mask2;

        //1.Преобразование в HSV
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        //2. Создание маски для красного цвета
        cv::inRange(hsv, cv::Scalar(0, 100, 100),
                    cv::Scalar(10, 255, 255), red_mask1);
        cv::inRange(hsv, cv::Scalar(160, 100, 100),
                    cv::Scalar(180, 255, 255), red_mask2);
        mask = red_mask1 | red_mask2;

        //3. Очистка от шума с использованием открытия
        cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE, cv::Size(5,5));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

        //Контроль результата
        // result = mask.clone();

        //4. Выделение и анализ контуров
        //Контуры
        std::vector<std::vector<cv::Point>> contours;
        //Иерархия контуров (для оценки вложенности)
        //Каждый элемент содержит 4 компонента:
        //1. Индекс следующего внешнего контура
        //2. Индекс предыдущего внешнего контура
        //3. Индекс первого вложенного контура
        //4. Индекс родителя (внешнего по отношению к текущему)
        //Если контура нет, то индекс -1
        std::vector<cv::Vec4i> hierarchy;

        //Определяем контуры
        cv::findContours(mask, contours, hierarchy,
                         cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

        result = frame.clone();

        //ROI для цифр
        cv::Mat digit_roi;
        //Координаты начала ROI для цифр
        cv::Point digit_roi_offset;
        //Проходим по контурам и анализируем их
        for(int i = 0; i >= 0; i = hierarchy[i][0]) {
            std::vector<cv::Point> contour = contours.at(i);
            //Анализируем контур на предмет окружности
            double area = cv::contourArea(contour);
            double perimeter = cv::arcLength(contour, true);

            //Критерий окружности
            double circularity = (4 * CV_PI * area) /
                                 (perimeter * perimeter);
            //Если контур является окружностью, то
            //значение критерия будет от 0,85 до 1,0
            if(circularity > 0.80) {
                //Смотрим на вложенный контур
                int child_idx = hierarchy[i][2];
                if(child_idx != -1) { //если контур существует
                    //Анализируем вложенный контур
                    std::vector<cv::Point> child_contour = contours.at(child_idx);
                    //Анализируем контур на предмет окружности
                    double child_area = cv::contourArea(child_contour);
                    double child_perimeter = cv::arcLength(child_contour, true);

                    //Критерий окружности
                    double child_circularity = (4 * CV_PI * child_area) /
                                         (child_perimeter * child_perimeter);
                    if(child_circularity > 0.8) {
                        // cv::drawContours(result,
                        //                  contours,
                        //                  i,
                        //                  cv::Scalar(0,255,0),
                        //                  4);
                        // cv::drawContours(result,
                        //                  contours,
                        //                  child_idx,
                        //                  cv::Scalar(0,255,0),
                        //                  4);
                        //Выделяем ROI для цифр
                        cv::Rect roi_rect = cv::boundingRect(child_contour);
                        //Делаем внутренние отступы чтобы убрать кольцо знака
                        int offset = roi_rect.width * 0.15; //15%
                        //Определяем внутренную рамку
                        cv::Rect inner_rect(roi_rect.x + offset,
                                            roi_rect.y + offset,
                                            roi_rect.width - 2 * offset,
                                            roi_rect.height - 2 * offset);
                        //Для проверки изобразим рамку
                        cv::rectangle(result,
                                      inner_rect,
                                      cv::Scalar(0,255,0),
                                      3);
                        //Фиксируем область цифр
                        digit_roi = frame(inner_rect);
                        digit_roi_offset = cv::Point(inner_rect.x, inner_rect.y);
                    }

                }
            }

        }
        //Проверяем, удалось ли обнаружить область цифр
        if(!digit_roi.empty()) {
            cv::Mat gray_roi, binary_roi;

            //Подготавливаем изображение для распознавания контуров цифр
            cv::cvtColor(digit_roi, gray_roi, cv::COLOR_BGR2GRAY);
            cv::threshold(gray_roi, binary_roi, 0, 255, cv::THRESH_BINARY_INV |
                                                            cv::THRESH_OTSU);

            //Выделяем контуры
            std::vector<std::vector<cv::Point>> digit_contours;
            cv::findContours(binary_roi,
                             digit_contours,
                             cv::RETR_EXTERNAL, //выделяются только внешние контуры
                             cv::CHAIN_APPROX_SIMPLE);

            //Анализ контуров
            std::vector<cv::Rect> digit_boxes;
            for(int i = 0; i < digit_contours.size(); i++) {
                std::vector<cv::Point> contour = digit_contours.at(i);
                //Описанный прямоугольник
                cv::Rect bbox = cv::boundingRect(contour);
                //Анализ размера блока
                if(bbox.height > digit_roi.rows * 0.4 &&
                    bbox.width < digit_roi.cols * 0.8) {

                    //Для проверки нарисуем прямоугольники
                    cv::Rect global_bbox = bbox;
                    global_bbox.x += digit_roi_offset.x;
                    global_bbox.y += digit_roi_offset.y;

                    cv::rectangle(result,
                                  global_bbox,
                                  cv::Scalar(255,0,0), 4);

                    digit_boxes.push_back(bbox);
                }
            }
            //Сортируем по положению слева направо рамки для цифр
            std::sort(digit_boxes.begin(), digit_boxes.end(),
                      [](const cv::Rect &a, const cv::Rect &b) {
                return a.x < b.x;
            });

            //Распознавание цифр
            std::string speed_limit_str;
            //Проходим в цикле по всем рамкам с цифрами
            for(int i = 0; i < digit_boxes.size(); i++) {
                cv::Rect box = digit_boxes.at(i);
                //Выделяем изображение
                cv::Mat single_digit = binary_roi(box);

                //Выполняем предобработку
                cv::Mat resized_digit =
                    prepare_image(single_digit);
                //Векторизация
                cv::Mat sample = resized_digit.clone();
                sample.convertTo(sample, CV_32F);
                sample = sample.reshape(1,1);

                //Отправка в классификатор
                cv::Mat response, //итоговый класс (лучший вариант)
                    neighborResponses, //классы из ближайших соседей
                    dist; //расстояние до ближайших соседей

                knn->findNearest(sample,
                                 3, //количество ближайших соседей
                                 response,
                                 neighborResponses,
                                 dist);
                //Выделяем распознанную цифру
                int detected_digit = (int)response.at<float>(0);

                RCLCPP_INFO_STREAM(this->get_logger(),
                                   "Цифра распознана как "<<detected_digit);

                //Для дополнительного анализа можно посмотреть данные
                //для ближайших соседей
                for(int j = 0; j < neighborResponses.cols; j++) {
                    int neighborVal = (int)neighborResponses.at<float>(0, j);
                    float distance = dist.at<float>(0,j);

                    RCLCPP_INFO_STREAM(this->get_logger(),
                                       "\tСосед "<<j+1<<" голосует за "<<
                                           neighborVal<<" Дистанция: "<<
                                           distance);
                }


                speed_limit_str += std::to_string(detected_digit);


            }
            RCLCPP_INFO_STREAM(this->get_logger(),
                               "Распознанная скорость: "<<speed_limit_str);

        }


        // RCLCPP_INFO(this->get_logger(), "Image ready");
        return result;
    }

    //Инициализация алгоритма
    void initialize_knn() {
        //Считывание обучающего множества
        cv::Mat digits_img = cv::imread("/home/ls/Downloads/digits_printed.png", 0);
        //проверяем открылся ли файл
        if(digits_img.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Ошибка при открытии файла с цифрами");
            return;
        }
        //Подготавливаем обучающее множество
        cv::Mat train_data;
        std::vector<int> train_labels;

        //Исходное изображение содержит по 5 рядов (в кажом ряде 100 изображений)
        //цифр от 0 до 90 размером 20 на 20 пикселей
        for(int i = 0; i < 50; i++) {
            for(int j = 0; j < 100; j++) {
                cv::Mat cell = digits_img(cv::Rect(j * 20,
                                                   i * 20, 20, 20));
                //Записываем данные в набор
                train_data.push_back(cell.clone().reshape(1,1));
                train_labels.push_back(i / 5);
            }
        }
        //Перед подачей в алгоритм необходимо преобразовать данные
        train_data.convertTo(train_data,CV_32F);

        //Выполняем типо обучение
        knn = cv::ml::KNearest::create();
        knn->train(train_data, cv::ml::ROW_SAMPLE, train_labels);
    }
    //Подготовка изображения с цифрой перед подачей на классификатор
    cv::Mat prepare_image(cv::Mat &digit_roi) {
        //Центрирование
        int size = std::max(digit_roi.cols, digit_roi.rows);
        cv::Mat square = cv::Mat::zeros(size,size, CV_8U); //чёрное квадратное

        //Вычисляем параметры
        int x = (size - digit_roi.cols)/2;
        int y = (size - digit_roi.rows)/2;
        digit_roi.copyTo(square(cv::Rect(x,
                                         y,
                                         digit_roi.cols,
                                         digit_roi.rows)));

        //Масштабируем
        cv::Mat resized;
        cv::resize(square, resized, cv::Size(20,20));
        return resized;
    }


};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageKNNProcessor>());
    rclcpp::shutdown();

    return 0;
}
