#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageSVMProcessor: public rclcpp::Node {
public:
    ImageSVMProcessor() : Node("image_processor_svm_node") {
        //Храним только последний кадр
        // auto QoS = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

        subscriber =
            this->create_subscription<sensor_msgs::msg::Image>("raw_image",
                                                               10,
                                                               std::bind(&ImageSVMProcessor::image_callback,this,std::placeholders::_1));

        //Инициализация публикатора
        publisher =
            this->create_publisher<sensor_msgs::msg::Image>("processed_image",
                                                            10);

        this->initialize_svm();

    }
private:
    //Публикатор изображения
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
        publisher;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber;

    //Объект классификатора SVM
    cv::Ptr<cv::ml::SVM> svm;

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
            if(circularity > 0.8) {
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

                float response = svm->predict(sample);
                //Выделяем распознанную цифру
                int detected_digit = (int)response;

                RCLCPP_INFO_STREAM(this->get_logger(),
                                   "Цифра распознана как "<<detected_digit);


                speed_limit_str += std::to_string(detected_digit);


            }
            RCLCPP_INFO_STREAM(this->get_logger(),
                               "Распознанная скорость: "<<speed_limit_str);

        }


        // RCLCPP_INFO(this->get_logger(), "Image ready");
        return result;
    }

    //Инициализация алгоритма
    void initialize_svm() {
        cv::Mat train_data;
        std::vector<int> train_labels;

        this->create_dataset(train_data, train_labels);

        //Проверяем полученные данные
        if(train_data.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Обучающие данные не получены");
            return;
        }
        //Инициализируем SVM
        svm = cv::ml::SVM::create();
        svm->setType(cv::ml::SVM::C_SVC);

        //Определяем ядро
        //Линейное ядро
        svm->setKernel(cv::ml::SVM::LINEAR);
        //RBF ядро
        // svm->setKernel(cv::ml::SVM::RBF);
        // svm->setC(100.0); //штраф
        // svm->setGamma(10); //радиус влияния
        //Определяем критерий остановки обучения
        svm->setTermCriteria(
            cv::TermCriteria(cv::TermCriteria::MAX_ITER, 100, 1e-8));
        //Подготавливаем данные
        train_data.convertTo(train_data, CV_32F);
        //Обучаем SVM
        svm->train(train_data, cv::ml::ROW_SAMPLE, train_labels);

        RCLCPP_INFO(this->get_logger(), "SVM обучен");
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

    //Функция по созданию датасета
    void create_dataset(cv::Mat &train_data, std::vector<int> &train_labels) {
        //Определим спискок шрифтов OpenCV, который будем использовать
        std::vector<int> fonts = {
            cv::FONT_HERSHEY_SIMPLEX,
            cv::FONT_HERSHEY_DUPLEX
            // cv::FONT_HERSHEY_PLAIN,
            // cv::FONT_HERSHEY_COMPLEX
        };

        //Набор цифр
        std::vector<cv::Mat> all_digits;
        //Перебираем варианты
        //1. Цифра
        for(int digit = 0; digit <= 9; digit++) {
            std::string text = std::to_string(digit);

            //2. Шрифт
            for(int f = 0; f < fonts.size(); f++) {
                int font = fonts.at(f);
                //3. Размер шрифта
                for(double scale = 0.5; scale < 1.75; scale += 0.01) {
                    //4. Толщина шрифта
                    for(int thickness = 1; thickness <=2; thickness++) {
                        //Создаём изображение, на котором будет цифра
                        cv::Mat canvas =
                            cv::Mat::zeros(40,40, CV_8U);

                        //Определяем размер текста для центрирования
                        int baseline = 0;
                        cv::Size textSize = cv::getTextSize(text,
                                                            font,
                                                            scale,
                                                            thickness,
                                                            &baseline);
                        cv::Point textOrg((40 - textSize.width) / 2,
                                          (40 + textSize.height) / 2);

                        //Рисуем белую цифру
                        cv::putText(canvas,
                                    text,
                                    textOrg,
                                    font, scale, cv::Scalar(255),
                                    thickness, cv::LINE_AA);

                        //Приводим к виду, необходимому для обучения
                        //Находим описанный многоугольник
                        std::vector<std::vector<cv::Point>> contours;
                        cv::findContours(canvas,
                                         contours,
                                         cv::RETR_EXTERNAL,
                                         cv::CHAIN_APPROX_SIMPLE);

                        //Если контур найден, то обрабатываем
                        if(!contours.empty()) {
                            cv::Rect bbox = cv::boundingRect(contours.at(0));
                            cv::Mat digit_roi = canvas(bbox);
                            cv::Mat final_digit =
                                prepare_image(digit_roi);


                            //Добавляем данные в локальный датасет
                            train_data.push_back(final_digit.reshape(1,1));
                            train_labels.push_back(digit);

                            //Добавляем цифру в набор
                            all_digits.push_back(final_digit);
                        }
                    }
                }
            }
        }
        //Сохраняем полученный датасет в файл
        std::string dataset_path =
            "/home/ls/Downloads/digits_printed.png";
        //Количество цифр в строке
        int digits_per_row = 100;
        int rows =
            (all_digits.size() + digits_per_row - 1)/
            digits_per_row;
        //Итоговое изображение со всеми цифрами
        cv::Mat dataset_vis =
            cv::Mat::zeros(rows * 20,
                           digits_per_row * 20, CV_8U);

        //В цикле записываем изображения
        for(int k = 0; k < all_digits.size(); k++) {
            int r = k / digits_per_row;
            int c = k % digits_per_row;
            all_digits.at(k).copyTo(
                dataset_vis(cv::Rect(c*20,r*20,20,20)));
        }

        cv::imwrite(dataset_path, dataset_vis);



    }


};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSVMProcessor>());
    rclcpp::shutdown();

    return 0;
}
