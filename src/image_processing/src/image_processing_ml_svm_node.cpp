#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageSVMProcessor: public rclcpp::Node {
public:
    ImageSVMProcessor() : Node("image_processor_svm_node") {
        subscriber = this->create_subscription<sensor_msgs::msg::Image>(
            "raw_image", 10,
            std::bind(&ImageSVMProcessor::image_callback, this, std::placeholders::_1));

        publisher = this->create_publisher<sensor_msgs::msg::Image>(
            "processed_image", 10);

        this->initialize_svm();
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber;
    cv::Ptr<cv::ml::SVM> svm;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat input_frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::Mat output_frame = recognize_digit(input_frame);

        std::string encoding = (output_frame.channels() == 1) ? "mono8" : "bgr8";
        sensor_msgs::msg::Image::SharedPtr out_msg =
            cv_bridge::CvImage(msg->header, encoding, output_frame).toImageMsg();
        publisher->publish(*out_msg);
    }

    cv::Mat recognize_digit(cv::Mat &frame) {
        cv::Mat result = frame.clone();
        cv::Mat hsv, yellow_mask;

        // 1. Ищем жёлтую карточку
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(20, 120, 100), cv::Scalar(35, 255, 255), yellow_mask);

        // Чистим маску от шума
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
        cv::morphologyEx(yellow_mask, yellow_mask, cv::MORPH_CLOSE, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(yellow_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        cv::Rect card_rect;
        bool card_found = false;
        double max_area = 0;

        // Ищем самый большой квадрат
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area > 5000 && area > max_area) {
                cv::Rect rect = cv::boundingRect(contour);
                double aspect = (double)rect.width / rect.height;

                if (aspect > 0.7 && aspect < 1.4) { // Проверка на квадратность
                    max_area = area;
                    card_rect = rect;
                    card_found = true;
                }
            }
        }

        // 2. Если карточка найдена, вырезаем цифру
        if (card_found) {
            // Рисуем зеленую рамку вокруг карточки
            cv::rectangle(result, card_rect, cv::Scalar(255, 0, 255), 3);

            // Берем кусок МАСКИ карточки
            cv::Mat card_mask_roi = yellow_mask(card_rect);

            // ИНВЕРТИРУЕМ маску (теперь фон черный, а цифра белая - как в датасете)
            cv::Mat binary_roi;
            cv::bitwise_not(card_mask_roi, binary_roi);

            // Ищем контуры цифры внутри карточки
            std::vector<std::vector<cv::Point>> digit_contours;
            cv::findContours(binary_roi, digit_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            cv::Rect digit_rect;
            bool digit_found = false;
            double max_digit_area = 0;

            for (const auto& d_contour : digit_contours) {
                double d_area = cv::contourArea(d_contour);
                if (d_area > 100) { // Игнорируем мелкий мусор
                    if (d_area > max_digit_area) {
                        max_digit_area = d_area;
                        digit_rect = cv::boundingRect(d_contour);
                        digit_found = true;
                    }
                }
            }

            // 3. Если цифру нашли — отправляем в SVM
            if (digit_found) {
                // Глобальные координаты цифры для синей рамки на экране
                cv::Rect global_digit_rect(digit_rect.x + card_rect.x,
                                           digit_rect.y + card_rect.y,
                                           digit_rect.width, digit_rect.height);
                cv::rectangle(result, global_digit_rect, cv::Scalar(255, 0, 255), 2);

                // Вырезаем саму цифру
                cv::Mat single_digit = binary_roi(digit_rect);

                // Готовим к отправке в SVM (сжимаем до 20x20)
                cv::Mat prepared = prepare_image(single_digit);

                // Конвертируем в нужный формат float
                cv::Mat sample;
                prepared.convertTo(sample, CV_32F);
                sample = sample.reshape(1, 1);

                // Запуск распознавания через SVM
                float response = svm->predict(sample);
                int detected_digit = (int)response;

                // Пишем результат на экран над карточкой
                cv::putText(result, "SVM Digit: " + std::to_string(detected_digit),
                            cv::Point(card_rect.x, card_rect.y - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);

                RCLCPP_INFO_STREAM(this->get_logger(), "SVM распознал цифру: " << detected_digit);
            }
        }
        return result;
    }

    // Инициализация алгоритма SVM
    void initialize_svm() {
        cv::Mat train_data;
        std::vector<int> train_labels;

        // Тот же путь, что и в KNN
        std::string folder_path = "/home/dmitry/Documents/computer_vision_ws_my_prj/src/dataset/dataset/4/*.png";

        std::vector<cv::String> filenames;
        try {
            cv::glob(folder_path, filenames);
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Папка не найдена! Проверь путь.");
            return;
        }

        if (filenames.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Ошибка: Картинки в папке не найдены!");
            return;
        }

        for (const auto& file : filenames) {
            cv::Mat img = cv::imread(file, cv::IMREAD_GRAYSCALE);
            if (img.empty()) continue;

            cv::Mat resized;
            cv::resize(img, resized, cv::Size(20, 20));

            train_data.push_back(resized.reshape(1, 1));
            train_labels.push_back(4); // Метка класса 4
        }

        train_data.convertTo(train_data, CV_32F);
        cv::Mat labels_mat = cv::Mat(train_labels);
        labels_mat.convertTo(labels_mat, CV_32S);

        // Создаем и настраиваем SVM
        svm = cv::ml::SVM::create();
        svm->setType(cv::ml::SVM::C_SVC);
        svm->setKernel(cv::ml::SVM::LINEAR); // Линейное ядро отлично работает для картинок
        svm->setC(1.0); // Штраф
        svm->setTermCriteria(cv::TermCriteria(cv::TermCriteria::MAX_ITER, 100, 1e-6));

        // Обучение
        svm->train(train_data, cv::ml::ROW_SAMPLE, labels_mat);

        RCLCPP_INFO(this->get_logger(), "SVM обучен на %zu картинках!", filenames.size());
    }

    // Подготовка изображения
    cv::Mat prepare_image(cv::Mat &digit_roi) {
        int size = std::max(digit_roi.cols, digit_roi.rows);
        cv::Mat square = cv::Mat::zeros(size, size, CV_8U);

        int x = (size - digit_roi.cols) / 2;
        int y = (size - digit_roi.rows) / 2;
        digit_roi.copyTo(square(cv::Rect(x, y, digit_roi.cols, digit_roi.rows)));

        cv::Mat resized;
        cv::resize(square, resized, cv::Size(20, 20));
        return resized;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSVMProcessor>());
    rclcpp::shutdown();
    return 0;
}
