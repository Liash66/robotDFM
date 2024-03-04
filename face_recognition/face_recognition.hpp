#pragma once

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

#include "serial_connector/serial_connector.hpp"

namespace camera
{
    class CameraCV
    {
    private:
        int face_x, face_y;
        int angle_x, angle_y;
        int cam_index;
        std::string com_port;
        cv::Mat img;
        cv::CascadeClassifier facedetect;

    private:
        // повернуть изображение
        static cv::Mat rotate(const cv::Mat& src, double angle);

        // получить координаты центра
        static cv::Point2f get_center_rect(cv::Point2f tl, cv::Point2f br);

        // найти индекс самого переднего лица
        static int get_front_face_index(std::vector<cv::Rect>& faces);

        // определить координаты наклоненного лица
        static void determ_true_face_coord(std::vector<cv::Rect>& faces, int front_face_index, int angle, int video_width, int video_height);

        // найти лица
        static void find_faces(cv::CascadeClassifier& facedetect, const cv::Mat& img, std::vector<cv::Rect_<int>>& faces, int& angle);

        // определить добавочные углы для поворота приводов
        static void determ_additional_angles(std::vector<cv::Rect_<int>>& faces, int front_face_index, cv::Point img_center, int& angle_x, int& angle_y);

    public:
        CameraCV(int index, std::string port);
        ~CameraCV() = default;

        void execute();
    };
}
