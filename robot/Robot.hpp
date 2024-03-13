#pragma once

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

#include "serial_connector/serial_connector.hpp"

namespace robot
{
    class Robot
    {
    private:
        int m_angle_x, m_angle_y; // углы поворота головы
        int m_add_angle_x, m_add_angle_y; // добавочные углы поворота головы
        int m_left_arm_angle{}; // угол левой руки
        int m_cam_index; // индекс камеры
        std::string m_com_port; // номер COM порт

        static void SetPosition(io::SerialConnector* m_sc, int p1, int p2, int p3);

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
        Robot(int index, std::string port);
        ~Robot() = default;

        void execute();
    };
}