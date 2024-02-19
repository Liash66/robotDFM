#pragma once

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

namespace camera
{
    class CameraCV
    {
    private:
        int k;
        int face_x, face_y;
        double leg_x, leg_y, dist, side;
        int angle_x, angle_y;
        unsigned int cam_index;
        std::string com_port;
        cv::Mat img;
        cv::CascadeClassifier facedetect;

    public:
        CameraCV(unsigned int index, std::string port);

        ~CameraCV() {}

        // функция поворота изображения
        cv::Mat rotate(const cv::Mat& src, double angle);

        // получить координаты центра
        cv::Point2f get_center_rect(cv::Point2f tl, cv::Point2f br);

        // функция нахождения индекса самого переднего лица
        int get_front_face_index(std::vector<cv::Rect>& faces);

        // функция определения координат наклоненного лица
        void determ_true_face_coord(std::vector<cv::Rect>& faces, const int front_face_index, int angle, const int video_width, int const video_height);

        void execute();
    };
}