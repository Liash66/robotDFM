#ifndef ROBOTDFM_IMG_PROCESSING_H
#define ROBOTDFM_IMG_PROCESSING_H

#include "../../Robot.hpp"

namespace robot {
    // Повернуть изображение
    cv::Mat Robot::rotate(const cv::Mat &src, double angle) {
        cv::Mat dst;
        cv::Point2f pt(static_cast<float>(src.cols / 2.), static_cast<float>(src.rows / 2.));
        cv::Mat r = getRotationMatrix2D(pt, angle, 1.0);
        warpAffine(src, dst, r, cv::Size(src.cols, src.rows));
        return dst;
    }

    // Получить координаты центра
    cv::Point2f Robot::get_center_rect(cv::Point2f tl, cv::Point2f br) {
        cv::Point2f center((tl.x + br.x) / 2, (tl.y + br.y) / 2);
        return center;
    }

    // Найти индекс самого переднего лица
    int Robot::get_front_face_index(std::vector<cv::Rect> &faces) {
        int front_face_index = 0;
        for (int i = 0; i < faces.size(); i++) {
            int max_side = 0;
            if (faces[i].height > max_side)
                front_face_index = i;
        }
        return front_face_index;
    }

    // Определить координаты наклоненного лица
    void Robot::determ_true_face_coord(std::vector<cv::Rect> &faces, const int front_face_index, int angle,
                                       const int video_width, int const video_height) {
        if (angle != 0) {
            // нахождение координат центра лица (из координат верхнего левого угла лица) в с.к. с центром в середине изображения
            int face_X = faces[front_face_index].x + faces[front_face_index].width / 2 - video_width / 2;
            int face_Y = -faces[front_face_index].y - faces[front_face_index].height / 2 + video_height / 2;

            // нахождение координат центра лица в с/к, повернутой обратно
            angle = -angle;
            int old_x = static_cast<int>(face_X * cos(angle) - face_Y * sin(angle));
            int old_y = static_cast<int>(face_Y * cos(angle) + face_X * sin(angle));
            if (abs(angle) == 35) {
                old_x = -old_x;
                old_y = -old_y;
            }

            // определение левого верхнего угла лица (из этого параметра мы находили центр лица)
            faces[front_face_index].x = old_x - faces[front_face_index].width / 2 + video_width / 2;
            faces[front_face_index].y = -old_y - faces[front_face_index].height / 2 + video_height / 2;
        }
    }

    // Найти лица
    void Robot::find_faces(cv::CascadeClassifier &facedetect, const cv::Mat &img, std::vector<cv::Rect_<int>> &faces,
                           int &angle) {
        // поворачиваем на 35 градусов влево/вправо пока не найдем лицо или дойдем до остановы и сохраняем параметры лица под найденным углом
        facedetect.detectMultiScale(rotate(img, angle), faces, 1.3, 5);
        if (faces.empty()) {
            angle = 35;
            facedetect.detectMultiScale(rotate(img, angle), faces, 1.3, 5);
            if (faces.empty()) {
                angle = -35;
                facedetect.detectMultiScale(rotate(img, angle), faces, 1.3, 5);
                if (faces.empty()) {
                    angle = 70;
                    facedetect.detectMultiScale(rotate(img, angle), faces, 1.3, 5);
                    if (faces.empty()) {
                        angle = -70;
                        facedetect.detectMultiScale(rotate(img, angle), faces, 1.3, 5);
                    }
                }
            }
        }
    }

}

#endif //ROBOTDFM_IMG_PROCESSING_H
