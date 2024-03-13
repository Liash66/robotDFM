#ifndef ROBOTDFM_DRIVER_DETERMINATION_H
#define ROBOTDFM_DRIVER_DETERMINATION_H

#include "../../Robot.hpp"

namespace robot {
    // Определить добавочные углы для поворота приводов
    void Robot::determ_additional_angles(std::vector<cv::Rect_<int>>& faces, int front_face_index, cv::Point img_center, int& angle_x, int& angle_y) {
        double side = faces[front_face_index].height;  //  сторона квадрата лица
        double leg_y = (get_center_rect(faces[front_face_index].tl(), faces[front_face_index].br()).y - static_cast<float>(img_center.y)) * 0.000264;   // расстояние между центром лица и изображения камеры по вертикали
        double leg_x = (get_center_rect(faces[front_face_index].tl(), faces[front_face_index].br()).x - static_cast<float>(img_center.x)) * 0.000264;   // расстояние между центром лица и изображения камеры по горизонтали
        const int k = 126; // константа, связывающая расстояние до лица и сторону квадрата: c = k/a, где c - расстояние, a - сторона квадрата
        double dist = k / side;    // дистанция от камеры до лица
        angle_y = static_cast<int>(-atan(leg_y / dist) * 180 / acos(-1)); // добавочный угол Y
        angle_x = static_cast<int>(-atan(leg_x / dist) * 180 / acos(-1)); // добавочный угол X
    }

}

#endif //ROBOTDFM_DRIVER_DETERMINATION_H
