#pragma once

#include "../../Robot.hpp"

namespace robot
{
    // Определить добавочные углы для поворота приводов
    void Robot::determ_additional_angles(std::vector<cv::Rect_<int>>& faces, int front_face_index, cv::Point img_center, int& angle_x, int& angle_y)
    {
        double side = faces[front_face_index].height;  //  сторона квадрата лица
        double leg_y = (get_center_rect(faces[front_face_index].tl(), faces[front_face_index].br()).y - static_cast<float>(img_center.y)) * 0.000264;   // расстояние между центром лица и изображения камеры по вертикали
        double leg_x = (get_center_rect(faces[front_face_index].tl(), faces[front_face_index].br()).x - static_cast<float>(img_center.x)) * 0.000264;   // расстояние между центром лица и изображения камеры по горизонтали
        const int k = 126; // константа, связывающая расстояние до лица и сторону квадрата: c = k/a, где c - расстояние, a - сторона квадрата
        double dist = k / side;    // дистанция от камеры до лица
        angle_y = static_cast<int>(-atan(leg_y / dist) * 180 / acos(-1)); // добавочный угол Y
        angle_x = static_cast<int>(-atan(leg_x / dist) * 180 / acos(-1)); // добавочный угол X
    }

    void Robot::SetPosition(io::SerialConnector* m_sc, int p1, int p2, int p3)
    {
        std::vector<uint8_t> buffer; // создаём буфер
        buffer.reserve(3); // резервируем 3 байта

        // смещение системы координат
        p1 += 45;
        p2 += 30;

        buffer[0] = p1;
        buffer[1] = p2;
        buffer[2] = p3;

        m_sc->Send(buffer.data(), 3); // отправляем буфер по последовательному порту
    }
}