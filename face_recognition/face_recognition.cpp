#pragma once

#include <iostream>
#include <stdio.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include "face_recognition.hpp"
#include "serial_connector/serial_connector.hpp"

namespace camera
{
    CameraCV::CameraCV(unsigned int index, std::string port)
    {
        k = 126; // константа, связывающая расстояние до лица и сторону квадрата: c = k/a, где c - расстояние, a - сторона квадрата
        face_x = 0;
        face_y = 0; // абсолютный угол поворота камеры
        angle_x = 0;
        angle_y = 0;
        cam_index = index;
        com_port = port;

        facedetect.load("./res/haarcascades/haarcascade_frontalface_default.xml");
    }

    // функция поворота изображения
    cv::Mat CameraCV::rotate(const cv::Mat& src, double angle)
    {
        cv::Mat dst;
        cv::Point2f pt(static_cast<float>(src.cols / 2.), static_cast<float>(src.rows / 2.));
        cv::Mat r = getRotationMatrix2D(pt, angle, 1.0);
        warpAffine(src, dst, r, cv::Size(src.cols, src.rows));
        return dst;
    }

    // получить координаты центра
    cv::Point2f CameraCV::get_center_rect(cv::Point2f tl, cv::Point2f br)
    {
        cv::Point2f center((tl.x + br.x) / 2, (tl.y + br.y) / 2);
        return center;
    }

    // функция нахождения индекса самого переднего лица
    int CameraCV::get_front_face_index(std::vector<cv::Rect>& faces)
    {
        int front_face_index = 0;
        for (int i = 0; i < faces.size(); i++)
        {
            int max_side = 0;
            if (faces[i].height > max_side)
            {
                front_face_index = i;
            }
        }
        return front_face_index;
    }

    // функция определения координат наклоненного лица
    void CameraCV::determ_true_face_coord(std::vector<cv::Rect>& faces, const int front_face_index, int angle, const int video_width, int const video_height)
    {
        // нахождение координат центра лица (из координат верхнего левого угла лица) в с.к. с центром в середине изображения
        int face_X = faces[front_face_index].x + faces[front_face_index].width / 2 - video_width / 2;
        int face_Y = -faces[front_face_index].y - faces[front_face_index].height / 2 + video_height / 2;

        // нахождение координат центра лица в с/к, повернутой обратно
        angle = -angle;
        int old_x = static_cast<int>(face_X * cos(angle) - face_Y * sin(angle));
        int old_y = static_cast<int>(face_Y * cos(angle) + face_X * sin(angle));
        if (abs(angle) == 35)
        {
            old_x = -old_x;
            old_y = -old_y;
        }

        // определение левого верхнего угла лица (из этого параметра мы находили центр лица)
        faces[front_face_index].x = old_x - faces[front_face_index].width / 2 + video_width / 2;
        faces[front_face_index].y = -old_y - faces[front_face_index].height / 2 + video_height / 2;
    }

    void CameraCV::execute()
    {
        io::SerialConnector* m_sc;
        m_sc = new io::SerialConnector(com_port, 9600); // последовательный порт
        m_sc->Open();

        cv::VideoCapture video(cam_index); // выставляем индекс камеры

        char buffer[7];

        const int video_width = static_cast<int>(video.get(cv::CAP_PROP_FRAME_WIDTH)); // ширина видеопотока
        const int video_height = static_cast<int>(video.get(cv::CAP_PROP_FRAME_HEIGHT)); // высота видеопотока

        while (true)
        {
            video >> img; // получение текущего изображения
            cv::Point center(static_cast<int>((img.cols - 1) / 2.0), static_cast<int>((img.rows - 1) / 2.0)); // центр изображения

            std::vector<cv::Rect> faces; // создание массива с лицами

            int angle = 0; // угол наклона лица

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

            if (!faces.empty()) {

                int front_face_index = get_front_face_index(faces); // индекс переднего лица

                // определение координат лица из перевернутого изображения
                if (angle != 0)
                    determ_true_face_coord(faces, front_face_index, angle, video_width, video_height);

                side = faces[front_face_index].height;  //  сторона квадрата лица
                leg_y = (get_center_rect(faces[front_face_index].tl(), faces[front_face_index].br()).y - static_cast<float>(center.y)) * 0.000264;   // расстояние между центром лица и изображения камеры по вертикали
                leg_x = (get_center_rect(faces[front_face_index].tl(), faces[front_face_index].br()).x - static_cast<float>(center.x)) * 0.000264;   // расстояние между центром лица и изображения камеры по горизонтали
                dist = k / side;    // дистанция от камеры до лица
                angle_y = static_cast<int>(-atan(leg_y / dist) * 180 / acos(-1)); // добавочный угол Y
                angle_x = static_cast<int>(-atan(leg_x / dist) * 180 / acos(-1)); // добавочный угол X

                face_x += angle_x;  // угол машинки оси X
                face_y += angle_y;  // угол машинки оси Y

                rectangle(img, faces[front_face_index].tl(), faces[front_face_index].br(), cv::Scalar(0, 255, 0), 5); // построение рамки лица
                putText(img, std::to_string(angle_x) + " " + std::to_string(angle_y), cv::Point(20, 40), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 1); // вывод углов до лица
            }

            if (face_x >= -45 && face_x <= 45 && face_y >= -20 && face_y <= 20)
            {
                sprintf_s(buffer, 7, "%d %d", (face_x + 45), (face_y + 20));
                std::cerr << buffer << std::endl;
                m_sc->Send(buffer, 7);
            }

            //выводим изображение
            std::cerr << faces.size() << std::endl;
            imshow("Eye", img);

            if (cv::waitKey(10) >= 0)
                break;
        }

        delete m_sc;
    }
}