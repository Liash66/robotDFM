#pragma once

#include <iostream>
#include <cstdio>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include "face_recognition.hpp"
#include "serial_connector/serial_connector.hpp"

#include <thread>
#include <chrono>
#include <utility>

namespace camera
{
    CameraCV::CameraCV(int index, std::string port)
    {
        face_x = 0;
        face_y = 0; // абсолютный угол поворота камеры
        angle_x = 0;
        angle_y = 0;
        cam_index = index;
        com_port = std::move(port);

        facedetect.load("./res/haarcascades/haarcascade_frontalface_default.xml");
    }

    // Повернуть изображение
    cv::Mat CameraCV::rotate(const cv::Mat& src, double angle)
    {
        cv::Mat dst;
        cv::Point2f pt(static_cast<float>(src.cols / 2.), static_cast<float>(src.rows / 2.));
        cv::Mat r = getRotationMatrix2D(pt, angle, 1.0);
        warpAffine(src, dst, r, cv::Size(src.cols, src.rows));
        return dst;
    }

    // Получить координаты центра
    cv::Point2f CameraCV::get_center_rect(cv::Point2f tl, cv::Point2f br)
    {
        cv::Point2f center((tl.x + br.x) / 2, (tl.y + br.y) / 2);
        return center;
    }

    // Найти индекс самого переднего лица
    int CameraCV::get_front_face_index(std::vector<cv::Rect>& faces)
    {
        int front_face_index = 0;
        for (int i = 0; i < faces.size(); i++)
        {
            int max_side = 0;
            if (faces[i].height > max_side)
                front_face_index = i;
        }
        return front_face_index;
    }

    // Определить координаты наклоненного лица
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

    // Найти лица
    void CameraCV::find_faces(cv::CascadeClassifier& facedetect, const cv::Mat& img, std::vector<cv::Rect_<int>>& faces, int& angle) {
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

    // Определить добавочные углы для поворота приводов
    void CameraCV::determ_additional_angles(std::vector<cv::Rect_<int>>& faces, int front_face_index, cv::Point img_center, int& angle_x, int& angle_y) {
        double side = faces[front_face_index].height;  //  сторона квадрата лица
        double leg_y = (get_center_rect(faces[front_face_index].tl(), faces[front_face_index].br()).y - static_cast<float>(img_center.y)) * 0.000264;   // расстояние между центром лица и изображения камеры по вертикали
        double leg_x = (get_center_rect(faces[front_face_index].tl(), faces[front_face_index].br()).x - static_cast<float>(img_center.x)) * 0.000264;   // расстояние между центром лица и изображения камеры по горизонтали
        const int k = 126; // константа, связывающая расстояние до лица и сторону квадрата: c = k/a, где c - расстояние, a - сторона квадрата
        double dist = k / side;    // дистанция от камеры до лица
        angle_y = static_cast<int>(-atan(leg_y / dist) * 180 / acos(-1)); // добавочный угол Y
        angle_x = static_cast<int>(-atan(leg_x / dist) * 180 / acos(-1)); // добавочный угол X
    }

    void CameraCV::execute()
    {
        io::SerialConnector* m_sc;
        m_sc = new io::SerialConnector(com_port, 9600); // последовательный порт
        m_sc->Open();

        cv::VideoCapture video(cam_index); // подключение камеры

        char buffer[10];
        int hand_angle; // угол поворота руки

        const int video_width = static_cast<int>(video.get(cv::CAP_PROP_FRAME_WIDTH)); // ширина видеопотока
        const int video_height = static_cast<int>(video.get(cv::CAP_PROP_FRAME_HEIGHT)); // высота видеопотока

        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now(); // таймер
        std::chrono::seconds mid_time(30);
        std::chrono::seconds reset_time(60); // время, через которое нужно обнулить таймер

        while (true)
        {
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start;

            // поворот руки раз в 30 сек
            if (elapsed_seconds < mid_time)
            {
                hand_angle = 90;
            }
            else if (elapsed_seconds > mid_time)
            {
                hand_angle = 0;
            }

            // обнуляем таймер
            if (elapsed_seconds > reset_time)
            {
                start = std::chrono::steady_clock::now();
            }

            video >> img; // получение текущего изображения
            cv::Point img_center(static_cast<int>((img.cols - 1) / 2.0), static_cast<int>((img.rows - 1) / 2.0)); // центр изображения

            std::vector<cv::Rect> faces; // создание массива с лицами

            int angle = 0; // угол наклона лица

            find_faces(facedetect, img, faces, angle);

            if (!faces.empty()) {

                int front_face_index = get_front_face_index(faces); // индекс переднего лица

                // определение координат лица из перевернутого изображения
                if (angle != 0)
                    determ_true_face_coord(faces, front_face_index, angle, video_width, video_height);

                determ_additional_angles(faces, front_face_index, img_center, angle_x, angle_y);

                rectangle(img, faces[front_face_index].tl(), faces[front_face_index].br(), cv::Scalar(0, 255, 0), 5); // построение рамки лица

                if (((face_x + angle_x) >= -45 && (face_x + angle_x) <= 45) && ((face_y + angle_y) >= -20 && (face_y + angle_y) <= 20))
                {
                    face_x += angle_x;  // угол привода оси X
                    face_y += angle_y;  // угол привода оси Y

                    // Копируем первое число в буфер
                    sprintf_s(buffer, sizeof(buffer), "%d", face_x + 45);
                    strcat_s(buffer, sizeof(buffer), " ");

                    // Копируем второе число в буфер
                    strcat_s(buffer, sizeof(buffer), std::to_string(face_y + 20).c_str());
                    strcat_s(buffer, sizeof(buffer), " ");

                    // Копируем третье число в буфер
                    strcat_s(buffer, sizeof(buffer), std::to_string(hand_angle).c_str());

                    // Выводим содержимое буфера
                    std::cout << buffer << std::endl;
                    m_sc->Send(buffer, 10);
                }
                else {
                    if (face_x != 0 && face_y != 0) {
                        face_x -= face_x/abs(face_x);
                        face_y -= face_y/abs(face_y);

                        // Копируем первое число в буфер
                        sprintf_s(buffer, sizeof(buffer), "%d", face_x + 45);
                        strcat_s(buffer, sizeof(buffer), " ");

                        // Копируем второе число в буфер
                        strcat_s(buffer, sizeof(buffer), std::to_string(face_y + 20).c_str());
                        strcat_s(buffer, sizeof(buffer), " ");

                        // Выводим содержимое буфера
                        std::cout << buffer << std::endl;
                        m_sc->Send(buffer, 10);
                    }
                }
            }
            else {
                // Возвращаем голову в начальное положение при отсутствии лиц
                if (face_x != 0 && face_y != 0) {
                    face_x -= face_x/abs(face_x);
                    face_y -= face_y/abs(face_y);

                    // Копируем первое число в буфер
                    sprintf_s(buffer, sizeof(buffer), "%d", face_x + 45);
                    strcat_s(buffer, sizeof(buffer), " ");

                    // Копируем второе число в буфер
                    strcat_s(buffer, sizeof(buffer), std::to_string(face_y + 20).c_str());
                    strcat_s(buffer, sizeof(buffer), " ");

                    // Выводим содержимое буфера
                    std::cout << buffer << std::endl;
                    m_sc->Send(buffer, 10);
                }
            }

            // выводим изображение
            std::cerr << faces.size() << std::endl;
            imshow("Eye", img);

            std::cerr << "Timer value: " << elapsed_seconds.count() << " seconds" << std::endl;

            if (cv::waitKey(10) >= 0)
                break;
        }

        delete m_sc;
    }
}