#pragma once

#include <iostream>
#include <cstdio>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

#include "Robot.hpp"
#include "serial_connector/serial_connector.hpp"
#include "head/head_moving/img_processing.h"
#include "head/head_moving/driver_determination.h"
#include "left_arm/left_arm_angle_determination.h"

#include <thread>
#include <chrono>
#include <utility>

namespace robot
{
    Robot::Robot(int index, std::string port)
    {
        m_angle_x = 0;
        m_angle_y = 0;
        m_add_angle_x = 0;
        m_add_angle_y = 0;
        m_cam_index = index;
        m_com_port = std::move(port);
    }

    void Robot::execute()
    {
        io::SerialConnector* m_sc;
        m_sc = new io::SerialConnector(m_com_port, 9600); // последовательный порт
        m_sc->Open();

        char buffer[10]; // буфер данных для порта

        cv::VideoCapture video(m_cam_index); // подключение камеры
        cv::Mat img; // текущее изображение с камеры
        cv::CascadeClassifier facedetect;
        facedetect.load("./res/haarcascades/haarcascade_frontalface_default.xml"); // подключаем детектор лиц

        const int video_width = static_cast<int>(video.get(cv::CAP_PROP_FRAME_WIDTH)); // ширина видеопотока
        const int video_height = static_cast<int>(video.get(cv::CAP_PROP_FRAME_HEIGHT)); // высота видеопотока

        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now(); // таймер
        std::chrono::duration<double> elapsed_seconds{};

        while (true)
        {
            determ_left_arm_angle(start, elapsed_seconds, m_left_arm_angle);

            video >> img; // получение текущего изображения
            cv::Point img_center(static_cast<int>((img.cols - 1) / 2.0), static_cast<int>((img.rows - 1) / 2.0)); // центр изображения
            std::vector<cv::Rect> faces; // создание массива с лицами
            int angle = 0; // угол наклона лица
            find_faces(facedetect, img, faces, angle);

            // Если лицо обнаружено
            if (!faces.empty()) {
                int front_face_index = get_front_face_index(faces); // индекс ближайшего лица
                determ_true_face_coord(faces, front_face_index, angle, video_width, video_height);
                determ_additional_angles(faces, front_face_index, img_center, m_add_angle_x, m_add_angle_y);

                rectangle(img, faces[front_face_index].tl(), faces[front_face_index].br(), cv::Scalar(0, 255, 0), 5); // построение рамки лица

                // В пределах допустимых углов поворота головы
                if (((m_angle_x + m_add_angle_x) >= -45 && (m_angle_x + m_add_angle_x) <= 45) && ((m_angle_y + m_add_angle_y) >= -20 && (m_angle_y + m_add_angle_y) <= 20))
                {
                    m_angle_x += m_add_angle_x; // угол привода оси x
                    m_angle_y += m_add_angle_y; // угол привода оси y

                    // Копируем первое число в буфер
                    sprintf_s(buffer, sizeof(buffer), "%d", m_angle_x + 45);
                    strcat_s(buffer, sizeof(buffer), " ");

                    // Копируем второе число в буфер
                    strcat_s(buffer, sizeof(buffer), std::to_string(m_angle_y + 20).c_str());
                    strcat_s(buffer, sizeof(buffer), " ");

                    // Копируем третье число в буфер
                    strcat_s(buffer, sizeof(buffer), std::to_string(m_left_arm_angle).c_str());

                    // Выводим содержимое буфера
                    std::cout << buffer << std::endl;
                    m_sc->Send(buffer, 10);
                }
                // В предельных углах поворота головы
                else {
                    // Возвращаем голову в начальное положение
                    if (m_angle_x != 0 && m_angle_y != 0) {
                        m_angle_x = 0;
                        m_angle_y = 0;

                        // Копируем первое число в буфер
                        sprintf_s(buffer, sizeof(buffer), "%d", m_angle_x + 45);
                        strcat_s(buffer, sizeof(buffer), " ");

                        // Копируем второе число в буфер
                        strcat_s(buffer, sizeof(buffer), std::to_string(m_angle_y + 20).c_str());
                        strcat_s(buffer, sizeof(buffer), " ");

                        // Копируем третье число в буфер
                        strcat_s(buffer, sizeof(buffer), std::to_string(m_left_arm_angle).c_str());

                        // Выводим содержимое буфера
                        std::cout << buffer << std::endl;
                        m_sc->Send(buffer, 10);
                    }
                }
            }

            // Если лицо не обнаружено
            else {
                // Возвращаем голову в начальное положение
                if (m_angle_x != 0 && m_angle_y != 0) {
                    m_angle_x = 0;
                    m_angle_y = 0;

                    // Копируем первое число в буфер
                    sprintf_s(buffer, sizeof(buffer), "%d", m_angle_x + 45);
                    strcat_s(buffer, sizeof(buffer), " ");

                    // Копируем второе число в буфер
                    strcat_s(buffer, sizeof(buffer), std::to_string(m_angle_y + 20).c_str());
                    strcat_s(buffer, sizeof(buffer), " ");

                    // Выводим содержимое буфера
                    std::cout << buffer << std::endl;
                    m_sc->Send(buffer, 10);
                }
            }

            std::cerr << "Faces number: " << faces.size() << std::endl;
            imshow("Eye", img);
            std::cerr << "Timer value: " << elapsed_seconds.count() << " seconds" << std::endl;

            if (cv::waitKey(10) >= 0)
                break;
        }
        delete m_sc;
    }
}