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

#include <thread>
#include <chrono>
#include <utility>
#include <Windows.h>
#include <mmsystem.h>

#pragma comment(lib, "winmm.lib")

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

        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        std::chrono::seconds reset_time(4); // Время, через которое нужно обнулить таймер

        std::this_thread::sleep_for(std::chrono::seconds(3));

        char buffer[10]; // буфер данных для порта

        cv::VideoCapture video(m_cam_index); // подключение камеры
        cv::Mat img; // текущее изображение с камеры
        cv::CascadeClassifier facedetect;
        facedetect.load("./res/haarcascades/haarcascade_frontalface_default.xml"); // подключаем детектор лиц

        const int video_width = static_cast<int>(video.get(cv::CAP_PROP_FRAME_WIDTH)); // ширина видеопотока
        const int video_height = static_cast<int>(video.get(cv::CAP_PROP_FRAME_HEIGHT)); // высота видеопотока

        double k_eye = 0; // коэффициент смещения координаты центра по x с учетом глаза слева

        while (true)
        {
            video >> img; // получение текущего изображения
            cv::Point img_center(static_cast<int>((img.cols - 1) / 2.0), static_cast<int>((img.rows - 1) / 2.0)); // центр изображения
            std::vector<cv::Rect> faces; // создание массива с лицами
            int angle = 0; // угол наклона лица
            find_faces(facedetect, img, faces, angle);

            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start;

            // Если лицо обнаружено
            if (!faces.empty())
            {
                int front_face_index = get_front_face_index(faces); // индекс ближайшего лица

                std::cerr << faces[front_face_index].height;

                if ((elapsed_seconds > reset_time) && (faces[front_face_index].height > 145))
                {
                    // Обнуляем таймер и выводим сообщение
                    PlaySound(TEXT("zayac_chort.wav"), NULL, SND_ASYNC);
                    start = std::chrono::steady_clock::now();
                }

                determ_true_face_coord(faces, front_face_index, angle, video_width, video_height);

                k_eye = 0.45 * (faces[front_face_index].br().x - faces[front_face_index].tl().x);

                faces[front_face_index].x -= k_eye; // сдвиг координаты лица для правильного поворота головы относительно центра
                determ_additional_angles(faces, front_face_index, img_center, m_add_angle_x, m_add_angle_y);

                std::cerr << faces[front_face_index].br().x - faces[front_face_index].tl().x << std::endl;

                faces[front_face_index].x += k_eye; // обратный сдвиг

                for (size_t i = 0; i < faces.size(); i++)
                {
                    if (i == front_face_index)
                    {
                        rectangle(img, faces[i].tl(), faces[i].br(), cv::Scalar(0, 0, 255), 5); // построение рамки ближайшего лица
                    }
                    else
                    {
                        rectangle(img, faces[i].tl(), faces[i].br(), cv::Scalar(0, 255, 0), 5); // построение рамок остальных лиц
                    }
                }

                // В пределах допустимых углов поворота головы
                if (((m_angle_x + m_add_angle_x) >= -45 && (m_angle_x + m_add_angle_x) <= 45) && ((m_angle_y + m_add_angle_y) >= -20 && (m_angle_y + m_add_angle_y) <= 20))
                {
                    m_angle_x += m_add_angle_x; // угол привода оси x
                    m_angle_y += m_add_angle_y; // угол привода оси y

                    SetPosition(m_sc, m_angle_x, m_angle_y, 0);
                }
                // В предельных углах поворота головы
                else
                {
                    // Возвращаем голову в начальное положение
                    if (m_angle_x != 0 && m_angle_y != 0)
                    {
                        m_angle_x = 0;
                        m_angle_y = 0;

                        SetPosition(m_sc, m_angle_x, m_angle_y, 113);
                    }
                }
            }

            // Если лицо не обнаружено
            else
            {
                m_angle_x = 0;
                m_angle_y = 0;

                SetPosition(m_sc, m_angle_x, m_angle_y, 113);
            }

            std::cerr << "Faces number: " << faces.size() << std::endl;
            imshow("Eye", img);

            if (cv::waitKey(10) >= 0)
                break;
        }

        m_sc->Close();
        delete m_sc;
    }
}