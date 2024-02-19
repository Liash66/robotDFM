#include <iostream>
#include <stdio.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include "serial_connector/serial_connector.hpp"

#include <asio.hpp>

#include <thread>
#include <chrono>

using namespace std;
using namespace cv;

// функция поворота изображения
Mat rotate(const Mat& src, double angle)
{
    Mat dst;
    Point2f pt(static_cast<float>(src.cols / 2.), static_cast<float>(src.rows / 2.));
    Mat r = getRotationMatrix2D(pt, angle, 1.0);
    warpAffine(src, dst, r, Size(src.cols, src.rows));
    return dst;
}

// получить координаты центра
Point2f get_center_rect(Point2f tl, Point2f br)
{
    Point2f center((tl.x + br.x) / 2, (tl.y + br.y) / 2);
    return center;
}

// функция нахождения индекса самого переднего лица
int get_front_face_index(vector<Rect>& faces) {
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
void determ_true_face_coord(vector<Rect>& faces, const int front_face_index, int angle, const int video_width, int const video_height) {
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

int main()
{
    io::SerialConnector* m_sc;
    m_sc = new io::SerialConnector("COM7", 9600); // последовательный порт
    m_sc->Open();
    char buffer[7];

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    VideoCapture video(0); // выставляем индекс камеры
    CascadeClassifier facedetect;
    Mat img;
    facedetect.load("./res/haarcascades/haarcascade_frontalface_default.xml");

    const int video_width = static_cast<int>(video.get(cv::CAP_PROP_FRAME_WIDTH)); // ширина видеопотока
    const int video_height = static_cast<int>(video.get(cv::CAP_PROP_FRAME_HEIGHT)); // высота видеопотока

    int k = 126; // константа, связывающая расстояние до лица и сторону квадрата: c = k/a, где c - расстояние, a - сторона квадрата
    int face_x = 0, face_y = 0; // абсолютный угол поворота камеры
    double leg_x, leg_y, dist, side;
    int angle_x = 0, angle_y = 0;

    while (true)
    {
        video >> img; // получение текущего изображения
        Point center(static_cast<int>((img.cols - 1) / 2.0), static_cast<int>((img.rows - 1) / 2.0)); // центр изображения

        vector<Rect> faces; // создание массива с лицами

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

            rectangle(img, faces[front_face_index].tl(), faces[front_face_index].br(), Scalar(0, 255, 0), 5); // построение рамки лица
            putText(img, to_string(angle_x) + " " + to_string(angle_y), Point(20, 40), FONT_HERSHEY_DUPLEX, 1, Scalar(0, 0, 0), 1); // вывод углов до лица
        }

        if (face_x >= -45 && face_x <= 45 && face_y >= -20 && face_y <= 20)
        {
            sprintf_s(buffer, 7, "%d %d", (face_x + 45), (face_y + 20));
            std::cout << buffer << std::endl;
            m_sc->Send(buffer, 7);
        }

        //выводим изображение
        std::cout << faces.size() << std::endl;
        imshow("Eye", img);

        if (waitKey(10) >= 0)
            break;
    }

    delete m_sc;

    return 0;
}