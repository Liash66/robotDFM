#include "face_recognition/face_recognition.hpp"

int main()
{
    camera::CameraCV* m_cv;
    m_cv = new camera::CameraCV(0, "COM7");
    m_cv->execute();

    delete m_cv;

    return 0;
}