#include "robot/Robot.hpp"

int main()
{
    robot::Robot* m_cv;
    m_cv = new robot::Robot(0, "COM7");
    m_cv->execute();

    delete m_cv;

    return 0;
}
