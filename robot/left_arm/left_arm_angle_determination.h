#ifndef ROBOTDFM_LEFT_ARM_ANGLE_DETERMINATION_H
#define ROBOTDFM_LEFT_ARM_ANGLE_DETERMINATION_H

#include <thread>
#include <chrono>
#include <utility>

#include "../Robot.hpp"

namespace robot {
    // Определение угла для левой руки
    void Robot::determ_left_arm_angle(std::chrono::steady_clock::time_point &start, std::chrono::duration<double> &elapsed_seconds, int &angle) {
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        elapsed_seconds = end - start;

        // поворот руки раз в 30 сек
        if (elapsed_seconds < std::chrono::seconds(30))
            angle = 90;
        else if (elapsed_seconds > std::chrono::seconds(30))
            angle = 0;

        // обнуляем таймер
        if (elapsed_seconds > std::chrono::seconds(60))
            start = std::chrono::steady_clock::now();
    }
}

#endif //ROBOTDFM_LEFT_ARM_ANGLE_DETERMINATION_H
