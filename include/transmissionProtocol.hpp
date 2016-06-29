#ifndef H_H_TRANSMISSION_H_H
#define H_H_TRANSMISSION_H_H
#pragma once
#include <inttypes.h>
struct Transmission {
    enum class Action : char {
        NO_ACTION = 0,

        // actual commands
        MOVE_FORWARD, MOVE_LEFT, MOVE_RIGHT, MOVE_BACKWARD,
        ROTATE_LEFT, ROTATE_RIGHT,
        HEAD_LEFT, HEAD_RIGHT, HEAD_UP, HEAD_DOWN,

        // optional features
        TELEPORT_X, TELEPORT_Y, TELEPORT_ROT
    };
    Transmission(Action a, int16_t parameter): action(a), actionParameter(parameter){}

    Action  action;
    int16_t actionParameter;

    static constexpr int WRITE_SIZE = 3;
    void writeData(char* buffer, int bufferOffset){
        static_assert(sizeof(action) == 1, R"(Byte size of "Action" is greater than 1)");
        static_assert(sizeof(actionParameter) == 2, R"(Byte size of "ActionParameter" is not equel to 2)");

        char* tmp = reinterpret_cast<char*>(&actionParameter);
        buffer[bufferOffset + 0] = static_cast<char>(action);
        buffer[bufferOffset + 1] = tmp[0];
        buffer[bufferOffset + 2] = tmp[1];
    }
};
#endif // H_H_TRANSMISSION_H_H
