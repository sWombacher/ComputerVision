#ifndef H_H_TRANSMISSION_H_H
#define H_H_TRANSMISSION_H_H
#pragma once
#include <inttypes.h>
struct Transmission {
    enum class Action : int8_t {
        NO_ACTION = 0,

        // optional features
        TELEPORT, MULTIPLE_ACTIONS,

        // actual commands
        MOVE_FORWARD, MOVE_LEFT, MOVE_RIGHT, MOVE_BACKWARD,
        ROTATE_LEFT, ROTATE_RIGHT,
        HEAD_LEFT, HEAD_RIGHT, HEAD_UP, HEAD_DOWN
    };

    Action  action;
    int16_t actionParameter;
    int writeData(int8_t* buffer, int bufferOffset){
        static_assert(sizeof(action) == 1, R"(Byte size of "Action" is greater than 1)");
        static_assert(sizeof(actionParameter) == 2, R"(Byte size of "ActionParameter" is not equel to 2)");

        char* tmp = reinterpret_cast<char*>(&actionParameter);
        buffer[bufferOffset + 0] = static_cast<char>(action);
        buffer[bufferOffset + 1] = tmp[0];
        buffer[bufferOffset + 2] = tmp[1];
        return 3;
    }
};
#endif // H_H_TRANSMISSION_H_H
