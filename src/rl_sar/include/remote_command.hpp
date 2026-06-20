#pragma once

#include <cstdint>

enum class RemoteCommand : uint8_t {
    NONE = 0,
    RECOVER_STAND = 1,
    SELECT_LOCOMOTION = 2,
    START_DREAMWAQ = 3,
    DISABLE = 4,
    DAMPING = 5,
    TOGGLE_RECORD = 6,
};
