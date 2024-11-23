#include "utils.hpp"

int16_t scale(int16_t value, int16_t old_min, int16_t old_max, int16_t new_min, int16_t new_max) {
    return int(new_min + (value - old_min) * (new_max - new_min) / (old_max - old_min));
}