#pragma once

#include <stdint.h>

#pragma pack(push, 1)

typedef struct {
    uint8_t base_to_car_heartbeat;
    uint8_t track_flag;
    uint8_t veh_flag;
    uint8_t veh_rank;
    uint8_t lap_count;
    float lap_distance;
    uint8_t round_target_speed;

    uint8_t laps;
    float lap_time;
    float time_stamp;
} RaceControl;

#pragma pack(pop)
