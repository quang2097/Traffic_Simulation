#ifndef VEHICLE_HPP
#define VEHICLE_HPP
#include <vector>

enum Vehicle_Type{
    BIKE, MOTORBIKE, CAR, TRUCK, BUS
};

struct Vehicle_Size{
    float Size_Horizontal;
    float Size_Vertical;
};

enum Direction{
    NORTH = 1,
    NORTH_EAST = 1<<1,
    SOUTH_EAST = 1<<2,
    SOUTH = 1<<3,
    SOUTH_WEST = 1<<4,
    WEST = 1<<5,
    NORTH_WEST = 1<<6
};

struct Vehicle{
    Vehicle_Type Type;
    float Speed;
    Vehicle_Size Size;
    std::vector<Direction> Path;
    Vehicle(const Vehicle_Type TYPE, const float SPEED, const Vehicle_Size SIZE, const std::vector<Direction> PATH){
        Type = TYPE;
        Speed = SPEED;
        Size = SIZE;
        Path = PATH;
    }
};
#endif