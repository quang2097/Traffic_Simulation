#ifndef TRAFFIC_TABLE
#define TRAFFIC_TABLE
#include <array>

enum Block_Type{
    HOUSING = 1,
    ROAD = 1<<1,
    TRAFFIC_LIGHT = 1<<2
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

enum Vehicle_Type{
    BIKE, MOTORBIKE, CAR, TRUCK, BUS
};

struct Size{
    float Size_Horizontal;
    float Size_Vertical;
};

struct Vehicle{
    Vehicle_Type Type;
    float Speed;
    Direction Path;
    Size Vehicle_Size;
    Vehicle(const Vehicle_Type TYPE, const float SPEED, const Direction PATH, const Size SIZE){
        Type = TYPE;
        Speed = SPEED;
        Path = PATH;
        Size = SIZE;
    }
};

enum Road_Type{
    ONE_WAY,
    TWO_WAY
};

struct Block{
    Block_Type Type;
    int Paths;
    Road_Type Road;
    int Number_Of_People;
    vector<Size> Road_Size;
    Block(const Block_Type TYPE, const int PATHS, const Road_Type ROAD){
        Type = TYPE;
        Paths = PATHS;
        Road = ROAD;
    }
};




#endif