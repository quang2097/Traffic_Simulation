#ifndef BLOCK_HPP
#define BLOCK_HPP
enum Block_Type{
    HOUSING = 1,
    ROAD = 1<<1,
    TRAFFIC_LIGHT = 1<<2
};

struct Block_Size{
    float Horizontal;
    float Vertical;
};

struct Block{
    Block_Size Size;
    int Type;
    Block(const Block_Size SIZE, const int TYPE){
        Size = SIZE;
        Type = TYPE;
    }
};
#endif