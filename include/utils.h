#ifndef UTILS_H
#define UTILS_H

using namespace std;

enum Stone 
{
    EMPTY,
    RED,
    BLACK
};

bool isWinner(Stone, Stone**, int);



#endif