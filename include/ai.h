#ifndef AI_H
#define AI_H

#include "board.h"

class tagNode {
public:
    tagNode();
    tagNode(Stone** status);
    ~tagNode();
	Stone** matrix;
	int degree;
	int score;
	int node; //for minimax algorithm
	tagNode* child;

    static int bsize;
};

int tagNode::bsize;

struct tagTree {
	tagNode* root;
};

class AI
{
public:
    AI(Board* pboard_, int bsize_);

    void train();
    void predict(int& x, int& y);

private:
    Board* pboard;
    int bsize;
};



#endif