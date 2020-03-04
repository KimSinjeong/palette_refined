#include "ai.h"
#include "utils.h"

tagNode::tagNode()
{
    matrix = new Stone*[bsize];
    for (int i = 0; i < bsize; i++)
    {
        matrix[i] = new Stone[bsize];
        memset(matrix[i], 0, sizeof(Stone) * bsize);
    }
    degree = 0;
	score = -2;
	node = -1;
    child = NULL;
}

tagNode::tagNode(Stone** status)
{
    matrix = new Stone*[bsize];
    for (int i = 0; i < bsize; i++)
    {
        matrix[i] = new Stone[bsize];
        memcpy(matrix[i], status[i], sizeof(Stone) * bsize);
    }
    degree = 0;
	score = -2;
	node = -1;
    child = NULL;
}

tagNode::~tagNode()
{
    for (int i = 0; i < bsize; i++)
        delete[] matrix[i];

    delete[] matrix;
}

AI::AI(Board* pboard_, int bsize_) : pboard(pboard_), bsize(bsize_)
{
    tagNode::bsize = bsize;



}