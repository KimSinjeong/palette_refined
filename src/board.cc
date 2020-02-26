#include "board.h"

void Board::clearBoard()
{
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
            gridpt[i][j].status = EMPTY;
        }
    }
}

bool Board::isWinner(Stone player)
{
    if (player == EMPTY) {
        std::cerr << "Player is not specified for winner detecting function" << std::endl;
        return false;
    }

	int count = 0;
	int count1 = 0;
	int count2 = 0;
	/* Search up and down */
	for (int j = 0; j < size; j++) {
		for (int i = 0; i < size; i++) {
			if (gridpt[i][j].status == player) count++;
			else count = 0;

			if (count == 5 && i == size-1) return true;
			if (count == 5 && i < size-1 && gridpt[i + 1][j].status != player) return true;
            if (count == 5 && i < size-1 && gridpt[i + 1][j].status == player) return false;
		}
	}


	/* Search left and right */
	count = 0;
	for (int i = 0; i < size; i++) {
		for (int j = 0; j < size; j++) {
			if (gridpt[i][j].status == player) count++;
			else count = 0;

			if (count == 5 && j == size-1) return true;
			if (count == 5 && j < size-1 && gridpt[i][j + 1].status != player) return true;
			if (count == 5 && j < size-1 && gridpt[i][j + 1].status == player) return false;
		}
	}


	/* Search diagonal */
	count1 = 0;
	count2 = 0;
	for (int i = 0; i < size-4; i++) {
		for (int j = 0; j < size - i; j++) {
			if (gridpt[i + j][j].status == player) count1++;
			else count1 = 0;
			if (gridpt[j][i + j].status == player) count2++;
			else count2 = 0;

			if (count1 == 5 && j == size-1 - i) return true;
			if (count1 == 5 && j < size-1 - i && gridpt[i + j + 1][j + 1].status != player) return true;
			if (count1 == 5 && j < size-1 - i && gridpt[i + j + 1][j + 1].status == player) return false;
			if (count2 == 5 && j == size-1 - i) return true;
			if (count2 == 5 && j < size-1 - i && gridpt[j + 1][i + j + 1].status != player) return true;
			if (count2 == 5 && j < size-1 - i && gridpt[j + 1][i + j + 1].status == player) return false;

		}
	}


	/* Search skew diagonal */
	count1 = 0;
	count2 = 0;
	for (int i = size-1; i > 3; i--) {
		for (int j = 0; j < i + 1; j++) {
			if (gridpt[i - j][j].status == player) count1++;
			else count1 = 0;
			if (gridpt[size-1 - j][size-1 - i + j].status == player) count2++;
			else count2 = 0;

			if (count1 == 5 && j == i) return true;
			if (count1 == 5 && j < i && gridpt[i - j - 1][j + 1].status != player) return true;
			if (count1 == 5 && j < i && gridpt[i - j - 1][j + 1].status == player) return false;
			if (count2 == 5 && j == i) return true;
			if (count2 == 5 && j < i && gridpt[size-1 - j - 1][size-1 - i + j + 1].status != player) return true;
			if (count2 == 5 && j < i && gridpt[size-1 - j - 1][size-1 - i + j + 1].status == player) return false;

		}
	}

	return false;
}