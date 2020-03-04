#include "utils.h"

#include <iostream>

// Determine whether the given user is a winner; if so, return true;
bool isWinner(Stone player, Stone** status, int size)
{
    if (player == EMPTY) {
        cerr << "Player is not specified for winner detecting function" << endl;
        return false;
    }

	int count = 0;
	int count1 = 0;
	int count2 = 0;
	/* Search up and down */
	for (int j = 0; j < size; j++) {
		for (int i = 0; i < size; i++) {
			if (status[i][j] == player) count++;
			else count = 0;

			if (count == 5 && i == size-1) return true;
			if (count == 5 && i < size-1 && status[i + 1][j] != player) return true;
            if (count == 5 && i < size-1 && status[i + 1][j] == player) return false;
		}
	}


	/* Search left and right */
	count = 0;
	for (int i = 0; i < size; i++) {
		for (int j = 0; j < size; j++) {
			if (status[i][j] == player) count++;
			else count = 0;

			if (count == 5 && j == size-1) return true;
			if (count == 5 && j < size-1 && status[i][j + 1] != player) return true;
			if (count == 5 && j < size-1 && status[i][j + 1] == player) return false;
		}
	}


	/* Search diagonal */
	count1 = 0;
	count2 = 0;
	for (int i = 0; i < size-4; i++) {
		for (int j = 0; j < size - i; j++) {
			if (status[i + j][j] == player) count1++;
			else count1 = 0;
			if (status[j][i + j] == player) count2++;
			else count2 = 0;

			if (count1 == 5 && j == size-1 - i) return true;
			if (count1 == 5 && j < size-1 - i && status[i + j + 1][j + 1] != player) return true;
			if (count1 == 5 && j < size-1 - i && status[i + j + 1][j + 1] == player) return false;
			if (count2 == 5 && j == size-1 - i) return true;
			if (count2 == 5 && j < size-1 - i && status[j + 1][i + j + 1] != player) return true;
			if (count2 == 5 && j < size-1 - i && status[j + 1][i + j + 1] == player) return false;

		}
	}


	/* Search skew diagonal */
	count1 = 0;
	count2 = 0;
	for (int i = size-1; i > 3; i--) {
		for (int j = 0; j < i + 1; j++) {
			if (status[i - j][j] == player) count1++;
			else count1 = 0;
			if (status[size-1 - j][size-1 - i + j] == player) count2++;
			else count2 = 0;

			if (count1 == 5 && j == i) return true;
			if (count1 == 5 && j < i && status[i - j - 1][j + 1] != player) return true;
			if (count1 == 5 && j < i && status[i - j - 1][j + 1] == player) return false;
			if (count2 == 5 && j == i) return true;
			if (count2 == 5 && j < i && status[size-1 - j - 1][size-1 - i + j + 1] != player) return true;
			if (count2 == 5 && j < i && status[size-1 - j - 1][size-1 - i + j + 1] == player) return false;

		}
	}

	return false;
}