#include "board.h"

#include <opencv2/imgproc.hpp>
#include <vector>
#include <algorithm>

using namespace cv;

Board::Board(int size_, int marginX_, int marginY_, int marginE_)
	: size(size_), marginS(marginX_, marginY_), marginE(marginE_, marginE_)
{
	// Allocate memory for grid points and lines
    gridpt = new Upoint*[size];
	for(int i = 0; i < size; i++)
	{
		gridpt[i] = new Upoint[size];
		// Initialize memory space.
		memset(gridpt[i], 0, sizeof(Upoint)*size);
	}

	gridline = new Uline*[size];
	for(int i = 0; i < size; i++)
	{
		gridline[i] = new Uline[size];
		// Initialize memory space.
		memset(gridline[i], 0, sizeof(Uline)*(size));
	}

	recentuser = Point2i(-1, -1);
}

Board::~Board()
{
	// Free the memory
	for(int i = 0; i < size; i++) delete [] gridpt[i];
	delete [] gridpt;

	for(int i = 0; i < size; i++) delete [] gridline[i];
	delete [] gridline;

	while (x_lines.size())
	{
		delete x_lines.back();
		x_lines.pop_back();
	}
	while (y_lines.size())
	{
		delete y_lines.back();
		y_lines.pop_back();
	}
}

inline bool lineComparator(Uline* a, Uline* b) { return abs(a->rho) < abs(b->rho); }

void Board::insert(float rho, float theta)
{
	if (theta > (CV_PI * 3 / 4)) {
		theta = theta - CV_PI;
		rho = -rho;
	}

	Uline* new_line = new Uline { rho, theta };
	if (new_line->theta > (CV_PI / 4)) {
		x_lines.push_back(new_line);
	}
	else {
		y_lines.push_back(new_line);
	}
}

void Board::findOrthogonalLines(std::vector<Uline*>& lines, const double rho_threshold, const double theta_threshold)
{
	if (lines.size() == 0) return;
	std::sort(lines.begin(), lines.end(), lineComparator);

	std::vector<Uline*> new_lines;
	new_lines.push_back(lines.back());
	lines.pop_back();

	while (lines.size() != 0) {
		double disDiff = abs(abs((double)new_lines.back()->rho) - abs((double)lines.back()->rho));
		double slopeDiff = abs((double)new_lines.back()->theta - (double)lines.back()->theta);
		if (slopeDiff > CV_PI / 9) {
			break;
		}
		else if (disDiff < rho_threshold && slopeDiff < theta_threshold) {
			new_lines.back()->rho = (new_lines.back()->rho * new_lines.back()->index + lines.back()->rho) / (new_lines.back()->index + 1);
			new_lines.back()->theta = (new_lines.back()->theta * new_lines.back()->index + lines.back()->theta) / (new_lines.back()->index + 1);
			new_lines.back()->index++;
			lines.pop_back();
		}
		else {
			new_lines.push_back(lines.back());
			lines.pop_back();
		}
	}

	while (new_lines.size() != 0) {
		lines.push_back(new_lines.back());
		new_lines.pop_back();
	}
}

void Board::XYcontour(const double rho_threshold, const double theta_threshold)
{
	findOrthogonalLines(y_lines, rho_threshold, theta_threshold);
	findOrthogonalLines(x_lines, rho_threshold, theta_threshold);
}

bool Board::houghDetection()
{
	Mat img_dilate, dst;
	Canny(baseframe, dst, 40, 150, 3);
	// imshow("canny", dst);
	Mat mask = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1, 1));
	dilate(dst, img_dilate, mask, Point(-1, -1), 2);
	erode(img_dilate, dst, mask, Point(-1, -1), 2);

	// Standard Hough Line Transform
	std::vector<Vec2f> lines; // will hold the results of the detection
	HoughLines(dst, lines, 1, CV_PI / 180, 200); // runs the actual detection
	while (lines.size() != 0) {
		insert(lines.back()[0], lines.back()[1]);
		lines.pop_back();
	}

	double rho_threshold = 5;
	double theta_threshold = 20 * CV_PI / 180;
	XYcontour(rho_threshold, theta_threshold);
	if (x_lines.size() == size && y_lines.size() == size) return true;
	return false;
}

void Board::findIntersections()
{
	float a1, a2, b1, b2;
	//solve the point of intersection of two lines
	for (int i = 0; i < x_lines.size(); i++)
	{
		float rho_i = x_lines[i]->rho, theta_i = x_lines[i]->theta;
		double a = cos(theta_i), b = sin(theta_i);
		double x0_i = a * rho_i, y0_i = b * rho_i;

		a1 = -x0_i / y0_i;
		b1 = y0_i - a1 * x0_i;

		for (int j = 0; j < y_lines.size(); j++) {
			float rho_j = y_lines[j]->rho, theta_j = y_lines[j]->theta;
			double a = cos(theta_j), b = sin(theta_j);
			double x0_j = a * rho_j, y0_j = b * rho_j;

			a2 = -x0_j / y0_j;
			b2 = y0_j - a2 * x0_j;
			gridpt[i][j].coord.x = (b2 - b1) / (a1 - a2);
			points[i][j].coord.y = a1 * gridpt[i][j].coord.x + b1;
		}
	}
}

int Board::findDotIndex(KeyPoint blob)
{
	float dx, dy, distance;
	for (int i = 0; i < 12; i++) {
		for (int j = 0; j < 12; j++) {
			dx = blob.pt.x - gridpt[i][j].coord.x;
			dy = blob.pt.y - gridpt[i][j].coord.y;
			distance = sqrt(pow(dx, 2) + pow(dy, 2));
			if (distance < blob.size) return i * 12 + j;
		}
	}
	return size*size;
}

bool Board::updateUserAction(const int x, const int y)
{
	if (gridpt[x][y].status == EMPTY)
	{
		// gridpt[x][y].status = RED;
		// 다른 작업에 혼선을 주지 않기 위해 실제로 놓는 것은 나중에
		// game thread가 감시자 역할을 수행한다. 반칙이 아닐 경우에만 실제로 놓음
		recentuser.x = x; recentuser.y = y;
		return true;
	}
	return false;
}

void Board::confirmUserAction()
{
	gridpt[recentuser.x][recentuser.y].status = RED;
}

bool Board::dotDetection()
{
	std::vector<KeyPoint> blobPoints;

	Mat contrastframe, tmp;
	Mat mask = getStructuringElement(MORPH_ELLIPSE, Size(3, 3), Point(1, 1));
	erode(baseframe, contrastframe, mask, Point(-1, -1), 1);
	dilate(contrastframe, tmp, mask, Point(-1, -1), 2);
	tmp.convertTo(contrastframe, -1, 2, 0);

	// Parameter setting for detection of go stones
	SimpleBlobDetector::Params params;
	params.minThreshold = 130;
	//params.maxThreshold = 220;
	params.filterByArea = true;
	params.minArea = 40;
	params.maxArea = 80;
	params.filterByColor = true;
	params.blobColor = 0; // dark : 0, bright : 255
	params.filterByCircularity = false;
	//params.minCircularity = 5;
	//params.filterByCircularity = 5;

	// Blob detection
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
	detector->detect(baseframe, blobPoints);

	// 이거 어딘가로 보내서 한번에 보여주기
	// drawKeypoints(contrastframe, blobPoints, frame, Scalar(255, 0, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	// imshow("dot detection", contrastframe);
	int index = size*size;
	int x, y;
	for (int i = 0; i < blobPoints.size(); i++) {
		if ((index = findDotIndex(points, blobPoints.at(i))) != size*size)
		{
			x = index / size;
			y = index % size;
			if (updateUserAction(x, y))
				return true;
		}
	}
	return false;
}

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