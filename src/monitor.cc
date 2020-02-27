#include "monitor.h"

#include <string>
#include <vector>
#include <opencv2/aruco.hpp>
#include <predefined_diectionaries.hpp>

using namespace cv;

Mat dictionary = Mat(250, (6 * 6 + 7) / 8, CV_8UC4, (uchar*)DICT_6X6_1000_BYTES);

Mat getByteListFromBits(const Mat& bits)
{
	// integer ceil
	int nbytes = (bits.cols * bits.rows + 8 - 1) / 8;

	Mat candidateByteList(1, nbytes, CV_8UC1, Scalar::all(0));
	unsigned char currentBit = 0;
	int currentByte = 0;

	uchar* rot0 = candidateByteList.ptr();
	for (int row = 0; row < bits.rows; row++) {
		for (int col = 0; col < bits.cols; col++) {
			// circular shift
			rot0[currentByte] <<= 1;

			// set bit
			rot0[currentByte] |= bits.at<uchar>(row, col);
			currentBit++;
			if (currentBit == 8) {
				// next byte
				currentBit = 0;
				currentByte++;
			}
		}
	}
	return candidateByteList;
}

// Hamming distance를 이용해 어떤 QR코드인지 알아내는 함수
bool identify(const Mat& onlyBits, int& idx, int& rotation)
{
	int markerSize = 6;
	//비트 매트릭스를 바이트 리스트로 변환합니다. 
	Mat candidateBytes = getByteListFromBits(onlyBits);
	idx = -1; // by default, not found
	//dictionary에서 가장 근접한 바이트 리스트를 찾습니다. 
	int MinDistance = markerSize * markerSize + 1;
	rotation = -1;
	for (int m = 0; m < dictionary.rows; m++) {
		//각 마커 ID
		for (unsigned int r = 0; r < 4; r++) {
			int currentHamming = hal::normHamming(
				dictionary.ptr(m) + r * candidateBytes.cols,
				candidateBytes.ptr(),
				candidateBytes.cols);

			//이전에 계산된 해밍 거리보다 작다면 
			if (currentHamming < MinDistance) {
				//현재 해밍 거리와 발견된 회전각도를 기록합니다. 
				MinDistance = currentHamming;
				rotation = r;
				idx = m;
			}
		}
	}
	//idx가 디폴트값 -1이 아니면 발견된 것
	return idx != -1;
}


// Frame에서 marker 후보군을 찾아내 다각형으로 근사하는 함수
void approx2Polynomials(const Mat& frame, std::vector<std::vector<Point2f>>& marker)
{
    // find contours on the image
    std::vector<std::vector<Point>> contours;
	findContours(frame, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
	
    // 그냥 contour 구해서 사각형인거만 저장
	std::vector<Point2f> approx;
    for (size_t i = 0; i < contours.size(); i++)
    {
        //contour를 근사화한다.
        approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.05, true);
        if (
            approx.size() == 4 && //사각형은 4개의 vertex를 가진다. 
            fabs(contourArea(Mat(approx))) > 1000 && //면적이 일정크기 이상이어야 한다.
            fabs(contourArea(Mat(approx))) < 50000 && //면적이 일정크기 이하여야 한다. 
            isContourConvex(Mat(approx))) //convex인지 검사한다.
        {
            std::vector<Point2f> points;
            for (int j = 0; j < 4; j++)
                points.push_back(Point2f(approx[j].x, approx[j].y));

            //반시계 방향으로 정렬
            Point v1 = points[1] - points[0];
            Point v2 = points[2] - points[0];

            double o = (v1.x * v2.y) - (v1.y * v2.x); //벡터 외적해서 시계방향인지 판단, 음수면 반시계방향이라는 거라서 1번, 3번 점 을 바꿔준다.
            if (o < 0.0)
                swap(points[1], points[3]);
            marker.push_back(points);
        }
    }
}

void decodePattern2Bits(const Mat& frame, std::vector<std::vector<Point2f>>& marker,
        std::vector<std::vector<Point2f>>& detectedMarkers, std::vector<Mat>& bitMatrices,
        std::vector<Mat>& perspectiveTF)
{
    //마커 6x6크기일때 검은색 테두리 영역 포함한 크기는 8x8
    //이후 단계에서 이미지를 격자로 분할할 시 셀하나의 픽셀너비를 10으로 한다면
    //마커 이미지의 한변 길이는 80
    int marker_image_side_length = 80;
    
    std::vector<Point2f> square_points;
    square_points.push_back(Point2f(0, 0));
    square_points.push_back(Point2f(marker_image_side_length - 1, 0));
    square_points.push_back(Point2f(marker_image_side_length - 1, marker_image_side_length - 1));
    square_points.push_back(Point2f(0, marker_image_side_length - 1));

    Mat marker_image;
    for (int i = 0; i < marker.size(); i++)
    {
        //마커를 사각형형태로 바꾸는 perspective transformation matrix를 적용한다.
        Mat PerspectiveTransformMatrix = getPerspectiveTransform(marker[i], square_points);
        warpPerspective(frame, marker_image, PerspectiveTransformMatrix, Size(marker_image_side_length, marker_image_side_length));
        //otsu 방법으로 이진화를 적용한다. 
        threshold(marker_image, marker_image, 125, 255, THRESH_BINARY | THRESH_OTSU);
        //마커의 크기는 6, 검은색 태두리를 포함한 크기는 8
        //마커 이미지 테두리만 검사하여 전부 검은색인지 확인한다. 
        int cellSize = marker_image.rows / 8;
        int white_cell_count = 0;
        for (int y = 0; y < 8; y++)
        {
            int inc = 7; // 첫번째 열과 마지막 열만 검사하기 위한 값
            if (y == 0 || y == 7) inc = 1; //첫번째 줄과 마지막줄은 모든 열을 검사한다. 
            for (int x = 0; x < 8; x += inc)
            {
                int cellX = x * cellSize;
                int cellY = y * cellSize;
                Mat cell = marker_image(Rect(cellX, cellY, cellSize, cellSize));
                int total_cell_count = countNonZero(cell);
                if (total_cell_count > (cellSize * cellSize) / 2)
                    white_cell_count++; //태두리에 흰색영역이 있다면, 셀내의 픽셀이 절반이상 흰색이면 흰색영역으로 본다 
            }
        }

        //검은색 태두리로 둘러쌓여 있는 것만 체크한다.
        if (white_cell_count == 0) {
            detectedMarkers.push_back(m);

            // 내부 6x6에 있는 정보를 비트로 저장하기 위한 변수
            Mat bitMatrix = Mat::zeros(6, 6, CV_8UC1);
            int cellSize = marker_image.rows / 8;

            for (int y = 0; y < 6; y++)
            {
                for (int x = 0; x < 6; x++)
                {
                    int cellX = (x + 1) * cellSize;
                    int cellY = (y + 1) * cellSize;
                    Mat cell = marker_image(Rect(cellX, cellY, cellSize, cellSize));

                    int total_cell_count = countNonZero(cell);
                    if (total_cell_count > (cellSize * cellSize) / 2)
                        bitMatrix.at<uchar>(y, x) = 1;
                }
            }
            bitMatrices.push_back(bitMatrix);
            perspectiveTF.push_back(PerspectiveTransformMatrix);
        }
    }
}

void matchMarkersID(std::vector<std::vector<Point2f>>& detectedMarkers, std::vector<Mat>& bitMatrice
        std::vector<int>& markerID, std::vector<std::vector<Point2f>>& validMarkers, std::vector<Mat>& perspectiveTF)
{
    int ptfIdx = 0;
    for (int i = 0; i < detectedMarkers.size(); i++)
    {
        std::vector<Point2f>& m = detectedMarkers[i];

        int rotation;
        int marker_id;

        if (identify(bitMatrices[i], marker_id, rotation)) {
            //회전을 고려하여 코너를 정렬합니다. 
            //마커의 회전과 상관없이 마커 코너는 항상 같은 순서로 저장됩니다.
            if (rotation != 0) std::rotate(m.begin(), m.begin() + 4 - rotation, m.end());
            markerID.push_back(marker_id);
            validMarkers.push_back(m);
            ptfIdx++;
        }
        else perspectiveTF.erase(vec.begin() + ptfIdx);
    }
}

void Monitor::Run()
{
    // Keep the position of the most current stone.
    // Update the board when needed. (setting the mutex)
    isrunning = true;

    while(isrunning)
    {
        Mat paperQR2Rect;
        std::vector<cv::Point2f> pBlob(4);
        {
            std::unique_lock<std::mutex> lock(pframe->imageframe.mframe);
            Mat imageframe = (pframe->imageframe.getFrame()).clone();
        }
        // There are proper number of markers and corners of Go board;
        //  the order of two evaluations are important!
        // TODO: 첫번째 조건으로 AI 활동 여부 등의 조건을 나타내는 boolean이 들어가야 함. (pgame->ismonitoractive)
        if (isMarker(imageframe, paperQR2Rect) && isCorner(imageframe, paperQR2Rect, pBlob))
        {
            // Set img <-> paper <-> global relations
            pframe->calculateRelations(pBlob);
            // Hough transform for line detection
            if (!pboard->houghDetection()) break;
            // Find intersections of lines (candidates of go stones' location)
            pboard->findIntersections();
            // Find the most recent stone of user.
            if(pboard->dotDetection()){
                // TODO: 여기에서 game thread에 user action update signal 보내기.
		        // game thread에서 뭔가 처리중에 그 signal이 들어오면 반칙처리.
                /*
                unique_lock<mutex> lock(<<lock 이름 넣기>>);
                pgame->userupdated++;
                 */
            }
        }
        else std::cerr << "There are too many/less markers/corners on image." << std::endl;
        
    } // 한 판이 끝나면 종료된다.
}

// 외부에서 이 함수가 불리면 Monitor가 종료된다.
void Monitor::Halt()
{
    isrunning = false;
}

// If proper number of markers are detected, return true.
// Otherwise, return false.
bool Monitor::isMarker(Mat imageframe, Mat& paperQR2Rect)
{
    // Imageframe은 grayscale이라 가정
    if (imageframe.empty()) return false;

    // adaptive threshold를 적용해서 이진화
    Mat binary_image;
    adaptiveThreshold(imageframe, binary_image, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 91, 7);

    // marker 후보군을 polynomial로 근사.
    std::vector<std::vector<Point2f>> marker;
    approx2Polynomials(imageframe, marker);
    
    if (marker.size() < 2) return false;
    
    // 테두리가 모두 검정색인 마커만 bit로 decode.
    std::vector<std::vector<Point2f>> detectedMarkers;
    std::vector<Mat> bitMatrices;
    std::vector<Mat> perspectiveTF;
    decodePattern2Bits(imageframe, marker, detectedMarkers, bitMatrices, perspectiveTF);

    if (detectedMarkers.size() < 2) return false;

    // Marker validation 및 validMarkers에 시계방향 순서로 점을 저장.
    std::vector<int> markerID;
    std::vector<std::vector<Point2f>> validMarkers;
    matchMarkersID(detectedMarkers, bitMatrices, validMarkers, markerID, perspectiveTF);

    if (validMarkers.size() != 2) return false;

    // Assign valid Markers' coordinates to proper frames.
    for (int i = 0; i < validMarkers.size(); i++)
    {
        if (markerID[i] == 0) {
            {
                std::unique_lock<std::mutex> lock(pframe->imageframe.mmarker);
                (pframe->imageframe.getFrame()).setMarker(validMarkers[i]);
            }
            paperQR2Rect = perspectiveTF[i].clone();
        }
        else {
            std::unique_lock<std::mutex> lock(pframe->paperframe.mmarker);
            (pframe->paperframe.getFrame()).setMarker(validMarkers[i]);
        }
    }

    // Print markers' indices on image frame
    /* imageframe을 다른 표시용 프레임으로 바꿔야 한다.
    {
        unique_lock<mutex> lock("<<NAME_OF_MUTEX>>");
        for (int i = 0; i < validMarkers.size(); i++) {
            int sumx = 0, sumy = 0;
            for (int j = 0; j < 4; j++) {
                putText(imageframe, to_string(j + 1), Point(validMarkers[i][j].x, validMarkers[i][j].y),
                        QT_FONT_NORMAL, 1, Scalar(255, 0, 0), 1, 1);
                sumx += validMarkers[i][j].x;
                sumy += validMarkers[i][j].y;
            }
            putText(imageframe, "id=" + to_string(markerID[i]),
                    Point(sumx / 4, sumy / 4), QT_FONT_NORMAL, 1, Scalar(255, 0, 0), 1, 1);
            cornerSubPix(imageframe, validMarkers[i], Size(5, 5), Size(-1, -1),
                    TermCriteria(TermCriteria::MAX_ITER | TermCriteria::EPS, 30, 0.01));
        }
    }
    */

    return true;
}

bool Monitor::isCorner(Mat imageframe, const Mat paperQR2Rect, std::vector<cv::Point2f>& pBlob)
{
    // Blob detection
    SimpleBlobDetector::Params params;
    params.minThreshold = 100;
    //params.maxThreshold = 220;
    params.filterByArea = true;
    params.minArea = 40;
    params.maxArea = 300;
    params.filterByColor = true;
    params.blobColor = 0; // 어두운 얼룩 추출 : 0, 밝은 얼룩 추출 : 255
    params.filterByCircularity = false;

    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
    std::vector<KeyPoint> keypoints;
    detector->detect(imageframe, keypoints);

    // 이 부분은 맨 뒤로 가야 함
    // drawKeypoints(imageframe, keypoints, imageframe, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    if (keypoints.size() < 3) return false;

    std::vector<Point2f> ptf(keypoints.size());
    for (int i = 0; i < keypoints.size(); i++)
        ptf[i] = keypoints[i].pt;
    perspectiveTransform(ptf, ptf, paperQR2Rect);
    
    Point2f p_x_blob, p_y_blob, p_xy_blob;
    float xval = -1.0f;
    float yval = -1.0f;
    float crossval = -1.0f;

    for (int i = 0; i < keypoints.size(); i++) {
        Point2f keyP = ptf[i];
        if (abs(keyP.dot(Point2f(1.0, 0.0))) > xval) {
            xval = abs(keyP.dot(Point2f(1.0, 0.0)));
            p_x_blob = keypoints[i].pt;
        }
        if (abs(keyP.dot(Point2f(0.0, 1.0))) > yval) {
            yval = abs(keyP.dot(Point2f(0.0, 1.0)));
            p_y_blob = keypoints[i].pt;
        }
        if (abs(keyP.dot(Point2f(1.0, 1.0))) > crossval) {
            crossval = abs(keyP.dot(Point2f(1.0, 1.0)));
            p_xy_blob = keypoints[i].pt;
        }
    }
    
    pBlob[1] = p_x_blob; pBlob[2] = p_xy_blob; pBlob[3] = p_y_blob;

    return true;
}