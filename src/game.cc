#include "game.h"

#include <vector>
#include <unistd.h>

#include <opencv2/core.hpp>

// robot coordinate로 들어온 직교좌표를 로봇 관절의 각도로 바꿈.
void cartesian2angular(Target& p, std_msgs::UInt16MultiArray& msg)
{
    // needed to 0 <= joint <= 4096 (XH, XM) or 0 <= joint <= 1024 (AX)
    // e1: end effector to end of 3rd joint, h1 : floor to pen center
	double d1 = 120.75 + 5.2, L2 = 200.5, L3 = 191.5, e1 = 37.5, h1 = 59.4;

	double p_x, p_y, p_z;
	double q1, q2, q3;
	// q1
	if (p.x > 0)
		q1 = atan((double)p.y / (double)p.x);
	else if (p.x < 0)
		q1 = atan((double)p.y / (double)p.x) + CV_PI;
	else
		if (p.y > 0)
			q1 = CV_PI / 2;
		else
			q1 = -CV_PI / 2;

	p_x = p.x - e1 * cos(q1);
	p_y = p.y - e1 * sin(q1);
	p_z = p.z + h1;

	// q3
	q3 = -acos((p_x * p_x + p_y * p_y + (p_z - d1) * (p_z - d1) - L2 * L2 - L3 * L3) / (2 * L2 * L3));

	// q2
	cv::Mat q2_vec_sol = (cv::Mat_<double>(2, 2) << L2 + L3 * cos(q3), -L3 * sin(q3), L3 * sin(q3), L2 + L3 * cos(q3));
	cv::Mat q2_vec = q2_vec_sol.inv() * (cv::Mat_<double>(2, 1) << cos(q1) * p_x + sin(q1) * p_y, p_z - d1);
	q2 = atan(q2_vec.at<double>(1) / q2_vec.at<double>(0));

	msg.data.push_back((uint16_t)round((180 / CV_PI) * (q1) / 0.0879));
	msg.data.push_back((uint16_t)round((180 / CV_PI) * (q2 + CV_PI / 2) / 0.0879));
	msg.data.push_back((uint16_t)round((180 / CV_PI) * (q3 + CV_PI) / 0.0879));
	msg.data.push_back((uint16_t)round((180 / CV_PI) * (-(q2 + q3) + 5 * CV_PI / 6) / 0.293));
}

GameManager::GameManager(ros::NodeHandle& nodeHandler_, Frame* frame_)
        : turn(RED), nodeHandler(nodeHandler_), pframe(frame_)
{
    pub = nodeHandler.advertise<std_msgs::UInt16MultiArray>("/arm/control_signal", 100);
}

void GameManager::Run()
{
    isrunning = true;
    while (isrunning)
    {
        Stone winner;
        // Run for until a game ends.
        for (countturn = 0; !(winner = pboard->iswinner()) && !isrunning; countturn++, turn=!(turn-1)+1)
        {
            if (turn == RED)
            { // Done by human
                while (!isupdated);
                usleep(1000*1000); // 두 개 이상의 input 검출
                
                unique_lock<mutex> lock(muserinput);
                {
                    if (isfoul) // If the user put more than 2 stones
                    {
                        cerr << "Foul is detected!! Must the user put stones twice."
                        // 여기서 반칙 처리 할 것.
                        break;
                    }
                    else
                        isupdated = false;
                }
                
            }
            else
            { // Prediction by AI
                isdetecting = false;

                int x, y;
                Target targetpt, movingpt;
                // pai->predict(&x, &y);
                // base coord -> paper coord
                pboard->updateAIAction(x, y, targetpt.px, targetpt.py);
                // paper coord -> global coord
                pframe->paper2globalTF(targetpt.px, targetpt.py, targetpt.x, targetpt.y);
                // put a stone at the global coord
                putStone(&targetpt);

                if (isupdated) // 로봇 두기도 전에 두 번 이상 둬버렸다면 반칙
                    isfoul = true;
                isdetecting = true; // 충분히 로봇이 범위를 벗어났을 때
            }            
        }
        if (!isrunning) break;
        
        turn = !(winner-1)+1;
        cout << "The winner is the " << (winner == RED)?"user":"computer" << endl;
    }
}

void GameManager::Halt()
{
    isrunning = false;
}

void GameManager::userInput()
{
    {
        unique_lock<mutex> lock(muserinput);
        if (isupdated)
            isfoul = true;
        else
            isupdated = true;
    }  
}

// Move to target point
void moveTo(std_msgs::UInt16MultiArray* msg, Target* targetpt)
{
    cartesian2angular(*targetpt, *msg);
    pub.publish(*msg);
}

// Point at target point
void pointHere(std_msgs::UInt16MultiArray* msg, Target* targetpt, ros::Rate loop_rate)
{
    int height = targetpt->z;
    // Boom Down
    for (; targetpt->z > 8; targetpt->z--, loop_rate.sleep())
        moveTo(msg, targetpt);
    // Boom Up
    for (; targetpt->z < height; targetpt->z++, loop_rate.sleep())
        moveTo(msg, targetpt);
}

void GameManager::putStone(Target* targetpt)
{
    // Control robot here.
    std_msgs::UInt16MultiArray msg;
    // Move to target point
    moveTo(&msg, targetpt);
    // Point at target point
    pointHere(&msg, targetpt, loop_rate);
    // Move back to origin (0, 300, 50)
    moveTo(&msg, origin);
}