/**********************************************************************
/　作成者：Shogo Endo
　 作成日：2020/01/18
/ 
***********************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <string>
#include <sstream>
#include <time.h>

class joy_listen
{
public:
	joy_listen();
	~joy_listen();

private:
	void joyCallback(const sensor_msgs::Joy& joy); // sensor_msgs::Joyというのがメッセージの型
	ros::Subscriber sub;  // サブスクライバの宣言
	ros::Publisher twist_pub; //パブリッシャの宣言
	geometry_msgs::Twist twist;
	ros::NodeHandle nh;  // ノードの宣言
	int L1_button;
	int R1_button;
	double rotation_speed;
	double straight_speed;
};

// サブスクライバを宣言するコンストラクタ
joy_listen::joy_listen()
{
	// joyノードをサブスクライブするサブスクライバであると宣言。Callback関数としてjoy_listen::joyCallback関数を呼ぶ。
	sub = nh.subscribe("joy", 10, &joy_listen::joyCallback, this);
	twist_pub = nh.advertise<geometry_msgs::Twist>("/dtw_robot/diff_drive_controller/cmd_vel", 1);
}

joy_listen::~joy_listen()
{
	; // デストラクタは何もしない
}
// サブスクライブしたときに呼ばれるCallback関数
void joy_listen::joyCallback(const sensor_msgs::Joy& joy)
{
        rotation_speed = 1.0 * joy.axes[0];
	straight_speed = 0.55 * joy.axes[4];
	L1_button = joy.buttons[4];
	R1_button = joy.buttons[5];	
	//送信するデータを書き込む。
	if(L1_button == 1 && R1_button == 0)
	{
		twist.linear.x = 0.0;
    		twist.linear.y = 0.0;
    		twist.linear.z = 0.0;
    		twist.angular.x = 0.0;
    		twist.angular.y = 0.0;
    		twist.angular.z = rotation_speed;
    		twist_pub.publish(twist);
	}
	else if(L1_button == 0 && R1_button == 1)
	{
		twist.linear.x = straight_speed;
    		twist.linear.y = 0.0;
    		twist.linear.z = 0.0;
    		twist.angular.x = 0.0;
    		twist.angular.y = 0.0;
    		twist.angular.z = 0.0;
    		twist_pub.publish(twist);
	}
	else if(L1_button == 1 && R1_button == 1)
	{
		twist.linear.x = straight_speed;
    		twist.linear.y = 0.0;
    		twist.linear.z = 0.0;
    		twist.angular.x = 0.0;
    		twist.angular.y = 0.0;
    		twist.angular.z = rotation_speed;
    		twist_pub.publish(twist);
	}
	else
	{
		twist.linear.x = 0.0;
    		twist.linear.y = 0.0;
    		twist.linear.z = 0.0;
    		twist.angular.x = 0.0;
    		twist.angular.y = 0.0;
    		twist.angular.z = 0.0;
    		twist_pub.publish(twist);
	}

	
}

int main(int argc, char** argv)
{
	char* data;
	ros::init(argc, argv, "joy_listen_node");  // ノードの宣言
	joy_listen joy1;  // オブジェクトの宣言
	ros::Rate rate(30.0);

	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
}
