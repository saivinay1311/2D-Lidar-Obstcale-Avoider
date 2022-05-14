#include "ros/ros.h"
#include "obstacle_avoider/custom.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

using namespace std;

obstacle_avoider::custom pub_msg;

geometry_msgs::Twist message;
ros::Publisher pub;
ros::Publisher custom_pub;
string status = "";
float linear_x = 0;
float angular_z = 0;
int counter = 0;
int obstacle_counter;
bool center_obsctacle;
bool right_obsctacle;
bool left_obsctacle;
float dist_arr[3];
float start_angles_arr[3];

float start_angle_center;
float start_angle_right;
float start_angle_left;

float* insertX(int n, float arr[],
             float x, int pos)
{
    int i;
  
    // increase the size by 1
    n++;
  
    // shift elements forward
    for (i = n; i >= pos; i--)
        arr[i] = arr[i - 1];
  
    // insert x at pos
    arr[pos - 1] = x;
  
    return arr;
}


int noOfObstacles(float arr1[], float arr2[], float arr3[])
{

  for (int i = 0; i < 240; i++)
  {
    if (arr3[i] < 5)
    {
      
      center_obsctacle = true;
      insertX(3,dist_arr,arr3[i],0);
      start_angle_center = (((i+240)*0.0043)*180)/1.57;
      insertX(3,start_angles_arr,start_angle_center,0);
      

      
    }
    if (arr1[i] < 5)
    {

      right_obsctacle = true;
      insertX(3,dist_arr,arr1[i],1);
      start_angle_right = (((i)*0.0043)*180)/1.57;
      insertX(3,start_angles_arr,start_angle_right,1);

    }
    if (arr2[i] < 5)
    {

      left_obsctacle = true;
      insertX(3,dist_arr,arr2[i],2);
      start_angle_left = (((i+480)*0.0043)*180)/1.57;
      insertX(3,start_angles_arr,start_angle_left,2);
    }
    
  }
  if (center_obsctacle)
  {
    counter = 1;
  }
   if (left_obsctacle)
  {
    counter = 1;
  }
   if (right_obsctacle)
  {
    counter = 1;
  }
   if (center_obsctacle and left_obsctacle)
  {
    counter = 2;
  }
  if (center_obsctacle and right_obsctacle)
  {
    counter = 2;
  }
  if (right_obsctacle and left_obsctacle)
  {
    counter = 2;
  }
  if (center_obsctacle and right_obsctacle and left_obsctacle)
  {
    counter = 3;
  }

  return counter;
  center_obsctacle = false;
  right_obsctacle = false;
  left_obsctacle = false;
  counter = 0;
}

float minValue(float arr[])
{
  float temp = 2.0;
  for (int i = 0; i < 240; i++)
  {
    if (temp > arr[i])
    {
      temp = arr[i];
    }
  }
  return temp;
}
void checkcase(float right, float center, float left)
{
  if (right > 1 and center > 1 and left > 1)
  {
    status = "NO OBSTACLE!";
    linear_x = 0.6;
    angular_z = 0;
  }
  else if (right > 1 and center < 1 and left > 1)
  {
    status = "OBSTACLE CENTER!";
    linear_x = 0;
    angular_z = -0.5;
  }
  else if (right < 1 and center > 1 and left > 1)
  {
    status = "OBSTACLE RIGHT!";
    linear_x = 0;
    angular_z = 0.5;
  }
  else if (right > 1 and center > 1 and left < 1)
  {
    status = "OBSTACLE LEFT!";
    linear_x = 0;
    angular_z = -0.5;
  }
  else if (right < 1 and center > 1 and left < 1)
  {
    status = "OBSTACLE RIGHT AND LEFT!";
    linear_x = 0.6;
    angular_z = 0;
  }
  else if (right > 1 and center < 1 and left < 1)
  {
    status = "OBSTACLE CENTER AND LEFT!";
    linear_x = 0;
    angular_z = -0.5;
  }
  else if (right < 1 and center < 1 and left > 1)
  {
    status = "OBSTACLE CENTER AND RIGHT!";
    linear_x = 0;
    angular_z = 0.5;
  }
  else if (right < 1 and center < 1 and left < 1)
  {
    status = "OBSTACLE AHEAD!";
    linear_x = 0;
    angular_z = 0.8;
  }

  message.linear.x = linear_x;
  message.angular.z = angular_z;
  
  pub_msg.obstcale_distances.data.push_back(dist_arr[0]);
  pub_msg.obstcale_distances.data.push_back(dist_arr[1]);
  pub_msg.obstcale_distances.data.push_back(dist_arr[2]);

  pub_msg.start_angles.data.push_back(start_angles_arr[0]);
  pub_msg.start_angles.data.push_back(start_angles_arr[1]);
  pub_msg.start_angles.data.push_back(start_angles_arr[2]);


  pub_msg.no_of_obstcales.data = obstacle_counter;
  pub_msg.status.data = status;
  pub.publish(message);
  custom_pub.publish(pub_msg);
   pub_msg.obstcale_distances.data.clear();
   pub_msg.start_angles.data.clear();
  

  cout << "  " << status;
  cout << "  " << right;
  cout << "  " << center;
  cout << "   " << left;
  cout << "   "        << "Obstacles : " << obstacle_counter;
  cout << "\n";
  cout <<"Distances bewtween obstacles are :" << dist_arr[0] << " " << dist_arr[1] << " " << dist_arr[2] << " " << "\n" ;
  cout <<"Start angles of obstacles are :" << start_angles_arr[0] << " " << start_angles_arr[1] << " " << start_angles_arr[2] << " " << "\n" ;
  obstacle_counter = 0;
}

void LaserCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  int len = msg->ranges.size();
  int iterator = len / 3;
  float right_reg[iterator], center_reg[iterator], left_reg[iterator];
  for (int i = 0; i < len; i++)
  {
    if (i < iterator)
    {
      right_reg[i] = msg->ranges[i];
    }
    else if ((i > iterator - 1) && (i < 2 * iterator))
    {
      center_reg[i - 240] = msg->ranges[i];
    }
    else if (((2 * iterator) - 1) && (i < len))
    {
      left_reg[i - 480] = msg->ranges[i];
    }
  }
  float min_val_left = minValue(left_reg);
  float min_val_right = minValue(right_reg);

  float min_val_center = minValue(center_reg);

  obstacle_counter = noOfObstacles(right_reg, left_reg, center_reg);

  checkcase(min_val_right, min_val_center, min_val_left);

  // float ranges[] = [msg->ranges];
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "topic_subscriber");
  ros::NodeHandle nh;
  pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  custom_pub = nh.advertise<obstacle_avoider::custom>("/scan_data", 10);

  ros::Subscriber sub = nh.subscribe("/laser/scan", 1000, LaserCallBack);

  ros::spin();
  return 0;
}
