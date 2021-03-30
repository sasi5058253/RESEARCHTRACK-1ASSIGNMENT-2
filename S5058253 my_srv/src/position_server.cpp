#include "ros/ros.h"
#include "my_srv/Position.h"

double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }


bool myrandom (my_srv::Position::Request &req, my_srv::Position::Response &res){
	res.x = randMToN(-6, 6);
        res.y = randMToN(-6, 6);
    return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_server");
  ros::NodeHandle n;
  ros::ServiceServer service= n.advertiseService("/position",myrandom);
  ros::spin();
  return 0;
}
