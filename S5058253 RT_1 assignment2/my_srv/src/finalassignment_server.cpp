#include "ros/ros.h"
#include "my_srv/Finalassignment.h"

double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }


bool myrandom (my_srv::Finalassignment::Request &req, my_srv::Finalassignment::Response &res){
     res.target_index = randMToN(req.min, req.max);
     return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "finalassignment_server");
  ros::NodeHandle n;
  ros::ServiceServer service= n.advertiseService("/finalassignment",myrandom);
  ros::spin();
  return 0;
}
