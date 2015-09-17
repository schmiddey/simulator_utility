
#include "Joy2Vel.h"

Joy2Vel::Joy2Vel() : _rate(0)
{
    //rosParam
    ros::NodeHandle privNh("~");
    std::string sub_name_joy;
    std::string pub_name_vel;
    double rate;
    double max_vel_lin;
    double max_vel_ang;

    
    privNh.param("sub_name_joy",        sub_name_joy,  std::string("joy"));
    privNh.param("pub_name_vel",        pub_name_vel,  std::string("cmd_vel"));
    privNh.param<double>("rate",        rate,          100.0);
    privNh.param<double>("max_vel_lin", max_vel_lin,   1.0);
    privNh.param<double>("max_vel_ang", max_vel_ang,   1.0);

    _max_vel_lin = max_vel_lin;
    _max_vel_ang = max_vel_ang;

    _loopRate = rate;

    //init publisher
    _pubTwist = _nh.advertise<geometry_msgs::Twist>(pub_name_vel,1);

    //inti subscriber
    _subJoy = _nh.subscribe(sub_name_joy, 1, &Joy2Vel::subCallback_joy, this);


    _rdy = false;

}

Joy2Vel::~Joy2Vel()
{
    delete _rate;
}

void Joy2Vel::start()
{
    delete _rate;
    _rate = new ros::Rate(_loopRate);
    this->run();
}

void Joy2Vel::run()
{
    unsigned int cnt = 0;

    while(ros::ok())
    {
       if(_rdy)
       {
          geometry_msgs::Twist msg;

          //13 vor
          //12 back
          //0 rot

          // origin value is negative so * -1
          double forward = _joy.axes[13];
          forward *= forward;
          double back    = _joy.axes[12];
          back *= back;
          back *= -1;

          double rot     = _joy.axes[0];
          int sign = rot < 0 ? -1 : 1;
          rot *= rot;
          rot *= sign;


          double lin = forward + back;

          msg.linear.x   = lin * _max_vel_lin;
          msg.angular.z  = rot * _max_vel_ang;

          //publish data;
          _pubTwist.publish(msg);
       }
        ros::spinOnce();
        _rate->sleep();
    }
}


void Joy2Vel::subCallback_joy(const sensor_msgs::Joy msg)
{
   _rdy = true;
   _joy = msg;


//   for(unsigned int i=0; i<_joy.axes.size(); ++i)
//   {
//      std::cout << i << ":" << _joy.axes[i] << std::endl;
//   }
//
//   std::cout << "------------------------------------------" << std::endl;

}













// ------------- main ---------------
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "NodeName");
    ros::NodeHandle nh("~");

    Joy2Vel node;
    node.start();

}


