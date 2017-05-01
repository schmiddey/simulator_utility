
#include "Joy2Vel_sc.h"

Joy2Vel_sc::Joy2Vel_sc() : _rate(0)
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
    _subJoy = _nh.subscribe(sub_name_joy, 1, &Joy2Vel_sc::subCallback_joy, this);


    _rdy = false;

}

Joy2Vel_sc::~Joy2Vel_sc()
{
    delete _rate;
}

void Joy2Vel_sc::start()
{
    delete _rate;
    _rate = new ros::Rate(_loopRate);
    this->run();
}

void Joy2Vel_sc::run()
{
    unsigned int cnt = 0;
    ROS_INFO("Start joy2vel_sc_node");

    while(ros::ok())
    {
       if(_rdy)
       {
          geometry_msgs::Twist msg;

          //5 forward    neutral:+1 max:-0.5
          //3 backward   neutral:+1 max:-0.5
          //2 angular    leftmax:1 , neutral:0, rightmax:-1

          // origin value is min->max 1 -> -0.5
          double forward = _joy.axes[5];
          forward -= 1; //set to 0
          forward *= -1; //ivert to 0..1.5
          forward /= 1.5;

          forward = (forward > 1 ? 1 : forward);

          forward *= forward;

          //ROS_INFO("forward: %f", forward);

          double back    = _joy.axes[2];
          back -= 1; //set to 0
          back *= -1; //ivert to 0..1.5
          back /= 1.5;

          back = (back > 1 ? 1 : back);

          back *= back;
          back *= -1;

          //ROS_INFO("backward: %f", back);

          //double rot     = _joy.axes[3];
          double rot     = _joy.axes[0];
          int sign = rot < 0 ? -1 : 1;
          rot /= 0.85;
          rot = (std::abs(rot) > 1 ? 1 : rot);

          rot = std::abs(rot);
          //rot *= rot;
          rot *= sign;


          //ROS_INFO("ang: %f", rot);

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


void Joy2Vel_sc::subCallback_joy(const sensor_msgs::Joy msg)
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
    ros::init(argc, argv, "joy2vel_sc_node");
    ros::NodeHandle nh("~");

    Joy2Vel_sc node;
    node.start();

}
