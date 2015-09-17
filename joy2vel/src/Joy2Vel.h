
#ifndef TEMPLATE_H_
#define TEMPLATE_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class Joy2Vel
{
private:    //dataelements
    double _loopRate;

    ros::Rate* _rate;
    ros::NodeHandle _nh;

    ros::Publisher _pubTwist;
    ros::Subscriber _subJoy;

    double _max_vel_lin;
    double _max_vel_ang;

    bool _rdy;

    sensor_msgs::Joy _joy;

public:
    Joy2Vel();
    virtual ~Joy2Vel();

    /**
     * @fn void start(const unsigned int frames = 10)
     *
     * @brief
     *
     * @return  void
     */
    void start();

private:    //functions

    /**
     * @fn void run()
     *
     * @brief this function containts the main working loop
     *
     * @param[in,out]  void
     *
     * @return 		   void
     */
    void run();

    //void subCallback(const ROS_PACK::MESSAGE& msg);
    void subCallback_joy(const sensor_msgs::Joy msg);

};

#endif /* TEMPLATE_H_ */
