
#include "SimMove.h"
#include "PathAnalyser/BasicAnalyser.h"
#include "Controller/ParabolaTransfere.h"

SimMove::SimMove() : _rate(0)
{
    _loopRate = 0;

    //rosParam
    ros::NodeHandle privNh("~");
    std::string pub_name_cmd_vel;
    std::string pub_name_state;
    std::string pub_name_process;
    std::string sub_name_path;
    std::string sub_name_pause;

    std::string tf_map_frame;
    std::string tf_robot_frame;

    double vel_lin_max;
    double vel_ang_max;

    double target_radius       ;
    double target_radius_final ;
    int    cos_pwr_n           ;
    double cos_fac_n           ;
    double ang_reached_range   ;
    double lin_end_approach    ;
    double lin_ctrl_scale      ;
    double ang_ctrl_scale      ;

    privNh.param("pub_name_cmd_vel",         pub_name_cmd_vel,       std::string("vel/teleop"));
    privNh.param("pub_name_state",           pub_name_state,         std::string("sim_move/state"));
    privNh.param("pub_name_process",         pub_name_process,       std::string("sim_move/process"));
    privNh.param("sub_name_path",            sub_name_path,          std::string("path"));
    privNh.param("sub_name_pause",           sub_name_pause,         std::string("sim_move/pause"));
    privNh.param("tf_map_frame",             tf_map_frame,           std::string("map"));
    privNh.param("tf_robot_frame",           tf_robot_frame,         std::string("base_footprint"));

    privNh.param<double>("vel_lin_max"    , vel_lin_max,  0.2);
    privNh.param<double>("vel_ang_max"    , vel_ang_max,  1.3);

    privNh.param<double>("target_radius"      ,   target_radius         , 0.24);
    privNh.param<double>("target_radius_final",   target_radius_final   , 0.1 );
    privNh.param<int>(   "cos_pwr_n"          ,   cos_pwr_n             , 4   );
    privNh.param<double>("cos_fac_n"          ,   cos_fac_n             , 1.0 );
    privNh.param<double>("ang_reached_range"  ,   ang_reached_range     , 0.1 );
    privNh.param<double>("lin_end_approach"   ,   lin_end_approach      , 1.0 );
    privNh.param<double>("lin_ctrl_scale"     ,   lin_ctrl_scale        , 2.0 );
    privNh.param<double>("ang_ctrl_scale"     ,   ang_ctrl_scale        , 4.0 );


    _tf_map_frame = tf_map_frame;
    _tf_robot_frame = tf_robot_frame;

    //init publisher
    _pub_cmd_vel  = _nh.advertise<geometry_msgs::Twist>(pub_name_cmd_vel,1);
    _pub_state    = _nh.advertise<std_msgs::Bool>(pub_name_state,1);
    _pub_progress = _nh.advertise<std_msgs::UInt32>(pub_name_process,1);

    //inti subscriber
    _sub_path = _nh.subscribe(sub_name_path , 1, &SimMove::subPath_callback, this);
    _sub_pause = _nh.subscribe(sub_name_pause, 1, &SimMove::subPause_callback, this);

    //_pathAnalyser = new analyser::BasicAnalyser(0.24, 0.1, 4, 1, 0.1, 1.0);
    _pathAnalyser = new analyser::BasicAnalyser(target_radius, target_radius_final, cos_pwr_n, cos_fac_n, ang_reached_range, lin_end_approach);
    _controller = new controller::ParabolaTransfere(vel_lin_max, vel_ang_max, 2, 4);

    _enable_analyse = false;
    _pause = false;

}

SimMove::~SimMove()
{
    delete _rate;
}

void SimMove::start(const unsigned int rate)
{
   delete _rate;
   _loopRate = rate;
   _rate = new ros::Rate(_loopRate);

   //wait for first transform
   bool rdy = false;
   do{
      try {
         ros::Time time = ros::Time::now();
         _tf_listnener.waitForTransform(_tf_map_frame, _tf_robot_frame, time, ros::Duration(1));
         rdy = true;

      } catch (tf::TransformException& e)
      {
         ROS_ERROR("sim_move -> Exeption at tf: %s", e.what());
         return;
      }
   }while(!rdy);

   this->run();
}

void SimMove::run()
{
   unsigned int cnt = 0;

   while(ros::ok())
   {
      this->doPathControl();

      ros::spinOnce();
      _rate->sleep();
   }
}


void SimMove::doPathControl(void)
{
   if(!_enable_analyse)
   {
      return;
   }

   //get tf
   tf::StampedTransform tf;
   try {
      ros::Time time = ros::Time(0);
      _tf_listnener.lookupTransform(_tf_map_frame, _tf_robot_frame, time, tf);

   } catch (tf::TransformException& e)
   {
      ROS_ERROR("ohm_path_control -> Exeption at tf: %s", e.what());
      return;
   }

   analyser::pose pose;
   pose.position = Vector3d( tf.getOrigin().x(), tf.getOrigin().y(), 0);
   Quaternion<double> tmp_q(tf.getRotation().w(),
                            tf.getRotation().x(),
                            tf.getRotation().y(),
                            tf.getRotation().z() );
   pose.orientation = analyser::PathAnalyser_base::quaternion_to_orientationVec(tmp_q);

   //get diff scale
   analyser::diff_scale diff_scale = _pathAnalyser->analyse(pose);

   //controll diffscale
   controller::velocity vel = _controller->control(diff_scale.linear, diff_scale.angular);

   //set twist msg
   geometry_msgs::Twist msgTwist;
   msgTwist.angular.z = vel.angular;
   msgTwist.linear.x = vel.linear;

//   if(pathInfo.reached_final_goal)
//   {
//      //_enable_analyse = false;
//   }

   if(_pause)
   {
      msgTwist.angular.z = 0;
      msgTwist.linear.x = 0;
   }

   //for old msg
   std_msgs::Bool state_msg_old;
   state_msg_old.data = false;
   std_msgs::UInt32 process;
   process.data = _pathAnalyser->getInfo().current_goal_id;

   if(_pathAnalyser->isReachedFinalGoal())
      state_msg_old.data = true;

   _pub_progress.publish(process);
   _pub_state.publish(state_msg_old);

   //publish Twist:
   _pub_cmd_vel.publish(msgTwist);
}

void SimMove::subPath_callback(const nav_msgs::Path& msg_)
{
   nav_msgs::Path msg = msg_;
   //path with length 1 is invalid, if invalid then clear
   if(msg.poses.size() <= 1)
   {
      msg.poses.clear();
   }
   std::vector<analyser::pose> path_comp;
   std::vector<analyser::pose> path_trunc;

   path_comp.resize(msg.poses.size());
   path_trunc.resize(msg.poses.size());

   ROS_INFO("ohm_path_control -> GOT Path");

   //set path
   for(unsigned int i=0; i<msg.poses.size(); i++)
   {
      analyser::pose tmp_pose;
      tmp_pose.position = Vector3d(msg.poses[i].pose.position.x,
                                   msg.poses[i].pose.position.y,
                                   msg.poses[i].pose.position.z);

      Quaternion<double> tmp_q(msg.poses[i].pose.orientation.w,
                               msg.poses[i].pose.orientation.x,
                               msg.poses[i].pose.orientation.y,
                               msg.poses[i].pose.orientation.z);

      tmp_pose.orientation = analyser::PathAnalyser_base::quaternion_to_orientationVec(tmp_q);
      path_comp[i] = tmp_pose;
      path_trunc[i] = tmp_pose;
   }

   //get tf
   bool tf_rdy = true;
   tf::StampedTransform tf;
   do{
      try {
         ros::Time time = ros::Time(0);
         _tf_listnener.lookupTransform(_tf_map_frame, _tf_robot_frame, time, tf);

      } catch (tf::TransformException& e)
      {
         ROS_ERROR("ohm_path_control -> Exeption at tf: %s", e.what());
         tf_rdy = false;
      }
   }while(!tf_rdy);
   //tf rdy

   //now porve if path is in radius

   analyser::BasicAnalyser* tmp_analyser = dynamic_cast<analyser::BasicAnalyser*>(_pathAnalyser);
   if(tmp_analyser)
   {//just do if cast is valid
      double detectionRaidus = tmp_analyser->getDetectionRadius();

      detectionRaidus *= 0.5; // todo use parameter to config this

      //px, py : robot pose
      double px = tf.getOrigin().x();
      double py = tf.getOrigin().y();

      unsigned int trunc_idx = 0;
      for(int i = (int)path_comp.size() - 1; i >= 0; --i)
      {
         //cx, cy : current path point
         double cx = path_comp[i].position.x();
         double cy = path_comp[i].position.y();

         //prove if path point is in robot pose + detection radius: (x-x0)^2 + (y-y0)^2 < r^2
         if( (cx - px)*(cx - px) + (cy - py)*(cy - py) < (detectionRaidus * detectionRaidus) )
         {//point is in radius
            //ROS_INFO("Found stuff in Radius!!!!!!!!!!!!!");
            trunc_idx = i;
            break;
         }
      }

      if(trunc_idx)
         path_trunc.erase(path_trunc.begin(), path_trunc.begin() + trunc_idx);

      if(!path_trunc.size())
      {//if path_tranc contains no points than use path_comp;
         path_trunc = path_comp;
      }
   }
   else
   {
      path_trunc = path_comp;
   }


   ROS_INFO("Path.size: %d",(int)path_trunc.size());
   _pathAnalyser->setPath(path_trunc);
   _enable_analyse = true;
   std_msgs::Bool reachedTarget;
   reachedTarget.data = false;

   //pub false
   _pub_cmd_vel.publish(reachedTarget);

}

void SimMove::subPause_callback(const std_msgs::Bool& msg)
{
   _pause = msg.data;
}




//--------main-----------------

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sim_move_node");
    ros::NodeHandle nh("~");

    SimMove node;
    node.start(50);

}


