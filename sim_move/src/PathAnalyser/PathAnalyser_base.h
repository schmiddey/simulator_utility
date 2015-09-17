/*
 * PathAnalyserbase.h
 *
 *  Created on: 24.10.2014
 *      Author: m1ch1
 */

#ifndef _PATHANALYSERBASE_H_
#define _PATHANALYSERBASE_H_

#include <vector>
#include <Eigen/Dense>
//#define EIGEN_USE_NEW_STDVECTOR
//#include <Eigen/StdVector>

using namespace Eigen;

namespace analyser
{

//enum enum_state {
//   IDLE = 0,
//   REACHED_GOAL,
//   MOVING,
//   ABORTED
//};

typedef struct {
   double linear;
   double angular;
} diff_scale;

typedef struct {
   Vector3d position;
   Vector3d orientation;
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} pose;

typedef struct {
   double path_length;
   double path_length_remaining;
   unsigned int num_goals;
   unsigned int current_goal_id;
   //analyser::enum_state state;
   bool reached_goal;
} info;

class PathAnalyser_base
{
public:
   /**
    * @brief constructor
    */
   PathAnalyser_base();

   /**
    * @brief destructor
    */
   virtual ~PathAnalyser_base() { }

   /**
    * @brief set new path
    *
    * @param path -> to analyse incrementelly
    *
    * @return void
    */
   void setPath(std::vector<analyser::pose> path);

   /**
    * @param void
    *
    * @return diffscale whitch have to be controled after
    */
   virtual analyser::diff_scale analyse(analyser::pose current_pose) = 0;

   analyser::info getInfo();

   /**
    * @todo write func
    * @return
    */
   static Vector3d quaternion_to_orientationVec(Quaternion<double> q) { return q * Vector3d(1,0,0); }

   void setDoEndRotate(const bool doEndRotate) { _do_end_rotate = doEndRotate; }
   bool isDoEndRotate() { return _do_end_rotate; }
   inline bool isReachedFinalGoal() const { return _reached_final_goal; }


protected: //dataelements
   std::vector<analyser::pose> _path;  ///< current path to analyse

protected:  //functions
   inline analyser::pose currentGoal() { return _path[_currentGoal_index]; }
   void nextGoal();
   inline unsigned int getCurrentGoalIndex() { return _currentGoal_index; }
   inline bool isLastGoal() { return _currentGoal_index == _path.size() - 1 ? true : false; }
   inline bool isFirstGoal() { return _currentGoal_index == 0 ? true : false; }

   inline double getDistToCurrentGoal() const { return _dist_to_current_goal; }
   inline void setDistToCurrentGoal(double distToCurrentPose) { _dist_to_current_goal = distToCurrentPose; }
   void setReachedFinalGoal(bool reachedFinalGoal) { _reached_final_goal = reachedFinalGoal; }
   //inline void setState(analyser::enum_state state)
   //{
   //   _state = state;
   //}
   //inline analyser::enum_state getState() { return _state; }
   inline double getPathLengthRest() const { return _path_lenth_rest; }


private:
   unsigned int _currentGoal_index;
   double _dist_to_current_goal;       ///< must set in analyse() by user of this baseclass
   double _path_lenth;
   double _path_lenth_rest;
   bool _reached_final_goal;           ///< must set in analyse() by user of this baseclass
   bool _do_end_rotate;
   //analyser::enum_state _state;
   //for eigen
//public:
   //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} /* namespace analyser */

#endif /* _PATHANALYSERBASE_H_ */
