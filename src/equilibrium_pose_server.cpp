//
// Created by lucas on 25.09.23.
//
/*
 * This file was a fruitless try to implement the equilibrium pose callback as action server
 * */
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "../../../devel/include/force_control/EquilibriumPoseAction.h"
#include "../../../devel/include/force_control/EquilibriumPoseFeedback.h"
#include "../../../devel/include/force_control/EquilibriumPoseResult.h"
#include "../../../devel/include/force_control/EquilibriumPoseGoal.h"

namespace force_control{
    class EquilibriumPoseServer
    {
    public:

        EquilibriumPoseServer(std::string name) :
                as_(nh_, name, false),
                action_name_(name)
        {
            //register the goal and feeback callbacks
            as_.registerGoalCallback(boost::bind(&EquilibriumPoseServer::goalCB, this));
            as_.registerPreemptCallback(boost::bind(&EquilibriumPoseServer::preemptCB, this));

            //subscribe to the data topic of interest
            sub_ = nh_.subscribe("/random_number", 1, &EquilibriumPoseServer::analysisCB, this);
            as_.start();
        }

        ~EquilibriumPoseServer(void)
        {
        }

        void goalCB()
        {
            // reset helper variables
            data_count_ = 0;
            sum_ = 0;
            sum_sq_ = 0;
            // accept the new goal
            //goal_ = as_.acceptNewGoal()->samples;
        }

        void preemptCB()
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            // set the action state to preempted
            as_.setPreempted();
        }

        void analysisCB(const std_msgs::HeaderConstPtr& msg)
        {
            int a = 0;
        }

    protected:

        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<force_control::EquilibriumPoseAction> as_;
        std::string action_name_;
        int data_count_, goal_;
        float sum_, sum_sq_;
        force_control::EquilibriumPoseFeedback feedback_;
        force_control::EquilibriumPoseResult result_;
        ros::Subscriber sub_;
    };

}
