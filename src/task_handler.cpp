/**
 * Copyright 2017 by Institute for Infocomm Research, Singapore (I2R). All rights reserved.
 * @author Ng Kam Pheng (ngkp@i2r.a-star.edu.sg)
 */
#include "task_handler.h"

namespace handler
{

/**
 * Class constructor
 *
 */
TaskHandler::TaskHandler()
    : BaseHandler(),
      m_demo_task_done_status_value(task_msgs::TaskStatus::K_TASK_STATUS_DONE),
      m_is_running(false)
{
}

/**
 * Perform generic initialisation
 *
 */
void TaskHandler::InitGeneric()
{
    ROS_DEBUG_STREAM(ros::this_node::getName() << " - Executing generic initialization");

    /// @todo add your generic initialization here
}

/**
 * Initialise all global and private ROS parameters
 *
 */
void TaskHandler::InitROSParams()
{
    ROS_DEBUG_STREAM(ros::this_node::getName() << " - Executing ROS params initialization");

    /// @todo retrieve your ROS params here

    /// for testing only, when demo timer has finished, publish this value instead
    if (m_nh_private.hasParam("demo_task_done_status_value"))
    {
        if (m_nh_private.getParam("demo_task_done_status_value", m_demo_task_done_status_value))
        {
            ROS_WARN_STREAM(ros::this_node::getName() << " - m_demo_task_done_status_value set to " << m_demo_task_done_status_value);
        }
        else
        {
            m_demo_task_done_status_value = task_msgs::TaskStatus::K_TASK_STATUS_DONE;
            ROS_ERROR_STREAM(ros::this_node::getName() << " - m_demo_task_done_status_value invalid, setting to " << m_demo_task_done_status_value);
        }
    }
}

/**
 * Initialise all the publishers
 *
 */
void TaskHandler::InitROSPublishers()
{
    ROS_DEBUG_STREAM(ros::this_node::getName() << " - Executing ROS publishers initialization");

    // create the action client
    // true causes the client to spin its own thread
    ac = new actionlib::SimpleActionClient<visual_marker_docking::DockAction>("visual_dock/dock_server/", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac->waitForServer(); //will wait for infinite time
    ROS_INFO("Action server initialization complete");
}

/**
 * Initialise all the subscribers
 *
 */
void TaskHandler::InitROSSubscribers()
{
    ROS_DEBUG_STREAM(ros::this_node::getName() << " - Executing ROS subscribers initialization");

    /// @todo add your subscribers here
}

/**
 * Perform initialisation before init() is called
 *
 */
void TaskHandler::PreInit()
{
    ROS_DEBUG_STREAM(ros::this_node::getName() << " - Executing pre-initialization initialization");

    /// @todo add your pre-initialization here
}

/**
 * Runs the idle task
 * @param jsonstr_data JSON data from CommandPub2
 */
void TaskHandler::runTask(const std::string &jsonstr_data)
{
    if (m_is_running)
    {
        ROS_WARN_STREAM(ros::this_node::getName() << " - The task is already running");
    }
    else
    {
        ROS_INFO_STREAM(ros::this_node::getName() << " - Running task");

        // Note that when mission data contains ROS parameters for your task, and it is
        // passed in via the commandpub2 message jsonstr_data

        // publish running status
        publishTaskStatus(task_msgs::TaskStatus::K_TASK_STATUS_RUNNING);
        m_is_running = true;

        /// @todo add codes to publish K_TASK_STATUS_DONE when task is done
        // Send action goal
    
        // send a goal to the action
        visual_marker_docking::DockGoal goal;
        goal.dockingCommand = 1;
        ac->sendGoal(goal, boost::bind(&TaskHandler::actionDoneCB, this, _1, _2), Client::SimpleActiveCallback(), Client::SimpleFeedbackCallback());
    }
}

/**
 * Stops the idle task
 *
 */
void TaskHandler::stopTask()
{
    if (m_is_running)
    {
        ROS_INFO_STREAM(ros::this_node::getName() << " - Stopping task");
        ac->cancelAllGoals();

        publishTaskStatus(task_msgs::TaskStatus::K_TASK_STATUS_IDLE);
        m_is_running = false;
    }
    else
    {
        if (getCurrentTaskStatus() != task_msgs::TaskStatus::K_TASK_STATUS_IDLE)
        {
            publishTaskStatus(task_msgs::TaskStatus::K_TASK_STATUS_IDLE);
        }
    }
}

void TaskHandler::actionDoneCB(const actionlib::SimpleClientGoalState& state,
              const visual_marker_docking::DockResultConstPtr& result)
{
    // publishTaskStatus(task_msgs::TaskStatus::K_TASK_STATUS_DONE);
    // stopTask();
}

} // handler
