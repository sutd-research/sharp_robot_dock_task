/**
 * Copyright 2017 by Institute for Infocomm Research, Singapore (I2R). All rights reserved.
 * @author Ng Kam Pheng (ngkp@i2r.a-star.edu.sg)
 */
#include "base.hpp"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <visual_marker_docking/DockAction.h>

namespace handler
{

typedef actionlib::SimpleActionClient<visual_marker_docking::DockAction> Client;
/**
 * Sample template
 *
 */
class TaskHandler: public BaseHandler
{
public:
    TaskHandler();
    ~TaskHandler() {}

private:
    void InitGeneric() override;
    void InitROSParams() override;
    void InitROSPublishers() override;
    void InitROSSubscribers() override;
    void PreInit() override;
    void runTask(const std::string &jsonstr_data) override;
    void stopTask() override;

    /// true if task is running, false otherwise
    bool m_is_running;

    /// @todo for demo only
    int m_demo_task_done_status_value;
   

    // Action server
    Client *ac;

    // Action complete callback
    void actionDoneCB(const actionlib::SimpleClientGoalState& state, const visual_marker_docking::DockResultConstPtr& result);
};

} // handler
