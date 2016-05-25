#ifndef SKILLTOOLS_H
#define SKILLTOOLS_H

#include <chrono>
#include <thread>
#include <stdexcept>

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>

#include <typeinfo>

namespace skiros_tools
{
    template <class Action, class Goal, class Feedback, class Result>
    class ActionClientWrapper
    {
    protected:
        typedef actionlib::SimpleClientGoalState State;

        std::string name_;
        std::string topic_;
        bool hasSucceeded_;

        actionlib::SimpleActionClient<Action> client_;
        std::vector<Feedback> feedback_;
        Result result_;

    public:
        ActionClientWrapper(const std::string & topic) : client_(topic, true), topic_(topic), hasSucceeded_(false) {
            this->name_ = "ActionClientWrapper";
        }
        virtual ~ActionClientWrapper() = 0;

        void connect(float waitDuration = 0.0)
        {
            ROS_INFO_STREAM("[" << this->name_ << "] Connecting to server on topic '" << this->topic_ << "");
            float waitDurationStep = 1.0;
            do
            {
                ROS_INFO_STREAM("[" << this->name_ << "] Waiting for server (" << waitDuration << " sec)");
                float wait = std::min(waitDurationStep, waitDuration);
                this->client_.waitForServer(ros::Duration(wait));
                waitDuration -= wait;
            }
            while (!this->client_.isServerConnected() && waitDuration > 0.0);

            if (!this->client_.isServerConnected())
                ROS_ERROR_STREAM("[" << this->name_ << "] TIMEOUT: Could not connect to server!");
            else
                ROS_INFO_STREAM("[" << this->name_ << "] Connected to server");
        }

        void cancel()
        {
            ROS_INFO_STREAM("[" << this->name_ << "] Action cancelled");
            this->client_.cancelGoal();
        }

        void wait(float waitDuration = 0.0)
        {
            float waitDurationStep = 1.0;
            do
            {
                ROS_INFO_STREAM("[" << this->name_ << "] Waiting for action result (" << waitDuration << " sec)");
                float wait = std::min(waitDurationStep, waitDuration);
                this->client_.waitForResult(ros::Duration(wait));
                waitDuration -= wait;
            }
            while (!this->isDone() && waitDuration > 0.0);

            if (!this->isDone())
                ROS_ERROR_STREAM("[" << this->name_ << "] TIMEOUT: Action still in progress!");
        }

        void waitForFeedback(float waitDuration = 0.0)
        {
            ros::Rate rate(10);
            while (this->feedback_.empty() && waitDuration > 0.0)
            {
                ROS_INFO_STREAM_THROTTLE(1.0, "[" << this->name_ << "] Waiting for action feedback (" << waitDuration << " sec)");
                rate.sleep();
                waitDuration -= rate.cycleTime().toSec();
            }

            if (this->feedback_.empty())
                ROS_ERROR_STREAM("[" << this->name_ << "] TIMEOUT: Feedback not received!");
        }

        bool isConnected()
        {
            return this->client_.isServerConnected();
        }

        bool isDone()
        {
            return this->client_.getState().isDone();
        }

        bool hasSucceeded()
        {
            return (hasSucceeded_ && this->client_.getState() == State::SUCCEEDED);
        }

        State getState()
        {
            return this->client_.getState();
        }

        std::vector<Feedback> getFeedback()
        {
            return this->feedback_;
        }

        Result getResult()
        {
            return this->client_.getResult();
        }

    protected:
        void execute(const Goal & goal)
        {
            ROS_INFO_STREAM("[" << this->name_ << "] Executing goal");
            this->client_.sendGoal(goal,
                boost::bind(&ActionClientWrapper::actionEnded, this, _1, _2),
                boost::bind(&ActionClientWrapper::actionStarted, this),
                boost::bind(&ActionClientWrapper::actionFeedback, this, _1)
            );
        }

        void failed()
        {
            this->hasSucceeded_ = false;
        }

        void reset()
        {
            ROS_INFO_STREAM("[" << this->name_ << "] Clear result");
            this->feedback_.clear();
            this->result_.reset();
            this->hasSucceeded_ = false;
        }

        void actionStarted()
        {
            ROS_INFO_STREAM("[" << this->name_ << "] Action started");
            this->reset();
        }

        void actionFeedback(const Feedback & feedback)
        {
            ROS_INFO_STREAM("[" << this->name_ << "] Feedback received");
            this->feedback_.push_back(feedback);
        }

        void actionEnded(const State & state, const Result & result)
        {
            if (state == State::SUCCEEDED)
            {
                ROS_INFO_STREAM("[" << this->name_ << "] Action ended: " << state.toString());
            this->hasSucceeded_ = true;
            } else {
                ROS_ERROR_STREAM("[" << this->name_ << "] Action ended: " << state.toString());
            }

            this->result_ = result;
        }

    }; // class ActionClientWrapper

    template <class Action, class Goal, class Feedback, class Result>
    inline ActionClientWrapper<Action, Goal, Feedback, Result>::~ActionClientWrapper() {}
}
#endif // SKILLTOOLS_H
