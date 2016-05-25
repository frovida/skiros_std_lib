/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Francesco Rovida
 *	Robotics, Vision and Machine Intelligence Laboratory
 *  Aalborg University, Denmark
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Aalborg Universitet nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#ifndef TOPIC_SWITCH_H
#define TOPIC_SWITCH_H

#include<string>
#include<ros/ros.h>
#include<topic_tools/MuxSelect.h>
#include<skiros_common/logger_sys.h>

namespace skiros_lib_tools
{

class TopicSwitch
{
public:
    TopicSwitch() : prev_topic_("")
    {}

    ~TopicSwitch(){}

    void init(ros::NodeHandle & nh, std::string multiplexer_address)
    {
        mux_client_ = nh.serviceClient<topic_tools::MuxSelect>(multiplexer_address);
    }

    bool switchTopic(std::string address)
    {
        if(!mux_client_.exists()) return false;
        if(address==prev_topic_)return true;
        topic_tools::MuxSelect msg;
        msg.request.topic = address;

        if(mux_client_.call(msg))
        {
            prev_topic_ = msg.response.prev_topic;
            FDEBUG("Switched to " << msg.request.topic << " previous " << prev_topic_);
            return true;
        }
        else
        {
            prev_topic_ = "";
            return false;
        }
    }

    bool restorePrevious()
    {
        if(prev_topic_!="")
        {
            topic_tools::MuxSelect msg;
            msg.request.topic = prev_topic_;
            mux_client_.call(msg);
            return true;
        }
        else return false;
    }

private:
    std::string prev_topic_;
    ros::ServiceClient mux_client_;
};

}

#endif // TOPIC_SWITCH_H
