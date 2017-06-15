#ifndef TEMPLATE_SUBSCRIBER_FNC_HPP_
#define TEMPLATE_SUBSCRIBER_FNC_HPP_

#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <functional>

template <typename ros_Message, typename func_Type>
class TemplateSubscriberFnc
{
protected:
    ros::NodeHandle *nh;
    ros::Subscriber sub;
public:
    ros_Message msg;
    bool msg_recived;


public:
    TemplateSubscriberFnc(ros::NodeHandle *nh, std::string topic){
        this->msg_recived = false;
        this->nh = nh;
        this->sub = nh->subscribe(topic, 100, &TemplateSubscriberFnc::msgInterrupt, this);
    }

    TemplateSubscriberFnc(ros::NodeHandle *nh, std::string topic, std::function< func_Type > functocall){
        this->msg_recived = false;
        this->nh = nh;
        this->sub = nh->subscribe<ros_Message>(topic, 100, boost::bind(&TemplateSubscriberFnc::msgInterrupt, this, _1, functocall));
    }

    ~TemplateSubscriberFnc(){}

    void msgInterrupt(const typename ros_Message::ConstPtr &msg){
        this->msg = *msg;
        this->msg_recived = true;
    }

    void msgInterrupt(const typename ros_Message::ConstPtr &msg, std::function< func_Type > functocall){
        this->msg = *msg;
        this->msg_recived = true;

        functocall();
    }


    bool firstMsgRecived(){
        return msg_recived;
    }

};

#endif
