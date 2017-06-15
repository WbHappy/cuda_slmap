#ifndef TEMPLATE_SUBSCRIBER_HPP_
#define TEMPLATE_SUBSCRIBER_HPP_

#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include <functional>

template <typename ros_Message>
class TemplateSubscriber
{
protected:
    ros::NodeHandle *nh;
    ros::Subscriber sub;
public:
    ros_Message* buff_ptr;
    bool msg_recived;


public:
    TemplateSubscriber(ros::NodeHandle *nh, std::string topic, ros_Message* buff_ptr){
        this->msg_recived = false;
        this->nh = nh;
        this->buff_ptr = buff_ptr;
        this->sub = nh->subscribe(topic, 100, &TemplateSubscriber::msgInterrupt, this);
    }

    TemplateSubscriber(ros::NodeHandle *nh, std::string topic, ros_Message* buff_ptr, std::function< void() > functocall){
        this->msg_recived = false;
        this->nh = nh;
        this->buff_ptr = buff_ptr;
        this->sub = nh->subscribe<ros_Message>(topic, 100, boost::bind(&TemplateSubscriber::msgInterrupt, this, _1, functocall));
    }

    ~TemplateSubscriber(){}

    void msgInterrupt(const typename ros_Message::ConstPtr &msg){
        *buff_ptr = *msg;
        this->msg_recived = true;
    }

    void msgInterrupt(const typename ros_Message::ConstPtr &msg, std::function< void() > functocall){
        *buff_ptr = *msg;
        this->msg_recived = true;

        functocall();
    }


    bool firstMsgRecived(){
        return msg_recived;
    }

};

#endif
