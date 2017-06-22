#ifndef TEMPLATE_PUBLISHER_HPP_
#define TEMPLATE_PUBLISHER_HPP_

#include <ros/ros.h>

template <typename ros_Message>
class TemplatePublisher
{
protected:
    ros::NodeHandle *nh;
    ros::Publisher pub;

public:
    ros_Message* buff_ptr;

public:
    TemplatePublisher(ros::NodeHandle* nh, std::string topic, ros_Message *buff_ptr)
    {
        this->nh = nh;
        this->pub = nh->advertise<ros_Message>(topic, 100);
        this->buff_ptr = buff_ptr;
    }

    ~TemplatePublisher(){}

    void publish()
    {
        pub.publish(*this->buff_ptr);
    }

    void publish(ros_Message msg)
    {
        pub.publish(this->msg);
    }
};

#endif
