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
    ros_Message msg;

public:
    TemplatePublisher(ros::NodeHandle* nh, std::string topic)
    {
        this->nh = nh;
        this->pub = nh->advertise<ros_Message>(topic, 100);
    }

    ~TemplatePublisher(){}

    void publish()
    {
        pub.publish(this->msg);
    }

    void publish(ros_Message msg)
    {
        this->msg = msg;
        pub.publish(this->msg);
    }

    void publish(double data)
    {
        this->msg.data = data;
        pub.publish(this->msg);
    }
};

#endif
