#include "../include/dyros_jet_joystick/dyros_joystick.h"

DyrosJoystick::DyrosJoystick()
{
    walking_cmd_pub_ = nh_.advertise<dyros_jet_msgs::WalkingCommand>("/dyros_jet/joystick_walking_command", 3);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &DyrosJoystick::joyCallback, this);
    
}

void DyrosJoystick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
//    if (joy->axes[1] > 0.5)
//    {
//        walk_cmd_ = true;
//        walk_cmd_msg_.x = 1.0;
//    }
//    else
//    {
//        walk_cmd_ = false;
//        walk_cmd_msg_.x = 0.0;
//    }
    // if(joy->buttons[8] == 1.0){
    //     walk_cmd_ = true;
    // }
    // if(joy->buttons[6] == 1.0){
    //     walk_cmd_ =false;
    // }

    walk_cmd_msg_.x = (joy->axes[0] + 1)/2;
    walk_cmd_msg_.y = (joy->axes[1] + 1)/2;
    walk_cmd_msg_.theta = joy->axes[2]*30*-1;
    walk_cmd_msg_.walk_mode = true;

    std::cout << "0:" << joy->axes[0] << " 1:" << joy->axes[1] << " 2:" << joy->axes[2] << std::endl;
    std::cout << "walk_x:" << walk_cmd_msg_.x << " walk_y:" << walk_cmd_msg_.y << " theta:" << walk_cmd_msg_.theta << std::endl;

    //walk_cmd_msg_.x = joy->axes[1];
    //walk_cmd_msg_.y = joy->axes[3];
    //walk_cmd_msg_.z = joy->axes[2]; //default is 1.0, when the button pushed, -1.0
    // std::cout << joy->axes[3] << std::endl;
    // walk_cmd_msg_.walk_mode = walk_cmd_;
    // walk_cmd_msg_.height = 0.0;
    // walk_cmd_msg_.height = joy->axes[0];
    // walk_cmd_msg_.theta = 0.0;
    // walk_cmd_msg_.step_length_x = joy->buttons[1];
    // walk_cmd_msg_.step_length_y = joy->buttons[2];
    // std::cout << "B" <<joy->buttons[1] << "X" <<joy->buttons[2] << std::endl;


   if (walk_cmd_ ==true)
    walking_cmd_pub_.publish(walk_cmd_msg_);

}




int main(int argc, char** argv)
{
    std::cout << "Started1"<<std::endl;
    ros::init(argc, argv, "dyros_jet_joystick");
    DyrosJoystick dyrosjoystick;
    std::cout << "Joystick Controller Started"<<std::endl;
    while(ros::ok())
    {
        ros::spinOnce();
    }
}
