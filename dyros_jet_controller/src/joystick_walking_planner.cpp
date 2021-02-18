#include "dyros_jet_controller/dyros_jet_model.h"
#include "dyros_jet_controller/walking_controller.h"
#include "cvxgen_6_8_0/cvxgen/solver.h"
#include <stdio.h>

namespace dyros_jet_controller
{

void WalkingController::JoystickWalkingPlanning(){
    if(joystick_walking_flag_ ==true)
    {
        if(joystick_planning_ == true)
        {
            JoyFootstepGoon();
            if(walking_tick_ == t_start_)
              cout << "\n\n foot_step_joy : \n" << foot_step_joy_ << endl;
        }
    }

}

void WalkingController::Joycrab(){
  std::cout << "Joycrab"<< std::endl;
  unsigned int number_of_footstep = 5;

  double width = 0.127794;
  double crab_length = joystick_input_(1)*0.1;
  int temp = -1;
  if(crab_length > 0)
    temp = 1;

  if(current_step_num_ !=0){
    lfoot_float_init_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)(0));
    rfoot_float_init_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)(1));
  }

  if(joystick_input_(0) == 0 && joystick_input_(1) == 0){
    joystick_rotation_num_ =0;
    joystick_iter_ =0;
  }

  if(current_step_num_ >2){
    if(foot_step_joy_(1,6) == 0){
      temp =1;
//    cout<<"temp +1"<<endl;
    }
    else if(foot_step_joy_(1,6) == 1){
//    cout<<"temp -1"<<endl;
      temp = -1;
    }
  }

  for(int i=0;i<5;i++){
      temp*= -1;
      foot_step_joy_(i,5) = 0.0;
      foot_step_joy_(i,6) = 0.5+ 0.5*temp;
  }

  if(current_step_num_<=2){
    temp = (foot_step_joy_(0,6) - 0.5)/0.5;
    foot_step_joy_(0,0) = 0;
    foot_step_joy_(0,1) = -temp*width + crab_length;

    temp = (foot_step_joy_(1,6) - 0.5)/0.5;
    foot_step_joy_(1,0) = 0;
    foot_step_joy_(1,1) = -temp*width + crab_length;
    for(int i=2;i<5;i++){
      temp = (foot_step_joy_(i,6) - 0.5)/0.5;
      foot_step_joy_(i,0) = 0;
      foot_step_joy_(i,1) = foot_step_joy_(i-2,1) + crab_length;
    }
  }
  else if(current_step_num_>2){
    if(foot_step_joy_(1,6) == 1){ // left foot support
      foot_step_joy_(0,0) = lfoot_float_init_.translation()(0);
      foot_step_joy_(0,1) = lfoot_float_init_.translation()(1);
      foot_step_joy_(1,0) = rfoot_float_init_.translation()(0);
      foot_step_joy_(1,1) = rfoot_float_init_.translation()(1);
    }
    else if(foot_step_joy_(1,6) == 0){ //right foot support
      foot_step_joy_(0,0) = rfoot_float_init_.translation()(0);
      foot_step_joy_(0,1) = rfoot_float_init_.translation()(1);
      foot_step_joy_(1,0) = lfoot_float_init_.translation()(0);
      foot_step_joy_(1,1) = lfoot_float_init_.translation()(1);
    }
    for(int i=2;i<5;i++){
        temp = (foot_step_joy_(i,6) - 0.5)/0.5;
        foot_step_joy_(i,0) = 0;
        foot_step_joy_(i,1) = foot_step_joy_(i-2,1) + crab_length - temp*(abs(rfoot_float_init_.translation()(1)) - abs(lfoot_float_init_.translation()(1)))/2;
        //foot_step_joy_(i,1) = foot_step_joy_(i-2,1) + crab_length;
      }
  }
}

void WalkingController::Joyori(){

    std::cout << "Joyori"<< std::endl;
    double theta_input = joystick_input_(2);
    Eigen::Vector5d rot_ang;
    double delta_theta = theta_input*DEG2RAD;
    unsigned int number_of_footstep = 5;


    double width = 0.127794;
    double dlength = joystick_input_(0)*0.1; // for stride. .

    double theta_input_hk = joystick_input_(2); //0~30degree
    double delta_theta_hk = theta_input_hk/3*DEG2RAD; //0~10DEG2RAD
    double center_hk = 9999;
    if(delta_theta_hk != 0){
      center_hk = dlength/(2*sin(delta_theta_hk/2));
      std::cout << "center_hk:" << center_hk << std::endl;
    }
    

//    dlength = step_length_x_;
    int temp = -1;


    if(current_step_num_ !=0){
        lfoot_float_init_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)(0));
        rfoot_float_init_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)(1));
    }


    if(walking_tick_ == 0 || walking_tick_ == t_start_){
        cout<<"joystick input : "<<joystick_input_(0)<<",  "<<joystick_input_(1)<<endl;
    }

    if(joystick_input_(0) == 0 && joystick_input_(1) == 0){
        theta_input = 0;
        joystick_rotation_num_ =0;
        joystick_iter_ =0;
        dlength = 0.0;

    }


    int iteration  = theta_input*RAD2DEG/10;
    double ori_input = iteration*DEG2RAD*10;

    if(theta_input !=0){
//        cout<<"with orientation "<<endl;
       if(ori_input != pre_theta_)
       {
           double iter = iteration - pre_theta_*RAD2DEG/10;
           joystick_rotation_num_= abs(iter) +1;
           joystick_iter_ =0;
//           theta_total_ = ori_input-pre_theta_;
       }
       else{
           if(joystick_rotation_num_ != 0)
                joystick_iter_ ++;
       }

       if(current_step_num_<=2){
           for(int i=0;i<number_of_footstep;i++){
               temp*= -1;
               foot_step_joy_(i,6) = 0.5+0.5*temp;
           }
       }
       else { // current step num is bigger than 3
           if(foot_step_joy_(1,6) ==0)// right foot support
           {
               temp = 1;
//               cout<<"right support "<<endl;

           }
           else if(foot_step_joy_(1,6) == 1)
           {
               temp = -1;
//               cout<<"left support "<<endl;
////               rot_ang(0) = rfoot_float_euler_init_(2);
////               rot_ang(1) = lfoot_float_euler_init_(2);
//               foot_step_joy_(0,5) = rfoot_float_euler_init_(2);
//               foot_step_joy_(0,6) = 0.5+ 0.5*temp;
//               temp*= -1;

//               foot_step_joy_(1,5) = lfoot_float_euler_init_(2);
//               foot_step_joy_(1,6) = 0.5+0.5*temp;
           }
           for(int i=0;i<5;i++){
               temp*=-1;
               foot_step_joy_(i,6) = 0.5+ 0.5*temp;
           }
       }
       int remain = joystick_rotation_num_ -1 - joystick_iter_;


       if(current_step_num_<=2){
           if(remain>=5)
               remain =5;
           for(int i=0;i<remain;i++){
              foot_step_joy_(i,5) = (i+1)*delta_theta_hk;
           }
           if(remain<5){
               for(int i=remain;i<5;i++){
                   foot_step_joy_(i,5) = foot_step_joy_(i-1,5);
               }
           }
//           temp = (foot_step_joy_(0,6) - 0.5)/0.5;
//           foot_step_joy_(0,0) = dlength;
//           foot_step_joy_(0,1) = -temp*width;

//           for(int i=1;i<5;i++){
//               temp = (foot_step_joy_(i,6) - 0.5)/0.5;
//               foot_step_joy_(i,0) = foot_step_joy_(i-1,0) + temp*2*width*sin(foot_step_joy_(i-1,5))+dlength*cos(foot_step_joy_(i-1,5));
//               foot_step_joy_(i,1) = foot_step_joy_(i-1,1) - temp*2*width*cos(foot_step_joy_(i-1,5)) + dlength*sin(foot_step_joy_(i-1,5));
//           }
       }
       else if(current_step_num_>2){
//           if(foot_step_joy_(1,6) ==1){ // left foot support
//               foot_step_joy_(0,0) = lfoot_float_init_.translation()(0);
//               foot_step_joy_(0,1) = lfoot_float_init_.translation()(1);

//               foot_step_joy_(1,0) = rfoot_float_init_.translation()(0);
//               foot_step_joy_(1,1) = rfoot_float_init_.translation()(1);
//           }
//           else if(foot_step_joy_(1,6) ==0){ //right foot support
//               foot_step_joy_(0,0) = rfoot_float_init_.translation()(0);
//               foot_step_joy_(0,1) = rfoot_float_init_.translation()(1);

//               foot_step_joy_(1,0) = lfoot_float_init_.translation()(0);
//               foot_step_joy_(1,1) = lfoot_float_init_.translation()(1);
//           }
           for(int i=2;i<5;i++){
             foot_step_joy_(i,5) = (i-1/2)*delta_theta_hk;
             std::cout << "hi" << std::endl;
           }
           if(remain >=3)
               remain = 3;

          //  for(int i=2;i<remain;i++){
          //     foot_step_joy_(i,5) = (i+1)*delta_theta_hk;
          //  }
           if(remain<3){
               for(int i=2+remain;i<5;i++){
                   foot_step_joy_(i,5) = foot_step_joy_(i-1,5);
               }
           }

//           for(int i=2;i<5;i++){
//               temp = (foot_step_joy_(i,6) - 0.5)/0.5;
//               foot_step_joy_(i,0) = foot_step_joy_(i-1,0) + temp*2*width*sin(foot_step_joy_(i-1,5))+dlength*cos(foot_step_joy_(i-1,5));
//               foot_step_joy_(i,1) = foot_step_joy_(i-1,1) - temp*2*width*cos(foot_step_joy_(i-1,5)) + dlength*sin(foot_step_joy_(i-1,5));
//           }

       }


       if(joystick_iter_== joystick_rotation_num_){
           joystick_iter_ =0;
           joystick_rotation_num_ =0;
       }

       if(current_step_num_<=2){
            //temp = (foot_step_joy_(0,6) - 0.5)/0.5; //1
            //foot_step_joy_(0,0) = (abs(center_hk) - temp*width) * sin(delta_theta_hk);
            //foot_step_joy_(0,1) = -(center_hk - (center_hk-temp*width)*cos(delta_theta_hk));

            for(int i=0;i<5;i++){
                temp = (foot_step_joy_(i,6) - 0.5)/0.5; //1
                if(dlength > 0){
                  if(delta_theta_hk < 0){
                    foot_step_joy_(i,0) = (abs(center_hk) - temp*width)*sin(M_PI+delta_theta_hk*(i+1));
                    foot_step_joy_(i,1) = - (abs(center_hk) - temp*width)*cos(M_PI+delta_theta_hk*(i+1)) + center_hk;
                  }
                  else if(delta_theta_hk > 0){
                    foot_step_joy_(i,0) = (abs(center_hk) + temp*width)*sin(delta_theta_hk*(i+1));
                    foot_step_joy_(i,1) = - (abs(center_hk) + temp*width)*cos(delta_theta_hk*(i+1)) + center_hk;
                  }
                }
                else if(dlength < 0){
                  if(delta_theta_hk < 0){
                    foot_step_joy_(i,0) = (abs(center_hk) + temp*width)*sin(delta_theta_hk*(i+1));
                    foot_step_joy_(i,1) = - (abs(center_hk) + temp*width)*cos(delta_theta_hk*(i+1)) + center_hk;
                  }
                  else if(delta_theta_hk > 0){
                    foot_step_joy_(i,0) = (abs(center_hk) - temp*width)*sin(M_PI+delta_theta_hk*(i+1));
                    foot_step_joy_(i,1) = - (abs(center_hk) - temp*width)*cos(M_PI+delta_theta_hk*(i+1)) + center_hk;
                  }
                }
                //foot_step_joy_(i,0) = foot_step_joy_(i-1,0) + temp*2*width*sin(foot_step_joy_(i-1,5)) + dlength*cos(foot_step_joy_(i-1,5));
                //foot_step_joy_(i,1) = foot_step_joy_(i-1,1) - temp*2*width*cos(foot_step_joy_(i-1,5)) + dlength*sin(foot_step_joy_(i-1,5));
            }
        }
        else if(current_step_num_>2){
            if(foot_step_joy_(1,6) ==1){ // left foot support
                foot_step_joy_(0,0) = lfoot_float_init_.translation()(0);
                foot_step_joy_(0,1) = lfoot_float_init_.translation()(1);

                foot_step_joy_(0,5) = rfoot_float_euler_init_(2);
                foot_step_joy_(0,5) = rfoot_float_euler_init_(2);

                foot_step_joy_(1,0) = rfoot_float_init_.translation()(0);
                foot_step_joy_(1,1) = rfoot_float_init_.translation()(1);

                foot_step_joy_(1,5) = lfoot_float_euler_init_(2);
                foot_step_joy_(1,5) = lfoot_float_euler_init_(2);
            }
            else if(foot_step_joy_(1,6) ==0){ //right foot support
                foot_step_joy_(0,0) = rfoot_float_init_.translation()(0);
                foot_step_joy_(0,1) = rfoot_float_init_.translation()(1);

                foot_step_joy_(0,5) = lfoot_float_euler_init_(2);
                foot_step_joy_(0,5) = lfoot_float_euler_init_(2);

                foot_step_joy_(1,0) = lfoot_float_init_.translation()(0);
                foot_step_joy_(1,1) = lfoot_float_init_.translation()(1);

                foot_step_joy_(1,5) = rfoot_float_euler_init_(2);
                foot_step_joy_(1,5) = rfoot_float_euler_init_(2);
            }

            for(int i=2;i<5;i++){
                temp = (foot_step_joy_(i,6) - 0.5)/0.5; //1
                if(dlength > 0){
                  if(delta_theta_hk < 0){
                    if(foot_step_joy_(i,6) == 0)
                      foot_step_joy_(i,0) = (abs(center_hk) - temp*width)*sin(M_PI+delta_theta_hk*(i))-(abs(rfoot_float_init_.translation()(0))-abs(lfoot_float_init_.translation()(0)))/2;
                    else if(foot_step_joy_(i,6) == 1)
                      foot_step_joy_(i,0) = (abs(center_hk) - temp*width)*sin(M_PI+delta_theta_hk*(i))-(abs(lfoot_float_init_.translation()(0))-abs(rfoot_float_init_.translation()(0)))/2;

                    foot_step_joy_(i,1) = - (abs(center_hk) - temp*width)*cos(M_PI+delta_theta_hk*(i)) + center_hk;
                  }
                  else if(delta_theta_hk > 0){
                    if(foot_step_joy_(i,6) == 0)
                      foot_step_joy_(i,0) = (abs(center_hk) + temp*width)*sin(delta_theta_hk*(i))-(abs(rfoot_float_init_.translation()(0))-abs(lfoot_float_init_.translation()(0)))/2;
                    else if(foot_step_joy_(i,6) == 1)
                      foot_step_joy_(i,0) = (abs(center_hk) + temp*width)*sin(delta_theta_hk*(i))-(abs(lfoot_float_init_.translation()(0))-abs(rfoot_float_init_.translation()(0)))/2;

                    foot_step_joy_(i,1) = - (abs(center_hk) + temp*width)*cos(delta_theta_hk*(i)) + center_hk;
                  }
                }
                else if(dlength < 0){
                  if(delta_theta_hk < 0){
                    if(foot_step_joy_(i,6) == 0)
                      foot_step_joy_(i,0) = (abs(center_hk) + temp*width)*sin(delta_theta_hk*(i))-(abs(rfoot_float_init_.translation()(0))-abs(lfoot_float_init_.translation()(0)))/2;
                    else if(foot_step_joy_(i,6) == 1)
                      foot_step_joy_(i,0) = (abs(center_hk) + temp*width)*sin(delta_theta_hk*(i))-(abs(lfoot_float_init_.translation()(0))-abs(rfoot_float_init_.translation()(0)))/2;
                      
                    foot_step_joy_(i,1) = - (abs(center_hk) + temp*width)*cos(delta_theta_hk*(i)) + center_hk;
                  }
                  else if(delta_theta_hk > 0){
                    if(foot_step_joy_(i,6) == 0)
                      foot_step_joy_(i,0) = (abs(center_hk) - temp*width)*sin(M_PI+delta_theta_hk*(i))-(abs(rfoot_float_init_.translation()(0))-abs(lfoot_float_init_.translation()(0)))/2;
                    else if(foot_step_joy_(i,6) == 1)
                      foot_step_joy_(i,0) = (abs(center_hk) - temp*width)*sin(M_PI+delta_theta_hk*(i))-(abs(lfoot_float_init_.translation()(0))-abs(rfoot_float_init_.translation()(0)))/2;
  
                    foot_step_joy_(i,1) = - (abs(center_hk) - temp*width)*cos(M_PI+delta_theta_hk*(i)) + center_hk;
                  }
                  
                }
                
                
                //foot_step_joy_(i,0) = foot_step_joy_(i-1,0) + temp*2*width*sin(foot_step_joy_(i-1,5)) + dlength*cos(foot_step_joy_(i-1,5));
                //foot_step_joy_(i,1) = foot_step_joy_(i-1,1) - temp*2*width*cos(foot_step_joy_(i-1,5)) + dlength*sin(foot_step_joy_(i-1,5));
            }
        }

    }
    else{ // no input orientation

//        cout<<"before temp setting : "<<endl<<foot_step_joy_<<endl;
//        cout<<"no orientation : "<<current_step_num_<<endl;
        if(current_step_num_ >2){
            if(foot_step_joy_(1,6) == 0){
                temp =1;
//                cout<<"temp +1"<<endl;
            }
            else if(foot_step_joy_(1,6) == 1){
//                cout<<"temp -1"<<endl;
                temp = -1;
            }
        }
        for(int i=0;i<5;i++){
            temp*= -1;

            foot_step_joy_(i,5) = 0.0;
            foot_step_joy_(i,6) = 0.5+ 0.5*temp;
        }

        if(current_step_num_<=2){
            temp = (foot_step_joy_(0,6) - 0.5)/0.5;
            foot_step_joy_(0,0) = dlength;
            foot_step_joy_(0,1) = -temp*width;

            for(int i=1;i<5;i++){
                temp = (foot_step_joy_(i,6) - 0.5)/0.5;
                foot_step_joy_(i,0) = foot_step_joy_(i-1,0) + temp*2*width*sin(foot_step_joy_(i-1,5))+dlength*cos(foot_step_joy_(i-1,5));
                foot_step_joy_(i,1) = foot_step_joy_(i-1,1) - temp*2*width*cos(foot_step_joy_(i-1,5)) + dlength*sin(foot_step_joy_(i-1,5));
            }
        }
        else if(current_step_num_>2){
            if(foot_step_joy_(1,6) ==1){ // left foot support
                foot_step_joy_(0,0) = lfoot_float_init_.translation()(0);
                foot_step_joy_(0,1) = lfoot_float_init_.translation()(1);

                foot_step_joy_(1,0) = rfoot_float_init_.translation()(0);
                foot_step_joy_(1,1) = rfoot_float_init_.translation()(1);
            }
            else if(foot_step_joy_(1,6) ==0){ //right foot support
                foot_step_joy_(0,0) = rfoot_float_init_.translation()(0);
                foot_step_joy_(0,1) = rfoot_float_init_.translation()(1);

                foot_step_joy_(1,0) = lfoot_float_init_.translation()(0);
                foot_step_joy_(1,1) = lfoot_float_init_.translation()(1);
            }

            for(int i=2;i<5;i++){
                temp = (foot_step_joy_(i,6) - 0.5)/0.5;
                foot_step_joy_(i,0) = foot_step_joy_(i-1,0) + temp*2*width*sin(foot_step_joy_(i-1,5))+dlength*cos(foot_step_joy_(i-1,5));
                foot_step_joy_(i,1) = foot_step_joy_(i-1,1) - temp*2*width*cos(foot_step_joy_(i-1,5)) + dlength*sin(foot_step_joy_(i-1,5));
            }
        }
    }

    //for(int i=0;i<5;i++){
        

    //}
    std::cout << center_hk << std::endl;


//    cout<<"joystick angle "<<ori_input*RAD2DEG<<", pre angle : "<<pre_theta_*RAD2DEG<<
//       "delta angle : "<<theta_total_*RAD2DEG<<",  at joystick iter: "<<joystick_iter_<<",  total num : "<<joystick_rotation_num_ <<endl<<foot_step_joy_<<endl;

    pre_theta_ = ori_input;
}

void WalkingController::JoyoriStop(double dlength, double width){
    cout<<"stop step planning  at "<<current_step_num_<<", at count : "<<walking_tick_<<endl;
    int temp;
    if(current_step_num_ <=2){
        temp = (foot_step_joy_(current_step_num_,6) - 0.5)/0.5;
        foot_step_joy_(current_step_num_,5) = foot_step_joy_(current_step_num_-1,5);

        foot_step_joy_(current_step_num_,0) = foot_step_joy_(current_step_num_-1,0) + temp*2*width*sin(foot_step_joy_(current_step_num_-1,5));
        foot_step_joy_(current_step_num_,1) = foot_step_joy_(current_step_num_-1,1) - temp*2*width*cos(foot_step_joy_(current_step_num_-1,5));

        for(int i=current_step_num_+1;i<5;i++){
            foot_step_joy_.row(i) = foot_step_joy_.row(i-2);
            foot_step_joy_(i,0) = foot_step_joy_(i-1,0);            
        }
    }
    else{
//        temp = (foot_step_joy_(4,6) -0.5)/0.5;
//        foot_step_joy_(4,0) = foot_step_joy_(3,0) + temp*2*width*sin(foot_step_joy_(3,5));
//        foot_step_joy_(4,1) = foot_step_joy_(3,1) - temp*2*width*cos(foot_step_joy_(3,5));
//        foot_step_joy_(4,5) = foot_step_joy_(3,5);
        for(int i=2;i<5;i++){
            temp = (foot_step_joy_(i,6) -0.5)/0.5;
            foot_step_joy_(i,0) = foot_step_joy_(i-1,0) + temp*2*width*sin(foot_step_joy_(i-1,5));
            foot_step_joy_(i,1) = foot_step_joy_(i-1,1) - temp*2*width*cos(foot_step_joy_(i-1,5));
            foot_step_joy_(i,5) = foot_step_joy_(i-1,5);
        }


    }
    //    if(joystick_planning_ == true)
    //        joystick_planning_ = false;
    if(joystick_walking_on_ == true){
        joystick_walking_on_ = false;
    }
}
void WalkingController::JoyFootstepGoon(){
   //function for joystick footstep update when left trigger on
    if(walking_tick_ == 0){
        foot_step_joy_.resize(5,7);
        foot_step_joy_support_frame_.resize(5,7);
        foot_step_joy_support_frame_offset_.resize(5,7);
        foot_step_joy_.setZero();
        foot_step_joy_support_frame_.setZero();
        foot_step_joy_support_frame_offset_.setZero();

        total_step_num_ = 5;
        pre_theta_ = 0.0;
    }
    unsigned int number_of_footstep =5;
    int temp =-1;
    int index =0;

    double dlength = joystick_input_(0)*0.2;
    double y_distance = 0.127794;
    double theta = joystick_input_(2);

    // setting the orientation of robot,


    if(joystick_input_(0) == 0){
        if(joystick_walking_on_ == true){
            total_step_num_joy_ = current_step_num_ +2; // finish at swing foot landing position.
            joystick_walking_on_ = false; // begin stop sequence
            joystick_stop_begin_num_ = current_step_num_;
            total_step_num_ = current_step_num_+2;
            cout<<" Joystick Walking Stop begin at "<<joystick_stop_begin_num_<<", at count : "<<walking_tick_<<endl;
            cout<<"new total step num : "<<total_step_num_<<endl;
        }

    }
    if(joystick_input_(0) != 0) {
        if(joystick_walking_on_ == false){
            joystick_walking_on_ = true;
            total_step_num_ = current_step_num_+5;
           cout<<"!!!!!!!!!!!!!!!Joystick Walking restart  :   "<<walking_tick_<<endl;
        }
    }
    Eigen::Matrix<double, 5,7> p_foot_step;
    p_foot_step.setZero();

    if(walking_tick_ ==0 || walking_tick_ == t_start_){
        if(joystick_input_(0) != 0){ // joystick walking cmd on
          total_step_num_ = current_step_num_+3;
          if(heel_toe_mode_ == false)
            Joyori();
          else if(heel_toe_mode_ == true)
            Joycrab();

        }//end of joysticik walking comd on
        else { // joystick_input(3)>0 and stop cmd on

            JoyoriStop(dlength,y_distance);
        }


    }
    else {
        foot_step_joy_ = foot_step_joy_;
    }
}
void WalkingController::JoyZMPtrajectory(){
    unsigned int planning_step_number  = 3;

    unsigned int norm_size = 0;

    //if(current_step_num_ >= total_step_num_ - planning_step_number)
    if(joystick_input_(0) == 0){ // stop cmd on
      norm_size = (t_last_-t_start_+1)*(total_step_num_joy_-current_step_num_)+20*hz_;
    }
    else
      norm_size = (t_last_-t_start_+1)*(planning_step_number);
    if(current_step_num_ == 0)
      norm_size = norm_size + t_temp_+1;


    JoyaddZmpOffset();

    JoyZMPGenerator(norm_size, planning_step_number);

    ref_zmp_.resize(norm_size,2);
    ref_zmp_ =ref_zmp_joy_;


}
void WalkingController::JoyZMPGenerator(const unsigned int norm_size, const unsigned planning_step_num){
    ref_zmp_joy_.resize(norm_size,2);

    com_offset_.setZero();

    // planning step num is 3
    Eigen::VectorXd temp_px;
    Eigen::VectorXd temp_py;

    unsigned int index =0;


    if(current_step_num_ ==0)
    {

      for (int i=0; i<= t_temp_; i++) //200 tick
      {          
        if(i <= 0.5*hz_)
        {
          ref_zmp_joy_(i,0) = com_support_init_(0)+com_offset_(0);
          ref_zmp_joy_(i,1) = com_support_init_(1)+com_offset_(1);
        }
        else if(i < 1.5*hz_)
        {
          double del_x = i-0.5*hz_;
          ref_zmp_joy_(i,0) = com_support_init_(0)+com_offset_(0)-del_x*(com_support_init_(0)+com_offset_(0))/(1.0*hz_);
          ref_zmp_joy_(i,1) = com_support_init_(1)+com_offset_(1);
        }
        else
        {
          ref_zmp_joy_(i,0) = 0.0;
          ref_zmp_joy_(i,1) = com_support_init_(1)+com_offset_(1);
        }
        index++;
      }
    }
    if(joystick_walking_on_==false && current_step_num_ != joystick_stop_begin_num_)
    {
        for(unsigned int i= current_step_num_; i<total_step_num_joy_;i++){           
                JoystopstepZmp(i,temp_px,temp_py);
            for (unsigned int j=0; j<t_total_; j++)
            {
              ref_zmp_joy_(index+j,0) = temp_px(j);
              ref_zmp_joy_(index+j,1) = temp_py(j);
             // ref_zmp_(index+j,0) = temp_px1(j);
            }

            index = index + t_total_;
        }

        for( unsigned int j=0;j<20*hz_;j++){
            ref_zmp_joy_(index+j,0) = ref_zmp_joy_(index-1,0);
            ref_zmp_joy_(index+j,1) = ref_zmp_joy_(index-1,1);
        }

    }
    else //joystick trigger ON --> walking sequence go on
    {
      for(unsigned int i=0; i < 3; i++)
      {
        JoyonestepZmp(i,temp_px,temp_py);
        for (unsigned int j=0; j<t_total_; j++)
        {
          ref_zmp_joy_(index+j,0) = temp_px(j);
          ref_zmp_joy_(index+j,1) = temp_py(j);

        }
        index = index+t_total_;
       }
      } //for if(current_step_number_>!!@!@!#~~~~~~~~!
//    if(walking_tick_ ==0 || walking_tick_==t_start_){
//        file[10]<<walking_tick_<<"\t"<<current_step_num_;
//        file[15]<<walking_tick_<<"\t"<<current_step_num_;
//        for(int i=0;i<norm_size;i++){
//            file[10]<<"\t"<<ref_zmp_joy_(i,1);
//            file[15]<<"\t"<<ref_zmp_joy_(i,0);
//        }
//        file[15]<<endl;
//        file[10]<<endl;
//    }
}

void WalkingController::JoyfloatToSupportFootstep(){
    Eigen::Isometry3d reference;
    double step_planning_num =5;

//    if(joystick_walking_on_ == true){
        if(current_step_num_ == 0){
            if(foot_step_joy_(0,6) == 0) //right support
            {
              reference.translation() = rfoot_float_init_.translation();
              reference.translation()(2) = 0.0;
              reference.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(rfoot_float_init_.linear())(2));
              reference.translation()(0) = 0.0;
            }
            else  //left support
            {
              reference.translation() = lfoot_float_init_.translation();
              reference.translation()(2) = 0.0;
              reference.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(lfoot_float_init_.linear())(2));
              reference.translation()(0) = 0.0;
            }
        }
        else if(current_step_num_<=2){
            reference.linear() = DyrosMath::rotateWithZ(foot_step_joy_(current_step_num_-1,5));
            for(int i=0 ;i<3; i++)
              reference.translation()(i) = foot_step_joy_(current_step_num_-1,i);

        }
        else {
            reference.linear() = DyrosMath::rotateWithZ(foot_step_joy_(1,5));
            for(int i=0 ;i<3; i++)
              reference.translation()(i) = foot_step_joy_(1,i);

           if(joystick_walking_on_ == false)
           {
                double seq = current_step_num_-joystick_stop_begin_num_;
                reference.linear() = DyrosMath::rotateWithZ(foot_step_joy_(1+seq,5));
                for(int i=0 ;i<3; i++)
                  reference.translation()(i) = foot_step_joy_(1+seq,i);
            }
        }



    Eigen::Vector3d temp_local_position;
    Eigen::Vector3d temp_global_position;

    if(current_step_num_ == 0)
    {

      for(int i=0; i<step_planning_num; i++)
      {
        for(int j=0; j<3; j++)
          temp_global_position(j)  = foot_step_joy_(i,j);

        temp_local_position = reference.linear().transpose()*(temp_global_position-reference.translation());

        for(int j=0; j<3; j++)
          foot_step_joy_support_frame_(i,j) = temp_local_position(j);

        foot_step_joy_support_frame_(i,3) = foot_step_joy_(i,3);
        foot_step_joy_support_frame_(i,4) = foot_step_joy_(i,4);
        foot_step_joy_support_frame_(i,5) = foot_step_joy_(i,5) - supportfoot_float_init_(5);

      }
    }
    else
    {
      for(int i=0; i<step_planning_num; i++)
      {
        for(int j=0; j<3; j++)
          temp_global_position(j)  = foot_step_joy_(i,j);

        temp_local_position = reference.linear().transpose()*(temp_global_position-reference.translation());

        for(int j=0; j<3; j++)
          foot_step_joy_support_frame_(i,j) = temp_local_position(j);

        if(current_step_num_ <=2){
            foot_step_joy_support_frame_(i,3) = foot_step_joy_(i,3);
            foot_step_joy_support_frame_(i,4) = foot_step_joy_(i,4);
            foot_step_joy_support_frame_(i,5) = foot_step_joy_(i,5) - foot_step_joy_(current_step_num_-1,5);
        }
        else {
            foot_step_joy_support_frame_(i,3) = foot_step_joy_(i,3);
            foot_step_joy_support_frame_(i,4) = foot_step_joy_(i,4);
            foot_step_joy_support_frame_(i,5) = foot_step_joy_(i,5) - foot_step_joy_(1,5);
        }
      }
    }


    for(int j=0;j<3;j++)
      temp_global_position(j) = supportfoot_float_init_(j);

    temp_local_position = reference.linear().transpose()*(temp_global_position-reference.translation());

    for(int j=0;j<3;j++)
      supportfoot_support_init_(j) = temp_local_position(j);

    supportfoot_support_init_(3) = supportfoot_float_init_(3);
    supportfoot_support_init_(4) = supportfoot_float_init_(4);

    if(current_step_num_ == 0)
      supportfoot_support_init_(5) = 0;
    else{
        if(current_step_num_<=2){
            supportfoot_support_init_(5) = supportfoot_float_init_(5) - foot_step_joy_(current_step_num_-1,5);
        }
        else {
            supportfoot_support_init_(5) = supportfoot_float_init_(5) - foot_step_joy_(1,5);
        }
    }


}
void WalkingController::JoyaddZmpOffset()
{
  lfoot_zmp_offset_ = -0.02;
  rfoot_zmp_offset_ = 0.02;

  double step_planning_num =5;


  foot_step_joy_support_frame_offset_ = foot_step_joy_support_frame_;

  if(foot_step_joy_(0,6) == 0) //right support foot
  {
    //  supportfoot_support_init_offset_(0) = supportfoot_support_init_(0)/2;
    supportfoot_support_init_offset_(1) = supportfoot_support_init_(1) + rfoot_zmp_offset_;
    swingfoot_support_init_offset_(1) = swingfoot_support_init_(1) + lfoot_zmp_offset_;
  }
  else //foot_step_joy_support_frame_
  {
    //  supportfoot_support_init_offset_(0) = supportfoot_support_init_(0);
    supportfoot_support_init_offset_(1) = supportfoot_support_init_(1) + lfoot_zmp_offset_;
    swingfoot_support_init_offset_(1) = swingfoot_support_init_(1) + rfoot_zmp_offset_;
  }
  if(joystick_walking_on_ == false){
          double seq = current_step_num_-joystick_stop_begin_num_;

          if(foot_step_joy_(seq,6) == 0) //right support foot
          {
            supportfoot_support_init_offset_(1) = supportfoot_support_init_(1) + rfoot_zmp_offset_;
            swingfoot_support_init_offset_(1) = swingfoot_support_init_(1) + lfoot_zmp_offset_;
          }
          else //foot_step_joy_support_frame_
          {
            supportfoot_support_init_offset_(1) = supportfoot_support_init_(1) + lfoot_zmp_offset_;
            swingfoot_support_init_offset_(1) = swingfoot_support_init_(1) + rfoot_zmp_offset_;
          }
//      }
  }

  if(joystick_walking_on_ == false && current_step_num_!= joystick_stop_begin_num_)
  {
      if(current_step_num_<=2){
          if(foot_step_joy_(current_step_num_-1,6) ==0)
          {
              for(int i=0;i<step_planning_num;i++){
                  foot_step_joy_support_frame_offset_(i,1) += lfoot_zmp_offset_;
              }
          }
          else {
              for(int i=0;i<step_planning_num;i++){
                  foot_step_joy_support_frame_offset_(i,1) += rfoot_zmp_offset_;
              }
          }
      }
      else{
          double seq = current_step_num_-joystick_stop_begin_num_;
          if(foot_step_joy_(1+seq,6)==0){
              for(int i=0;i<step_planning_num;i++){
                  foot_step_joy_support_frame_offset_(i,1) += lfoot_zmp_offset_;
              }
          }
          else {
              for(int i=0;i<step_planning_num;i++){
                  foot_step_joy_support_frame_offset_(i,1) += rfoot_zmp_offset_;
              }
          }
      }


  }
  else {
      for(int i=0; i<step_planning_num; i++)
      {
        if(foot_step_joy_(i,6) == 0)//right support, left swing
        {
          foot_step_joy_support_frame_offset_(i,1) += lfoot_zmp_offset_;
        }
        else
        {
          foot_step_joy_support_frame_offset_(i,1) += rfoot_zmp_offset_;
        }
      }
  }

}
void WalkingController::JoyonestepZmp(unsigned int tick, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py)
{
  temp_px.resize(t_total_);
  temp_py.resize(t_total_);
  temp_px.setZero();
  temp_py.setZero();

  double Kx = 0.0;
  double Kx2 = 0.0;
  double Ky = 0.0;
  double Ky2 = 0.0;

  if(current_step_num_ == 0)
  {
      if(tick ==0){
          Kx = supportfoot_support_init_offset_(0);
          Kx2 = (foot_step_joy_support_frame_(0,0)- supportfoot_support_init_offset_(0))/2.0;

          //    Kx2 = (foot_step_joy_support_frame_(current_step_number,0))/2.0;


          Ky = supportfoot_support_init_offset_(1) - com_support_init_(1);
          Ky2 = (foot_step_joy_support_frame_(0,1)- supportfoot_support_init_offset_(1))/2.0;

          //    Ky2 = (foot_step_joy_support_frame_(current_step_number,1))/2.0;

          for(int i=0; i<t_total_; i++)
          {
            if(i < t_rest_init_)
            {
              temp_px(i) = 0.0;
              temp_py(i) = com_support_init_(1)+com_offset_(1);
            }
            else if(i >= t_rest_init_ && i < t_rest_init_+t_double1_)
            {
              temp_px(i) = Kx/t_double1_*(i+1-t_rest_init_);
              temp_py(i) = com_support_init_(1)+com_offset_(1) + Ky/t_double1_*(i+1-t_rest_init_);
            }
            else if(i>= t_rest_init_+t_double1_ && i< t_total_-t_rest_last_-t_double2_)
            {
              temp_px(i) = supportfoot_support_init_offset_(0);
              temp_py(i) = supportfoot_support_init_offset_(1);
            }
            else if(i >= t_total_-t_rest_last_-t_double2_ && i< t_total_-t_rest_last_)
            {
              temp_px(i) = supportfoot_support_init_offset_(0) + Kx2/(t_double2_)*(i+1-(t_total_-t_double2_-t_rest_last_));
              temp_py(i) = supportfoot_support_init_offset_(1) + Ky2/(t_double2_)*(i+1-(t_total_-t_double2_-t_rest_last_));
            }
            else
            {
              temp_px(i) = temp_px(i-1);
              temp_py(i) = temp_py(i-1);
            }
          }
      } // end of tick 0
      else if(tick ==1){
          Kx = foot_step_joy_support_frame_offset_(tick-1,0) - (foot_step_joy_support_frame_(tick-1,0) + supportfoot_support_init_(0))/2.0;
          Kx2 = (foot_step_joy_support_frame_(tick,0)+foot_step_joy_support_frame_(tick-1,0))/2.0 - foot_step_joy_support_frame_offset_(tick-1,0);

          Ky =  foot_step_joy_support_frame_offset_(tick-1,1) - (foot_step_joy_support_frame_(tick-1,1) + supportfoot_support_init_(1))/2.0;
          Ky2 = (foot_step_joy_support_frame_(tick,1)+foot_step_joy_support_frame_(tick-1,1))/2.0 - foot_step_joy_support_frame_offset_(tick-1,1);
          for(int i=0; i<t_total_; i++)
          {
            if(i < t_rest_init_)
            {
              temp_px(i) = (foot_step_joy_support_frame_(tick-1,0)+supportfoot_support_init_(0))/2.0;
              temp_py(i) = (foot_step_joy_support_frame_(tick-1,1)+supportfoot_support_init_(1))/2.0;

            }
            else if(i >= t_rest_init_ && i < t_rest_init_+t_double1_)
            {
              temp_px(i) = (foot_step_joy_support_frame_(tick-1,0)+supportfoot_support_init_(0))/2.0 + Kx/t_double1_*(i+1-t_rest_init_);
              temp_py(i) = (foot_step_joy_support_frame_(tick-1,1)+supportfoot_support_init_(1))/2.0 + Ky/t_double1_*(i+1-t_rest_init_);
            }
            else if(i>= t_rest_init_+t_double1_ & i< t_total_-t_rest_last_-t_double2_)
            {
              temp_px(i) = foot_step_joy_support_frame_offset_(tick-1,0);
              temp_py(i) = foot_step_joy_support_frame_offset_(tick-1,1);
            }
            else if(i >= t_total_-t_rest_last_-t_double2_ && i< t_total_-t_rest_last_)
            {
              temp_px(i) = foot_step_joy_support_frame_offset_(tick-1,0) + Kx2/(t_double2_)*(i+1-(t_total_-t_double2_-t_rest_last_));
              temp_py(i) = foot_step_joy_support_frame_offset_(tick-1,1) + Ky2/(t_double2_)*(i+1-(t_total_-t_double2_-t_rest_last_));
            }
            else
            {
              temp_px(i) = temp_px(i-1);
              temp_py(i) = temp_py(i-1);
            }
          }
      }
      else{// tick is 2
          Kx = foot_step_joy_support_frame_offset_(tick-1,0) - (foot_step_joy_support_frame_(tick-1,0) + foot_step_joy_support_frame_(tick-2,0))/2.0;
          Kx2 = (foot_step_joy_support_frame_(tick,0)+foot_step_joy_support_frame_(tick-1,0))/2.0 - foot_step_joy_support_frame_offset_(tick-1,0);

          Ky =  foot_step_joy_support_frame_offset_(tick-1,1) - (foot_step_joy_support_frame_(tick-1,1) + foot_step_joy_support_frame_(tick-2,1))/2.0;
          Ky2 = (foot_step_joy_support_frame_(tick,1)+foot_step_joy_support_frame_(tick-1,1))/2.0 -  foot_step_joy_support_frame_offset_(tick-1,1);

          for(int i=0; i<t_total_; i++)
          {
            if(i < t_rest_init_)
            {
              temp_px(i) = (foot_step_joy_support_frame_(tick-1,0)+foot_step_joy_support_frame_(tick-2,0))/2.0;
              temp_py(i) = (foot_step_joy_support_frame_(tick-1,1)+foot_step_joy_support_frame_(tick-2,1))/2.0;
            }
            else if(i >= t_rest_init_ && i < t_rest_init_+t_double1_)
            {
              temp_px(i) = (foot_step_joy_support_frame_(tick-1,0)+foot_step_joy_support_frame_(tick-2,0))/2.0 + Kx/t_double1_*(i+1-t_rest_init_);
              temp_py(i) = (foot_step_joy_support_frame_(tick-1,1)+foot_step_joy_support_frame_(tick-2,1))/2.0 + Ky/t_double1_*(i+1-t_rest_init_);
            }
            else if(i>= t_rest_init_+t_double1_ & i< t_total_-t_rest_last_-t_double2_)
            {
              temp_px(i) = foot_step_joy_support_frame_offset_(tick-1,0);
              temp_py(i) = foot_step_joy_support_frame_offset_(tick-1,1);
            }
            else if(i >= t_total_-t_rest_last_-t_double2_ && i< t_total_-t_rest_last_)
            {
              temp_px(i) = foot_step_joy_support_frame_offset_(tick-1,0) + Kx2/(t_double2_)*(i+1-(t_total_-t_double2_-t_rest_last_));
              temp_py(i) = foot_step_joy_support_frame_offset_(tick-1,1) + Ky2/(t_double2_)*(i+1-(t_total_-t_double2_-t_rest_last_));
            }
            else
            {
              temp_px(i) = temp_px(i-1);
              temp_py(i) = temp_py(i-1);
            }
          }
      }

  }
  else if(current_step_num_ ==1){
      if(tick ==0){
          Kx = foot_step_joy_support_frame_offset_(tick,0) - (foot_step_joy_support_frame_(tick,0) + supportfoot_support_init_(0))/2.0;
          Kx2 = (foot_step_joy_support_frame_(tick+1,0)+foot_step_joy_support_frame_(tick,0))/2.0 - foot_step_joy_support_frame_offset_(tick,0);

          Ky =  foot_step_joy_support_frame_offset_(tick,1) - (foot_step_joy_support_frame_(tick,1) + supportfoot_support_init_(1))/2.0;
          Ky2 = (foot_step_joy_support_frame_(tick+1,1)+foot_step_joy_support_frame_(tick,1))/2.0 - foot_step_joy_support_frame_offset_(tick,1);
          for(int i=0; i<t_total_; i++)
          {
            if(i < t_rest_init_)
            {
              temp_px(i) = (foot_step_joy_support_frame_(tick,0)+supportfoot_support_init_(0))/2.0;
              temp_py(i) = (foot_step_joy_support_frame_(tick,1)+supportfoot_support_init_(1))/2.0;

            }
            else if(i >= t_rest_init_ && i < t_rest_init_+t_double1_)
            {
              temp_px(i) = (foot_step_joy_support_frame_(tick,0)+supportfoot_support_init_(0))/2.0 + Kx/t_double1_*(i+1-t_rest_init_);
              temp_py(i) = (foot_step_joy_support_frame_(tick,1)+supportfoot_support_init_(1))/2.0 + Ky/t_double1_*(i+1-t_rest_init_);
            }
            else if(i>= t_rest_init_+t_double1_ & i< t_total_-t_rest_last_-t_double2_)
            {
              temp_px(i) = foot_step_joy_support_frame_offset_(tick,0);
              temp_py(i) = foot_step_joy_support_frame_offset_(tick,1);
            }
            else if(i >= t_total_-t_rest_last_-t_double2_ && i< t_total_-t_rest_last_)
            {
              temp_px(i) = foot_step_joy_support_frame_offset_(tick,0) + Kx2/(t_double2_)*(i+1-(t_total_-t_double2_-t_rest_last_));
              temp_py(i) = foot_step_joy_support_frame_offset_(tick,1) + Ky2/(t_double2_)*(i+1-(t_total_-t_double2_-t_rest_last_));
            }
            else
            {
              temp_px(i) = temp_px(i-1);
              temp_py(i) = temp_py(i-1);
            }
          }
      }
      else if(tick ==1){
          Kx = foot_step_joy_support_frame_offset_(tick,0) - (foot_step_joy_support_frame_(tick-1,0) + foot_step_joy_support_frame_(tick,0))/2.0;
          Kx2 = (foot_step_joy_support_frame_(tick,0)+foot_step_joy_support_frame_(tick+1,0))/2.0 - foot_step_joy_support_frame_offset_(tick,0);

          Ky =  foot_step_joy_support_frame_offset_(tick,1) - (foot_step_joy_support_frame_(tick-1,1) + foot_step_joy_support_frame_(tick,1))/2.0;
          Ky2 = (foot_step_joy_support_frame_(tick,1)+foot_step_joy_support_frame_(tick+1,1))/2.0 -  foot_step_joy_support_frame_offset_(tick,1);

          for(int i=0; i<t_total_; i++)
          {
            if(i < t_rest_init_)
            {
              temp_px(i) = (foot_step_joy_support_frame_(tick-1,0)+foot_step_joy_support_frame_(tick,0))/2.0;
              temp_py(i) = (foot_step_joy_support_frame_(tick-1,1)+foot_step_joy_support_frame_(tick,1))/2.0;
            }
            else if(i >= t_rest_init_ && i < t_rest_init_+t_double1_)
            {
              temp_px(i) = (foot_step_joy_support_frame_(tick-1,0)+foot_step_joy_support_frame_(tick,0))/2.0 + Kx/t_double1_*(i+1-t_rest_init_);
              temp_py(i) = (foot_step_joy_support_frame_(tick-1,1)+foot_step_joy_support_frame_(tick,1))/2.0 + Ky/t_double1_*(i+1-t_rest_init_);
            }
            else if(i>= t_rest_init_+t_double1_ & i< t_total_-t_rest_last_-t_double2_)
            {
              temp_px(i) = foot_step_joy_support_frame_offset_(tick,0);
              temp_py(i) = foot_step_joy_support_frame_offset_(tick,1);
            }
            else if(i >= t_total_-t_rest_last_-t_double2_ && i< t_total_-t_rest_last_)
            {
              temp_px(i) = foot_step_joy_support_frame_offset_(tick,0) + Kx2/(t_double2_)*(i+1-(t_total_-t_double2_-t_rest_last_));
              temp_py(i) = foot_step_joy_support_frame_offset_(tick,1) + Ky2/(t_double2_)*(i+1-(t_total_-t_double2_-t_rest_last_));
            }
            else
            {
              temp_px(i) = temp_px(i-1);
              temp_py(i) = temp_py(i-1);
            }
          }
      }
      else { // tick is 2
          Kx = foot_step_joy_support_frame_offset_(tick,0) - (foot_step_joy_support_frame_(tick-1,0) + foot_step_joy_support_frame_(tick,0))/2.0;
          Kx2 = (foot_step_joy_support_frame_(tick,0)+foot_step_joy_support_frame_(tick+1,0))/2.0 - foot_step_joy_support_frame_offset_(tick,0);

          Ky =  foot_step_joy_support_frame_offset_(tick,1) - (foot_step_joy_support_frame_(tick-1,1) + foot_step_joy_support_frame_(tick,1))/2.0;
          Ky2 = (foot_step_joy_support_frame_(tick,1)+foot_step_joy_support_frame_(tick+1,1))/2.0 -  foot_step_joy_support_frame_offset_(tick,1);

          for(int i=0; i<t_total_; i++)
          {
            if(i < t_rest_init_)
            {
              temp_px(i) = (foot_step_joy_support_frame_(tick-1,0)+foot_step_joy_support_frame_(tick,0))/2.0;
              temp_py(i) = (foot_step_joy_support_frame_(tick-1,1)+foot_step_joy_support_frame_(tick,1))/2.0;
            }
            else if(i >= t_rest_init_ && i < t_rest_init_+t_double1_)
            {
              temp_px(i) = (foot_step_joy_support_frame_(tick-1,0)+foot_step_joy_support_frame_(tick,0))/2.0 + Kx/t_double1_*(i+1-t_rest_init_);
              temp_py(i) = (foot_step_joy_support_frame_(tick-1,1)+foot_step_joy_support_frame_(tick,1))/2.0 + Ky/t_double1_*(i+1-t_rest_init_);
            }
            else if(i>= t_rest_init_+t_double1_ & i< t_total_-t_rest_last_-t_double2_)
            {
              temp_px(i) = foot_step_joy_support_frame_offset_(tick,0);
              temp_py(i) = foot_step_joy_support_frame_offset_(tick,1);
            }
            else if(i >= t_total_-t_rest_last_-t_double2_ && i< t_total_-t_rest_last_)
            {
              temp_px(i) = foot_step_joy_support_frame_offset_(tick,0) + Kx2/(t_double2_)*(i+1-(t_total_-t_double2_-t_rest_last_));
              temp_py(i) = foot_step_joy_support_frame_offset_(tick,1) + Ky2/(t_double2_)*(i+1-(t_total_-t_double2_-t_rest_last_));
            }
            else
            {
              temp_px(i) = temp_px(i-1);
              temp_py(i) = temp_py(i-1);
            }
          }
      }
  }
  else{ // current step num >= 2
      Kx = foot_step_joy_support_frame_offset_(tick+1,0) - (foot_step_joy_support_frame_(tick+1,0) + foot_step_joy_support_frame_(tick,0))/2.0;
      Kx2 = (foot_step_joy_support_frame_(tick+2,0)+foot_step_joy_support_frame_(tick+1,0))/2.0 - foot_step_joy_support_frame_offset_(tick+1,0);

      Ky =  foot_step_joy_support_frame_offset_(tick+1,1) - (foot_step_joy_support_frame_(tick+1,1) + foot_step_joy_support_frame_(tick,1))/2.0;
      Ky2 = (foot_step_joy_support_frame_(tick+2,1)+foot_step_joy_support_frame_(tick+1,1))/2.0 -  foot_step_joy_support_frame_offset_(tick+1,1);

      for(int i=0; i<t_total_; i++)
      {
        if(i < t_rest_init_)
        {
          temp_px(i) = (foot_step_joy_support_frame_(tick+1,0)+foot_step_joy_support_frame_(tick,0))/2.0;
          temp_py(i) = (foot_step_joy_support_frame_(tick+1,1)+foot_step_joy_support_frame_(tick,1))/2.0;
        }
        else if(i >= t_rest_init_ && i < t_rest_init_+t_double1_)
        {
          temp_px(i) = (foot_step_joy_support_frame_(tick+1,0)+foot_step_joy_support_frame_(tick,0))/2.0 + Kx/t_double1_*(i+1-t_rest_init_);
          temp_py(i) = (foot_step_joy_support_frame_(tick+1,1)+foot_step_joy_support_frame_(tick,1))/2.0 + Ky/t_double1_*(i+1-t_rest_init_);
        }
        else if(i>= t_rest_init_+t_double1_ & i< t_total_-t_rest_last_-t_double2_)
        {
          temp_px(i) = foot_step_joy_support_frame_offset_(tick+1,0);
          temp_py(i) = foot_step_joy_support_frame_offset_(tick+1,1);
        }
        else if(i >= t_total_-t_rest_last_-t_double2_ && i< t_total_-t_rest_last_)
        {
          temp_px(i) = foot_step_joy_support_frame_offset_(tick+1,0) + Kx2/(t_double2_)*(i+1-(t_total_-t_double2_-t_rest_last_));
          temp_py(i) = foot_step_joy_support_frame_offset_(tick+1,1) + Ky2/(t_double2_)*(i+1-(t_total_-t_double2_-t_rest_last_));
        }
        else
        {
          temp_px(i) = temp_px(i-1);
          temp_py(i) = temp_py(i-1);
        }
      }
  }
}
void WalkingController::JoystopstepZmp(unsigned int step_number, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py)
{
  temp_px.resize(t_total_);
  temp_py.resize(t_total_);
  temp_px.setZero();
  temp_py.setZero();

  double Kx = 0.0;
  double Kx2 = 0.0;
  double Ky = 0.0;
  double Ky2 = 0.0;

 int tick = step_number - joystick_stop_begin_num_;


 if(current_step_num_ ==2)
     tick -=1;

      Kx = foot_step_joy_support_frame_offset_(tick+1,0) - (foot_step_joy_support_frame_(tick+1,0) + foot_step_joy_support_frame_(tick,0))/2.0;
      Kx2 = (foot_step_joy_support_frame_(tick+2,0)+foot_step_joy_support_frame_(tick+1,0))/2.0 - foot_step_joy_support_frame_offset_(tick+1,0);

      Ky =  foot_step_joy_support_frame_offset_(tick+1,1) - (foot_step_joy_support_frame_(tick+1,1) + foot_step_joy_support_frame_(tick,1))/2.0;
      Ky2 = (foot_step_joy_support_frame_(tick+2,1)+foot_step_joy_support_frame_(tick+1,1))/2.0 -  foot_step_joy_support_frame_offset_(tick+1,1);

      for(int i=0; i<t_total_; i++)
      {

        if(i < t_rest_init_)
        {
          temp_px(i) = (foot_step_joy_support_frame_(tick+1,0)+foot_step_joy_support_frame_(tick,0))/2.0;
          temp_py(i) = (foot_step_joy_support_frame_(tick+1,1)+foot_step_joy_support_frame_(tick,1))/2.0;
        }
        else if(i >= t_rest_init_ && i < t_rest_init_+t_double1_)
        {
          temp_px(i) = (foot_step_joy_support_frame_(tick+1,0)+foot_step_joy_support_frame_(tick,0))/2.0 + Kx/t_double1_*(i+1-t_rest_init_);
          temp_py(i) = (foot_step_joy_support_frame_(tick+1,1)+foot_step_joy_support_frame_(tick,1))/2.0 + Ky/t_double1_*(i+1-t_rest_init_);
        }
        else if(i>= t_rest_init_+t_double1_ & i< t_total_-t_rest_last_-t_double2_)
        {
          temp_px(i) = foot_step_joy_support_frame_offset_(tick+1,0);
          temp_py(i) = foot_step_joy_support_frame_offset_(tick+1,1);
        }
        else if(i >= t_total_-t_rest_last_-t_double2_ && i< t_total_-t_rest_last_)
        {
          temp_px(i) = foot_step_joy_support_frame_offset_(tick+1,0) + Kx2/(t_double2_)*(i+1-(t_total_-t_double2_-t_rest_last_));
          temp_py(i) = foot_step_joy_support_frame_offset_(tick+1,1) + Ky2/(t_double2_)*(i+1-(t_total_-t_double2_-t_rest_last_));
        }
        else
        {
          temp_px(i) = temp_px(i-1);
          temp_py(i) = temp_py(i-1);
        }
      }
}
void WalkingController::JoyFootTrajectory()
{
  Eigen::Vector6d target_swing_foot;

  bool left_foot_support;
  bool right_foot_support;

  if(joystick_walking_flag_==true){
      if(current_step_num_<=2){
          for(int i=0; i<6; i++)
            target_swing_foot(i) = foot_step_joy_support_frame_(current_step_num_,i);

          if(foot_step_joy_(current_step_num_,6) ==1){
              left_foot_support = true;
              right_foot_support = false;
          }
          else {
              left_foot_support = false;
              right_foot_support = true;
          }
      }
      else {
          if(joystick_walking_on_ == true){
              for(int i=0; i<6; i++)
                target_swing_foot(i) = foot_step_joy_support_frame_(2,i);

              if(foot_step_joy_(2,6) ==1){
                  left_foot_support = true;
                  right_foot_support = false;
              }
              else {
                  left_foot_support = false;
                  right_foot_support = true;
              }
          }
          else {
              double tick;
              tick = current_step_num_ - joystick_stop_begin_num_;
              for(int i=0;i<6;i++)
                  target_swing_foot(i) = foot_step_joy_support_frame_(2+tick,i);

              if(foot_step_joy_(2+tick,6) ==1){
                  left_foot_support = true;
                  right_foot_support = false;
              }
              else {
                  left_foot_support = false;
                  right_foot_support = true;
              }
          }

      }

  }

  if(walking_tick_ < t_start_real_+t_double1_)
  {
    lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
    lfoot_trajectory_dot_support_.setZero();


    if(left_foot_support) //left foot support
      lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,lfoot_support_init_.translation()(2),0.0,0.0,0.0);
    else // swing foot (right foot support)
    {
      if(current_step_num_ == 0)
        lfoot_trajectory_support_.translation()(2) = lfoot_support_init_.translation()(2);
      else
      {
        if(walking_tick_ < t_start_)
          lfoot_trajectory_support_.translation()(2) = lfoot_support_init_.translation()(2);
        else if(walking_tick_ >= t_start_ && walking_tick_ < t_start_real_)
          lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,lfoot_support_init_.translation()(2),0.0,0.0,0.0);
        else
          lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_,t_start_real_+t_double1_,0.0,0.0,0.0,0.0);
      }
    }


    lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;

    for(int i=0; i<2; i++)
      lfoot_trajectory_euler_support_(i) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,lfoot_support_euler_init_(i),0.0,0.0,0.0);

    lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));


    rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
    rfoot_trajectory_dot_support_.setZero();
    //rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_,_T_Start_real,rfoot_trajectory_init_.translation()(2),0.0,0.0,0.0);


    if(right_foot_support) //right foot support
      rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,rfoot_support_init_.translation()(2),0.0,0.0,0.0);
    else // swing foot (left foot support)
    {
      if(current_step_num_ == 0)
        rfoot_trajectory_support_.translation()(2) = rfoot_support_init_.translation()(2);
      else
      {
        if(walking_tick_ < t_start_)
          rfoot_trajectory_support_.translation()(2) = rfoot_support_init_.translation()(2);
        else if(walking_tick_ >= t_start_ && walking_tick_ < t_start_real_)
          rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,rfoot_support_init_.translation()(2),0.0,0.0,0.0);
        else
          rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_,t_start_real_+t_double1_,0.0,0.0,0.0,0.0);
      }
    }


    rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;

    for(int i=0; i<2; i++)
      rfoot_trajectory_euler_support_(i) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,rfoot_support_euler_init_(i),0.0,0.0,0.0);

    rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));

  }
  else if(walking_tick_ >= t_start_real_+t_double1_ && walking_tick_ < t_start_+t_total_-t_double2_-t_rest_last_)
  {
    double t_rest_temp = 0.05*hz_;
    double ankle_temp;
    ankle_temp = 0*DEG2RAD;

    if(left_foot_support) //Left foot support : Left foot is fixed at initial values, and Right foot is set to go target position
    {
      lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
      lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;
      lfoot_trajectory_euler_support_.setZero();

      lfoot_trajectory_dot_support_.setZero();
      lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));

      // setting for Left supporting foot

      if(walking_tick_ < t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0) // the period for lifting the right foot
      {
        rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0,foot_height_,0.0,0.0);
        rfoot_trajectory_dot_support_(2) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0,foot_height_,0.0,0.0,hz_);

        rfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0.0,ankle_temp,0.0,0.0);
        rfoot_trajectory_dot_support_(4) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0.0,ankle_temp,0.0,0.0,hz_);
      } // the period for lifting the right foot
      else
      {
        rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-t_rest_temp,foot_height_,target_swing_foot(2),0.0,0.0);
        rfoot_trajectory_dot_support_(2) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-t_rest_temp,foot_height_,target_swing_foot(2),0.0,0.0,hz_);

        rfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_-t_rest_temp,t_start_+t_total_-t_rest_last_,ankle_temp,0.0,0.0,0.0);
        rfoot_trajectory_dot_support_(4) = DyrosMath::cubicDot(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_-t_rest_temp,t_start_+t_total_-t_rest_last_,ankle_temp,0.0,0.0,0.0,hz_);
      } // the period for putting the right foot

      rfoot_trajectory_euler_support_(0) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,0.0,target_swing_foot(0+3),0.0,0.0);
      rfoot_trajectory_dot_support_(0+3) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,0.0,target_swing_foot(0+3),0.0,0.0,hz_);

      for(int i=0; i<2; i++)
      {
        rfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+2*t_rest_temp,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-2*t_rest_temp,rfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0);
        rfoot_trajectory_dot_support_(i) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+2*t_rest_temp,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-2*t_rest_temp,rfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0,hz_);
      }

      rfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,rfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0);
      rfoot_trajectory_dot_support_(5) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,rfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0,hz_);

      rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
    }
    else if(right_foot_support) // Right foot support : Right foot is fixed at initial values, and Left foot is set to go target position
    {
      rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
      rfoot_trajectory_support_.translation()(2) = 0.0;
      //rfoot_trajectory_support_.linear() = rfoot_trajectory_init_.linear();
      rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;
      rfoot_trajectory_euler_support_(0) = 0.0;
      rfoot_trajectory_euler_support_(1) = 0.0;
      rfoot_trajectory_dot_support_.setZero();

      double ankle_temp;
      ankle_temp = 0*DEG2RAD;
      //ankle_temp = -15*DEG2RAD;

      rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));

      if(walking_tick_ < t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0)
      {

        lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0,foot_height_,0.0,0.0);
        lfoot_trajectory_dot_support_(2) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0,foot_height_,0.0,0.0,hz_);

        lfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0.0,ankle_temp,0.0,0.0);
        lfoot_trajectory_dot_support_(4) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0.0,ankle_temp,0.0,0.0,hz_);

      }
      else
      {
        lfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_-t_rest_temp,t_start_+t_total_-t_rest_last_,ankle_temp,0.0,0.0,0.0);
        lfoot_trajectory_dot_support_(4) = DyrosMath::cubicDot(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_-t_rest_temp,t_start_+t_total_-t_rest_last_,ankle_temp,0.0,0.0,0.0,hz_);


        lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-t_rest_temp,foot_height_,target_swing_foot(2),0.0,0.0);
        lfoot_trajectory_dot_support_(2) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-t_rest_temp,foot_height_,target_swing_foot(2),0.0,0.0,hz_);
      }

      lfoot_trajectory_euler_support_(0) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,0.0,target_swing_foot(0+3),0.0,0.0);
      lfoot_trajectory_dot_support_(0+3) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,0.0,target_swing_foot(0+3),0.0,0.0,hz_);

      for(int i=0; i<2; i++)
      {
        lfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+2*t_rest_temp,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-2*t_rest_temp,lfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0);
        lfoot_trajectory_dot_support_(i) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+2*t_rest_temp,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-2*t_rest_temp,lfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0,hz_);
      }

      //  for(int i=0; i<3; i++)
      //  {
      //      lfoot_trajectory_euler_support_(i) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,_T_Start+t_total_-t_rest_last_-t_double2_-t_imp_,0.0,0.0,target_swing_foot(i+3),0.0);
      //      lfoot_trajectory_dot_support_(i+3) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_,_T_Start+t_total_-t_rest_last_-t_double2_-t_imp_,0.0,0.0,target_swing_foot(i+3),0.0,hz_);
      //  }


      lfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,lfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0);
      lfoot_trajectory_dot_support_(5) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,lfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0,hz_);

      lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
    }
    else
    {
      lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
      lfoot_trajectory_support_.linear() = lfoot_support_init_.linear();
      lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;
      lfoot_trajectory_dot_support_.setZero();

      rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
      rfoot_trajectory_support_.linear() = rfoot_support_init_.linear();
      rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;
      rfoot_trajectory_dot_support_.setZero();
    }
  }
  else
  {
    if(left_foot_support)
    {
      lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
      lfoot_trajectory_support_.translation()(2) = 0.0;
      lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;
      lfoot_trajectory_euler_support_(0) = 0.0;
      lfoot_trajectory_euler_support_(1) = 0.0;
      lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
      lfoot_trajectory_dot_support_.setZero();

      for(int i=0; i<3; i++)
      {
        rfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
        rfoot_trajectory_euler_support_(i) = target_swing_foot(i+3);
      }
      rfoot_trajectory_dot_support_.setZero();

      rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
    }
    else if (right_foot_support)
    {
      rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
      rfoot_trajectory_support_.translation()(2) = 0.0;
      //rfoot_trajectory_support_.linear() = rfoot_trajectory_init_.linear();
      rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;
      rfoot_trajectory_euler_support_(0) = 0.0;
      rfoot_trajectory_euler_support_(1) = 0.0;
      rfoot_trajectory_dot_support_.setZero();

      rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));


      for(int i=0; i<3; i++)
      {
        lfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
        lfoot_trajectory_euler_support_(i) = target_swing_foot(i+3);
      }
      lfoot_trajectory_dot_support_.setZero();
      lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
    }
    else
    {
      lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
      lfoot_trajectory_support_.linear() = lfoot_support_init_.linear();
      lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;
      lfoot_trajectory_dot_support_.setZero();

      rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
      rfoot_trajectory_support_.linear() = rfoot_support_init_.linear();
      rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;
      rfoot_trajectory_dot_support_.setZero();
    }
  }
}
void WalkingController::JoyupdateInitialState(){
    if( walking_tick_ ==0)
    {
      thread_tick_ = 0;

      q_init_ = current_q_;
      desired_q_not_compensated_ = current_q_;
      lfoot_float_init_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)(0));
      rfoot_float_init_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)(1));
      com_float_init_ = model_.getCurrentCom();

      lfoot_float_euler_init_ = DyrosMath::rot2Euler(lfoot_float_init_.linear());
      rfoot_float_euler_init_ = DyrosMath::rot2Euler(rfoot_float_init_.linear());

      pelv_float_init_.setIdentity();

      Eigen::Isometry3d ref_frame;

      if(joystick_walking_flag_ == true){
          if(joystick_walking_on_ == true){
              if(foot_step_joy_(0, 6) == 0)  //right foot support
              {
                ref_frame = rfoot_float_init_;
              }
              else if(foot_step_joy_(0, 6) == 1)
              {
                ref_frame = lfoot_float_init_;
              }
          }
          else{
              double tick = current_step_num_ - joystick_stop_begin_num_;
              if(foot_step_joy_(0+tick, 6) == 0)  //right foot support
              {
                ref_frame = rfoot_float_init_;
              }
              else if(foot_step_joy_(0+tick, 6) == 1)
              {
                ref_frame = lfoot_float_init_;
              }
          }


      }

      lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),lfoot_float_init_);
      rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),rfoot_float_init_);
      pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame);
      com_support_init_ = pelv_support_init_.linear()*com_float_init_ + pelv_support_init_.translation();



      pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear());
      rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
      lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());

      supportfoot_float_init_.setZero();
      swingfoot_float_init_.setZero();



      if(joystick_walking_flag_ == true){
          if(joystick_walking_on_ == true){
              if(foot_step_joy_(0,6) == 1)  //left suppport foot
              {
                for(int i=0; i<2; i++)
                  supportfoot_float_init_(i) = lfoot_float_init_.translation()(i);
                for(int i=0; i<3; i++)
                  supportfoot_float_init_(i+3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

                for(int i=0; i<2; i++)
                  swingfoot_float_init_(i) = rfoot_float_init_.translation()(i);
                for(int i=0; i<3; i++)
                  swingfoot_float_init_(i+3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

                supportfoot_float_init_(0) = 0.0;
                swingfoot_float_init_(0) = 0.0;
              }
              else
              {
                for(int i=0; i<2; i++)
                  supportfoot_float_init_(i) = rfoot_float_init_.translation()(i);
                for(int i=0; i<3; i++)
                  supportfoot_float_init_(i+3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

                for(int i=0; i<2; i++)
                  swingfoot_float_init_(i) = lfoot_float_init_.translation()(i);
                for(int i=0; i<3; i++)
                  swingfoot_float_init_(i+3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

                supportfoot_float_init_(0) = 0.0;
                swingfoot_float_init_(0) = 0.0;
              }

          }
          else {
              double tick = current_step_num_ - joystick_stop_begin_num_;
              if(foot_step_joy_(0+tick,6) == 1)  //left suppport foot
              {
                for(int i=0; i<2; i++)
                  supportfoot_float_init_(i) = lfoot_float_init_.translation()(i);
                for(int i=0; i<3; i++)
                  supportfoot_float_init_(i+3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

                for(int i=0; i<2; i++)
                  swingfoot_float_init_(i) = rfoot_float_init_.translation()(i);
                for(int i=0; i<3; i++)
                  swingfoot_float_init_(i+3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

                supportfoot_float_init_(0) = 0.0;
                swingfoot_float_init_(0) = 0.0;
              }
              else
              {
                for(int i=0; i<2; i++)
                  supportfoot_float_init_(i) = rfoot_float_init_.translation()(i);
                for(int i=0; i<3; i++)
                  supportfoot_float_init_(i+3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

                for(int i=0; i<2; i++)
                  swingfoot_float_init_(i) = lfoot_float_init_.translation()(i);
                for(int i=0; i<3; i++)
                  swingfoot_float_init_(i+3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

                supportfoot_float_init_(0) = 0.0;
                swingfoot_float_init_(0) = 0.0;
              }
          }

      }

      zc_ = com_support_init_(2);
      pelv_suppprt_start_ = pelv_support_init_;


      cout<<"total number of step : "<<total_step_num_<<endl;

      ///preview control variable///
      if(com_control_mode_ == true)
      {
        xi_ = com_support_init_(0);
        yi_ = com_support_init_(1);
      }
      else
      {
        xi_ = pelv_support_init_.translation()(0)+com_offset_(0);
        yi_ = pelv_support_init_.translation()(1)+com_offset_(1);
      }

    }
    else if(current_step_num_!=0 && walking_tick_ == t_start_)
    {
      //model_.updateKinematics(current_q_);
      q_init_ = current_q_;
      lfoot_float_init_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)(0));
      rfoot_float_init_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)(1));
      com_float_init_ = model_.getCurrentCom();

      lfoot_float_euler_init_ = DyrosMath::rot2Euler(lfoot_float_init_.linear());
      rfoot_float_euler_init_ = DyrosMath::rot2Euler(rfoot_float_init_.linear());


      pelv_float_init_.setIdentity();

      Eigen::Isometry3d ref_frame;


      if(joystick_walking_flag_ ==true){
          if(current_step_num_<=2){
              if(foot_step_joy_(current_step_num_, 6) == 0)  //right foot support
              {
                ref_frame = rfoot_float_init_;
              }
              else if(foot_step_joy_(current_step_num_, 6) == 1)
              {
                ref_frame = lfoot_float_init_;
              }
          }
          else{
              if(foot_step_joy_(2, 6) == 0)  //right foot support
              {
                ref_frame = rfoot_float_init_;
              }
              else if(foot_step_joy_(2, 6) == 1)
              {
                ref_frame = lfoot_float_init_;
              }
          }
          if(joystick_walking_on_ == false){// begin steop sequence
              double seq = current_step_num_-joystick_stop_begin_num_;

              if(current_step_num_<=2){
                  if(foot_step_joy_(current_step_num_, 6) == 0)  //right foot support
                  {
                    ref_frame = rfoot_float_init_;
                  }
                  else if(foot_step_joy_(current_step_num_, 6) == 1)
                  {
                    ref_frame = lfoot_float_init_;
                  }
              }
              else{
                  if(foot_step_joy_(2+ seq, 6) == 0)  //right foot support
                  {
                    ref_frame = rfoot_float_init_;
                  }
                  else if(foot_step_joy_(2+ seq, 6) == 1)
                  {
                    ref_frame = lfoot_float_init_;
                  }
              }
          }
      }

      lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),lfoot_float_init_);
      rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),rfoot_float_init_);
      pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame);
      com_support_init_ = pelv_support_init_.linear()*com_float_init_ + pelv_support_init_.translation();

      pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear());
      rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
      lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());



    }
}
void WalkingController::JoygetRobotState(){



      com_sim_old_ = com_sim_current_;
      com_float_old_ = com_float_current_;
      com_support_old_ = com_support_current_;

      lfoot_float_current_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)0);
      rfoot_float_current_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)1);
      com_float_current_ = model_.getCurrentCom();
      com_float_current_(0) -= 0.03;

//      if(walking_tick_ ==0)
//        cout<<"com float  "<<com_float_current_<<endl;

      com_sim_current_ = model_.getSimulationCom();
      gyro_sim_current_ = model_.getSimulationGyro();
      accel_sim_current_ = model_.getSimulationAccel();
      lfoot_sim_global_current_ = model_.getSimulationLfoot();
      rfoot_sim_global_current_ = model_.getSimulationRfoot();
      base_sim_global_current_ = model_.getSimulationBase();

      r_ft_ = model_.getRightFootForce();
      l_ft_ = model_.getLeftFootForce();
      imu_acc_ = model_.getImuAccel();
      imu_ang_ = model_.getImuAngvel();
      imu_grav_rpy_ = model_.getImuGravityDirection();


//      DyrosMath::ComplimantaryFilter(imu_acc_,imu_ang_,angle_,hz_);
//      getZmpReal();


      pelv_float_current_.setIdentity();

      rfoot_sim_float_current_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(base_sim_global_current_), rfoot_sim_global_current_);
      lfoot_sim_float_current_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(base_sim_global_current_), lfoot_sim_global_current_);


      com_sim_current_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_sim_current_), com_sim_current_);

      current_leg_jacobian_l_ = model_.getLegJacobian((DyrosJetModel::EndEffector) 0);
      current_leg_jacobian_r_ = model_.getLegJacobian((DyrosJetModel::EndEffector) 1);
//      current_arm_jacobian_l_ = model_.getArmJacobian((DyrosJetModel::EndEffector) 2);
//      current_arm_jacobian_r_ = model_.getArmJacobian((DyrosJetModel::EndEffector) 3);
//      current_waist_jacobian_[0] = model_.getWaistJacobian(0);
//      current_waist_jacobian_[1] = model_.getWaistJacobian(1);



      //      for(unsigned int i=0;i<29;i++){
      //          link_local_com_position_[i] = model_.getLinkComPosition(i);
      //          link_inertia_[i] = model_.getLinkInertia(i);
      //          link_transform_[i] = model_.getLinkTransform(i);
      //          link_mass_[i] = model_.getLinkMass(i);
      //      }




//      link_mass_[0] =3.90994;
//      link_mass_[1] =1.54216;   link_mass_[2] =1.16907;   link_mass_[3] =3.28269;   link_mass_[4] =2.04524;  link_mass_[5] =1.1845;   link_mass_[6] =1.42541;
//      link_mass_[7] =1.54216;   link_mass_[8] =1.16907;   link_mass_[9] =3.28269;  link_mass_[10] =2.04524;   link_mass_[11] = 1.1845;   link_mass_[12] = 1.42541;
//      link_mass_[13] = 0.18235;  link_mass_[14] = 14.09938;


//      if(walking_tick_ == 0 ){
//          total_mass_ =0;
//          for(int i=0;i<29;i++)
//              total_mass_ += link_mass_[i];
//      }

      if(joystick_walking_flag_ == true){
          if(current_step_num_<=2){
              if(foot_step_joy_(current_step_num_, 6) == 0)  //right foot support
              {
                supportfoot_float_current_ = rfoot_float_current_;
                supportfoot_float_sim_current_ = rfoot_sim_float_current_;
              }
              else if(foot_step_joy_(current_step_num_, 6) == 1)
              {
                supportfoot_float_current_ = lfoot_float_current_;
                supportfoot_float_sim_current_ = lfoot_sim_float_current_;
              }
          }
          else{
              if(foot_step_joy_(2, 6) == 0)  //right foot support
              {
                supportfoot_float_current_ = rfoot_float_current_;
                supportfoot_float_sim_current_ = rfoot_sim_float_current_;
              }
              else if(foot_step_joy_(2, 6) == 1)
              {
                supportfoot_float_current_ = lfoot_float_current_;
                supportfoot_float_sim_current_ = lfoot_sim_float_current_;
              }
          }

          if(joystick_walking_on_ == false){
              double seq = current_step_num_-joystick_stop_begin_num_;
              if(current_step_num_<=2){
                  if(foot_step_joy_(current_step_num_, 6) == 0)  //right foot support
                  {

                    supportfoot_float_current_ = rfoot_float_current_;
                    supportfoot_float_sim_current_ = rfoot_sim_float_current_;
                  }
                  else if(foot_step_joy_(current_step_num_, 6) == 1)
                  {

                    supportfoot_float_current_ = lfoot_float_current_;
                    supportfoot_float_sim_current_ = lfoot_sim_float_current_;
                  }
              }
              else{
                  if(foot_step_joy_(2 + seq, 6) == 0)  //right foot support
                  {
                    supportfoot_float_current_ = rfoot_float_current_;
                    supportfoot_float_sim_current_ = rfoot_sim_float_current_;
                  }
                  else if(foot_step_joy_(2 + seq, 6) == 1)
                  {
                    supportfoot_float_current_ = lfoot_float_current_;
                    supportfoot_float_sim_current_ = lfoot_sim_float_current_;
                  }
              }
          }

      }

      pelv_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_);


      lfoot_support_current_ = DyrosMath::multiplyIsometry3d(pelv_support_current_,lfoot_float_current_);
      rfoot_support_current_ = DyrosMath::multiplyIsometry3d(pelv_support_current_,rfoot_float_current_);

      com_support_current_ = DyrosMath::multiplyIsometry3dVector3d(pelv_support_current_, com_float_current_);




      slowcalc_mutex_.lock();
      thread_q_ = current_q_;
      current_motor_q_leg_ = current_q_.segment<12>(0);
      current_link_q_leg_ = model_.getCurrentExtencoder();
      //current_link_q_leg_ = current_q_.segment<12>(0);

      slowcalc_mutex_.unlock();
}
void WalkingController::JoygetComTrajectory(){
    modifiedPreviewControl();

    //xd_ = x_1_;
    xs_ = xd_;
    ys_ = yd_;


    if (walking_tick_ == t_start_+t_total_-1 && current_step_num_ != total_step_num_-1)
    {
      Eigen::Vector3d com_pos_prev;
      Eigen::Vector3d com_pos;
      Eigen::Vector3d com_vel_prev;
      Eigen::Vector3d com_vel;
      Eigen::Vector3d com_acc_prev;
      Eigen::Vector3d com_acc;

      Eigen::Matrix3d temp_rot;
      Eigen::Vector3d temp_pos;

      if(joystick_walking_flag_ == true){
          if(current_step_num_<=2){
              temp_rot = DyrosMath::rotateWithZ(-foot_step_joy_support_frame_(current_step_num_,5));
              for(int i=0; i<3; i++)
                temp_pos(i) = foot_step_joy_support_frame_(current_step_num_,i);
          }
          else {
              if(joystick_walking_on_ == true){
                  temp_rot = DyrosMath::rotateWithZ(-foot_step_joy_support_frame_(2,5));
                  for(int i=0; i<3; i++)
                    temp_pos(i) = foot_step_joy_support_frame_(2,i);
              }
              else {
                  double tick = current_step_num_ - joystick_stop_begin_num_;
                  temp_rot = DyrosMath::rotateWithZ(-foot_step_joy_support_frame_(2+tick,5));
                  for(int i=0; i<3; i++)
                    temp_pos(i) = foot_step_joy_support_frame_(2+tick,i);

              }

          }
      }


      com_pos_prev(0) = xs_(0);
      com_pos_prev(1) = ys_(0);
      com_pos = temp_rot*(com_pos_prev - temp_pos);

      com_vel_prev(0) = xs_(1);
      com_vel_prev(1) = ys_(1);
      com_vel_prev(2) = 0.0;
      com_vel = temp_rot*com_vel_prev;

      com_acc_prev(0) = xs_(2);
      com_acc_prev(1) = ys_(2);
      com_acc_prev(2) = 0.0;
      com_acc = temp_rot*com_acc_prev;

      xs_(0) = com_pos(0);
      ys_(0) = com_pos(1);
      xs_(1) = com_vel(0);
      ys_(1) = com_vel(1);
      xs_(2) = com_acc(0);
      ys_(2) = com_acc(1);

    }


    double start_time;

    if(current_step_num_ == 0)
      start_time = 0;
    else
      start_time = t_start_;

    zmp_desired_(0) = ref_zmp_(walking_tick_-start_time,0);
    zmp_desired_(1) = ref_zmp_(walking_tick_-start_time,1);

    if(com_control_mode_ == true)
    {
      com_desired_(0) = xd_(0);
      com_desired_(1) = yd_(0);
      com_desired_(2) = DyrosMath::cubic(walking_tick_, t_start_, t_start_real_, pelv_support_init_.translation()(2), pelv_suppprt_start_.translation()(2), 0, 0);

      com_dot_desired_(0) = xd_(1);
      com_dot_desired_(1) = yd_(1);
      com_dot_desired_(2) = DyrosMath::cubicDot(walking_tick_, t_start_, t_start_real_, pelv_support_init_.translation()(2), pelv_suppprt_start_.translation()(2), 0, 0, hz_);

      double k= 100.0;
      p_ref_(0) = xd_(1)+k*(xd_(0)-com_support_current_(0));
      p_ref_(1) = yd_(1)+k*(yd_(0)-com_support_current_(1));
      p_ref_(2) = k*(com_desired_(2)-com_support_current_(2));
      l_ref_.setZero();
    }
    else
    {
      com_desired_(0) = xd_(0);
      com_desired_(1) = yd_(0);
      com_desired_(2) = DyrosMath::cubic(walking_tick_, t_start_, t_start_real_, pelv_support_init_.translation()(2), pelv_suppprt_start_.translation()(2), 0, 0);

      com_dot_desired_(0) = xd_(1);
      com_dot_desired_(1) = yd_(1);
      com_dot_desired_(2) = DyrosMath::cubicDot(walking_tick_, t_start_, t_start_real_, pelv_support_init_.translation()(2), pelv_suppprt_start_.translation()(2), 0, 0, hz_);

      double k= 100.0;
      p_ref_(0) = xd_(1)+k*(xd_(0)-com_support_current_(0));
      p_ref_(1) = yd_(1)+k*(yd_(0)-com_support_current_(1));
      p_ref_(2) = k*(com_desired_(2)-com_support_current_(2));
      l_ref_.setZero();
    }
}
void WalkingController::JoygetPelvTrajectory(){
     double z_rot;
    if(current_step_num_<=2){
        z_rot = foot_step_joy_support_frame_(current_step_num_,5);
     }
    else{
        z_rot = foot_step_joy_support_frame_(2,5);
    }


    //Trunk Position
    if(com_control_mode_ == true)
    {
      double kp = 1.2;
      if(estimator_flag_ == false || l_ft_(2)+r_ft_(2)<0.7*51*9.81)
      {
        pelv_trajectory_support_.translation()(0) = pelv_support_current_.translation()(0) + kp*(com_desired_(0) - com_support_current_(0));
        pelv_trajectory_support_.translation()(1) = pelv_support_current_.translation()(1) + kp*(com_desired_(1) - com_support_current_(1));
      }
      else
      {
        pelv_trajectory_support_.translation()(0) = pelv_support_current_.translation()(0) + kp*(com_desired_(0) - X_hat_post_2_(0));
        pelv_trajectory_support_.translation()(1) = pelv_support_current_.translation()(1) + kp*(com_desired_(1) - X_hat_post_2_(1));
      }

      // kp = Cubic(abs(_COM_desired(0)-_COM_real_support(0)),0.0,0.05,1.0,0.0,3.0,0.0);
      pelv_trajectory_support_.translation()(2) = com_desired_(2); //_T_Trunk_support.translation()(2) + kp*(_COM_desired(2) - _COM_real_support(2));
    }
    else
    {
      double kp = 3.0;
      double d = 0.5;

      if(walking_tick_ >= t_start_ && walking_tick_ < t_start_+0.3*hz_)
      {
        kp = 0+ 3.0*(walking_tick_-t_start_)/(0.3*hz_);
        d = 0+ 0.5*(walking_tick_-t_start_)/(0.3*hz_);
      }


      double offset_x = 0.0;
      if(current_step_num_<=2){
          if(foot_step_joy_(current_step_num_,6) == 1)
          {
              double temp_time = 0.1*hz_;
              if(walking_tick_ < t_start_real_)
                offset_x = DyrosMath::cubic(walking_tick_, t_start_+temp_time,t_start_real_-temp_time,0.0,0.02,0.0,0.0);
              else
                offset_x = DyrosMath::cubic(walking_tick_, t_start_+t_total_-t_rest_last_+temp_time,t_start_+t_total_-temp_time,0.02,0.0,0.0,0.0);
          }
      }
      else {
          if(foot_step_joy_(2,6) == 1)
          {
              double temp_time = 0.1*hz_;
              if(walking_tick_ < t_start_real_)
                offset_x = DyrosMath::cubic(walking_tick_, t_start_+temp_time,t_start_real_-temp_time,0.0,0.02,0.0,0.0);
              else
                offset_x = DyrosMath::cubic(walking_tick_, t_start_+t_total_-t_rest_last_+temp_time,t_start_+t_total_-temp_time,0.02,0.0,0.0,0.0);
          }
      }


      pelv_trajectory_support_.translation()(0) = pelv_support_current_.translation()(0) + 1.0*(com_desired_(0)-com_support_current_(0));
      pelv_trajectory_support_.translation()(1) = pelv_support_current_.translation()(1) + 1.0*(com_desired_(1)-com_support_current_(1));
      pelv_trajectory_support_.translation()(2) = com_desired_(2);

      double dt = 1.0/hz_;
      kp = 100.0;
      d = 2000.0;
    }
    //pelv_trajectory_support_.translation()(2) = 0.7652;//pelv_support_init_.translation()(2); //0.7652;//

    //Trunk orientation
    Eigen::Vector3d Trunk_trajectory_euler;

    if(walking_tick_ < t_start_real_+t_double1_)
    {
      for(int i=0; i<2; i++)
        Trunk_trajectory_euler(i) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_+t_double1_,pelv_support_euler_init_(i),0.0,0.0,0.0);;
      Trunk_trajectory_euler(2) = pelv_support_euler_init_(2);
    }
    else if(walking_tick_ >= t_start_real_+t_double1_ && walking_tick_ < t_start_+t_total_-t_double2_-t_rest_last_)
    {
      for(int i=0; i<2; i++)
        Trunk_trajectory_euler(i) = 0.0;

        Trunk_trajectory_euler(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_double2_-t_rest_last_, pelv_support_euler_init_(2),z_rot/2.0,0.0,0.0);
    }
    else
    {
      for(int i=0; i<2; i++)
        Trunk_trajectory_euler(i) = 0.0;

        Trunk_trajectory_euler(2) = z_rot/2.0;
    }

    pelv_trajectory_support_.linear() = DyrosMath::rotateWithZ(Trunk_trajectory_euler(2))*DyrosMath::rotateWithY(Trunk_trajectory_euler(1))*DyrosMath::rotateWithX(Trunk_trajectory_euler(0));

}

void WalkingController::setJoystickWalking(int walk_mode, bool hip_compensation, bool lqr, int ik_mode, bool heel_toe,
                                           bool is_right_foot_swing, double x, double y, double z, double height, double theta,
                                           double step_length_x, double step_length_y)
{
    target_x_ = x;
    target_y_ = y;
    target_z_ = z;
    com_height_ = height;
    target_theta_ = theta;
    step_length_x_ = step_length_x;
    step_length_y_ = step_length_y;
    ik_mode_ = ik_mode;
    walk_mode_ = walk_mode;
    hip_compensator_mode_ = hip_compensation; //uint32 compensator_mode[0] : HIP_COMPENSTOR    uint32 compensator_mode[1] : EXTERNAL_ENCODER
    lqr_compensator_mode_ = lqr;
    heel_toe_mode_ = heel_toe;
    is_right_foot_swing_ = is_right_foot_swing;

    calc_start_flag_ = lqr;
    joystick_walking_flag_ =true;

    if(heel_toe_mode_ == true)
        std::cout << "Crab Walking Mode !" << std::endl;
    else if(heel_toe_mode_ == false)
        std::cout << "General Walking Mode !" << std::endl;


    parameterSetting();
}
void WalkingController::getJoystick(double x, double y, double theta)
{

//    cout<<"get joystick : "<<endl;
// joystick_input_(0) is for general walking
  if(x >= y)
    joystick_input_(0) = x;
  else
    joystick_input_(0) = -y;

// joystick_input_(1) is for crab walking
  if(x >= y)
    joystick_input_(1) = -x;
  else
    joystick_input_(1) = y;
  //joystick_input_(2) = atan2(y,x);
  joystick_input_(2) = theta;

//    joystick_planning_ = true;
    if(joystick_walking_flag_ == true){
        if(walking_enable_ == false){
            if(joystick_input_(0) != 0){
                cout<<"joystick walking enable"<<endl;
                setEnable(true);
                joystick_walking_on_ = true;
                joystick_planning_ = true;

                if(walking_end_ == 1){
                    walking_end_ =0;
                    walking_state_send = true;
                    parameterSetting();
                }
            }
        }
        else{
            if(joystick_planning_ == false){
                if(joystick_input_(0) != 0){
                    joystick_planning_ = true;
                    parameterSetting();
                }
            }
        }
    }
}
}
