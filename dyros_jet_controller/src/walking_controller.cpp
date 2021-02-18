#include "dyros_jet_controller/dyros_jet_model.h"
#include "dyros_jet_controller/walking_controller.h"
#include "cvxgen_6_8_0/cvxgen/solver.h"

Vars vars;
Params params;
Workspace work;
Settings settings;

namespace dyros_jet_controller
{



void WalkingController::compute()
{
  if(walking_enable_ == true)
  {
      if(joystick_walking_flag_ == true){
          JoystickWalkingPlanning();
          JoyupdateInitialState();
          JoygetRobotState();
          JoyfloatToSupportFootstep();

      }
      else{

          updateInitialState();
          getRobotState();
          floatToSupportFootstep();

      }


      if(walking_tick_ == 0){
          unsigned int step_check = total_step_num_/2;
          cout<<"total step num : "<<total_step_num_<<", total step num /2 : "<<total_step_num_/2<<", total step num /2 /2 remain : "<<step_check%2<<endl;
      }
//      IntrisicMPC();
//      calculateStride();

    if(ready_for_thread_flag_ == false)
    {
      ready_for_thread_flag_ = true;
    }

    if(ready_for_compute_flag_ == true || lqr_compensator_mode_ == false)
    {

//      ////////Kalman state estimation/////////
//      kalmanFilter2();
//      kalmanFilter1();
//      kalmanFilter3();

//      /////QP state estimation///////////
//      getQpEstimationInputMatrix();

//      solve();
      /////////////////////////////////////////


      if(current_step_num_< total_step_num_)
      {
        if(joystick_walking_flag_ == true){            
            JoyZMPtrajectory();
            JoygetComTrajectory();
            JoygetPelvTrajectory();
            JoyFootTrajectory();
        }
        else{

            getZmpTrajectory();
//            MovingZMPtrajectory();

//            if(current_step_num_ == 0 || current_step_num_ >total_step_num_ -2){
//                getComTrajectory();
//            }
//            else {
//                if(heel_toe_mode_ ==false)
                    getComTrajectory();
//                if(heel_toe_mode_ == true)
                    qp3();
//            }
//            if(ik_mode_ !=2 || current_step_num_ == total_step_num_-1){

//                getComTrajectory();
//    //            getComTrajectory_MJ();

//            }
//            else if(ik_mode_ ==2 && current_step_num_ != total_step_num_-1){
//                qp3();
//            }

//            QP_legdynamics();


            if(com_height_ != 0.75)
              comHeight();
//            getCOMvelocity();

            getCOMestimation();
            getPelvTrajectory();

            if(ik_mode_==2)
                getFoottrajectory_heel_toe();
            else{
                getFootTrajectory();
            }
//            getFootSinTrajectory();
//            getOptimizedFootTrajectory();
//            getAdaptiveFootTrajectory();

      }

//        CalculateCenterOfMassSupportBody();

//       ZMP_2(l_ft_,r_ft_,lfoot_trajectory_support_.translation(),rfoot_trajectory_support_.translation(),ZMP_2_);



//        com_desired_(0) = com_data_(walking_tick_,7);
//        com_desired_(1) = com_data_(walking_tick_,10);

//        com_dot_desired_(0) = com_data_(walking_tick_,8);
//        com_dot_desired_(1) = com_data_(walking_tick_,11);

//       if(walking_tick_ >= 2473){
////            com_desired_(0) = com_data_(walking_tick_-1,1);
//          com_desired_(1) = com_data_(walking_tick_-1,4);

////            com_dot_desired_(0) = com_data_(walking_tick_-1,2);
//          com_dot_desired_(1) = com_data_(walking_tick_-1,5);
//        }
//        com_desired_(0) = com_data_(walking_tick_,1);
//        com_desired_(1) = com_data2_(walking_tick_,2);

//        com_dot_desired_(0) = com_data_(walking_tick_,7);
//        com_dot_desired_(1) = com_data2_(walking_tick_,8);



        ////////////////////////////////////////////////////////////
        // for swinging one leg
//        if(current_step_num_>=2){
//            pelv_trajectory_support_ = pelv_support_init_2_;
//            lfoot_trajectory_support_ = lfoot_support_init_2_;
//            rfoot_trajectory_support_ = rfoot_support_init_2_;

//            for(int i=0;i<2;i++)
//                pelv_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,1000,1200,pelv_support_init_2_.translation()(i),lfoot_support_init_.translation()(i),0.0,0.0);

//            int swingtime = 50;
//            rfoot_trajectory_support_.translation()(0) = DyrosMath::cubic(walking_tick_,1300,1300+swingtime,rfoot_support_init_2_.translation()(0) ,rfoot_support_init_2_.translation()(0) + step_length_x_*2, 0.0, 0.0);


//            if(walking_tick_>=1300)
//                rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,1300,1300+swingtime*0.5,rfoot_support_init_2_.translation()(2),0.05,0.0,0.0);
//            if(walking_tick_>=1300+swingtime*0.5)
//                rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,1300+swingtime*0.5,1300+swingtime,0.05,rfoot_support_init_2_.translation()(2),0.0,0.0);
//        }
        ////////////////////////////////////////////////////////////



        supportToFloatPattern();
//        getToeHeelTrajectory();
//        calculateFootEdgeJaco();

//        supportToWaistPattern();



//        QP_legdynamics();

        ////////////////////////////////////////////
        ///      Calculating Capture Point       ///
        ////////////////////////////////////////////

//        Eigen::Vector2d CP_current, CP_desired;
//        CP_current.setZero(); CP_desired.setZero();

//        double w=sqrt(GRAVITY/zc_);

//        for(int i=0;i<2;i++){
//            CP_current(i) = com_support_current_(i) + com_support_vel_(i)/w;
//            CP_desired(i) = com_desired_(i) + com_dot_desired_(i)/w;
//        }

//        // calculating Orbital Energy;
//        Eigen::Vector2d orbital_E_d, orbital_E_c;

//        for(int i=0;i<2;i++){
//            orbital_E_c(i) = com_support_vel_(i)*com_support_vel_(i)/2.0 - GRAVITY*com_support_current_(i)*com_support_current_(i)/(2*zc_);
//            orbital_E_d(i) = com_dot_desired_(i)*com_dot_desired_(i)/2.0 - GRAVITY*com_desired_(i)*com_desired_(i)/(2*zc_);
//        }

//        getDesiredVelocity(lp_,rp_,ltoe_p_,rtoe_p_,lheel_p_,rheel_p_,lp_clik_,rp_clik_,ltoe_clik_, rtoe_clik_,lheel_clik_,rheel_clik_);

//        file[10]<<walking_tick_
//               <<"\t"<<lp_(0)<<"\t"<<lp_(1)<<"\t"<<lp_(2)<<"\t"<<lp_(3)<<"\t"<<lp_(4)<<"\t"<<lp_(5)
//              <<"\t"<<rp_(0)<<"\t"<<rp_(1)<<"\t"<<rp_(2)<<"\t"<<rp_(3)<<"\t"<<rp_(4)<<"\t"<<rp_(5)
//             <<"\t"<<ltoe_p_(0)<<"\t"<<ltoe_p_(1)<<"\t"<<ltoe_p_(2)<<"\t"<<ltoe_p_(3)<<"\t"<<ltoe_p_(4)<<"\t"<<ltoe_p_(5)
//            <<"\t"<<rtoe_p_(0)<<"\t"<<rtoe_p_(1)<<"\t"<<rtoe_p_(2)<<"\t"<<rtoe_p_(3)<<"\t"<<rtoe_p_(4)<<"\t"<<rtoe_p_(5)
//           <<"\t"<<lheel_p_(0)<<"\t"<<lheel_p_(1)<<"\t"<<lheel_p_(2)<<"\t"<<lheel_p_(3)<<"\t"<<lheel_p_(4)<<"\t"<<lheel_p_(5)
//          <<"\t"<<rheel_p_(0)<<"\t"<<rheel_p_(1)<<"\t"<<rheel_p_(2)<<"\t"<<rheel_p_(3)<<"\t"<<rheel_p_(4)<<"\t"<<rheel_p_(5)
//         <<"\t"<<lheel_clik_(0)<<"\t"<<lheel_clik_(1)<<"\t"<<lheel_clik_(2)<<"\t"<<lheel_clik_(3)<<"\t"<<lheel_clik_(4)<<"\t"<<lheel_clik_(5)
//           <<"\t"<<rheel_clik_(0)<<"\t"<<rheel_clik_(1)<<"\t"<<rheel_clik_(2)<<"\t"<<rheel_clik_(3)<<"\t"<<rheel_clik_(4)<<"\t"<<rheel_clik_(5)
//             <<endl;

        /////////////////////compute/////////////////////////
        if (ik_mode_ == 0)
        {

          computeIkControl(pelv_trajectory_float_, lfoot_trajectory_float_, rfoot_trajectory_float_, desired_leg_q_);
          for(int i=0; i<12; i++)
          {
            desired_q_(i) = desired_leg_q_(i);
          }

//          Eigen::VectorXd desired_waist_leg_qdot;
//          desired_waist_leg_qdot.resize(13);

//          computeWaistJacobianCtrl(lfoot_trajectory_waist_,rfoot_trajectory_waist_,lfoot_trajectory_euler_waist_,rfoot_trajectory_euler_waist_,desired_waist_leg_qdot);

//          if(walking_tick_ ==0)
//              desired_q_.segment(0,12) = q_init_.segment(0,12);

//          else{
//              desired_q_.segment(0,13) = desired_waist_leg_qdot.segment(0,13)/hz_ + desired_q_not_compensated_.segment(0,13);
//          }
        }
        else if (ik_mode_ == 1)
        {
          computeJacobianControl(lfoot_trajectory_float_, rfoot_trajectory_float_, lfoot_trajectory_euler_float_, rfoot_trajectory_euler_float_, desired_leg_q_dot_);

          for(int i=0; i<12; i++)
          {
            if(walking_tick_ == 0)
            {
              desired_q_not_compensated_(i) = q_init_(i);
            }
//            desired_q_(i) = desired_leg_q_dot_(i)/hz_+current_q_(i);
            desired_q_(i) = desired_leg_q_dot_(i)/hz_+desired_q_not_compensated_(i);
          }
        }
        else if (ik_mode_ == 2){
            //        qpIK();
            qpIK_test();
        }


//        if(walking_tick_ > 301)
//            desired_q_ = desired_q_not_compensated_;

//        Centroidal_Dynamics();
        //////////////////////////////////////////////////////

        //ResolvedMomentumCtrl();


        //////// zmp calculated and measured //
        ///  xd_ yd_ original com desired,
        ///  x_1_ y_1_ mpc com desired


        Eigen::Matrix<double, 1, 3>c;
        c(0) = 1.0; c(1) = 0.0; c(2) = -zc_/GRAVITY;

        Eigen::Vector2d zmp_calculated;
        zmp_calculated.setZero();

        zmp_calculated(0) = c*xd_;
        zmp_calculated(1) = c*yd_;

//        zmp_calculated(0) = c*x_d1_;
//        zmp_calculated(1) = c*y_d1_;

//        zmp_calculated(0) = com_data_(walking_tick_,1) - c(2) * com_data_(walking_tick_,3);
//        //zmp_calculated(1) = com_data_(walking_tick_,4) - c(2) * com_data_(walking_tick_,6);


//        file[30]<<walking_tick_<<"\t"<<zmp_desired_(0)<<"\t"<<zmp_desired_(1)<<"\t"<<zmp_calculated(0)<<"\t"<<zmp_calculated(1)<<"\t"<<zmp_measured_(0)<<"\t"<<zmp_measured_(1)<<endl;
//        file[31]<<walking_tick_<<"\t"<<xd_(0)<<"\t"<<xd_(1)<<"\t"<<xd_(2)<<"\t"<<yd_(0)<<"\t"<<yd_(1)<<"\t"<<yd_(2)<<"\t"<<x_d1_(0)<<"\t"<<x_d1_(1)<<"\t"<<x_d1_(2)<<"\t"<<y_d1_(0)<<"\t"<<y_d1_(1)<<"\t"<<y_d1_(2)<<endl;


//        settingParameter();

//        solve();


//        for(int i=0;i<6;i++)
//            desired_q_(i) = vars.x[i]/hz_ + desired_q_not_compensated_(i);

        //desired_q_(WA_BEGIN) = vars.x[6]/hz_ + desired_q_not_compensated_(WA_BEGIN);

        //desired_q_(WA_BEGIN) = desired_waist_leg_qdot(12)/hz_ + desired_q_not_compensated_(WA_BEGIN);
//        desired_q_(WA_BEGIN) = target_speed_(6)/hz_ + desired_q_not_compensated_(WA_BEGIN);


       // desired_q_(WA_BEGIN) = DyrosMath::cubic(walking_tick_,10,100,0.0,45*DEGREE,0.0,0.0);

        for(int i=0;i<7;i++){
           desired_q_(RA_BEGIN+i) = target_speed_(7+i)/hz_ + desired_q_not_compensated_(RA_BEGIN+i);
           desired_q_(LA_BEGIN+i) = target_speed_(14+i)/hz_ + desired_q_not_compensated_(LA_BEGIN+i);
         //desired_q_(RA_BEGIN+i) = target_speed_(1+i) + desired_q_not_compensated_(RA_BEGIN+i);
         //desired_q_(LA_BEGIN+i) = target_speed_(8+i) + desired_q_not_compensated_(LA_BEGIN+i);
        }


//        Eigen::Vector12d qp_q;
//        for(int i=0;i<6;i++){
//            qp_q(i) = vars.x[i]/hz_ + desired_q_not_compensated_[LF_BEGIN + i];
//            qp_q(i+6) = vars.x[i+6]/hz_ + desired_q_not_compensated_[RF_BEGIN +i];
//        }

//        desired_q_ = desired_q_not_compensated_;

      // desired_q_.segment(LF_BEGIN,12) = qp_q;
       desired_q_not_compensated_ = desired_q_;

     //  desired_q_ = current_q_;




       compensator();

       //calculating engergy consumption for leg joints.///
       Eigen::Vector12d E_con;
       double E_con_total;
       E_con_total = 0;

       for(int i=0;i<12;i++){
           E_con(i) = abs(current_torque_(i))*abs(current_q_dot_(i))/hz_;
           E_con_total += E_con(i);
       }

//       file[7]<<walking_tick_<<"\t"<<qp_q[0]*RAD2DEG<<"\t"<<qp_q[1]*RAD2DEG<<"\t"<<qp_q[2]*RAD2DEG<<"\t"<<qp_q[3]*RAD2DEG<<"\t"<<qp_q[4]*RAD2DEG<<"\t"<<qp_q[5]*RAD2DEG
//                <<"\t"<<qp_q[6]*RAD2DEG<<"\t"<<qp_q[7]*RAD2DEG<<"\t"<<qp_q[8]*RAD2DEG<<"\t"<<qp_q[9]*RAD2DEG<<"\t"<<qp_q[10]*RAD2DEG<<"\t"<<qp_q[11]*RAD2DEG<<endl;

        //std::cout<<"com_support_current_:"<<com_support_current_<<endl;
        //std::cout<<"zmp_measured_:"<<zmp_measured_<<endl;


        //////////////data saving in text files///////////
       file[0]<<walking_tick_<<"\t"<<current_step_num_<<"\t"<<zmp_desired_(0)<<"\t"<<zmp_desired_(1)<<endl;//<<"\t"<<foot_step_(current_step_num_, 0)<<"\t"<<foot_step_(current_step_num_, 1)<<"\t"<<
                // foot_step_support_frame_(current_step_num_, 0)<<"\t"<<foot_step_support_frame_(current_step_num_, 1)<<"\t"<<foot_step_support_frame_(current_step_num_, 2)<<endl;
//        file[1]<<walking_tick_<<"\t"<<current_step_num_<<"\t"<<com_desired_(0)<<"\t"<<com_desired_(1)<<"\t"<<com_desired_(2)<<"\t"<<com_dot_desired_(0)<<"\t"<<com_dot_desired_(1)<<"\t"<<
//                 com_dot_desired_(2)<<"\t"<<com_support_init_(0)<<"\t"<<com_support_init_(0)<<"\t"<<com_support_init_(0)<<endl;
//        file[2]<<walking_tick_<<"\t"<<current_step_num_<<"\t"<<desired_leg_q_(0)<<"\t"<<desired_leg_q_(1)<<"\t"<<desired_leg_q_(2)<<"\t"<<desired_leg_q_(3)<<"\t"<<desired_leg_q_(4)<<"\t"<<
//                 desired_leg_q_(5)<<"\t"<<desired_leg_q_(6)<<"\t"<<desired_leg_q_(7)<<"\t"<<desired_leg_q_(8)<<"\t"<<desired_leg_q_(9)<<"\t"<<desired_leg_q_(10)<<"\t"<<desired_leg_q_(11)<<endl;
//        //     file[3]<<walking_tick_<<"\t"<<current_step_num_<<"\t"<<current_q_(0)<<"\t"<<current_q_(1)<<"\t"<<current_q_(2)<<"\t"<<current_q_(3)<<"\t"<<current_q_(4)<<"\t"<<current_q_(5)<<"\t"<<
//        //             current_q_(6)<<"\t"<<current_q_(7)<<"\t"<<current_q_(8)<<"\t"<<current_q_(9)<<"\t"<<current_q_(10)<<"\t"<<current_q_(11)<<endl;
//        file[4]<<walking_tick_<<"\t"<<current_step_num_<<"\t"<<rfoot_trajectory_support_.translation()(0)<<"\t"<<rfoot_trajectory_support_.translation()(1)<<"\t"<<
//                 rfoot_trajectory_support_.translation()(2)<<"\t"<<lfoot_trajectory_support_.translation()(0)<<"\t"<<lfoot_trajectory_support_.translation()(1)<<"\t"<<lfoot_trajectory_support_.translation()(2)<<"\t"<<
//                 rfoot_support_init_.translation()(0)<<"\t"<<rfoot_support_init_.translation()(1)<<"\t"<<rfoot_support_init_.translation()(2)<<endl;
//        file[5]<<walking_tick_<<"\t"<<current_step_num_<<"\t"<<pelv_trajectory_support_.translation()(0)<<"\t"<<pelv_trajectory_support_.translation()(1)<<"\t"<<pelv_trajectory_support_.translation()(2)
//              <<endl;
//        file[6]<<walking_tick_<<"\t"<<current_step_num_<<"\t"<<com_support_current_(0)<<"\t"<<com_support_current_(1)<<"\t"<<com_support_current_(2)
//              <<"\t"<<pelv_support_current_.translation()(0)<<"\t"<<pelv_support_current_.translation()(1)<<"\t"<<pelv_support_current_.translation()(2)<<"\t"<<com_support_dot_current_(0)<<"\t"<<com_support_dot_current_(1)<<"\t"<<com_support_dot_current_(2)<<
//                "\t"<<com_sim_current_(0)<<"\t"<<com_sim_current_(1)<<"\t"<<com_sim_current_(2)<<"\t"<<com_sim_dot_current_(0)<<"\t"<<com_sim_dot_current_(1)<<"\t"<<com_sim_dot_current_(2)<<endl;
//        file[7]<<walking_tick_<<"\t"<<current_step_num_<<"\t"<<rfoot_support_current_.translation()(0)<<"\t"<<rfoot_support_current_.translation()(1)<<"\t"<<rfoot_support_current_.translation()(2)
//              <<"\t"<<lfoot_support_current_.translation()(0)<<"\t"<<lfoot_support_current_.translation()(1)<<"\t"<<lfoot_support_current_.translation()(2)<<endl;
//        file[8]<<walking_tick_<<"\t"<<current_step_num_<<"\t"<<vars.x[0]<<"\t"<<vars.x[1]<<"\t"<<vars.x[2]<<"\t"<<vars.x[3]<<"\t"<<vars.x[4]<<"\t"<<vars.x[5]<<"\t"<<zmp_measured_(0)<<"\t"<<zmp_measured_(1)<<"\t"<<zmp_r_(0)<<"\t"<<zmp_r_(1)<<"\t"<<zmp_l_(0)<<"\t"<<zmp_l_(1)<<endl;
//        file[13]<<walking_tick_<<"\t"<<current_step_num_
//               <<"\t"<<l_ft_(0)<<"\t"<<l_ft_(1)<<"\t"<<l_ft_(2)<<"\t"<<l_ft_(3)<<"\t"<<l_ft_(4)<<"\t"<<l_ft_(5)
//                 <<"\t"<<r_ft_(0)<<"\t"<<r_ft_(1)<<"\t"<<r_ft_(2)<<"\t"<<r_ft_(3)<<"\t"<<r_ft_(4)<<"\t"<<r_ft_(5)
//                   <<"\t"<<l_ft_filtered_(0)<<"\t"<<l_ft_filtered_(1)<<"\t"<<l_ft_filtered_(2)<<"\t"<<l_ft_filtered_(3)<<"\t"<<l_ft_filtered_(4)<<"\t"<<l_ft_filtered_(5)
//               <<"\t"<<r_ft_filtered_(0)<<"\t"<<r_ft_filtered_(1)<<"\t"<<r_ft_filtered_(2)<<"\t"<<r_ft_filtered_(3)<<"\t"<<r_ft_filtered_(4)<<"\t"<<r_ft_filtered_(5)
//              <<"\t"<<pushing_force_<<endl;
//        file[10]<<walking_tick_<<"\t"<<current_step_num_<<"\t"<<current_link_q_leg_(0)<<"\t"<<current_link_q_leg_(1)<<"\t"<<current_link_q_leg_(2)<<"\t"<<current_link_q_leg_(3)<<"\t"<<current_link_q_leg_(4)<<"\t"<<current_link_q_leg_(5)<<"\t"<<
//                  current_link_q_leg_(6)<<"\t"<<current_link_q_leg_(7)<<"\t"<<current_link_q_leg_(8)<<"\t"<<current_link_q_leg_(9)<<"\t"<<current_link_q_leg_(10)<<"\t"<<current_link_q_leg_(11)<<endl;
//        //DEBUG
//        file[11]<<walking_tick_<<"\t"<<current_step_num_<<"\t"<<ext_encoder_debug_(0)<<"\t"<<ext_encoder_debug_(1)<<"\t"<<ext_encoder_debug_(2)<<"\t"<<ext_encoder_debug_(3)<<"\t"<<ext_encoder_debug_(4)<<"\t"<<ext_encoder_debug_(5)<<"\t"<<
//                  ext_encoder_debug_(6)<<"\t"<<ext_encoder_debug_(7)<<"\t"<<ext_encoder_debug_(8)<<"\t"<<ext_encoder_debug_(9)<<"\t"<<ext_encoder_debug_(10)<<"\t"<<ext_encoder_debug_(11)<<endl;


////        file[11]<<walking_tick_<<"\t"<<X_hat_post_2_(0)<<"\t"<<X_hat_post_2_(1)<<"\t"<<X_hat_post_2_(2)<<"\t"<<X_hat_post_2_(3)<<"\t"<<X_hat_post_2_(4)<<"\t"<<X_hat_post_2_(5)<<"\t"<<X_hat_post_2_(6)<<"\t"<<X_hat_post_2_(7)<<endl;
//        file[12]<<walking_tick_<<"\t"<<X_hat_post_1_(0)<<"\t"<<X_hat_post_1_(1)<<"\t"<<X_hat_post_1_(2)<<"\t"<<X_hat_post_1_(3)<<"\t"<<X_hat_post_1_(4)<<"\t"<<X_hat_post_1_(5)<<endl;
//        file[13]<<walking_tick_<<"\t"<<X_hat_post_3_(0)<<"\t"<<X_hat_post_3_(1)<<"\t"<<X_hat_post_3_(2)<<"\t"<<X_hat_post_3_(3)<<"\t"<<X_hat_post_3_(4)<<"\t"<<X_hat_post_3_(5)<<"\t"<<X_hat_post_3_(6)<<"\t"<<X_hat_post_3_(7)<<"\t"<<X_hat_post_3_(8)<<"\t"<<X_hat_post_3_(9)<<endl;

       if(joystick_walking_flag_ == false){
//        ///// data saving in text files new version /////
//        file[0]<<walking_tick_<<"\t"<<zmp_desired_(0)<<"\t"<<zmp_desired_(1)<<"\t"<<zmp_measured_(0)<<"\t"<<zmp_measured_(1)<<endl;
//        file[1]<<walking_tick_<<"\t"<<current_step_num_<<"\t"<<com_desired_(0)<<"\t"<<com_desired_(1)<<"\t"<<com_desired_(2)<<"\t"<<com_support_current_(0)<<"\t"<<com_support_current_(1)<<"\t"<<com_support_current_(2)
//              <<"\t"<<com_float_current_(0)<<"\t"<<com_float_current_(1)<<"\t"<<com_float_current_(2)
//              <<"\t"<<com_dot_desired_(0)<<"\t"<<com_dot_desired_(1)<<"\t"<<com_dot_float_current_(0)<<"\t"<<com_dot_float_current_(1)
//             <<"\t"<<CP_current(0)<<"\t"<<CP_current(1)<<"\t"<<CP_desired(0)<<"\t"<<CP_desired(1)
//            <<"\t"<<orbital_E_c(0)<<"\t"<<orbital_E_c(1)<<"\t"<<orbital_E_d(0)<<"\t"<<orbital_E_d(1)
//           <<"\t"<<com_support_body_(0)<<"\t"<<com_support_body_(1)<<"\t"<<com_support_body_(2)<<endl;
//        file[2]<<walking_tick_
//              <<"\t"<<desired_q_(0)*RAD2DEG<<"\t"<<desired_q_(1)*RAD2DEG<<"\t"<<desired_q_(2)*RAD2DEG<<"\t"<<desired_q_(3)*RAD2DEG<<"\t"<<desired_q_(4)*RAD2DEG<<"\t"<<desired_q_(5)*RAD2DEG
//                 <<"\t"<<desired_q_(6)*RAD2DEG<<"\t"<<desired_q_(7)*RAD2DEG<<"\t"<<desired_q_(8)*RAD2DEG<<"\t"<<desired_q_(9)*RAD2DEG<<"\t"<<desired_q_(10)*RAD2DEG<<"\t"<<desired_q_(11)*RAD2DEG
//                <<"\t"<<current_q_(0)*RAD2DEG<<"\t"<<current_q_(1)*RAD2DEG<<"\t"<<current_q_(2)*RAD2DEG<<"\t"<<current_q_(3)*RAD2DEG<<"\t"<<current_q_(4)*RAD2DEG<<"\t"<<current_q_(5)*RAD2DEG
//                <<"\t"<<current_q_(6)*RAD2DEG<<"\t"<<current_q_(7)*RAD2DEG<<"\t"<<current_q_(8)*RAD2DEG<<"\t"<<current_q_(9)*RAD2DEG<<"\t"<<current_q_(10)*RAD2DEG<<"\t"<<current_q_(11)*RAD2DEG<<endl;
//        file[3]<<walking_tick_<<"\t"<<desired_q_(12)*RAD2DEG<<"\t"<<desired_q_(13)*RAD2DEG
//                 <<"\t"<<desired_q_(14)*RAD2DEG<<"\t"<<desired_q_(15)*RAD2DEG<<"\t"<<desired_q_(16)*RAD2DEG<<"\t"<<desired_q_(17)*RAD2DEG<<"\t"<<desired_q_(18)*RAD2DEG<<"\t"<<desired_q_(19)*RAD2DEG<<"\t"<<desired_q_(20)*RAD2DEG
//                   <<"\t"<<desired_q_(21)*RAD2DEG<<"\t"<<desired_q_(22)*RAD2DEG<<"\t"<<desired_q_(23)*RAD2DEG<<"\t"<<desired_q_(24)*RAD2DEG<<"\t"<<desired_q_(25)*RAD2DEG<<"\t"<<desired_q_(26)*RAD2DEG<<"\t"<<desired_q_(27)*RAD2DEG
//                  <<"\t"<<current_q_(12)*RAD2DEG<<"\t"<<current_q_(13)*RAD2DEG
//                  <<"\t"<<current_q_(14)*RAD2DEG<<"\t"<<current_q_(15)*RAD2DEG<<"\t"<<current_q_(16)*RAD2DEG<<"\t"<<current_q_(17)*RAD2DEG<<"\t"<<current_q_(18)*RAD2DEG<<"\t"<<current_q_(19)*RAD2DEG<<"\t"<<current_q_(20)*RAD2DEG
//                  <<"\t"<<current_q_(21)*RAD2DEG<<"\t"<<current_q_(22)*RAD2DEG<<"\t"<<current_q_(23)*RAD2DEG<<"\t"<<current_q_(24)*RAD2DEG<<"\t"<<current_q_(25)*RAD2DEG<<"\t"<<current_q_(26)*RAD2DEG<<"\t"<<current_q_(27)*RAD2DEG<<endl;

//        file[4]<<walking_tick_
//                 <<"\t"<<lfoot_trajectory_support_.translation()(0)<<"\t"<<lfoot_trajectory_support_.translation()(1)<<"\t"<<lfoot_trajectory_support_.translation()(2)
//                   <<"\t"<<rfoot_trajectory_support_.translation()(0)<<"\t"<<rfoot_trajectory_support_.translation()(1)<<"\t"<<rfoot_trajectory_support_.translation()(2)
//               <<"\t"<<lfoot_support_current_.translation()(0)<<"\t"<<lfoot_support_current_.translation()(1)<<"\t"<<lfoot_support_current_.translation()(2)
//                 <<"\t"<<rfoot_support_current_.translation()(0)<<"\t"<<rfoot_support_current_.translation()(1)<<"\t"<<rfoot_support_current_.translation()(2)
//             <<"\t"<<lfoot_trajectory_dot_support_(0)<<"\t"<<lfoot_trajectory_dot_support_(1)<<"\t"<<lfoot_trajectory_dot_support_(2)
//            <<"\t"<<rfoot_trajectory_dot_support_(0)<<"\t"<<rfoot_trajectory_dot_support_(1)<<"\t"<<rfoot_trajectory_dot_support_(2)
//           <<"\t"<<lfoot_trajectory_euler_support_(0)*RAD2DEG<<"\t"<<lfoot_trajectory_euler_support_(1)*RAD2DEG<<"\t"<<lfoot_trajectory_euler_support_(2)*RAD2DEG
//             <<"\t"<<rfoot_trajectory_euler_support_(0)*RAD2DEG<<"\t"<<rfoot_trajectory_euler_support_(1)*RAD2DEG<<"\t"<<rfoot_trajectory_euler_support_(2)*RAD2DEG
//            <<"\t"<<lfoot_support_current_euler_(0)*RAD2DEG<<"\t"<<lfoot_support_current_euler_(1)*RAD2DEG<<"\t"<<lfoot_support_current_euler_(2)*RAD2DEG
//              <<"\t"<<rfoot_support_current_euler_(0)*RAD2DEG<<"\t"<<rfoot_support_current_euler_(1)*RAD2DEG<<"\t"<<rfoot_support_current_euler_(2)*RAD2DEG
//              <<endl;
//        file[5]<<walking_tick_<<"\t"<<pelv_trajectory_support_.translation()(0)<<"\t"<<pelv_trajectory_support_.translation()(1)<<"\t"<<pelv_trajectory_support_.translation()(2)
//              <<"\t"<<pelv_support_current_.translation()(0)<<"\t"<<pelv_support_current_.translation()(1)<<"\t"<<pelv_support_current_.translation()(2)<<endl;
////        file[6]<<walking_tick_<<"\t"<<target_speed_(6)*RAD2DEG
////              <<"\t"<<target_speed_(7)*RAD2DEG<<"\t"<<target_speed_(8)*RAD2DEG<<"\t"<<target_speed_(9)*RAD2DEG<<"\t"<<target_speed_(10)*RAD2DEG<<"\t"<<target_speed_(11)*RAD2DEG<<"\t"<<target_speed_(12)*RAD2DEG<<"\t"<<target_speed_(13)*RAD2DEG
////             <<"\t"<<target_speed_(14)*RAD2DEG<<"\t"<<target_speed_(15)*RAD2DEG<<"\t"<<target_speed_(16)*RAD2DEG<<"\t"<<target_speed_(17)*RAD2DEG<<"\t"<<target_speed_(18)*RAD2DEG<<"\t"<<target_speed_(19)*RAD2DEG<<"\t"<<target_speed_(20)*RAD2DEG<<endl;
//        file[6]<<walking_tick_<<"\t"<<ltoe_trajectory_float_.translation()(0)<<"\t"<<ltoe_trajectory_float_.translation()(1)<<"\t"<<ltoe_trajectory_float_.translation()(2)
//                 <<"\t"<<rtoe_trajectory_float_.translation()(0)<<"\t"<<rtoe_trajectory_float_.translation()(1)<<"\t"<<rtoe_trajectory_float_.translation()(2)
//                <<"\t"<<ltoe_float_current_.translation()(0)<<"\t"<<ltoe_float_current_.translation()(1)<<"\t"<<ltoe_float_current_.translation()(2)
//                  <<"\t"<<rtoe_float_current_.translation()(0)<<"\t"<<rtoe_float_current_.translation()(1)<<"\t"<<rtoe_float_current_.translation()(2)
//                <<"\t"<<lheel_trajectory_float_.translation()(0)<<"\t"<<lheel_trajectory_float_.translation()(1)<<"\t"<<lheel_trajectory_float_.translation()(2)
//                <<"\t"<<rheel_trajectory_float_.translation()(0)<<"\t"<<rheel_trajectory_float_.translation()(1)<<"\t"<<rheel_trajectory_float_.translation()(2)
//                <<"\t"<<lheel_float_current_.translation()(0)<<"\t"<<lheel_float_current_.translation()(1)<<"\t"<<lheel_float_current_.translation()(2)
//                  <<"\t"<<rheel_float_current_.translation()(0)<<"\t"<<rheel_float_current_.translation()(1)<<"\t"<<rheel_float_current_.translation()(2)
//               <<"\t"<<ltoe_trajectory_euler_(0)<<"\t"<<ltoe_trajectory_euler_(1)<<"\t"<<ltoe_trajectory_euler_(2)
//                 <<"\t"<<rtoe_trajectory_euler_(0)<<"\t"<<rtoe_trajectory_euler_(1)<<"\t"<<rtoe_trajectory_euler_(2)
//                <<"\t"<<lheel_trajectory_euler_(0)<<"\t"<<lheel_trajectory_euler_(1)<<"\t"<<lheel_trajectory_euler_(2)
//                  <<"\t"<<rheel_trajectory_euler_(0)<<"\t"<<rheel_trajectory_euler_(1)<<"\t"<<rheel_trajectory_euler_(2)
//               <<endl;

//        file[8]<<walking_tick_<<"\t"<<desired_leg_q_dot_(0)<<"\t"<<desired_leg_q_dot_(1)<<"\t"<<desired_leg_q_dot_(2)<<"\t"<<desired_leg_q_dot_(3)<<"\t"<<desired_leg_q_dot_(4)<<"\t"<<desired_leg_q_dot_(5)
//              <<"\t"<<desired_leg_q_dot_(6)<<"\t"<<desired_leg_q_dot_(7)<<"\t"<<desired_leg_q_dot_(8)<<"\t"<<desired_leg_q_dot_(9)<<"\t"<<desired_leg_q_dot_(10)<<"\t"<<desired_leg_q_dot_(11)<<endl;
//        file[9]<<walking_tick_<<"\t"<<lfoot_trajectory_float_.translation()(0)<<"\t"<<lfoot_trajectory_float_.translation()(1)<<"\t"<<lfoot_trajectory_float_.translation()(2)
//              <<"\t"<<rfoot_trajectory_float_.translation()(0)<<"\t"<<rfoot_trajectory_float_.translation()(1)<<"\t"<<rfoot_trajectory_float_.translation()(2)
//             <<"\t"<<lfoot_float_current_.translation()(0)<<"\t"<<lfoot_float_current_.translation()(1)<<"\t"<<lfoot_float_current_.translation()(2)
//               <<"\t"<<rfoot_float_current_.translation()(0)<<"\t"<<rfoot_float_current_.translation()(1)<<"\t"<<rfoot_float_current_.translation()(2)
//              <<"\t"<<lfoot_trajectory_euler_float_(0)*RAD2DEG<<"\t"<<lfoot_trajectory_euler_float_(1)*RAD2DEG<<"\t"<<lfoot_trajectory_euler_float_(2)*RAD2DEG
//             <<"\t"<<rfoot_trajectory_euler_float_(0)*RAD2DEG<<"\t"<<rfoot_trajectory_euler_float_(1)*RAD2DEG<<"\t"<<rfoot_trajectory_euler_float_(2)*RAD2DEG
//             <<"\t"<<lfoot_float_current_euler_(0)*RAD2DEG<<"\t"<<lfoot_float_current_euler_(1)*RAD2DEG<<"\t"<<lfoot_float_current_euler_(2)*RAD2DEG
//                <<"\t"<<rfoot_float_current_euler_(0)*RAD2DEG<<"\t"<<rfoot_float_current_euler_(1)*RAD2DEG<<"\t"<<rfoot_float_current_euler_(2)*RAD2DEG
//              <<endl;
//        file[21]<<walking_tick_<<"\t"<<current_q_dot_(0)<<"\t"<<current_q_dot_(1)<<"\t"<<current_q_dot_(2)<<"\t"<<current_q_dot_(3)<<"\t"<<current_q_dot_(4)<<"\t"<<current_q_dot_(5)
//               <<"\t"<<current_q_dot_(6)<<"\t"<<current_q_dot_(7)<<"\t"<<current_q_dot_(8)<<"\t"<<current_q_dot_(9)<<"\t"<<current_q_dot_(10)<<"\t"<<current_q_dot_(11)
//              <<"\t"<<current_q_dot_(LA_BEGIN)<<"\t"<<current_q_dot_(LA_BEGIN+1)<<"\t"<<current_q_dot_(LA_BEGIN+2)<<"\t"<<current_q_dot_(LA_BEGIN+3)<<"\t"<<current_q_dot_(LA_BEGIN+4)<<"\t"<<current_q_dot_(LA_BEGIN+5)<<"\t"<<current_q_dot_(LA_BEGIN+6)
//                <<"\t"<<current_q_dot_(RA_BEGIN)<<"\t"<<current_q_dot_(RA_BEGIN+1)<<"\t"<<current_q_dot_(RA_BEGIN+2)<<"\t"<<current_q_dot_(RA_BEGIN+3)<<"\t"<<current_q_dot_(RA_BEGIN+4)<<"\t"<<current_q_dot_(RA_BEGIN+5)<<"\t"<<current_q_dot_(RA_BEGIN+6)
//              <<"\t"<<E_con(0)<<"\t"<<E_con(1)<<"\t"<<E_con(2)<<"\t"<<E_con(3)<<"\t"<<E_con(4)<<"\t"<<E_con(5)
//                <<"\t"<<E_con(6)<<"\t"<<E_con(7)<<"\t"<<E_con(8)<<"\t"<<E_con(9)<<"\t"<<E_con(10)<<"\t"<<E_con(11)
//              <<"\t"<<E_con_total<<endl;
//        file[32]<<walking_tick_<<"\t"<<lfoot_trajectory_euler_float_(0)*RAD2DEG<<"\t"<<lfoot_trajectory_euler_float_(1)*RAD2DEG<<"\t"<<lfoot_trajectory_euler_float_(2)*RAD2DEG
//               <<"\t"<<rfoot_trajectory_euler_float_(0)*RAD2DEG<<"\t"<<rfoot_trajectory_euler_float_(1)*RAD2DEG<<"\t"<<rfoot_trajectory_euler_float_(2)*RAD2DEG
//              <<"\t"<<lfoot_trajectory_euler_support_(0)*RAD2DEG<<"\t"<<lfoot_trajectory_euler_support_(1)*RAD2DEG<<"\t"<<lfoot_trajectory_euler_support_(2)*RAD2DEG
//                <<"\t"<<rfoot_trajectory_euler_support_(0)*RAD2DEG<<"\t"<<rfoot_trajectory_euler_support_(1)*RAD2DEG<<"\t"<<rfoot_trajectory_euler_support_(2)*RAD2DEG
//               <<"\t"<<rfoot_trajectory_dot_support_(3)<<"\t"<<rfoot_trajectory_dot_support_(4)<<"\t"<<rfoot_trajectory_dot_support_(5)
//              <<"\t"<<lfoot_trajectory_dot_support_(3)<<"\t"<<lfoot_trajectory_dot_support_(4)<<"\t"<<lfoot_trajectory_dot_support_(5)
//               <<endl;
//        file[34]<<walking_tick_<<"\t"<<current_torque_(0)<<"\t"<<current_torque_(1)<<"\t"<<current_torque_(2)<<"\t"<<current_torque_(3)<<"\t"<<current_torque_(4)<<"\t"<<current_torque_(5)
//               <<"\t"<<current_torque_(6)<<"\t"<<current_torque_(7)<<"\t"<<current_torque_(8)<<"\t"<<current_torque_(9)<<"\t"<<current_torque_(10)<<"\t"<<current_torque_(11)<<endl;

        ///////////////////////////////////////////////

        }

//        if(joystick_walking_flag_ == true){
////            cout<<"joystick walking flag on:"<<endl;
//            if(joystick_planning_ == true){
////                cout<<"joustick planning on "<<endl;
//                if(joystick_input_(3) > 0){
//                //joystick trigger unpushed

//                }
//                else{
//                    //cout<<"joystick left button pushed"<<endl;
////                    desired_q_ = q_init_;
//                //    desired_q_ = current_q_;
////                    if(walking_tick_ == 0)
////                    {
////                      desired_q_ = q_init_;
////                    }

//                }
//                updateNextStepTime();
//                //cout<<"walking tick : "<<walking_tick_<<endl;
//       //         updateNextStepTime();
//            }
//            else {
//               // cout<<"joystick planning off"<<endl;

//                desired_q_ = current_q_;
//                if(walking_tick_ == 0)
//                {
//                  desired_q_ = q_init_;
//                }
//                current_step_num_ =0;
//            }

//             desired_q_ = q_init_;
//        }
//        else


            updateNextStepTime();

//            if(joystick_planning_ == false)
//                desired_q_ = current_q_;


      }
      else
      {

        desired_q_ = current_q_;
        cout<<"a"<<endl;
      }
    }// end of if(ready_for_compute_flag_ == true || lqr_compensator_mode_ == false)
    else
    {

      desired_q_ = current_q_;
      cout<<"b"<<endl;

    }
    p_q_ = current_q_;



    pre_rfoot_trajectory_float_ = rfoot_trajectory_float_;
    pre_lfoot_trajectory_float_ = lfoot_trajectory_float_;

    pre_lfoot_trajectory_waist_ = lfoot_trajectory_waist_;
    pre_rfoot_trajectory_waist_ = rfoot_trajectory_waist_;

//    file[3]<<walking_tick_<<"\t"<<current_step_num_<<"\t"<<current_q_(0)<<"\t"<<current_q_(1)<<"\t"<<current_q_(2)<<"\t"<<current_q_(3)<<"\t"<<current_q_(4)<<"\t"<<current_q_(5)<<"\t"<<
//             current_q_(6)<<"\t"<<current_q_(7)<<"\t"<<current_q_(8)<<"\t"<<current_q_(9)<<"\t"<<current_q_(10)<<"\t"<<current_q_(11)<<endl;


  }
}

void WalkingController::getZmpReal(){
    Eigen::Vector2d left_zmp, right_zmp;

    left_zmp(0) = l_ft_(4)/l_ft_(2) + lfoot_support_current_.translation()(0);// adding support foot position
    left_zmp(1) = l_ft_(3)/l_ft_(2) + lfoot_support_current_.translation()(1);

    right_zmp(0) = r_ft_(4)/r_ft_(2) + rfoot_support_current_.translation()(0);// adding support foot position
    right_zmp(1) = r_ft_(3)/r_ft_(2) + rfoot_support_current_.translation()(1);


    zmp_measured_(0) = (left_zmp(0) * l_ft_(2) + right_zmp(0) * r_ft_(2))/(l_ft_(2) + r_ft_(2));
    zmp_measured_(1) = (left_zmp(1) * l_ft_(2) + right_zmp(1) * r_ft_(2))/(l_ft_(2) + r_ft_(2));

//    file[33]<<walking_tick_<<"\t"<<zmp_measured_(0)<<"\t"<<zmp_measured_(1);

    left_zmp(0) = l_ft_(4)/l_ft_(2) + lfoot_trajectory_support_.translation()(0);// adding support foot position
    left_zmp(1) = l_ft_(3)/l_ft_(2) + lfoot_trajectory_support_.translation()(1);

    right_zmp(0) = r_ft_(4)/r_ft_(2) + rfoot_trajectory_support_.translation()(0);// adding support foot position
    right_zmp(1) = r_ft_(3)/r_ft_(2) + rfoot_trajectory_support_.translation()(1);


    zmp_measured_(0) = (left_zmp(0) * l_ft_(2) + right_zmp(0) * r_ft_(2))/(l_ft_(2) + r_ft_(2));
    zmp_measured_(1) = (left_zmp(1) * l_ft_(2) + right_zmp(1) * r_ft_(2))/(l_ft_(2) + r_ft_(2));

//    file[33]<<"\t"<<zmp_measured_(0)<<"\t"<<zmp_measured_(1);

}

void WalkingController::setTarget(int walk_mode, bool hip_compensation, bool lqr, int ik_mode, bool heel_toe,
                                  bool is_right_foot_swing, double x, double y, double z, double height, double theta,
                                  double step_length_x, double step_length_y,bool walking_pattern)
{
    cout<<"set target at walking ctrl" <<endl;
    cout<<"joystick walkingflag : "<<joystick_walking_flag_<<endl;
    joystick_walking_flag_ = false;
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

  parameterSetting();
}



void WalkingController::setEnable(bool enable)
{
  walking_enable_=enable;
  desired_q_ = current_q_;
}

void WalkingController::updateControlMask(unsigned int *mask)
{
  if(walking_enable_)
  {
    for (int i=0; i<total_dof_; i++) //control only leg
    {
      mask[i] = (mask[i] | PRIORITY);
    }
    mask[total_dof_-1] = (mask[total_dof_-1] & ~PRIORITY); //Gripper
    mask[total_dof_-2] = (mask[total_dof_-2] & ~PRIORITY); //Gripper
    mask[total_dof_-3] = (mask[total_dof_-3] & ~PRIORITY); //Head
    mask[total_dof_-4] = (mask[total_dof_-4] & ~PRIORITY); //Head
  }
  else
  {
    for (int i=0; i<total_dof_; i++)
    {
      mask[i] = (mask[i] & ~PRIORITY);
    }
  }
}

void WalkingController::writeDesired(const unsigned int *mask, VectorQd& desired_q)
{
  for(unsigned int i=0; i<total_dof_; i++)
  {
//      if(lqr_compensator_mode_ ==  true){
//          if( mask[i] >= PRIORITY && mask[i] < PRIORITY * 2 )
//          {
//            if(walking_tick_ == 0)
//            {
//              desired_q(i) = desired_q_(i);
//            }
//            else
//            {
//              desired_q(i) = DOB_IK_output_b_(i);
//              desired_q(12) = desired_q_(12); // 허리
//              desired_q(13) = desired_q_(13); // 허리
//            }
//          }
//      }

//      else{
          if( mask[i] >= PRIORITY && mask[i] < PRIORITY * 2 )
          {
            desired_q(i) = desired_q_(i);
             if(lqr_compensator_mode_ ==  true)
             {
                 if(i <12)
                    desired_q(i) = DOB_IK_output_b_(i);
             }
          }
//      }

  }
}

void WalkingController::parameterSetting()
{
  /*
  t_last_ = 0.1*hz_;
  t_start_= 0.1*hz_;
  t_temp_= 0.1*hz_; /
  t_rest_init_ = 0.1*hz_; /
  t_rest_last_= 0.1*hz_; /
  t_double1_= 0.1*hz_; /
  t_double2_= 0.1*hz_; /
  t_total_= 1.3*hz_; /
  t_temp_ = 3.0*hz_; /
  */

  double ratio = 1.2;

    t_double1_ = ratio*0.15*hz_;
    t_double2_ = ratio*0.15*hz_;
    t_rest_init_ = ratio*0.05*hz_;
    t_rest_last_ = ratio*0.05*hz_;
    //t_total_= 0.8*hz_;
    t_total_ = ratio*1.0*hz_;

    t_total_init_ = t_total_;
    t_double1_init_ = t_double1_;


/*
  t_double1_ = 0.10*hz_;
  t_double2_ = 0.10*hz_;
  t_rest_init_ = 1.0*hz_;
  t_rest_last_ = 1.0*hz_;
  t_total_= 3.0*hz_;
*/
  t_temp_ = 3.0*hz_;
  t_last_ = t_total_ + t_temp_;
  t_start_ = t_temp_+1;

  //initialize (should revise)


  t_start_real_ = t_start_ + t_rest_init_;

  current_step_num_ = 0;
  walking_tick_ = 0;
  walking_time_ = 0;
  joystick_rotation_num_ = 0;
  pre_theta_ = 0;
//  theta_total_ =0;

  foot_height_ = 0.1168;//0.05;
  //com_update_flag_ = true; // frome A to B1
  gyro_frame_flag_ = false;
  com_control_mode_ = true;
  estimator_flag_ = false;
  swing_foot_flag_ = false;
  matrix_get_flag_ = false;
  MPC_Matrix_cal_ = false;

  //zc_ = 0.75;

  ///////////cvxgen setting////////////
  set_defaults();
  setup_indexing();
  //////////kalman filter//////////////////

  /// q limit setting /////
//  q_leg_max_.resize(12);
//  q_leg_min_.resize(12);
  SettingJointLimit();
//  kalmanStateSpace3();
//  kalmanStateSpace2(); //kalman statespace eq with com bias term
//  kalmanStateSpace1();

/**Foot step related fuctions
 */
}

void WalkingController::getRobotState()
{
    Eigen::Matrix<double, DyrosJetModel::MODEL_WITH_VIRTUAL_DOF, 1> q_temp, qdot_temp;
      q_temp.setZero();
      qdot_temp.setZero();


      UpdateCentroidalMomentumMatrix();

      q_temp.segment<28>(6) = current_q_.segment<28>(0);
      if(walking_tick_ > 0)
      {
        q_temp.segment<12>(6) =   desired_q_not_compensated_.segment<12>(0);
//          q_temp.segment<28>(6) =   desired_q_not_compensated_.segment<28>(0);
      }

      qdot_temp.segment<28>(6) = current_q_dot_.segment<28>(0);
      model_.updateKinematics(q_temp,qdot_temp);


      Eigen::Vector6d CMM_temp;
      CMM_temp = Augmented_Centroidal_Momentum_Matrix_*current_q_dot_.segment<28>(0);

      Eigen::Vector3d linear_momentum;
//      Eigen::Vector3d com_velocity;
      current_Angular_momentum_ = model_.getCurrentComAngularMomentum();
      linear_momentum = model_.getCurrentComLinearMomentum();

      file[27]<<walking_tick_<<"\t"<<CMM_temp(0)<<"\t"<<CMM_temp(1)<<"\t"<<CMM_temp(2)<<"\t"<<CMM_temp(3)<<"\t"<<CMM_temp(4)<<"\t"<<CMM_temp(5)
             <<"\t"<<linear_momentum(0)<<"\t"<<linear_momentum(1)<<"\t"<<linear_momentum(2)<<"\t"<<current_Angular_momentum_(0)<<"\t"<<current_Angular_momentum_(1)<<"\t"<<current_Angular_momentum_(2)<<endl;
//      Eigen::Vector3d Linear_momentum_times_mass;
//      Linear_momentum_times_mass = 50.3581*Linear_momentum_temp;
//      com_velocity = model_.getCurrentComDot();
//      file[27]<<walking_tick_<<"\t"<<current_Angular_momentum_(0)<<"\t"<<current_Angular_momentum_(1)<<"\t"<<current_Angular_momentum_(2)<<"\t"<<Angular_momentum_temp(0)<<"\t"<<Angular_momentum_temp(1)<<"\t"<<Angular_momentum_temp(2)
//             <<"\t"<<com_velocity(0)<<"\t"<<com_velocity(1)<<"\t"<<com_velocity(2)<<"\t"<<Linear_momentum_temp(0)<<"\t"<<Linear_momentum_temp(1)<<"\t"<<Linear_momentum_temp(2)
//            <<"\t"<<linear_momentum(0)<<"\t"<<linear_momentum(1)<<"\t"<<linear_momentum(2)<<"\t"<<Linear_momentum_times_mass(0)<<"\t"<<Linear_momentum_times_mass(1)<<"\t"<<Linear_momentum_times_mass(2)<<endl;

      /////////////////////////////FTsensor low-pass-filtering////////////////////////////////

      if(walking_tick_ == 0)
      {
        r_ft_pre_ = r_ft_;
        r_ft_ppre_ = r_ft_;
        r_ft_filtered_pre_ = r_ft_;
        r_ft_filtered_ppre_ = r_ft_;

        l_ft_pre_ = l_ft_;
        l_ft_ppre_ = l_ft_;
        l_ft_filtered_pre_ = l_ft_;
        l_ft_filtered_ppre_ = l_ft_;
      }

      for(int i=0; i<6; i++)
      {
        r_ft_filtered_(i) = DyrosMath::secondOrderLowPassFilter(r_ft_(i), r_ft_pre_(i), r_ft_ppre_(i), r_ft_filtered_pre_(i), r_ft_filtered_ppre_(i), 10, 1, hz_);
        l_ft_filtered_(i) = DyrosMath::secondOrderLowPassFilter(l_ft_(i), l_ft_pre_(i), l_ft_ppre_(i), l_ft_filtered_pre_(i), l_ft_filtered_ppre_(i), 10, 1, hz_);
        // r_ft_filtered_(i) = 0.8*r_ft_filtered_pre_(i) + 0.2*r_ft_(i);
        // l_ft_filtered_(i) = 0.8*l_ft_filtered_pre_(i) + 0.2*l_ft_(i);
        // r_ft_filtered_(i) = r_ft_(i);
        // l_ft_filtered_(i) = l_ft_(i);
      }

      r_ft_ppre_ = r_ft_pre_;
      r_ft_pre_ = r_ft_;
      r_ft_filtered_ppre_ = r_ft_filtered_pre_;
      r_ft_filtered_pre_ = r_ft_filtered_;

      l_ft_ppre_ = l_ft_pre_;
      l_ft_pre_ = l_ft_;
      l_ft_filtered_ppre_ = l_ft_filtered_pre_;
      l_ft_filtered_pre_ = l_ft_filtered_;
      ////////////////////////////////////////////////////////


      com_sim_old_ = com_sim_current_;
      com_float_old_ = com_float_current_;
      com_support_old_ = com_support_current_;

      lfoot_float_current_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)0);
      rfoot_float_current_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)1);
      com_float_current_ = model_.getCurrentCom();
      com_dot_float_current_ = model_.getCurrentComDot();
      current_Angular_momentum_ = model_.getCurrentComAngularMomentum();



      com_float_current_(0) -= 0.03;

      lfoot_float_current_euler_ = DyrosMath::rot2Euler(lfoot_float_current_.linear());
      rfoot_float_current_euler_ = DyrosMath::rot2Euler(rfoot_float_current_.linear());

        Eigen::Vector3d ankle_to_heel;
      ankle_to_heel(0) = -0.15; ankle_to_heel(1) = 0; ankle_to_heel(2) = -0.11;

      Eigen::Isometry3d lfoot_plane_temp, rfoot_plane_temp;
      Eigen::Vector3d lheel_temp, rheel_temp;
      lfoot_plane_temp.translation() = lfoot_float_current_.translation() + lfoot_float_current_.linear()*ankle_to_heel - ankle_to_heel;
      rfoot_plane_temp.translation() = rfoot_float_current_.translation() + rfoot_float_current_.linear()*ankle_to_heel - ankle_to_heel;
      lfoot_plane_temp.linear().setIdentity();
      rfoot_plane_temp.linear().setIdentity();

      lheel_temp = lfoot_float_current_.translation() + lfoot_float_current_.linear()*ankle_to_heel;
      rheel_temp = rfoot_float_current_.translation() + rfoot_float_current_.linear()*ankle_to_heel;


      Eigen::Vector3d l_ankle_float,r_ankle_float,l_ankle_local,r_ankle_local;
      l_ankle_float = DyrosMath::rot2Euler(lfoot_float_current_.linear());
      r_ankle_float = DyrosMath::rot2Euler(rfoot_float_current_.linear());

      l_ankle_local = DyrosMath::rot2Euler(relative_rotation_[LF_BEGIN+4]);
      r_ankle_local = DyrosMath::rot2Euler(relative_rotation_[RF_BEGIN+4]);

      Eigen::Vector3d lfoot_rot_y, rfoot_rot_y;
      lfoot_rot_y = lfoot_float_current_.translation() + DyrosMath::rotateWithY(l_ankle_float(1))*ankle_to_heel - ankle_to_heel;
      rfoot_rot_y = rfoot_float_current_.translation() + DyrosMath::rotateWithY(r_ankle_float(1))*ankle_to_heel - ankle_to_heel;

      file[11]<<walking_tick_<<"\t"<<lfoot_plane_temp.translation()(0)<<"\t"<<lfoot_plane_temp.translation()(1)<<"\t"<<lfoot_plane_temp.translation()(2)
             <<"\t"<<rfoot_plane_temp.translation()(0)<<"\t"<<rfoot_plane_temp.translation()(1)<<"\t"<<rfoot_plane_temp.translation()(2)
            <<"\t"<<lheel_temp(0)<<"\t"<<lheel_temp(1)<<"\t"<<lheel_temp(2)
           <<"\t"<<rheel_temp(0)<<"\t"<<rheel_temp(1)<<"\t"<<rheel_temp(2)
          <<"\t"<<lfoot_rot_y(0)<<"\t"<<lfoot_rot_y(1)<<"\t"<<lfoot_rot_y(2)
         <<"\t"<<rfoot_rot_y(0)<<"\t"<<rfoot_rot_y(1)<<"\t"<<rfoot_rot_y(2)<<endl;

//      if(walking_tick_ ==0)
//        cout<<"com float  "<<com_float_current_<<endl;

      com_sim_current_ = model_.getSimulationCom();
      gyro_sim_current_ = model_.getSimulationGyro();
      accel_sim_current_ = model_.getSimulationAccel();
      lfoot_sim_global_current_ = model_.getSimulationLfoot();
      rfoot_sim_global_current_ = model_.getSimulationRfoot();
      base_sim_global_current_ = model_.getSimulationBase();

      // q_sim_virtual_ = model_.getMujocoCom();
      // q_sim_dot_virtual_ = model_.getMujocoComDot();

      r_ft_ = model_.getRightFootForce();
      l_ft_ = model_.getLeftFootForce();
      imu_acc_ = model_.getImuAccel();
      imu_ang_ = model_.getImuAngvel();
      imu_grav_rpy_ = model_.getImuGravityDirection();


      DyrosMath::ComplimantaryFilter(imu_acc_,imu_ang_,angle_,hz_);
      getZmpReal(); //1~5
    //  ZMP_2(l_ft_,r_ft_,lfoot_trajectory_support_.translation(),rfoot_trajectory_support_.translation(),ZMP_2_);//6~9
      ZMP_2(l_ft_filtered_,r_ft_filtered_,lfoot_trajectory_support_.translation(),rfoot_trajectory_support_.translation(),ZMP_2_);//10~13

      file[33]<<endl;
      zmptoInitFloat();//14~17
      computeZmp();//18~22

      file[26]<<walking_tick_<<"\t"<<gyro_sim_current_(0)<<"\t"<<gyro_sim_current_(1)<<"\t"<<gyro_sim_current_(2)
             <<"\t"<<accel_sim_current_(0)<<"\t"<<accel_sim_current_(1)<<"\t"<<accel_sim_current_(2)
            <<"\t"<<imu_acc_(0)<<"\t"<<imu_acc_(1)<<"\t"<<imu_acc_(2)
              <<"\t"<<imu_ang_(0)*RAD2DEG<<"\t"<<imu_ang_(1)*RAD2DEG<<"\t"<<imu_ang_(2)*RAD2DEG
             <<"\t"<<angle_(0)*RAD2DEG<<"\t"<<angle_(1)*RAD2DEG<<"\t"<<angle_(2)*RAD2DEG
               <<"\t"<<angle_init_(0)*RAD2DEG<<"\t"<<angle_init_(1)*RAD2DEG<<"\t"<<angle_init_(2)*RAD2DEG
             <<endl;

      pelv_float_current_.setIdentity();

      rfoot_sim_float_current_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(base_sim_global_current_), rfoot_sim_global_current_);
      lfoot_sim_float_current_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(base_sim_global_current_), lfoot_sim_global_current_);

    //  angle_(0) = DyrosMath::cubic(walking_tick_,200,400,0.0,5.0*DEGREE,0.0,0.0);
    //  if(walking_tick_ >=400)
    //      angle_(0) = DyrosMath::cubic(walking_tick_,400,600,5.0*DEGREE,0.0,0.0,0.0);


      com_sim_current_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_sim_current_), com_sim_current_);

      //com_sim_dot_current_ = (base_sim_global_current_.linear()).transpose()*com_sim_dot_current_;

      //com_sim_current_ = (base_sim_global_current_.linear()).transpose()*com_sim_current_; //change frame from glrobal to pelv
      //com_sim_current_ = DyrosMath::multiplyIsometry3dVector3d(pelv_support_current_, com_sim_current_); //change frame from pelv to support foot

      current_leg_jacobian_l_ = model_.getLegJacobian((DyrosJetModel::EndEffector) 0);
      current_leg_jacobian_r_ = model_.getLegJacobian((DyrosJetModel::EndEffector) 1);
      current_arm_jacobian_l_ = model_.getArmJacobian((DyrosJetModel::EndEffector) 2);
      current_arm_jacobian_r_ = model_.getArmJacobian((DyrosJetModel::EndEffector) 3);
//      current_waist_jacobian_[0] = model_.getWaistJacobian(0);
//      current_waist_jacobian_[1] = model_.getWaistJacobian(1);




      for(unsigned int i=0;i<29;i++){
          link_local_com_position_[i] = model_.getLinkComPosition(i);
          link_inertia_[i] = model_.getLinkInertia(i);
          link_transform_[i] = model_.getCurrentLinkTransform(i);
//          link_mass_[i] = model_.getLinkMass(i);
      }


      Eigen::Vector3d base_frame;
      base_frame = model_.getBasePosition();

      if(walking_tick_ == 0){
          cout<<" base frame : "<<base_frame(0)<<"\t"<<base_frame(1)<<"\t"<<base_frame(2)<<endl;
          cout<<"pelvis fram : "<<link_transform_[0].translation()(0)<<"\t"<<link_transform_[0].translation()(1)<<"\t"<<link_transform_[0].translation()(2)<<endl;
      }

      if(walking_tick_ == 0){
          Eigen::Matrix<double, 29, 1> link_mass_vector;
          link_mass_vector = model_.getLinkMassVector();

          for(int i=0;i<29;i++){
            link_mass_[i] = link_mass_vector(i);
//            cout<<"com local position of "<<i<<" th link? : "<<link_local_com_position_[i](0)<<"\t"<<link_local_com_position_[i](1)<<"\t"<<link_local_com_position_[i](2)<<endl;
          }
      }


      Eigen::Vector3d lankle_euler, rankle_euler;
      lankle_euler = DyrosMath::rot2Euler(link_transform_[LF_BEGIN+5].linear());
      rankle_euler = DyrosMath::rot2Euler(link_transform_[RF_BEGIN+5].linear());

//      if(walking_tick_ == 550)
//          cout<<" left ankle float euler : "<<lankle_euler(0)<<"\t"<<lankle_euler(1)<<"\t"<<lankle_euler(2)<<endl<<"right ankle float euler : "<<rankle_euler(0)<<"\t"<<rankle_euler(1)<<"\t"<<rankle_euler(2)<<endl;

//      if(walking_tick_ == 0){
//          cout<<"center of mass at link "<<endl;
//          for(int i=0;i<29;i++)
//              cout<<i<<"th link : "<<link_local_com_position_[i][0]<<",  "<<link_local_com_position_[i][1]<<",  "<<link_local_com_position_[i][2]<<endl;

//      }


//      link_mass_[0] =3.90994;
//      link_mass_[1] =1.54216;   link_mass_[2] =1.16907;   link_mass_[3] =3.28269;   link_mass_[4] =2.04524;  link_mass_[5] =1.1845;   link_mass_[6] =1.42541;
//      link_mass_[7] =1.54216;   link_mass_[8] =1.16907;   link_mass_[9] =3.28269;  link_mass_[10] =2.04524;   link_mass_[11] = 1.1845;   link_mass_[12] = 1.42541;
//      link_mass_[13] = 0.18235;  link_mass_[14] = 14.09938;


      if(walking_tick_ == 0 ){
          total_mass_ =0;
          for(int i=0;i<29;i++)
              total_mass_ += link_mass_[i];

//          cout<<"total mass : "<<total_mass_<<endl;
      }

      if(walking_tick_ ==0){
//          cout<<"before gyro compensation"<<endl;
//          cout<<"pelvis float  linear  "<<endl<<pelv_float_current_.linear()<<endl;
//          cout<<"lfoot float position"<<endl<<lfoot_float_current_.translation()<<endl;
//          cout<<"lfoot float linea"<<endl<<lfoot_float_current_.linear()<<endl;
//          cout<<"rfoot float position"<<endl<<rfoot_float_current_.translation()<<endl;
//          cout<<"rfoot float linea"<<endl<<rfoot_float_current_.linear()<<endl<<endl;

    //      cout<<"pelivs support "<<endl<<pelv_support_current_.translation()<<endl<<"pelvis support linear : "<<endl<<pelv_support_current_.linear()<<endl;
    //      cout<<"lfoot support position"<<endl<<lfoot_support_current_.translation()<<endl;
    //      cout<<"lfoot support linea"<<endl<<lfoot_support_current_.linear()<<endl;
    //      cout<<"rfoot support position"<<endl<<rfoot_support_current_.translation()<<endl;
    //      cout<<"rfoot support linea"<<endl<<rfoot_support_current_.linear()<<endl;

      }
    //  if(gyro_frame_flag_ == true)
    //     changeGyroframe();

      if(foot_step_(current_step_num_, 6) == 0)  //right foot support
      {
        supportfoot_float_current_ = rfoot_float_current_;
        supportfoot_float_current_.linear().setIdentity();
//          supportfoot_float_current_ = rfoot_plane_temp;
//          supportfoot_float_current_.translation()(2) = lfoot_float_current_.translation()(2);
        supportfoot_float_sim_current_ = rfoot_sim_float_current_;
      }
      else if(foot_step_(current_step_num_, 6) == 1)
      {
        supportfoot_float_current_ = lfoot_float_current_;
        supportfoot_float_current_.linear().setIdentity();
//        supportfoot_float_current_ = lfoot_plane_temp;
//        supportfoot_float_current_.translation()(2) = rfoot_float_current_.translation()(2);
        supportfoot_float_sim_current_ = lfoot_sim_float_current_;
      }

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

      com_support_current_CLIPM_b = com_support_current_CLIPM_;
      com_support_current_CLIPM_ =  DyrosMath::multiplyIsometry3dVector3d(pelv_support_current_, com_float_current_);
      com_support_current_CLIPM_(0) -= 0.07;

      //rfoot_float_euler_current_ = DyrosMath::rot2Euler(rfoot_float_current_.linear());
     // lfoot_float_euler_current_ = DyrosMath::rot2Euler(lfoot_float_current_.linear());

      lfoot_support_current_ = DyrosMath::multiplyIsometry3d(pelv_support_current_,lfoot_float_current_);
      rfoot_support_current_ = DyrosMath::multiplyIsometry3d(pelv_support_current_,rfoot_float_current_);

      lfoot_support_current_euler_ = DyrosMath::rot2Euler(lfoot_support_current_.linear());
      rfoot_support_current_euler_ = DyrosMath::rot2Euler(rfoot_support_current_.linear());
      com_support_current_ = DyrosMath::multiplyIsometry3dVector3d(pelv_support_current_, com_float_current_);

      if(walking_tick_ == t_start_ + t_double1_){
          lfoot_lifting_float_euler_init_ = lfoot_float_current_euler_;
          rfoot_lifting_float_euler_init_ = rfoot_float_current_euler_;

          lfoot_lifting_float_init_ = lfoot_float_current_;
          rfoot_lifting_float_init_ = rfoot_float_current_;

          lfoot_lifting_support_init_ = lfoot_support_current_;
          rfoot_lifting_support_init_ = rfoot_support_current_;

          lfoot_lifting_support_euler_init_ = lfoot_support_current_euler_;
          rfoot_lifting_support_euler_init_ = rfoot_support_current_euler_;
      }
      if(walking_tick_ == t_start_+t_total_-t_double2_-t_rest_last_)
      {
          lfoot_dsp2_float_euler_init_ = lfoot_float_current_euler_;
          rfoot_dsp2_float_euler_init_ = rfoot_float_current_euler_;

          lfoot_DSP2_float_init_ = lfoot_float_current_;
          rfoot_DSP2_float_init_ = rfoot_float_current_;

          lfoot_dsp2_support_euler_init_ = lfoot_support_current_euler_;
          rfoot_dsp2_support_euler_init_ = rfoot_support_current_euler_;

          lfoot_DSP2_support_init_ = lfoot_support_current_;
          rfoot_DSP2_support_init_ = rfoot_support_current_;
      }

//      getRobotStatefromWaist();

//      if(walking_tick_ == 0){
//         cout<<"left leg jacobian : "<<endl<<current_leg_jacobian_l_<<endl;
//         cout<<"right leg jacobian : "<<endl<<current_leg_jacobian_r_<<endl;
//         cout<<"left arm jacobian : "<<endl<<current_arm_jacobian_l_<<endl;
//         cout<<"right arm jacobian "<<endl<<current_arm_jacobian_r_<<endl;
//         cout<<"waist jacobian : "<<endl<<current_waist_jacobian_[1]<<endl;

//         cout<<"jacobian including waist new "<<endl<<current_leg_waist_l_jacobian_<<endl;
//      }

//      if(walking_tick_ ==0){
//          cout<<"after gyro compensation"<<endl;
//          cout<<"pelvis float  linear  "<<endl<<pelv_float_current_.linear()<<endl;
//          cout<<"lfoot float position"<<endl<<lfoot_float_current_.translation()<<endl;
//          cout<<"lfoot float linea"<<endl<<lfoot_float_current_.linear()<<endl;
//          cout<<"rfoot float position"<<endl<<rfoot_float_current_.translation()<<endl;
//          cout<<"rfoot float linea"<<endl<<rfoot_float_current_.linear()<<endl;

//          cout<<"pelivs support "<<endl<<pelv_support_current_.translation()<<endl<<"pelvis support linear : "<<endl<<pelv_support_current_.linear()<<endl;
//          cout<<"lfoot support position"<<endl<<lfoot_support_current_.translation()<<endl;
//          cout<<"lfoot support linea"<<endl<<lfoot_support_current_.linear()<<endl;
//          cout<<"rfoot support position"<<endl<<rfoot_support_current_.translation()<<endl;
//          cout<<"rfoot support linea"<<endl<<rfoot_support_current_.linear()<<endl;


//      }
    //  if(walking_tick_ == 0 ){
    //      cout<<" check rbdl "<<endl;
    //      cout<<"link com position : "<<endl;
    //      for(unsigned int i=0;i<29;i++)
    //          cout<<link_local_com_position_[i]<<endl;

    //      cout<<"link_inertia : "<<endl;
    //      for(unsigned int i=0;i<29;i++)
    //          cout<<link_inertia_[i]<<endl;
    //      cout<<"link transform position: "<<endl;
    //      for(unsigned int i=0;i<29;i++)
    //          cout<<link_transform_[i].translation()<<endl;
    //      cout<<"link transform rotation matrix : "<<endl;
    //      for(unsigned int i=0;i<29;i++)
    //          cout<<link_transform_[i].linear()<<endl;
    //      cout<<"link mass : "<<endl;
    //      for(unsigned int i=0;i<29;i++)
    //          cout<<link_mass_[i]<<endl;
    //  }

      slowcalc_mutex_.lock();
      thread_q_ = current_q_;
      current_motor_q_leg_ = current_q_.segment<12>(0);
      //current_link_q_leg_ = model_.getCurrentExtencoder();
      current_link_q_leg_ = current_q_.segment<12>(0);

      slowcalc_mutex_.unlock();

}
void WalkingController::calculateFootStepTotal()
{
  /***
   * this function calculate foot steps which the robot should put on
   * algorith: set robot orientation to the destination -> go straight -> set target orientation on the destination
   *
   * foot_step_(current_step_num_, i) is the foot step where the robot will step right after
   * foot_step_(crrennt_step_num_, 6) = 0 means swingfoot is left(support foot is right)
   */


  double initial_rot;
  double final_rot = 0.0;
  double initial_drot = 0.0;
  double final_drot = 0.0;

  initial_rot= atan2(target_y_, target_x_);

  if(initial_rot > 0.0)
    initial_drot = 10*DEG2RAD;
  else
    initial_drot = -10*DEG2RAD;

  unsigned int initial_total_step_number = initial_rot/initial_drot;
  double initial_residual_angle = initial_rot-initial_total_step_number*initial_drot;

  final_rot = target_theta_-initial_rot;
  if(final_rot > 0.0)
    final_drot = 10*DEG2RAD;
  else
    final_drot = -10*DEG2RAD;

  unsigned int final_total_step_number = final_rot/final_drot;
  double final_residual_angle = final_rot-final_total_step_number*final_drot;
  double length_to_target = sqrt(target_x_*target_x_+target_y_*target_y_);
  double dlength = step_length_x_; //footstep length;
  unsigned int middle_total_step_number = length_to_target/(dlength);
  double middle_residual_length = length_to_target-middle_total_step_number*(dlength);

  if(length_to_target == 0)
  {
    middle_total_step_number = 5; //walking on the spot 10 times
    dlength = 0;
  }


  unsigned int number_of_foot_step;


  int del_size;

  del_size = 1;
  number_of_foot_step = initial_total_step_number*del_size+middle_total_step_number *del_size+final_total_step_number*del_size;

  cout<<"fist number of foot step size : "<<number_of_foot_step<<endl;
  if(initial_total_step_number!=0 || abs(initial_residual_angle)>=0.0001)
  {
    if(initial_total_step_number%2==0)
      number_of_foot_step = number_of_foot_step+2*del_size;
    else
    {
      if(abs(initial_residual_angle)>= 0.0001)
        number_of_foot_step = number_of_foot_step+3*del_size;
      else
        number_of_foot_step = number_of_foot_step+del_size;
    }
  }

  if(middle_total_step_number!=0 || abs(middle_residual_length)>=0.0001)
  {
    if(middle_total_step_number%2==0){
      number_of_foot_step = number_of_foot_step+2*del_size;
      cout<<"middle&2==0 number of foot step size : "<<number_of_foot_step<<endl;
    }
    else
    {
      if(abs(middle_residual_length)>= 0.0001){
        number_of_foot_step = number_of_foot_step+3*del_size;
        cout<<"middle&2>0.0001 number of foot step size : "<<number_of_foot_step<<endl;
      }
      else{
        number_of_foot_step = number_of_foot_step+del_size;
        cout<<"middle&2 =! 0 number of foot step size : "<<number_of_foot_step<<endl;
      }
    }
  }

  if(final_total_step_number!=0 || abs(final_residual_angle)>= 0.0001)
  {
    if(abs(final_residual_angle)>= 0.0001)
      number_of_foot_step = number_of_foot_step+2*del_size;
    else
      number_of_foot_step = number_of_foot_step+del_size;
  }



  foot_step_.resize(number_of_foot_step, 7);
  foot_step_.setZero();
  foot_step_support_frame_.resize(number_of_foot_step, 7);
  foot_step_support_frame_.setZero();
  foot_step_support_frame_offset_.resize(number_of_foot_step, 7);
  foot_step_support_frame_offset_.setZero();

  int index = 0;
  int temp, temp2, temp3, is_right;

  if(is_right_foot_swing_ == true)
    is_right = 1;
  else
    is_right = -1;


  temp = -is_right; //right foot will be first swingfoot
  temp2 = -is_right;
  temp3 = -is_right;


  if(initial_total_step_number!=0 || abs(initial_residual_angle)>=0.0001)
  {
    for (int i =0 ; i<initial_total_step_number; i++)
    {
      temp *= -1;
      foot_step_(index,0) = temp*0.127794*sin((i+1)*initial_drot);
      foot_step_(index,1) = -temp*0.127794*cos((i+1)*initial_drot);
      foot_step_(index,5) = (i+1)*initial_drot;
      foot_step_(index,6) = 0.5+0.5*temp;
      index++;
    }

    if(temp==is_right)
    {
      if(abs(initial_residual_angle) >= 0.0001)
      {
        temp *= -1;

        foot_step_(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot+initial_residual_angle);
        foot_step_(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot+initial_residual_angle;
        foot_step_(index,6) = 0.5+0.5*temp;
        index++;

        temp *= -1;

        foot_step_(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot+initial_residual_angle);
        foot_step_(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot+initial_residual_angle;
        foot_step_(index,6) = 0.5+0.5*temp;
        index++;

        temp *= -1;

        foot_step_(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot+initial_residual_angle);
        foot_step_(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot+initial_residual_angle;
        foot_step_(index,6) = 0.5+0.5*temp;
        index++;

      }
      else
      {
        temp *= -1;

        foot_step_(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot+initial_residual_angle);
        foot_step_(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot+initial_residual_angle;
        foot_step_(index,6) = 0.5+0.5*temp;
        index++;
      }
    }
    else if(temp==-is_right)
    {
      temp *= -1;

      foot_step_(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot+initial_residual_angle);
      foot_step_(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
      foot_step_(index,5) = (initial_total_step_number)*initial_drot+initial_residual_angle;
      foot_step_(index,6) = 0.5+0.5*temp;
      index++;

      temp *= -1;

      foot_step_(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot+initial_residual_angle);
      foot_step_(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
      foot_step_(index,5) = (initial_total_step_number)*initial_drot+initial_residual_angle;
      foot_step_(index,6) = 0.5+0.5*temp;
      index++;
    }
  }



  if(middle_total_step_number!=0 || abs(middle_residual_length)>=0.0001)
  {
    for (int i =0 ; i<middle_total_step_number; i++)
    {
      temp2 *= -1;

      foot_step_(index,0) = cos(initial_rot)*(dlength*(i+1))+temp2*sin(initial_rot)*(0.127794);
      foot_step_(index,1) = sin(initial_rot)*(dlength*(i+1))-temp2*cos(initial_rot)*(0.127794);
      foot_step_(index,5) = initial_rot;
      foot_step_(index,6) = 0.5+0.5*temp2;
      index++;
    }

    if(temp2==is_right)
    {
      if(abs(middle_residual_length) >= 0.0001)
      {
        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length)+temp2*sin(initial_rot)*(0.127794);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length)-temp2*cos(initial_rot)*(0.127794);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5+0.5*temp2;
        index++;

        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length)+temp2*sin(initial_rot)*(0.127794);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length)-temp2*cos(initial_rot)*(0.127794);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5+0.5*temp2;
        index++;

        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length)+temp2*sin(initial_rot)*(0.127794);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length)-temp2*cos(initial_rot)*(0.127794);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5+0.5*temp2;
        index++;
      }
      else
      {
        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length)+temp2*sin(initial_rot)*(0.127794);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length)-temp2*cos(initial_rot)*(0.127794);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5+0.5*temp2;
        index++;
      }
    }
    else if(temp2==-is_right)
    {
      temp2 *= -1;

      foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length)+temp2*sin(initial_rot)*(0.127794);
      foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length)-temp2*cos(initial_rot)*(0.127794);
      foot_step_(index,5) = initial_rot;
      foot_step_(index,6) = 0.5+0.5*temp2;
      index++;

      temp2 *= -1;

      foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length)+temp2*sin(initial_rot)*(0.127794);
      foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length)-temp2*cos(initial_rot)*(0.127794);
      foot_step_(index,5) = initial_rot;
      foot_step_(index,6) = 0.5+0.5*temp2;
      index++;
    }
  }




  double final_position_x = cos(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length);
  double final_position_y = sin(initial_rot)*(dlength*(middle_total_step_number)+middle_residual_length);


  if(final_total_step_number!=0 || abs(final_residual_angle)>= 0.0001)
  {
    for (int i =0 ; i<final_total_step_number; i++)
    {
      temp3 *= -1;

      foot_step_(index,0) = final_position_x+temp3*0.127794*sin((i+1)*final_drot+initial_rot);
      foot_step_(index,1) = final_position_y-temp3*0.127794*cos((i+1)*final_drot+initial_rot);
      foot_step_(index,5) = (i+1)*final_drot+initial_rot;
      foot_step_(index,6) = 0.5+0.5*temp3;
      index++;
    }

    if(abs(final_residual_angle) >= 0.0001)
    {
      temp3 *= -1;

      foot_step_(index,0) = final_position_x+temp3*0.127794*sin(target_theta_);
      foot_step_(index,1) = final_position_y-temp3*0.127794*cos(target_theta_);
      foot_step_(index,5) = target_theta_;
      foot_step_(index,6) = 0.5+0.5*temp3;
      index++;

      temp3 *= -1;

      foot_step_(index,0) = final_position_x+temp3*0.127794*sin(target_theta_);
      foot_step_(index,1) = final_position_y-temp3*0.127794*cos(target_theta_);
      foot_step_(index,5) = target_theta_;
      foot_step_(index,6) = 0.5+0.5*temp3;
      index++;
    }
    else
    {
      temp3 *= -1;

      foot_step_(index,0) = final_position_x+temp3*0.127794*sin(target_theta_);
      foot_step_(index,1) = final_position_y-temp3*0.127794*cos(target_theta_);
      foot_step_(index,5) = target_theta_;
      foot_step_(index,6) = 0.5+0.5*temp3;
      index++;
    }
  }
  //foot_step_(0,1) = +0.127794;
  //foot_step_(0,6) = 0;
  //
  //foot_step_(1,1) = -0.127794;
  //foot_step_(1,6) = 1;
  //
  //foot_step_(2,1) = +0.127794;
  //foot_step_(2,6) = 0;
  //
  //foot_step_(3,1) = -0.127794;
  //foot_step_(3,6) = 1;
  //
  //foot_step_(4,1) = +0.127794;
  //foot_step_(4,6) = 0;
  int final_index = index;

//  for(int i=0;i<final_index;i++){
//      file[28]<<walking_tick_<<"\t"<<i<<"\t"<<foot_step_(i,0)<<"\t"<<foot_step_(i,1)<<"\t"<<foot_step_(i,2)<<"\t"<<foot_step_(i,3)<<"\t"<<foot_step_(i,4)<<"\t"<<foot_step_(i,5)<<"\t"<<foot_step_(i,6)<<endl;
//  }
}

void WalkingController::calculateFootStepSeparate()
{
  /***
   * this function calculate foot steps which the robot should put on
   * algorith: go straight to X direction -> side walk to Y direction -> set target orientation on the destination
   *
   * foot_step_(current_step_num_, i) is the foot step where the robot will step right after
   * foot_step_(crrennt_step_num_, 6) = 0 means swingfoot is left(support foot is right)
   */

  double x = target_x_;
  double y = target_y_;
  double alpha = target_theta_;

  const double dx = step_length_x_;
  const double dy = step_length_y_;
  const double dtheta = 10.0*DEG2RAD;
  if(x<0.0)
    const double dx = -step_length_x_;
  if(y<0.0)
    const double dy = -step_length_y_;
  if(alpha<0.0)
    const double dtheta = -10.0*DEG2RAD;

  int x_number = x/dx;
  int y_number = y/dy;
  int theta_number = alpha/dtheta;

  double x_residual = x-x_number*dx;
  double y_residual = y-y_number*dy;
  double theta_residual = alpha-theta_number*dtheta;
  unsigned int number_of_foot_step = 0;
  int temp = -1;

  if(x_number!=0 || abs(x_residual)>=0.001)
  {
    temp *= -1;
    number_of_foot_step += 1;

    for(int i=0;i<x_number;i++)
    {
      temp *= -1;
    }
    number_of_foot_step += x_number;

    if(abs(x_residual)>=0.001)
    {
      temp *= -1;
      temp *= -1;
      number_of_foot_step += 2;
    }
    else
    {
      temp *= -1;
      number_of_foot_step += 1;
    }
  }

  if(y_number!=0 || abs(y_residual)>=0.001)
  {
    if(x==0)
    {
      if(y>=0)
        temp = -1;
      else
        temp = 1;
      temp *= -1;
      number_of_foot_step += 1;
    }

    if(y>=0 && temp==-1)
    {
      number_of_foot_step += 1;
    }
    else if(y<0 && temp==1)
    {
      number_of_foot_step += 1;
    }

    number_of_foot_step += 2*y_number;

    if(abs(y_residual)>=0.001)
    {
      number_of_foot_step += 2;
    }
  }

  if(theta_number!=0 || abs(theta_residual)>= 0.02)
  {
    number_of_foot_step += theta_number;

    if(abs(theta_residual) >= 0.02)
    {
      number_of_foot_step += 2;
    }
    else
    {
      number_of_foot_step += 1;
    }
  }


  foot_step_.resize(number_of_foot_step, 7);
  foot_step_.setZero();
  foot_step_support_frame_.resize(number_of_foot_step, 7);
  foot_step_support_frame_.setZero();
  foot_step_support_frame_offset_.resize(number_of_foot_step, 7);
  foot_step_support_frame_offset_.setZero();

  //int temp = -1;
  temp = 1; //fisrt support step is left foot
  int index = 0;

  if(x_number!=0 || abs(x_residual)>=0.001)
  {
    temp *= -1;

    foot_step_(index,0) = 0;
    foot_step_(index,1) = -temp*0.127794;
    foot_step_(index,6) = 0.5+temp*0.5;
    index++;

    for(int i=0;i<x_number;i++)
    {
      temp *= -1;

      foot_step_(index,0) = (i+1)*dx;
      foot_step_(index,1) = -temp*0.127794;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;
    }

    if(abs(x_residual)>=0.001)
    {
      temp *= -1;

      foot_step_(index,0) = x;
      foot_step_(index,1) = -temp*0.127794;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;

      temp *= -1;

      foot_step_(index,0) = x;
      foot_step_(index,1) = -temp*0.127794;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;
    }
    else
    {
      temp *= -1;

      foot_step_(index,0) = x;
      foot_step_(index,1) = -temp*0.127794;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;
    }
  }

  if(y_number!=0 || abs(y_residual)>=0.001)
  {
    if(x==0)
    {
      if(y>=0)
        temp = -1;
      else
        temp = 1;

      temp *= -1;

      foot_step_(index,0) = x;
      foot_step_(index,1) = -temp*0.127794;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;
    }

    if(y>=0 && temp==-1)
    {
      temp *= -1;

      foot_step_(index,0) = x;
      foot_step_(index,1) = -temp*0.127794;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;
    }
    else if(y<0 && temp==1)
    {
      temp *= -1;


      foot_step_(index,0) = x;
      foot_step_(index,1) = -temp*0.127794;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;
    }

    for(int i=0;i<y_number;i++)
    {
      temp *= -1;

      foot_step_(index,0) = x;
      foot_step_(index,1) = -temp*0.127794+(i+1)*dy;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;

      temp *= -1;

      foot_step_(index,0) = x;
      foot_step_(index,1) = -temp*0.127794+(i+1)*dy;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;
    }

    if(abs(y_residual)>=0.001)
    {
      temp *= -1;

      foot_step_(index,0) = x;
      foot_step_(index,1) = -temp*0.127794+y;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;

      temp *= -1;

      foot_step_(index,0) = x;
      foot_step_(index,1) = -temp*0.127794+y;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;
    }
  }


  if(theta_number!=0 || abs(theta_residual)>= 0.02)
  {
    for (int i =0 ; i<theta_number; i++)
    {
      temp *= -1;

      foot_step_(index,0) = temp*0.127794*sin((i+1)*dtheta)+x;
      foot_step_(index,1) = -temp*0.127794*cos((i+1)*dtheta)+y;
      foot_step_(index,5) = (i+1)*dtheta;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;
    }

    if(abs(theta_residual) >= 0.02)
    {
      temp *= -1;

      foot_step_(index,0) = temp*0.127794*sin(alpha)+x;
      foot_step_(index,1) = -temp*0.127794*cos(alpha)+y;
      foot_step_(index,5) = alpha;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;

      temp *= -1;

      foot_step_(index,0) = temp*0.127794*sin(alpha)+x;
      foot_step_(index,1) = -temp*0.127794*cos(alpha)+y;
      foot_step_(index,5) = alpha;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;
    }
    else
    {
      temp *= -1;

      foot_step_(index,0) = temp*0.127794*sin(alpha)+x;
      foot_step_(index,1) = -temp*0.127794*cos(alpha)+y;
      foot_step_(index,5) = alpha;
      foot_step_(index,6) = 0.5+temp*0.5;
      index++;
    }
  }
}

void WalkingController::usingFootStepPlanner()
{
  if( foot_plan_walking_last_ == true)
    {
      int temp1 = foot_step_(foot_step_plan_num_-1, 6);
      double foot_stepx, foot_stepy,foot_step_angle, foot_stepx_prev, foot_stepy_prev;

      foot_stepy_prev = foot_step_(foot_step_plan_num_ -2, 1);
      foot_stepx_prev = foot_step_(foot_step_plan_num_ -2, 0);
      foot_stepx = foot_step_(foot_step_plan_num_ -1, 0);
      foot_stepy = foot_step_(foot_step_plan_num_ -1, 1);
      foot_step_angle =  foot_step_(foot_step_plan_num_ -1,5);
      foot_step_.resize(1,7);
      foot_step_.setZero();
      foot_step_support_frame_.resize(1, 7);
      foot_step_support_frame_.setZero();
      foot_step_support_frame_offset_.resize(1, 7);
      foot_step_support_frame_offset_.setZero();
      foot_last_walking_end_ = true;

      if(temp1 == 0)
        {
      //    std::cout << "asdfaswwwww" << std::endl;
          foot_step_(0,0) = (foot_stepx - foot_stepx_prev)/2.0 + 2*(0.127794)*sin(foot_step_angle);
          foot_step_(0,1) = (foot_stepy - foot_stepy_prev)/2.0 -2*(0.127794)*cos(foot_step_angle);
          foot_step_(0,5) = foot_step_angle;
          foot_step_(0,6) = 1;
        }
      if(temp1 == 1)
        {
          std::cout << "asdfaswwwww11111" << std::endl;
          foot_step_(0,0) = (foot_stepx - foot_stepx_prev)/2.0 - 2*(0.127794)*sin(foot_step_angle);
          foot_step_(0,1) = (foot_stepy - foot_stepy_prev)/2.0 + 2*(0.127794)*cos(foot_step_angle);
          foot_step_(0,5) = foot_step_angle;
          foot_step_(0,6) = 0;
        }
    }
  else
    {
      foot_step_.resize(foot_step_plan_num_, 7);
      foot_step_.setZero();
      foot_step_support_frame_.resize(foot_step_plan_num_, 7);
      foot_step_support_frame_.setZero();
      foot_step_support_frame_offset_.resize(foot_step_plan_num_, 7);
      foot_step_support_frame_offset_.setZero();

      int temp = foot_step_start_foot_ ;
      int index = 0;
      for (int i =0 ; i<foot_step_plan_num_; i++)
      {
        foot_step_(index,0) = foot_pose_(index,0);
        foot_step_(index,1) = foot_pose_(index,1);
        foot_step_(index,5) = foot_pose_(index,5);
        foot_step_(index,6) = 0.5+0.5*temp;
        temp *= -1;
        index++;
      }

      for (int i =0 ; i<foot_step_plan_num_; i++)
      {
          if(i==0)
            {
              foot_step_(i,0) = foot_pose_(0,0);
            }
          else
            {
              foot_step_(i,0) = foot_pose_(i,0) + foot_pose_(i-1,0);
            }
         }

    }

  walking_end_foot_side_ =foot_step_(foot_step_plan_num_ - 1,6);
}

void WalkingController:: getZmpTrajectory()
{

  unsigned int planning_step_number  = 3;

  unsigned int norm_size = 0;

  if(current_step_num_ >= total_step_num_ - planning_step_number)
    norm_size = (t_last_-t_start_+1)*(total_step_num_-current_step_num_)+20*hz_;
  else
    norm_size = (t_last_-t_start_+1)*(planning_step_number);
  if(current_step_num_ == 0)
    norm_size = norm_size + t_temp_+1;
  addZmpOffset();
  zmpGenerator(norm_size, planning_step_number);
//  zmpTest(norm_size,planning_step_number);
}

void WalkingController::floatToSupportFootstep()
{
  Eigen::Isometry3d reference;
  //foot_step_support_frame_.resize(total_step_num_, 6);

  if(current_step_num_ == 0)
  {
    if(foot_step_(0,6) == 0) //right support
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
  else
  {
    reference.linear() = DyrosMath::rotateWithZ(foot_step_(current_step_num_-1,5));
    for(int i=0 ;i<3; i++)
      reference.translation()(i) = foot_step_(current_step_num_-1,i);
  }

  Eigen::Vector3d temp_local_position;
  Eigen::Vector3d temp_global_position;

  if(current_step_num_ == 0)
  {
    for(int i=0; i<total_step_num_; i++)
    {
      for(int j=0; j<3; j++)
        temp_global_position(j)  = foot_step_(i,j);

      temp_local_position = reference.linear().transpose()*(temp_global_position-reference.translation());

      for(int j=0; j<3; j++)
        foot_step_support_frame_(i,j) = temp_local_position(j);

      foot_step_support_frame_(i,3) = foot_step_(i,3);
      foot_step_support_frame_(i,4) = foot_step_(i,4);
      foot_step_support_frame_(i,5) = foot_step_(i,5) - supportfoot_float_init_(5);

    }
  }
  else
  {
    for(int i=0; i<total_step_num_; i++)
    {
      for(int j=0; j<3; j++)
        temp_global_position(j)  = foot_step_(i,j);

      temp_local_position = reference.linear().transpose()*(temp_global_position-reference.translation());

      for(int j=0; j<3; j++)
        foot_step_support_frame_(i,j) = temp_local_position(j);

      foot_step_support_frame_(i,3) = foot_step_(i,3);
      foot_step_support_frame_(i,4) = foot_step_(i,4);
      foot_step_support_frame_(i,5) = foot_step_(i,5) - foot_step_(current_step_num_-1,5);

    }
  }

  for(int j=0;j<3;j++)
    temp_global_position(j) = swingfoot_float_init_(j);

  temp_local_position = reference.linear().transpose()*(temp_global_position-reference.translation());

  for(int j=0;j<3;j++)
    swingfoot_support_init_(j) = temp_local_position(j);

  swingfoot_support_init_(3) = swingfoot_float_init_(3);
  swingfoot_support_init_(4) = swingfoot_float_init_(4);

  if(current_step_num_ == 0)
    swingfoot_support_init_(5) = swingfoot_float_init_(5) - supportfoot_float_init_(5);
  else
    swingfoot_support_init_(5) = swingfoot_float_init_(5) - foot_step_(current_step_num_-1,5);



  for(int j=0;j<3;j++)
    temp_global_position(j) = supportfoot_float_init_(j);

  temp_local_position = reference.linear().transpose()*(temp_global_position-reference.translation());

  for(int j=0;j<3;j++)
    supportfoot_support_init_(j) = temp_local_position(j);

  supportfoot_support_init_(3) = supportfoot_float_init_(3);
  supportfoot_support_init_(4) = supportfoot_float_init_(4);

  if(current_step_num_ == 0)
    supportfoot_support_init_(5) = 0;
  else
    supportfoot_support_init_(5) = supportfoot_float_init_(5) - foot_step_(current_step_num_-1,5);
}

void WalkingController::updateInitialState()
{  
  if( walking_tick_ ==0)
  {
    thread_tick_ = 0;

    calculateFootStepTotal();

//    for(int i=0;i<5;i++){
//        foot_step_.row(i+1) = foot_step_.row(0);
//    }
    cout<<"foot step"<<endl<<foot_step_<<endl;

    cout<<"current numbre of step : "<<current_step_num_<<endl;

    //ifstream com_data_1("/home/beom/catkin_ws/src/dyros_jet/dyros_jet_controller/01_com_desired_current.txt");
    ifstream com_data_2("/home/beom/catkin_ws/src/dyros_jet/dyros_jet_controller/2_com_y.txt");
    ifstream com_data_1("/home/beom/catkin_ws/src/dyros_jet/dyros_jet_controller/31_comdesired.txt");
    ifstream foot_data_1("/home/beom/catkin_ws/src/dyros_jet/dyros_jet_controller/04_foot_position_desired_current.txt");
    Obtain_com_pattern(com_data_1,com_data_2,foot_data_1,com_data_, com_data2_, foot_data_);


    q_init_ = current_q_;
    desired_q_not_compensated_ = current_q_;

    desired_leg_q_dot_.setZero();
    lfoot_float_init_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)(0));
    rfoot_float_init_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)(1));
    com_float_init_ = model_.getCurrentCom();

    lfoot_float_euler_init_ = DyrosMath::rot2Euler(lfoot_float_init_.linear());
    rfoot_float_euler_init_ = DyrosMath::rot2Euler(rfoot_float_init_.linear());

    lfoot_float_init_0_ = lfoot_float_init_;
    rfoot_float_init_0_ = rfoot_float_init_;

    pelv_float_init_.setIdentity();

    Eigen::Isometry3d ref_frame;

    Eigen::Vector3d ankle_to_heel;
    ankle_to_heel(0) = -0.15; ankle_to_heel(1) = 0; ankle_to_heel(2) = -0.11;

    Eigen::Isometry3d lfoot_plane_temp, rfoot_plane_temp;
    lfoot_plane_temp.translation() = lfoot_float_current_.translation() + lfoot_float_current_.linear()*ankle_to_heel - ankle_to_heel;
    rfoot_plane_temp.translation() = rfoot_float_current_.translation() + rfoot_float_current_.linear()*ankle_to_heel - ankle_to_heel;
    lfoot_plane_temp.linear().setIdentity();
    rfoot_plane_temp.linear().setIdentity();

    if(foot_step_(0, 6) == 0)  //right foot support
    {
      ref_frame = rfoot_float_init_;
//        ref_frame = rfoot_plane_temp;
    }
    else if(foot_step_(0, 6) == 1)
    {
      ref_frame = lfoot_float_init_;
//        ref_frame = lfoot_plane_temp;
    }

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
    com_support_init_ = pelv_support_init_.linear()*com_float_init_ + pelv_support_init_.translation();\

    lfoot_support_init_0_ = lfoot_support_init_;
    rfoot_support_init_0_ = rfoot_support_init_;
    pelv_support_init_0_ = pelv_support_init_;




    pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear());
    rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
    lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());

    supportfoot_float_init_.setZero();
    swingfoot_float_init_.setZero();


    if(foot_step_(0,6) == 1)  //left suppport foot
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

    zc_ = com_support_init_(2);// + com_height_;

//    pelv_support_init_.translation()(2) += com_height_;
//    cout<<"com height input : "<<com_height_<<endl<<"pelv support init "<<endl<<pelv_support_init_.translation()<<endl;
    pelv_suppprt_start_ = pelv_support_init_;

//    cout<<"com support init "<<endl<<com_support_init_<<endl;
//    pelv_suppprt_start_.translation()(2) = pelv_support_init_.translation()(2) + 0.07;


    total_step_num_ = foot_step_.col(1).size();

//    cout<<"total number of step at ui: "<<total_step_num_<<endl;
//    cout<<"count  " <<walking_tick_<<endl;

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
//    updateInitialStatefromWaist();

  }
//  else if(current_step_num_ ==0 && walking_tick_ == t_start_){

//      Eigen::Matrix<double, DyrosJetModel::MODEL_WITH_VIRTUAL_DOF, 1> q_temp, qdot_temp;
//        q_temp.setZero();
//        q_temp.segment<28>(6) = current_q_.segment<28>(0);
//        qdot_temp.segment<28>(6) = current_q_dot_.segment<28>(0);
//        if(walking_tick_ > 0)
//        {
//          //q_temp.segment<12>(6) =   desired_q_not_compensated_.segment<12>(0);
//            q_temp.segment<28>(6) =   desired_q_not_compensated_.segment<28>(0);
//        }
//       // cout<<"q temp in coumpute of walking ctrl:"<<endl<<q_temp<<endl;
//        model_.updateKinematics(q_temp,qdot_temp);

//      lfoot_float_init_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)(0));
//      rfoot_float_init_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)(1));
//      com_float_init_ = model_.getCurrentCom();

//      lfoot_float_init_ = pre_lfoot_trajectory_float_;
//      rfoot_float_init_ = pre_rfoot_trajectory_float_;

//      lfoot_float_euler_init_ = DyrosMath::rot2Euler(lfoot_float_init_.linear());
//      rfoot_float_euler_init_ = DyrosMath::rot2Euler(rfoot_float_init_.linear());

//      pelv_float_init_.setIdentity();

//      Eigen::Isometry3d ref_frame;

//      if(foot_step_(current_step_num_, 6) == 0)  //right foot support
//      {
//        ref_frame = rfoot_float_init_;
//      }
//      else if(foot_step_(current_step_num_, 6) == 1)
//      {
//        ref_frame = lfoot_float_init_;
//      }
//      if(joystick_walking_flag_ ==true){
//          if(current_step_num_<=2){
//              if(foot_step_joy_(current_step_num_, 6) == 0)  //right foot support
//              {
//                ref_frame = rfoot_float_init_;
//              }
//              else if(foot_step_joy_(current_step_num_, 6) == 1)
//              {
//                ref_frame = lfoot_float_init_;
//              }
//          }
//          else{
//              if(foot_step_joy_(2, 6) == 0)  //right foot support
//              {
//                ref_frame = rfoot_float_init_;
//              }
//              else if(foot_step_joy_(2, 6) == 1)
//              {
//                ref_frame = lfoot_float_init_;
//              }
//          }
//          if(joystick_walking_on_ == false){// begin steop sequence
//              double seq = current_step_num_-joystick_stop_begin_num_;

//              if(current_step_num_<=2){
//                  if(foot_step_joy_(current_step_num_, 6) == 0)  //right foot support
//                  {
//                    ref_frame = rfoot_float_init_;
//                  }
//                  else if(foot_step_joy_(current_step_num_, 6) == 1)
//                  {
//                    ref_frame = lfoot_float_init_;
//                  }
//              }
//              else{
//                  if(foot_step_joy_(2+ seq, 6) == 0)  //right foot support
//                  {
//                    ref_frame = rfoot_float_init_;
//                  }
//                  else if(foot_step_joy_(2+ seq, 6) == 1)
//                  {
//                    ref_frame = lfoot_float_init_;
//                  }
//              }
//          }
//      }

//      pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame);
//      com_support_init_ = pelv_support_init_.linear()*com_float_init_ + pelv_support_init_.translation();

//      zc_ = com_support_init_(2);

//  }
  else if(current_step_num_!=0 && walking_tick_ == t_start_)
  {
      Eigen::Matrix<double, DyrosJetModel::MODEL_WITH_VIRTUAL_DOF, 1> q_temp, qdot_temp;
        q_temp.setZero();
        q_temp.segment<28>(6) = current_q_.segment<28>(0);
        qdot_temp.segment<28>(6) = current_q_dot_.segment<28>(0);
        if(walking_tick_ > 0)
        {
          q_temp.segment<12>(6) =   desired_q_not_compensated_.segment<12>(0);
//            q_temp.segment<28>(6) =   desired_q_not_compensated_.segment<28>(0);
        }
       // cout<<"q temp in coumpute of walking ctrl:"<<endl<<q_temp<<endl;
        model_.updateKinematics(q_temp,qdot_temp);


    q_init_ = current_q_;
    lfoot_float_init_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)(0));
    rfoot_float_init_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)(1));
    com_float_init_ = model_.getCurrentCom();

//    lfoot_float_init_ = pre_lfoot_trajectory_float_;
//    rfoot_float_init_ = pre_rfoot_trajectory_float_;

    lfoot_float_euler_init_ = DyrosMath::rot2Euler(lfoot_float_init_.linear());
    rfoot_float_euler_init_ = DyrosMath::rot2Euler(rfoot_float_init_.linear());


//    cout<<"walking tick : "<<walking_tick_<<", current step num : "<<current_step_num_<<endl;
//    cout<<"lfoot init from moel : " <<lfoot_float_init_.translation()<<endl<<"link transform : "<<endl<<link_transform_[LF_LINK+5].translation()<<endl;

    pelv_float_init_.setIdentity();

    Eigen::Isometry3d ref_frame;

    Eigen::Vector3d ankle_to_heel;
    ankle_to_heel(0) = -0.15; ankle_to_heel(1) = 0; ankle_to_heel(2) = -0.11;

    Eigen::Isometry3d lfoot_plane_temp, rfoot_plane_temp;
    lfoot_plane_temp.translation() = lfoot_float_current_.translation() + lfoot_float_current_.linear()*ankle_to_heel - ankle_to_heel;
    rfoot_plane_temp.translation() = rfoot_float_current_.translation() + rfoot_float_current_.linear()*ankle_to_heel - ankle_to_heel;
    lfoot_plane_temp.linear().setIdentity();
    rfoot_plane_temp.linear().setIdentity();

    if(foot_step_(current_step_num_, 6) == 0)  //right foot support
    {
      ref_frame = rfoot_float_init_;
//        ref_frame = rfoot_plane_temp;
//        ref_frame.translation()(2) = lfoot_float_init_.translation()(2);
      ref_frame.linear().setIdentity();
//      cout<<"right foot support "<<endl;

    }
    else if(foot_step_(current_step_num_, 6) == 1)
    {
//        cout<<"left foot support "<<endl;

      ref_frame = lfoot_float_init_;
      ref_frame.linear().setIdentity();
//        ref_frame = lfoot_plane_temp;
//        ref_frame.translation()(2) = rfoot_float_init_.translation()(2);
    }
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

    lfoot_DSP1_support_init_ = lfoot_support_init_;
    rfoot_DSP1_support_init_ = rfoot_support_init_;

//    zc_ = com_support_init_(2);
//    com_support_init_(2) = com_height_;
//    pelv_support_init_.translation()(2) = com_height_;
//    cout<<"walkingt tick : "<< walking_tick_<<" com height init : "<<com_support_init_(2)<<", pelv support init height : "<<pelv_support_init_.translation()(2)<<endl;

//    zc_ = com_support_init_(2);
    if(com_height_ != 0.75){
      com_support_init_(2) = com_height_;
      pelv_support_init_.translation()(2) = com_height_;
    }
    if(current_step_num_>=2){
        lfoot_support_init_2_ = lfoot_support_init_;
        rfoot_support_init_2_ = rfoot_support_init_;
        pelv_support_init_2_ = pelv_support_init_;
    }

//    cout<<walking_tick_<<":  com support init : "<<com_support_init_(2)<<", pelv support init : "<<pelv_support_init_.translation()(2)<<"\t"<<"lfoot float height  : "<<lfoot_float_init_.translation()(2)<<"\t"<<"rfoot float height  : "<<rfoot_float_init_.translation()(2)<<endl;


    pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear());
    rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
    lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());


//    cout<<walking_tick_<<"  lfoot euler init : "<<lfoot_support_euler_init_(0)*RAD2DEG<<"\t"<<lfoot_support_euler_init_(1)*RAD2DEG<<"\t"<<lfoot_support_euler_init_(2)*RAD2DEG<<endl;
//    cout<<walking_tick_<<"  rfoot euler init : "<<rfoot_support_euler_init_(0)*RAD2DEG<<"\t"<<rfoot_support_euler_init_(1)*RAD2DEG<<"\t"<<rfoot_support_euler_init_(2)*RAD2DEG<<endl;
//    cout<<walking_tick_<<" lfoot euler init float : "<<lfoot_float_euler_init_(0)*RAD2DEG<<"\t"<<lfoot_float_euler_init_(1)*RAD2DEG<<"\t"<<lfoot_float_euler_init_(2)*RAD2DEG<<endl;
//    cout<<walking_tick_<<" rfoot euler init float : "<<rfoot_float_euler_init_(0)*RAD2DEG<<"\t"<<rfoot_float_euler_init_(1)*RAD2DEG<<"\t"<<rfoot_float_euler_init_(2)*RAD2DEG<<endl;

//    cout<<"l foot support init : "<<lfoot_support_init_.translation()(0)<<"\t"<<lfoot_support_init_.translation()(1)<<"\t"<<lfoot_support_init_.translation()(2)<<endl;
//    cout<<"r foot support init : "<<rfoot_support_init_.translation()(0)<<"\t"<<rfoot_support_init_.translation()(1)<<"\t"<<rfoot_support_init_.translation()(2)<<endl;
//    cout<<"l foot float init : "<<lfoot_float_init_.translation()(0)<<"\t"<<lfoot_float_init_.translation()(1)<<"\t"<<lfoot_float_init_.translation()(2)<<endl;
//    cout<<"r foot float init : "<<rfoot_float_init_.translation()(0)<<"\t"<<rfoot_float_init_.translation()(1)<<"\t"<<rfoot_float_init_.translation()(2)<<endl;
//    zc_ = com_support_init_(2);

//    updateInitialStatefromWaist();

  }
}

void WalkingController::updateNextStepTime()
{
  if(walking_tick_ == t_last_)
  {

    if(current_step_num_ != total_step_num_-1)
    {
      t_start_ = t_last_ +1;
      t_start_real_ = t_start_ + t_rest_init_;
      t_total_ = t_total_init_;
      t_double1_ = t_double1_init_;

      t_last_ = t_start_ + t_total_ -1;

      current_step_num_ ++;
      swing_foot_flag_ = false;

    }    
  }


  if(current_step_num_ == total_step_num_ -1 && walking_tick_ >= t_last_){
      if(joystick_walking_flag_ == true){
          joystick_planning_ = false;

          if(walking_tick_ == t_last_){
              cout<<"joystick planning finished "<<endl;
//              joystick_walking_flag_ = false;

          }
      }
  }
  if(current_step_num_ == total_step_num_-1 && walking_tick_ >= t_last_ +4.0*hz_)


  {
    walking_state_send = true;
    walking_end_ = !walking_end_;

  }


  if(current_step_num_ == total_step_num_-1 && walking_tick_ >= t_last_ +4.0*hz_+1)
  {
//    if(joystick_walking_flag_ == false)
       walking_state_send = false;

  }

  if(current_step_num_ == total_step_num_-1 && walking_tick_ >= t_last_ +5.0*hz_)
  {

//     if(joystick_walking_flag_ == false){
        walking_enable_ = false;
//     }

  }

  walking_tick_ ++;

}

void WalkingController::addZmpOffset()
{
//  lfoot_zmp_offset_ = -0.04;
//  rfoot_zmp_offset_ = 0.04;

    lfoot_zmp_offset_ = 0.02;
    rfoot_zmp_offset_ = -0.02;

//  lfoot_zmp_offset_ = -0.01;
//  rfoot_zmp_offset_ = 0.01;

  foot_step_support_frame_offset_ = foot_step_support_frame_;


  if(foot_step_(0,6) == 0) //right support foot
  {
    //  supportfoot_support_init_offset_(0) = supportfoot_support_init_(0)/2;
    supportfoot_support_init_offset_(1) = supportfoot_support_init_(1) + rfoot_zmp_offset_;
    swingfoot_support_init_offset_(1) = swingfoot_support_init_(1) + lfoot_zmp_offset_;
  }
  else
  {
    //  supportfoot_support_init_offset_(0) = supportfoot_support_init_(0);
    supportfoot_support_init_offset_(1) = supportfoot_support_init_(1) + lfoot_zmp_offset_;
    swingfoot_support_init_offset_(1) = swingfoot_support_init_(1) + rfoot_zmp_offset_;
  }

  for(int i=0; i<total_step_num_; i++)
  {
    if(foot_step_(i,6) == 0)//right support, left swing
    {
      foot_step_support_frame_offset_(i,1) += lfoot_zmp_offset_;
    }
    else
    {
      foot_step_support_frame_offset_(i,1) += rfoot_zmp_offset_;
    }
  }
//    if(walking_tick_ == t_start_){
//        cout<<"foot step support frame of "<<current_step_num_<<", -1 "<<foot_step_support_frame_offset_(current_step_num_-1,1)<<", "<<foot_step_support_frame_offset_(current_step_num_,1)<<endl;
//    }

}

void WalkingController::zmpGenerator(const unsigned int norm_size, const unsigned planning_step_num)
{

    Eigen::MatrixXd ref_zmp1;
  ref_zmp_.resize(norm_size, 2);
  ref_zmp1.resize(norm_size,2);
  com_offset_.setZero();

  Eigen::VectorXd temp_px, temp_px1;
  Eigen::VectorXd temp_py;

  unsigned int index =0;


  if(current_step_num_ ==0)
  {
//      if(walking_tick_ ==0)
//          cout<<"com init on at 0 : "<<com_support_init_(1)+com_offset_(1)<<endl;
    for (int i=0; i<= t_temp_; i++) //200 tick
    {
      if(i <= 0.5*hz_)
      {
        ref_zmp_(i,0) = com_support_init_(0)+com_offset_(0);
        ref_zmp_(i,1) = com_support_init_(1)+com_offset_(1);
      }
      else if(i < 1.5*hz_)
      {
        double del_x = i-0.5*hz_;
        ref_zmp_(i,0) = com_support_init_(0)+com_offset_(0)-del_x*(com_support_init_(0)+com_offset_(0))/(1.0*hz_);
        ref_zmp_(i,1) = com_support_init_(1)+com_offset_(1);
      }
      else
      {
        ref_zmp_(i,0) = 0.0;
        ref_zmp_(i,1) = com_support_init_(1)+com_offset_(1);
      }

      ref_zmp1 = ref_zmp_;

      index++;
    }
  }


  if(current_step_num_ >= total_step_num_-planning_step_num)
  {
    for(unsigned int i = current_step_num_; i<total_step_num_ ; i++)
    {
     // zmpPattern(i,temp_px1,temp_py);
      onestepZmp(i,temp_px,temp_py);
     // onestepZmp_modified(i,temp_px,temp_py);

      for (unsigned int j=0; j<t_total_; j++)
      {
        ref_zmp_(index+j,0) = temp_px(j);
        ref_zmp_(index+j,1) = temp_py(j);
       // ref_zmp_(index+j,0) = temp_px1(j);
      }
      index = index+t_total_;
    }

    for (unsigned int j=0; j<20*hz_; j++)
    {
      ref_zmp_(index+j,0) = ref_zmp_(index-1,0);
      ref_zmp_(index+j,1) = ref_zmp_(index-1,1);
      //ref_zmp_(index+j,0) = ref_zmp1(index-1,0);
    }
    index = index+20*hz_;
  }
  else
  {
    for(unsigned int i=current_step_num_; i < current_step_num_+planning_step_num; i++)
    {
    //  zmpPattern(i,temp_px1,temp_py);
      onestepZmp(i,temp_px,temp_py);
      //onestepZmp_modified(i,temp_px,temp_py);
      for (unsigned int j=0; j<t_total_; j++)
      {
        ref_zmp_(index+j,0) = temp_px(j);
        ref_zmp_(index+j,1) = temp_py(j);
        //ref_zmp_(index+j,0) = temp_px1(j);
      }
      index = index+t_total_;
    }
  }
  if(walking_tick_ == 0 || walking_tick_==t_start_){
//      cout<<"in walking controoler : "<<foot_step_support_frame_offset_(current_step_num_-1,1)<<endl;
      file[35]<<walking_tick_;
      file[22]<<walking_tick_;
      for(int i=0;i<norm_size;i++){
          file[35]<<"\t"<<ref_zmp_(i,1);
          file[22]<<"\t"<<ref_zmp_(i,0);
      }
      file[22]<<endl;
      file[35]<<endl;
  }
}

void WalkingController::onestepZmp(unsigned int current_step_number, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py)
{
  temp_px.resize(t_total_);
  temp_py.resize(t_total_);
  temp_px.setZero();
  temp_py.setZero();

  double Kx = 0.0;
  double Kx2 = 0.0;
  double Ky = 0.0;
  double Ky2 = 0.0;


  if(current_step_number == 0)
  {
    Kx = supportfoot_support_init_offset_(0);
    Kx2 = (foot_step_support_frame_(current_step_number,0)- supportfoot_support_init_offset_(0))/2.0;

    //    Kx2 = (foot_step_support_frame_(current_step_number,0))/2.0;

    Ky = supportfoot_support_init_offset_(1) - com_support_init_(1);
    Ky2 = (foot_step_support_frame_(current_step_number,1)- supportfoot_support_init_offset_(1))/2.0;

    //    Ky2 = (foot_step_support_frame_(current_step_number,1))/2.0;

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
  }
  else if(current_step_number == 1)
  {
    Kx = foot_step_support_frame_offset_(current_step_number-1,0) - (foot_step_support_frame_(current_step_number-1,0) + supportfoot_support_init_(0))/2.0;
    Kx2 = (foot_step_support_frame_(current_step_number,0)+foot_step_support_frame_(current_step_number-1,0))/2.0 - foot_step_support_frame_offset_(current_step_number-1,0);

    Ky =  foot_step_support_frame_offset_(current_step_number-1,1) - (foot_step_support_frame_(current_step_number-1,1) + supportfoot_support_init_(1))/2.0;
    Ky2 = (foot_step_support_frame_(current_step_number,1)+foot_step_support_frame_(current_step_number-1,1))/2.0 - foot_step_support_frame_offset_(current_step_number-1,1);
    for(int i=0; i<t_total_; i++)
    {
      if(i < t_rest_init_)
      {
        temp_px(i) = (foot_step_support_frame_(current_step_number-1,0)+supportfoot_support_init_(0))/2.0;
        temp_py(i) = (foot_step_support_frame_(current_step_number-1,1)+supportfoot_support_init_(1))/2.0;

      }
      else if(i >= t_rest_init_ && i < t_rest_init_+t_double1_)
      {
        temp_px(i) = (foot_step_support_frame_(current_step_number-1,0)+supportfoot_support_init_(0))/2.0 + Kx/t_double1_*(i+1-t_rest_init_);
        temp_py(i) = (foot_step_support_frame_(current_step_number-1,1)+supportfoot_support_init_(1))/2.0 + Ky/t_double1_*(i+1-t_rest_init_);
      }
      else if(i>= t_rest_init_+t_double1_ && i< t_total_-t_rest_last_-t_double2_)
      {
        temp_px(i) = foot_step_support_frame_offset_(current_step_number-1,0);
        temp_py(i) = foot_step_support_frame_offset_(current_step_number-1,1);
      }
      else if(i >= t_total_-t_rest_last_-t_double2_ && i< t_total_-t_rest_last_)
      {
        temp_px(i) = foot_step_support_frame_offset_(current_step_number-1,0) + Kx2/(t_double2_)*(i+1-(t_total_-t_double2_-t_rest_last_));
        temp_py(i) = foot_step_support_frame_offset_(current_step_number-1,1) + Ky2/(t_double2_)*(i+1-(t_total_-t_double2_-t_rest_last_));
      }
      else
      {
        temp_px(i) = temp_px(i-1);
        temp_py(i) = temp_py(i-1);
      }
    }
  }
  else
  {
    Kx = foot_step_support_frame_offset_(current_step_number-1,0) - (foot_step_support_frame_(current_step_number-1,0) + foot_step_support_frame_(current_step_number-2,0))/2.0;
    Kx2 = (foot_step_support_frame_(current_step_number,0)+foot_step_support_frame_(current_step_number-1,0))/2.0 - foot_step_support_frame_offset_(current_step_number-1,0);

    Ky =  foot_step_support_frame_offset_(current_step_number-1,1) - (foot_step_support_frame_(current_step_number-1,1) + foot_step_support_frame_(current_step_number-2,1))/2.0;
    Ky2 = (foot_step_support_frame_(current_step_number,1)+foot_step_support_frame_(current_step_number-1,1))/2.0 -  foot_step_support_frame_offset_(current_step_number-1,1);

    for(int i=0; i<t_total_; i++)
    {
      if(i < t_rest_init_)
      {
        temp_px(i) = (foot_step_support_frame_(current_step_number-1,0)+foot_step_support_frame_(current_step_number-2,0))/2.0;
        temp_py(i) = (foot_step_support_frame_(current_step_number-1,1)+foot_step_support_frame_(current_step_number-2,1))/2.0;
      }
      else if(i >= t_rest_init_ && i < t_rest_init_+t_double1_)
      {
        temp_px(i) = (foot_step_support_frame_(current_step_number-1,0)+foot_step_support_frame_(current_step_number-2,0))/2.0 + Kx/t_double1_*(i+1-t_rest_init_);
        temp_py(i) = (foot_step_support_frame_(current_step_number-1,1)+foot_step_support_frame_(current_step_number-2,1))/2.0 + Ky/t_double1_*(i+1-t_rest_init_);
      }
      else if(i>= t_rest_init_+t_double1_ && i< t_total_-t_rest_last_-t_double2_)
      {
        temp_px(i) = foot_step_support_frame_offset_(current_step_number-1,0);
        temp_py(i) = foot_step_support_frame_offset_(current_step_number-1,1);
      }
      else if(i >= t_total_-t_rest_last_-t_double2_ && i< t_total_-t_rest_last_)
      {
        temp_px(i) = foot_step_support_frame_offset_(current_step_number-1,0) + Kx2/(t_double2_)*(i+1-(t_total_-t_double2_-t_rest_last_));
        temp_py(i) = foot_step_support_frame_offset_(current_step_number-1,1) + Ky2/(t_double2_)*(i+1-(t_total_-t_double2_-t_rest_last_));
      }
      else
      {
        temp_px(i) = temp_px(i-1);
        temp_py(i) = temp_py(i-1);
      }
    }
  }
}

void WalkingController::getComTrajectory()
{

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

    temp_rot = DyrosMath::rotateWithZ(-foot_step_support_frame_(current_step_num_,5));
    for(int i=0; i<3; i++)
      temp_pos(i) = foot_step_support_frame_(current_step_num_,i);

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

    //file[26]<<walking_tick_<<"\t"<<xs_(0)<<"\t"<<xs_(1)<<"\t"<<xs_(2)<<endl;

//    com_pos_prev(0) = xs_(0);
    com_pos_prev(1) = ys_(0);
    com_pos = temp_rot*(com_pos_prev - temp_pos);

//    com_vel_prev(0) = xs_(1);
    com_vel_prev(1) = ys_(1);
    com_vel_prev(2) = 0.0;
    com_vel = temp_rot*com_vel_prev;

//    com_acc_prev(0) = xs_(2);
    com_acc_prev(1) = ys_(2);
    com_acc_prev(2) = 0.0;
    com_acc = temp_rot*com_acc_prev;

//    xs_(0) = com_pos(0);
//    xs_(1) = com_vel(0);
    xs_(2) = com_acc(0);

    ys_(0) = com_pos(1);
    ys_(1) = com_vel(1);
    ys_(2) = com_acc(1);



    //file[27]<<walking_tick_<<"\t"<<xs_(0)<<"\t"<<xs_(1)<<"\t"<<xs_(2)<<endl;
  }

//  x_p1_ = xs_;
//  y_p1_ = ys_;
  double start_time;

  if(current_step_num_ == 0)
    start_time = 0;
  else
    start_time = t_start_;

  zmp_desired_(0) = ref_zmp_(walking_tick_-start_time,0);
  zmp_desired_(1) = ref_zmp_(walking_tick_-start_time,1);

  if(com_control_mode_ == true)
  {
//    com_desired_(0) = xd_(0);
    com_desired_(1) = yd_(0);
    com_desired_(2) = DyrosMath::cubic(walking_tick_, t_start_, t_start_real_, pelv_support_init_.translation()(2), pelv_suppprt_start_.translation()(2), 0, 0);

//    com_dot_desired_(0) = xd_(1);
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
//    com_desired_(0) = xd_(0);
    com_desired_(1) = yd_(0);
    com_desired_(2) = DyrosMath::cubic(walking_tick_, t_start_, t_start_real_, pelv_support_init_.translation()(2), pelv_suppprt_start_.translation()(2), 0, 0);

//    com_dot_desired_(0) = xd_(1);
    com_dot_desired_(1) = yd_(1);
    com_dot_desired_(2) = DyrosMath::cubicDot(walking_tick_, t_start_, t_start_real_, pelv_support_init_.translation()(2), pelv_suppprt_start_.translation()(2), 0, 0, hz_);

    double k= 100.0;
    p_ref_(0) = xd_(1)+k*(xd_(0)-com_support_current_(0));
    p_ref_(1) = yd_(1)+k*(yd_(0)-com_support_current_(1));
    p_ref_(2) = k*(com_desired_(2)-com_support_current_(2));
    l_ref_.setZero();
  }

//  com_desired_(0) = com_data_(walking_tick_,4);
//  com_dot_desired_(0) = com_data_(walking_tick_,5);

}

void WalkingController::getPelvTrajectory()
{
  double z_rot = foot_step_support_frame_(current_step_num_,5);

  //Trunk Position
  if(com_control_mode_ == true)
  {
    double kp = 1.0;//2.0;
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
//    pelv_trajectory_support_.translation()(2) = pelv_support_current_.translation()(2) + 1.5*(com_desired_(2) - pelv_support_current_.translation()(2));
//    pelv_trajectory_support_.translation()(2) = pelv_suppprt_start_.translation()(2) + kp*(com_desired_(2)- com_support_current_(2));
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

    //if(walking_tick_ ==0)
    // COM_pd = 0.0;
    //Trunk_trajectory.translation()(0) = _T_Trunk_support.translation()(0)+kp*(_COM_desired(0) - _COM_real_support(0) + 0.06) + d*xd_(1)- d*(_COM_real_support(0)-COM_prev(0))/Hz  ;


    double offset_x = 0.0;
    if(foot_step_(current_step_num_,6) == 1) //right foot swing(left foot support)
    {
      double temp_time = 0.1*hz_;
      if(walking_tick_ < t_start_real_)
        offset_x = DyrosMath::cubic(walking_tick_, t_start_+temp_time,t_start_real_-temp_time,0.0,0.02,0.0,0.0);
      else
        offset_x = DyrosMath::cubic(walking_tick_, t_start_+t_total_-t_rest_last_+temp_time,t_start_+t_total_-temp_time,0.02,0.0,0.0,0.0);
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

    if(foot_step_(current_step_num_,6) == 2)
      Trunk_trajectory_euler(2) = pelv_support_euler_init_(2);
    else
      Trunk_trajectory_euler(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_double2_-t_rest_last_, pelv_support_euler_init_(2),z_rot/2.0,0.0,0.0);
  }
  else
  {
    for(int i=0; i<2; i++)
      Trunk_trajectory_euler(i) = 0.0;

    if(foot_step_(current_step_num_,6) == 2)
      Trunk_trajectory_euler(2) = pelv_support_euler_init_(2);
    else
      Trunk_trajectory_euler(2) = z_rot/2.0;
  }

  pelv_trajectory_support_.linear() = DyrosMath::rotateWithZ(Trunk_trajectory_euler(2))*DyrosMath::rotateWithY(Trunk_trajectory_euler(1))*DyrosMath::rotateWithX(Trunk_trajectory_euler(0));
}

void WalkingController::getFootTrajectory()
{
  Eigen::Vector6d target_swing_foot;

  for(int i=0; i<6; i++)
    target_swing_foot(i) = foot_step_support_frame_(current_step_num_,i);

  if(joystick_walking_flag_==true){
      if(current_step_num_<=2){
          for(int i=0; i<6; i++)
            target_swing_foot(i) = foot_step_joy_support_frame_(current_step_num_,i);
      }
      else {
          for(int i=0; i<6; i++)
            target_swing_foot(i) = foot_step_joy_support_frame_(2,i);
      }

  }

  if(walking_tick_ < t_start_real_+t_double1_)
  {

    if(foot_step_(current_step_num_,6) == 1){// left foot support
        lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
        lfoot_trajectory_dot_support_.setZero();
//        lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,lfoot_support_init_.translation()(2),0.0,0.0,0.0);
        lfoot_trajectory_support_.translation()(2) = 0.0;
        lfoot_trajectory_euler_support_(2) = lfoot_support_euler_init_(2);
        for(int i=0;i<2;i++)
            lfoot_trajectory_euler_support_(i) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,lfoot_support_init_.translation()(i),0.0,0.0,0.0);

        rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
        rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;
        rfoot_trajectory_euler_support_(1) = 0.0;
    }
    else { // right foot support
        rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
//        rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,rfoot_support_init_.translation()(2),0.0,0.0,0.0);
        rfoot_trajectory_support_.translation()(2) = 0.0;
        rfoot_trajectory_euler_support_(2) = rfoot_support_euler_init_(2);
        for(int i=0;i<2;i++)
            rfoot_trajectory_euler_support_(i) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,rfoot_support_init_.translation()(i),0.0,0.0,0.0);

        lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
        lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;
        lfoot_trajectory_euler_support_(1) = 0.0;
    }

    lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
    rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));

  }
  else if(walking_tick_ >= t_start_real_+t_double1_ && walking_tick_ < t_start_+t_total_-t_double2_-t_rest_last_)
  {
    double t_rest_temp = 0.05*hz_;
    double ankle_temp;
    ankle_temp = 0*DEG2RAD;

    if(foot_step_(current_step_num_,6) == 1) //Left foot support : Left foot is fixed at initial values, and Right foot is set to go target position
    {
      lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
      lfoot_trajectory_support_.translation()(2) = 0.0;
      lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;
      lfoot_trajectory_euler_support_.setZero();

      lfoot_trajectory_dot_support_.setZero();
      lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));

      // setting for Left supporting foot

      if(walking_tick_ < t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0) // the period for lifting the right foot
      {
        rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0,foot_height_,0.0,0.0);
        rfoot_trajectory_dot_support_(2) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0,foot_height_,0.0,0.0,hz_);

//        if(ik_mode_ == 2)
//            rfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_start_+t_double1_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,rfoot_lifting_euler_init_(1),0.0,0.0,0.0);
//        else {
            rfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0.0,ankle_temp,0.0,0.0);
//        }
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
    else if(foot_step_(current_step_num_,6) == 0) // Right foot support : Right foot is fixed at initial values, and Left foot is set to go target position
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

//        if(ik_mode_ == 2)
//            lfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_start_+t_double1_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,lfoot_lifting_euler_init_(1),0.0,0.0,0.0);
//        else{
            lfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0.0,ankle_temp,0.0,0.0);
//        }
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
    if(foot_step_(current_step_num_,6) == 1) // left foot support
    {
      lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
      lfoot_trajectory_support_.translation()(2) = 0.0;
      lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;
      lfoot_trajectory_euler_support_(0) = 0.0;
      lfoot_trajectory_euler_support_(1) = 0.0;
//      lfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_double2_-t_rest_last_,t_start_+t_total_-1,lfoot_support_euler_(1),lfoot_support_euler_init_(1),0.0,0.0);
      lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
      lfoot_trajectory_dot_support_.setZero();

      for(int i=0; i<3; i++)
      {
        rfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
//          rfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_double2_-t_rest_last_,t_start_+t_total_-1,rfoot_support_heel_.translation()(i),target_swing_foot(i),0.0,0.0);
        rfoot_trajectory_euler_support_(i) = target_swing_foot(i+3);
      }
      rfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_double2_-t_rest_last_,t_start_+t_total_-1,rfoot_dsp2_support_euler_init_(1),0.0,0.0,0.0);
      rfoot_trajectory_dot_support_.setZero();

      rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
    }
    else if (foot_step_(current_step_num_,6) == 0)
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
//          lfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_double2_-t_rest_last_,t_start_+t_total_-1,lfoot_support_heel_.translation()(i),target_swing_foot(i),0.0,0.0);
        lfoot_trajectory_euler_support_(i) = target_swing_foot(i+3);
      }
      lfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_double2_-t_rest_last_,t_start_+t_total_-1,lfoot_dsp2_support_euler_init_(1),0.0,0.0,0.0);
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
void WalkingController::getAdaptiveFootTrajectory()
{
  Eigen::Vector6d target_swing_foot;

  for(int i=0; i<6; i++)
    target_swing_foot(i) = foot_step_support_frame_(current_step_num_,i);



  double t_rest_temp = 0.05*hz_;
  double ankle_temp;
  ankle_temp = 0*DEG2RAD;

  int swing_time = t_start_+t_total_-t_double2_-t_rest_last_ - (t_start_real_+t_double1_);

  if(walking_tick_ == t_start_real_+t_double1_){
      cout<<" swing duration : "<<swing_time<<endl;
      cout<<"swing time check  tick : "<<walking_tick_<<", tstart "<<t_start_<<"t total "<<t_total_<<"t doubl2 "<<t_double2_<<"t restlast "<<t_rest_last_<<", tstart real "<<t_start_real_<<", t double 1"<<t_double1_<<endl;
  }

  if(walking_tick_ < t_start_real_ + t_double1_init_){
      lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
      lfoot_trajectory_dot_support_.setZero();


      if(foot_step_(current_step_num_,6) == 1) //left foot support
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


      if(foot_step_(current_step_num_,6) == 0) //right foot support
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
  else if(walking_tick_ >= t_start_real_ + t_double1_init_ && walking_tick_ < t_start_ + t_total_ - t_double2_-t_rest_last_){
      if(swing_foot_flag_ == true)
      {
          if(foot_step_(current_step_num_,6) == 1){
    //          cout<<"left foot support : "<<endl;
              lfoot_trajectory_support_.translation() =lfoot_support_init_.translation();
              lfoot_trajectory_euler_support_.setZero();

              lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));

              if(walking_tick_ < t_swing_start_ + swing_time/2.0){
                  rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_swing_start_+t_rest_temp,t_swing_start_ + swing_time/2.0,0,foot_height_,0.0,0.0);
                  rfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_swing_start_ + t_rest_temp,t_swing_start_ + swing_time/2.0,0.0,ankle_temp,0.0,0.0);
              }
              else{
                  rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_swing_start_ + swing_time/2.0,t_swing_start_ + swing_time - t_rest_temp,foot_height_,target_swing_foot(2),0.0,0.0);
                  rfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_swing_start_ + swing_time/2.0,t_swing_start_ + swing_time - t_rest_temp,ankle_temp,0.0,0.0,0.0);
              }

              rfoot_trajectory_euler_support_(0) = DyrosMath::cubic(walking_tick_,t_swing_start_,t_swing_start_+swing_time,0.0, target_swing_foot(0+3),0.0,0.0);
              rfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_,t_swing_start_,t_swing_start_+swing_time,0.0, target_swing_foot(2+3),0.0,0.0);

              for(int i=0;i<2;i++)
              {
                rfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_swing_start_ + 2*t_rest_temp,t_swing_start_ + swing_time -2*t_rest_temp,rfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0);
              }

              rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
          }
          else { // right foot support
    //           cout<<"right foot support "<<endl;
              rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
              rfoot_trajectory_euler_support_.setZero();

              rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));

              if(walking_tick_ < t_swing_start_ + swing_time/2.0){
                  lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_swing_start_+t_rest_temp,t_swing_start_ + swing_time/2.0,0,foot_height_,0.0,0.0);
                  lfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_swing_start_ + t_rest_temp,t_swing_start_ + swing_time/2.0,0.0,ankle_temp,0.0,0.0);
              }
              else{
                  lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_swing_start_ + swing_time/2.0,t_swing_start_ + swing_time - t_rest_temp,foot_height_,target_swing_foot(2),0.0,0.0);
                  lfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_swing_start_ + swing_time/2.0,t_swing_start_ + swing_time - t_rest_temp,ankle_temp,0.0,0.0,0.0);
              }

              lfoot_trajectory_euler_support_(0) = DyrosMath::cubic(walking_tick_,t_swing_start_,t_swing_start_+swing_time,0.0, target_swing_foot(0+3),0.0,0.0);
              lfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_,t_swing_start_,t_swing_start_+swing_time,0.0, target_swing_foot(2+3),0.0,0.0);

              for(int i=0;i<2;i++)
              {
                lfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_swing_start_ + 2*t_rest_temp,t_swing_start_ + swing_time -2*t_rest_temp,lfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0);
              }

              lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));

          }
          if(walking_tick_ == t_swing_start_ + swing_time-1){
              //swing_foot_flag_ = false;
              cout<<"swing foot sequence stop  tick :"<<walking_tick_<<" "<<endl;
          }
      }
      else{
          lfoot_trajectory_support_.translation() =lfoot_support_init_.translation();
          lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;

          lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));

          rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
          rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;

          rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
      }
  }
  else {
      if(foot_step_(current_step_num_,6) == 1)
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
      else if (foot_step_(current_step_num_,6) == 0)
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
void WalkingController::getFootSinTrajectory(){

    Eigen::Vector6d target_swing_foot;

    for(int i=0;i<6;i++)
        target_swing_foot(i) = foot_step_support_frame_(current_step_num_,i);

    if(walking_tick_< t_start_real_ + t_double1_){
        lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
        lfoot_trajectory_dot_support_.setZero();
        lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;

        rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
        rfoot_trajectory_dot_support_.setZero();
        rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;

        if(foot_step_(current_step_num_,6) ==1){// left foot support
            lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,lfoot_support_init_.translation()(2),0.0,0.0,0.0);
            rfoot_trajectory_support_.translation()(2) = 0.0;
        }
        else{
            lfoot_trajectory_support_.translation()(2) = 0.0;
            rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,rfoot_support_init_.translation()(2),0.0,0.0,0.0);
        }
        for(int i=0; i<2; i++){
          lfoot_trajectory_euler_support_(i) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,lfoot_support_euler_init_(i),0.0,0.0,0.0);
          rfoot_trajectory_euler_support_(i) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,rfoot_support_euler_init_(i),0.0,0.0,0.0);
        }

        lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
        rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
    }
    else if(walking_tick_ >= t_start_real_+t_double1_ && walking_tick_ < t_start_+t_total_-t_double2_-t_rest_last_)
    {
        double ellipse_length, ellipse_height, ellipse_angle;
        ellipse_height = 0.05;
        double t_rest_temp = 0.05*hz_;

       ellipse_angle = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+2*t_rest_temp,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-2*t_rest_temp,0.0,180*DEG2RAD,0.0,0.0);


        if(foot_step_(current_step_num_,6) ==1 )//left foot support
        {
            lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
            lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;
            lfoot_trajectory_euler_support_.setZero();

            lfoot_trajectory_dot_support_.setZero();
            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));

            ellipse_length = 0.5*(target_swing_foot(0) - rfoot_support_init_.translation()(0));

            if(current_step_num_ == 0)
            {
                rfoot_trajectory_support_.translation()(0) = ellipse_length-ellipse_length*cos(ellipse_angle);
                rfoot_trajectory_dot_support_(0) = ellipse_length*sin(ellipse_angle);
            }
            else if(current_step_num_ == total_step_num_-1)
            {
                rfoot_trajectory_support_.translation()(0) = -ellipse_length-ellipse_length*cos(ellipse_angle);
                rfoot_trajectory_dot_support_(0) = ellipse_length*sin(ellipse_angle);
            }
            else {
                rfoot_trajectory_support_.translation()(0) = -ellipse_length*cos(ellipse_angle);
                rfoot_trajectory_dot_support_(0) = ellipse_length*sin(ellipse_angle);
            }

            rfoot_trajectory_support_.translation()(2) = ellipse_height*sin(ellipse_angle);
            rfoot_trajectory_dot_support_(2) = ellipse_height*cos(ellipse_angle);

            rfoot_trajectory_support_.translation()(1) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+2*t_rest_temp,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-2*t_rest_temp,rfoot_support_init_.translation()(1),target_swing_foot(1),0.0,0.0);
            rfoot_trajectory_dot_support_(1) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+2*t_rest_temp,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-2*t_rest_temp,rfoot_support_init_.translation()(1),target_swing_foot(1),0.0,0.0,hz_);
            for(int i=0;i<3;i++){
                rfoot_trajectory_euler_support_(i) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,rfoot_support_euler_init_(i),target_swing_foot(i+3),0.0,0.0);
            }


        }
        else //right foot support
        {
            rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
            rfoot_trajectory_support_.translation()(2) = 0.0;
            //rfoot_trajectory_support_.linear() = rfoot_trajectory_init_.linear();
            rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;
            rfoot_trajectory_euler_support_(0) = 0.0;
            rfoot_trajectory_euler_support_(1) = 0.0;
            rfoot_trajectory_dot_support_.setZero();

            ellipse_length = 0.5*(target_swing_foot(0) - lfoot_support_init_.translation()(0));

            if(current_step_num_ == 0)
            {
                lfoot_trajectory_support_.translation()(0) = ellipse_length -ellipse_length*cos(ellipse_angle);
                lfoot_trajectory_dot_support_(0) = ellipse_length*sin(ellipse_angle);
            }
            else if(current_step_num_ == total_step_num_-1)
            {
                lfoot_trajectory_support_.translation()(0) = -ellipse_length -ellipse_length*cos(ellipse_angle);
                lfoot_trajectory_dot_support_(0) = ellipse_length*sin(ellipse_angle);
            }
            else {
                lfoot_trajectory_support_.translation()(0) = -ellipse_length*cos(ellipse_angle);
                lfoot_trajectory_dot_support_(0) = ellipse_length*sin(ellipse_angle);
            }
            lfoot_trajectory_support_.translation()(2) = ellipse_height*sin(ellipse_angle);
            lfoot_trajectory_dot_support_(2) = ellipse_height*cos(ellipse_angle);

            lfoot_trajectory_support_.translation()(1) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+2*t_rest_temp,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-2*t_rest_temp,lfoot_support_init_.translation()(1),target_swing_foot(1),0.0,0.0);
            lfoot_trajectory_dot_support_(1) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+2*t_rest_temp,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-2*t_rest_temp,lfoot_support_init_.translation()(1),target_swing_foot(1),0.0,0.0,hz_);

            for(int i=0;i<3;i++){
                lfoot_trajectory_euler_support_(i) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,lfoot_support_euler_init_(i),target_swing_foot(i+3),0.0,0.0);
            }
        }
        rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
        lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));

    }
    else{
        if(foot_step_(current_step_num_,6) == 1)
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
        else if (foot_step_(current_step_num_,6) == 0)
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
void WalkingController::getFootTrajectory2()
{
  Eigen::Vector6d target_swing_foot;


  for(int i=0; i<6; i++)
    target_swing_foot(i) = foot_step_support_frame_(current_step_num_,i);


  if(walking_tick_ < t_start_real_+t_double1_)
  {
    lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
    lfoot_trajectory_dot_support_.setZero();


    if(foot_step_(current_step_num_,6) == 1) //left foot support
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


    if(foot_step_(current_step_num_,6) == 0) //right foot support
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
    double swing_start = t_start_real_+t_double1_;
    double swing_end = t_start_+t_total_-t_double2_-t_rest_last_;
    double swing_period = swing_end - swing_start;

    double ankle_temp;
    ankle_temp = 0*DEG2RAD;

    if(foot_step_(current_step_num_,6) == 1) //Left foot support : Left foot is fixed at initial values, and Right foot is set to go target position
    {
      lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
      lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;
      lfoot_trajectory_euler_support_.setZero();

      lfoot_trajectory_dot_support_.setZero();
      lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));

      // setting for Left supporting foot

      //if(walking_tick_ < t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0) // the period for lifting the right foot
      if(walking_tick_ < swing_start + 0.5*swing_period)
      {
//        rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0,foot_height_,0.0,0.0);
//        rfoot_trajectory_dot_support_(2) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0,foot_height_,0.0,0.0,hz_);

//        rfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0.0,ankle_temp,0.0,0.0);
//        rfoot_trajectory_dot_support_(4) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0.0,ankle_temp,0.0,0.0,hz_);
          rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_, swing_start, swing_start + 0.5*swing_period,0,foot_height_, 0.0, 0.0);
          rfoot_trajectory_dot_support_(2) = DyrosMath::cubicDot(walking_tick_,swing_start, swing_start+0.5*swing_period,0,foot_height_,0.0,0.0,hz_);

          rfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,swing_start,0.5*swing_period,0.0,ankle_temp,0.0,0.0);
          rfoot_trajectory_dot_support_(4) = DyrosMath::cubicDot(walking_tick_, swing_start,0.5*swing_period,0.0, ankle_temp,0.0,0.0,hz_);
      } // the period for lifting the right foot
      else
      {
//        rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-t_rest_temp,foot_height_,target_swing_foot(2),0.0,0.0);
//        rfoot_trajectory_dot_support_(2) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-t_rest_temp,foot_height_,target_swing_foot(2),0.0,0.0,hz_);

//        rfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_-t_rest_temp,t_start_+t_total_-t_rest_last_,ankle_temp,0.0,0.0,0.0);
//        rfoot_trajectory_dot_support_(4) = DyrosMath::cubicDot(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_-t_rest_temp,t_start_+t_total_-t_rest_last_,ankle_temp,0.0,0.0,0.0,hz_);
          rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,swing_start + 0.5*swing_period,swing_end,foot_height_,target_swing_foot(2),0.0,0.0);
          rfoot_trajectory_dot_support_(2) = DyrosMath::cubicDot(walking_tick_,swing_start + 0.5*swing_period, swing_end,foot_height_,target_swing_foot(2),0.0,0.0,hz_);

          rfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,swing_start + 0.5*swing_period, swing_end,ankle_temp,0.0,0.0,0.0);
          rfoot_trajectory_dot_support_(4) = DyrosMath::cubicDot(walking_tick_,swing_start + 0.5*swing_period, swing_end,ankle_temp,0.0,0.0,0.0,hz_);
      } // the period for putting the right foot

//      rfoot_trajectory_euler_support_(0) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,0.0,target_swing_foot(0+3),0.0,0.0);
//      rfoot_trajectory_dot_support_(3) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,0.0,target_swing_foot(0+3),0.0,0.0,hz_);
        rfoot_trajectory_euler_support_(0) = DyrosMath::cubic(walking_tick_,swing_start,swing_end,0.0,target_swing_foot(0+3),0.0,0.0);
        rfoot_trajectory_dot_support_(3) = DyrosMath::cubicDot(walking_tick_,swing_start,swing_end,0.0,target_swing_foot(0+3),0.0,0.0,hz_);

      for(int i=0; i<2; i++)
      {
//        rfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+2*t_rest_temp,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-2*t_rest_temp,rfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0);
//        //rfoot_trajectory_dot_support_(i) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+2*t_rest_temp,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-2*t_rest_temp,rfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0,hz_);
//        rfoot_trajectory_dot_support_(i) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_,rfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0,hz_);

          rfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,swing_start,swing_end,rfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0);
          rfoot_trajectory_dot_support_(i) = DyrosMath::cubicDot(walking_tick_,swing_start,swing_end,rfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0,hz_);
      }

//      rfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,rfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0);
//      rfoot_trajectory_dot_support_(5) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,rfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0,hz_);
      rfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_,swing_start,swing_end,rfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0);
      rfoot_trajectory_dot_support_(5) = DyrosMath::cubicDot(walking_tick_,swing_start,swing_end,rfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0,hz_);

      rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
    }
    else if(foot_step_(current_step_num_,6) == 0) // Right foot support : Right foot is fixed at initial values, and Left foot is set to go target position
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

      //if(walking_tick_ < t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0)
      if(walking_tick_ < swing_start + 0.5*swing_period)
      {

//        lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0,foot_height_,0.0,0.0);
//        lfoot_trajectory_dot_support_(2) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0,foot_height_,0.0,0.0,hz_);

//        lfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0.0,ankle_temp,0.0,0.0);
//        lfoot_trajectory_dot_support_(4) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0.0,ankle_temp,0.0,0.0,hz_);
          lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,swing_start,swing_start + 0.5*swing_period,0,foot_height_,0.0,0.0);
          lfoot_trajectory_dot_support_(2) = DyrosMath::cubicDot(walking_tick_,swing_start,swing_start + 0.5*swing_period,0,foot_height_,0.0,0.0,hz_);

//          lfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0.0,ankle_temp,0.0,0.0);
//          lfoot_trajectory_dot_support_(4) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+t_rest_temp,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0.0,ankle_temp,0.0,0.0,hz_);

          lfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,swing_start, swing_start + 0.5*swing_period,0.0,ankle_temp,0.0,0.0);
          lfoot_trajectory_dot_support_(4) = DyrosMath::cubicDot(walking_tick_,swing_start, swing_start + 0.5*swing_period,0.0,ankle_temp,0.0,0.0,hz_);

      }
      else
      {
//        lfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_-t_rest_temp,t_start_+t_total_-t_rest_last_,ankle_temp,0.0,0.0,0.0);
//        lfoot_trajectory_dot_support_(4) = DyrosMath::cubicDot(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_-t_rest_temp,t_start_+t_total_-t_rest_last_,ankle_temp,0.0,0.0,0.0,hz_);
          lfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,swing_start + 0.5*swing_period,swing_end,ankle_temp,0.0,0.0,0.0);
          lfoot_trajectory_dot_support_(4) = DyrosMath::cubicDot(walking_tick_,swing_start + 0.5*swing_period,swing_end,ankle_temp,0.0,0.0,0.0,hz_);


//        lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-t_rest_temp,foot_height_,target_swing_foot(2),0.0,0.0);
//        lfoot_trajectory_dot_support_(2) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-t_rest_temp,foot_height_,target_swing_foot(2),0.0,0.0,hz_);

          lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_, swing_start + 0.5*swing_period, swing_end, foot_height_,target_swing_foot(2),0.0,0.0);
          lfoot_trajectory_dot_support_(2) = DyrosMath::cubicDot(walking_tick_,swing_start + 0.5*swing_period, swing_end,foot_height_,target_swing_foot(2),0.0,0.0,hz_);
      }

//      lfoot_trajectory_euler_support_(0) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,0.0,target_swing_foot(0+3),0.0,0.0);
//      lfoot_trajectory_dot_support_(0+3) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,0.0,target_swing_foot(0+3),0.0,0.0,hz_);
      lfoot_trajectory_euler_support_(0) = DyrosMath::cubic(walking_tick_,swing_start,swing_end,0.0,target_swing_foot(0+3),0.0,0.0);
      lfoot_trajectory_dot_support_(0+3) = DyrosMath::cubicDot(walking_tick_,swing_start,swing_end,0.0,target_swing_foot(0+3),0.0,0.0,hz_);

      for(int i=0; i<2; i++)
      {
//        lfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+2*t_rest_temp,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-2*t_rest_temp,lfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0);
//        lfoot_trajectory_dot_support_(i) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+2*t_rest_temp,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_-2*t_rest_temp,lfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0,hz_);
          lfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,swing_start,swing_end,lfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0);
          lfoot_trajectory_dot_support_(i) = DyrosMath::cubicDot(walking_tick_,swing_start,swing_end,lfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0,hz_);
      }

      //  for(int i=0; i<3; i++)
      //  {
      //      lfoot_trajectory_euler_support_(i) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,_T_Start+t_total_-t_rest_last_-t_double2_-t_imp_,0.0,0.0,target_swing_foot(i+3),0.0);
      //      lfoot_trajectory_dot_support_(i+3) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_,_T_Start+t_total_-t_rest_last_-t_double2_-t_imp_,0.0,0.0,target_swing_foot(i+3),0.0,hz_);
      //  }


//      lfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,lfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0);
//      lfoot_trajectory_dot_support_(5) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_-t_imp_,lfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0,hz_);

      lfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_,swing_start,swing_end,lfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0);
      lfoot_trajectory_dot_support_(5) = DyrosMath::cubicDot(walking_tick_,swing_start,swing_end,lfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0,hz_);

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
    if(foot_step_(current_step_num_,6) == 1)
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
    else if (foot_step_(current_step_num_,6) == 0)
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
void WalkingController::supportToFloatPattern()
{
  if(gyro_frame_flag_ == true)
  {
    Eigen::Isometry3d reference = pelv_trajectory_float_;
    DyrosMath::floatGyroframe(pelv_trajectory_support_,reference,pelv_trajectory_float_);
    DyrosMath::floatGyroframe(lfoot_trajectory_support_,reference,lfoot_trajectory_float_);
    DyrosMath::floatGyroframe(rfoot_trajectory_support_,reference,rfoot_trajectory_float_);
    lfoot_trajectory_euler_float_ = DyrosMath::rot2Euler(lfoot_trajectory_float_.linear());
    rfoot_trajectory_euler_float_ = DyrosMath::rot2Euler(rfoot_trajectory_float_.linear());
  }
  else
  {
    pelv_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_)*pelv_trajectory_support_;
    lfoot_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_)*lfoot_trajectory_support_;
    rfoot_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_)*rfoot_trajectory_support_;
    lfoot_trajectory_euler_float_ = DyrosMath::rot2Euler(lfoot_trajectory_float_.linear());
    rfoot_trajectory_euler_float_ = DyrosMath::rot2Euler(rfoot_trajectory_float_.linear());
  }
}


void WalkingController::computeIkControl(Eigen::Isometry3d float_trunk_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector12d& desired_leg_q)
{

  Eigen::Vector3d lp, rp, d;
  d.setZero();
  d(2) = -0.095;
  lp = float_lleg_transform.linear().transpose()*(float_trunk_transform.translation()-float_lleg_transform.translation());
  rp = float_rleg_transform.linear().transpose()*(float_trunk_transform.translation()-float_rleg_transform.translation());

  Eigen::Matrix3d trunk_lleg_rotation,trunk_rleg_rotation;
  trunk_lleg_rotation = float_trunk_transform.linear().transpose()*float_lleg_transform.linear();
  trunk_rleg_rotation = float_trunk_transform.linear().transpose()*float_rleg_transform.linear();

  Eigen::Vector3d ld, rd;
  ld.setZero(); rd.setZero();
  ld(0) = 0;
  ld(1) = 0.105;
  ld(2) = -0.1119;
  rd(0) = 0;
  rd(1) = -0.105;
  rd(2) = -0.1119;

  ld = trunk_lleg_rotation.transpose() * ld;
  rd = trunk_rleg_rotation.transpose() * rd;

  Eigen::Vector3d lr, rr;
  lr = lp + ld;
  rr = rp + rd;

  double l_upper = 0.3713; //direct length from hip to knee
  double l_lower = 0.3728; //direct length from knee to ankle

  double offset_hip_pitch = 24.0799945102*DEG2RAD;
  double offset_knee_pitch = 14.8197729791*DEG2RAD;
  double offset_ankle_pitch = 9.2602215311*DEG2RAD;

  //////////////////////////// LEFT LEG INVERSE KINEMATICS ////////////////////////////

  double lc = sqrt(lr(0)*lr(0)+lr(1)*lr(1)+lr(2)*lr(2));
  desired_leg_q(3) = (- acos((l_upper*l_upper + l_lower*l_lower - lc*lc) / (2*l_upper*l_lower))+ M_PI); // - offset_knee_pitch //+ alpha_lower

  double l_ankle_pitch = asin((l_upper*sin(M_PI-desired_leg_q(3)))/lc);
  desired_leg_q(4) = -atan2(lr(0), sqrt(lr(1)*lr(1)+lr(2)*lr(2))) - l_ankle_pitch;// - offset_ankle_pitch ;
  desired_leg_q(5) = atan2(lr(1), lr(2));

  Eigen::Matrix3d r_tl2;
  Eigen::Matrix3d r_l2l3;
  Eigen::Matrix3d r_l3l4;
  Eigen::Matrix3d r_l4l5;

  r_tl2.setZero();
  r_l2l3.setZero();
  r_l3l4.setZero();
  r_l4l5.setZero();

  r_l2l3 = DyrosMath::rotateWithY(desired_leg_q(3));
  r_l3l4 = DyrosMath::rotateWithY(desired_leg_q(4));
  r_l4l5 = DyrosMath::rotateWithX(desired_leg_q(5));

  r_tl2 = trunk_lleg_rotation * r_l4l5.transpose() * r_l3l4.transpose() * r_l2l3.transpose();

  desired_leg_q(1) = asin(r_tl2(2,1));

  double c_lq5 = -r_tl2(0,1)/cos(desired_leg_q(1));
  if (c_lq5 > 1.0)
  {
    c_lq5 =1.0;
  }
  else if (c_lq5 < -1.0)
  {
    c_lq5 = -1.0;
  }

  desired_leg_q(0) = -asin(c_lq5);
  desired_leg_q(2) = -asin(r_tl2(2,0)/cos(desired_leg_q(1))) + offset_hip_pitch;
  desired_leg_q(3) = desired_leg_q(3)- offset_knee_pitch;
  desired_leg_q(4) = desired_leg_q(4)- offset_ankle_pitch;

  //////////////////////////// RIGHT LEG INVERSE KINEMATICS ////////////////////////////

  double rc = sqrt(rr(0)*rr(0)+rr(1)*rr(1)+rr(2)*rr(2));
  desired_leg_q(9) = (-acos((l_upper*l_upper + l_lower*l_lower - rc*rc) / (2*l_upper*l_lower))+ M_PI); // - offset_knee_pitch //+ alpha_lower

  double r_ankle_pitch = asin((l_upper*sin(M_PI-desired_leg_q(9)))/rc);
  desired_leg_q(10) = -atan2(rr(0), sqrt(rr(1)*rr(1)+rr(2)*rr(2)))-r_ankle_pitch;
  desired_leg_q(11) = atan2(rr(1),rr(2));

  Eigen::Matrix3d r_tr2;
  Eigen::Matrix3d r_r2r3;
  Eigen::Matrix3d r_r3r4;
  Eigen::Matrix3d r_r4r5;

  r_tr2.setZero();
  r_r2r3.setZero();
  r_r3r4.setZero();
  r_r4r5.setZero();

  r_r2r3 = DyrosMath::rotateWithY(desired_leg_q(9));
  r_r3r4 = DyrosMath::rotateWithY(desired_leg_q(10));
  r_r4r5 = DyrosMath::rotateWithX(desired_leg_q(11));

  r_tr2 = trunk_rleg_rotation * r_r4r5.transpose() * r_r3r4.transpose() * r_r2r3.transpose();

  desired_leg_q(7) = asin(r_tr2(2,1));

  double c_rq5 = -r_tr2(0,1)/cos(desired_leg_q(7));
  if (c_rq5 > 1.0)
  {
    c_rq5 =1.0;
  }
  else if (c_rq5 < -1.0)
  {
    c_rq5 = -1.0;
  }

  desired_leg_q(6) = -asin(c_rq5);
  desired_leg_q(8) = asin(r_tr2(2,0)/cos(desired_leg_q(7))) - offset_hip_pitch;
  desired_leg_q(9) = -desired_leg_q(9) + offset_knee_pitch;
  desired_leg_q(10) = -desired_leg_q(10) + offset_ankle_pitch;

  //desired_leg_q(8) = desired_leg_q(8);  //motor axis direction
  //desired_leg_q(9) = desired_leg_q(9);
  //desired_leg_q(10) =desired_leg_q(10);

}


void WalkingController::computeJacobianControl(Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector3d float_lleg_transform_euler, Eigen::Vector3d float_rleg_transform_euler, Eigen::Vector12d& desired_leg_q_dot)
{
    if(walking_tick_ ==0)
           cout<<endl<<" !!!!  jacobian ctrl   !!!!"<<endl;


     Eigen::Matrix6d jacobian_temp_l, jacobian_temp_r, current_leg_jacobian_l_inv, current_leg_jacobian_r_inv,
         J_damped, I_matrix;
     double wl, wr, w0, lambda, a;
     w0 = 0.001;
     lambda = 0.05;
     jacobian_temp_l=current_leg_jacobian_l_*current_leg_jacobian_l_.transpose();
     jacobian_temp_r=current_leg_jacobian_r_*current_leg_jacobian_r_.transpose();
     wr = sqrt(jacobian_temp_r.determinant());
     wl = sqrt(jacobian_temp_l.determinant());

     if (wr<=w0)
     { //Right Jacobi
       a = lambda * pow(1-wr/w0,2);
       J_damped = current_leg_jacobian_r_.transpose()*current_leg_jacobian_r_+a*Eigen::Matrix6d::Identity();
       J_damped = J_damped.inverse();

       cout << "Singularity Region of right leg: " << wr << endl;
       current_leg_jacobian_r_inv = J_damped*current_leg_jacobian_r_.transpose();
     }
     else
     {
       //current_leg_jacobian_r_inv = DyrosMath::pinv(current_leg_jacobian_r_);
       current_leg_jacobian_r_inv = (current_leg_jacobian_r_.transpose()*current_leg_jacobian_r_).inverse()*current_leg_jacobian_r_.transpose();
     }

     if (wl<=w0)
     {
       a = lambda*pow(1-wl/w0,2);
       J_damped = current_leg_jacobian_l_.transpose()*current_leg_jacobian_l_+a*Eigen::Matrix6d::Identity();
       J_damped = J_damped.inverse();

       cout << "Singularity Region of right leg: " << wr << endl;
       current_leg_jacobian_l_inv = J_damped*current_leg_jacobian_l_.transpose();
     }
     else
     {
       current_leg_jacobian_l_inv = (current_leg_jacobian_l_.transpose()*current_leg_jacobian_l_).inverse()*current_leg_jacobian_l_.transpose();
       //current_leg_jacobian_l_inv = DyrosMath::pinv(current_leg_jacobian_r_);
     }
     current_leg_jacobian_l_inv_ = current_leg_jacobian_l_inv;
     current_leg_jacobian_r_inv_ = current_leg_jacobian_r_inv;


//     current_leg_jacobian_l_inv = current_leg_jacobian_l_.inverse();
//     current_leg_jacobian_r_inv = current_leg_jacobian_r_.inverse();



     Eigen::Vector6d q_lfoot_dot,q_rfoot_dot;
     q_lfoot_dot=current_leg_jacobian_l_inv*(lp_+5*lp_clik_);
     q_rfoot_dot=current_leg_jacobian_r_inv*(rp_+5*rp_clik_);

     q_lfoot_dot = current_left_heel_jacobian_.inverse()*(lheel_p_ + 5*lheel_clik_);
     q_rfoot_dot = current_right_heel_jacobian_.inverse()*(rheel_p_ + 5*rheel_clik_);

//     q_lfoot_dot = current_left_toe_jacobian_.inverse() * (ltoe_p_ + 5*ltoe_clik_);
//     q_rfoot_dot = current_right_toe_jacobian_.inverse() * (rtoe_p_ + 5*rtoe_clik_);

     for (int i=0; i<6; i++)
     {
       desired_leg_q_dot(i+6) = q_rfoot_dot(i);
       desired_leg_q_dot(i) = q_lfoot_dot(i);
     }
}

void WalkingController::modifiedPreviewControl()
{
  /////reference: http://www.tandfonline.com/doi/pdf/10.1080/0020718508961156?needAccess=true/////////////

  if(walking_tick_==0)
  {
    previewControlParameter(1.0/hz_, 16*hz_/10, k_ ,com_support_init_, gi_, gp_l_, gx_, a_, b_, c_);//16*hz_/10
  }

  if(current_step_num_ == 0)
    zmp_start_time_ = 0.0;
  else
    zmp_start_time_ = t_start_;

  ux_1_ = 0.0;
  uy_1_ = 0.0;

  previewControl(1.0/hz_, 16*hz_/10, walking_tick_-zmp_start_time_, xi_, yi_, xs_, ys_, ux_1_, uy_1_, ux_, uy_, gi_, gp_l_, gx_, a_, b_, c_, xd_, yd_);
  ///// com at once
  Eigen::Vector3d xd_test, yd_test;
  int zmp_size;
  zmp_size = ref_zmp_.col(1).size();

//  if(walking_tick_ == t_start_){
//      Eigen::Vector3d xs_test, ys_test;
//      file[11]<<walking_tick_;
//      file[12]<<walking_tick_;
//      for(int i=0;i<t_total_;i++){
//          previewControl(1.0/hz_, 16*hz_/10, i, xi_, yi_, xs_test, ys_test, ux_1_, uy_1_, ux_, uy_, gi_, gp_l_, gx_, a_, b_, c_, xd_test, yd_test);
//          file[11]<<"\t"<<xd_test(0);
//          file[12]<<"\t"<<xd_test(1);
//          xs_test = xd_test;
//          ys_test = yd_test;
//      }
//      file[11]<<endl;
//      file[12]<<endl;
//  }


  Eigen::Vector3d xs_matrix, ys_matrix, xs, ys;
  for (int i=0; i<3; i++)
    xs_matrix(i) = xd_(i);
  for (int i=0; i<3; i++)
    ys_matrix(i) = yd_(i);

  //double est_zmp_error_x, est_zmp_error_y, est_zmp;
  //est_zmp_error_x = c_*xs_matrix;
  //est_zmp_error_y = c_*ys_matrix;

//  previewControl(1.0/hz_, 16*hz_/10, walking_tick_-zmp_start_time_, xi_, yi_, xs_, ys_, ux_1_, uy_1_, ux_, uy_, gi_, gp_l_, gx_, a_, b_, c_, xd_, yd_);

//  ux_1_ = ux_;
//  uy_1_ = uy_;

}

void WalkingController::previewControl(
    double dt, int NL, int tick, double x_i, double y_i, Eigen::Vector3d xs, Eigen::Vector3d ys,
    double ux_1, double uy_1 , double& ux, double& uy, double gi, Eigen::VectorXd gp_l,
    Eigen::Matrix1x3d gx, Eigen::Matrix3d a, Eigen::Vector3d b, Eigen::Matrix1x3d c,
    Eigen::Vector3d &xd, Eigen::Vector3d &yd)
{
  int zmp_size;
  zmp_size = ref_zmp_.col(1).size();

  Eigen::VectorXd px_ref, py_ref;
  px_ref.resize(zmp_size);
  py_ref.resize(zmp_size);

  for (int i=0; i<zmp_size; i++)
  {
    px_ref(i) = ref_zmp_(i,0);
    py_ref(i) = ref_zmp_(i,1);
  }

  Eigen::Vector3d x, y, x_1, y_1;
  x.setZero();
  y.setZero();
  x_1.setZero();
  y_1.setZero();

  //file[18]<<walking_tick_<<"\t"<<px_ref(tick)<<endl;


  if(tick==0 && current_step_num_ == 0)
  {
    x(0) = x_i;
    y(0) = y_i;
  }
  else
  {
    x = xs;
    y = ys;
  }
  //  if(walking_tick_ == 920){
  //      x(0) -= 0.03;
  //      x(1) -= 0.03;
  //  }


  x_1(0) = x(0)-x(1)*dt;
  x_1(1) = x(1)-x(2)*dt;
//  x_1(2) = x(2);
  x_1(2) = xs_(2);
  y_1(0) = y(0)-y(1)*dt;
  y_1(1) = y(1)-y(2)*dt;
//  y_1(2) = y(2);
  y_1(2) = ys_(2);


  double xzmp_err =0.0, yzmp_err = 0.0;

  Eigen::Matrix<double, 1, 1> px, py;
  px = (c*x);
  py = (c*y);
  xzmp_err = px(0) - px_ref(tick);
  yzmp_err = py(0) - py_ref(tick);

//  xzmp_err = 0.0;
//  yzmp_err = 0.0;

  double sum_gp_px_ref = 0.0, sum_gp_py_ref =0.0;
  for(int i = 0; i < NL; i++)
  {
    sum_gp_px_ref = sum_gp_px_ref + gp_l(i)*(px_ref(tick+1+i)-px_ref(tick+i));
    sum_gp_py_ref = sum_gp_py_ref + gp_l(i)*(py_ref(tick+1+i)-py_ref(tick+i));
  }
  double gx_x, gx_y, del_ux, del_uy;
  gx_x = gx*(x-x_1);
  gx_y = gx*(y-y_1);

  del_ux = -(xzmp_err*gi)-gx_x-sum_gp_px_ref;
  del_uy = -(yzmp_err*gi)-gx_y-sum_gp_py_ref;

  ux = ux_1 + del_ux;
  uy = uy_1 + del_uy;

  xd = a*x + b*ux;
  yd = a*y + b*uy;

  //file[16]<<walking_tick_<<"\t"<<xd(0)<<"\t"<<xd(1)<<"\t"<<xd(2)<<"\t"<<x(0)<<"\t"<<x(1)<<"\t"<<x(2)<<"\t"<<ux<<endl;

}

void WalkingController::previewControlParameter(
    double dt, int NL, Eigen::Matrix4d& k, Eigen::Vector3d com_support_init_,
    double& gi, Eigen::VectorXd& gp_l, Eigen::Matrix1x3d& gx,
    Eigen::Matrix3d& a, Eigen::Vector3d& b, Eigen::Matrix1x3d& c)
{
  a.setIdentity();
  a(0,1) = dt;
  a(0,2) = dt*dt/2.0;
  a(1,2) = dt;

  b.setZero();
  b(0) =dt*dt*dt/6.0;
  b(1) =dt*dt/2.0;
  b(2) =dt;

  c(0,0) = 1;
  c(0,1) = 0;
  c(0,2) = -zc_/GRAVITY;


  Eigen::Vector4d b_bar;
  b_bar(0) = c*b;
  b_bar.segment(1,3) = b;

  Eigen::Matrix1x4d b_bar_tran;
  b_bar_tran = b_bar.transpose();

  Eigen::Vector4d i_p;
  i_p.setZero();
  i_p(0) = 1;

  Eigen::Matrix4x3d f_bar;
  f_bar.setZero();
  f_bar.block<1,3>(0,0) = c*a;
  f_bar.block<3,3>(1,0) = a;

  Eigen::Matrix4d a_bar;
  a_bar.block<4,1>(0,0) = i_p;
  a_bar.block<4,3>(0,1) = f_bar;

  double qe;
  qe = 1.0; // gain for zmp error originally 1.0 used
  Eigen::Matrix<double, 1,1> r;
  r(0,0) = 0.000001;

  Eigen::Matrix3d qx;
  qx.setZero();
  Eigen::Matrix4d q_bar;
  q_bar.setZero();
  q_bar(0,0) = qe;
  q_bar.block<3,3>(1,1) = qx;

  k=discreteRiccatiEquationPrev(a_bar, b_bar, r, q_bar);


  double temp_mat;
  temp_mat = r(0)+b_bar_tran*k*b_bar;

  Eigen::Matrix4d ac_bar;
  ac_bar.setZero();
  ac_bar = a_bar - b_bar*b_bar_tran*k*a_bar/temp_mat;

  gi = b_bar_tran*k*i_p;
  gi *= 1/temp_mat;
  gx = b_bar_tran*k*f_bar/temp_mat;

  Eigen::MatrixXd x_l(4, NL);
  Eigen::Vector4d x_l_column;
  x_l.setZero();
  x_l_column.setZero();
  x_l_column = -ac_bar.transpose()*k*i_p;
  for(int i=0; i<NL; i++)
  {
    x_l.col(i) = x_l_column;
    x_l_column = ac_bar.transpose()*x_l_column;
  }
  gp_l.resize(NL);
  double gp_l_column;
  gp_l_column = -gi;
  for(int i=0; i<NL; i++)
  {
    gp_l(i) = gp_l_column;
    gp_l_column = b_bar_tran*x_l.col(i);
    gp_l_column = gp_l_column/temp_mat;
  }

}

void WalkingController::compensator()
{
  bool left_support;//foot_step_(current_step_num_, 6) == 1) //left support foot

  if(joystick_walking_flag_ == true){
      if(current_step_num_<=2){
          left_support = foot_step_joy_(current_step_num_,6);
      }
      else {
          left_support = foot_step_joy_(2,6);
      }
  }
  else{
      left_support = foot_step_(current_step_num_,6);
  }

  if(hip_compensator_mode_ == true)
  {
    hipCompensation();
    if(lqr_compensator_mode_ == false)
    {
      hipCompensator(left_support);
    }
  }

  if(lqr_compensator_mode_ == true)
  {
    Eigen::Vector12d d_q;

    for (int i=0; i<12; i++)
      d_q(i) = desired_q_(i);

    Eigen::Vector12d lqr_joint_input;


    slowcalc_mutex_.lock();


    ad_copy_ = ad_right_;
    bd_copy_ = bd_right_;
    ad_total_copy_ = ad_total_right_;
    bd_total_copy_ = bd_total_right_;
    kkk_copy_ = kkk_motor_right_;
    slowcalc_mutex_.unlock();


//    vibrationControl(d_q,lqr_joint_input, left_support);
//    vibrationControl_modified(d_q,lqr_joint_input);
    Compliant_control(d_q);


    //std::cout<< "d_q - lqr_joint_input"<<d_q-lqr_joint_input<<endl;
    //std::cout<< "lqr_joint_input"<<lqr_joint_input<<endl;




    double grav_gain_timing = 1.0;
    //if(foot_step_(current_step_num_,6) == 1) // left foot (right foot gain)
    if(left_support)
    {
      if(walking_tick_ > t_start_+t_rest_init_+t_double1_ && walking_tick_ < t_start_+t_total_-t_rest_last_-t_double2_)
        grav_gain_timing = DyrosMath::cubic(walking_tick_,t_start_+t_rest_init_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_,1.0,0.3,0.0,0.0);
      else
        grav_gain_timing = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_,t_start_+t_total_-t_rest_last_,0.3,1.0,0.0,0.0);
    }


    for (int n = 7 ; n < 12; n++) // left foot
    {
      if(abs(lqr_joint_input(n)-desired_q_(n)) > 20.0*DEG2RAD )
      {
      }
      else
      {
        if(n == 7 || n == 8)
          desired_q_(n) = desired_q_(n) - 0.0022*grav_gain_timing*grav_ground_torque_[n]; //0.0024
        else if (n == 9)
          desired_q_(n) = desired_q_(n) - 0.0010*grav_gain_timing*grav_ground_torque_[n]; //0.0015
        else
          desired_q_(n) = desired_q_(n);
      }
    }

    grav_gain_timing = 1.0;
    //if(foot_step_(current_step_num_,6) == 0) // left foot (right foot gain)
    if(!left_support)
    {
      if(walking_tick_ > t_start_+t_rest_init_+t_double1_ && walking_tick_ < t_start_+t_total_-t_rest_last_-t_double2_)
        grav_gain_timing = DyrosMath::cubic(walking_tick_,t_start_+t_rest_init_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_,1.0,0.3,0.0,0.0);
      else
        grav_gain_timing = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_,t_start_+t_total_-t_rest_last_,0.3,1.0,0.0,0.0);
    }


    for (int n = 1 ; n < 6; n++)
    {
      if(abs(lqr_joint_input(n)-desired_q_(n)) > 20.0*DEG2RAD )
      {
      }
      else
      {
        if ( n == 1 || n == 2)
          desired_q_(n)  = desired_q_(n) - 0.0022*grav_gain_timing*grav_ground_torque_[n];// 0.0024
        else if (n == 3)
          desired_q_(n) = desired_q_(n) - 0.0010*grav_gain_timing*grav_ground_torque_[n]; //0.0015
        else
          desired_q_(n) = desired_q_(n);
      }
    }

    //  std::cout<< "d_q"<<d_q<<endl;

    // std::cout<< "grav_ground_torque_"<<grav_ground_torque_<<endl;


  }

}

void WalkingController::hipCompensator(bool left_support)
{
//  double left_hip_angle = 4.5*DEG2RAD, right_hip_angle = 4.0*DEG2RAD, left_hip_angle_first_step = 4.5*DEG2RAD, right_hip_angle_first_step = 4.0*DEG2RAD,
//      left_hip_angle_temp = 0.0, right_hip_angle_temp = 0.0, temp_time = 0.1*hz_, left_pitch_angle = 0.0*DEG2RAD; // for real robot

    double left_hip_angle = 2.0*DEG2RAD, right_hip_angle = 2.0*DEG2RAD, left_hip_angle_first_step = 2.0*DEG2RAD, right_hip_angle_first_step = 2.0*DEG2RAD,

        left_hip_angle_temp = 0.0, right_hip_angle_temp = 0.0, temp_time = 0.1*hz_, left_pitch_angle = 0.0*DEG2RAD;// for simulation

  if (current_step_num_ == 0)
  {
    //if(foot_step_(current_step_num_, 6) == 1) //left support foot
    if(left_support)
    {
      if(walking_tick_ < t_start_+t_total_-t_rest_last_-t_double2_-temp_time)
        left_hip_angle_temp = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_real_+t_double1_+temp_time,0.0*DEG2RAD, left_hip_angle_first_step, 0.0, 0.0);
      else if(walking_tick_ >= t_start_+t_total_-t_rest_last_-t_double2_-temp_time)
        left_hip_angle_temp = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_-temp_time,t_start_+t_total_-t_rest_last_,left_hip_angle_first_step, 0.0, 0.0, 0.0);
      else
        left_hip_angle_temp = 0.0*DEG2RAD;
    }
    //else if (foot_step_(current_step_num_, 6) == 0) // right support foot
    else if(!left_support)
    {
      if(walking_tick_ < t_start_+t_total_-t_rest_last_-t_double2_-temp_time)
        right_hip_angle_temp = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_real_+t_double1_+temp_time,0.0*DEG2RAD, right_hip_angle_first_step, 0.0, 0.0);
      else if(walking_tick_ >= t_start_+t_total_-t_rest_last_-t_double2_-temp_time)
        right_hip_angle_temp = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_-temp_time,t_start_+t_total_-t_rest_last_,right_hip_angle_first_step, 0.0, 0.0, 0.0);
      else
        right_hip_angle_temp = 0.0*DEG2RAD;
    }
    else
    {
      left_hip_angle_temp = 0.0*DEG2RAD;
      right_hip_angle_temp = 0.0*DEG2RAD;
    }
  }
  else
  {
    //if(foot_step_(current_step_num_, 6) == 1)
    if(left_support)
    {
      if(walking_tick_ < t_start_+t_total_-t_rest_last_-t_double2_-temp_time)
        left_hip_angle_temp = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_real_+t_double1_+temp_time,0.0*DEG2RAD,left_hip_angle,0.0,0.0);
      else if (walking_tick_ >= t_start_+t_total_-t_rest_last_-t_double2_-temp_time)
        left_hip_angle_temp = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_-temp_time,t_start_+t_total_-t_rest_last_,left_hip_angle,0.0,0.0,0.0);
      else
        left_hip_angle_temp = 0.0*DEG2RAD;

    }
    //else if(foot_step_(current_step_num_,6) == 0)
    else if(!left_support)
    {
      if(walking_tick_ < t_start_+t_total_-t_rest_last_-t_double2_-temp_time)
        right_hip_angle_temp = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_real_+t_double1_+temp_time,0.0*DEG2RAD,right_hip_angle,0.0,0.0);
      else if(walking_tick_ >= t_start_+t_total_-t_rest_last_-t_double2_-temp_time)
        right_hip_angle_temp = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_-temp_time,t_start_+t_total_-t_rest_last_,right_hip_angle,0.0,0.0,0.0);
      else
        right_hip_angle_temp = 0.0*DEG2RAD;
    }
    else
    {
      left_hip_angle_temp = 0.0*DEG2RAD;
      right_hip_angle_temp = 0.0*DEG2RAD;
    }
  }
  desired_q_(1) = desired_q_(1) + left_hip_angle_temp;
  desired_q_(7) = desired_q_(7) - right_hip_angle_temp;
  joint_offset_angle_(1) = left_hip_angle_temp;
  joint_offset_angle_(7) = -right_hip_angle_temp;
}

void WalkingController::hipCompensation()
{
  double a_total, b_total, alpha_l, alpha_r, rq0, rq1, rq2, rq3, rq4, rq5, lq0, lq1, lq2, lq3, lq4, lq5, robotweight, fromright, fromleft, alpha, alpha_1, f_r, f_l; //alpha is weighting factor

  a_total = -0.0012;
  b_total = 0.00087420;
  //robotweight = 50.3082*9.81;

  robotweight =46.892*9.81;


  lq0= desired_q_(0);
  lq1= desired_q_(1);
  lq2= desired_q_(2);
  lq3= desired_q_(3);
  lq4= desired_q_(4);
  lq5= desired_q_(5);
  rq0= desired_q_(6);
  rq1= desired_q_(7);
  rq2= desired_q_(8);
  rq3= desired_q_(9);
  rq4= desired_q_(10);
  rq5= desired_q_(11);

  //for simulation
   fromright = cos(rq2)*sin(rq0)*(-1.454E-1)-sin(rq0)*sin(rq2)*(3.39E2/1.0E3)-cos(rq3)*(cos(rq2)*sin(rq0)+cos(rq0)*sin(rq1)*sin(rq2))*(3.0/5.0E1)-cos(rq3)*(sin(rq0)*sin(rq2)-cos(rq0)*cos(rq2)*sin(rq1))*(4.6E1/1.25E2)-sin(rq3)*(cos(rq2)*sin(rq0)+cos(rq0)*sin(rq1)*sin(rq2))*(4.6E1/1.25E2)+sin(rq3)*(sin(rq0)*sin(rq2)-cos(rq0)*cos(rq2)*sin(rq1))*(3.0/5.0E1)+cos(rq0)*cos(rq2)*sin(rq1)*(3.39E2/1.0E3)-cos(rq0)*sin(rq1)*sin(rq2)*1.454E-1-2.1E1/2.0E2;
  fromleft = cos(lq2)*sin(lq0)*(-1.454E-1)+sin(lq0)*sin(lq2)*(3.39E2/1.0E3)+cos(lq3)*(sin(lq0)*sin(lq2)+cos(lq0)*cos(lq2)*sin(lq1))*(4.6E1/1.25E2)-cos(lq3)*(cos(lq2)*sin(lq0)-cos(lq0)*sin(lq1)*sin(lq2))*(3.0/5.0E1)+sin(lq3)*(sin(lq0)*sin(lq2)+cos(lq0)*cos(lq2)*sin(lq1))*(3.0/5.0E1)+sin(lq3)*(cos(lq2)*sin(lq0)-cos(lq0)*sin(lq1)*sin(lq2))*(4.6E1/1.25E2)+cos(lq0)*cos(lq2)*sin(lq1)*(3.39E2/1.0E3)+cos(lq0)*sin(lq1)*sin(lq2)*1.454E-1+2.1E1/2.0E2;

  // for robot
//  if(estimator_flag_ == false)
//  {
//    fromright = -com_float_current_(1)+rfoot_float_current_.translation()(1);
//    fromleft = -com_float_current_(1)+lfoot_float_current_.translation()(1);
//  }
//  else
//  {
//    Eigen::Vector3d com_float_estimated;
//    Eigen::Vector3d com_support_estimated;
//    com_support_estimated(0) = X_hat_post_1_(0);
//    com_support_estimated(1) = X_hat_post_1_(1);
//    com_support_estimated(2) = com_support_current_(2);
//    com_float_estimated = DyrosMath::multiplyIsometry3dVector3d(supportfoot_float_current_, com_support_estimated);
//    fromright = -com_float_estimated(1)+rfoot_float_current_.translation()(1);
//    fromleft = -com_float_estimated(1)+lfoot_float_current_.translation()(1);
//  } // on the robot


  //fromright = rfoot_float_current_.translation()(1);
  //fromleft = lfoot_float_current_.translation()(1);


  alpha = -fromleft/(fromright-fromleft);

  if(fromright>=0)
  {
    alpha=1;
  }
  if(fromleft<=0)
  {
    alpha=0;
  }

  alpha_1 = 1-alpha;

  f_r = robotweight*alpha;
  f_l = robotweight*alpha_1;


  Eigen::Vector6d lTau, rTau, Jc2, Jc8;
  lTau.setZero();
  rTau.setZero();
  Jc2.setZero();
  Jc8.setZero();
  /*
  Jc2(0) = 0;
  Jc2(1) = cos(rq2)*sin(rq1)*(3.39E2/1.0E3)-sin(rq1)*sin(rq2)*1.454E-1-sin(rq1)*sin(rq2)*sin(rq3)*(4.6E1/1.25E2)+cos(rq2)*cos(rq3)*sin(rq1)*(4.6E1/1.25E2)-cos(rq2)*sin(rq1)*sin(rq3)*(3.0/5.0E1)-cos(rq3)*sin(rq1)*sin(rq2)*(3.0/5.0E1);
  Jc2(2) = cos(rq1)*cos(rq2)*1.454E-1+cos(rq1)*sin(rq2)*(3.39E2/1.0E3)+cos(rq1)*cos(rq2)*cos(rq3)*(3.0/5.0E1)+cos(rq1)*cos(rq2)*sin(rq3)*(4.6E1/1.25E2)+cos(rq1)*cos(rq3)*sin(rq2)*(4.6E1/1.25E2)-cos(rq1)*sin(rq2)*sin(rq3)*(3.0/5.0E1);
  Jc2(3) = cos(rq1)*cos(rq2)*cos(rq3)*(3.0/5.0E1)+cos(rq1)*cos(rq2)*sin(rq3)*(4.6E1/1.25E2)+cos(rq1)*cos(rq3)*sin(rq2)*(4.6E1/1.25E2)-cos(rq1)*sin(rq2)*sin(rq3)*(3.0/5.0E1);
  Jc2(4) = 0;
  Jc2(5) = 0;

  Jc8(0) = 0;
  Jc8(1) = cos(lq2)*sin(lq1)*(3.39E2/1.0E3)+sin(lq1)*sin(lq2)*1.454E-1-sin(lq1)*sin(lq2)*sin(lq3)*(4.6E1/1.25E2)+cos(lq2)*cos(lq3)*sin(lq1)*(4.6E1/1.25E2)+cos(lq2)*sin(lq1)*sin(lq3)*(3.0/5.0E1)+cos(lq3)*sin(lq1)*sin(lq2)*(3.0/5.0E1);
  Jc8(2) = cos(lq1)*cos(lq2)*(-1.454E-1)+cos(lq1)*sin(lq2)*(3.39E2/1.0E3)-cos(lq1)*cos(lq2)*cos(lq3)*(3.0/5.0E1)+cos(lq1)*cos(lq2)*sin(lq3)*(4.6E1/1.25E2)+cos(lq1)*cos(lq3)*sin(lq2)*(4.6E1/1.25E2)+cos(lq1)*sin(lq2)*sin(lq3)*(3.0/5.0E1);
  Jc8(3) = cos(lq1)*cos(lq2)*cos(lq3)*(-3.0/5.0E1)+cos(lq1)*cos(lq2)*sin(lq3)*(4.6E1/1.25E2)+cos(lq1)*cos(lq3)*sin(lq2)*(4.6E1/1.25E2)+cos(lq1)*sin(lq2)*sin(lq3)*(3.0/5.0E1);
  Jc8(4) = 0;
  Jc8(5) = 0;
*/
  Eigen::Vector3d ankle_to_sole;
  ankle_to_sole.setZero();
  ankle_to_sole(2) = -0.09;
  Eigen::Matrix6d adjoint_l, adjoint_r;
  adjoint_l.setIdentity();
  adjoint_l.block<3,3>(0,3) = -DyrosMath::skew(lfoot_float_current_.linear()*ankle_to_sole);
  adjoint_r.setIdentity();
  adjoint_r.block<3,3>(0,3) = -DyrosMath::skew(lfoot_float_current_.linear()*ankle_to_sole);

  Eigen::Matrix6d Jcl, Jcr;
  Jcl = adjoint_l*current_leg_jacobian_l_.transpose();
  Jcr = adjoint_r*current_leg_jacobian_r_.transpose();
  Jc8 = Jcl.col(2);
  Jc2 = Jcr.col(2);

  for(int i=0; i<6; i++)
  {
    rTau(i)=Jc2(i)*f_r;
    lTau(i)=Jc8(i)*f_l;
  }

  double rising = 1.0, timingtiming = 1.0, k = 0.2, k1 = 0.2;
  /*
  if(walking_tick_ > t_start_+ t_rest_init_ && walking_tick_ <= t_start_+t_rest_init_+t_double1_*timingtiming)
  {
      rising = (walking_tick_-t_start_-t_rest_init_)/(t_double1_*timingtiming);
  }
  else if(walking_tick_ > t_start_+t_rest_init_+t_double1_*timingtiming && walking_tick_<= t_start_+t_total_-t_rest_last_-t_double2_*timingtiming)
  {
      rising =1;
  }
  else if(walking_tick_ > t_start_+t_total_-t_rest_last_-t_double2_*timingtiming &&  walking_tick_<= t_start_+t_total_-t_rest_last_)
  {
      rising = -(walking_tick_- (t_start_+t_total_-t_rest_last_))/(t_double2_*timingtiming);
  }*/

  joint_offset_angle_.setZero();
  grav_ground_torque_.setZero();

  if (lqr_compensator_mode_ == false)
  {
    //desired_q_(7)=desired_q_(7)+(a_total*rTau(1)+b_total)*rising*k1;//offwhenslow
    desired_q_(8)=desired_q_(8)+(a_total*rTau(2)+b_total)*rising*k1;//offwhenslow
    desired_q_(9)=desired_q_(9)+(a_total*rTau(3)+b_total)*rising*0.3;//offwhenslow
    desired_q_(10)=desired_q_(10)+(a_total*rTau(4)+b_total)*rising*k1;//offwhenslow
    //desired_q_(11)=desired_q_(11)+(a_total*rTau(5)+b_total)*rising*k1;//offwhenslow
  }
  // _desired_q(23-2)=_desired_q(23-2)+(a_total*rTau(5)+b_total)*rising;

  joint_offset_angle_(8) = (a_total*rTau(2)+b_total)*rising*k;
  joint_offset_angle_(9) = (a_total*rTau(3)+b_total)*rising*0.2;
  joint_offset_angle_(10) = (a_total*rTau(4)+b_total)*rising*k;

  if (lqr_compensator_mode_  == false)
  {
    //desired_q_(1)=desired_q_(1)+(a_total*lTau(1)+b_total)*rising*k;//offwhenslow
    desired_q_(2)=desired_q_(2)+(a_total*lTau(2)+b_total)*rising*k;//offwhenslow
    desired_q_(3)=desired_q_(3)+(a_total*lTau(3)+b_total)*rising*0.3;//offwhenslow
    desired_q_(4)=desired_q_(4)+(a_total*lTau(4)+b_total)*rising*k;//offwhenslow
    //desired_q_(5)=desired_q_(5)+(a_total*lTau(5)+b_total)*rising*k;//offwhenslow

    //  _desired_q(29-2)=_desired_q(29-2)+(a_total*lTau(5)+b_total)*rising*k;
  }

  joint_offset_angle_(2) = (a_total*lTau(2)+b_total)*rising*k1;
  joint_offset_angle_(3) = (a_total*lTau(3)+b_total)*rising*0.2;
  joint_offset_angle_(4) = (a_total*lTau(4)+b_total)*rising*k1;



  for (int i=0; i<6; i++)
  {
    grav_ground_torque_(i) = lTau[i];
    grav_ground_torque_(i+6) = rTau[i];
  }
 // cout<<joint_offset_angle_<<endl;
}


void WalkingController::vibrationControl(const Eigen::Vector12d desired_leg_q, Eigen::Vector12d &output, bool left_support)
{
  if(walking_tick_ == 0)
  {
    pre_motor_q_leg_ = current_motor_q_leg_;
    pre_link_q_leg_ = current_link_q_leg_;
    lqr_output_pre_ = current_motor_q_leg_;
  }
  x_bar_right_.setZero();

  // right foot X_bar
  for (int i = 0; i < 12; i++)
  {
    x_bar_right_(i, 0) = current_link_q_leg_(i) - desired_leg_q(i); //left

    x_bar_right_(i+12,0) = current_motor_q_leg_(i) - pre_motor_q_leg_(i); //left

    x_bar_right_(i+24,0) = 0.0;//current_link_q_leg_(i) - pre_link_q_leg_(i); //left

  }


  Eigen::Vector12d del_u_right;

  //std::cout<< "kkk_copy"<< kkk_copy_<<endl;
  //std::cout<< "x_bar_right_"<< x_bar_right_<<endl;


  del_u_right = -kkk_copy_*x_bar_right_;


  // if(_cnt == 0)
  //  std::cout << "ad_copy_" << ad_copy_ << endl;
  //  std::cout << "bd_copy_" << bd_copy_ << endl;

  x_bar_right_ = ad_total_copy_*x_bar_right_ + bd_total_copy_*del_u_right;


  Eigen::Vector12d current_u;// left right order


  for (int i = 0; i < 6; i++)
  {
    //left
    current_u(i) = (current_motor_q_leg_(i) - ad_copy_(i, i)*pre_motor_q_leg_(i)) / bd_copy_(i, i);

    //right
    current_u(i+6) = (current_motor_q_leg_(i+6) - ad_copy_(i, i)*pre_motor_q_leg_(i+6)) / bd_copy_(i, i);
  }


  Eigen::Vector12d dist;//left right order


  dist = lqr_output_pre_ - current_u;


  if(walking_tick_ == 0)
    dist_prev_ = dist;

  dist = 0.7*dist_prev_+0.3*dist;

  //std::cout << "current_u" << current_u << endl;
  //std::cout << "dist" << dist << endl;
  //std::cout << "ad_copy_" << ad_copy_ << endl;
  //std::cout << "bd_copy_" << bd_copy_ << endl;


  // Mingon's LQR contorller gain(using external encoder)
  double default_gain = 0.200;

  double compliant_gain = 1.0;

  double compliant_tick = 0.1*hz_;


  for (int i = 0; i < 12; i++)
  {
    if(i < 6) //left foot
    {
      double gain_temp = default_gain;


      if(walking_enable_ == true)
      {
        //if (foot_step_(current_step_num_,6) == 0) // right support foot
        if(!left_support)
        {
          if(walking_tick_ < t_start_+t_total_-t_rest_last_-t_double2_-compliant_tick) // gain_temp -> 0.2
          {
            gain_temp = default_gain;
          }
          else if(walking_tick_ >= t_start_+t_total_-t_rest_last_-t_double2_-compliant_tick && walking_tick_ < t_start_+t_total_-t_rest_last_-t_double2_)
          {
            gain_temp = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_-compliant_tick,t_start_+t_total_-t_rest_last_-t_double2_,default_gain,compliant_gain,0.0,0.0);
          }
          else
          {
            gain_temp = DyrosMath::cubic(walking_tick_, t_start_+t_total_-t_rest_last_,t_start_+t_total_,compliant_gain,default_gain,0.0,0.0);
          }
        }
        else //left support foot
        {
          gain_temp = default_gain;

        }
      }

      lqr_output_(i) = lqr_output_pre_(i) + del_u_right(i, 0) - gain_temp*dist(i);
    }
    else // right foot
    {
      double gain_temp = default_gain;


      if(walking_enable_ == true)
      {
        //if (foot_step_(current_step_num_,6) == 1) // left suppor foot
        if(left_support)
        {
          if(walking_tick_ < t_start_+t_total_-t_rest_last_-t_double2_-compliant_tick) // gain_temp -> 0.2
          {
            gain_temp = default_gain;
          }
          else if(walking_tick_ >= t_start_+t_total_-t_rest_last_-t_double2_-compliant_tick && walking_tick_ < t_start_+t_total_-t_rest_last_-t_double2_)
          {
            gain_temp = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_-compliant_tick,t_start_+t_total_-t_rest_last_-t_double2_,default_gain,compliant_gain,0.0,0.0);
          }
          else
          {
            gain_temp = DyrosMath::cubic(walking_tick_, t_start_+t_total_-t_rest_last_,t_start_+t_total_,compliant_gain,default_gain,0.0,0.0);
          }
        }
        else //right foot support
        {
          gain_temp = default_gain;

        }

      }
      lqr_output_(i) = lqr_output_pre_(i) + del_u_right(i, 0) - gain_temp*dist(i);
    }


    lqr_output_pre_ = lqr_output_;
    pre_motor_q_leg_ = current_motor_q_leg_;
    pre_link_q_leg_ = current_link_q_leg_;
    dist_prev_ = dist;

    for (int i=0; i<12; i++)
    {
      output(i) = lqr_output_(i);
    }


  }
}

void WalkingController::massSpringMotorModel(double spring_k, double damping_d, double motor_k, Eigen::Matrix12d & mass, Eigen::Matrix<double, 36, 36>& a, Eigen::Matrix<double, 36, 12>& b, Eigen::Matrix<double, 12, 36>& c)
{
  int dof = 12;
  Eigen::Matrix12d spring_k_mat;
  Eigen::Matrix12d damping_d_mat;
  Eigen::Matrix12d motor_k_mat;

  spring_k_mat.setZero();
  damping_d_mat.setZero();
  motor_k_mat.setZero();

  for (int i = 0; i < dof; i++)
  {
    spring_k_mat(i, i) = spring_k;
    damping_d_mat(i, i) = damping_d;
    motor_k_mat(i, i) = motor_k;
  }

  // knee joint
  motor_k_mat(3,3) =  18.0; //18.0
  motor_k_mat(9,9) =  18.0;


  Eigen::Matrix12d inv_mass;
  inv_mass = mass;

  Eigen::Matrix12d zero_12;
  zero_12.setZero();

  Eigen::Matrix12d eye_12;
  eye_12.setIdentity();

  Eigen::Matrix12d mass_temp;
  mass_temp = inv_mass;

  Eigen::Matrix12d a1;
  a1 = mass_temp*(spring_k_mat - damping_d_mat*motor_k_mat);

  Eigen::Matrix12d a2;
  a2 = -mass_temp*(spring_k_mat);

  Eigen::Matrix12d a3;
  a3 = -mass_temp*(damping_d_mat);

  a.setZero();
  a << -motor_k_mat, zero_12, zero_12, zero_12, zero_12, eye_12, a1, a2, a3;

  Eigen::Matrix12d b1;
  b1 = mass_temp*damping_d_mat*motor_k_mat;


  b.setZero();
  b << motor_k_mat, zero_12, b1;

  c.setZero();
  c << zero_12, eye_12, zero_12;




}

void WalkingController::discreteModel(Eigen::Matrix<double, 36, 36>& a, Eigen::Matrix<double, 36, 12>& b, Eigen::Matrix<double, 12, 36>& c, int np, double dt,
                                      Eigen::Matrix<double, 36, 36>& ad, Eigen::Matrix<double, 36, 12>& bd, Eigen::Matrix<double, 12, 36>& cd,
                                      Eigen::Matrix<double, 48, 48>& ad_total, Eigen::Matrix<double, 48, 12>& bd_total)
{
  int n = a.rows(); //state \B0\B9\BC\F6  (36)
  int r = b.cols(); // Input \B0\B9\BC\F6  (12)
  int p = c.rows(); // output \B0\B9\BC\F6  (12)

  // ad = A*dt;
  // ad = ad.exp();


  Eigen::Matrix<double, 36, 36> inv_a;
  inv_a = a.inverse();

  Eigen::Matrix<double, 36, 36> eye6;
  eye6.setIdentity();

  // bd = inv_a*(ad - eye6)*B;

  ad = eye6 +a*dt;
  bd = b*dt;

  cd = c;

  //std::cout << "AAA_bar" << AAA_bar << std::endl;
  //std::cout << "BBB_bar" << BBB_bar << std::endl;

  Eigen::Matrix<double, 12, 36> ca;
  ca = cd*ad;

  Eigen::Matrix<double, 12, 12> eye_p;
  eye_p.setIdentity();

  Eigen::Matrix<double, 36, 12> zero_n_p;
  zero_n_p.setZero();

  Eigen::Matrix<double, 12, 12> cb;
  cb = cd*bd;

  if (np < 1)
  {

    ad_total << eye_p, ca, zero_n_p, ad;
    bd_total << cb, bd;
  }
  else
  {
    ad_total.resize(n + p + np, n + p + np);
    bd_total.resize(n + p + np, r);

    Eigen::MatrixXd zero_temp1;
    zero_temp1.resize(p, (np - 1)*p); zero_temp1.setZero();

    Eigen::MatrixXd zero_temp2;
    zero_temp2.resize(n, np*p); zero_temp2.setZero();

    Eigen::MatrixXd zero_temp3;
    zero_temp3.resize(np*p, p + n); zero_temp3.setZero();

    Eigen::MatrixXd shift_register;
    shift_register.resize(np*p, np*p); shift_register.setZero();


    Eigen::MatrixXd zero_temp4;
    zero_temp4.resize(np*p, r); zero_temp4.setZero();
    for (int i = 0; i<p*(np - 1); i++)
      shift_register(i, i + p) = 1;

    ad_total << eye_p, ca, -eye_p, zero_temp1, zero_n_p, ad, zero_temp2, zero_temp3, shift_register;
    bd_total << cb, bd, zero_temp4;
  }

}

void WalkingController::riccatiGain(Eigen::Matrix<double, 48, 48>& ad_total, Eigen::Matrix<double, 48, 12>& bd_total, Eigen::Matrix<double, 48, 48>& q, Eigen::Matrix12d& r, Eigen::Matrix<double, 12, 48>& k)
{

  const int n = ad_total.rows(); //state \B0\B9\BC\F6 (48)
  const int m = bd_total.cols(); // Input \B0\B9\BC\F6 (12)

  //  std::cout<<"c1" <<kkk_<<endl;

  static bool InitDare;

  if (!InitDare)
  {
    std::cout << "insiadfnisdaifnadisfasd" <<std::endl;
    discreteRiccatiEquationInitialize(ad_total, bd_total);
    InitDare = true;
  }


  kkk_ = discreteRiccatiEquationLQR(ad_total, bd_total, r, q);

  //std::cout<<"kkk_" <<kkk_<<endl;
  //KKK.resize(8,8); KKK.setIdentity();

  Eigen::Matrix<double, 12, 48> trans_bd;
  trans_bd = bd_total.transpose();
  Eigen::Matrix<double, 12, 12> temp_r_inv;
  temp_r_inv = (r + trans_bd*kkk_*bd_total);
  temp_r_inv = temp_r_inv.inverse();
  k = temp_r_inv *trans_bd*kkk_*ad_total;

  //k = k.inverse();
  //k = k*trans_bd*kkk_*ad_total;

}



void WalkingController::slowCalc()
{
  while(true)
  {
    if(ready_for_thread_flag_)
    {
      slowCalcContent();
      if (ready_for_compute_flag_ == false)
      {
        ready_for_compute_flag_ = true;
      }
    }

    this_thread::sleep_for(chrono::milliseconds(100));

  }
}

void WalkingController::slowCalcContent()
{
  Eigen::Vector28d qqq;
  slowcalc_mutex_.lock();

  for(int i =0; i<28; i++)
  {
    qqq(i) = thread_q_(i);
  }
  slowcalc_mutex_.unlock();

  Eigen::Matrix<double, 6, 18> contact_j;
  Eigen::Matrix6d lamda;
  Eigen::Matrix<double, 6, 18> j_c_bar;
  Eigen::Matrix<double, 18, 18> pc;
  Eigen::Matrix<double, 18, 18> eye_18;
  Eigen::Matrix<double, 18, 18> mass_inverse;
  Eigen::Matrix<double, 12, 12> temp22;
  Eigen::Matrix<double, 48, 48> q_mat;
  Eigen::Matrix12d r_mat;

  Eigen::Matrix<double, 12, 48> lqr_k; //lqr_k.resize(12, 12*4);

  if(thread_tick_ == 0)
  {
    mass_matrix_.setZero();
    mass_matrix_pc_.setZero();
    mass_matrix_sel_.setZero();
  }

  mass_matrix_ = model_.getLegInertia();


  //std::cout << "MASSMATRIXxxxx" << mass_matrix_ << std::endl;
  bool left_support;//foot_step_(current_step_num_, 6) == 1) //left support foot

  if(joystick_walking_flag_ == true){
      if(current_step_num_<=2){
          left_support = foot_step_joy_(current_step_num_,6);
      }
      else {
          left_support = foot_step_joy_(2,6);
      }
  }
  else{
      left_support = foot_step_(current_step_num_,6);
  }

  if((walking_enable_) == true)
  {
    //if(foot_step_(current_step_num_,6) == 0) //right foot
    if(!left_support)
    {
      contact_j = model_.getLegWithVLinkJacobian((DyrosJetModel::EndEffector)(1));
    }
    //else if(foot_step_(current_step_num_,6) == 1)
    else if(left_support)
    {
      contact_j = model_.getLegWithVLinkJacobian((DyrosJetModel::EndEffector)(0));
    }

    lamda = (contact_j*mass_matrix_.inverse()*contact_j.transpose());
    lamda = lamda.inverse();
    j_c_bar = lamda*contact_j*mass_matrix_.inverse();

    //std::cout<<"contact_j"<<contact_j<<endl;
    pc = contact_j.transpose()*j_c_bar;

    eye_18.setIdentity();


    mass_inverse = mass_matrix_.inverse();

    mass_matrix_pc_ = mass_inverse*(eye_18-pc);

    for (int i=0; i<12; i++)
    {
      for (int j=0; j<12; j++)
        mass_matrix_sel_(i,j) = mass_matrix_pc_(i+6,j+6);
    }

  }
  else
  {
    mass_inverse = mass_matrix_.inverse();

    for (int i=0; i<12; i++)
    {
      for (int j=0; j<12; j++)
        mass_matrix_sel_(i,j) = mass_inverse(i+6,j+6);
    }
  }

  temp22.setZero();
  temp22 = mass_matrix_sel_;

  mass_matrix_sel_.setZero();
  for (int i=0; i<12; i++)
  {
    mass_matrix_sel_(i,i) = temp22(i,i);
  }


  //if(thread_tick_ == 0)
  //std::cout << "masssel" << mass_matrix_sel_ << endl;

  double spring_k = 2000.0;
  double damping_d = 100.0;
  double motor_k = 10.0;
  massSpringMotorModel(spring_k, damping_d, motor_k, mass_matrix_sel_, a_right_mat_, b_right_mat_, c_right_mat_);
  //  std::cout<<"massSpringMotor pass"<<endl;
  discreteModel(a_right_mat_, b_right_mat_, c_right_mat_, 0, 1.0/hz_, a_disc_, b_disc_, c_right_mat_, a_disc_total_, b_disc_total_);
  //  std::cout<<"discreteModel pass"<<endl;



  q_mat.setIdentity();
  q_mat = q_mat*1.0;

  for (int i=0; i<12; i++)
  {
    q_mat(i,i) = 10.0;

  }

  r_mat.setIdentity();
  r_mat = r_mat * 1.0;




  //std::cout << "c" << q_mat << std::endl;

  //std::cout << "d" << r_mat << std::endl;

  riccatiGain(a_disc_total_, b_disc_total_, q_mat, r_mat, lqr_k);

  //std::cout<<"lqr_k"<<lqr_k<<endl;
  //std::cout << "a" << a_disc_total_ << std::endl;
  //std::cout << "b" << b_disc_total_ << std::endl;

  if(ready_for_compute_flag_==false)
  {


  }


  calc_update_flag_ = true;
  thread_tick_++;

  slowcalc_mutex_.lock();

  ad_right_ = a_disc_;
  bd_right_ = b_disc_;
  ad_total_right_ = a_disc_total_;
  bd_total_right_ = b_disc_total_;
  kkk_motor_right_ = lqr_k;
  slowcalc_mutex_.unlock();
}


void WalkingController::discreteRiccatiEquationInitialize(Eigen::MatrixXd a, Eigen::MatrixXd b)

{
  int n=a.rows(); //number of rows
  int	m=b.cols(); //number of columns

  Z11.resize(n, n);
  Z12.resize(n, n);
  Z21.resize(n, n);
  Z22.resize(n, n);
  temp1.resize(m, m);
  temp2.resize(n, n);
  temp3.resize(m, n);

  eigVal_real.resize(2 * n); //eigen value\C0\C7 real\B0\AA
  eigVal_img.resize(2 * n); //eigen value\C0\C7 img\B0\AA
  eigVec_real.resize(2 * n); //eigen vector\C0\C7 real\B0\AA
  eigVec_img.resize(2 * n); //eigen vector\C0\C7 img\B0\AA

  Z.resize(2 * n, 2 * n);

  deigVal_real.resize(2 * n);
  deigVal_img.resize(2 * n);

  deigVec_real.resize(2 * n, 2 * n);
  deigVec_img.resize(2 * n, 2 * n);

  tempZ_real.resize(2 * n, n);
  tempZ_img.resize(2 * n, n);

  U11_inv.resize(n, n);
  X.resize(n, n);
  X_sol.resize(n, n);

  tempZ_comp.resize(n * 2, n);
  U11.resize(n, n);
  U21.resize(n, n);

}

Eigen::MatrixXd WalkingController::discreteRiccatiEquationLQR(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd R, Eigen::MatrixXd Q)
{
  int n=A.rows(); //number of rows
  int n2 = n * 2;
  int	m=B.cols(); //number of columns

  for (int i = 0; i<2 * n; i++) //\B0\AA\C0\C7 \C3ʱ\E2ȭ
  {
    eigVec_real[i].resize(n2);
    eigVec_real[i].setZero();
    eigVec_img[i].resize(n2);
    eigVec_img[i].setZero();
  }

  deigVal_real.setZero();
  deigVal_img.setZero();
  deigVec_real.setZero();
  deigVec_img.setZero();
  tempZ_real.setZero();
  tempZ_img.setZero();
  U11_inv.setZero();
  X.setZero();
  X_sol.setZero();


  Z11 = A.inverse();
  temp1 = R.inverse();
  Z21 = Q*Z11;

  Eigen::MatrixXd B_T;
  B_T = B.transpose();
  temp2 = B * R.inverse() * B.transpose();     //B*inv(R)*B'
  Z12 = Z11*temp2;
  Eigen::MatrixXd A_T;
  A_T = A.transpose();
  Z22 = A.transpose() + Z21*temp2;
  //construct the Z with Z11,Z12,Z21,Z22
  Z.setZero();

  //ggory15
  Z.topLeftCorner(n, n) = Z11;
  Z.topRightCorner(n, n) = Z12;
  Z.bottomLeftCorner(n, n) = Z21;
  Z.bottomRightCorner(n, n) = Z22;


  /*
  Z.set(0,0,n,n,Z11);
  Z.set(0,n,n,n,Z12);
  Z.set(n,0,n,n,Z21);
  Z.set(n,n,n,n,Z22);
  */


  using namespace std;

  Eigen::MatrixXd Z_evr = Z; // \C0ӽ\C3 \C0\FA\C0\E5, rmath\C0\C7 evr\C0\BB \C7ϸ\E9 \BF\F8\BA\BB Z matrix\B0\A1 \BA\AF\C7\FC\B5\CA

  /////////////////////////
  Eigen::EigenSolver<Eigen::MatrixXd> es(Z_evr);	//8ms


  Z_eig = Z.eigenvalues();	//5ms
  es_eig = es.eigenvectors();

  deigVal_real = Z_eig.real();
  deigVal_img = Z_eig.imag();


  for (int i = 0; i<n2; i++)
  {
    for (int ii = 0; ii<n2; ii++)
    {
      deigVec_real(ii, i) = es_eig.col(i)(ii).real();
      deigVec_img(ii, i) = es_eig.col(i)(ii).imag();
    }
  }

  int c1 = 0;

  for (int i = 0; i<n2; i++)
  {
    if ((deigVal_real(i)*deigVal_real(i) + deigVal_img(i)*deigVal_img(i))>1.0) //outside the unit cycle
    {
      for (int j = 0; j<n2; j++)
      {
        tempZ_real(j, c1) = deigVec_real(j, i);
        tempZ_img(j, c1) = deigVec_img(j, i);
      }
      c1++;
    }
  }

  using namespace Eigen;

  for (int i = 0; i<n2; i++)
  {
    for (int j = 0; j<n; j++)
    {
      tempZ_comp.real()(i, j) = tempZ_real(i, j);
      tempZ_comp.imag()(i, j) = tempZ_img(i, j);
    }
  }

  for (int i = 0; i<n; i++)
  {
    for (int j = 0; j<n; j++)
    {
      U11(i, j) = tempZ_comp(i, j);
      U21(i, j) = tempZ_comp(i + n, j);
    }
  }

  U11_inv = U11.inverse();
  X = U21*(U11_inv);

  for (int i = 0; i<n; i++)
  {
    for (int j = 0; j<n; j++)
    {
      X_sol(i, j) = X.real()(i, j);
    }
  }

  return X_sol;
}

Eigen::MatrixXd WalkingController::discreteRiccatiEquationPrev(Eigen::MatrixXd a, Eigen::MatrixXd b, Eigen::MatrixXd r, Eigen::MatrixXd q)
{
  int n=a.rows(); //number of rows
  int	m=b.cols(); //number of columns

  Eigen::MatrixXd z11(n, n), z12(n, n), z21(n, n), z22(n, n);

  z11 = a.inverse();
  z12 = a.inverse()*b*r.inverse()*b.transpose();
  z21 = q*a.inverse();
  z22 = a.transpose() + q*a.inverse()*b*r.inverse()*b.transpose();

  Eigen::MatrixXd z; z.resize(2*n, 2*n);
  z.setZero();
  z.topLeftCorner(n,n) = z11;
  z.topRightCorner(n,n) = z12;
  z.bottomLeftCorner(n,n) = z21;
  z.bottomRightCorner(n,n) = z22;


  std::vector<Eigen::VectorXd> eigVec_real(2*n);
  std::vector<Eigen::VectorXd> eigVec_img(2*n);

  for(int i=0; i<8; i++)
  {
    eigVec_real[i].resize(2*n);
    eigVec_real[i].setZero();
    eigVec_img[i].resize(2*n);
    eigVec_img[i].setZero();
  }

  Eigen::VectorXd deigVal_real(2*n);
  Eigen::VectorXd deigVal_img(2*n);
  deigVal_real.setZero();
  deigVal_img.setZero();
  Eigen::MatrixXd deigVec_real(2*n,2*n);
  Eigen::MatrixXd deigVec_img(2*n,2*n);
  deigVec_real.setZero();
  deigVec_img.setZero();

  deigVal_real = z.eigenvalues().real();
  deigVal_img = z.eigenvalues().imag();

  Eigen::EigenSolver<Eigen::MatrixXd> ev(z);
  //EigenVector Solver
  //Matrix3D ones = Matrix3D::Ones(3,3);
  //EigenSolver<Matrix3D> ev(ones);
  //cout << "The first eigenvector of the 3x3 matrix of ones is:" << endl << ev.eigenvectors().col(1) << endl;

  for(int i=0;i<2*n; i++)
  {
    for(int j=0; j<2*n; j++)
    {
      deigVec_real(j,i) = ev.eigenvectors().col(i)(j).real();
      deigVec_img(j,i) = ev.eigenvectors().col(i)(j).imag();
    }
  }

  //Order the eigenvectors
  //move e-vectors correspnding to e-value outside the unite circle to the left

  Eigen::MatrixXd tempZ_real(2*n, n), tempZ_img(2*n, n);
  tempZ_real.setZero();
  tempZ_img.setZero();
  int c=0;

  for (int i=0;i<2*n;i++)
  {
    if ((deigVal_real(i)*deigVal_real(i)+deigVal_img(i)*deigVal_img(i))>1.0) //outside the unit cycle
    {
      for(int j=0; j<2*n; j++)
      {
        tempZ_real(j,c) = deigVec_real(j,i);
        tempZ_img(j,c) = deigVec_img(j,i);
      }
      c++;
    }
  }

  Eigen::MatrixXcd tempZ_comp(2*n, n);
  for(int i=0;i<2*n;i++)
  {
    for(int j=0;j<n;j++)
    {
      tempZ_comp.real()(i,j) = tempZ_real(i,j);
      tempZ_comp.imag()(i,j) = tempZ_img(i,j);
    }
  }

  Eigen::MatrixXcd U11(n, n), U21(n, n), X(n, n);
  for(int i=0;i<n;i++)
  {
    for(int j=0;j<n;j++)
    {
      U11(i,j) = tempZ_comp(i,j);
      U21(i,j) = tempZ_comp(i+n,j);
    }
  }
  X = U21*(U11.inverse());

  Eigen::MatrixXd X_sol(n, n);
  for(int i=0;i<n;i++)
  {
    for(int j=0;j<n;j++)
    {
      X_sol(i,j) = X.real()(i,j);
    }
  }
  /*
  Eigen::Matrix4d z11;
  Eigen::Matrix4d z12;
  Eigen::Matrix4d z21;
  Eigen::Matrix4d z22;

  Eigen::Matrix8d z;

  std::vector<Eigen::Vector8d> eigVec_real;
  std::vector<Eigen::Vector8d> eigVec_img;
  Eigen::Vector8d deigVal_real;
  Eigen::Vector8d deigVal_img;
  Eigen::Matrix8x2d deigVec_real;
  Eigen::Matrix8d deigVec_img;

  Eigen::Matrix8x4d tempZ_real;
  Eigen::Matrix8x4d tempZ_img;
  Eigen::Matrix8x4cd tempZ_comp;

  Eigen::Matrix4cd U11;
  Eigen::Matrix4cd U21;
  Eigen::Matrix4d X_sol;
  Eigen::Matrix4cd X;

  z11 = a.inverse();
  z12 = a.inverse()*b*r.inverse()*b.transpose();
  z21 = q*a.inverse();
  z22 = a.transpose() + q*a.inverse()*b*r.inverse()*b.transpose();
 // std::cout<<"c2" <<std::endl;

  z.setZero();
  z.topLeftCorner(n,n) = z11;
  z.topRightCorner(n,n) = z12;
  z.bottomLeftCorner(n,n) = z21;
  z.bottomRightCorner(n,n) = z22;

  for(int i=0; i<8; i++)
  {
    eigVec_real[i].resize(2*n);
    eigVec_real[i].setZero();
    eigVec_img[i].resize(2*n);
    eigVec_img[i].setZero();
  }
 // std::cout<<"c3" <<std::endl;

  deigVal_real.setZero();
  deigVal_img.setZero();
  deigVec_real.setZero();
  deigVec_img.setZero();

  deigVal_real = z.eigenvalues().real();
  deigVal_img = z.eigenvalues().imag();

  Eigen::EigenSolver<Eigen::MatrixXd> ev(z);
  //EigenVector Solver
  //Matrix3D ones = Matrix3D::Ones(3,3);
  //EigenSolver<Matrix3D> ev(ones);
  //cout << "The first eigenvector of the 3x3 matrix of ones is:" << endl << ev.eigenvectors().col(1) << endl;
//  std::cout<<"c3-1" <<std::endl;

  for(int i=0;i<2*n; i++)
  {
    for(int j=0; j<2*n; j++)
    {
      deigVec_real(j,i) = ev.eigenvectors().col(i)(j).real();
      deigVec_img(j,i) = ev.eigenvectors().col(i)(j).imag();
   //   std::cout<<"c3-2" <<std::endl;
    }
  }
//  std::cout<<"c4" <<std::endl;

  //Order the eigenvectors
  //move e-vectors correspnding to e-value outside the unite circle to the left

  tempZ_real.setZero();
  tempZ_img.setZero();
  int c=0;

  for (int i=0;i<2*n;i++)
  {
    if ((deigVal_real(i)*deigVal_real(i)+deigVal_img(i)*deigVal_img(i))>1.0) //outside the unit cycle
    {
      for(int j=0; j<2*n; j++)
      {
        tempZ_real(j,c) = deigVec_real(j,i);
        tempZ_img(j,c) = deigVec_img(j,i);
      }
      c++;

    }

    this_thread::sleep_for(chrono::milliseconds(100));

  }

  for(int i=0;i<2*n;i++)
  {
    for(int j=0;j<n;j++)
    {
      tempZ_comp.real()(i,j) = tempZ_real(i,j);
      tempZ_comp.imag()(i,j) = tempZ_img(i,j);
    }
  }
 // std::cout<<"c5" <<std::endl;

  for(int i=0;i<n;i++)
  {
    for(int j=0;j<n;j++)
    {
      U11(i,j) = tempZ_comp(i,j);
      U21(i,j) = tempZ_comp(i+n,j);
    }
  }
  X = U21*(U11.inverse());

  for(int i=0;i<n;i++)
  {
    for(int j=0;j<n;j++)
    {
      X_sol(i,j) = X.real()(i,j);
    }
  }
*/
  return X_sol;
}
//void WalkingController::Relative_link_position(Eigen::Isometry3d* transform_matrix){
//    ////////////////////////////
//    //// calculating the distance between adjacent two joint according to tree structure
//    ///

////    relative_distance_[RA_BEGIN] = transform_matrix[22].translation();//_T_RArm_global[0].translation();
////    relative_distance_[LA_BEGIN] = transform_matrix[15].translation();//_T_LArm_global[0].translation();
////    relative_distance_[RF_BEGIN] = transform_matrix[7].translation();
////    relative_distance_[LF_BEGIN] = transform_matrix[1].translation();
//    relative_distance_[RA_BEGIN] = transform_matrix[RA_LINK].translation();//RA_BEGIN
//    relative_distance_[LA_BEGIN] = transform_matrix[LA_LINK].translation();//LA_BEGIN
//    relative_distance_[RF_BEGIN] = transform_matrix[RF_LINK].translation(); // RF_BEGIN
//    relative_distance_[LF_BEGIN] = transform_matrix[LF_LINK].translation();//LF_BEGIN


//    //cout<<"check relative distance  ra begin : "<<_relative_distance[RA_BEGIN]<<endl;
//    for(int i=1;i<7;i++){
//       // relative_distance_[RA_BEGIN+i] = transform_matrix[RA_LINK+i].translation() - relative_distance_[RA_BEGIN + i-1];//_T_RArm_global[i].translation() - relative_distance_[RA_BEGIN + i-1];//Right arm
//       // relative_distance_[LA_BEGIN+i] = transform_matrix[LA_LINK+i].translation() - relative_distance_[LA_BEGIN + i-1];// Left arm
//       relative_distance_[RA_BEGIN+i] = transform_matrix[RA_LINK+i].translation() - transform_matrix[RA_LINK + i-1].translation();
//       relative_distance_[LA_BEGIN+i] = transform_matrix[LA_LINK+i].translation() - transform_matrix[LA_LINK + i-1].translation();
//    }

//    for(int i=1;i<6;i++){
////        relative_distance_[RF_BEGIN+i] = transform_matrix[RF_LINK+i].translation() - relative_distance_[RF_BEGIN + i-1];//Right foot
////        relative_distance_[LF_BEGIN+i] = transform_matrix[LF_LINK+i].translation() - relative_distance_[LF_BEGIN + i-1];//Left foot
//        relative_distance_[RF_BEGIN +i] = transform_matrix[RF_LINK+i].translation() - transform_matrix[RF_LINK + i-1].translation();
//        relative_distance_[LF_BEGIN +i] = transform_matrix[LF_LINK+i].translation() - transform_matrix[LF_LINK + i-1].translation();
//    }

//    relative_distance_[WA_BEGIN] = transform_matrix[WA_LINK].translation();
//    relative_distance_[WA_BEGIN+1] = transform_matrix[WA_LINK+1].translation() - transform_matrix[WA_LINK].translation();

////    if(walking_tick_ ==0){
////        cout<<"relative distance "<<endl;
////        for(int i=0;i<29;i++)
////            cout<<i<<"th relative distance "<<endl<<relative_distance_[i]<<endl;
////    }

//}
//void WalkingController::relative_link_trans_matrix(Eigen::Isometry3d* transform_matrix){
//    /// calculating transform matrix between adjecnt two joints
//    ///
//    ///     Based on the tree structure,
//    ///     RA          LA
//    ///        \       /
//    ///            WA
//    ///            |
//    ///           Base
//    ///           /   \\
//    ///        RLEG   LLEG



//    relative_link_transform_[RF_BEGIN] = transform_matrix[RF_LINK];
//    relative_link_transform_[LF_BEGIN] = transform_matrix[LF_LINK];
//    relative_link_transform_[WA_BEGIN] = transform_matrix[WA_LINK];

//    for(int i=1;i<6;i++){
//        relative_link_transform_[RF_BEGIN+i].linear() = (transform_matrix[RF_LINK+i-1].linear().transpose())*transform_matrix[RF_LINK+i].linear();
//        relative_link_transform_[RF_BEGIN+i].translation() = transform_matrix[RF_LINK+i-1].linear().transpose()*transform_matrix[RF_LINK+i].translation() - transform_matrix[RF_LINK+i-1].linear()*transform_matrix[RF_LINK+i-1].translation();

//        relative_link_transform_[LF_BEGIN+i].linear() = (transform_matrix[LF_LINK+i-1].linear().transpose())*transform_matrix[LF_LINK+i].linear();
//        relative_link_transform_[LF_BEGIN+i].translation() = transform_matrix[LF_LINK+i-1].linear().transpose()*transform_matrix[LF_LINK+i].translation() - transform_matrix[LF_LINK+i-1].linear()*transform_matrix[LF_LINK+i-1].translation();
//    }

//    // for waist and arm first joint

//    relative_link_transform_[WA_BEGIN+1].linear() = (transform_matrix[WA_LINK].linear().transpose())*transform_matrix[WA_LINK+1].linear();
//    relative_link_transform_[WA_BEGIN+1].translation() = transform_matrix[WA_LINK].linear().transpose()*transform_matrix[WA_LINK+1].translation() - transform_matrix[WA_LINK].linear()*transform_matrix[WA_LINK].translation();

//    // for both arm first joint

//    relative_link_transform_[RA_BEGIN].linear() = (transform_matrix[WA_LINK+1].linear().transpose())*transform_matrix[RA_LINK].linear();
//    relative_link_transform_[RA_BEGIN].translation() = transform_matrix[WA_LINK+1].linear().transpose()*transform_matrix[RA_LINK].translation() - transform_matrix[WA_LINK+1].linear()*transform_matrix[WA_LINK+1].translation();

//    relative_link_transform_[LA_BEGIN].linear() = (transform_matrix[WA_LINK+1].linear().transpose())*transform_matrix[LA_LINK].linear();
//    relative_link_transform_[LA_BEGIN].translation() = transform_matrix[WA_LINK+1].linear().transpose()*transform_matrix[LA_LINK].translation() - transform_matrix[WA_LINK+1].linear()*transform_matrix[WA_LINK+1].translation();

//    for(int i=1;i<7;i++){
//        relative_link_transform_[RA_BEGIN+i].linear() = (transform_matrix[RA_LINK+i-1].linear().transpose())*transform_matrix[RA_LINK+i].linear();
//        relative_link_transform_[RA_BEGIN+i].translation() = transform_matrix[RA_LINK+i-1].linear().transpose()*transform_matrix[RA_LINK+i].translation() - transform_matrix[RA_LINK+i-1].linear()*transform_matrix[RA_LINK+i-1].translation();

//        relative_link_transform_[LA_BEGIN+i].linear() = (transform_matrix[LA_LINK+i-1].linear().transpose())*transform_matrix[LA_LINK+i].linear();
//        relative_link_transform_[LA_BEGIN+i].translation() = transform_matrix[LA_LINK+i-1].linear().transpose()*transform_matrix[LA_LINK+i].translation() - transform_matrix[LA_LINK+i-1].linear()*transform_matrix[LA_LINK+i-1].translation();
//    }

//    for(int i=0;i<28;i++){
//        relative_rotation_[i] = relative_link_transform_[i].linear();
//        relative_distance_[i] = relative_link_transform_[i].translation();
//    }
////    file[14]<<walking_tick_;
////    for(int i=0;i<28;i++){
////        file[14]<<"\t"<<relative_distance_[i](0)<<"\t"<<relative_distance_[i](1)<<"\t"<<relative_distance_[i](2);
////    }
////    file[14]<<endl;

//}
//void WalkingController::Relative_link_rotation(Eigen::Isometry3d* transform_matrix){
//    ////////////////////////////
//    //// calculating the rotation matrix between adjacent two joint according to tree structure
//    ///

//////    cout<<"relative rotation : "<<_relative_rotation[WA_BEGIN]<<endl;
//////    cout<<"global rotation : "<<_T_Waist_global[0].linear()<<endl;
//////    cout<<"global rotation transpoise = "<<_relative_rotation[WA_BEGIN].transpose()*_T_Waist_global[0].linear()<<endl;


//    relative_rotation_[WA_BEGIN] = transform_matrix[WA_LINK].linear();
//    relative_rotation_[RA_BEGIN] = transform_matrix[RA_LINK].linear();
//    relative_rotation_[LA_BEGIN] = transform_matrix[LA_LINK].linear();
//    relative_rotation_[RF_BEGIN] = transform_matrix[RF_LINK].linear();
//    relative_rotation_[LF_BEGIN] = transform_matrix[LF_LINK].linear();

////    cout<<"relative rotation : "<<relative_rotation_[WA_BEGIN]<<endl;
////    cout<<"global rotation : "<<_T_Waist_global[0].linear()<<endl;
////    cout<<"global rotation transpoise = "<<relative_rotation_[WA_BEGIN].transpose()*_T_Waist_global[0].linear()<<endl;
//    for(int i=1;i<7;i++){
//        //relative_rotation_[RA_BEGIN+i] = relative_rotation_[RA_BEGIN + i-1].transpose()*transform_matrix[RA_LINK+i].linear();
//        //relative_rotation_[LA_BEGIN+i] = relative_rotation_[LA_BEGIN + i-1].transpose()*transform_matrix[LA_LINK+i].linear();
//        relative_rotation_[RA_BEGIN +i] = transform_matrix[RA_LINK + i-1].linear().transpose() * transform_matrix[RA_LINK + i].linear();
//        relative_rotation_[LA_BEGIN +i] = transform_matrix[LA_LINK + i-1].linear().transpose() * transform_matrix[LA_LINK + i].linear();
//    }

//    for(int i=1;i<6;i++){
////        relative_rotation_[RF_BEGIN+i] = relative_rotation_[RF_BEGIN + i-1].transpose()*transform_matrix[RF_LINK+i].linear();
////        relative_rotation_[LF_BEGIN+i] = relative_rotation_[LF_BEGIN + i-1].transpose()*transform_matrix[LF_LINK+i].linear();
//        relative_rotation_[RF_BEGIN +i] = transform_matrix[RF_LINK + i-1].linear().transpose() * transform_matrix[RF_LINK + i].linear();
//        relative_rotation_[LF_BEGIN +i] = transform_matrix[LF_LINK + i-1].linear().transpose() * transform_matrix[LF_LINK + i].linear();
//    }

//    relative_rotation_[WA_BEGIN+1] = transform_matrix[WA_LINK].linear().transpose()*transform_matrix[WA_LINK+1].linear();

////    cout<<"relative rotation wa : "<<_relative_rotation[WA_BEGIN]<<endl;
////    cout<<"relative rotation WA+1 : "<<_relative_rotation[WA_BEGIN+1]<<endl;

////    if(walking_tick_ == 0 ){
////        cout<<"relative rotation "<<endl;
////        for(int i=0;i<28;i++){
////            cout<<i<<"th rotation matrix"<<endl<<relative_rotation_[i]<<endl;
////        }
////    }
//}

//void WalkingController::Spatial_transform(){
//    //////////////////////// calculating spatial matrix //////////////
//    ///
//    ///       order : linear velocity and angular velocity
//    ///    i^X_p(i)     [ R    RS(p)]
//    ///                 [ 0       R ]
//    Eigen::Matrix<double, 3,3> Skew_temp, Rotation_temp;
//    Skew_temp.setIdentity();
//    Rotation_temp.setIdentity();

//    if(walking_tick_==0){
//        for(int i=0;i<28;i++){
//            Spatial_Matrix_[i].setZero();
//        }
//    }

//    for(int i=0;i<28;i++){
//        Skew_temp = DyrosMath::skew(relative_distance_[i]);
//        Rotation_temp = relative_rotation_[i]*Skew_temp.transpose();


//        Spatial_Matrix_[i].block<3,3>(0,0) = relative_rotation_[i];
//        Spatial_Matrix_[i].block<3,3>(0,3) = Rotation_temp;
//        Spatial_Matrix_[i].block<3,3>(3,3) = relative_rotation_[i];

////        if(walking_tick_ ==0)
////        cout<<i<<"th spatial matrix "<<endl<<Spatial_Matrix_[i]<<endl;
//    }
//}
//void WalkingController::Spatial_COM_transform(){
//    Eigen::Matrix<double, 3, 3> Skew_temp, Rotation_temp, Iden_3d;
//    Skew_temp.setIdentity();
//    Rotation_temp.setIdentity();
//    Iden_3d.setIdentity();

//    //Skew_temp = DyrosMath::skew(com_support_current_);
//    Skew_temp = DyrosMath::skew(com_float_current_);
//    //_COM_real_support

//    if(walking_tick_ == 0)
//        cout<<"com real position "<<com_support_current_<<endl;

////    for(int j=0;j<3;j++){
////        for(int k=0;k<3;k++){
////            Spatial_COM_Transform_[28](j,k) = Skew_temp(j,k);
////        }
////        Spatial_COM_Transform_[28](j+3,j) = 1.0;
////        Spatial_COM_Transform_[28](j,j+3) = 1.0;
////    }
//    Spatial_COM_Transform_[28].block<3,3>(0,0) = Iden_3d;
//    Spatial_COM_Transform_[28].block<3,3>(0,3) = Skew_temp;
//    Spatial_COM_Transform_[28].block<3,3>(3,0).setZero();
//    Spatial_COM_Transform_[28].block<3,3>(3,3).setIdentity();

//    Spatial_COM_Transform_[RF_BEGIN] = Spatial_Matrix_[RF_BEGIN]*Spatial_COM_Transform_[28];
//    Spatial_COM_Transform_[LF_BEGIN] = Spatial_Matrix_[LF_BEGIN]*Spatial_COM_Transform_[28];
//    Spatial_COM_Transform_[WA_BEGIN] = Spatial_Matrix_[WA_BEGIN]*Spatial_COM_Transform_[28];
//    Spatial_COM_Transform_[WA_BEGIN+1] = Spatial_Matrix_[WA_BEGIN+1]*Spatial_COM_Transform_[WA_BEGIN];

//    Spatial_COM_Transform_[RA_BEGIN] = Spatial_Matrix_[RA_BEGIN]*Spatial_COM_Transform_[WA_BEGIN+1];
//    Spatial_COM_Transform_[LA_BEGIN] = Spatial_Matrix_[LA_BEGIN]*Spatial_COM_Transform_[WA_BEGIN+1];


//    for(int i=1;i<7;i++){
//        Spatial_COM_Transform_[RA_BEGIN+i] = Spatial_Matrix_[RA_BEGIN+i]*Spatial_COM_Transform_[RA_BEGIN+i-1];
//        Spatial_COM_Transform_[LA_BEGIN+i] = Spatial_Matrix_[LA_BEGIN+i]*Spatial_COM_Transform_[LA_BEGIN+i-1];
//    }

//    for(int i=1;i<6;i++){
//        Spatial_COM_Transform_[RF_BEGIN+i] = Spatial_Matrix_[RF_BEGIN+i]*Spatial_COM_Transform_[RF_BEGIN+i-1];
//        Spatial_COM_Transform_[LF_BEGIN+i] = Spatial_Matrix_[LF_BEGIN+i]*Spatial_COM_Transform_[LF_BEGIN+i-1];
//    }


//}

//void WalkingController::Relative_Inertia(){
//    //////////////////////// calculating spatial inertia matrix //////////////
//    ///
//    ///       order : linear velocity and angular velocity
//    ///    I_i     [ m_i          m_i*S(c_i)^T]
//    ///            [ m_i*S(c_i)         bar_I ]
//    ///
//    ///    bar_I   = I^(cm)_i + m_i*S(c_i)*S(c_i) , bar_I  is rotational inertia(i)
//    Eigen::Matrix<double, 3, 3> Skew_temp, rotational_inertia_temp;
//    Skew_temp.setIdentity();
//    rotational_inertia_temp.setIdentity();

////    if(walking_tick_ == 0){
////        cout<<"llims mass in relative inertia func "<<endl;
////        for(int i=0;i<29;i++)
////            cout<<link_mass_[i]<<endl;
////    }

//    if(walking_tick_ == 0){
//        for(int i=0;i<29;i++)
//            relative_Inertia_[i].setZero();
//    }
//    for(int i=0;i<7;i++){// for arm link
//        Skew_temp = DyrosMath::skew(link_local_com_position_[RA_LINK+i]);//right arm

//        rotational_inertia_temp = link_inertia_[RA_LINK+i] + link_mass_[RA_LINK+i]*Skew_temp*Skew_temp.transpose();

//        for(int j=0;j<3;j++){
//            relative_Inertia_[RA_BEGIN+i](j,j) = link_mass_[RA_LINK+i];
//        }
//        relative_Inertia_[RA_BEGIN+i].block<3,3>(0,3) = link_mass_[RA_LINK+i]*Skew_temp.transpose();
//        relative_Inertia_[RA_BEGIN+i].block<3,3>(3,0)  = link_mass_[RA_LINK+i]*Skew_temp;
//        relative_Inertia_[RA_BEGIN+i].block<3,3>(3,3) = rotational_inertia_temp;

//        Skew_temp = DyrosMath::skew(link_local_com_position_[LA_LINK+i]);//left arm
//        rotational_inertia_temp = link_inertia_[LA_LINK+i] + link_mass_[LA_LINK+i]*Skew_temp*Skew_temp.transpose();

//        for(int j=0;j<3;j++){
//            relative_Inertia_[LA_BEGIN+i](j,j) = link_mass_[LA_LINK+i];
//        }

//        relative_Inertia_[LA_BEGIN+i].block<3,3>(0,3) = link_mass_[LA_LINK+i]*Skew_temp.transpose();
//        relative_Inertia_[LA_BEGIN+i].block<3,3>(3,0)  = link_mass_[LA_LINK+i]*Skew_temp;
//        relative_Inertia_[LA_BEGIN+i].block<3,3>(3,3) = rotational_inertia_temp;
//    }

//    for(int i=0;i<6;i++){// for foot link
//        Skew_temp = DyrosMath::skew(link_local_com_position_[RF_LINK+i]);//right foot
//        rotational_inertia_temp = link_inertia_[RF_LINK+i] + link_mass_[RF_LINK+i]*Skew_temp*Skew_temp.transpose();

//        for(int j=0;j<3;j++){
//            for(int k=0;k<3;k++){
//                relative_Inertia_[RF_BEGIN+i](j,k) = link_mass_[RF_LINK+i]*Skew_temp.transpose()(j,k);
//                relative_Inertia_[RF_BEGIN+i](j+3,k+3) = link_mass_[RF_LINK+i]*Skew_temp(j,k);
//                relative_Inertia_[RF_BEGIN+i](j+3,k) = rotational_inertia_temp(j,k);
//            }
//            relative_Inertia_[RF_BEGIN+i](j,j+3) = link_mass_[RF_LINK+i];
//        }

//        Skew_temp = DyrosMath::skew(link_local_com_position_[LF_LINK+i]);//left foot
//        rotational_inertia_temp = link_inertia_[LF_LINK+i] + link_mass_[LF_LINK+i]*Skew_temp*Skew_temp.transpose();

//        for(int j=0;j<3;j++){
//            for(int k=0;k<3;k++){
//                relative_Inertia_[LF_BEGIN+i](j,k) = link_mass_[LF_LINK+i]*Skew_temp.transpose()(j,k);
//                relative_Inertia_[LF_BEGIN+i](j+3,k+3) = link_mass_[LF_LINK+i]*Skew_temp(j,k);
//                relative_Inertia_[LF_BEGIN+i](j+3,k) = rotational_inertia_temp(j,k);
//            }
//            relative_Inertia_[LF_BEGIN+i](j,j+3) = link_mass_[LF_LINK+i];
//        }
//    }

//    Skew_temp = DyrosMath::skew(link_local_com_position_[WA_LINK]);//waist
//    rotational_inertia_temp = link_inertia_[WA_LINK] + link_mass_[WA_LINK]*Skew_temp*Skew_temp.transpose();

//    for(int j=0;j<3;j++){
//        for(int k=0;k<3;k++){
//            relative_Inertia_[WA_BEGIN](j,k) = link_mass_[WA_LINK]*Skew_temp.transpose()(j,k);
//            relative_Inertia_[WA_BEGIN](j+3,k+3) = link_mass_[WA_LINK]*Skew_temp(j,k);
//            relative_Inertia_[WA_BEGIN](j+3,k) = rotational_inertia_temp(j,k);
//        }
//        relative_Inertia_[WA_BEGIN](j,j+3) = link_mass_[WA_LINK];
//    }

//    Skew_temp = DyrosMath::skew(link_local_com_position_[WA_LINK+1]);//waist +1
//    rotational_inertia_temp = link_inertia_[WA_LINK+1] + link_mass_[WA_LINK+1]*Skew_temp*Skew_temp.transpose();

//    for(int j=0;j<3;j++){
//        for(int k=0;k<3;k++){
//            relative_Inertia_[WA_BEGIN+1](j,k) = link_mass_[WA_LINK+1]*Skew_temp.transpose()(j,k);
//            relative_Inertia_[WA_BEGIN+1](j+3,k+3) = link_mass_[WA_LINK+1]*Skew_temp(j,k);
//            relative_Inertia_[WA_BEGIN+1](j+3,k) = rotational_inertia_temp(j,k);
//        }
//        relative_Inertia_[WA_BEGIN+1](j,j+3) = link_mass_[WA_LINK+1];
//    }

//    Skew_temp = DyrosMath::skew(link_local_com_position_[0]);//mass of pelvis
//    rotational_inertia_temp = link_inertia_[0] + link_mass_[0]*Skew_temp*(Skew_temp.transpose());

//    for(int j=0;j<3;j++){
//        for(int k=0;k<3;k++){
//            relative_Inertia_[28](j,k) = link_mass_[0]*Skew_temp.transpose()(j,k);
//            relative_Inertia_[28](j+3,k+3) = link_mass_[0]*Skew_temp(j,k);
//            relative_Inertia_[28](j+3,k) = rotational_inertia_temp(j,k);
//        }
//        relative_Inertia_[28](j,j+3) = link_mass_[0];
//    }

//}
//void WalkingController::New_Relative_Inertia()
//{
//    //////////////////////// calculating spatial inertia matrix //////////////
//    ///
//    ///       order : linear velocity and angular velocity
//    ///    I_i     [ m_i          m_i*S(c_i)^T]
//    ///            [ m_i*S(c_i)         bar_I ]
//    ///
//    ///    bar_I   = I^(cm)_i + m_i*S(c_i)*S(c_i) , bar_I  is rotational inertia(i)
//    ///

//    Eigen::Matrix<double, 3, 3> Skew_temp, rotational_inertia_link;
//    Skew_temp.setZero(); rotational_inertia_link.setZero();

//    Eigen::Matrix3d Iden_3d;
//    Iden_3d.setIdentity();

//    if(walking_tick_ == 0){
//        for(int i=0;i<29;i++)
//            relative_Inertia_[i].setZero();
//    }

//    for(int i=0;i<28;i++){
//        Skew_temp = DyrosMath::skew(link_local_com_position_[i+1]); // input link is one order high than ouput joint-begin

//        rotational_inertia_link = link_inertia_[1+i] + link_mass_[i+1]*Skew_temp*(Skew_temp.transpose());

//        relative_Inertia_[i].block<3,3>(0,0) = link_mass_[1+i]*Iden_3d;
//        relative_Inertia_[i].block<3,3>(0,3) = link_mass_[1+i]*(Skew_temp.transpose());
//        relative_Inertia_[i].block<3,3>(3,0) = link_mass_[1+i]*Skew_temp;
//        relative_Inertia_[i].block<3,3>(3,3) = rotational_inertia_link;
//    }

//    Skew_temp = DyrosMath::skew(link_local_com_position_[0]);
//    rotational_inertia_link = link_inertia_[0] + link_mass_[0]*(Skew_temp*Skew_temp.transpose());

//    relative_Inertia_[28].block<3,3>(0,0) = link_mass_[0]*Iden_3d;
//    relative_Inertia_[28].block<3,3>(0,3) = link_mass_[0]*(Skew_temp.transpose());
//    relative_Inertia_[28].block<3,3>(3,0) = link_mass_[0]*Skew_temp;
//    relative_Inertia_[28].block<3,3>(3,3) = rotational_inertia_link;

//}
//void WalkingController::Spatial_Inertia(){

//    //////////////////////// calculating spatial inertia matrix //////////////
//    ///
//    ///       order : linear velocity and angular velocity
//    ///    I^c _p(i) = I^c _p(i) + X^i_p(i).transpose()*I^c_i*X^i_p(i)
//    ///
//    ///
//    ///
//    //cout<<"Spatial inertia function :"<<endl;

//    for(int i=0;i<29;i++)
//        Spatial_Inertia_[i] = relative_Inertia_[i];

//    for(int i=7; 1<= i; i--){
//        Spatial_Inertia_[RA_BEGIN+i-1] = Spatial_Inertia_[RA_BEGIN+i-1] + Spatial_Matrix_[RA_BEGIN+i].transpose()*Spatial_Inertia_[RA_BEGIN+i]*Spatial_Matrix_[RA_BEGIN+i];
//        Spatial_Inertia_[LA_BEGIN+i-1] = Spatial_Inertia_[LA_BEGIN+i-1] + Spatial_Matrix_[LA_BEGIN+i].transpose()*Spatial_Inertia_[LA_BEGIN+i]*Spatial_Matrix_[LA_BEGIN+i];
//    }
//    for(int i=6; 1<= i; i--){
//        Spatial_Inertia_[RF_BEGIN+i-1] = Spatial_Inertia_[RF_BEGIN+i-1] + Spatial_Matrix_[RF_BEGIN+i].transpose()*Spatial_Inertia_[RF_BEGIN+i]*Spatial_Matrix_[RF_BEGIN+i];
//        Spatial_Inertia_[LF_BEGIN+i-1] = Spatial_Inertia_[LF_BEGIN+i-1] + Spatial_Matrix_[LF_BEGIN+i].transpose()*Spatial_Inertia_[LF_BEGIN+i]*Spatial_Matrix_[LF_BEGIN+i];
//    }
//    Spatial_Inertia_[WA_BEGIN+1] = Spatial_Inertia_[WA_BEGIN+1] + Spatial_Matrix_[RA_BEGIN].transpose()*Spatial_Inertia_[RA_BEGIN]*Spatial_Matrix_[RA_BEGIN] + Spatial_Matrix_[LA_BEGIN].transpose()*Spatial_Inertia_[LA_BEGIN]*Spatial_Matrix_[LA_BEGIN];
//    Spatial_Inertia_[WA_BEGIN] = Spatial_Inertia_[WA_BEGIN] +Spatial_Matrix_[WA_BEGIN+1].transpose()*Spatial_Inertia_[WA_BEGIN+1]*Spatial_Matrix_[WA_BEGIN+1];

////    cout<<"spartial inertia RA_BEGIN : "<<Spatial_Inertia_[RA_BEGIN]<<endl;
////    cout<<"spartial inertia LA_BEGIN : "<<Spatial_Inertia_[LA_BEGIN]<<endl;
////    cout<<"spartial inertia WA_BEGIn : "<<Spatial_Inertia_[WA_BEGIN]<<endl;

//    //Spatial_Inertia_[28] += Spatial_Matrix_[RA_BEGIN].transpose()*Spatial_Inertia_[RA_BEGIN]*Spatial_Matrix_[RA_BEGIN];
//    //Spatial_Inertia_[28] += Spatial_Matrix_[LA_BEGIN].transpose()*Spatial_Inertia_[LA_BEGIN]*Spatial_Matrix_[LA_BEGIN];
//    Spatial_Inertia_[28] += Spatial_Matrix_[RF_BEGIN].transpose()*Spatial_Inertia_[RF_BEGIN]*Spatial_Matrix_[RF_BEGIN];
//    Spatial_Inertia_[28] += Spatial_Matrix_[LF_BEGIN].transpose()*Spatial_Inertia_[LF_BEGIN]*Spatial_Matrix_[LF_BEGIN];
//    Spatial_Inertia_[28] += Spatial_Matrix_[WA_BEGIN].transpose()*Spatial_Inertia_[WA_BEGIN]*Spatial_Matrix_[WA_BEGIN];
//}
//void WalkingController::Centroidal_Momentum_Matrix(){

//    ///////////////////////////////////////////////////////////////
//    /// calculating the centroidal momentum matrx A_G
//    ///  A_G(i) = i_X_p(i).transpose*I_i_toCoM*joint_axis
//    ///  Joint_axis is expressed in local coordinates, Joint_axis
//    ///
//    ///
//    Eigen::Vector6d joint_axis;
//    joint_axis.setZero();
//    Eigen::Vector6d joint_axis_local[28];
//    for(int i=0;i<28;i++)
//        joint_axis_local[i].setZero();

//    //left leg - right leg
//    joint_axis_local[0](5) = -1; joint_axis_local[1](3) = 1;  joint_axis_local[2](4) = 1;  joint_axis_local[3](4) =  1; joint_axis_local[4](4) =  1;  joint_axis_local[5](3) =  1;
//    joint_axis_local[6](5) = -1; joint_axis_local[7](3) = 1; joint_axis_local[8](4) = -1; joint_axis_local[9](4) = -1; joint_axis_local[10](4) = -1; joint_axis_local[11](3) = 1;

//    //waist
//    joint_axis_local[12](5) = 1; joint_axis_local[13](3) = -1;

//    //left arm - right arm
//    joint_axis_local[14](4) = 1;    joint_axis_local[15](3) = 1;    joint_axis_local[16](4) = 1;    joint_axis_local[17](3) = 1;    joint_axis_local[18](4) = 1;    joint_axis_local[19](3) = 1;    joint_axis_local[20](4) = 1;
//    joint_axis_local[21](4) = -1;    joint_axis_local[22](3) = 1;    joint_axis_local[23](4) = -1;    joint_axis_local[24](3) = 1;    joint_axis_local[25](4) = -1;    joint_axis_local[26](3) = 1;    joint_axis_local[27](4) = -1;

//    q_dot_2 = (p_q_ - current_q_)*hz_;

//    //q_dot_ = current_q_dot_;
//    q_dot_.setZero();
//    q_dot_.segment<12>(0) = desired_leg_q_dot_;

//    for(int i=0;i<7;i++){
//        for(int j=3;j<6;j++){
//            joint_axis(j)=current_arm_jacobian_r_(j,i);
//        }
//        //cout<<joint_axis<<endl;

//        Centroidal_Momentum_Matrix_[RA_BEGIN+i] = Spatial_COM_Transform_[RA_BEGIN+i].transpose()*Spatial_Inertia_[RA_BEGIN+i]*joint_axis;

//        for(int j=3;j<6;j++){
//            joint_axis(j)=current_arm_jacobian_l_(j,i);
//        }

//        Centroidal_Momentum_Matrix_[LA_BEGIN+i] = Spatial_COM_Transform_[LA_BEGIN+i].transpose()*Spatial_Inertia_[LA_BEGIN+i]*joint_axis;
//    }

//    for(int i=0;i<6;i++){
//        for(int j=3;j<6;j++){
//            joint_axis(j)=current_leg_jacobian_r_(j,i);
//        }

//        Centroidal_Momentum_Matrix_[RF_BEGIN+i] = Spatial_COM_Transform_[RF_BEGIN+i].transpose()*Spatial_Inertia_[RF_BEGIN+i]*joint_axis;

//        for(int j=3;j<6;j++){
//            joint_axis(j)=current_leg_jacobian_l_(j,i);
//        }

//        Centroidal_Momentum_Matrix_[LF_BEGIN+i] = Spatial_COM_Transform_[LF_BEGIN+i].transpose()*Spatial_Inertia_[LF_BEGIN+i]*joint_axis;
//    }
//    for(int i=0;i<2;i++){
//        for(int j=3;j<6;j++){
//            joint_axis(j) = current_waist_jacobian_[i](j,1);
//        }


//       // cout<<"waist axis : "<<i<<" " <<joint_axis<<endl;
//        Centroidal_Momentum_Matrix_[WA_BEGIN+i] = Spatial_COM_Transform_[WA_BEGIN+i].transpose()*Spatial_Inertia_[WA_BEGIN+i]*joint_axis;
//    }

////    for(int i=0;i<28;i++){
////        Centroidal_Momentum_Matrix_[i] = Spatial_COM_Transform_[i].transpose()*Spatial_Inertia_[i]*joint_axis_local[i];
////    }

////    file[36]<<walking_tick_<<"\t"<<Spatial_COM_Transform_[RF_BEGIN](3)<<"\t"<<Spatial_COM_Transform_[RF_BEGIN](4)<<"\t"<<Spatial_COM_Transform_[RF_BEGIN](5)//1,2~4
////                           <<"\t"<<Spatial_COM_Transform_[RF_BEGIN+1](3)<<"\t"<<Spatial_COM_Transform_[RF_BEGIN+1](4)<<"\t"<<Spatial_COM_Transform_[RF_BEGIN+1](5) //5~7
////                           <<"\t"<<Spatial_COM_Transform_[RF_BEGIN+2](3)<<"\t"<<Spatial_COM_Transform_[RF_BEGIN+2](4)<<"\t"<<Spatial_COM_Transform_[RF_BEGIN+2](5) //8~10
////                           <<"\t"<<Spatial_COM_Transform_[RF_BEGIN+3](3)<<"\t"<<Spatial_COM_Transform_[RF_BEGIN+3](4)<<"\t"<<Spatial_COM_Transform_[RF_BEGIN+3](5) //11~13
////                           <<"\t"<<Spatial_COM_Transform_[RF_BEGIN+4](3)<<"\t"<<Spatial_COM_Transform_[RF_BEGIN+4](4)<<"\t"<<Spatial_COM_Transform_[RF_BEGIN+4](5) //14~16
////                           <<"\t"<<Spatial_COM_Transform_[RF_BEGIN+5](3)<<"\t"<<Spatial_COM_Transform_[RF_BEGIN+5](4)<<"\t"<<Spatial_COM_Transform_[RF_BEGIN+5](5)  //17~19
////                           <<"\t"<<Spatial_COM_Transform_[LF_BEGIN](3)<<"\t"<<Spatial_COM_Transform_[LF_BEGIN](4)<<"\t"<<Spatial_COM_Transform_[LF_BEGIN](5) //20~22
////                           <<"\t"<<Spatial_COM_Transform_[LF_BEGIN+1](3)<<"\t"<<Spatial_COM_Transform_[LF_BEGIN+1](4)<<"\t"<<Spatial_COM_Transform_[LF_BEGIN+1](5) //23~25
////                           <<"\t"<<Spatial_COM_Transform_[LF_BEGIN+2](3)<<"\t"<<Spatial_COM_Transform_[LF_BEGIN+2](4)<<"\t"<<Spatial_COM_Transform_[LF_BEGIN+2](5)//26~28
////                           <<"\t"<<Spatial_COM_Transform_[LF_BEGIN+3](3)<<"\t"<<Spatial_COM_Transform_[LF_BEGIN+3](4)<<"\t"<<Spatial_COM_Transform_[LF_BEGIN+3](5)//29~31
////                           <<"\t"<<Spatial_COM_Transform_[LF_BEGIN+4](3)<<"\t"<<Spatial_COM_Transform_[LF_BEGIN+4](4)<<"\t"<<Spatial_COM_Transform_[LF_BEGIN+4](5)//32~35
////                           <<"\t"<<Spatial_COM_Transform_[LF_BEGIN+5](3)<<"\t"<<Spatial_COM_Transform_[LF_BEGIN+5](4)<<"\t"<<Spatial_COM_Transform_[LF_BEGIN+5](5)//36~38

//    file[36]<<walking_tick_;
//    for(int i=0;i<6;i++){
//        for(int j=0;j<3;j++){
//            for(int k=0;k<3;k++){
//                file[36]<<"\t"<<Spatial_COM_Transform_[RF_BEGIN+i](j+3,k+3);
//            }
//        }
//    }
//    for(int i=0;i<6;i++){
//        for(int j=0;j<3;j++){
//            for(int k=0;k<3;k++){
//                file[36]<<"\t"<<Spatial_COM_Transform_[LF_BEGIN+i](j+3,k+3);
//            }
//        }
//    }
//    file[36]<<endl;

//    Centroidal_Momentum_[28].setZero();
////    for(int i=0;i<28;i++){
//////        cout<<"spatial inertia : "<<i <<"  "<<Spatial_Inertia_[i]<<endl;
//////        cout<<"Spatial com transform : "<<i << "  " <<Spatial_COM_Transform_[i]<<endl;
//////        cout<<"centorida momentum matrix "<<i<< "  " <<Centroidal_Momentum_Matrix_[i]<<endl;
////        Centroidal_Momentum_[i] = Centroidal_Momentum_Matrix_[i] *q_dot_(i);
////        //Centroidal_Momentum_Matrix_[28] += Centroidal_Momentum_Matrix_[i]*q_dot_(i); //h_G (Centroidal momentum)
////        Centroidal_Momentum_[28] += Centroidal_Momentum_[i];
////    }
//    Eigen::Matrix6d CCM_LLeg, CCM_RLeg;
//    CCM_LLeg.setZero(); CCM_RLeg.setZero();

//    for(int i=0;i<6;i++){
//        CCM_LLeg.col(i) = Centroidal_Momentum_Matrix_[i];
//        CCM_RLeg.col(i) = Centroidal_Momentum_Matrix_[6+i];
//    }
//    Eigen::Matrix6d kp; // for setting CLIK gains
//    kp.setZero();
//    kp(0,0) = 200;
//    kp(1,1) = 200;
//    kp(2,2) = 200;
//    kp(3,3) = 250;
//    kp(4,4) = 250;
//    kp(5,5) = 250;

//    Eigen::Vector6d CM_lleg, CM_rleg;
//    CM_lleg.setZero(); CM_rleg.setZero();

////    if(walking_tick_<t_start_real_+t_double1_ || walking_tick_>= t_start_+t_total_-t_double2_)
////    {
////        lp_.setZero();
////        rp_.setZero();
////    }

//    CM_lleg = CCM_LLeg*current_leg_jacobian_l_inv_*kp*lp_;
//    CM_rleg = CCM_RLeg*current_leg_jacobian_r_inv_*kp*rp_;


//    cout<<"Centroial momentum check : "<<endl<<"cm leg   "<<CM_lleg<<endl<<"CCM_lleg "<<CCM_LLeg<<endl;

//    Centroidal_Momentum_[28] = CM_lleg + CM_rleg;

//    file[15]<<walking_tick_;
//    for(int i=0;i<6;i++){
//        file[15]<<"\t"<<Centroidal_Momentum_[28](i);
//    }

//    file[15]<<"\t"<<CM_lleg(5)<<"\t"<<CM_rleg(5)<<endl;


//}
//void WalkingController::Centroidal_Dynamics(){
// ////// refer Centroidal dynamics of Humanoid robot , David E. Orin, Ambarish Goswami, Sung-Hee Lee, 'Auton Robot, 2013'
// /// ' https://link.springer.com/content/pdf/10.1007/s10514-013-9341-4.pdf '
// ///

//    Relative_link_position(link_transform_);
//    Relative_link_rotation(link_transform_);
////    relative_link_trans_matrix(link_transform_);
//    Spatial_transform();



////    Relative_Inertia();
//    New_Relative_Inertia();
//    Spatial_Inertia();

//    Spatial_COM_transform();
//    //if(_cnt==0)

//    Centroidal_Momentum_Matrix();
//    //cout<<"centroidal dynamics : "<<_m_R[6]<<endl;

//    //cout<<"check relative_distance : "<<_relative_distance[2]<<", _T_arm_global "<<_T_RArm_global[2].translation()<<endl;
//    Eigen::Vector6d Centroidal_Momentum_matrix_dot[29];
//    Eigen::Vector6d Centroidal_Momentum_dot[29];

//    for(int i=0;i<29;i++){
//        Centroidal_Momentum_matrix_dot[i] = (Centroidal_Momentum_Matrix_[i] - pre_Centroidal_Momentum_Matrix_[i])/hz_; // last CMM_dot[27] is h_g_dot (whole centroidal momentum dot)
//        Centroidal_Momentum_dot[i] = Centroidal_Momentum_[i] - pre_Centroidal_Momentum_[i];
//    }

//    CM_R_leg_.setZero(); CM_L_leg_.setZero();


////    for(int i=0;i<6;i++){
////        CM_L_leg_ += Centroidal_Momentum_Matrix_[LF_BEGIN+i] * desired_leg_q_dot_filtered_(i)/hz_;
////        file[15]<<"\t"<<Centroidal_Momentum_Matrix_[LF_BEGIN+i](5) * desired_leg_q_dot_filtered_(i)/hz_;
////    }
//////    Eigen::Vector6d RF_CM;
//////    RF_CM = Spatial_COM_Transform_[RF_BEGIN+5].transepose()**rp_;
////    for(int i=0;i<6;i++){
//////        CMM_R_leg_dot += Centroidal_Momentum_matrix_dot[RF_BEGIN+i]*_q_RFoot_dot(i);
//////        CMM_L_leg_dot += Centroidal_Momentum_matrix_dot[LF_BEGIN+i]*_q_LFoot_dot(i);
////        //CM_R_leg_ += Centroidal_Momentum_Matrix_[RF_BEGIN+i] * desired_leg_q_dot_(6+i);
////        //CM_L_leg_ += Centroidal_Momentum_Matrix_[LF_BEGIN+i] * desired_leg_q_dot_(i);
////        CM_R_leg_ += Centroidal_Momentum_Matrix_[RF_BEGIN+i] * desired_leg_q_dot_filtered_(6+i)/hz_;
////        file[15]<<"\t"<<Centroidal_Momentum_Matrix_[RF_BEGIN+i](5)* desired_leg_q_dot_filtered_(6+i)/hz_;

////        //CM_L_leg_ += Centroidal_Momentum_Matrix_[LF_BEGIN+i] * desired_leg_q_dot_filtered_(i);
////    }

////    for(int i=0;i<12;i++)
////        file[15]<<"\t"<<desired_leg_q_dot_filtered_(i);

////        file[15]<<endl;


//    Eigen::Vector6d momentum_temp;
//    momentum_temp.setZero();

//    Eigen::Matrix<double, 6, 6> Gain;
//    Gain.setIdentity();
//    Gain *= 20.0*Gain;

//    //momentum_temp = Gain*(-Centroidal_Momentum_Matrix_[28]) - CMM_R_leg_dot - CMM_L_leg_dot;
//    //momentum_temp = Centroidal_Momentum_[28] -CMM_R_leg_dot - CMM_L_leg_dot;
//    momentum_temp =  - CM_R_leg_ - CM_L_leg_;

//    double temp_dot;
//    //temp_dot = CMM_dot[WA_BEGIN](5);
//    temp_dot = Centroidal_Momentum_Matrix_[WA_BEGIN](5);

//    Eigen::Matrix3d temp_A;
//    Eigen::Vector3d momentum_3_size;

//    for(int i=0;i<3;i++){
//        temp_A(i,0) = Centroidal_Momentum_Matrix_[WA_BEGIN](i+3);
//        temp_A(i,1) = Centroidal_Momentum_Matrix_[RA_BEGIN](i+3);
//        temp_A(i,2) = Centroidal_Momentum_Matrix_[LA_BEGIN](i+3);

//        momentum_3_size(i) = momentum_temp(i+3);
//    }
////    cout<<"waist momentum : "<<Centroidal_Momentum_Matrix_[WA_BEGIN]<<endl;// ---why the value is small ??
////    cout<<"Rarm momentum : "<<Centroidal_Momentum_Matrix_[RA_BEGIN]<<endl;
////    cout<<"Larm momentum : "<<Centroidal_Momentum_Matrix_[LA_BEGIN]<<endl;
////    cout<<"momentum temp : "<<momentum_temp<<endl;
////    cout<<"waist momentum dot : "<<CMM_dot[WA_BEGIN]<<endl;

//    Eigen::Vector3d joint_dot_temp;

//    joint_dot_temp = temp_A.inverse() * momentum_3_size;
////    q_dot_CMM_(0) = momentum_temp(5)/(temp_dot*100);

//    q_dot_CMM_(0) = joint_dot_temp(0);
//    q_dot_CMM_(1) = joint_dot_temp(1);
//    q_dot_CMM_(2) = joint_dot_temp(2);

//   // file[15]<<walking_tick_<<"\t"<<joint_dot_temp(0)<<"\t"<<joint_dot_temp(1)<<"\t"<<joint_dot_temp(2)<<endl;

////    file[18]<<_cnt<<"\t"<<q_dot_CMM_(0)/hz_<<"\t"<<1<<"\t"<<momentum_temp(5);
////    file[22]<<_cnt<<"\t"<<CM_R_leg_(0)<<"\t"<<CM_R_leg_(1)<<"\t"<<CM_R_leg_(2)<<"\t"<<CM_R_leg_(3)<<"\t"<<CM_R_leg_(4)<<"\t"<<CM_R_leg_(5)<<endl;
////    file[23]<<_cnt<<"\t"<<CM_L_leg_(0)<<"\t"<<CM_L_leg_(1)<<"\t"<<CM_L_leg_(2)<<"\t"<<CM_L_leg_(3)<<"\t"<<CM_L_leg_(4)<<"\t"<<CM_L_leg_(5)<<endl;
////    file[24]<<_cnt<<"\t"<<momentum_temp(0)<<"\t"<<momentum_temp(1)<<"\t"<<momentum_temp(2)<<"\t"<<momentum_temp(3)<<"\t"<<momentum_temp(4)<<"\t"<<momentum_temp(5)<<endl;
////    cout<<"waist yaw dot : "<<q_dot_CMM_<<endl;

//    //calculate CCRBI(Centoridal composite rigid body inertia

//    CCRBI_ = Spatial_COM_Transform_[28].transpose()*Spatial_Inertia_[28]*Spatial_COM_Transform_[28];
//    average_spatial_velocity_ = CCRBI_.inverse()*Centroidal_Momentum_[28];

//}
void WalkingController::qpOASES_example(){


    for(int i=0;i<3;i++){
        lb_[i] = -20.0;
        ub_[i] = 20.0;
    }

    /* Setting up QProblem object. */
    //SQProblem example( 9,6 );
    QProblemB example(3); // for simply Bounded QPs # of variables
    //QProblem example(3,1, HST_ZERO);

    Options myOptions;
    myOptions.printLevel= PL_NONE;
    example.setOptions(myOptions);



    getOptimizationInputMatrix2();


    int_t nWSR = 100;


    example.init(H_,g_,lb_,ub_,nWSR);
    example.hotstart(g_,lb_,ub_,nWSR);

    //example.init(0,g_,A_,lb_, ub_,lbA_, ubA_,nWSR,0);
    //example.hotstart(g_,lb_, ub_,lbA_, ubA_, nWSR,0);


//    example.init(H_,g_,A_,lb_, ub_, lbA_, ubA_,nWSR);
//    example.hotstart(g_,lb_, ub_, lbA_, ubA_,nWSR);

    example.getPrimalSolution(xOpt_);




//     /* Get and print solution of first QP. */
//     real_t xOpt[9];
//     real_t yOpt[2+1];
//     example.getPrimalSolution( xOpt );
//     //example.getDualSolution( yOpt );
//    printf( "\nxOpt = [ %e, %e, %e, %e, %e, %e,%e, %e, %e];  objVal = %e\n\n",
//            xOpt_[0],xOpt_[1],xOpt_[2],xOpt_[3],xOpt_[4],xOpt_[5],xOpt_[6],xOpt_[7],xOpt_[8],example.getObjVal() );
//    printf( "\nxOpt = [ %e, %e, %e, %e, %e, %e];  objVal = %e\n\n",
//           xOpt_[0],xOpt_[1],xOpt_[2], xOpt_[3],xOpt_[4],xOpt_[5], example.getObjVal() );

    //_pre_q_dot(WA_BEGIN) = xOpt__[0];
//    pre_q_dot_(WA_BEGIN) = xOpt_[0];//_q_dot_CMM(0);
//    pre_q_dot_(RA_BEGIN) = xOpt_[1];
//    pre_q_dot_(LA_BEGIN) = xOpt_[2];


    q_dot_(WA_BEGIN) = DyrosMath::lowPassFilter(xOpt_[0],pre_q_dot_(WA_BEGIN),1.0/hz_,0.05);
    q_dot_(RA_BEGIN) = DyrosMath::lowPassFilter(xOpt_[1],pre_q_dot_(RA_BEGIN),1.0/hz_,0.05);
    q_dot_(LA_BEGIN) = DyrosMath::lowPassFilter(xOpt_[2],pre_q_dot_(LA_BEGIN),1.0/hz_,0.05);

    //pre_q_dot_ = q_dot_;


//    pre_q_dot_(WA_BEGIN) = xOpt_[0];
//    pre_q_dot_(RA_BEGIN) = xOpt_[1];    pre_q_dot_(RA_BEGIN+1) = xOpt_[2];     pre_q_dot_(RA_BEGIN+2) = xOpt_[3];    pre_q_dot_(RA_BEGIN+4) = xOpt_[4];
//    pre_q_dot_(LA_BEGIN) = xOpt_[5];    pre_q_dot_(LA_BEGIN+1) = xOpt_[6];     pre_q_dot_(LA_BEGIN+2) = xOpt_[7];    pre_q_dot_(LA_BEGIN+4) = xOpt_[8];


    //file[16]<<walking_tick_<<"\t"<<xOpt_[0]<<"\t"<<xOpt_[1]<<"\t"<<xOpt_[2]<<"\t"<<q_dot_(WA_BEGIN)<<"\t"<<q_dot_(RA_BEGIN)<<"\t"<<q_dot_(LA_BEGIN)<<"\t"<<xOpt_[6]<<"\t"<<xOpt_[7]<<"\t"<<xOpt_[8]<<"\t"<<example.getObjVal()<<endl;
    //file[29]<<_cnt<<"\t"<<xOpt[0]<<"\t"<<


}
void WalkingController:: getOptimizationInputMatrix(){

  // MatrixXd A_c(3,9);
   Eigen::Matrix3d A_c;
 //  Eigen::Matrix<double, 3, 5> A_c;
   A_c.setZero();



   for(int i=0;i<3;i++){
       A_c(i,0) = 10*Centroidal_Momentum_Matrix_[WA_BEGIN](i+3);
       A_c(i,1) = 10*Centroidal_Momentum_Matrix_[RA_BEGIN](i+3);
       A_c(i,2) = 10*Centroidal_Momentum_Matrix_[LA_BEGIN](i+3);
//       A_c(i,2) = 100*Centroidal_Momentum_Matrix_[RA_BEGIN+1](i+3);
//       A_c(i,3) = 100*Centroidal_Momentum_Matrix_[LA_BEGIN](i+3);
//       A_c(i,4) = 100*Centroidal_Momentum_Matrix_[LA_BEGIN+1](i+3);
   }
   //A_c = 30.0*A_c;
//    MatrixXd A_c(1,9);
//    A_c.setZero();

//    A_c(0) = _Centroidal_Momentum_Matrix[WA_BEGIN](5);
//    for(int i=0;i<3;i++){
//     A_c(1+i) = _Centroidal_Momentum_Matrix[RA_BEGIN+i](5);
//     A_c(5+i) = _Centroidal_Momentum_Matrix[LA_BEGIN+i](5);
//    }

//    A_c(4) = _Centroidal_Momentum_Matrix[RA_BEGIN+4](5);
//    A_c(8) = _Centroidal_Momentum_Matrix[LA_BEGIN+4](5);



   Eigen::Matrix3d H_temp;
   H_temp.setZero();




   Eigen::Vector6d Y_temp, pre_CM_c, pre_CM_leg, CM_dot_desired;
   Y_temp.setZero(); pre_CM_c.setZero(); pre_CM_leg.setZero(); CM_dot_desired.setZero();

   pre_CM_c = pre_Centroidal_Momentum_Matrix_[WA_BEGIN]*pre_q_dot_(WA_BEGIN);

   pre_CM_c += pre_Centroidal_Momentum_Matrix_[RA_BEGIN]*pre_q_dot_(RA_BEGIN) + pre_Centroidal_Momentum_Matrix_[LA_BEGIN]*pre_q_dot_(LA_BEGIN);


//    for(int i=0;i<3;i++){
//       pre_CM_c += _pre_Centroidal_Momentum_Matrix[RA_BEGIN+i]*_pre_q_dot(RA_BEGIN+i) + _pre_Centroidal_Momentum_Matrix[LA_BEGIN+i]*_pre_q_dot(LA_BEGIN+i);
//    }
//    pre_CM_c += _pre_Centroidal_Momentum_Matrix[RA_BEGIN+4]*_pre_q_dot(RA_BEGIN+4) + _pre_Centroidal_Momentum_Matrix[LA_BEGIN+4]*_pre_q_dot(LA_BEGIN+4);

   //pre_CM_leg = (CM_R_leg_ - pre_CM_R_leg_ ) + (CM_L_leg_ - pre_CM_L_leg_);
   pre_CM_leg = DyrosMath::lowPassFilter(CM_R_leg_,pre_CM_R_leg_,1.0/hz_,0.2) + DyrosMath::lowPassFilter(CM_L_leg_, pre_CM_L_leg_, 1.0/hz_,0.2);


   file[19]<<walking_tick_//1
          <<"\t"<<pre_Centroidal_Momentum_Matrix_[WA_BEGIN](3)<<"\t"<<pre_Centroidal_Momentum_Matrix_[WA_BEGIN](4)<<"\t"<<pre_Centroidal_Momentum_Matrix_[WA_BEGIN](5)//234
          <<"\t"<<pre_Centroidal_Momentum_Matrix_[RA_BEGIN](3)<<"\t"<<pre_Centroidal_Momentum_Matrix_[RA_BEGIN](4)<<"\t"<<pre_Centroidal_Momentum_Matrix_[RA_BEGIN](5)//567
          <<"\t"<<pre_Centroidal_Momentum_Matrix_[LA_BEGIN](3)<<"\t"<<pre_Centroidal_Momentum_Matrix_[LA_BEGIN](4)<<"\t"<<pre_Centroidal_Momentum_Matrix_[LA_BEGIN](5)//8910
          <<"\t"<<pre_q_dot_[WA_BEGIN]<<"\t"<<pre_q_dot_[RA_BEGIN]<<"\t"<<pre_q_dot_[LA_BEGIN]//11 12 13
          <<"\t"<<CM_R_leg_(3)<<"\t"<<CM_R_leg_(4)<<"\t"<<CM_R_leg_(5)<<"\t"<<CM_L_leg_(3)<<"\t"<<CM_L_leg_(4)<<"\t"<<CM_L_leg_(5)//14 15 16 17 18 19
          <<"\t"<<pre_CM_R_leg_(3)<<"\t"<<pre_CM_R_leg_(4)<<"\t"<<pre_CM_R_leg_(5)<<"\t"<<pre_CM_L_leg_(3)<<"\t"<<pre_CM_L_leg_(4)<<"\t"<<pre_CM_L_leg_(5)//20 21 22 23 24 25
          <<"\t"<<pre_CM_leg(3)<<"\t"<<pre_CM_leg(4)<<"\t"<<pre_CM_leg(5)<<endl;//26 27 28
   Eigen::Vector6d Momentum_Gain;
   Momentum_Gain.setOnes();


//   Momentum_Gain(3) = 10.0;
//   Momentum_Gain(4) = 10.0;
//   Momentum_Gain(5) = 10.0;
   for(int i=0;i<3;i++){
       CM_dot_desired(i) =Momentum_Gain(i)*(com_dot_desired_(i) - average_spatial_velocity_(i)) + Momentum_Gain(i+3) * (com_desired_(i) -com_support_current_(i));
       CM_dot_desired(i+3) =Momentum_Gain(i+3)*(-Centroidal_Momentum_[28](i+3));
   }



//   cout<<"desired com velocity "<<endl<<com_dot_desired_<<endl;
//   cout<<"average spatial velocity : "<<endl<<average_spatial_velocity_<<endl;

//   cout<<"desired com position : "<<endl<<com_desired_<<endl;
//   cout<<"current com position : "<<endl<<com_support_current_<<endl;

   /////////////////// Y Vector ////////////////////////
   Y_temp = CM_dot_desired - pre_CM_leg + pre_CM_c;
   ////////////////////////////////////////////////////

   file[18]<<walking_tick_<<"\t"<<Y_temp(3)<<"\t"<<Y_temp(4)<<"\t"<<Y_temp(5)<<"\t"<<CM_dot_desired(3)<<"\t"<<CM_dot_desired(4)<<"\t"<<CM_dot_desired(5)<<"\t"<<pre_CM_leg(3)<<"\t"<<pre_CM_leg(4)<<"\t"<<pre_CM_leg(5)<<"\t"<<pre_CM_c(3)<<"\t"<<pre_CM_c(4)<<"\t"<<pre_CM_c(5)
          <<"\t"<<Centroidal_Momentum_[28](3)<<"\t"<<Centroidal_Momentum_[28](4)<<"\t"<<Centroidal_Momentum_[28](5)<<endl;

   Eigen::Vector3d    g_temp;
   g_temp.setZero();

   Eigen::Vector3d    y_temp_3;

   for(int i=0;i<3;i++){
       y_temp_3(i) = Y_temp(i+3);
   }

   ///////////////////////g Vector////////////////////////21`
   g_temp = - A_c.transpose()*y_temp_3;


//   Eigen::Vector3d  check_value;
//   check_value = A_c.inverse()*y_temp_3;
//   Eigen::Matrix3d A_ci;
//   A_ci = A_c.inverse();

////   q_dot_CMM_(0) = check_value(0);
////   q_dot_CMM_(1) = check_value(1);
////   q_dot_CMM_(2) = check_value(2);


   file[20]<<walking_tick_<<"\t"<<A_c(0,0)<<"\t"<<A_c(0,1)<<"\t"<<A_c(0,2)<<"\t"<<A_c(1,0)<<"\t"<<A_c(1,1)<<"\t"<<A_c(1,2)<<"\t"<<A_c(2,0)<<"\t"<<A_c(2,1)<<"\t"<<A_c(2,2)<<endl;
          //<<"\t"<<A_ci(0,0)<<"\t"<<A_ci(0,1)<<"\t"<<A_ci(0,2)<<"\t"<<A_ci(1,0)<<"\t"<<A_ci(1,1)<<"\t"<<A_ci(1,2)<<"\t"<<A_ci(2,0)<<"\t"<<A_ci(2,1)<<"\t"<<A_ci(2,2)<<endl;
//   file[17]<<walking_tick_<<"\t"<<check_value(0)<<"\t"<<check_value(1)<<"\t"<<check_value(2)<<endl;

   double K_Gain;
   double weight;
   if(walking_tick_ >= t_start_real_){
       if(walking_tick_ >= t_start_real_+t_double1_ && walking_tick_ < t_start_+t_total_-t_double2_-t_rest_last_){
           weight = 0.9;
       K_Gain = 100.0;
       }
       else{
           weight = 0.0;
           K_Gain = 100.0;
       }

   }
   else
       weight = 0.0;

//q_init_

   Eigen::Matrix3d Identity;
   Identity.setIdentity();

//   for(int i=0;i<3;i++){
//       for(int j=0;j<3;j++){
//           A_c(i,j) = weight*A_c(i,j) + (1-weight)*Identity(i,j);
//       }
//   }

   ////////////////// H matrix ////////////////////////
   H_temp = A_c.transpose()*A_c;
   //////////////////////////////////////////////////
   ///

      for(int i=0;i<3;i++){
          for(int j=0;j<3;j++){
              H_temp(i,j) = weight*H_temp(i,j) + (1-weight)*Identity(i,j);
          }
      }


   /// matchig parameter /////

   for(int j=0;j<3;j++){
       for(int i=0;i<3;i++){
           H_[j*3+i] = H_temp(i,j);
       }
   }
   double torso_dot;
   torso_dot = (Y_temp(5) / A_c(2,0))/hz_;

//   file[28]<<walking_tick_<<"\t"<<torso_dot<<"\t"<<Y_temp(5)<<"\t"<<A_c(0)<<"\t"<<CM_dot_desired(5)<<"\t"<<pre_CM_leg(5)<<"\t"<<pre_CM_c(5)<<endl;


//   file[18]<<"\t"<<2<<"\t"<<Y_temp(5)<<endl;


//    cout<<"check H "<<endl;
//    for(int i=0;i<81;i++)
//        cout<<"  "<<H[i]<<" ";

//    cout<<endl<<"check H_temp"<<H_temp<<endl;


   K_Gain =1.0;

   g_temp(0) = weight*g_temp(0) - (1-weight) *K_Gain*(q_init_(WA_BEGIN) - current_q_(WA_BEGIN));
   g_temp(1) = weight*g_temp(1) - (1-weight) *K_Gain*(q_init_(RA_BEGIN) - current_q_(RA_BEGIN));
   g_temp(2) = weight*g_temp(2) - (1-weight) *K_Gain*(q_init_(LA_BEGIN) - current_q_(LA_BEGIN));
//   g_temp(2) = weight*g_temp(2) - (1-weight) *K_Gain*(q_init_(RA_BEGIN+1) - current_q_(RA_BEGIN+1));
//   g_temp(3) = weight*g_temp(3) - (1-weight) *K_Gain*(q_init_(LA_BEGIN) - current_q_(LA_BEGIN));
//   g_temp(4) = weight*g_temp(4) - (1-weight) *K_Gain*(q_init_(LA_BEGIN+1) - current_q_(LA_BEGIN+1));
//   cout<<"init q of waist , right arm, left arm : "<<q_init_(WA_BEGIN)<<", "<<q_init_(RA_BEGIN)<<", "<<q_init_(LA_BEGIN)<<endl;
//   cout<<"curr q of waist , right arm, left arm : "<<current_q_(WA_BEGIN)<<", "<<current_q_(RA_BEGIN)<<", "<<current_q_(LA_BEGIN)<<endl;

//   cout<<"init waist : "<<q_init_(WA_BEGIN)<<", current waist : "<<current_q_(WA_BEGIN)<<endl;

   for(int i=0;i<3;i++)
       g_[i] = g_temp(i);

//   for(int j=0;j<3;j++){
//       for(int i=0;i<3;i++)
//           //A_[j*3+i] = A_c(i,j);
//   }

}
void WalkingController:: getOptimizationInputMatrix2(){


   Eigen::Matrix3d A_c;
   A_c.setZero();



   for(int i=0;i<3;i++){
       A_c(i,0) = 10*Centroidal_Momentum_Matrix_[WA_BEGIN](i+3);
       A_c(i,1) = 10*Centroidal_Momentum_Matrix_[RA_BEGIN](i+3);
       A_c(i,2) = 10*Centroidal_Momentum_Matrix_[LA_BEGIN](i+3);
   }
   //A_c *= hz_;


   Eigen::Matrix3d H_temp;
   H_temp.setZero();




   Eigen::Vector6d Y_temp, CM_leg_dot, pre_CM_torso, CM_dot_desired, cm_leg;
   Y_temp.setZero(); CM_leg_dot.setZero(); pre_CM_torso.setZero(); CM_dot_desired.setZero(); cm_leg;


    if(walking_tick_ >= t_start_real_+t_double1_ && walking_tick_ < t_start_+t_total_-t_double2_-t_rest_last_){
       if(foot_step_(current_step_num_,6) == 1){// right foot swing
           CM_leg_dot = (CM_R_leg_ - pre_CM_R_leg_)*hz_;
           cm_leg = CM_R_leg_;
           }
       //left foot support
       else if (foot_step_(current_step_num_,6) == 0){ //left foot swing
           CM_leg_dot = (CM_L_leg_ - pre_CM_L_leg_)*hz_;
           cm_leg = CM_L_leg_;
       }
    }
    else{
        CM_leg_dot.setZero();
        cm_leg.setZero();
    }

    file[19]<<walking_tick_<<"\t"<<Centroidal_Momentum_Matrix_[RF_BEGIN](3)<<"\t"<<Centroidal_Momentum_Matrix_[RF_BEGIN](4)<<"\t"<<Centroidal_Momentum_Matrix_[RF_BEGIN](5)//1,2~4
                           <<"\t"<<Centroidal_Momentum_Matrix_[RF_BEGIN+1](3)<<"\t"<<Centroidal_Momentum_Matrix_[RF_BEGIN+1](4)<<"\t"<<Centroidal_Momentum_Matrix_[RF_BEGIN+1](5) //5~7
                           <<"\t"<<Centroidal_Momentum_Matrix_[RF_BEGIN+2](3)<<"\t"<<Centroidal_Momentum_Matrix_[RF_BEGIN+2](4)<<"\t"<<Centroidal_Momentum_Matrix_[RF_BEGIN+2](5) //8~10
                           <<"\t"<<Centroidal_Momentum_Matrix_[RF_BEGIN+3](3)<<"\t"<<Centroidal_Momentum_Matrix_[RF_BEGIN+3](4)<<"\t"<<Centroidal_Momentum_Matrix_[RF_BEGIN+3](5) //11~13
                           <<"\t"<<Centroidal_Momentum_Matrix_[RF_BEGIN+4](3)<<"\t"<<Centroidal_Momentum_Matrix_[RF_BEGIN+4](4)<<"\t"<<Centroidal_Momentum_Matrix_[RF_BEGIN+4](5) //14~16
                           <<"\t"<<Centroidal_Momentum_Matrix_[RF_BEGIN+5](3)<<"\t"<<Centroidal_Momentum_Matrix_[RF_BEGIN+5](4)<<"\t"<<Centroidal_Momentum_Matrix_[RF_BEGIN+5](5)  //17~19
                           <<"\t"<<desired_leg_q_dot_(6)<<"\t"<<desired_leg_q_dot_(7)<<"\t"<<desired_leg_q_dot_(8)<<"\t"<<desired_leg_q_dot_(9)<<"\t"<<desired_leg_q_dot_(10)<<"\t"<<desired_leg_q_dot_(11) //20~25
                           <<"\t"<<Centroidal_Momentum_Matrix_[LF_BEGIN](3)<<"\t"<<Centroidal_Momentum_Matrix_[LF_BEGIN](4)<<"\t"<<Centroidal_Momentum_Matrix_[LF_BEGIN](5) //26~28
                           <<"\t"<<Centroidal_Momentum_Matrix_[LF_BEGIN+1](3)<<"\t"<<Centroidal_Momentum_Matrix_[LF_BEGIN+1](4)<<"\t"<<Centroidal_Momentum_Matrix_[LF_BEGIN+1](5) //29~31
                           <<"\t"<<Centroidal_Momentum_Matrix_[LF_BEGIN+2](3)<<"\t"<<Centroidal_Momentum_Matrix_[LF_BEGIN+2](4)<<"\t"<<Centroidal_Momentum_Matrix_[LF_BEGIN+2](5)//32~34
                           <<"\t"<<Centroidal_Momentum_Matrix_[LF_BEGIN+3](3)<<"\t"<<Centroidal_Momentum_Matrix_[LF_BEGIN+3](4)<<"\t"<<Centroidal_Momentum_Matrix_[LF_BEGIN+3](5)//35~37
                           <<"\t"<<Centroidal_Momentum_Matrix_[LF_BEGIN+4](3)<<"\t"<<Centroidal_Momentum_Matrix_[LF_BEGIN+4](4)<<"\t"<<Centroidal_Momentum_Matrix_[LF_BEGIN+4](5)//38~40
                           <<"\t"<<Centroidal_Momentum_Matrix_[LF_BEGIN+5](3)<<"\t"<<Centroidal_Momentum_Matrix_[LF_BEGIN+5](4)<<"\t"<<Centroidal_Momentum_Matrix_[LF_BEGIN+5](5)//41~43
                           <<"\t"<<desired_leg_q_dot_(0)<<"\t"<<desired_leg_q_dot_(1)<<"\t"<<desired_leg_q_dot_(2)<<"\t"<<desired_leg_q_dot_(3)<<"\t"<<desired_leg_q_dot_(4)<<"\t"<<desired_leg_q_dot_(5)<<endl;//44~49

     //CM_leg_dot = ((CM_R_leg_ + CM_L_leg_ ) - (pre_CM_R_leg_ + pre_CM_L_leg_))/hz_;

     //cm_leg = CM_R_leg_ + CM_L_leg_;

     CM_leg_dot_ = DyrosMath::lowPassFilter(CM_leg_dot, pre_CM_leg_dot_,1.0/hz_, 0.05);

     pre_CM_torso = (Left_arm_CM_ + Right_arm_CM_ + waist_CM_);

//   file[19]<<walking_tick_//1
//          <<"\t"<<pre_Centroidal_Momentum_Matrix_[WA_BEGIN](3)<<"\t"<<pre_Centroidal_Momentum_Matrix_[WA_BEGIN](4)<<"\t"<<pre_Centroidal_Momentum_Matrix_[WA_BEGIN](5)//234
//          <<"\t"<<pre_Centroidal_Momentum_Matrix_[RA_BEGIN](3)<<"\t"<<pre_Centroidal_Momentum_Matrix_[RA_BEGIN](4)<<"\t"<<pre_Centroidal_Momentum_Matrix_[RA_BEGIN](5)//567
//          <<"\t"<<pre_Centroidal_Momentum_Matrix_[LA_BEGIN](3)<<"\t"<<pre_Centroidal_Momentum_Matrix_[LA_BEGIN](4)<<"\t"<<pre_Centroidal_Momentum_Matrix_[LA_BEGIN](5)//8910
//          <<"\t"<<pre_q_dot_[WA_BEGIN]<<"\t"<<pre_q_dot_[RA_BEGIN]<<"\t"<<pre_q_dot_[LA_BEGIN]//11 12 13
//          <<"\t"<<CM_R_leg_(3)<<"\t"<<CM_R_leg_(4)<<"\t"<<CM_R_leg_(5)<<"\t"<<CM_L_leg_(3)<<"\t"<<CM_L_leg_(4)<<"\t"<<CM_L_leg_(5)//14 15 16 17 18 19
//          <<"\t"<<pre_CM_R_leg_(3)<<"\t"<<pre_CM_R_leg_(4)<<"\t"<<pre_CM_R_leg_(5)<<"\t"<<pre_CM_L_leg_(3)<<"\t"<<pre_CM_L_leg_(4)<<"\t"<<pre_CM_L_leg_(5)//20 21 22 23 24 25
//          <<"\t"<<pre_CM_leg(3)<<"\t"<<pre_CM_leg(4)<<"\t"<<pre_CM_leg(5)<<endl;//26 27 28
   Eigen::Vector6d Momentum_Gain;
   Momentum_Gain.setOnes();


//   Momentum_Gain(3) = 10.0;
//   Momentum_Gain(4) = 10.0;
   Momentum_Gain(5) = 1.0;
   for(int i=0;i<3;i++){
       CM_dot_desired(i) =Momentum_Gain(i)*(com_dot_desired_(i) - average_spatial_velocity_(i)) + Momentum_Gain(i+3) * (com_desired_(i) -com_support_current_(i));
       CM_dot_desired(i+3) =Momentum_Gain(i+3)*(-Centroidal_Momentum_[28](i+3));
   }



//   cout<<"desired com velocity "<<endl<<com_dot_desired_<<endl;
//   cout<<"average spatial velocity : "<<endl<<average_spatial_velocity_<<endl;

//   cout<<"desired com position : "<<endl<<com_desired_<<endl;
//   cout<<"current com position : "<<endl<<com_support_current_<<endl;

   /////////////////// Y Vector ////////////////////////
   //Y_temp = - CM_R_leg_ - CM_L_leg_;// -CM_dot_;
    //Y_temp = CM_R_leg_ + CM_L_leg_;// -CM_dot_;
   Y_temp = CM_dot_desired - CM_leg_dot_ + pre_CM_torso;
   ////////////////////////////////////////////////////
   //Y_temp = 1.75*cm_leg;

   file[18]<<walking_tick_<<"\t"<<Y_temp(3)<<"\t"<<Y_temp(4)<<"\t"<<Y_temp(5)<<"\t"<<CM_dot_desired(3)<<"\t"<<CM_dot_desired(4)<<"\t"<<CM_dot_desired(5)
          <<"\t"<<CM_leg_dot_(3)<<"\t"<<CM_leg_dot_(4)<<"\t"<<CM_leg_dot_(5)
          <<"\t"<<pre_CM_torso(3)<<"\t"<<pre_CM_torso(4)<<"\t"<<pre_CM_torso(5)<<"\t"<<cm_leg(3)<<"\t"<<cm_leg(4)<<"\t"<<cm_leg(5)<<"\t"<<Centroidal_Momentum_[28](3)<<"\t"<<Centroidal_Momentum_[28](4)<<"\t"<<Centroidal_Momentum_[28](5)<<endl;

   file[30]<<walking_tick_<<"\t"<<CM_R_leg_(3)<<"\t"<<CM_R_leg_(4)<<"\t"<<CM_R_leg_(5)<<"\t"<<CM_L_leg_(3)<<"\t"<<CM_L_leg_(4)<<"\t"<<CM_L_leg_(5)<<"\t"<<cm_leg(3)<<"\t"<<cm_leg(4)<<"\t"<<cm_leg(5)<<endl;

   Eigen::Vector3d    g_temp;
   g_temp.setZero();

   Eigen::Vector3d    y_temp_3;

   for(int i=0;i<3;i++){
       y_temp_3(i) = Y_temp(i+3);
       y_temp_(i) = Y_temp(i+3);
       file[34]<<y_temp_(i)<<"\t";
   }
   file[34]<<endl;
   for(int i=0;i<3;i++){
       for(int j=0;j<3;j++)
           A_c_(i,j) = A_c(i,j);
   }

   ///////////////////////g Vector////////////////////////21`
   g_temp =  -A_c.transpose()*y_temp_3;

  // g_temp = A_c.transpose()*y_temp_3;
   //g_temp = y_temp_3.transpose()*A_c;
//   Eigen::Vector3d  check_value;
//   check_value = A_c.inverse()*y_temp_3;
//   Eigen::Matrix3d A_ci;
//   A_ci = A_c.inverse();

//////   q_dot_CMM_(0) = check_value(0);
//////   q_dot_CMM_(1) = check_value(1);
//////   q_dot_CMM_(2) = check_value(2);

//   q_dot_(WA_BEGIN) = DyrosMath::lowPassFilter(check_value(0),pre_q_dot_(WA_BEGIN),1.0/hz_,0.2);
//   q_dot_(RA_BEGIN) = DyrosMath::lowPassFilter(check_value(1),pre_q_dot_(RA_BEGIN),1.0/hz_,0.2);
//   q_dot_(LA_BEGIN) = DyrosMath::lowPassFilter(check_value(2),pre_q_dot_(LA_BEGIN),1.0/hz_,0.2);

   file[20]<<walking_tick_<<"\t"<<A_c(0,0)<<"\t"<<A_c(0,1)<<"\t"<<A_c(0,2)<<"\t"<<A_c(1,0)<<"\t"<<A_c(1,1)<<"\t"<<A_c(1,2)<<"\t"<<A_c(2,0)<<"\t"<<A_c(2,1)<<"\t"<<A_c(2,2)<<endl;
          //<<"\t"<<A_ci(0,0)<<"\t"<<A_ci(0,1)<<"\t"<<A_ci(0,2)<<"\t"<<A_ci(1,0)<<"\t"<<A_ci(1,1)<<"\t"<<A_ci(1,2)<<"\t"<<A_ci(2,0)<<"\t"<<A_ci(2,1)<<"\t"<<A_ci(2,2)<<endl;
//   file[17]<<walking_tick_<<"\t"<<check_value(0)<<"\t"<<check_value(1)<<"\t"<<check_value(2)<<endl;

   double K_Gain;
   double weight;
   if(walking_tick_ >= t_start_real_){
       if(walking_tick_ >= t_start_real_+t_double1_ && walking_tick_ < t_start_+t_total_-t_double2_-t_rest_last_){
           weight = 0.9;
       K_Gain = 100.0;
       }
       else{
           weight = 0.1;
           K_Gain = 100.0;
       }

   }
   else
       weight = 0.0;

//q_init_

   Eigen::Matrix3d Identity;
   Identity.setIdentity();

//   for(int i=0;i<3;i++){
//       for(int j=0;j<3;j++){
//           A_c(i,j) = weight*A_c(i,j) + (1-weight)*Identity(i,j);
//       }
//   }

   ////////////////// H matrix ////////////////////////
   H_temp = A_c.transpose()*A_c;
   //////////////////////////////////////////////////
   ///

      for(int i=0;i<3;i++){
          for(int j=0;j<3;j++){
              H_temp(i,j) = weight*H_temp(i,j) + (1-weight)*Identity(i,j);
          }
      }


   /// matchig parameter /////

   for(int j=0;j<3;j++){
       for(int i=0;i<3;i++){
           H_[j*3+i] = H_temp(i,j);

           file[31]<<H_temp(i,j)<<"\t";
       }
   }
   file[31]<<endl;

   double torso_dot;
   torso_dot = (Y_temp(5) / A_c(2,0))/hz_;

//   file[28]<<walking_tick_<<"\t"<<torso_dot<<"\t"<<Y_temp(5)<<"\t"<<A_c(0)<<"\t"<<CM_dot_desired(5)<<"\t"<<pre_CM_leg(5)<<"\t"<<pre_CM_c(5)<<endl;


//   file[18]<<"\t"<<2<<"\t"<<Y_temp(5)<<endl;


//    cout<<"check H "<<endl;
//    for(int i=0;i<81;i++)
//        cout<<"  "<<H[i]<<" ";

//    cout<<endl<<"check H_temp"<<H_temp<<endl;


   K_Gain =1.0;

    g_temp(0) = weight*g_temp(0) - (1-weight) *K_Gain*(q_init_(WA_BEGIN) - current_q_(WA_BEGIN));
    g_temp(1) = weight*g_temp(1) - (1-weight) *K_Gain*(q_init_(RA_BEGIN) - current_q_(RA_BEGIN));
    g_temp(2) = weight*g_temp(2) - (1-weight) *K_Gain*(q_init_(LA_BEGIN) - current_q_(LA_BEGIN));

//   g_temp(2) = weight*g_temp(2) - (1-weight) *K_Gain*(q_init_(RA_BEGIN+1) - current_q_(RA_BEGIN+1));
//   g_temp(3) = weight*g_temp(3) - (1-weight) *K_Gain*(q_init_(LA_BEGIN) - current_q_(LA_BEGIN));
//   g_temp(4) = weight*g_temp(4) - (1-weight) *K_Gain*(q_init_(LA_BEGIN+1) - current_q_(LA_BEGIN+1));
//   cout<<"init q of waist , right arm, left arm : "<<q_init_(WA_BEGIN)<<", "<<q_init_(RA_BEGIN)<<", "<<q_init_(LA_BEGIN)<<endl;
//   cout<<"curr q of waist , right arm, left arm : "<<current_q_(WA_BEGIN)<<", "<<current_q_(RA_BEGIN)<<", "<<current_q_(LA_BEGIN)<<endl;

//   cout<<"init waist : "<<q_init_(WA_BEGIN)<<", current waist : "<<current_q_(WA_BEGIN)<<endl;

   for(int i=0;i<3;i++){
       g_[i] = g_temp(i);
       //g_[i] = y_temp_3(i);
       file[32]<<g_temp(i)<<"\t";
    }
   file[32]<<endl;

//   for(int j=0;j<3;j++){
//       for(int i=0;i<3;i++){
//           A_[j*3+i] = A_c(i,j);
//       file[33]<<A_c(i,j)<<"\t";
//       }
//   }
//   file[33]<<endl;


   for(int i=0;i<3;i++){
       lbA_[i] = -cm_leg(i+3);
       ubA_[i] = -cm_leg(i+3);
   }

//   Eigen::Vector3d  temp;

//   temp = -A_c.inverse()*g_temp;

//   q_dot_(WA_BEGIN) = -g_temp(0)/A_c(0,0);
//   q_dot_(RA_BEGIN) = temp(1);
//   q_dot_(LA_BEGIN) = temp(2);

}
//void WalkingController:: getOptimizationInputMatrix9(){

//  // MatrixXd A_c(3,9);

//   Eigen::Matrix<double, 3, 9> A_c;
//   A_c.setZero();


//   for(int i=0;i<3;i++){
//       A_c(i,0) = 100*Centroidal_Momentum_Matrix_[WA_BEGIN](i+3);
//       for(int j=0;j<3;j++){
//            A_c(i,1+j) = 100*Centroidal_Momentum_Matrix_[RA_BEGIN+j](i+3);
//            A_c(i,5+j) = 100*Centroidal_Momentum_Matrix_[LA_BEGIN+j](i+3);
//       }
//       A_c(i,4) = Centroidal_Momentum_Matrix_[RA_BEGIN+4](i+3);
//       A_c(i,8) = Centroidal_Momentum_Matrix_[LA_BEGIN+4](i+3);
//   }


//   Eigen::Matrix<double, 9, 9> H_temp;
//   H_temp.setZero();



//   Eigen::Vector6d Y_temp, pre_CM_c, pre_CM_leg, CM_dot_desired;
//   Y_temp.setZero(); pre_CM_c.setZero(); pre_CM_leg.setZero(); CM_dot_desired.setZero();

//   pre_CM_c = pre_Centroidal_Momentum_Matrix_[WA_BEGIN]*pre_q_dot_(WA_BEGIN);

//   for(int i=0;i<3;i++){
//        pre_CM_c += pre_Centroidal_Momentum_Matrix_[RA_BEGIN+i]*pre_q_dot_(RA_BEGIN+i) + pre_Centroidal_Momentum_Matrix_[LA_BEGIN+i]*pre_q_dot_(LA_BEGIN+i);
//   }
//   pre_CM_c += pre_Centroidal_Momentum_Matrix_[RA_BEGIN+4]*pre_q_dot_(RA_BEGIN+4) + pre_Centroidal_Momentum_Matrix_[LA_BEGIN+4]*pre_q_dot_(LA_BEGIN+4);


//   //pre_CM_leg = (CM_R_leg_ - pre_CM_R_leg_ ) + (CM_L_leg_ - pre_CM_L_leg_);
//   pre_CM_leg = DyrosMath::lowPassFilter(CM_R_leg_,pre_CM_R_leg_,1.0/hz_,0.05) + DyrosMath::lowPassFilter(CM_L_leg_, pre_CM_L_leg_, 1.0/hz_,0.05);


//   file[19]<<walking_tick_//1
//          <<"\t"<<pre_Centroidal_Momentum_Matrix_[WA_BEGIN](3)<<"\t"<<pre_Centroidal_Momentum_Matrix_[WA_BEGIN](4)<<"\t"<<pre_Centroidal_Momentum_Matrix_[WA_BEGIN](5)//234
//          <<"\t"<<pre_Centroidal_Momentum_Matrix_[RA_BEGIN](3)<<"\t"<<pre_Centroidal_Momentum_Matrix_[RA_BEGIN](4)<<"\t"<<pre_Centroidal_Momentum_Matrix_[RA_BEGIN](5)//567
//          <<"\t"<<pre_Centroidal_Momentum_Matrix_[LA_BEGIN](3)<<"\t"<<pre_Centroidal_Momentum_Matrix_[LA_BEGIN](4)<<"\t"<<pre_Centroidal_Momentum_Matrix_[LA_BEGIN](5)//8910
//          <<"\t"<<pre_q_dot_[WA_BEGIN]<<"\t"<<pre_q_dot_[RA_BEGIN]<<"\t"<<pre_q_dot_[LA_BEGIN]//11 12 13
//          <<"\t"<<CM_R_leg_(3)<<"\t"<<CM_R_leg_(4)<<"\t"<<CM_R_leg_(5)<<"\t"<<CM_L_leg_(3)<<"\t"<<CM_L_leg_(4)<<"\t"<<CM_L_leg_(5)//14 15 16 17 18 19
//          <<"\t"<<pre_CM_R_leg_(3)<<"\t"<<pre_CM_R_leg_(4)<<"\t"<<pre_CM_R_leg_(5)<<"\t"<<pre_CM_L_leg_(3)<<"\t"<<pre_CM_L_leg_(4)<<"\t"<<pre_CM_L_leg_(5)//20 21 22 23 24 25
//          <<"\t"<<pre_CM_leg(3)<<"\t"<<pre_CM_leg(4)<<"\t"<<pre_CM_leg(5)<<endl;//26 27 28
//   Eigen::Vector6d Momentum_Gain;
//   Momentum_Gain.setOnes();


//   Momentum_Gain(3) = 100.0;
//   Momentum_Gain(4) = 100.0;
//   Momentum_Gain(5) = 100.0;
//   for(int i=0;i<3;i++){
//       CM_dot_desired(i) =Momentum_Gain(i)*(com_dot_desired_(i) - average_spatial_velocity_(i)) + Momentum_Gain(i+3) * (com_desired_(i) -com_support_current_(i));
//       CM_dot_desired(i+3) =Momentum_Gain(i+3)*(-Centroidal_Momentum_[28](i+3));
//   }



////   cout<<"desired com velocity "<<endl<<com_dot_desired_<<endl;
////   cout<<"average spatial velocity : "<<endl<<average_spatial_velocity_<<endl;

////   cout<<"desired com position : "<<endl<<com_desired_<<endl;
////   cout<<"current com position : "<<endl<<com_support_current_<<endl;

//   /////////////////// Y Vector ////////////////////////
//   Y_temp = CM_dot_desired - pre_CM_leg + pre_CM_c;
//   ////////////////////////////////////////////////////

//   file[18]<<walking_tick_<<"\t"<<Y_temp(3)<<"\t"<<Y_temp(4)<<"\t"<<Y_temp(5)<<"\t"<<CM_dot_desired(3)<<"\t"<<CM_dot_desired(4)<<"\t"<<CM_dot_desired(5)<<"\t"<<pre_CM_leg(3)<<"\t"<<pre_CM_leg(4)<<"\t"<<pre_CM_leg(5)<<"\t"<<pre_CM_c(3)<<"\t"<<pre_CM_c(4)<<"\t"<<pre_CM_c(5)
//          <<"\t"<<Centroidal_Momentum_[28](3)<<"\t"<<Centroidal_Momentum_[28](4)<<"\t"<<Centroidal_Momentum_[28](5)<<endl;

//   Eigen::Vector9d    g_temp;
//   g_temp.setZero();

//   Eigen::Vector3d    y_temp_3;

//   for(int i=0;i<3;i++){
//       y_temp_3(i) = Y_temp(i+3);
//   }

//   ///////////////////////g Vector////////////////////////21`
//   g_temp = - A_c.transpose()*y_temp_3;


////   Eigen::Vector3d  check_value;
////   check_value = A_c.inverse()*y_temp_3;
////   Eigen::Matrix3d A_ci;
////   A_ci = A_c.inverse();


//   file[20]<<walking_tick_<<"\t"<<A_c(0,0)<<"\t"<<A_c(0,1)<<"\t"<<A_c(0,2)<<"\t"<<A_c(1,0)<<"\t"<<A_c(1,1)<<"\t"<<A_c(1,2)<<"\t"<<A_c(2,0)<<"\t"<<A_c(2,1)<<"\t"<<A_c(2,2)<<endl;
//          //<<"\t"<<A_ci(0,0)<<"\t"<<A_ci(0,1)<<"\t"<<A_ci(0,2)<<"\t"<<A_ci(1,0)<<"\t"<<A_ci(1,1)<<"\t"<<A_ci(1,2)<<"\t"<<A_ci(2,0)<<"\t"<<A_ci(2,1)<<"\t"<<A_ci(2,2)<<endl;
////   file[17]<<walking_tick_<<"\t"<<check_value(0)<<"\t"<<check_value(1)<<"\t"<<check_value(2)<<endl;

//   double K_Gain;
//   double weight;
//   if(walking_tick_ >= t_start_real_){
//       if(walking_tick_ >= t_start_real_+t_double1_ && walking_tick_ < t_start_+t_total_-t_double2_-t_rest_last_){
//           weight = 0.9;
//       K_Gain = 10000.0;
//       }
//       else{
//           weight = 0.1;
//           K_Gain = 50000.0;
//       }

//   }
//   else
//       weight = 1.0;

////q_init_

//   Eigen::Matrix<double, 9, 9> Identity;
//   Identity.setIdentity();


//   ////////////////// H matrix ////////////////////////
//   H_temp = A_c.transpose()*A_c;
//   //////////////////////////////////////////////////
//   ///

//      for(int i=0;i<9;i++){
//          for(int j=0;j<9;j++){
//              H_temp(i,j) = weight*H_temp(i,j) + (1-weight)*Identity(i,j);
//          }
//      }


//   /// matchig parameter /////

//   for(int j=0;j<9;j++){
//       for(int i=0;i<9;i++){
//           H_[j*9+i] = H_temp(i,j);
//       }
//   }



//   g_temp(0) = weight*g_temp(0) - (1-weight) *K_Gain*(q_init_(WA_BEGIN) - current_q_(WA_BEGIN));
//   g_temp(1) = weight*g_temp(1) - (1-weight) *K_Gain*(q_init_(RA_BEGIN) - current_q_(RA_BEGIN));
//   g_temp(2) = weight*g_temp(2) - (1-weight) *K_Gain*(q_init_(RA_BEGIN+1) - current_q_(RA_BEGIN+1));
//   g_temp(3) = weight*g_temp(3) - (1-weight) *K_Gain*(q_init_(RA_BEGIN+2) - current_q_(RA_BEGIN+2));
//   g_temp(4) = weight*g_temp(4) - (1-weight) *K_Gain*(q_init_(RA_BEGIN+4) - current_q_(RA_BEGIN+4));

//   g_temp(5) = weight*g_temp(5) - (1-weight) *K_Gain*(q_init_(LA_BEGIN) - current_q_(LA_BEGIN));
//   g_temp(6) = weight*g_temp(6) - (1-weight) *K_Gain*(q_init_(LA_BEGIN+1) - current_q_(LA_BEGIN+1));
//   g_temp(7) = weight*g_temp(7) - (1-weight) *K_Gain*(q_init_(LA_BEGIN+2) - current_q_(LA_BEGIN+2));
//   g_temp(8) = weight*g_temp(8) - (1-weight) *K_Gain*(q_init_(LA_BEGIN+4) - current_q_(LA_BEGIN+4));
////   cout<<"init q of waist , right arm, left arm : "<<q_init_(WA_BEGIN)<<", "<<q_init_(RA_BEGIN)<<", "<<q_init_(LA_BEGIN)<<endl;
////  cout<<"curr q of waist , right arm, left arm : "<<current_q_(WA_BEGIN)<<", "<<current_q_(RA_BEGIN)<<", "<<current_q_(LA_BEGIN)<<endl;

////   cout<<"init waist : "<<q_init_(WA_BEGIN)<<", current waist : "<<current_q_(WA_BEGIN)<<endl;

//   for(int i=0;i<9;i++)
//       g_[i] = g_temp(i);

//   for(int j=0;j<9;j++){
//       for(int i=0;i<3;i++)
//           A_[j*3+i] = A_c(i,j);
//   }
//   for(int i=0;i<3;i++){
//       lbA_[i] = y_temp_3(i);
//       ubA_[i] = y_temp_3(i);
//   }

//}
//void WalkingController::ComputeVelocityTrajectory(){
//    Eigen::Isometry3d   reference = pelv_trajectory_float_;


//}

void WalkingController::CalculateInertiaMatrix(){

    Eigen::Vector6d joint_axis_local[28];

    for(int i=0;i<28;i++)
        joint_axis_local[i].setZero();

    joint_axis_local[0](5) = -1; joint_axis_local[1](3) = 1;  joint_axis_local[2](4) = 1;  joint_axis_local[3](4) =  1; joint_axis_local[4](4) =  1;  joint_axis_local[5](3) =  1;
    joint_axis_local[6](5) = -1; joint_axis_local[7](3) = 1; joint_axis_local[8](4) = -1; joint_axis_local[9](4) = -1; joint_axis_local[10](4) = -1; joint_axis_local[11](3) = 1;

    joint_axis_local[12](5) = 1; joint_axis_local[13](3) = -1;

    joint_axis_local[14](4) = 1;    joint_axis_local[15](3) = 1;    joint_axis_local[16](4) = 1;    joint_axis_local[17](3) = 1;    joint_axis_local[18](4) = 1;    joint_axis_local[19](3) = 1;    joint_axis_local[20](4) = 1;
    joint_axis_local[21](4) = -1;    joint_axis_local[22](3) = 1;    joint_axis_local[23](4) = -1;    joint_axis_local[24](3) = 1;    joint_axis_local[25](4) = -1;    joint_axis_local[26](3) = 1;    joint_axis_local[27](4) = -1;


    double             stacked_mass[29];
    Eigen::Vector3d    stacked_com_of_link[29];
    Eigen::Matrix3d    stacked_inertia_of_link[29];

    Eigen::Matrix3d    stacked_skew_temp, stacked_left_skew, stacked_right_skew;
    Eigen::Matrix3d    skew_temp, left_skew, right_skew;
    Eigen::Vector3d    relative_com, stacked_relative_com,  stacked_right_relative_com, stacked_left_relative_com;
    Eigen::Matrix3d    D_skew, stacked_D_skew, left_D_skew, right_D_skew;

    stacked_mass[RF_BEGIN+5] = link_mass_[RF_LINK+5]; stacked_com_of_link[RF_BEGIN+5] = link_local_com_position_[RF_LINK +5];
    stacked_mass[LF_BEGIN+5] = link_mass_[LF_LINK+5]; stacked_com_of_link[LF_BEGIN+5] = link_local_com_position_[LF_LINK +5];
    stacked_mass[RA_BEGIN+5] = link_mass_[RA_LINK+5]; stacked_com_of_link[RA_BEGIN+5] = link_local_com_position_[RA_LINK +5];
    stacked_mass[LA_BEGIN+5] = link_mass_[LA_LINK+5]; stacked_com_of_link[LA_BEGIN+5] = link_local_com_position_[LA_LINK +5];

    stacked_inertia_of_link[RF_BEGIN+5] = link_transform_[RF_BEGIN+5].linear()*link_inertia_[RF_LINK+5] *link_transform_[RF_BEGIN+5].linear().transpose();
    stacked_inertia_of_link[LF_BEGIN+5] = link_transform_[LF_BEGIN+5].linear()*link_inertia_[LF_LINK+5] *link_transform_[LF_BEGIN+5].linear().transpose();
    stacked_inertia_of_link[RA_BEGIN+5] = link_transform_[RA_BEGIN+5].linear()*link_inertia_[RA_LINK+5] *link_transform_[RA_BEGIN+5].linear().transpose();
    stacked_inertia_of_link[LA_BEGIN+5] = link_transform_[LA_BEGIN+5].linear()*link_inertia_[LA_LINK+5] *link_transform_[LA_BEGIN+5].linear().transpose();

    //for foot inertia matrix calculation
    for(int i=5; i >=1; i--){
        stacked_mass[RF_BEGIN+i-1] = stacked_mass[RF_BEGIN+i] + link_mass_[RF_LINK+i-1];
        stacked_mass[LF_BEGIN+i-1] = stacked_mass[LF_BEGIN+i] + link_mass_[LF_LINK+i-1];

        stacked_com_of_link[RF_BEGIN+i-1] = (stacked_mass[RF_BEGIN+i] * stacked_com_of_link[RF_BEGIN+i] + link_mass_[RF_LINK+i-1]*link_local_com_position_[RF_LINK+i-1])/(stacked_mass[RF_BEGIN+i] + link_mass_[RF_LINK+i-1]);
        stacked_com_of_link[LF_BEGIN+i-1] = (stacked_mass[LF_BEGIN+i] * stacked_com_of_link[LF_BEGIN+i] + link_mass_[LF_LINK+i-1]*link_local_com_position_[LF_LINK+i-1])/(stacked_mass[LF_BEGIN+i] + link_mass_[LF_LINK+i-1]);

        //right foot
        stacked_relative_com = stacked_com_of_link[RF_BEGIN+i] - stacked_com_of_link[RF_BEGIN+i-1];
        relative_com = link_local_com_position_[RF_LINK+i-1] - stacked_com_of_link[RF_BEGIN+i-1];

        skew_temp = DyrosMath::skew(relative_com);
        stacked_skew_temp = DyrosMath::skew(stacked_relative_com);
        D_skew = skew_temp.transpose()*skew_temp;
        stacked_D_skew = stacked_skew_temp.transpose()*stacked_skew_temp;

        stacked_inertia_of_link[RF_BEGIN+i-1] =stacked_inertia_of_link[RF_BEGIN + i] + stacked_mass[RF_BEGIN+i]*stacked_D_skew + link_transform_[RF_LINK + i-1].linear()*link_inertia_[RF_LINK+i-1]*link_transform_[RF_LINK+i-1].linear().transpose() + link_mass_[RF_LINK+i-1]*D_skew;

        //left foot
        stacked_relative_com = stacked_com_of_link[LF_BEGIN+i] - stacked_com_of_link[LF_BEGIN+i-1];
        relative_com = link_local_com_position_[LF_LINK+i-1] - stacked_com_of_link[LF_BEGIN+i-1];

        skew_temp = DyrosMath::skew(relative_com);
        stacked_skew_temp = DyrosMath::skew(stacked_relative_com);
        D_skew = skew_temp.transpose()*skew_temp;
        stacked_D_skew = stacked_skew_temp.transpose()*stacked_skew_temp;

        stacked_inertia_of_link[LF_BEGIN+i-1] =stacked_inertia_of_link[LF_BEGIN + i] + stacked_mass[LF_BEGIN+i]*stacked_D_skew + link_transform_[LF_LINK + i-1].linear()*link_inertia_[LF_LINK+i-1]*link_transform_[LF_LINK+i-1].linear().transpose() + link_mass_[LF_LINK+i-1]*D_skew;

    }
    //for arm inertia matrix calculation
    for(int i=6; i>=1; i--){
        stacked_mass[RA_BEGIN + i-1] = stacked_mass[RA_BEGIN+i] + link_mass_[RA_LINK + i-1];
        stacked_mass[LA_BEGIN + i-1] = stacked_mass[LA_BEGIN+i] + link_mass_[LA_LINK + i-1];

        stacked_com_of_link[RA_BEGIN+i-1] = (stacked_mass[RA_BEGIN+i] * stacked_com_of_link[RA_BEGIN+i] + link_mass_[RA_LINK+i-1]*link_local_com_position_[RA_LINK+i-1])/(stacked_mass[RA_BEGIN+i] + link_mass_[RA_LINK+i-1]);
        stacked_com_of_link[LA_BEGIN+i-1] = (stacked_mass[LA_BEGIN+i] * stacked_com_of_link[LA_BEGIN+i] + link_mass_[LA_LINK+i-1]*link_local_com_position_[LA_LINK+i-1])/(stacked_mass[LA_BEGIN+i] + link_mass_[LA_LINK+i-1]);

        //right arm
        stacked_relative_com = stacked_com_of_link[RA_BEGIN + i] - stacked_com_of_link[RA_BEGIN + i-1];
        relative_com = link_local_com_position_[RA_LINK + i-1] - stacked_com_of_link[RA_BEGIN + i-1];

        skew_temp = DyrosMath::skew(relative_com);
        stacked_skew_temp = DyrosMath::skew(stacked_relative_com);
        D_skew = skew_temp.transpose()*skew_temp;
        stacked_D_skew = stacked_skew_temp.transpose()*stacked_skew_temp;

        stacked_inertia_of_link[RA_BEGIN+i-1] =stacked_inertia_of_link[RA_BEGIN + i] + stacked_mass[RA_BEGIN+i]*stacked_D_skew + link_transform_[RA_LINK + i-1].linear()*link_inertia_[RA_LINK+i-1]*link_transform_[RA_LINK+i-1].linear().transpose() + link_mass_[RA_LINK+i-1]*D_skew;


        //left arm
        stacked_relative_com = stacked_com_of_link[LA_BEGIN+i] - stacked_com_of_link[LA_BEGIN+i-1];
        relative_com = link_local_com_position_[LA_LINK+i-1] - stacked_com_of_link[LA_BEGIN+i-1];

        skew_temp = DyrosMath::skew(relative_com);
        stacked_skew_temp = DyrosMath::skew(stacked_relative_com);
        D_skew = skew_temp.transpose()*skew_temp;
        stacked_D_skew = stacked_skew_temp.transpose()*stacked_skew_temp;

        stacked_inertia_of_link[LA_BEGIN+i-1] =stacked_inertia_of_link[LA_BEGIN + i] + stacked_mass[LA_BEGIN+i]*stacked_D_skew + link_transform_[LA_LINK + i-1].linear()*link_inertia_[LA_LINK+i-1]*link_transform_[LA_LINK+i-1].linear().transpose() + link_mass_[LA_LINK+i-1]*D_skew;

    }

    stacked_mass[WA_BEGIN + 1] = stacked_mass[RA_BEGIN] + stacked_mass[LA_BEGIN] + link_mass_[WA_LINK+1];
    stacked_com_of_link[WA_BEGIN + 1] = (stacked_mass[RA_BEGIN] * stacked_com_of_link[RA_BEGIN] + stacked_mass[LA_BEGIN]*stacked_com_of_link[LA_BEGIN] + link_mass_[WA_LINK + 1 ]*link_local_com_position_[WA_LINK + 1])/(stacked_mass[RA_BEGIN] + stacked_mass[LA_BEGIN] + link_mass_[WA_LINK+1]);

    /////waist inertia matrix calculation
    //waist roll joint ( 2nd joint)
    stacked_right_relative_com = stacked_com_of_link[RA_BEGIN] - stacked_com_of_link[WA_BEGIN+1];
    stacked_left_relative_com = stacked_com_of_link[LA_BEGIN] - stacked_com_of_link[WA_BEGIN+1];

    relative_com = link_local_com_position_[WA_LINK + 1] - stacked_com_of_link[WA_BEGIN + 1];

    skew_temp = DyrosMath::skew(relative_com);
    stacked_left_skew = DyrosMath::skew(stacked_left_relative_com);
    stacked_right_skew = DyrosMath::skew(stacked_right_relative_com);

    D_skew = skew_temp.transpose()*skew_temp;
    left_D_skew = stacked_left_skew.transpose()*stacked_left_skew;
    right_D_skew = stacked_right_skew.transpose()*stacked_right_skew;

    stacked_inertia_of_link[WA_BEGIN+1] =stacked_inertia_of_link[RA_BEGIN]  + stacked_inertia_of_link[LA_BEGIN] + stacked_mass[RA_BEGIN]*right_D_skew +  stacked_mass[LA_BEGIN]*left_D_skew
             + link_transform_[WA_LINK + 1].linear()*link_inertia_[WA_LINK + 1]*link_transform_[WA_LINK + 1].linear().transpose() + link_mass_[WA_LINK + 1]*D_skew;

    // waist yaw joint (1st joint_
    stacked_mass[WA_BEGIN] = stacked_mass[WA_BEGIN+1] + link_mass_[WA_BEGIN];
    stacked_com_of_link[WA_BEGIN] = (stacked_mass[WA_BEGIN+1] * stacked_com_of_link[WA_BEGIN+1] + link_mass_[WA_LINK]*link_local_com_position_[WA_LINK])/(stacked_mass[WA_BEGIN] + link_mass_[WA_LINK]);

    stacked_relative_com = stacked_com_of_link[WA_BEGIN+1] - stacked_com_of_link[WA_BEGIN];
    relative_com = link_local_com_position_[WA_LINK] - stacked_com_of_link[WA_BEGIN];

    skew_temp = DyrosMath::skew(relative_com);
    stacked_skew_temp = DyrosMath::skew(stacked_relative_com);

    D_skew = skew_temp.transpose()*skew_temp;
    stacked_D_skew = stacked_skew_temp.transpose()*stacked_skew_temp;

    stacked_inertia_of_link[WA_BEGIN] = stacked_inertia_of_link[WA_BEGIN+1] + stacked_mass[WA_BEGIN+1]*stacked_D_skew + link_transform_[WA_LINK].linear()*link_inertia_[WA_LINK]*link_transform_[WA_LINK].linear().transpose() + link_mass_[WA_LINK]*D_skew;


    //calculating total inertia
    stacked_mass[28] =stacked_mass[RF_BEGIN] + stacked_mass[LF_BEGIN] + link_mass_[0];
    stacked_com_of_link[28] = (stacked_mass[RF_BEGIN]*stacked_com_of_link[RF_BEGIN] + stacked_mass[LF_BEGIN]*stacked_com_of_link[LF_BEGIN] + link_mass_[0]*link_local_com_position_[0])/(stacked_mass[RF_BEGIN] + stacked_mass[LF_BEGIN] + link_mass_[0]);

    relative_com = link_local_com_position_[0] - stacked_com_of_link[28];
    stacked_right_relative_com  = stacked_com_of_link[RF_BEGIN] - stacked_com_of_link[28];
    stacked_left_relative_com  = stacked_com_of_link[LF_BEGIN] - stacked_com_of_link[28];

    stacked_right_skew =DyrosMath::skew(stacked_right_relative_com);
    stacked_left_skew =DyrosMath::skew(stacked_left_relative_com);
    skew_temp = DyrosMath::skew(relative_com);

    right_D_skew = stacked_right_skew.transpose()*stacked_right_skew;
    left_D_skew = stacked_left_skew.transpose()*stacked_left_skew;
    D_skew = skew_temp.transpose()*skew_temp;

    stacked_inertia_of_link[28] = stacked_inertia_of_link[RF_BEGIN] + stacked_mass[RF_BEGIN]*right_D_skew
            + stacked_inertia_of_link[LF_BEGIN] +stacked_mass[LF_BEGIN]*left_D_skew
            + link_inertia_[0] + link_mass_[0]*D_skew;


    // calculatinf at the com
    //adding pelvis
    relative_com = stacked_com_of_link[28] - com_float_current_;
    skew_temp = DyrosMath::skew(relative_com);
    D_skew = skew_temp.transpose()*skew_temp;

    stacked_inertia_of_link[28] += stacked_mass[28] * D_skew;
    //adding waist
    relative_com = stacked_com_of_link[WA_BEGIN] - com_float_current_;
    skew_temp = DyrosMath::skew(relative_com);
    D_skew = skew_temp.transpose()*skew_temp;

    stacked_inertia_of_link[28] += stacked_inertia_of_link[WA_BEGIN] + stacked_mass[WA_BEGIN]*D_skew; //total inertia (adding pelvis and waist) and pelivs consists of two legs and waist consists of two arms

    total_inertia_ = stacked_inertia_of_link[28];

    ////////// calculating columns of mass and inertia tensor of all link

    Eigen::Vector3d     joint_axis;
    Eigen::Matrix3d     joint_axis_skew;
    Eigen::Matrix3d     stacked_com_skew;

    //calculating mass and inertia tensor of amr
    for(int i=0;i<7;i++){
        for(int j=0;j<3;j++)
            joint_axis(j) =current_arm_jacobian_r_(j+3,i);

        joint_axis_skew = DyrosMath::skew(joint_axis);

        inertia_m_vector_[RA_BEGIN+i] =  joint_axis_skew * (stacked_com_of_link[RA_BEGIN+i] - link_transform_[RA_LINK+i].translation())*stacked_mass[RA_BEGIN+i];

        stacked_com_skew = DyrosMath::skew(stacked_com_of_link[RA_BEGIN+i]);

        inertia_h_0_vector_[RA_BEGIN+i] = stacked_com_skew *inertia_m_vector_[RA_BEGIN+i] + stacked_inertia_of_link[RA_BEGIN+i]*joint_axis;

        for(int j=0;j<3;j++)
            joint_axis(j) =current_arm_jacobian_l_(j+3,i);

        joint_axis_skew = DyrosMath::skew(joint_axis);
        inertia_m_vector_[LA_BEGIN+i] =  joint_axis_skew * (stacked_com_of_link[LA_BEGIN+i] - link_transform_[LA_LINK+i].translation())*stacked_mass[LA_BEGIN+i];

        stacked_com_skew = DyrosMath::skew(stacked_com_of_link[LA_BEGIN+i]);

        inertia_h_0_vector_[LA_BEGIN+i] = stacked_com_skew *inertia_m_vector_[LA_BEGIN+i] + stacked_inertia_of_link[LA_BEGIN+i]*joint_axis;
    }

    //calculating mass and inertia tensor of leg
    for(int i=0;i<6;i++){
        for(int j=0;j<3;j++)
            joint_axis(j) =current_leg_jacobian_r_(j+3,i);

        joint_axis_skew = DyrosMath::skew(joint_axis);

        inertia_m_vector_[RF_BEGIN+i] =  joint_axis_skew * (stacked_com_of_link[RF_BEGIN+i] - link_transform_[RF_LINK+i].translation())*stacked_mass[RF_BEGIN+i];

        stacked_com_skew = DyrosMath::skew(stacked_com_of_link[RF_BEGIN+i]);

        inertia_h_0_vector_[RF_BEGIN+i] = stacked_com_skew *inertia_m_vector_[RF_BEGIN+i] + stacked_inertia_of_link[RF_BEGIN+i]*joint_axis;

        for(int j=0;j<3;j++)
            joint_axis(j) =current_leg_jacobian_l_(j+3,i);

        joint_axis_skew = DyrosMath::skew(joint_axis);
        inertia_m_vector_[LF_BEGIN+i] =  joint_axis_skew * (stacked_com_of_link[LF_BEGIN+i] - link_transform_[LF_LINK+i].translation())*stacked_mass[LF_BEGIN+i];

        stacked_com_skew = DyrosMath::skew(stacked_com_of_link[LF_BEGIN+i]);

        inertia_h_0_vector_[LF_BEGIN+i] = stacked_com_skew *inertia_m_vector_[LF_BEGIN+i] + stacked_inertia_of_link[LF_BEGIN+i]*joint_axis;
    }

    for(int i=0;i<2;i++){
        for(int j=0;j<3;j++)
            joint_axis(j) = current_waist_jacobian_[i](j+3,1);

        joint_axis_skew = DyrosMath::skew(joint_axis);

        inertia_m_vector_[WA_BEGIN+i] = joint_axis_skew * (stacked_com_of_link[WA_BEGIN + i] - link_transform_[WA_BEGIN+i].translation())*stacked_mass[WA_BEGIN+i];

        stacked_com_skew = DyrosMath::skew(stacked_com_of_link[WA_BEGIN+i]);

        inertia_h_0_vector_[WA_BEGIN+i] = stacked_com_skew *inertia_m_vector_[WA_BEGIN+i] + stacked_inertia_of_link[WA_BEGIN+i]*joint_axis;
    }

    Eigen::Matrix3d  com_skew;
    com_skew = DyrosMath::skew(com_float_current_);
    for(int i=0;i<28;i++){
        inertia_h_vector_[i] = inertia_h_0_vector_[i] - com_skew*inertia_m_vector_[i];
    }

    //total_inertia_ =


}
void WalkingController::ResolvedMomentumCtrl(){

    CalculateInertiaMatrix();

    // current_leg_jacobian_l_inv_ inverse of leg jacobian

    if(walking_tick_ == 0)
        cout<<"resolved momentum control "<<endl;

    Eigen::Matrix6d    right_leg_momentum_temp, left_leg_momentum_temp;

    for(int i=0;i<6;i++){
        for(int j=0;j<3;j++){
            right_leg_momentum_temp(j,i) = inertia_m_vector_[RF_BEGIN + i](j);
            right_leg_momentum_temp(j+3,i) = inertia_h_vector_[RF_BEGIN +i](j);
        }
    }
    inertia_right_leg_ = right_leg_momentum_temp * current_leg_jacobian_r_inv_;

    for(int i=0;i<6;i++){
        for(int j=0;j<3;j++){
            left_leg_momentum_temp(j,i) = inertia_m_vector_[LF_BEGIN + i](j);
            left_leg_momentum_temp(j+3,i) = inertia_h_vector_[LF_BEGIN +i](j);
        }
    }
    inertia_left_leg_ = left_leg_momentum_temp * current_leg_jacobian_l_inv_;



    Eigen::Matrix6d  body_inertia_temp;
    Eigen::Matrix3d  skew_temp;

    skew_temp = DyrosMath::skew(com_float_current_);

    body_inertia_temp.setZero();
    for(int i=0;i<3;i++){
//        for(int j=0;j<3;j++){
//            body_inertia_temp(i,i) = total_mass;
//            body_inertia_temp(i,j+3) = - skew_temp(i,j) * total_mass;
//            body_inertia_temp(i+3,j+3) = total_inertia_(i,j);
//        }
        body_inertia_temp(i,i) = total_mass_;
    }
    body_inertia_temp.block<3,3>(0,3) = -skew_temp*total_mass_;
    body_inertia_temp.block<3,3>(3,3) = total_inertia_;

    Eigen::Matrix6d  right_leg_direction_matrix, left_leg_direction_matrix;
    Eigen::Matrix3d  r_leg_skew, l_leg_skew;
    r_leg_skew = DyrosMath::skew(link_transform_[RF_BEGIN+5].translation());
    l_leg_skew = DyrosMath::skew(link_transform_[LF_BEGIN+5].translation());

    for(int i=0;i<3;i++){
//        for(int j=0;j<3;j++){
//            right_leg_direction_matrix(i,j) = 1.0;
//            right_leg_direction_matrix(i,j+3) = r_leg_skew(i,j);
//            right_leg_direction_matrix(i+3,j+3) = 1.0;

//            left_leg_direction_matrix(i,j) = 1.0;
//            left_leg_direction_matrix(i,j+3) = l_leg_skew(i,j);
//            left_leg_direction_matrix(i+3,j+3) = 1.0;
//        }
        right_leg_direction_matrix.setIdentity();
        right_leg_direction_matrix.block<3,3>(0,3) = r_leg_skew;

        left_leg_direction_matrix.setIdentity();
        left_leg_direction_matrix.block<3,3>(0,3) = l_leg_skew;
    }


    inertia_body_ = body_inertia_temp - inertia_right_leg_*right_leg_direction_matrix - inertia_left_leg_*left_leg_direction_matrix;

//    cout<<"body inertia temp :"<<endl<<body_inertia_temp<<endl<<"inertia right leg "<<endl<<inertia_right_leg_*right_leg_direction_matrix
//       <<endl<<"inertia left leg "<<endl<<inertia_left_leg_*left_leg_direction_matrix
//      <<endl<<"right leg direction Matrix "<<endl<<right_leg_direction_matrix<<endl<<"left leg direction matrix "<<left_leg_direction_matrix
//     <<endl;
    Eigen::Matrix<double, 6, 21> A;
    Eigen::Matrix<double, 21, 6> A_inv;
//    Eigen::Matrix<double, 6, 15>  A;
//    Eigen::Matrix<double, 15, 6>  A_inv;

    Eigen::Matrix6d     S;
    S.setIdentity();

    A.block<6,6>(0,0) = inertia_body_;
    A.block<3,1>(0,6) = inertia_m_vector_[WA_BEGIN];
    A.block<3,1>(3,6) = inertia_h_vector_[WA_BEGIN];
    for(int i=0;i<7;i++){
        A.block<3,1>(0,7+i) = inertia_m_vector_[RA_BEGIN+i];
        A.block<3,1>(3,7+i) = inertia_h_vector_[RA_BEGIN+i];

        A.block<3,1>(0,14+i) = inertia_m_vector_[LA_BEGIN+i];
        A.block<3,1>(3,14+i) = inertia_h_vector_[LA_BEGIN+i];
    }
//    A.block<3,1>(0,0) = inertia_m_vector_[WA_BEGIN];
//    A.block<3,1>(3,0) = inertia_h_vector_[WA_BEGIN];
//    for(int i=0;i<7;i++){
//        A.block<3,1>(0,1+i) = inertia_m_vector_[RA_BEGIN+i];
//        A.block<3,1>(3,1+i) = inertia_h_vector_[RA_BEGIN+i];

//        A.block<3,1>(0,8+i) = inertia_m_vector_[LA_BEGIN+i];
//        A.block<3,1>(3,8+i) = inertia_h_vector_[LA_BEGIN+i];
//    }

//    A.block<3,1>(0,7) = inertia_m_vector_[RA_BEGIN];
//    A.block<3,1>(3,7) = inertia_h_vector_[RA_BEGIN];
//    A.block<3,1>(0,8) = inertia_m_vector_[LA_BEGIN];
//    A.block<3,1>(3,8) = inertia_h_vector_[LA_BEGIN];


    A = S*A;

    //calculating A  inverse
    double w0 , lambda, a, w;
    w0 = 0.001; lambda = 0.05;

    Eigen::Matrix6d A_inv_temp;

    A_inv_temp = A*A.transpose();
    w = sqrt(A_inv_temp.determinant());

    a = lambda*pow(1-w/w0,2);
    //A_inv = A.transpose()*(a*Eigen::Matrix6d::Identity() + A*A.transpose()).inverse();

    A_inv = A.transpose()*(A*A.transpose()).inverse();

    //A_inv = DyrosMath::pinv(A);
    //A_inv = A.completeOrthogonalDecomposition().pseudoInverse();

    Eigen::Matrix<double, 6, 6> H;

    H = A*A_inv;

//    if(walking_tick_ ==0){
//        cout<<"S matrix : "<<endl<<S<<endl;
//        cout<<"A matrix "<<endl<<A<<endl<<"inertia M vextor : "<<endl<<inertia_m_vector_[WA_BEGIN]<<endl<<"inertia h vector "<<endl<<inertia_h_vector_[WA_BEGIN]<<endl;
//        cout<<"H matrix : "<<endl<<H<<endl;
//    }

    Eigen::Vector6d    Y, ref_momentum;
    Eigen::VectorXd    ref_vector;
    ref_vector.resize(15);
    Eigen::Vector6d    ref_lfoot, ref_rfoot, cubic_xr, cubic_xl;
    double kp;
    kp = 50;
//    for(int i=0;i<3;i++){
//        ref_vector(i) = 0.0;//kp*(pelv_trajectory_float_.translation()(i));
//        ref_vector(i+3) = 0.0;
//    }

    ref_vector(0) = kp*(0.0*DEGREE - current_q_(WA_BEGIN));
    //ref_vector(7) = kp*(q_init_(RA_BEGIN) - current_q_(RA_BEGIN));
    //ref_vector(8) = kp*(q_init_(LA_BEGIN) - current_q_(LA_BEGIN));
    for(int i=0;i<7;i++){
        ref_vector(7+i) = kp*(q_init_(RA_BEGIN+i) - current_q_(RA_BEGIN+i));
        ref_vector(8+i) = kp*(q_init_(LA_BEGIN+i) - current_q_(LA_BEGIN+i));
    }

//    file[41]<<walking_tick_;
//    for(int i=0;i<15;i++){
//        file[41]<<"\t"<<ref_vector(i);
//    }
//    file[41]<<endl;

    cubic_xr.segment(0,3) = rfoot_trajectory_float_.translation();
    cubic_xr.segment(3,3) = rfoot_trajectory_euler_float_;

    cubic_xl.segment(0,3) = lfoot_trajectory_float_.translation();
    cubic_xl.segment(3,3) = lfoot_trajectory_euler_float_;

    ref_rfoot.segment(0,3) = (rfoot_trajectory_float_.translation() - rfoot_float_current_.translation());
    ref_rfoot.segment(3,3) = -DyrosMath::legGetPhi(rfoot_float_current_, rfoot_float_init_,cubic_xr);

    ref_lfoot.segment(0,3) = (lfoot_trajectory_float_.translation() - lfoot_float_current_.translation());
    ref_lfoot.segment(3,3) = -DyrosMath::legGetPhi(lfoot_float_current_, lfoot_float_init_,cubic_xl);

    //check foot coordinate
    Eigen::Vector6d     xr_support, xl_support, rfoot_support, lfoot_support;
    xr_support.segment(0,3) = rfoot_trajectory_support_.translation();
    xr_support.segment(3,3) = rfoot_trajectory_euler_support_;
    xl_support.segment(0,3) = lfoot_trajectory_support_.translation();
    xl_support.segment(3,3) = lfoot_trajectory_euler_support_;

    rfoot_support.segment(0,3) =100*(rfoot_trajectory_support_.translation() -rfoot_support_current_.translation());
    rfoot_support.segment(3,3) = -DyrosMath::legGetPhi(rfoot_support_current_, rfoot_support_init_, xr_support);

    lfoot_support.segment(0,3) =lfoot_trajectory_support_.translation() -lfoot_support_current_.translation();
    lfoot_support.segment(3,3) = -DyrosMath::legGetPhi(lfoot_support_current_, lfoot_support_init_, xl_support);

    double kp_momentum;
    kp_momentum = 10;
    ref_momentum.setZero();

    Eigen::Matrix6d leg_kp;
    leg_kp(0,0) = 160;
    leg_kp(1,1) = 160;
    leg_kp(2,2) = 160;
    leg_kp(3,3) = 100;
    leg_kp(4,4) = 100;
    leg_kp(5,5) = 100;

    leg_kp.setIdentity();

    ref_rfoot = rfoot_trajectory_dot_support_;
    ref_lfoot = lfoot_trajectory_dot_support_;

//    ref_rfoot = rfoot_support;
//    ref_lfoot = lfoot_support;

    ref_rfoot.segment(3,3).setZero();
    ref_lfoot.segment(3,3).setZero();


    ref_momentum.setZero();
    for(int i=0;i<2;i++){
        //ref_momentum(i) = kp_momentum* total_mass * (com_desired_(i) - com_support_current_(i) ) + com_dot_desired_(i);
        ref_momentum(i) = total_mass_*com_dot_desired_(i);
        //ref_momentum(i) = p_ref_(i);
    }
    //ref_momentum(2) = total_mass* kp_momentum*(pelv_support_init_.translation()(2) - pelv_support_current_.translation()(2));


    Y = ref_momentum - inertia_left_leg_*(ref_lfoot) - inertia_right_leg_*(ref_rfoot);
    //Y = - inertia_left_leg_*(ref_lfoot) - inertia_right_leg_*(ref_rfoot);

//    file[27]<<walking_tick_;
//    for(int i=0;i<6;i++){
//        for(int j=0;j<15;j++){
//            file[27]<<"\t"<<A(i,j);
//        }
//    }
//    file[27]<<endl;

//    file[28]<<walking_tick_;
//    for(int i=0;i<6;i++)
//        file[28]<<"\t"<<Y(i);
////    for(int i=0;i<15;i++)
////        file[28]<<"\t"<<ref_vector(i);

//    file[28]<<endl;

//    if(walking_tick_ >=  3*hz_ && walking_tick_ <3.5*hz_){
//        ref_rfoot(4) = DyrosMath::cubic(walking_tick_,3*hz_,3.5*hz_,0.0,-1.0, 0.0, 0.0);
//    }
//    else if(walking_tick_ >= 3.5*hz_ && walking_tick_ < 4.0*hz_){
//        ref_rfoot(4) = DyrosMath::cubic(walking_tick_,3.5*hz_,4*hz_, -1.0, 0.0, 0.0, 0.0);
//    }

//    if(foot_step_(current_step_num_,6) == 1){ //left foot support
//      Y = ref_momentum - inertia_right_leg_*(leg_kp*ref_rfoot);
//    }
//    else if(foot_step_(current_step_num_,6) == 0){// right foot support
//        Y = ref_momentum - inertia_left_leg_*(leg_kp*ref_lfoot);
//    }

    //Eigen::Vector9d target_speed_temp;

    target_speed_.segment(0,21) = A_inv * Y ;//+ (Eigen::Matrix<double, 15, 15>::Identity() - A_inv*A)*ref_vector;

    //target_speed_.segment(0,9) = target_speed_temp;

    Eigen::Vector6d  q_dot_r_leg, q_dot_l_leg;


    Eigen::Vector6d     rf,lf;
    lf = inertia_left_leg_*(ref_lfoot);
    rf = inertia_right_leg_*(ref_rfoot);

//    file[37]<<walking_tick_
//  /*2,3,4,*/<<"\t"<<pelv_trajectory_float_.translation()(0)<<"\t"<<pelv_trajectory_float_.translation()(1)<<"\t"<<pelv_trajectory_float_.translation()(2)
//   /*5,6.7*/<<"\t"<<rfoot_trajectory_float_.translation()(0)<<"\t"<<rfoot_trajectory_float_.translation()(1)<<"\t"<<rfoot_trajectory_float_.translation()(2)
//   /*8,9,10*/<<"\t"<<rfoot_float_current_.translation()(0)<<"\t"<<rfoot_float_current_.translation()(1)<<"\t"<<rfoot_float_current_.translation()(2)
///*11,12,13,14*/<<"\t"<<com_desired_(0)<<"\t"<<com_desired_(1)<<"\t"<<com_float_current_(0)<<"\t"<<com_float_current_(1)
///*15,16*/ <<"\t"<<com_dot_desired_(0)<<"\t"<<com_dot_desired_(1)
//  /*17,18*/ <<"\t"<<pelv_support_init_.translation()(2)<<"\t"<<pelv_support_current_.translation()(2)
///*19,20,21,22,23,24*/<<"\t"<<Y(0)<<"\t"<<Y(1)<<"\t"<<Y(2)<<"\t"<<Y(3)<<"\t"<<Y(4)<<"\t"<<Y(5)
///*25,26,27,28,29,30*/<<"\t"<<ref_momentum(0)<<"\t"<<ref_momentum(1)<<"\t"<<ref_momentum(2)<<"\t"<<ref_momentum(3)<<"\t"<<ref_momentum(4)<<"\t"<<ref_momentum(5)
///*31,32,33,34,35,36*/<<"\t"<<rf(0)<<"\t"<<rf(1)<<"\t"<<rf(2)<<"\t"<<rf(3)<<"\t"<<rf(4)<<"\t"<<rf(5)
///*37,38,39,40,41,42*/<<"\t"<<lf(0)<<"\t"<<lf(1)<<"\t"<<lf(2)<<"\t"<<lf(3)<<"\t"<<lf(4)<<"\t"<<lf(5)
///*43,44,45,46,47,48*/<<"\t"<<ref_rfoot(0)<<"\t"<<ref_rfoot(1)<<"\t"<<ref_rfoot(2)<<"\t"<<ref_rfoot(3)<<"\t"<<ref_rfoot(4)<<"\t"<<ref_rfoot(5)
///*49,50,51,52,53,54*/<<"\t"<<ref_lfoot(0)<<"\t"<<ref_lfoot(1)<<"\t"<<ref_lfoot(2)<<"\t"<<ref_lfoot(3)<<"\t"<<ref_lfoot(4)<<"\t"<<ref_lfoot(5)
///*55,56,57,58,59,60*/<<"\t"<<rfoot_support(0)<<"\t"<<rfoot_support(1)<<"\t"<<rfoot_support(2)<<"\t"<<rfoot_support(3)<<"\t"<<rfoot_support(4)<<"\t"<<rfoot_support(5)
///*61,62,63,64,65,66*/<<"\t"<<lfoot_support(0)<<"\t"<<lfoot_support(1)<<"\t"<<lfoot_support(2)<<"\t"<<lfoot_support(3)<<"\t"<<lfoot_support(4)<<"\t"<<lfoot_support(5)
///*67,68,69,70,71,72*/<<"\t"<<inertia_right_leg_.row(5)(0)<<"\t"<<inertia_right_leg_.row(5)(1)<<"\t"<<inertia_right_leg_.row(5)(2)<<"\t"<<inertia_right_leg_.row(5)(3)<<"\t"<<inertia_right_leg_.row(5)(4)<<"\t"<<inertia_right_leg_.row(5)(5)
///*73,74,75,76,77,78*/<<"\t"<<inertia_left_leg_.row(5)(0)<<"\t"<<inertia_left_leg_.row(5)(1)<<"\t"<<inertia_left_leg_.row(5)(2)<<"\t"<<inertia_left_leg_.row(5)(3)<<"\t"<<inertia_left_leg_.row(5)(4)<<"\t"<<inertia_left_leg_.row(5)(5)
//           <<endl;

   // q_dot_r_leg = current_leg_jacobian_r_inv_*(leg_kp*ref_rfoot - right_leg_direction_matrix*ref_vector.segment(0,6));
   // q_dot_l_leg = current_leg_jacobian_l_inv_*(leg_kp*ref_lfoot - left_leg_direction_matrix*ref_vector.segment(0,6));

//    if(walking_tick_ >= 4*hz_)
//        target_speed_.setZero();

     q_dot_r_leg = current_leg_jacobian_r_inv_*(ref_rfoot - right_leg_direction_matrix*target_speed_.segment(0,6));
     q_dot_l_leg = current_leg_jacobian_l_inv_*(ref_lfoot - left_leg_direction_matrix*target_speed_.segment(0,6));


//    if(walking_tick_ == t_start_){

//        cout<<"reference body speed : "<<ref_vector.segment(0,9)<<endl;
//        cout<<"target body speed : "<<target_speed_.segment(0,9)<<endl;
//    }

    //momentum_ = A * target_speed_.segment(0,21) + lf+rf;
    momentum_ = A*target_speed_.segment(0,21) + right_leg_momentum_temp*q_dot_r_leg + left_leg_momentum_temp*q_dot_l_leg;

    Eigen::Vector6d momentum_basic;

    momentum_basic = A* target_speed_.segment(0,21) + lf + rf;

//    file[39]<<walking_tick_<<"\t"<<target_speed_(0)<<"\t"<<target_speed_(1)<<"\t"<<target_speed_(2)<<"\t"<<target_speed_(3)<<"\t"<<target_speed_(4)<<"\t"<<target_speed_(5)<<"\t"<<target_speed_(6)<<"\t"<<target_speed_(7)<<"\t"<<target_speed_(8)
//           <<"\t"<<ref_vector(0)<<"\t"<<ref_vector(1)<<"\t"<<ref_vector(2)<<"\t"<<ref_vector(3)<<"\t"<<ref_vector(4)<<"\t"<<ref_vector(5)<<"\t"<<ref_vector(6)<<"\t"<<ref_vector(7)<<"\t"<<ref_vector(8)
//           <<endl;

//    file[39]<<walking_tick_<<"\t"<<momentum_(0)<<"\t"<<momentum_(1)<<"\t"<<momentum_(2)<<"\t"<<momentum_(3)<<"\t"<<momentum_(4)<<"\t"<<momentum_(5)
//           <<"\t"<<ref_momentum(0)<<"\t"<<ref_momentum(1)<<"\t"<<ref_momentum(2)<<"\t"<<ref_momentum(3)<<"\t"<<ref_momentum(4)<<"\t"<<ref_momentum(5)
//             <<"\t"<<momentum_basic(0)<<"\t"<<momentum_basic(1)<<"\t"<<momentum_basic(2)<<"\t"<<momentum_basic(3)<<"\t"<<momentum_basic(4)<<"\t"<<momentum_basic(5);
///*20~*/    for(int i=0;i<21;i++)
//           file[39]<<"\t"<<target_speed_(i);

//    file[39]<<endl;


//     desired_leg_q_dot_.segment(0,6) = q_dot_l_leg;
//     desired_leg_q_dot_.segment(6,6) = q_dot_r_leg;
    if(walking_tick_ >= 2*hz_){
//        cout<<"check in resolved momentum"<<endl;
//        cout<<"Amatrix : "<<endl<<A<<endl;
//        cout<<"hmatrix :"<<endl<<H<<endl;
//        cout<<"a inverse "<<endl<<A_inv<<endl;

    }
}
//void WalkingController::MPCwQP(){

//    int zmp_size;
//    zmp_size = ref_zmp_.col(1).size();

//    Eigen::VectorXd px_ref, py_ref;
//    px_ref.resize(zmp_size);
//    py_ref.resize(zmp_size);

//    px_ref = ref_zmp_.col(0);
//    py_ref = ref_zmp_.col(1);

//      cout<<"b"<<endl;
//    int NL = 16*hz_/10;

//    Eigen::Matrix4d a;
//    Eigen::Matrix<double, 1,3>c;

//    double dt =1.0/hz_;

//    a.setIdentity();
//    a(0,1) = dt; a(0,2) = 0.5*pow(dt,2); a(0,3) = pow(dt,3)/6;
//    a(1,2) = dt; a(1,3) = pow(dt,2)/2;
//    a(2,3) = dt;

//    c(0,0) = 1; c(0,1) = 0; c(0,2) = -zc_/GRAVITY;
//cout<<"c"<<endl;
////    Eigen::Vector4d b_bar;
////    b_bar(0) = c*b;
////    b_bar.segment(1,3) = b;

////    Eigen::Matrix4x3d f_bar;
////    f_bar.block<1,3>(0,0) = c*A;
////    f_bar.block<3,3>(0,1) = A;

////    Eigen::Matrix4d A_bar;
////    A_bar.setZero();
////    A_bar(0,0) = 1.0;
////    A_bar.block<4,3>(0,1) = f_bar;

//    Eigen::MatrixXd extended_A;
//    extended_A.resize(4*NL,4*NL);
//    extended_A.setZero();
//    extended_A.block<4,4>(0,0).setIdentity();

//    cout<<"a"<<endl;
//    for(int i=1;i<NL;i++){
//        extended_A.block<4,4>(4*i,4*(i-1)) = a;
//    }

//    cout<<"test 1 "<<endl;
//    Eigen::MatrixXd extended_c;
//    extended_c.resize(NL,4*NL);
//    extended_c.setZero();

//    for(int i=1;i<NL;i++){
//        extended_c.block<1,3>(i,0) = c;
//    }

//    Eigen::MatrixXd I_a;
//    I_a.setIdentity();

//    Eigen::MatrixXd A_input;
//    A_input.resize(4*NL,4*NL);
//    A_input = extended_A - I_a; // for constraint A

//    cout<<"test 2"<<endl;
//    Eigen::VectorXd p_ref;
//    p_ref.resize(NL);

//    for(int i=0;i<NL;i++)
//        p_ref(i) = ref_zmp_(i);

//    Eigen::MatrixXd S;
//    S.resize(NL,4*NL);
//    Eigen::MatrixXd sel_vector;
//    sel_vector.resize(1,8);
//    sel_vector.setZero();
//    sel_vector(0,3) = -1; sel_vector(0,7) = 1;
//    for(int i=0;i<NL;i++)
//        S.block<1,8>(i,4*i) = sel_vector;

//    cout<<"test 3 "<<endl;
//    Eigen::MatrixXd h_temp;
//    h_temp.resize(4*NL,4*NL);
//    Eigen::Matrix<double, 1,1> r;
//    r(0,0) = 0.000001;

//    h_temp = extended_c.transpose()*extended_c + S.transpose()*r*S;

//    Eigen::MatrixXd g_temp;
//    g_temp.resize(4*NL,1);

//    g_temp = -extended_c.transpose()*p_ref;

//cout<<"test 4 "<<endl;

//    int size = 1200;
//    real_t H[1200*1200], g[1200], A[1200*1200];

//    for(int i=0;i<1200;i++){
//        for(int j=0;j<1200;j++){
//            H[j*1200 + i] = h_temp(i,j);
//            cout<<"test 5 "<<endl;
//            A[j*1200 + i] = A_input(i,j);
//        }
//        g[i] = g_temp(i,0);
//    }


//    real_t xOpt[4*NL];

//    QProblem example(4*NL,1);

//    int_t nWSR = 1000;
//    real_t lbA[4*NL], lb[4*NL], ub[4*NL];
//    for(int i=0;i<4*NL;i++){
//        lbA[i] = 0.0;
//        lb[i] = 0.0;
//        ub[i] = 0.0;
//    }
//    for(int i=0;i<NL;i++){
//        lb[4*i+3] = -100;
//        ub[4*i+3] = 100;
//    }
//cout<<"test 6 "<<endl;
//    example.init(H,g,A,lb,ub,lbA,lbA,nWSR);
//    example.getPrimalSolution(xOpt);

//cout<<"test 7 "<<endl;

//}
void WalkingController::getPelvOriTrajectory(){
    double z_rot;

    if(foot_step_(current_step_num_,6) == 0){// right foot support
        z_rot = -atan2(foot_step_(current_step_num_,0)-foot_step_(current_step_num_-1,0),foot_step_(current_step_num_,1) - foot_step_(current_step_num_-1,1));
    }
    else{ // left foot support
        z_rot = atan2(foot_step_(current_step_num_,0)-foot_step_(current_step_num_-1,0),-(foot_step_(current_step_num_,1) - foot_step_(current_step_num_-1,1)));
    }

    Eigen::Vector3d Trunk_trajectory_euler;
    Trunk_trajectory_euler.setZero();
    if(walking_tick_ < t_start_real_ + t_double1_){
        for(int i=0;i<2;i++){
            Trunk_trajectory_euler(i) = DyrosMath::cubic(walking_tick_, t_start_,t_start_real_+t_double1_,pelv_support_euler_init_(i),0.0, 0.0, 0.0 );
        }

//        if(current_step_num_==0)
//            Trunk_trajectory_euler(2) = pelv_support_euler_init_(2);
//        else{

//        }
        //Trunk_trajectory_euler(2) = pelv_support_euler_init_(2);
    }
    else if(walking_tick_ >= t_start_real_ +t_double1_ && walking_tick_ < t_start_ + t_total_-t_double2_-t_rest_last_)
    {
        for(int i=0;i<2;i++)
            Trunk_trajectory_euler(i) = 0.0;
    }
    else {
        for(int i=0;i<2;i++){
            Trunk_trajectory_euler(i) = 0.0;
        }
    }
}
void WalkingController::getCOMvelocity(){


// setting support foot position ;

    Eigen::Vector2d support_foot;
    if(current_step_num_ == 0){
        for(int i=0;i<2;i++)
            support_foot(i) = supportfoot_support_init_offset_(i);
    }
    else if(current_step_num_ >= 1){
        for(int i=0;i<2;i++)
            support_foot(i) = foot_step_support_frame_offset_(current_step_num_-1,i);
    }


    if(walking_tick_ ==  t_start_real_+t_double1_init_){
        //cout<<"swing start, walking tick " <<walking_tick_<<",  support foot : "<<foot_step_(current_step_num_-1,6)<<", "<<foot_step_(current_step_num_-1,1)<<",  "<<foot_step_(current_step_num_,1)<<",  "<<foot_step_(current_step_num_+1,1)<<endl;
        cout<<"defined swing start , walking tick  " <<walking_tick_<<", support foot : "<<support_foot(0)<<", "<<support_foot(1)<<endl;
        cout<<"zmp est : "<<zmp_est_(0)<<", "<<zmp_est_(1)<<endl;
    }
    if(walking_tick_ == t_start_+t_total_-t_double2_-t_rest_last_ )
    {
//        cout<<"swing end, walking tick " <<walking_tick_<<",  support foot : "<<foot_step_support_frame_(current_step_num_-1,1)<<",  "<<foot_step_support_frame_(current_step_num_,1)<<",  "<<foot_step_support_frame_(current_step_num_+1,1)<<endl;
        cout<<"defined swing end , walking tick  " <<walking_tick_<<", support foot : "<<support_foot(0)<<", "<<support_foot(1)<<endl;
        cout<<"zmp est : "<<zmp_est_(0)<<", "<<zmp_est_(1)<<endl;
    }

    // decision for swing timing
//    if(foot_step_(current_step_num_,6) == 1 ) // right foot swing left foot support
//    {
//        if(swing_foot_flag_ == false){
//            //cout<<"foot step suppor frame : "<<support_foot(1) - 0.08<<endl;
////            cout<<"check "<<zmp_est(1) <<", "<<support_foot(1)<<endl;
//            if((zmp_est(1) >= (support_foot(1)-0.05 )) && (zmp_est(0)> (support_foot(0) - 0.06))){
//                t_swing_start_ = walking_tick_;
//                double t_Temp ;
//                if(t_swing_start_ > (t_start_real_ + t_double1_init_)){
//                    t_Temp = t_swing_start_ - (t_start_real_ + t_double1_init_);
////                    t_total_ = t_total_+ t_Temp;
////                    t_last_ = t_last_ + t_Temp;
//                    cout<<"swing time : "<<t_swing_start_<<" original double support time : "<<t_start_real_ + t_double1_init_<<endl;
//                }

//                swing_foot_flag_ = true;
//                matrix_get_flag_ = true;
//                cout<<endl<<"calculated right foot swing start  tick "<<walking_tick_<<"t_total : "<<t_total_<<", step num " <<current_step_num_<<", zmp est "<<zmp_est(0)<<", "<<zmp_est(1)<<", supportfoot "<<support_foot(0)<<", "<<support_foot(1)<<endl;
//            }
//            else{
//                if(walking_tick_>= t_start_real_+t_double1_init_){
////                   cout<<"double support 2 ver:"<<endl;
//                    t_double1_ ++;
//                    t_total_ ++;
//                    t_last_ ++;
//                }
//            }

//        }

//    }
//    if(foot_step_(current_step_num_,6) == 0){ // left foot swing right foot support
//        if(swing_foot_flag_ == false){
////            cout<<"check "<<zmp_est(1) <<", "<<support_foot(1)<<endl;
//            if((zmp_est(1) <= (support_foot(1)+0.05)) && (zmp_est(0)> (support_foot(0) - 0.06))){
//                t_swing_start_ = walking_tick_;
//                swing_foot_flag_ = true;
//                matrix_get_flag_ = true;
//                cout<<endl<<"calculated left foot swing start tick "<<walking_tick_<<"t_total : "<<t_total_<<", step num " <<current_step_num_<<", zmp est "<<zmp_est(0)<<", "<<zmp_est(1)<<", supportfoot "<<support_foot(0)<<", "<<support_foot(1)<<endl;

//                double t_Temp ;
//                if(t_swing_start_ > (t_start_real_ + t_double1_init_)){
//                    t_Temp = t_swing_start_ - (t_start_real_ + t_double1_init_);
//                    cout<<"swing time : "<<t_swing_start_<<" original double support time : "<<t_start_real_ + t_double1_init_<<endl;
////                    t_total_ = t_total_+ t_Temp;
////                    t_last_ = t_last_ + t_Temp;
//                }
//            }
//            else{
//                if(walking_tick_>= t_start_real_+t_double1_init_){
////                    cout<<"double support 2 ver:"<<endl;
//                    t_double1_ ++;
//                    t_total_ ++;
//                    t_last_ ++;
//                }
//            }
//        }

//    }


}
void WalkingController::getCOMestimation(){
    Eigen::Vector2d com_velocity;
    Eigen::Vector2d com_acceleration;

    com_velocity.setZero();
    com_acceleration.setZero();

    if(walking_tick_ == 0){
         com_velocity.setZero();
         com_acceleration.setZero();
         com_support_acc_pre_.setZero();
         com_support_pre_vel_.setZero();

         com_support_pre_pre_.setZero();

     }
     else if(walking_tick_ == 1){
         com_support_pre_pre_.setZero();
     }
     else {
         com_velocity(0) = (com_support_current_(0) - com_support_pre_(0))*hz_;
         com_velocity(1) = (com_support_current_(1) - com_support_pre_(1))*hz_;

 //        com_acceleration(0) = (com_support_pre_vel_(0) - com_velocity(0))*hz_;
 //        com_acceleration(1) = (com_support_pre_vel_(1) - com_velocity(1))*hz_;
         com_acceleration(0) = (com_support_current_(0) - 2*com_support_pre_(0) + com_support_pre_pre_(0))*hz_*hz_;
         com_acceleration(1) = (com_support_current_(1) - 2*com_support_pre_(1) + com_support_pre_pre_(1))*hz_*hz_;
     }
     com_support_acc_ = DyrosMath::lowPassFilter(com_acceleration,com_support_acc_pre_,1.0/hz_,0.01);
     com_support_vel_ = DyrosMath::lowPassFilter(com_velocity,com_support_pre_vel_,1.0/hz_,0.04);

     com_support_pre_vel_ = com_support_vel_;
     com_support_acc_pre_ = com_support_acc_;

     Eigen::Matrix<double, 1, 3>c;
     c(0) = 1.0; c(1) = 0.0; c(2) = -zc_/GRAVITY;

//     Eigen::Vector3d com_x_est, com_y_est;

//     com_x_est(0) = com_support_current_(0); com_x_est(1) = com_velocity(0);com_x_est(2) = com_support_acc_(0);
//     com_y_est(0) = com_support_current_(1); com_y_est(1) = com_velocity(1);com_y_est(2) = com_support_acc_(1);

//     Eigen::Vector2d zmp_est;

//     zmp_est_(0) = c*com_x_est;
//     zmp_est_(1) = c*com_y_est;

     Eigen::Vector3d com_x_est, com_y_est;

         com_x_est(0) = com_support_current_(0); com_x_est(1) = com_velocity(0); com_x_est(2) = com_support_acc_(0);
         com_y_est(0) = com_support_current_(1); com_y_est(1) = com_velocity(1); com_y_est(2) = com_support_acc_(1);

         zmp_est_(0) = c*com_x_est;
         zmp_est_(1) = c*com_y_est;

         Eigen::Vector3d com_x_ext_est, com_y_ext_est;

         Eigen::Vector2d zmp_ext_est_;
//         com_x_ext_est(0) = com_support_current_measured_(0); com_x_ext_est(1) = com_velocity(0);  com_x_ext_est(2) = com_ext_acc_(0); //xd_(2);//
//         com_y_ext_est(0) = com_support_current_measured_(1); com_y_ext_est(1) = com_velocity(1);  com_y_ext_est(2) = com_ext_acc_(1); //yd_(2);//

         zmp_ext_est_(0) = c*com_x_ext_est;
         zmp_ext_est_(1) = c*com_y_ext_est;

         Eigen::Vector2d zmp_ext1;
         Eigen::Vector3d com_ext_xd, com_ext_yd;

//         com_ext_xd(0) = com_support_current_measured_(0); com_ext_xd(1) = com_velocity(0); com_ext_xd(2) = x_d1_(2);
//         com_ext_yd(0) = com_support_current_measured_(1); com_ext_yd(1) = com_velocity(1); com_ext_yd(2) = y_d1_(2);

         zmp_ext1(0) = c*com_ext_xd;
         zmp_ext1(1) = c*com_ext_yd; // position external encoder, accleration with MPC

         Eigen::Vector2d zmp_ext2;
         Eigen::Vector3d com_ext_preview_x, com_ext_preview_y;

         com_ext_preview_x(0) = com_support_current_(0); com_ext_preview_x(1) = com_velocity(0); com_ext_preview_x(2) = xd_(2);
         com_ext_preview_y(0) = com_support_current_(1); com_ext_preview_y(1) = com_velocity(1); com_ext_preview_y(2) = yd_(2);

         zmp_ext2(0) = c*com_ext_preview_x;
         zmp_ext2(1) = c*com_ext_preview_y; // position external encoder, accleration with preview

         Eigen::Vector2d zmp_xd1;
         Eigen::Vector3d com_xd1, com_yd1;

         com_xd1(0) = x_d1_(0); com_xd1(1) = x_d1_(0); com_xd1(2) = x_d1_(2);
         com_yd1(0) = y_d1_(0); com_yd1(1) = y_d1_(1); com_yd1(2) = y_d1_(2);

         zmp_xd1(0) = c*com_xd1;
         zmp_xd1(1) = c*com_yd1; // with MPC desired

         Eigen::Vector2d zmp_xd;
         Eigen::Vector3d com_xd, com_yd;

    //     com_pa_x(0) = xd_(0); com_pa_x(1) = xd_(0); com_pa_x(2) = xd_(2);
    //     com_pa_y(0) = yd_(0); com_pa_y(1) = yd_(1); com_pa_y(2) = yd_(2);


         com_xd(0) = xd_(0); com_xd(1) = xd_(0); com_xd(2) = xd_(2);
         com_yd(0) = yd_(0); com_yd(1) = yd_(1); com_yd(2) = yd_(2);
         zmp_xd(0) = c*com_xd;
         zmp_xd(1) = c*com_yd; // with preview desired

 //    zmp_est = DyrosMath::lowPassFilter(zmp_est,zmp_est_,1.0/hz_,2.0);

 //    zmp_est_ = zmp_est;

     file[23]<<walking_tick_<<"\t"<<com_acceleration(0)<<"\t"<<com_acceleration(1)<<"\t"<<com_support_vel_(0)<<"\t"<<com_support_vel_(1)<<endl;
               //com_velocity(0)<<"\t"<<com_velocity(1)<<"\t"<<com_acceleration(0)<<"\t"<<com_acceleration(1)<<endl;


     com_support_pre_pre_ = com_support_pre_;
     com_support_pre_  = com_support_current_;

     if(walking_tick_ == t_start_ + t_total_- 1 && current_step_num_!= total_step_num_-1){
         Eigen::Vector3d com_pos_pre;
         Eigen::Vector3d com_pos;

         Eigen::Vector3d com_pos_pre_pre;
         Eigen::Vector3d com_pos_pre_temp;

         Eigen::Vector3d com_vel_pre;
         Eigen::Vector3d com_vel;

         Eigen::Matrix3d temp_rot;
         Eigen::Vector3d temp_pos;

         temp_rot = DyrosMath::rotateWithZ(-foot_step_support_frame_(current_step_num_,5));
         for(int i=0;i<3;i++)
             temp_pos(i) = foot_step_support_frame_(current_step_num_,i);

         com_pos_pre(0) = com_support_pre_(0);
         com_pos_pre(1) = com_support_pre_(1);
         com_pos = temp_rot*(com_pos_pre - temp_pos);

         com_pos_pre_pre(0) = com_support_pre_pre_(0);
         com_pos_pre_pre(1) = com_support_pre_pre_(1);
         com_pos_pre_temp = temp_rot*(com_pos_pre_pre - temp_pos);


         com_vel_pre(0) = com_support_pre_vel_(0);
         com_vel_pre(1) = com_support_pre_vel_(1);
         com_vel_pre(2) = 0.0;
         com_vel = temp_rot*com_vel_pre;

         com_support_pre_(0) = com_pos(0);
         com_support_pre_(1) = com_pos(1);
         com_support_pre_(2) = 0.0;

         com_support_pre_pre_(0) = com_pos_pre_temp(0);
         com_support_pre_pre_(1) = com_pos_pre_temp(1);
         com_support_pre_pre_(2) = 0.0;

         com_support_pre_vel_(0) = com_vel(0);
         com_support_pre_vel_(1) = com_vel(1);
         com_support_pre_vel_(2) = 0.0;
     }

     file[24]<<walking_tick_<<"\t"<<com_acceleration(0)<<"\t"<<com_acceleration(1)<<"\t"<<zmp_est_(0)<<"\t"<<zmp_est_(1)<<"\t"<<zmp_ext_est_(0)<<"\t"<<zmp_ext_est_(1)
            <<"\t"<<zmp_ext2(0)<<"\t"<<zmp_ext2(1)<<"\t"<<zmp_ext1(0)<<"\t"<<zmp_ext1(1) //position-external acc - preview and mpc
           <<"\t"<<zmp_xd(0)<<"\t"<<zmp_xd(1)<<"\t"<<zmp_xd1(0)<<"\t"<<zmp_xd1(1)<<endl; //with preview desired and mpc desired
}
void WalkingController::ZMP_2(Eigen::Vector6d ft_l, Eigen::Vector6d ft_r, Eigen::Vector3d l_pos, Eigen::Vector3d r_pos, Eigen::Vector3d& ZMP){




  Eigen::Vector2d zmp_r, zmp_l, zmp_r1, zmp_l1;
//  zmp_l1(0) = (ft_l(4)-0.0062*ft_l(0) - lfoot_support_current_.translation()(0)*ft_l(2))/(-ft_l(2));
//  zmp_l1(1) = (ft_l(3) + 0.0062*ft_l(1) - lfoot_support_current_.translation()(1)*ft_l(2))/(-ft_l(2));

//  zmp_r1(0) = (ft_r(4) - 0.0062*ft_r(0) - rfoot_support_current_.translation()(0)*ft_r(2))/(-ft_r(2));
//  zmp_r1(1) = (ft_r(3) + 0.0062*ft_r(1) - rfoot_support_current_.translation()(1)*ft_r(2))/(-ft_r(2));

  zmp_l1(0) = (ft_l(4)-0.0062*ft_l(0) - lfoot_support_current_.translation()(0)*ft_l(2))/(-ft_l(2));
  zmp_l1(1) = (ft_l(3) - 0.0062*ft_l(1) - lfoot_support_current_.translation()(1)*ft_l(2))/(-ft_l(2));

  zmp_r1(0) = (ft_r(4) - 0.0062*ft_r(0) - rfoot_support_current_.translation()(0)*ft_r(2))/(-ft_r(2));
  zmp_r1(1) = (ft_r(3) - 0.0062*ft_r(1) - rfoot_support_current_.translation()(1)*ft_r(2))/(-ft_r(2));


  ZMP(0) = (+zmp_l1(0)*ft_l(2) + zmp_r1(0)*ft_r(2))/(+ft_l(2) + ft_r(2));
  ZMP(1) = (zmp_l1(1)*ft_l(2) + zmp_r1(1)*ft_r(2))/(+ft_l(2) +ft_r(2));

//  file[33]<<"\t"<<ZMP(0)<<"\t"<<ZMP(1);


  file[33]<<walking_tick_<<"\t"<<ZMP(0)<<"\t"<<ZMP(1);

//  zmp_l(0) = (-ft_l(4)-0.0062*ft_l(0) + lfoot_trajectory_support_.translation()(0)*ft_l(2))/(ft_l(2));
//  zmp_l(1) = (ft_l(3) - 0.0062*ft_l(1) + lfoot_trajectory_support_.translation()(1)*ft_l(2))/ft_l(2);

//  zmp_r(0) = (-ft_r(4)-0.0062*ft_r(0) + rfoot_trajectory_support_.translation()(0)*ft_r(2))/(ft_r(2));
//  zmp_r(1) = (ft_r(3) - 0.0062*ft_r(1) + rfoot_trajectory_support_.translation()(1)*ft_r(2))/ft_r(2);

  zmp_l(0) = (-ft_l(4)-0.0062*ft_l(0) + lfoot_trajectory_support_.translation()(0)*ft_l(2))/(ft_l(2));
  zmp_l(1) = (ft_l(3) - 0.0062*ft_l(1) + lfoot_trajectory_support_.translation()(1)*ft_l(2))/ft_l(2);

  zmp_r(0) = (-ft_r(4)-0.0062*ft_r(0) + rfoot_trajectory_support_.translation()(0)*ft_r(2))/(ft_r(2));
  zmp_r(1) = (ft_r(3) - 0.0062*ft_r(1) + rfoot_trajectory_support_.translation()(1)*ft_r(2))/ft_r(2);

  if(ft_l(2) >0)
      ft_l(2) = 0.0;
  if(ft_r(2) >0)
      ft_r(2) = 0.0;


  ZMP(0) = (zmp_l(0)*ft_l(2) + zmp_r(0)*ft_r(2))/(ft_l(2) + ft_r(2));
  ZMP(1) = (zmp_l(1)*ft_l(2) + zmp_r(1)*ft_r(2))/(ft_l(2) + ft_r(2));

//  file[33]<<"\t"<<ZMP(0)<<"\t"<<ZMP(1);
  Eigen::Vector2d zmp_rr, zmp_ll;
  zmp_rr(0) = (-ft_r(4) - ft_r(0)*0.0062)/ft_r(2);
  zmp_rr(1) = (ft_r(3) - ft_r(1)*0.0062)/ft_r(2);

  zmp_ll(0) = (-ft_l(4) - ft_l(0)*0.0062)/ft_l(2);
  zmp_ll(1) = (ft_l(3) - ft_l(1)*0.0062)/ft_l(2);


  Eigen::Vector2d zmp_test;
  zmp_test(0) = (zmp_rr(0)*ft_r(2) + zmp_ll(0)*ft_l(2))/(ft_r(2)+ft_l(2));
  zmp_test(1) = (zmp_rr(1)*ft_r(2) + zmp_ll(1)*ft_l(2))/(ft_r(2)+ft_l(2));

//  file[33]<<walking_tick_<<"\t"<<zmp_rr(0)<<"\t"<<zmp_rr(1)<<"\t"<<zmp_ll(0)<<"\t"<<zmp_ll(1)<<"\t";

//  zmp_rr(0) = (-ft_r(4) )/ft_r(2);
//  zmp_rr(1) = (ft_r(3) )/ft_r(2);

//  zmp_ll(0) = (-ft_l(4) )/ft_l(2);
//  zmp_ll(1) = (ft_l(3) )/ft_l(2);

  file[33]<<"\t"<<ZMP(0)<<"\t"<<ZMP(1)<<"\t"<<zmp_test(0)<<"\t"<<zmp_test(1)<<"\t";

  if(foot_step_(current_step_num_,6) ==1)//left foot support
  {
      zmp_test(0) = (((rfoot_support_current_.linear().topLeftCorner<2,2>()*zmp_rr)(0) + rfoot_support_current_.translation()(0))*ft_r(2) + zmp_ll(0)*ft_l(2))/(ft_r(2)+ft_l(2));
      zmp_test(1) = (((rfoot_support_current_.linear().topLeftCorner<2,2>()*zmp_rr)(1) + rfoot_support_current_.translation()(1))*ft_r(2) + zmp_ll(1)*ft_l(2))/(ft_r(2)+ft_l(2));

  }
  else {
      zmp_test(0) = (((lfoot_support_current_.linear().topLeftCorner<2,2>()*zmp_ll)(0) + lfoot_support_current_.translation()(0))*ft_l(2) + zmp_rr(0)*ft_r(2))/(ft_r(2)+ft_l(2));
      zmp_test(1) = (((lfoot_support_current_.linear().topLeftCorner<2,2>()*zmp_ll)(1) + lfoot_support_current_.translation()(1))*ft_l(2) + zmp_rr(1)*ft_r(2))/(ft_r(2)+ft_l(2));
  }

  file[33]<<"\t"<<zmp_test(0)<<"\t"<<zmp_test(1)<<"\t";
}
void WalkingController::zmptoInitFloat()
{
  Eigen::Isometry3d reference;
  Eigen::Isometry3d reference1;
  Eigen::Vector3d temp_local_position;
  Eigen::Vector3d temp_global_position;

  Eigen::Isometry3d current_step_float_support_,current_step_support_float_;
   if(current_step_num_ == 0)
   {
     if(foot_step_(0,6) == 0) //right support
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
    if (walking_tick_ == 0)
    {
      reference.translation() = reference.translation();
      reference.linear() =  reference.linear();
      reference1.linear() = reference.linear().transpose();
      reference1.translation() = -1*reference.translation();
      current_step_float_support_ = reference;
      current_step_support_float_ = reference1;
    }
/*  if(walking_tick_ <= t_temp_+t_total_)
    {
      reference.translation()(1) = reference.translation()(1)/2.0;
    }*/
   }
   else
   {
     reference.linear() = DyrosMath::rotateWithZ(foot_step_(current_step_num_-1,5));
     for(int i=0 ;i<3; i++)
     reference.translation()(i) = foot_step_(current_step_num_-1,i);
     if(walking_tick_ == t_start_)
     {
       reference.translation() = reference.translation();
       reference.linear() =  reference.linear();
       reference1.linear() = reference.linear().transpose();
       reference1.translation() = -1*reference.translation();
       current_step_float_support_ = reference;
       current_step_support_float_ = reference1;
      }
   }

   Eigen::Vector2d left_zmp, right_zmp;

   left_zmp(0) = l_ft_(4)/l_ft_(2) ;// adding support foot position
   left_zmp(1) = l_ft_(3)/l_ft_(2) ;

   right_zmp(0) = r_ft_(4)/r_ft_(2);// adding support foot position
   right_zmp(1) = r_ft_(3)/r_ft_(2);


   zmp_measured_(0) = (left_zmp(0) * l_ft_(2) + right_zmp(0) * r_ft_(2))/(l_ft_(2) + r_ft_(2));
   zmp_measured_(1) = (left_zmp(1) * l_ft_(2) + right_zmp(1) * r_ft_(2))/(l_ft_(2) + r_ft_(2));

   Eigen::Vector3d zmp;
   zmp(0) = zmp_measured_(0); zmp(1) = zmp_measured_(1); zmp(2) = 0.0;

//   zmp_measured_ = current_step_support_float_.linear()*zmp + current_step_support_float_.translation();

   ////////////////////////////////////////////////////
   Eigen::Vector2d sum;
       Eigen::Vector3d ZMP_real_val;
       sum.setZero();
       ZMP_real_val.setZero();

       sum(0) = l_ft_(4)+r_ft_(4)-(lfoot_support_current_.translation()(0)*l_ft_(2)+rfoot_support_current_.translation()(0)*r_ft_(2));
       sum(1) = -l_ft_(3)-r_ft_(3)+(lfoot_support_current_.translation()(1)*l_ft_(2)+rfoot_support_current_.translation()(1)*r_ft_(2));

       ZMP_real_val(0) = -sum(0)/(l_ft_(2)+r_ft_(2));
       ZMP_real_val(1) = sum(1)/(l_ft_(2)+r_ft_(2));
       ZMP_real_val(2) = 0.0;

       Eigen::Vector2d Left_ZMP;
       Left_ZMP(0) = l_ft_(4)/l_ft_(2);
       Left_ZMP(1) = l_ft_(3)/l_ft_(2);
       Left_ZMP(0) = Left_ZMP(0)+lfoot_support_current_.translation()(0);
       Left_ZMP(1) = Left_ZMP(1)+lfoot_support_current_.translation()(1);

       Eigen::Vector2d Right_ZMP;
       Right_ZMP(0) = r_ft_(4)/r_ft_(2);
       Right_ZMP(1) = r_ft_(3)/r_ft_(2);
       Right_ZMP(0) = Right_ZMP(0)+rfoot_support_current_.translation()(0);
       Right_ZMP(1) = Right_ZMP(1)+rfoot_support_current_.translation()(1);

       ZMP_real_val(0) = (Left_ZMP(0)*l_ft_(2)+Right_ZMP(0)*r_ft_(2))/(l_ft_(2)+r_ft_(2));
       ZMP_real_val(1) = (Left_ZMP(1)*l_ft_(2)+Right_ZMP(1)*r_ft_(2))/(l_ft_(2)+r_ft_(2));


       Eigen::Vector4d zmp_temp, zmp_measured;
        zmp_temp(0) = ZMP_real_val(0);
        zmp_temp(1) = ZMP_real_val(1);
        zmp_temp(2) = 0.0;
        zmp_temp(3) = 1.0;

        zmp_measured = (current_step_float_support_*zmp_temp);


//   file[33]<<"\t"<<zmp_measured_(0)<<"\t"<<zmp_measured_(1)<<"\t"<<zmp_measured(0)<<"\t"<<zmp_measured(1);
}
void WalkingController::computeZmp()
{
  if(foot_step_(current_step_num_,6)==1 && walking_tick_ >= t_start_real_+t_double1_ && walking_tick_ < t_start_+t_total_-t_double2_-t_rest_last_) // left support and right swing phase
  {
    zmp_r_.setZero();
  }
  else
  {
    //zmp_r_(0) = (-r_ft_(4) - r_ft_(0)*0.0062) / r_ft_(2);
    //zmp_r_(1) = (r_ft_(3) - r_ft_(1)*0.0062) / r_ft_(2);
    zmp_r_(0) = (-r_ft_filtered_(4)) / r_ft_filtered_(2);
    zmp_r_(1) = (r_ft_filtered_(3)) / r_ft_filtered_(2);
  }

  if(foot_step_(current_step_num_,6)==0 && walking_tick_ >= t_start_real_+t_double1_ && walking_tick_ < t_start_+t_total_-t_double2_-t_rest_last_) // right support and left swing phase
  {
    zmp_l_.setZero();
  }
  else
  {
    //zmp_l_(0) = (-l_ft_(4) - l_ft_(0)*0.0062) / l_ft_(2);
    //zmp_l_(1) = (l_ft_(3) - l_ft_(1)*0.0062) / l_ft_(2);
    zmp_l_(0) = (-l_ft_filtered_(4)) / l_ft_filtered_(2);
    zmp_l_(1) = (l_ft_filtered_(3)) / l_ft_filtered_(2);

  }

  zmp_measured_ppre_ = zmp_measured_pre_;
  zmp_measured_pre_ = zmp_measured1_;

  if (foot_step_(current_step_num_,6)==1) //left foot support
  {
    zmp_measured1_(0) = ((((rfoot_support_current_.linear()).topLeftCorner<2, 2>()*zmp_r_)(0) + (rfoot_support_current_.translation())(0))*r_ft_(2) + zmp_l_(0)*l_ft_(2)) / (r_ft_(2) + l_ft_(2));
    zmp_measured1_(1) = ((((rfoot_support_current_.linear()).topLeftCorner<2, 2>()*zmp_r_)(1) + (rfoot_support_current_.translation())(1))*r_ft_(2) + zmp_l_(1)*l_ft_(2)) / (r_ft_(2) + l_ft_(2));
    f_ft_support_ = l_ft_.segment<3>(0) + rfoot_support_current_.linear()*r_ft_.segment<3>(0);
  }
  else
  {
    zmp_measured1_(0) = ((((lfoot_support_current_.linear()).topLeftCorner<2, 2>()*zmp_l_)(0) + (lfoot_support_current_.translation())(0))*l_ft_(2) + zmp_r_(0)*r_ft_(2)) / (r_ft_(2) + l_ft_(2));
    zmp_measured1_(1) = ((((lfoot_support_current_.linear()).topLeftCorner<2, 2>()*zmp_l_)(1) + (lfoot_support_current_.translation())(1))*l_ft_(2) + zmp_r_(1)*r_ft_(2)) / (r_ft_(2) + l_ft_(2));
    f_ft_support_ = r_ft_.segment<3>(0) + lfoot_support_current_.linear()*l_ft_.segment<3>(0);
  }

//  file[33]<<"\t"<<zmp_measured1_(0)<<"\t"<<zmp_measured1_(1)<<"\t"<<f_ft_support_(0)<<"\t"<<f_ft_support_(1)<<"\t"<<f_ft_support_(2)<<endl;
}
void WalkingController::zmpTest(const unsigned int norm_size, const unsigned planning_step_num){
    Eigen::MatrixXd ref_zmp1;
  ref_zmp_.resize(norm_size, 2);
  ref_zmp1.resize(norm_size,2);
  com_offset_.setZero();

  Eigen::VectorXd temp_px, temp_px1;
  Eigen::VectorXd temp_py;

  unsigned int index =0;

  if(current_step_num_ ==0)
  {
//      if(walking_tick_ ==0)
//          cout<<"com init on at 0 : "<<com_support_init_(1)+com_offset_(1)<<endl;
    for (int i=0; i<= t_temp_; i++) //200 tick
    {
      if(i <= 0.5*hz_)
      {
        ref_zmp_(i,0) = com_support_init_(0)+com_offset_(0);
        ref_zmp_(i,1) = com_support_init_(1)+com_offset_(1);
      }
      else if(i < 1.5*hz_)
      {
        double del_x = i-0.5*hz_;
        ref_zmp_(i,0) = com_support_init_(0)+com_offset_(0)-del_x*(com_support_init_(0)+com_offset_(0))/(1.0*hz_);
        ref_zmp_(i,1) = com_support_init_(1)+com_offset_(1);
      }
      else
      {
        ref_zmp_(i,0) = 0.0;
        ref_zmp_(i,1) = com_support_init_(1)+com_offset_(1);
      }

      ref_zmp1 = ref_zmp_;

      index++;
    }
  }

  if(current_step_num_ >= total_step_num_-planning_step_num)
  {
    for(unsigned int i = current_step_num_; i<total_step_num_ ; i++)
    {
     // zmpPattern(i,temp_px1,temp_py);
      onestepTest(i,temp_px,temp_py);
     // onestepZmp_modified(i,temp_px,temp_py);

      for (unsigned int j=0; j<t_total_; j++)
      {
        ref_zmp_(index+j,0) = temp_px(j);
        ref_zmp_(index+j,1) = temp_py(j);
       // ref_zmp_(index+j,0) = temp_px1(j);
      }
      index = index+t_total_;
    }

    for (unsigned int j=0; j<20*hz_; j++)
    {
      ref_zmp_(index+j,0) = ref_zmp_(index-1,0);
      ref_zmp_(index+j,1) = ref_zmp_(index-1,1);
      //ref_zmp_(index+j,0) = ref_zmp1(index-1,0);
    }
    index = index+20*hz_;
  }
  else
  {
    for(unsigned int i=current_step_num_; i < current_step_num_+planning_step_num; i++)
    {
    //  zmpPattern(i,temp_px1,temp_py);
      onestepTest(i,temp_px,temp_py);
      //onestepZmp_modified(i,temp_px,temp_py);
      for (unsigned int j=0; j<t_total_; j++)
      {
        ref_zmp_(index+j,0) = temp_px(j);
        ref_zmp_(index+j,1) = temp_py(j);
        //ref_zmp_(index+j,0) = temp_px1(j);
      }
      index = index+t_total_;
    }
  }
}
void WalkingController::onestepTest(unsigned int current_step_number, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py)
{
    temp_px.resize(t_total_);
    temp_py.resize(t_total_);
    temp_px.setZero();
    temp_py.setZero();

    double Kx = 0.0;
    double Kx2 = 0.0;
    double Ky = 0.0;
    double Ky2 = 0.0;

    if(current_step_number == 0)
    {
      Kx = supportfoot_support_init_offset_(0);
      Kx2 = (foot_step_support_frame_(current_step_number,0)- supportfoot_support_init_offset_(0))/2.0;

      //    Kx2 = (foot_step_support_frame_(current_step_number,0))/2.0;


      Ky = supportfoot_support_init_offset_(1) - com_support_init_(1);
      Ky2 = (foot_step_support_frame_(current_step_number,1)- supportfoot_support_init_offset_(1))/2.0;

      //    Ky2 = (foot_step_support_frame_(current_step_number,1))/2.0;

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
          temp_px(i) = supportfoot_support_init_offset_(0);// + Kx2/(t_double2_)*(i+1-(t_total_-t_double2_-t_rest_last_));
          temp_py(i) = supportfoot_support_init_offset_(1);// + Ky2/(t_double2_)*(i+1-(t_total_-t_double2_-t_rest_last_));
        }
        else
        {
          temp_px(i) = temp_px(i-1);
          temp_py(i) = temp_py(i-1);
        }
      }
    }
    else {
        for(int i=0;i<t_total_;i++){
            temp_px(i) = supportfoot_support_init_offset_(0);
            temp_py(i) = supportfoot_support_init_offset_(1);
        }
    }
}
void WalkingController::calculateStride(){
    double thigh = sqrt(pow(0.06,2)+pow(0.339,2));
    double calf = sqrt(pow(0.06,2)+pow(0.368,2));

    Eigen::Isometry3d left_hip_info,right_hip_info;
    left_hip_info = link_transform_[LF_LINK];
    right_hip_info = link_transform_[RF_LINK];

//    if(walking_tick_ == 0 ){
//        cout<<"length of thigh and calf : "<<thigh<<", "<<calf<<endl;
//        cout<<"infor of left hip : "<<endl<<left_hip_info.translation()<<endl<<"info of right hip "<<endl<<right_hip_info.translation()<<endl;
//    }

    Eigen::Isometry3d left_hip_support, right_hip_support;

    Eigen::Isometry3d ref_frame;

    if(current_step_num_ ==0){
        if(foot_step_(0, 6) == 0)  //right foot support
        {
          ref_frame = link_transform_[RF_LINK+5];//rfoot_float_init_;
        }
        else if(foot_step_(0, 6) == 1)
        {
          ref_frame = link_transform_[LF_LINK+5];//lfoot_float_init_;
        }
    }
    else{
        if(foot_step_(current_step_num_, 6) == 0)  //right foot support
        {
          ref_frame = link_transform_[RF_LINK+5];//rfoot_float_init_;
        }
        else if(foot_step_(current_step_num_, 6) == 1)
        {
          ref_frame = link_transform_[LF_LINK+5];//lfoot_float_init_;
        }
    }

    left_hip_support = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),left_hip_info);
    right_hip_support = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),right_hip_info);

//    if(walking_tick_ ==0){
//        cout<<"hip  for support "<<endl<<"left "<<endl<<left_hip_support.translation()<<endl<<"right "<<endl<<right_hip_support.translation()<<endl;
//    }

    double step_max, max_foot_pos;

    if(current_step_num_ ==0){
        if(foot_step_(0, 6) == 0)  //right foot support
        {
          step_max = sqrt(pow((thigh + calf),2)-pow(left_hip_support.translation()(2),2));
          max_foot_pos = rfoot_support_init_.translation()(0)+left_hip_support.translation()(0) + step_max;
        }
        else if(foot_step_(0, 6) == 1)
        {
            step_max = sqrt(pow((thigh + calf),2)-pow(right_hip_support.translation()(2),2));
            max_foot_pos = lfoot_support_init_.translation()(0)+ right_hip_support.translation()(0) + step_max;
        }
    }
    else{
        if(foot_step_(current_step_num_, 6) == 0)  //right foot support
        {
            step_max = sqrt(pow((thigh + calf),2)-pow(left_hip_support.translation()(2),2));
            max_foot_pos = rfoot_support_init_.translation()(0)+ left_hip_support.translation()(0) + step_max;
        }
        else if(foot_step_(current_step_num_, 6) == 1)
        {
            step_max = sqrt(pow((thigh + calf),2)-pow(right_hip_support.translation()(2),2));
            max_foot_pos = lfoot_support_init_.translation()(0) + right_hip_support.translation()(0)+ step_max;
        }
    }

    file[25]<<walking_tick_<<"\t"<<step_max<<"\t"<<max_foot_pos<<"\t"<<left_hip_info.translation()(2)<<"\t"<<right_hip_info.translation()(2)
           <<"\t"<<left_hip_info.translation()(0)<<"\t"<<right_hip_info.translation()(0)<<endl;

}
void WalkingController::calculateFootEdgeJaco(){
    double h_ankle, foot_length_half;
    h_ankle = 0.11; foot_length_half = 0.15;

    Eigen::Vector3d ankle_to_Toe, ankle_to_Heel;
    ankle_to_Toe.setZero(); ankle_to_Heel.setZero();

    ankle_to_Toe(0) = foot_length_half; ankle_to_Toe(2) = -h_ankle;
    ankle_to_Heel(0) = -foot_length_half; ankle_to_Heel(2) = -h_ankle;


    Eigen::Vector3d float_lleg_toe, float_lleg_heel, float_rleg_toe, float_rleg_heel;

    ltoe_float_current_.translation() = lfoot_float_current_.translation() + lfoot_float_current_.linear()*ankle_to_Toe;
    lheel_float_current_.translation() = lfoot_float_current_.translation() + lfoot_float_current_.linear()*ankle_to_Heel;

    rtoe_float_current_.translation() = rfoot_float_current_.translation() + rfoot_float_current_.linear()*ankle_to_Toe;
    rheel_float_current_.translation() = rfoot_float_current_.translation() + rfoot_float_current_.linear()*ankle_to_Heel;

//    ltoe_float_current_.linear().setIdentity();
//    rtoe_float_current_.linear().setIdentity();

//    lheel_float_current_.linear().setIdentity();
//    rheel_float_current_.linear().setIdentity();

    ltoe_float_current_.linear() = lfoot_float_current_.linear();
    lheel_float_current_.linear() = lfoot_float_current_.linear();

    rtoe_float_current_.linear() = rfoot_float_current_.linear();
    rheel_float_current_.linear() = rfoot_float_current_.linear();


    if(walking_tick_ == 0){
        pre_toe_q = current_q_.segment<12>(0);
    }
    if(walking_tick_ == 0 || walking_tick_ == t_start_){
        ltoe_float_init_ = ltoe_float_current_;
        rtoe_float_init_ = rtoe_float_current_;

        lheel_float_init_ = lheel_float_current_;
        rheel_float_init_ = rheel_float_current_;
    }

    Eigen::Matrix3d l_ankle_to_toe, l_ankle_to_heel, r_ankle_to_toe, r_ankle_to_heel;

    l_ankle_to_toe = DyrosMath::skew(lfoot_float_current_.linear()*ankle_to_Toe);
    l_ankle_to_heel = DyrosMath::skew(lfoot_float_current_.linear()*ankle_to_Heel);

    r_ankle_to_toe = DyrosMath::skew(rfoot_float_current_.linear()*ankle_to_Toe);
    r_ankle_to_heel = DyrosMath::skew(rfoot_float_current_.linear()*ankle_to_Heel);

    Eigen::Matrix6d left_toe_jaco, left_heel_jaco, right_toe_jaco, right_heel_jaco;
    Eigen::Matrix6d product_left_toe, product_left_heel, product_right_toe, product_right_heel;
    product_left_toe.setIdentity();    product_left_heel.setIdentity();    product_right_toe.setIdentity();    product_right_heel.setIdentity();

    product_left_toe.block<3,3>(0,3) = -l_ankle_to_toe;
    product_left_heel.block<3,3>(0,3) = -l_ankle_to_heel;
    product_right_toe.block<3,3>(0,3) = -r_ankle_to_toe;
    product_right_heel.block<3,3>(0,3) = -r_ankle_to_heel;

    current_left_toe_jacobian_ = product_left_toe*current_leg_jacobian_l_;
    current_left_heel_jacobian_ = product_left_heel*current_leg_jacobian_l_;
    current_right_toe_jacobian_ = product_right_toe*current_leg_jacobian_r_;
    current_right_heel_jacobian_ = product_right_heel*current_leg_jacobian_r_;


    Eigen::Matrix6d ltoe_jaco_inv,rtoe_jaco_inv, lheel_jaco_inv, rheel_jaco_inv;
    ltoe_jaco_inv = current_left_toe_jacobian_.inverse();
    rtoe_jaco_inv = current_right_toe_jacobian_.inverse();

    lheel_jaco_inv = current_left_heel_jacobian_.inverse();
    rheel_jaco_inv = current_right_heel_jacobian_.inverse();

}
void WalkingController::getOptimizedFootTrajectory(){
    double duration_ds1, duration_toe, duration_lo, duration_ld, duration_heel, duration_ds2;
    double theta_toe, theta_heel;
    Eigen::Vector3d ankle_to_toe, ankle_to_heel;
    ankle_to_toe.setZero(); ankle_to_heel.setZero();
    ankle_to_toe(0) = 0.15; ankle_to_toe(2) = -0.10;
    ankle_to_heel(0) = -0.15; ankle_to_heel(2) = -0.10;

    double duration_single_support;
    duration_single_support = t_start_+t_total_-t_double2_-(t_start_real_+t_double1_);
    double a_r, a_s;
    a_r = 0.2; a_s = 0.5-a_r;

    duration_toe = a_r*duration_single_support;         duration_heel = duration_toe;
    duration_lo = a_s*duration_single_support;          duration_ld = duration_lo;

    double t_ds1, t_toe, t_lo, t_ld, t_heel, t_ds2;

//    if(walking_tick_ == 0){
//        cout<<"duration of single : "<<duration_single_support<<endl;
//    }
    t_ds1 = t_start_real_ + t_double1_;
    t_toe = t_start_real_ + t_double1_ + duration_toe;
    t_lo = t_toe + duration_lo;
    t_ld = t_lo + duration_ld;
    t_heel = t_ld + duration_heel;

    if(walking_tick_ == 0 ){
        cout<<"swing timing check "<<endl<<"original landing time : "<<t_start_+t_total_-t_rest_last_-t_double2_<<endl<<"heel toe swing time : "<<t_ld<<endl;
    }

//    if(current_step_num_ == 1 && walking_tick_ == t_start_){
//        cout<<"==== time ==== "<<endl<<"t ds1 : "<<t_ds1<<endl<<"t toe " <<t_toe<<endl<<"t lo : "<<t_lo<<endl<<"t ld : "<<t_ld<<endl<<"t heel : "<<t_heel<<endl;
//    }
    Eigen::Vector6d target_swing_foot;

    Eigen::Vector3d ankle_at_toe, ankle_at_heel;
    double delta_foot_height;


//    if(walking_tick_==t_start_){
//        cout<<"t ds1 : "<<t_ds1<<", t toe : "<<t_toe<<",  t lo : "<<t_lo<<"< t ld : "<<t_ld<<", t heel " <<t_heel<<endl;
//    }
    for(int i=0;i<6;i++)
        target_swing_foot(i) = foot_step_support_frame_(current_step_num_,i);


    theta_toe = 45*DEG2RAD; theta_heel = -10*DEG2RAD;
    if(current_step_num_ == total_step_num_-1){
        theta_toe = 0.0; theta_heel = 0.0;
    }
    Eigen::Vector3d foot_velocity, ankle_toe_velocity, ankle_heel_velocity;


    //if(walking_tick_ == t_start_){
        if(foot_step_(current_step_num_,6) ==1)// left foot support
        {
            ankle_at_toe = rfoot_support_init_.translation() + ankle_to_toe + DyrosMath::rotateWithY(theta_toe)*(-ankle_to_toe);
//            if(walking_tick_ == t_start_)
//                cout<<"right ankle at toe : "<<ankle_at_toe<<endl;
            ankle_toe_velocity = (ankle_at_heel-rfoot_support_init_.translation())/(t_toe-t_start_real_);
        }
        else
        {
            ankle_at_toe = lfoot_support_init_.translation() + ankle_to_toe + DyrosMath::rotateWithY(theta_toe)*(-ankle_to_toe);
//            if(walking_tick_ == t_start_)
//                cout<<"left ankle at toe : "<<ankle_at_toe<<endl
            ankle_toe_velocity = (ankle_at_heel-lfoot_support_init_.translation())/(t_toe-t_start_real_);
        }

        ankle_at_heel = target_swing_foot.segment<3>(0) + ankle_to_heel + DyrosMath::rotateWithY(theta_heel)*(-ankle_to_heel);
//        if(walking_tick_ == t_start_)
//            cout<<"ankle at heel : "<<ankle_at_heel<<endl;
    //}

     delta_foot_height = ankle_at_toe(2) + 0.04;

     foot_velocity = (ankle_at_heel-ankle_at_toe)/(t_heel -t_start_real_);
     foot_velocity(2) = (delta_foot_height- 0.0)/(t_lo-t_start_real_);
     ankle_heel_velocity = (target_swing_foot.segment<3>(0) - ankle_at_heel)/duration_heel;

//     if(walking_tick_ == t_start_){
//         cout<<"velocity condition "<<endl<<"ankle at toe velocity : "<<ankle_toe_velocity<<endl<<"swing foot velocity : "<<foot_velocity<<endl<<"ankle at heel velocity : "<<ankle_heel_velocity<<endl;
//     }

     Eigen::Matrix3d diff_R_y;
     diff_R_y.setZero();
     diff_R_y(0,0) = -sin(theta_toe); diff_R_y(0,2) = cos(theta_toe);
     diff_R_y(2,0) = -cos(theta_toe); diff_R_y(2,2) = -sin(theta_toe);

     Eigen::Matrix<double, 3,1> R_P_toea_ankle;
     R_P_toea_ankle = diff_R_y*(-ankle_to_toe);

     double theta_condition;
     theta_condition = (R_P_toea_ankle(0)*foot_velocity(0) + R_P_toea_ankle(1) * foot_velocity(1)+R_P_toea_ankle(2)*foot_velocity(2))/(pow(R_P_toea_ankle(0),2) + pow(R_P_toea_ankle(1),2)+pow(R_P_toea_ankle(2),2) );


    if(walking_tick_< t_start_real_ ){//if(walking_tick_ < t_start_real_ + t_double1_){
        lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
        lfoot_trajectory_dot_support_.setZero();

        lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;
        rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;

        rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
        rfoot_trajectory_dot_support_.setZero();
        if(foot_step_(current_step_num_,6) ==1){ // left foot support
            lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,lfoot_support_init_.translation()(2),0.0,0.0,0.0);
            lfoot_trajectory_dot_support_(2) = DyrosMath::cubicDot(walking_tick_,t_start_,t_start_real_,lfoot_support_init_.translation()(2),0.0,0.0,0.0,hz_);
            for(int i=0;i<2;i++){
                lfoot_trajectory_euler_support_(i) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,lfoot_support_euler_init_(i),0.0,0.0,0.0);
                lfoot_trajectory_dot_support_(i+3) = DyrosMath::cubicDot(walking_tick_,t_start_,t_start_real_,lfoot_support_euler_init_(i),0.0,0.0,0.0,hz_);
            }

        }
        else { // right foot support
            rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,rfoot_support_init_.translation()(2),0.0,0.0,0.0);
            rfoot_trajectory_dot_support_(2) = DyrosMath::cubicDot(walking_tick_,t_start_,t_start_real_,rfoot_support_init_.translation()(2),0.0,0.0,0.0,hz_);
            for(int i=0;i<2;i++){
                rfoot_trajectory_euler_support_(i) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,rfoot_support_euler_init_(i),0.0,0.0,0.0);
                rfoot_trajectory_dot_support_(i+3) = DyrosMath::cubicDot(walking_tick_,t_start_,t_start_real_,rfoot_support_euler_init_(i),0.0,0.0,0.0,hz_);
            }
        }

        lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
        rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
    }
//    else if(walking_tick_ >= t_start_real_+t_double1_ && walking_tick_ < t_ld)//t_start_+t_total_-t_double2_-t_rest_last_)
    else if(walking_tick_ >= t_start_real_ && walking_tick_ < t_ld)//t_start_+t_total_-t_double2_-t_rest_last_)
    {// swing phase
        t_ds1 = t_start_real_;
        if(walking_tick_<t_toe)
        {// rotating around toe phase
            if(foot_step_(current_step_num_,6) ==1){// left foot support
                rfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_ds1,t_toe,0.0,theta_toe,0.0,theta_condition);
                rfoot_trajectory_dot_support_(1+3) = DyrosMath::cubicDot(walking_tick_,t_ds1,t_toe,0.0,theta_toe,0.0,0.0,hz_);

                rfoot_trajectory_support_.translation()=rfoot_support_init_.translation() + ankle_to_toe + DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*(-ankle_to_toe);
//                for(int i=0;i<3;i++)
//                    rfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_ds1,t_toe,rfoot_support_init_.translation()(i),ankle_at_toe(i),0.0,foot_velocity(i));

                rfoot_trajectory_dot_support_.segment<3>(0) = DyrosMath::rotateDotWithY(rfoot_trajectory_euler_support_(1))*(-ankle_to_toe);

            }
            else{
                lfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_ds1,t_toe,0.0,theta_toe,0.0,theta_condition);
                lfoot_trajectory_dot_support_(1+3) = DyrosMath::cubicDot(walking_tick_,t_ds1,t_toe,0.0,theta_toe,0.0,0.0,hz_);

                lfoot_trajectory_support_.translation()=lfoot_support_init_.translation() + ankle_to_toe + DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*(-ankle_to_toe);
//                for(int i=0;i<3;i++)
//                    lfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_ds1,t_toe,lfoot_support_init_.translation()(i),ankle_at_toe(i),0.0,foot_velocity(i));


                lfoot_trajectory_dot_support_.segment<3>(0) = DyrosMath::rotateDotWithY(lfoot_trajectory_euler_support_(1))*(-ankle_to_toe);
            }
        }
        else if(walking_tick_ >= t_toe && walking_tick_ < t_lo)
        { // lifting the foot upto highest height
            if(foot_step_(current_step_num_,6) ==1){// left foot support
                rfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_toe,t_ld,theta_toe,theta_heel,theta_condition,0.0);
                rfoot_trajectory_dot_support_(1+3) = DyrosMath::cubicDot(walking_tick_,t_toe,t_ld,theta_toe,theta_heel,0.0,0.0,hz_);

                for(int i=0;i<2;i++){
                    rfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_toe,t_ld,ankle_at_toe(i),ankle_at_heel(i),foot_velocity(i),foot_velocity(i));
                    rfoot_trajectory_dot_support_(i) = DyrosMath::cubicDot(walking_tick_,t_toe,t_ld,ankle_at_toe(i),ankle_at_heel(i),0.0,0.0,hz_);
                }

                rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_toe,t_lo,ankle_at_toe(2),delta_foot_height,foot_velocity(2),0.0);
                rfoot_trajectory_dot_support_(2) = DyrosMath::cubicDot(walking_tick_,t_toe,t_lo,ankle_at_toe(2),delta_foot_height,0.0,0.0,hz_);
            }
            else{
                lfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_toe,t_ld,theta_toe,theta_heel,theta_condition,0.0);
                lfoot_trajectory_dot_support_(1+3) = DyrosMath::cubicDot(walking_tick_,t_toe,t_ld,theta_toe,theta_heel,0.0,0.0,hz_);

                for(int i=0;i<2;i++){
                    lfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_toe,t_ld,ankle_at_toe(i),ankle_at_heel(i),foot_velocity(i),foot_velocity(i));
                    lfoot_trajectory_dot_support_(i) = DyrosMath::cubicDot(walking_tick_,t_toe,t_ld,ankle_at_toe(i),ankle_at_heel(i),0.0,0.0,hz_);
                }

                lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_toe,t_lo,ankle_at_toe(2),delta_foot_height,foot_velocity(2),0.0);
                lfoot_trajectory_dot_support_(2) = DyrosMath::cubicDot(walking_tick_,t_toe,t_lo,ankle_at_toe(2),delta_foot_height,0.0,0.0,hz_);
            }
        }
        else if(walking_tick_ >= t_lo && walking_tick_ < t_ld)
        { // landing the foot down to heel contact point
            if(foot_step_(current_step_num_,6) ==1){// left foot support
                rfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_toe,t_ld,theta_toe,theta_heel,0.0,theta_condition);
                rfoot_trajectory_dot_support_(1+3) = DyrosMath::cubicDot(walking_tick_,t_toe,t_ld,theta_toe,theta_heel,0.0,0.0,hz_);

                for(int i=0;i<2;i++){
                    rfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_toe,t_ld,ankle_at_toe(i),ankle_at_heel(i),foot_velocity(i),foot_velocity(i));
                    rfoot_trajectory_dot_support_(i) = DyrosMath::cubicDot(walking_tick_,t_toe,t_ld,ankle_at_toe(i),ankle_at_heel(i),0.0,0.0,hz_);
                }

                rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_lo,t_ld,delta_foot_height,ankle_at_heel(2),0,foot_velocity(2));
                rfoot_trajectory_dot_support_(2) = DyrosMath::cubicDot(walking_tick_,t_lo,t_ld,delta_foot_height,ankle_at_heel(2),0.0,0.0,hz_);
            }
            else{
                lfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_toe,t_ld,theta_toe,theta_heel,0.0,theta_condition);
                lfoot_trajectory_dot_support_(1) = DyrosMath::cubicDot(walking_tick_,t_toe,t_ld,theta_toe,theta_heel,0.0,0.0,hz_);

                for(int i=0;i<2;i++){
                    lfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_toe,t_ld,ankle_at_toe(i),ankle_at_heel(i),foot_velocity(i),foot_velocity(i));
                    lfoot_trajectory_dot_support_(i) = DyrosMath::cubicDot(walking_tick_,t_toe,t_ld,ankle_at_toe(i),ankle_at_heel(i),0.0,0.0,hz_);
                }

                lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_lo,t_ld,delta_foot_height,ankle_at_heel(2),0,foot_velocity(2));
                lfoot_trajectory_dot_support_(2) = DyrosMath::cubicDot(walking_tick_,t_lo,t_ld,delta_foot_height,ankle_at_heel(2),0.0,0.0,hz_);
            }
        }
//        else {// rotating around the heel line
//            if(foot_step_(current_step_num_,6) ==1){// left foot support
//                rfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_ld,t_heel,theta_heel,0.0,0.0,0.0);
//                rfoot_trajectory_dot_support_(1+3) = DyrosMath::cubicDot(walking_tick_,t_ld,t_heel,theta_heel,0.0,0.0,0.0,hz_);

//                rfoot_trajectory_support_.translation()=target_swing_foot.segment<3>(0) + ankle_to_heel + DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*(-ankle_to_heel);
//                rfoot_trajectory_dot_support_.segment<3>(0) = DyrosMath::rotateDotWithY(rfoot_trajectory_euler_support_(1))*(-ankle_to_heel);
//            }
//            else{
//                lfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_ld,t_heel,theta_heel,0.0,0.0,0.0);
//                lfoot_trajectory_dot_support_(1+3) = DyrosMath::cubicDot(walking_tick_,t_ld,t_heel,theta_heel,0.0,0.0,0.0,hz_);

//                lfoot_trajectory_support_.translation()=target_swing_foot.segment<3>(0) + ankle_to_heel + DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*(-ankle_to_heel);
//                lfoot_trajectory_dot_support_.segment<3>(0) = DyrosMath::rotateDotWithY(lfoot_trajectory_euler_support_(1))*(-ankle_to_heel);
//            }
//        }

        lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
        rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));

    }
    else{
        if(walking_tick_>= t_ld && walking_tick_ < t_heel){
            if(foot_step_(current_step_num_,6) ==1){// left foot support
                rfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_ld,t_heel,theta_heel,0.0,theta_condition,0.0);
                rfoot_trajectory_dot_support_(1+3) = DyrosMath::cubicDot(walking_tick_,t_ld,t_heel,theta_heel,0.0,0.0,0.0,hz_);

                rfoot_trajectory_support_.translation()=target_swing_foot.segment<3>(0) + ankle_to_heel + DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*(-ankle_to_heel);
//                for(int i=0;i<3;i++)
//                    rfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_ld,t_heel,ankle_at_heel(i),target_swing_foot(i),foot_velocity(i),0.0);
                rfoot_trajectory_dot_support_.segment<3>(0) = DyrosMath::rotateDotWithY(rfoot_trajectory_euler_support_(1))*(-ankle_to_heel);
            }
            else{
                lfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_ld,t_heel,theta_heel,0.0,theta_condition,0.0);
                lfoot_trajectory_dot_support_(1+3) = DyrosMath::cubicDot(walking_tick_,t_ld,t_heel,theta_heel,0.0,0.0,0.0,hz_);

                lfoot_trajectory_support_.translation()=target_swing_foot.segment<3>(0) + ankle_to_heel + DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*(-ankle_to_heel);
//                for(int i=0;i<3;i++)
//                    lfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_ld,t_heel,ankle_at_heel(i),target_swing_foot(i),foot_velocity(i),0.0);

                lfoot_trajectory_dot_support_.segment<3>(0) = DyrosMath::rotateDotWithY(lfoot_trajectory_euler_support_(1))*(-ankle_to_heel);
            }

            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
        }
        else{
            if(foot_step_(current_step_num_,6) == 1){//left foot support
                for(int i=0; i<3; i++)
                {
                    rfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
                    rfoot_trajectory_euler_support_(i) = target_swing_foot(i+3);
                }
                rfoot_trajectory_dot_support_.setZero();
            }
            else{//right foot support

                for(int i=0; i<3; i++)
                {
                  lfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
                  lfoot_trajectory_euler_support_(i) = target_swing_foot(i+3);
                }
                lfoot_trajectory_dot_support_.setZero();
            }

            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
        }
    }
}
void WalkingController::comHeight(){
    if(walking_tick_ == 0){
        cout<<"com height modification "<<endl;
        cout<<"pelv support init : "<<pelv_support_init_.translation()(2)<<"\t"<<com_height_<<endl;
    }
    if(walking_tick_<t_start_){
//                com_desired_(2) = DyrosMath::cubic(walking_tick_,50,300,pelv_support_init_.translation()(2),pelv_support_init_.translation()(2)+com_height_,0.0,0.0);
        com_desired_(2) = DyrosMath::cubic(walking_tick_,50,300,pelv_support_init_.translation()(2),com_height_,0.0,0.0);

        if(walking_tick_ == 300){
         //   pelv_suppprt_start_.translation()(2) = pelv_suppprt_start_.translation()(2) + com_height_;
            pelv_suppprt_start_.translation()(2) = com_height_;
            pelv_support_init_.translation()(2) = com_height_;
        }
    }

}
void WalkingController::MovingZMPtrajectory()
{
    unsigned int planning_step_number  = 3;

    unsigned int norm_size = 0;

    if(current_step_num_ >= total_step_num_ - planning_step_number)
      norm_size = (t_last_-t_start_+1)*(total_step_num_-current_step_num_)+20*hz_;
    else
      norm_size = (t_last_-t_start_+1)*(planning_step_number);
    if(current_step_num_ == 0)
      norm_size = norm_size + t_temp_+1;

    addZmpOffset();
    MovingZMPGenerator(norm_size,planning_step_number);

}
void WalkingController::MovingZMPGenerator(const unsigned int norm_size, const unsigned planning_step_num){
    Eigen::MatrixXd ref_zmp1;
  ref_zmp_.resize(norm_size, 2);
  ref_zmp1.resize(norm_size,2);
  com_offset_.setZero();

  Eigen::VectorXd temp_px, temp_px1;
  Eigen::VectorXd temp_py;

  unsigned int index =0;

  if(current_step_num_ ==0)
  {
//      if(walking_tick_ ==0)
//          cout<<"com init on at 0 : "<<com_support_init_(1)+com_offset_(1)<<endl;
    for (int i=0; i<= t_temp_; i++) //200 tick
    {
      if(i <= 0.5*hz_)
      {
        ref_zmp_(i,0) = com_support_init_(0)+com_offset_(0);
        ref_zmp_(i,1) = com_support_init_(1)+com_offset_(1);
      }
      else if(i < 1.5*hz_)
      {
        double del_x = i-0.5*hz_;
        ref_zmp_(i,0) = com_support_init_(0)+com_offset_(0)-del_x*(com_support_init_(0)+com_offset_(0))/(1.0*hz_);
        ref_zmp_(i,1) = com_support_init_(1)+com_offset_(1);
      }
      else
      {
        ref_zmp_(i,0) = 0.0;
        ref_zmp_(i,1) = com_support_init_(1)+com_offset_(1);
      }

      index++;
    }
  }
  if(current_step_num_ >= total_step_num_-planning_step_num)
  {
    for(unsigned int i = current_step_num_; i<total_step_num_ ; i++)
    {
     // zmpPattern(i,temp_px1,temp_py);
      MovingonestepZMP(i,temp_px,temp_py);

      if(walking_tick_ ==t_start_)
          cout<<"walking tick "<<walking_tick_<<"  zmp onestepzmp ,current step  "<<current_step_num_<<", i : "<<i<<endl;

      for (unsigned int j=0; j<t_total_; j++)
      {
        ref_zmp_(index+j,0) = temp_px(j);
        ref_zmp_(index+j,1) = temp_py(j);

      }
      index = index+t_total_;
    }

    for (unsigned int j=0; j<20*hz_; j++)
    {
      ref_zmp_(index+j,0) = ref_zmp_(index-1,0);
      ref_zmp_(index+j,1) = ref_zmp_(index-1,1);

    }
    index = index+20*hz_;
  }
  else
  {
    for(unsigned int i=current_step_num_; i < current_step_num_+planning_step_num; i++)
    {
      MovingonestepZMP(i,temp_px,temp_py);

      for (unsigned int j=0; j<t_total_; j++)
      {
        ref_zmp_(index+j,0) = temp_px(j);
        ref_zmp_(index+j,1) = temp_py(j);
        //ref_zmp_(index+j,0) = temp_px1(j);
      }

      index = index+t_total_;
    }
  }
//  if(walking_tick_ == 0 || walking_tick_==t_start_){
////      cout<<"in walking controoler : "<<foot_step_support_frame_offset_(current_step_num_-1,1)<<endl;
//      file[11]<<walking_tick_;
//      file[14]<<walking_tick_;
//      for(int i=0;i<norm_size;i++){
//          file[11]<<"\t"<<ref_zmp_(i,1);
//          file[14]<<"\t"<<ref_zmp_(i,0);
//      }
//      file[14]<<endl;
//      file[11]<<endl;
//  }
}
void WalkingController::MovingonestepZMP(unsigned int current_step_number, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py){
    temp_px.resize(t_total_);
    temp_py.resize(t_total_);
    temp_px.setZero();
    temp_py.setZero();

    double Kx = 0.0;
    double Kx2 = 0.0;
    double Ky = 0.0;
    double Ky2 = 0.0;

    double moving_x_offset;

    if((step_length_x_-0.15)<0){
        moving_x_offset = 0.0;
    }
    else if(((step_length_x_-0.15)>=0 )&&((step_length_x_-0.15) < 5 ))
    {
        moving_x_offset = 0.05;
    }
    else{
        moving_x_offset = (int) step_length_x_/3.0;

        if(moving_x_offset >=10)
            moving_x_offset = 10;

        moving_x_offset *=0.01;
    }

    moving_x_offset = 0.1*step_length_x_;
//    if(current_step_number == total_step_num_-1)
//        moving_x_offset = 0.0;

    double moving_duration;
    moving_duration = t_total_-t_rest_last_-t_double2_-(t_rest_init_+t_double1_);

    double duration_toe, duration_lo, duration_ld, duration_heel;

    double duration_single_support;
    duration_single_support = t_start_+t_total_-t_double2_-(t_start_real_+t_double1_);
    double a_r, a_s;
    a_r = 0.2; a_s = 0.5-a_r;

    duration_toe = a_r*duration_single_support;         duration_heel = duration_toe;
    duration_lo = a_s*duration_single_support;          duration_ld = duration_lo;

    double t_ds1, t_toe, t_lo, t_ld, t_heel;

    t_ds1 = t_rest_init_ + t_double1_;
    t_toe = t_rest_init_ + t_double1_ + duration_toe;
    t_lo = t_toe + duration_lo;
    t_ld = t_lo + duration_ld;
    t_heel = t_ld + duration_heel;



    if(current_step_number == 0)
    {
      Kx = supportfoot_support_init_offset_(0);
      Kx2 = (foot_step_support_frame_(current_step_number,0)- supportfoot_support_init_offset_(0))/2.0 - moving_x_offset;

      //    Kx2 = (foot_step_support_frame_(current_step_number,0))/2.0;


      Ky = supportfoot_support_init_offset_(1) - com_support_init_(1);
      Ky2 = (foot_step_support_frame_(current_step_number,1)- supportfoot_support_init_offset_(1))/2.0;

      //    Ky2 = (foot_step_support_frame_(current_step_number,1))/2.0;

      for(int i=0; i<t_total_; i++)
      {
        if(i < t_rest_init_)
        {
          temp_px(i) = 0.0;
          temp_py(i) = com_support_init_(1)+com_offset_(1);
        }
        else if(i >= t_rest_init_ && i < t_toe)
        {
          temp_px(i) = Kx/(t_double1_+duration_toe)*(i+1-t_rest_init_);
          temp_py(i) = com_support_init_(1)+com_offset_(1) + Ky/(t_double1_+duration_toe)*(i+1-t_rest_init_);
        }
        else if(i>= t_toe && i< t_ld)
        {
          temp_px(i) = supportfoot_support_init_offset_(0) + moving_x_offset/(t_ld-t_toe)*(i+1-(t_toe));
          temp_py(i) = supportfoot_support_init_offset_(1);
        }
        else if(i >= t_ld && i< t_total_-t_rest_last_)
        {
          temp_px(i) = supportfoot_support_init_offset_(0) + moving_x_offset + Kx2/(t_total_-t_rest_last_-t_ld)*(i+1-(t_ld));
          temp_py(i) = supportfoot_support_init_offset_(1) + Ky2/(t_total_-t_rest_last_-t_ld)*(i+1-(t_ld));
        }
        else
        {
          temp_px(i) = temp_px(i-1);
          temp_py(i) = temp_py(i-1);
        }
      }
    }
    else if(current_step_number == 1)
    {
      Kx = foot_step_support_frame_offset_(current_step_number-1,0) - (foot_step_support_frame_(current_step_number-1,0) + supportfoot_support_init_(0))/2.0 - moving_x_offset;
      Kx2 = (foot_step_support_frame_(current_step_number,0)+foot_step_support_frame_(current_step_number-1,0))/2.0 - foot_step_support_frame_offset_(current_step_number-1,0) -moving_x_offset;

      Ky =  foot_step_support_frame_offset_(current_step_number-1,1) - (foot_step_support_frame_(current_step_number-1,1) + supportfoot_support_init_(1))/2.0;
      Ky2 = (foot_step_support_frame_(current_step_number,1)+foot_step_support_frame_(current_step_number-1,1))/2.0 - foot_step_support_frame_offset_(current_step_number-1,1);


      for(int i=0; i<t_total_; i++)
      {
        if(i < t_rest_init_)
        {
          temp_px(i) = (foot_step_support_frame_(current_step_number-1,0)+supportfoot_support_init_(0))/2.0;
          temp_py(i) = (foot_step_support_frame_(current_step_number-1,1)+supportfoot_support_init_(1))/2.0;

        }
        else if(i >= t_rest_init_ && i < t_toe)
        {
          temp_px(i) = (foot_step_support_frame_(current_step_number-1,0)+supportfoot_support_init_(0))/2.0 + Kx/(t_toe - t_rest_init_)*(i+1-t_rest_init_);
          temp_py(i) = (foot_step_support_frame_(current_step_number-1,1)+supportfoot_support_init_(1))/2.0 + Ky/(t_toe - t_rest_init_)*(i+1-t_rest_init_);
        }
        else if(i>= t_toe && i< t_ld)
        {
          temp_px(i) = foot_step_support_frame_offset_(current_step_number-1,0)-moving_x_offset + 2*moving_x_offset/(t_ld-t_toe)*(i+1 - t_toe);
          temp_py(i) = foot_step_support_frame_offset_(current_step_number-1,1);
        }
        else if(i >= t_ld && i< t_total_-t_rest_last_)
        {
          temp_px(i) = foot_step_support_frame_offset_(current_step_number-1,0) +moving_x_offset + Kx2/(t_total_-t_rest_last_-t_ld)*(i+1-(t_ld));
          temp_py(i) = foot_step_support_frame_offset_(current_step_number-1,1) + Ky2/(t_total_-t_rest_last_-t_ld)*(i+1-(t_ld));
        }
        else
        {
          temp_px(i) = temp_px(i-1);
          temp_py(i) = temp_py(i-1);
        }
      }
    }
    else if(current_step_number >= total_step_num_-2){
        unsigned int step_check = total_step_num_/2;
        int offset_time;
        if(step_check%2 ==0){
            if(current_step_number == total_step_num_-2){
                if(walking_tick_ == t_start_)
                    cout<<"1111 total step num -2 " <<endl;
                Kx = foot_step_support_frame_offset_(current_step_number-1,0) - (foot_step_support_frame_(current_step_number-1,0) + foot_step_support_frame_(current_step_number-2,0))/2.0 - moving_x_offset;
                Kx2 = (foot_step_support_frame_(current_step_number,0)+foot_step_support_frame_(current_step_number-1,0))/2.0 - foot_step_support_frame_offset_(current_step_number-1,0);

                offset_time = 1.0;
            }
            else if(current_step_number == total_step_num_-1){
                if(walking_tick_ == t_start_)
                    cout<<"1111 total step num - 1  " <<endl;
                Kx = foot_step_support_frame_offset_(current_step_number-1,0) - (foot_step_support_frame_(current_step_number-1,0) + foot_step_support_frame_(current_step_number-2,0))/2.0;
                Kx2 = (foot_step_support_frame_(current_step_number,0)+foot_step_support_frame_(current_step_number-1,0))/2.0 - foot_step_support_frame_offset_(current_step_number-1,0);

                moving_x_offset = 0;
                offset_time = 1.0;
            }
        }
        else{
            if(current_step_number == total_step_num_-2){
                if(walking_tick_ == t_start_)
                    cout<<" total step num -2 " <<endl;
                Kx = foot_step_support_frame_offset_(current_step_number-1,0) - (foot_step_support_frame_(current_step_number-1,0) + foot_step_support_frame_(current_step_number-2,0))/2.0 - moving_x_offset;
                Kx2 = (foot_step_support_frame_(current_step_number,0)+foot_step_support_frame_(current_step_number-1,0))/2.0 - foot_step_support_frame_offset_(current_step_number-1,0) - moving_x_offset;

                offset_time = 2.0;
            }
            else if(current_step_number == total_step_num_-1){
                if(walking_tick_ == t_start_)
                    cout<<" total step num - 1  " <<endl;
                Kx = foot_step_support_frame_offset_(current_step_number-1,0) - (foot_step_support_frame_(current_step_number-1,0) + foot_step_support_frame_(current_step_number-2,0))/2.0 -moving_x_offset ;
                Kx2 = (foot_step_support_frame_(current_step_number,0)+foot_step_support_frame_(current_step_number-1,0))/2.0 - foot_step_support_frame_offset_(current_step_number-1,0);

                offset_time = 1.0;
            }


        }

        Ky =  foot_step_support_frame_offset_(current_step_number-1,1) - (foot_step_support_frame_(current_step_number-1,1) + foot_step_support_frame_(current_step_number-2,1))/2.0;
        Ky2 = (foot_step_support_frame_(current_step_number,1)+foot_step_support_frame_(current_step_number-1,1))/2.0 -  foot_step_support_frame_offset_(current_step_number-1,1);

        for(int i=0; i<t_total_; i++)
        {
          if(i < t_rest_init_)
          {
            temp_px(i) = (foot_step_support_frame_(current_step_number-1,0)+foot_step_support_frame_(current_step_number-2,0))/2.0;
            temp_py(i) = (foot_step_support_frame_(current_step_number-1,1)+foot_step_support_frame_(current_step_number-2,1))/2.0;
          }
          else if(i >= t_rest_init_ && i < t_toe)
          {
            temp_px(i) = (foot_step_support_frame_(current_step_number-1,0)+foot_step_support_frame_(current_step_number-2,0))/2.0 + Kx/(t_toe - t_rest_init_)*(i+1-t_rest_init_);
            temp_py(i) = (foot_step_support_frame_(current_step_number-1,1)+foot_step_support_frame_(current_step_number-2,1))/2.0 + Ky/(t_toe - t_rest_init_)*(i+1-t_rest_init_);
          }
          else if(i>= t_toe && i< t_ld)
          {
            temp_px(i) = (foot_step_support_frame_(current_step_number-1,0)+foot_step_support_frame_(current_step_number-2,0))/2.0 +Kx +offset_time*moving_x_offset/(t_ld-t_toe)*(i+1 - t_toe);
            temp_py(i) = foot_step_support_frame_offset_(current_step_number-1,1);
          }
          else if(i >= t_ld && i< t_total_-t_rest_last_)
          {
              temp_px(i) = foot_step_support_frame_offset_(current_step_number-1,0) + (offset_time-1)*moving_x_offset+ Kx2/(t_total_-t_rest_last_-t_ld)*(i+1-(t_ld));
              temp_py(i) = foot_step_support_frame_offset_(current_step_number-1,1) + Ky2/(t_total_-t_rest_last_-t_ld)*(i+1-(t_ld));
          }
          else
          {
            temp_px(i) = temp_px(i-1);
            temp_py(i) = temp_py(i-1);
          }
        }
    }

//    else if(current_step_number >= total_step_num_-1){
//        Kx = foot_step_support_frame_offset_(current_step_number-1,0) - (foot_step_support_frame_(current_step_number-1,0) + foot_step_support_frame_(current_step_number-2,0))/2.0 - moving_x_offset;
//        Kx2 = (foot_step_support_frame_(current_step_number,0)+foot_step_support_frame_(current_step_number-1,0))/2.0 - foot_step_support_frame_offset_(current_step_number-1,0);

//        Ky =  foot_step_support_frame_offset_(current_step_number-1,1) - (foot_step_support_frame_(current_step_number-1,1) + foot_step_support_frame_(current_step_number-2,1))/2.0;
//        Ky2 = (foot_step_support_frame_(current_step_number,1)+foot_step_support_frame_(current_step_number-1,1))/2.0 -  foot_step_support_frame_offset_(current_step_number-1,1);

//        for(int i=0; i<t_total_; i++)
//        {
//          if(i < t_rest_init_)
//          {
//            temp_px(i) = (foot_step_support_frame_(current_step_number-1,0)+foot_step_support_frame_(current_step_number-2,0))/2.0;
//            temp_py(i) = (foot_step_support_frame_(current_step_number-1,1)+foot_step_support_frame_(current_step_number-2,1))/2.0;
//          }
//          else if(i >= t_rest_init_ && i < t_toe)
//          {
//              temp_px(i) = (foot_step_support_frame_(current_step_number-1,0)+foot_step_support_frame_(current_step_number-2,0))/2.0 + Kx/(t_toe - t_rest_init_)*(i+1-t_rest_init_);
//              temp_py(i) = (foot_step_support_frame_(current_step_number-1,1)+foot_step_support_frame_(current_step_number-2,1))/2.0 + Ky/(t_toe - t_rest_init_)*(i+1-t_rest_init_);
//          }
//          else if(i>= t_toe && i< t_ld)
//          {
//            temp_px(i) = foot_step_support_frame_offset_(current_step_number-1,0) - moving_x_offset +moving_x_offset/(t_ld-t_toe)*(i+1 - t_toe);
//            temp_py(i) = foot_step_support_frame_offset_(current_step_number-1,1);
//          }
//          else if(i >= t_ld && i< t_total_-t_rest_last_)
//          {
//              temp_px(i) = foot_step_support_frame_offset_(current_step_number-1,0) + Kx2/(t_total_-t_rest_last_-t_ld)*(i+1-(t_ld));
//              temp_py(i) = foot_step_support_frame_offset_(current_step_number-1,1) + Ky2/(t_total_-t_rest_last_-t_ld)*(i+1-(t_ld));
//          }
//          else
//          {
//            temp_px(i) = temp_px(i-1);
//            temp_py(i) = temp_py(i-1);
//          }
//        }
//    }
    else
    {
      Kx = foot_step_support_frame_offset_(current_step_number-1,0) - (foot_step_support_frame_(current_step_number-1,0) + foot_step_support_frame_(current_step_number-2,0))/2.0 - moving_x_offset;
      Kx2 = (foot_step_support_frame_(current_step_number,0)+foot_step_support_frame_(current_step_number-1,0))/2.0 - foot_step_support_frame_offset_(current_step_number-1,0) - moving_x_offset;

      Ky =  foot_step_support_frame_offset_(current_step_number-1,1) - (foot_step_support_frame_(current_step_number-1,1) + foot_step_support_frame_(current_step_number-2,1))/2.0;
      Ky2 = (foot_step_support_frame_(current_step_number,1)+foot_step_support_frame_(current_step_number-1,1))/2.0 -  foot_step_support_frame_offset_(current_step_number-1,1);


      for(int i=0; i<t_total_; i++)
      {
        if(i < t_rest_init_)
        {
          temp_px(i) = (foot_step_support_frame_(current_step_number-1,0)+foot_step_support_frame_(current_step_number-2,0))/2.0;
          temp_py(i) = (foot_step_support_frame_(current_step_number-1,1)+foot_step_support_frame_(current_step_number-2,1))/2.0;
        }
         else if(i >= t_rest_init_ && i < t_toe)
        {
          temp_px(i) = (foot_step_support_frame_(current_step_number-1,0)+foot_step_support_frame_(current_step_number-2,0))/2.0 + Kx/(t_toe - t_rest_init_)*(i+1-t_rest_init_);
          temp_py(i) = (foot_step_support_frame_(current_step_number-1,1)+foot_step_support_frame_(current_step_number-2,1))/2.0 + Ky/(t_toe - t_rest_init_)*(i+1-t_rest_init_);
        }
        else if(i>= t_toe && i< t_ld)
        {
          temp_px(i) = foot_step_support_frame_offset_(current_step_number-1,0) - moving_x_offset +2*moving_x_offset/(t_ld-t_toe)*(i+1 - t_toe);
          temp_py(i) = foot_step_support_frame_offset_(current_step_number-1,1);
        }
        else if(i >= t_ld && i< t_total_-t_rest_last_)
        {
          temp_px(i) = foot_step_support_frame_offset_(current_step_number-1,0) + moving_x_offset+ Kx2/(t_total_-t_rest_last_-t_ld)*(i+1-(t_ld));
          temp_py(i) = foot_step_support_frame_offset_(current_step_number-1,1) + Ky2/(t_total_-t_rest_last_-t_ld)*(i+1-(t_ld));
        }
        else
        {
          temp_px(i) = temp_px(i-1);
          temp_py(i) = temp_py(i-1);
        }
      }
    }
}

void WalkingController::getComTrajectory_MJ()
{
  // Act CoM preview
  xs_(0) = com_support_current_(0); ys_(0) = com_support_current_(1);
//  xs_(0) = com_support_fixed_(0); ys_(0) = com_support_fixed_(1);

  modifiedPreviewControl_MJ();

  // Des CoM preview
  //xs_(0) = xd_(0); ys_(0) = yd_(0);
  // Des, Act CoM preview
  //xs_(1) = xd_(1); ys_(1) = yd_(1);
  //xs_(2) = xd_(2); ys_(2) = yd_(2);

  // CLIPM Preview
  xs_(1) = XD_(1); ys_(1) = YD_(1);

  if (walking_tick_ == t_start_ + t_total_-1 && current_step_num_ != total_step_num_-1) // 지지 발이 바뀌기 1tick 전
  { // 지지 발이 바뀌기 1tick 전에 아래의 작업을 하는 이유는 지지 발이 바뀌는 순간에도 이전 tick의 CoM desired를 써야하는데 이전 tick의 CoM desired는 이전 지지 발에서 생성한 것이기 때문.

    Eigen::Vector3d com_pos_prev;
    Eigen::Vector3d com_pos;
    Eigen::Vector3d com_ver_prev;
    Eigen::Vector3d com_vel;
    Eigen::Vector3d com_acc_prev;
    Eigen::Vector3d com_acc;
    //
    Eigen::Vector3d com_u_prev;
    Eigen::Vector3d com_u;
    //
    Eigen::Matrix3d temp_rot;
    Eigen::Vector3d temp_pos;

    temp_rot = DyrosMath::rotateWithZ(-foot_step_support_frame_(current_step_num_,5)); // 회전
    for(int i=0; i<3; i++)
      temp_pos(i) = foot_step_support_frame_(current_step_num_,i);

    com_pos_prev(0) = xs_(0);
    com_pos_prev(1) = ys_(0);
    com_pos = temp_rot*(com_pos_prev - temp_pos);

    com_ver_prev(0) = xs_(1);
    com_ver_prev(1) = ys_(1);
    com_ver_prev(2) = 0.0;
    com_vel = temp_rot*com_ver_prev;

    com_acc_prev(0) = xs_(2);
    com_acc_prev(1) = ys_(2);
    com_acc_prev(2) = 0.0;
    com_acc = temp_rot*com_acc_prev;

    com_u_prev(0) = UX_;
    com_u_prev(1) = UY_;

    com_u = temp_rot*(com_u_prev - temp_pos);

    xs_(0) = com_pos(0);
    ys_(0) = com_pos(1);
    xs_(1) = com_vel(0);
    ys_(1) = com_vel(1);
    xs_(2) = com_acc(0);
    ys_(2) = com_acc(1);

    // Act CoM preview
    //preview_x = xs_;
    //preview_y = ys_;

    // CLIPM Preview
    UX_ = com_u(0);
    UY_ = com_u(1);
  }

  double start_time;

  if(current_step_num_ == 0)
    start_time = 0;
  else
    start_time = t_start_;

  if(com_control_mode_ == true)
  {
    /*
    if(walking_tick_ <= 840)
    {
      com_desired_(0) = ref_com_(walking_tick_,0);
      com_desired_(1) = ref_com_(walking_tick_,1);
      Com_tra_graph << com_desired_(0) << "," << com_desired_(1)  << std::endl;
    }
    else if(walking_tick_ > 840 )
    {
      if(com_tick_ % 240 == 0 )//&& print_flag != 10)
      {
        com_tick_ = 0;
        print_flag ++;
        if(print_flag % (int)totar_step_num_ == 0)
        {com_tick_ = 240; print_flag = 0;}
      }
      com_desired_(0) = ref_com_(com_tick_,0);
      com_desired_(1) = ref_com_(com_tick_,1);
      Com_tra_graph << com_desired_(0) << "," << com_desired_(1)  << std::endl;
      com_tick_ ++;
    }*/
    /*
    if(walking_tick_ <= 840)
    {
      com_desired_(0) = ref_com_(walking_tick_,0);
      if(walking_tick_ >= 600)
      {
        com_desired_(0) = 0;//ref_com_(walking_tick_,0);
      }
      com_desired_(1) = ref_com_(walking_tick_,1);

      Com_tra_graph << com_desired_(0) << "," << com_desired_(1) + 0.129510  << "," << com_support_current_(0) << "," << com_support_current_(1) + 0.12960 << std::endl;
    }
    else if(walking_tick_ > 840 )
    {
      if(com_tick_ % 240 == 0)//&& print_flag != 10)
      {
        com_tick_ = 0;
        //print_flag ++;
        //if(print_flag % (int)totar_step_num_ == 0)
        //{com_tick_ = 240; print_flag = 0;}
      }
      com_desired_(0) = 0;//ref_com_(com_tick_,0);
      com_desired_(1) = ref_com_(com_tick_,1);
      if((int)current_step_num_ % 2 == 0)
      {
        Com_tra_graph << com_desired_(0) << "," << com_desired_(1) + 0.12707  << "," << com_support_current_(0) << "," << com_support_current_(1) + 0.12707 << std::endl;
      }
      else if((int)current_step_num_ % 2 == 1)
      {
        Com_tra_graph << com_desired_(0) << "," << com_desired_(1) - 0.12707  << "," << com_support_current_(0) << "," << com_support_current_(1) - 0.12707 << std::endl;
      }
      //Com_tra_graph << com_desired_(0) << "," << com_desired_(1) << "," << com_support_current_(0) << "," << com_support_current_(1) << std::endl;
      com_tick_ ++;
    }
    */
    // CLIPM Preview
    com_desired_(0) = XD_(0);
    com_desired_(1) = YD_(0)+0*y_des;
    // Des, Act Preview
    //com_desired_(0) = xd_(0);
    //com_desired_(1) = yd_(0);
    //com_desired_(2) = DyrosMath::cubic(walking_tick_, t_start_, t_start_rear_, pelv_support_init_.translation()(2), pelv_support_start_.translation()(2), 0, 0);
    com_desired_(2) = pelv_suppprt_start_.translation()(2);
    //std::cout << com_desired_(2) << std::endl;
  }
  else
  {
    com_desired_(0) = xd_(0);
    com_desired_(1) = yd_(0);
    com_desired_(2) = pelv_suppprt_start_.translation()(2);
  }
}

void WalkingController::modifiedPreviewControl_MJ()
{
  /////reference: http://www.tandfonline.com/doi/pdf/10.1080/0020718508961156?needAccess=true/////////////

  if(walking_tick_ == 0)
  {
   //previewParam_MJ(1.0/hz_, 16*hz_/10, K_act_ ,com_support_init_, Gi_, Gd_, Gx_, A_, B_, C_, D_, A_bar_, B_bar_);
   previewParam_MJ_CPM(1.0/hz_, 16*hz_/10, K_ ,com_support_init_, Gi_, Gd_, Gx_, A_, B_vector_, C_, D_, A_bar_, B_bar_);

   // CLIPM Preview
   UX_ = com_support_init_(0);
   UY_ = com_support_init_(1);
//   UX_ = com_support_fixed_(0);
//   UY_ = com_support_fixed_(1);
   xs_(0) = xi_; xs_(1) = 0; xs_(2) = 0;
   ys_(0) = yi_; ys_(1) = 0; xs_(2) = 0;
  }

  if(current_step_num_ == 0)
    zmp_start_time_ = 0.0;
  else
    zmp_start_time_ = t_start_;

  //preview_MJ(1.0/hz_, 16*hz_/10, walking_tick_-zmp_start_time_, xi_, yi_, xs_, ys_, UX_, UY_, Gi_, Gd_, Gx_, A_, B_, xd_, yd_);
  preview_MJ_CPM(1.0/hz_, 16*hz_/10, walking_tick_-zmp_start_time_, xi_, yi_, xs_, ys_, UX_, UY_, Gi_, Gd_, Gx_, A_, B_vector_, A_bar_, B_bar_, XD_, YD_, X_bar_p_, Y_bar_p_);
}

void WalkingController::previewParam_MJ_CPM(double dt, int NL, Eigen::Matrix3d& K, Eigen::Vector3d com_support_init_, Eigen::MatrixXd& Gi, Eigen::VectorXd& Gd, Eigen::MatrixXd& Gx,
  Eigen::MatrixXd& A, Eigen::VectorXd& B, Eigen::MatrixXd& C, Eigen::MatrixXd& D, Eigen::MatrixXd& A_bar, Eigen::VectorXd& B_bar)
  {
    //double Kp = 102; double Kv = 13.8;
    double Kp = 300; double Kv = 10.8;
    //double Kp = 3508; double Kv = 32.2;
    //double Kp = 50;
    //double Kv = 6;

    A.resize(2,2);
    A(0,0) = 1.0 - Kp*dt*dt*0.5;
    A(0,1) = dt - 0.5*Kv*dt*dt;
    A(1,0) = -Kp*dt;
    A(1,1) = 1.0 - Kv*dt;

    B.resize(2);
    B(0) = 0.5*Kp*dt*dt;
    B(1) = Kp*dt;

    C.resize(1,2);
    C(0,0) = 1 + zc_/GRAVITY*Kp;
    C(0,1) = zc_/GRAVITY*Kv;
    //std::cout << GRAVITY << "," << zc_/GRAVITY*Kp << std::endl;
    D.resize(1,1);
    D(0,0) = -zc_/GRAVITY*Kp;
    // A, B, C, D discrete matrix 정의

    B_bar.resize(3,1);
    B_bar(0,0) = D(0,0);
    B_bar(1,0) = B(0);
    B_bar(2,0) = B(1);

    Eigen::Matrix1x3d B_bar_tran;
    B_bar_tran = B_bar.transpose();

    Eigen::MatrixXd I_bar;
    Eigen::MatrixXd F_bar;
    A_bar.resize(3,3);
    I_bar.resize(3,1);
    F_bar.resize(3,2);
    F_bar.setZero();

    F_bar(0,0) = C(0,0);
    F_bar(0,1) = C(0,1);

    F_bar(1,0) = A(0,0);
    F_bar(1,1) = A(0,1);
    F_bar(2,0) = A(1,0);
    F_bar(2,1) = A(1,1);

    I_bar.setZero();
    I_bar(0,0) = 1.0;

    A_bar(0,0) = I_bar(0,0);
    A_bar(1,0) = I_bar(1,0);
    A_bar(2,0) = I_bar(2,0);

    A_bar(0,1) = F_bar(0,0);
    A_bar(0,2) = F_bar(0,1);
    A_bar(1,1) = F_bar(1,0);
    A_bar(1,2) = F_bar(1,1);
    A_bar(2,1) = F_bar(2,0);
    A_bar(2,2) = F_bar(2,1);

    Eigen::MatrixXd Qe;
    Qe.resize(1,1);
    Qe(0,0) = 1.0;

    Eigen::MatrixXd R;
    R.resize(1,1);
    R(0,0) = 1.0;

    Eigen::MatrixXd Qx;
    Qx.resize(3,3);
    Qx.setZero();
    /*
    Eigen::MatrixXd Q_bar;
    Q_bar.resize(3,3);
    Q_bar.setZero();
    Q_bar(0,0) = Qe(0,0);
    */
    //K = discreteRiccatiEquationLQR(A_bar, B_bar, R, Q_bar);
    // 150,20, 14000
    /*K(0,0) = 110.018390877079;
    K(0,1) = 5996.513970157503;
    K(0,2) = 1619.215576371881;
    K(1,1) = 329860.521958015510;
    K(1,2) = 89069.020403138958;
    K(2,2) = 24051.153277765603;*/

    // 400,32, 17000
//    K(0,0) = 110.011913664238;
//    K(0,1) = 5995.804617080670;
//    K(0,2) = 1619.026391243628;
//    K(1,1) = 329792.857002781064;
//    K(1,2) = 89049.060371173240;
//    K(2,2) = 24045.684616441791;

    K(0,0) = 110.921396840004;
    K(0,1) = 6095.817438477827;
    K(0,2) = 1659.891577822412;
    K(1,1) = 338075.251980493485;
    K(1,2) = 92052.157854040022;
    K(2,2) = 25065.952797316477;

    K(1, 0) = K(0, 1);
    K(2, 0) = K(0, 2);
    K(2, 1) = K(1, 2);

    Eigen::MatrixXd Temp_mat;
    Eigen::MatrixXd Temp_mat_inv;
    Eigen::MatrixXd Ac_bar;
    Temp_mat.resize(1,1);
    Temp_mat.setZero();
    Temp_mat_inv.resize(1,1);
    Temp_mat_inv.setZero();
    Ac_bar.setZero();
    Ac_bar.resize(3,3);

    Temp_mat = R + B_bar_tran * K * B_bar;
    Temp_mat_inv = Temp_mat.inverse();
    Ac_bar = A_bar - B_bar * Temp_mat_inv * B_bar_tran * K * A_bar;

    Eigen::MatrixXd Ac_bar_tran(3,3);
    Ac_bar_tran = Ac_bar.transpose();

    Gi.resize(1,1); Gx.resize(1,2);
    Gi = Temp_mat_inv * B_bar_tran * K * I_bar ;
    Gx = Temp_mat_inv * B_bar_tran * K * F_bar ;

    Eigen::MatrixXd X_bar;
    Eigen::Vector3d X_bar_col;
    X_bar.resize(3, NL);
    X_bar.setZero();
    X_bar_col.setZero();
    X_bar_col = - Ac_bar_tran * K * I_bar;

    for(int i = 0; i < NL; i++)
    {
      X_bar.block<3,1>(0,i) = X_bar_col;
      X_bar_col = Ac_bar_tran*X_bar_col;
    }

    Gd.resize(NL);
    Eigen::VectorXd Gd_col(1);
    Gd_col(0) = -Gi(0,0);

    for(int i = 0; i < NL; i++)
    {
      Gd.segment(i,1) = Gd_col;
      Gd_col = Temp_mat_inv * B_bar_tran * X_bar.col(i) ;
    }

  }

void WalkingController::preview_MJ_CPM(double dt, int NL, int tick, double x_i, double y_i, Eigen::Vector3d xs, Eigen::Vector3d ys, double& UX, double& UY,
       Eigen::MatrixXd Gi, Eigen::VectorXd Gd, Eigen::MatrixXd Gx, Eigen::MatrixXd A, Eigen::VectorXd B, Eigen::MatrixXd A_bar, Eigen::VectorXd B_bar, Eigen::Vector2d &XD, Eigen::Vector2d &YD, Eigen::VectorXd& X_bar_p, Eigen::VectorXd& Y_bar_p)
{
  int zmp_size;
  zmp_size = ref_zmp_.col(1).size(); // 보행 중 720개 (240 * 3)
  Eigen::VectorXd px_ref, py_ref;
  px_ref.resize(zmp_size);
  py_ref.resize(zmp_size);

  modified_ref_zmp_.resize(16*hz_/10, 2);

  for(int i = 0; i < zmp_size; i++)
  {
    px_ref(i) = ref_zmp_(i,0);
    py_ref(i) = ref_zmp_(i,1);
    //Static ZMP//
    /*if((int)current_step_num_ % 2 == 0)
    {
      py_ref(i) = - 0.129425;
    }
    else if((int)current_step_num_ % 2 == 1)
    {
      py_ref(i) = 0.129425;
    }*/
    //
  }

  Eigen::Matrix1x3d C;
  C(0,0) = 1; C(0,1) = 0; C(0,2) = -zc_/GRAVITY;

  Eigen::VectorXd px, py;
  px.resize(1); py.resize(1);

  X_bar_p.resize(3); Y_bar_p.resize(3);

  if(tick == 0 && current_step_num_ == 0)
  {
    Preview_X_b(0) = x_i;
    Preview_Y_b(0) = y_i;
    Preview_X(0) = x_i;
    Preview_Y(0) = y_i;
    Preview_X(1) = 0;
    Preview_Y(1) = 0;
    X_bar_p.setZero(); Y_bar_p.setZero();
  }
  else
  {
    X_bb = X_b; Y_bb = Y_b;
    X_b = com_support_current_CLIPM_b(0); Y_b = com_support_current_CLIPM_b(1);
    Preview_X(0) = xs(0); Preview_Y(0) = ys(0);
    Preview_X(1) = xs(1); Preview_Y(1) = ys(1);
  }

  Eigen::VectorXd Temp_mat_X, Temp_mat_Y;
  Temp_mat_X.resize(3); Temp_mat_Y.resize(3);
  Temp_mat_X.setZero(); Temp_mat_Y.setZero();

  Temp_mat_X(0) = Preview_X(0); Temp_mat_Y(0) = Preview_Y(0); // preview_x(0)이 x_i를 안넣고 xs를??
  Temp_mat_X(2) = X_bar_p(2)/dt; Temp_mat_Y(2) = Y_bar_p(2)/dt;  // preview_x(1)-preview_x_b(1)로 하면 언더슛생김 일반 프리뷰처럼

  px = C*Temp_mat_X;
  py = C*Temp_mat_Y;

  Temp_mat_X(0) = com_support_current_CLIPM_(0); Temp_mat_Y(0) = com_support_current_CLIPM_(1);
  double px_, py_;

  px_ = C*Temp_mat_X;
  py_ = C*Temp_mat_Y;

  //MJ_graph << px(0) << "," << py(0) << "," << px_ << "," << py_ << std::endl;
  ////////////////////////////////////////////////////////////////
  //modified_zmp_trajectory_update_MJ(16*hz_/10, walking_tick_-zmp_start_time_, px(0), py(0), modified_ref_zmp_);
  //Com_tra_graph << px(0) << "," << py(0) << "," << modified_ref_zmp_(tick, 0) << "," << modified_ref_zmp_(tick, 1) << "," << px_ref(tick) << "," << py_ref(tick) << "," << xs(0) << "," << ys(0) << std::endl;
  //Com_tra_graph << modified_ref_zmp_(tick, 0) << "," << modified_ref_zmp_(tick, 1) << std::endl;
  ////////////////////////////////////////////////////////////////
  //std::cout << ref_zmp_(tick,1) << ","  << modified_ref_zmp_(tick,1) << "," << tick << std::endl;

  X_bar_p(0) = px(0) - (px_ref(tick) + del_px_cp); //- px_ref(tick); //e(i) 정의
  Y_bar_p(0) = py(0) - (py_ref(tick) + del_py_cp);

  double sum_Gd_px_ref = 0, sum_Gd_py_ref = 0;


  for(int i = 0; i < NL; i++) // 현재 tick 기준으로 Preview Step 개수만큼 더함.
  {
    sum_Gd_px_ref = sum_Gd_px_ref + Gd(i)*(px_ref(tick + 1 + i) - px_ref(tick + i));
    sum_Gd_py_ref = sum_Gd_py_ref + Gd(i)*(py_ref(tick + 1 + i) - py_ref(tick + i));
  }

  Eigen::MatrixXd temp; temp.resize(2, 1);
  Eigen::VectorXd GxX(1); Eigen::VectorXd GxY(1);

  temp(0, 0) = X_bar_p(1); //Preview_X(0) - Preview_X_b(0);
  temp(1, 0) = X_bar_p(2); //Preview_X(1) - Preview_X_b(1);
  GxX = Gx*temp;

  temp(0, 0) = Y_bar_p(1);//Preview_Y(0) - Preview_Y_b(0);
  temp(1, 0) = Y_bar_p(2);//Preview_Y(1) - Preview_Y_b(1); // 두개 비교 해볼것
  GxY = Gx*temp;

  Eigen::MatrixXd der_ux(1,1);
  Eigen::MatrixXd der_uy(1,1);
  der_ux.setZero();
  der_uy.setZero();

  der_ux(0,0) = -(X_bar_p(0) * Gi(0,0)) - GxX(0) - sum_Gd_px_ref;
  der_uy(0,0) = -(Y_bar_p(0) * Gi(0,0)) - GxY(0) - sum_Gd_py_ref;

  X_bar_p = A_bar*X_bar_p + B_bar*der_ux;
  Y_bar_p = A_bar*Y_bar_p + B_bar*der_uy;

  UX = UX + der_ux(0,0);
  UY = UY + der_uy(0,0);

  //////////////////////CP FB ///////////////////////////////

  XD_bb = XD_b; YD_bb = YD_b;
  XD_b = XD(0); YD_b = YD(0);

  if(walking_tick_ == 0)
  {
    XD_b = 0; YD_b = 0;
  }

  XD = A*Preview_X + B*UX;
  YD = A*Preview_Y + B*UY;
  Wn = sqrt(GRAVITY/zc_);

  if(walking_tick_ == t_start_ && current_step_num_ > 0)
  {
    XD_b = XD_bb + XD(0)-XD_b;
    YD_b = YD_bb + YD(0)-YD_b;
    X_b = X_bb + com_support_current_CLIPM_(0) - X_b;
    Y_b = Y_bb + com_support_current_CLIPM_(1) - Y_b;
  }
  // ?? LPF, Des,Act
  XD_vel_b = XD_vel; YD_vel_b = YD_vel;
  XA_vel_b = XA_vel; YA_vel_b = YA_vel;

  XD_vel = (XD(0)-XD_b)/dt; YD_vel = (YD(0)-YD_b)/dt;
  XA_vel = (com_support_current_CLIPM_(0)-X_b)/dt; YA_vel = (com_support_current_CLIPM_(1)-Y_b)/dt;

  XD_vel = XD_vel_b*0.8 + XD_vel*0.2; YD_vel = YD_vel_b*0.8 + YD_vel*0.2; // 7Hz LPF
  XA_vel = XA_vel_b*0.8 + XA_vel*0.2; YA_vel = YA_vel_b*0.8 + YA_vel*0.2;


  if(walking_tick_ == 0)
  {
    XD_vel_b = 0; XD_vel = 0;
    YD_vel_b = 0; YD_vel = 0;
    XA_vel_b = 0; XA_vel = 0;
    YA_vel_b = 0; YA_vel = 0;
  }

  x_cp_ref = XD(0) + XD_vel/Wn;
  y_cp_ref = YD(0) + YD_vel/Wn;

  x_cp_act = com_support_current_CLIPM_(0) + XA_vel/Wn;
  y_cp_act = com_support_current_CLIPM_(1) + YA_vel/Wn;

  if(walking_tick_ == 0)
  {
    x_cp_ref = XD(0); y_cp_ref = YD(0);
    x_cp_act = com_support_current_CLIPM_(0); y_cp_act = com_support_current_CLIPM_(1);
  }
//  MJ_graph << x_cp_ref << "," << x_cp_act << "," << y_cp_ref << "," << y_cp_act << "," << YD_(0) << "," << YD_(0) + y_des << std::endl;
  cp_err_x = x_cp_act - x_cp_ref;
  cp_err_y = y_cp_act - y_cp_ref;

  del_px_cp = 0.2*cp_err_x;
  del_py_cp = 0.2*cp_err_y;

  //ZMP_controller_MJ(tick);

  /////////////////////////////////////////////////////////////////////
//  if((int)current_step_num_ % 2 == 0)
//  {
//    Com_tra_graph << xs(0) << "," << ys(0) + 0.127070935 << "," << px(0) << "," << py(0) + 0.129425  << "," << px_ref(tick) << "," << py_ref(tick) + 0.129425 << "," << XD(0) << "," << YD(0) + 0.127070935 << std::endl;
//  }
//  else if((int)current_step_num_ % 2 == 1)
//  {
//    Com_tra_graph << xs(0) << "," << ys(0) - 0.127070935 << "," << px(0) << "," << py(0) - 0.129425  << "," << px_ref(tick) << "," << py_ref(tick) - 0.129425 << "," << XD(0) << "," << YD(0) - 0.127070935 << std::endl;
//  }

}
void WalkingController::getRobotState_MJ()
{
  Eigen::Matrix<double, DyrosJetModel::MODEL_WITH_VIRTUAL_DOF, 1> q_temp, qdot_temp;
  q_temp.setZero();
  qdot_temp;
  q_temp.segment<28>(6) = current_q_.segment<28>(0); //segment<포함갯수>(시작점)
//  qdot_temp.segment<28>(6)= current_qdot_.segment<28>(0);

  //////////////////////////////////////////////////////////////////////////////////

  model_.updateKinematics(q_temp, qdot_temp);
  com_float_current_ = model_.getCurrentCom();
//  com_float_current_dot_= model_.getCurrentComDot();

  model_.updateKinematics(q_temp, qdot_temp);
  // Pelvis에서 왼쪽, 오른쪽 발 FK
  lfoot_float_current_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)0);
  rfoot_float_current_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)1);

  if(foot_step_(current_step_num_, 6) == 0) //right foot support
  {
    supportfoot_float_current_ = rfoot_float_current_;
  }
  else if(foot_step_(current_step_num_, 6) == 1)
  {
    supportfoot_float_current_ = lfoot_float_current_;
  }

  pelv_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_);

  com_support_current_CLIPM_b = com_support_current_CLIPM_;
  com_support_current_CLIPM_ =  DyrosMath::multiplyIsometry3dVector3d(pelv_support_current_, com_float_current_);
  com_support_current_CLIPM_(0) -= 0.07;
  //////////////////////////////////////////////////////////////////////////////////

  /*if(walking_tick_ > 0) // Using desired joint angle for kinematics update
  {
    q_temp.segment<12>(6) = desired_q_not_compensated_.segment<12>(0); //segment<포함갯수>(시작점), desired_q_not_compensated_ 는 IK에서 구해지는 Desired Joint angle.
  }*/

  // real robot X com offset
  com_float_current_(0) -= 0.07;
//  com_float_current_dot_= model_.getCurrentComDot();

  if(walking_tick_ > 0) // Using desired joint angle for kinematics update
  {
    q_temp.segment<12>(6) = desired_q_not_compensated_.segment<12>(0); //segment<포함갯수>(시작점), desired_q_not_compensated_ 는 IK에서 구해지는 Desired Joint angle.
  }
  model_.updateKinematics(q_temp, qdot_temp);
  // Pelvis에서 왼쪽, 오른쪽 발 FK
  lfoot_float_current_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)0);
  rfoot_float_current_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)1);
  r_ft_ = model_.getRightFootForce();
  r_ft_ = model_.getLeftFootForce();
  imu_acc_ = model_.getImuAccel();
  imu_ang_ = model_.getImuAngvel();
  imu_grav_rpy_ = model_.getImuGravityDirection();
  pelv_float_current_.setIdentity();

  //MJ_graph << r_ft_(2) << "," << r_ft_(2) << std::endl;

  if(imu_grav_rpy_(0)>0)
  {
    imu_grav_rpy_(0)=imu_grav_rpy_(0)-3.1415926535897;
  }
  else if(imu_grav_rpy_(0)<0)
  {
    imu_grav_rpy_(0)=imu_grav_rpy_(0)+3.1415926535897;
  }
  imu_grav_rpy_(1)=-imu_grav_rpy_(1);
  //printf("r:%f,p:%f,y:%f \n",imu_grav_rpy_(0),imu_grav_rpy_(1),imu_grav_rpy_(2));
  //MJ_graph << imu_grav_rpy_(0) << "," << imu_grav_rpy_(1) << "," << imu_grav_rpy_(2) << std::endl;

  if(foot_step_(current_step_num_, 6) == 0) //right foot support
  {
    supportfoot_float_current_ = rfoot_float_current_;
  }
  else if(foot_step_(current_step_num_, 6) == 1)
  {
    supportfoot_float_current_ = lfoot_float_current_;
  }

  pelv_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_);
  lfoot_support_current_ = DyrosMath::multiplyIsometry3d(pelv_support_current_,lfoot_float_current_);
  rfoot_support_current_ = DyrosMath::multiplyIsometry3d(pelv_support_current_,rfoot_float_current_);
  com_support_current_ =  DyrosMath::multiplyIsometry3dVector3d(pelv_support_current_, com_float_current_);
  current_leg_jacobian_r_ = model_.getLegJacobian((DyrosJetModel::EndEffector) 0);
  current_leg_jacobian_r_ = model_.getLegJacobian((DyrosJetModel::EndEffector) 1);

//  if(walking_tick_ == 0)
//  {
//    com_float_prev_ = com_float_current_;
//  }
  // slowcalc_mutex_.lock();
  thread_q_ = current_q_;
  current_motor_q_leg_ = current_q_.segment<12>(0); // 시뮬레이션에서 모터 각도와 링크 각도에 같은 값이 들어가고 있음.
  current_link_q_leg_ = current_q_.segment<12>(0);
  // slowcalc_mutex_.unlock();

}
void WalkingController::CalculateCenterOfMassSupportBody(){
    Eigen::Vector3d com_link[29];

    for(int i=0;i<29;i++){
        com_link[i] = link_transform_[i].translation() + link_transform_[i].linear()*link_local_com_position_[i];
    }

    double mass_upper,mass_leg;
    mass_upper = 0.0; mass_leg = 0.0;
    Eigen::Vector3d com_arm;
    com_arm.setZero();
    for(int i= 13;i<29;i++){
        com_arm += link_mass_[i]*com_link[i];
        mass_upper += link_mass_[i];
    }
    com_arm += link_mass_[0]*com_link[0];
    mass_upper += link_mass_[0];

    Eigen::Vector3d com_lleg, com_rleg;
    com_lleg.setZero(); com_rleg.setZero();
    for(int i=1;i<7;i++){
        com_lleg += link_mass_[i]*com_link[i];
        com_rleg += link_mass_[i+6]*com_link[i+6];

        mass_leg += link_mass_[i];
    }

    Eigen::Vector3d com_float_support_body;

    if(foot_step_(current_step_num_,6) == 0){// right foot support
        com_float_support_body = (com_arm + com_rleg)/(mass_upper + mass_leg);
    }
    else if(foot_step_(current_step_num_,6) == 1){
        com_float_support_body = (com_arm + com_lleg)/(mass_upper + mass_leg);
    }

    com_support_body_ = DyrosMath::multiplyIsometry3dVector3d(pelv_support_current_, com_float_support_body);

}
void WalkingController::SettingJointLimit(){

    q_leg_min_(0) = -3.14; q_leg_max_(0) = 3.14;
    q_leg_min_(1) = -3.14; q_leg_max_(1) = 3.14;
    q_leg_min_(2) = -40*DEG2RAD; q_leg_max_(2) = 25*DEG2RAD;
    q_leg_min_(3) =   0.0; q_leg_max_(3) = 3.14;
    q_leg_min_(4) = -45*DEG2RAD; q_leg_max_(4) = -20*DEG2RAD; // max : -25 min :-45
//    q_leg_min_(4) = -3.14; q_leg_max_(4) = 3.14;
    q_leg_min_(5) = -3.14; q_leg_max_(5) = 3.14;

    q_leg_min_(6) = -3.14; q_leg_max_(6) = 3.14;
    q_leg_min_(7) = -3.14; q_leg_max_(7) = 3.14;
    q_leg_min_(8) = -25*DEG2RAD; q_leg_max_(8) = 40*DEG2RAD;
    q_leg_min_(9) = -3.14; q_leg_max_(9) = 0.0;
    q_leg_min_(10) = 20*DEG2RAD; q_leg_max_(10) = 45*DEG2RAD;//3.14; min : 25 max 45
//    q_leg_min_(10) = -3.14; q_leg_max_(10) = 3.14;
    q_leg_min_(11) = -3.14; q_leg_max_(11) = 3.14;

//    if(walking_tick_<=t_start_ + t_double1_){
//        q_leg_max_(4) = -32*DEG2RAD;
//        q_leg_min_(10) = 32*DEG2RAD;
//    }
    // simulation height 80, stride from  20 to 45
    // q max :left :  3.14 / 3.14 / 25 / 3.14 / -20 / 3.14  // right : 3.14 / 3.14 / 40 / 0 / 45 / 3.14
    // q min :left :  -3.14 / -3.14/ / -40 / 0.0 / -45 / -3.14  // right : -3.14 / -3.14 / -25 / -3.14 / 20 / -3.14


}
void WalkingController::getToeHeelTrajectory(){
    // setting the toe and heel trajectory based on support foot frame.

    double h_ankle, foot_length_half;
    h_ankle = 0.11; foot_length_half = 0.15;

    Eigen::Vector3d ankle_to_Toe, ankle_to_Heel;
    ankle_to_Toe.setZero(); ankle_to_Heel.setZero();

    ankle_to_Toe(0) = foot_length_half; ankle_to_Toe(2) = -h_ankle;
    ankle_to_Heel(0) = -foot_length_half; ankle_to_Heel(2) = -h_ankle;

    if(walking_tick_ == t_start_ ){
        ltoe_DSP1_support_init_.translation() = lfoot_support_init_.translation() + lfoot_support_init_.linear()*ankle_to_Toe;
        rtoe_DSP1_support_init_.translation() = rfoot_support_init_.translation() + rfoot_support_init_.linear()*ankle_to_Toe;
    }
    // toe trajectory coincides toe position of initial foot
    if(foot_step_(current_step_num_,6) == 1)//left foot support
    {
        ltoe_trajectory_support_.translation() =  lfoot_support_init_.translation() + lfoot_support_init_.linear()*ankle_to_Toe;
        ltoe_trajectory_support_.linear() = lfoot_support_init_.linear();

        rtoe_trajectory_support_.translation() = rfoot_trajectory_support_.translation() + rfoot_trajectory_support_.linear()*ankle_to_Toe;
        rtoe_trajectory_support_.linear() = rfoot_trajectory_support_.linear();

//        if(current_step_num_>1){
//            rtoe_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_,t_start_real_+t_double1_,rtoe_DSP1_support_init_.translation()(2),rtoe_DSP1_support_init_.translation()(2)-0.015,0.0,0.0);
//            rtoe_trajectory_support_.translation()(0) = DyrosMath::cubic(walking_tick_,t_start_real_,t_start_real_+t_double1_,rtoe_DSP1_support_init_.translation()(0),rtoe_DSP1_support_init_.translation()(0)-0.015,0.0,0.0);
//        }
    }
    else {
        ltoe_trajectory_support_.translation() =  lfoot_trajectory_support_.translation() + lfoot_trajectory_support_.linear()*ankle_to_Toe;
        ltoe_trajectory_support_.linear() = lfoot_trajectory_support_.linear();

        rtoe_trajectory_support_.translation() = rfoot_support_init_.translation() + rfoot_support_init_.linear()*ankle_to_Toe;
        rtoe_trajectory_support_.linear() = rfoot_support_init_.linear();

//        if(current_step_num_>1){
//            ltoe_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_,t_start_real_+t_double1_,ltoe_DSP1_support_init_.translation()(2),ltoe_DSP1_support_init_.translation()(2)-0.015,0.0,0.0);
//            ltoe_trajectory_support_.translation()(0) = DyrosMath::cubic(walking_tick_,t_start_real_,t_start_real_+t_double1_,ltoe_DSP1_support_init_.translation()(0),ltoe_DSP1_support_init_.translation()(0)-0.015,0.0,0.0);
//        }
    }

//    if(walking_tick_ == t_start_){
//        cout<<"l toe init with support init : "<<lfoot_support_init_.translation() +lfoot_support_init_.linear()*ankle_to_Toe<<endl;
//        cout<<"l toe init with support trajectory : "<<lfoot_trajectory_support_.translation() + lfoot_trajectory_support_.linear()*ankle_to_Toe<<endl;
//    }

    // heel trajectory coincides heel position of foot(ankle) traejctory
    lheel_trajectory_support_.translation() = lfoot_trajectory_support_.translation() + lfoot_trajectory_support_.linear()*ankle_to_Heel;
    lheel_trajectory_support_.linear() = lfoot_trajectory_support_.linear();

    rheel_trajectory_support_.translation() = rfoot_trajectory_support_.translation() + rfoot_trajectory_support_.linear()*ankle_to_Heel;
    rheel_trajectory_support_.linear() = rfoot_trajectory_support_.linear();

    // change coordinates from support foot to pelvis
    ltoe_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_)*ltoe_trajectory_support_;
    rtoe_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_)*rtoe_trajectory_support_;
    ltoe_trajectory_euler_ = DyrosMath::rot2Euler(ltoe_trajectory_float_.linear());
    rtoe_trajectory_euler_ = DyrosMath::rot2Euler(rtoe_trajectory_float_.linear());

    lheel_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_)*lheel_trajectory_support_;
    rheel_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_)*rheel_trajectory_support_;
    lheel_trajectory_euler_ = DyrosMath::rot2Euler(lheel_trajectory_float_.linear());
    rheel_trajectory_euler_ = DyrosMath::rot2Euler(rheel_trajectory_float_.linear());

}
void WalkingController::vibrationControl_modified(const Eigen::Vector12d desired_leg_q, Eigen::Vector12d &output)
{
  if(walking_tick_ == 0) // 모터 엔코더 각도, 링크에 부착 된 외부 엔코더의 각도 초기화.
  {
    pre_motor_q_leg_ = current_motor_q_leg_;
    pre_link_q_leg_ = current_link_q_leg_;
    //lqr_output_pre_ = current_motor_q_leg_;
    DOB_IK_output_b_ = current_motor_q_leg_;
  }
  x_bar_right_.setZero(); // X_bar는 e(t)와 theta_m_dot, j_dot, j_ddot 으로 구성된 48 x 1 상태 변수.

  for (int i = 0; i < 12; i++)
  { // Simulation이라서 그런지 getRobotState 에서 link_q와 motor_q에 같은 값이 들어가고 있다.
    x_bar_right_(i, 0) = current_link_q_leg_(i) - desired_leg_q(i); // e(t) // current_link_q_leg는 현재 외부 엔코더값이 아니라, Motor angle이 들어가고 있음. desired_leg_q는 IK를 풀어서 나온 Desired 관절 각
    x_bar_right_(i+12,0) = current_motor_q_leg_(i) - pre_motor_q_leg_(i); // theta_m_dot -> theta_m 의 변화량
    x_bar_right_(i+24,0) = 0.0; // current_link_q_leg_(i) - pre_link_q_leg_(i); // theta_j_dot -> theta_j 의 변화량, 민곤이형 실수로 계속 0들어가고 있었음.
    //x_bar_right_(i+36,0)은 의도적으로 노이즈때문에 뺐음.
  }

  Eigen::Vector12d del_u_right;
  del_u_right = -kkk_copy_*x_bar_right_; // (12 x 48) X (48 x 1) -> u_c dot이 discrete니까 del_u_c
  x_bar_right_ = ad_total_copy_*x_bar_right_ + bd_total_copy_*del_u_right;
  /////////////////////////////////////////////////////////////////////////////////////////////////////LQR 기반 위치제어 입력.

  // ad,bd_total 은 LQR Dynamics, ad,bd는 모터 제어 Dynamics
  Eigen::Vector12d current_u; // left right ordel

  for (int i = 0; i < 12; i++)
  { // (theta_m(k+1) - theta_m(k)) / dt = Kp (u - theta_m(k))
    current_u(i) = (current_motor_q_leg_(i) - ad_copy_(i, i)*pre_motor_q_leg_(i)) / bd_copy_(i, i); // ad_copy, bd_copy에 들어가는 Kp를 구하는게 애매함.
    // ad_copy는 A (36x36), bd_copy는 B (36x12)를 Discretization 한 것이고, 거기서 첫번째 상태 변수인 theta_m (12개)만 뽑아서 사용한 것.
  }

  Eigen::Vector12d d_hat; //left right ordel
  // d_hat = lqr_output_pre_ - current_u ; //lqr_output_pre_ : u' , current_u : (Pn^-1(s))*theta_m(motor angles) -> u' + d
  d_hat = DOB_IK_output_b_ - current_u ; // 부호가 반대
  // d_hat 을 이런식으로 관측함.

  if(walking_tick_ == 0)
    d_hat_b_ = d_hat;

  d_hat = 0.7*d_hat_b_ + 0.3*d_hat; // 필터링

  // Mingon's LQR contorller gain (using external encodel)
  double default_gain = 0.2; // Kp가 정확하다면 시뮬레이션이나 실제 로봇이나 0.2~1의 의미는 같다.
  double compliant_gain = 1.0;
  double compliant_tick = 0.1*hz_;
  double gain_temp;
  for (int i = 0; i < 12; i ++)
  {
    if(i < 6) //왼쪽 다리 관절
    {
      //double gain_temp = default_gain;
      gain_temp = default_gain;
      if(walking_enable_ == true)
      {
        if (foot_step_(current_step_num_,6) == 0) // 오른발 지지 상태
        {
          if(walking_tick_ < t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick) // gain_temp -> 0.2
          { // t_total_: 240tick, t_rest_init_,last: 10tick (0.05초), t_double1,2_: 20tick (0.1초), compliant_tick : 20tick (0.1초)
            gain_temp = default_gain; // 1step 240tick 동안 0~190 tick까지 계속 기본 gain.
          }
          else if(walking_tick_ >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick && walking_tick_ < t_start_ + t_total_ - t_rest_last_ - t_double2_)
          {
            gain_temp = DyrosMath::cubic(walking_tick_, t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick, t_start_ + t_total_ - t_rest_last_ - t_double2_, default_gain, compliant_gain, 0.0, 0.0);
          } // 1step 240tick 동안 190~210 tick까지 기본 gain 0.2에서 1까지 올림.
          else
          {
            gain_temp = DyrosMath::cubic(walking_tick_, t_start_ + t_total_ - t_rest_last_, t_start_ + t_total_, compliant_gain, default_gain, 0.0, 0.0);
          } // 1step 240tick 동안 210~240 tick까지 기본 gain 1에서 0.2까지 낮춤.
        }
        else // 왼발 지지 상태
        {
          gain_temp = default_gain;
        }
      }
      //lqr_output_(i) = lqr_output_pre_(i) + del_u_right(i, 0) - gain_temp*d_hat(i); // u(tk) = uc(tk - del t) + del_u_c(discrete) + d_hat
      DOB_IK_output_(i) = desired_leg_q(i) - gain_temp*d_hat(i);  // LQR 위치 대신 단순 IK 기반 위치 ( u_c + d_hat = u' (논문))
      //DOB_IK_output_(4) = desired_leg_q(4) - 2.0*gain_temp*d_hat(4);
      //DOB_IK_output_(5) = desired_leg_q(5) - 2.0*gain_temp*d_hat(5);
    }
    else //오른 다리 관절
    {
      //double gain_temp = default_gain;
      gain_temp = default_gain;

      if(walking_enable_ == true)
      {
        if (foot_step_(current_step_num_,6) == 1) // 왼발 지지 상태
        {
          if(walking_tick_ < t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick) // gain_temp -> 0.2
          {
            gain_temp = default_gain;
          }
          else if(walking_tick_ >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick && walking_tick_ < t_start_ + t_total_ - t_rest_last_ - t_double2_)
          {
            gain_temp = DyrosMath::cubic(walking_tick_, t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick, t_start_ + t_total_ - t_rest_last_ - t_double2_, default_gain, compliant_gain, 0.0, 0.0);
          }
          else
          {
            gain_temp = DyrosMath::cubic(walking_tick_, t_start_ + t_total_ - t_rest_last_, t_start_ + t_total_, compliant_gain, default_gain, 0.0, 0.0);
          }
        }
        else // 오른발 지지 상태
        {
          gain_temp = default_gain;
        }
      }
      //lqr_output_(i) = lqr_output_pre_(i) + del_u_right(i, 0) - gain_temp*d_hat(i);
      DOB_IK_output_(i) = desired_leg_q(i) - gain_temp*d_hat(i);  // LQR 위치 대신 단순 IK 기반 위치
      //DOB_IK_output_(10) = desired_leg_q(10) - 2.0*gain_temp*d_hat(10);
      //DOB_IK_output_(11) = desired_leg_q(11) - 2.0*gain_temp*d_hat(11);
    }
  }

  lqr_output_pre_ = lqr_output_; // 시뮬에선 안씀.
  pre_motor_q_leg_ = current_motor_q_leg_;
  pre_link_q_leg_ = current_link_q_leg_;
  d_hat_b_ = d_hat;
  DOB_IK_output_b_ = DOB_IK_output_;
//  LQR_q_tra_graph << desired_q_(0) << "," << desired_q_(1) << "," << desired_q_(2) << "," << desired_q_(3) << "," << desired_q_(4) << "," << desired_q_(5) << "," << d_hat(0) << "," << d_hat(1) << "," << d_hat(2) << "," << d_hat(3) << "," << d_hat(4) << "," << d_hat(5) << std::endl;
  for (int i=0; i<12; i++)
  {
    output(i) = DOB_IK_output_(i);
  }
}
void WalkingController::Compliant_control(Eigen::Vector12d desired_leg_q)
{
  if(walking_tick_ == 0)
  {
    pre_motor_q_leg_ = current_motor_q_leg_;
    DOB_IK_output_b_ = current_motor_q_leg_;
  }

  Eigen::Vector12d current_u; // left right order
  double del_t = 0, Kp = 0;
  del_t = 1/hz_; Kp = 30;

  for (int i = 0; i < 12; i++)
  { // (theta_m(k+1) - theta_m(k)) / dt = Kp (u - theta_m(k))
    current_u(i) = (current_motor_q_leg_(i) - (1 - Kp*del_t)*pre_motor_q_leg_(i)) / (Kp*del_t);
  }

  Eigen::Vector12d d_hat;
  d_hat = current_u - DOB_IK_output_b_ ; //current_u -> u' + d , DOB_IK_output_b_ -> u'= IK output + d hat

  if(walking_tick_ == 0)
    d_hat_b_ = d_hat;

  d_hat = 0.7*d_hat_b_ + 0.3*d_hat; // 필터링

  // Mingon's LQR contorller gain (using external encoder)
  double default_gain = 0.0; // Kp가 정확하다면 시뮬레이션이나 실제 로봇이나 0.2~1의 의미는 같다.
  double compliant_gain = 1.0;
  double compliant_tick = 0.1*hz_;
  double gain_temp;
  for (int i = 0; i < 12; i ++)
  {
    if(i < 6) //왼쪽 다리 관절
    {
      gain_temp = default_gain;
      if(walking_enable_ == true)
      {
        if (foot_step_(current_step_num_,6) == 0) // 오른발 지지 상태
        {
          if(walking_tick_ < t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick) // gain_temp -> 0.2
          { // t_total_: 240tick, t_rest_init_,last: 10tick (0.05초), t_double1,2_: 20tick (0.1초), compliant_tick : 20tick (0.1초)
            gain_temp = default_gain; // 1step 240tick 동안 0~190 tick까지 계속 기본 gain.
          }
          else if(walking_tick_ >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick && walking_tick_ < t_start_ + t_total_ - t_rest_last_ - t_double2_)
          {
            gain_temp = DyrosMath::cubic(walking_tick_, t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick, t_start_ + t_total_ - t_rest_last_ - t_double2_, default_gain, compliant_gain, 0.0, 0.0);
          } // 1step 240tick 동안 190~210 tick까지 기본 gain 0.2에서 1까지 올림.
          else
          {
            gain_temp = DyrosMath::cubic(walking_tick_, t_start_ + t_total_ - t_rest_last_, t_start_ + t_total_, compliant_gain, default_gain, 0.0, 0.0);
          } // 1step 240tick 동안 210~240 tick까지 기본 gain 1에서 0.2까지 낮춤.
        }
        else // 왼발 지지 상태
        {
          gain_temp = default_gain;
        }
      }
      DOB_IK_output_(i) = desired_leg_q(i) + gain_temp*d_hat(i);  // LQR 위치 대신 단순 IK 기반 위치 ( u_c + d_hat = u' (논문))
    }
    else //오른 다리 관절
    {
      //double gain_temp = default_gain;
      gain_temp = default_gain;

      if(walking_enable_ == true)
      {
        if (foot_step_(current_step_num_,6) == 1) // 왼발 지지 상태
        {
          if(walking_tick_ < t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick) // gain_temp -> 0.2
          {
            gain_temp = default_gain;
          }
          else if(walking_tick_ >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick && walking_tick_ < t_start_ + t_total_ - t_rest_last_ - t_double2_)
          {
            gain_temp = DyrosMath::cubic(walking_tick_, t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick, t_start_ + t_total_ - t_rest_last_ - t_double2_, default_gain, compliant_gain, 0.0, 0.0);
          }
          else
          {
            gain_temp = DyrosMath::cubic(walking_tick_, t_start_ + t_total_ - t_rest_last_, t_start_ + t_total_, compliant_gain, default_gain, 0.0, 0.0);
          }
        }
        else // 오른발 지지 상태
        {
          gain_temp = default_gain;
        }
      }
      //lqr_output_(i) = lqr_output_pre_(i) + del_u_right(i, 0) - gain_temp*d_hat(i);
      DOB_IK_output_(i) = desired_leg_q(i) + gain_temp*d_hat(i);  // LQR 위치 대신 단순 IK 기반 위치
    }
  }
  pre_motor_q_leg_ = current_motor_q_leg_;
  d_hat_b_ = d_hat;
//  cout << d_hat*180/M_PI << endl;
  DOB_IK_output_b_ = DOB_IK_output_;
}
}



