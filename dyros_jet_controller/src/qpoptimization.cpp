#include "dyros_jet_controller/dyros_jet_model.h"
#include "dyros_jet_controller/walking_controller.h"
#include "cvxgen_6_8_0/cvxgen/solver.h"
#include <chrono>
#include <stdio.h>


namespace dyros_jet_controller
{
void WalkingController::qpIK(){

    if(walking_tick_ == 0)
        cout<<"optimization IK  "<<endl;
    // using qpoases
    Eigen::Matrix<double, 12, 12> jacobian_A;
    jacobian_A.setZero();
    jacobian_A.block<6,6>(0,0) = current_leg_jacobian_l_;
    jacobian_A.block<6,6>(6,6) = current_leg_jacobian_r_;

    Eigen::VectorXd y_input(12);
    Eigen::Matrix6d Kp;
    Kp.setIdentity();
    Kp *= 200.0;

    y_input.segment(0,6) = lp_;
    y_input.segment(6,6) = rp_;

//    y_input(5) *= 0.5;
//    y_input(11) *= 0.5;

//    for(int i=0;i<10;i++){
//    //    params.Y[i] = rfoot_trajectory_float_.translation()(i) ;
//          params.Y[i] = y_input(i) ;
//    }

    //    for(int i=0;i<12;i++){
    //        for(int j=0;j<12;j++){
    //            params.Q[i+ 12*j] = iden(i,j);
    //        }
    //    }

    Eigen::Vector12d current_q_dot;
    if(walking_tick_ == 0){
        current_q_dot.setZero();
    }
    else{
        current_q_dot = pre_q_dot_.segment<12>(0);
    }

    Eigen::Matrix12d iden;
    iden.setIdentity();


    real_t H[12*12],A[12*12],lbA[12],ubA[12],lb[12],ub[12], g[12];

    for(int i=0;i<12;i++){
        for(int j=0;j<12;j++){
            H[12*i+j] = iden(i,j);
            A[12*i+j] = jacobian_A(i,j);
        }
    }
    for(int i=0;i<12;i++){
        lbA[i] = y_input(i)-0.001;
        ubA[i] = y_input(i)+0.001;
        lb[i] =-10;
        ub[i] = 10;
        g[i] = -current_q_dot(i);
    }


    real_t xOpt[12];
    real_t yOpt[12+12];
//    QProblem example(12,12, HST_IDENTITY);
    QProblem example(12,12);

    Options options;
    options.initialStatusBounds =ST_LOWER;
    options.numRefinementSteps = 1;
    options.enableCholeskyRefactorisation = 1;
//    options.printLevel = PL_NONE;
    options.enableEqualities = BT_TRUE;

    example.setOptions(options);

    int_t nWSR = 10000000;

    example.init(H,g,A,lb,ub,lbA,ubA,nWSR);
    example.getPrimalSolution(xOpt);
    example.getDualSolution(yOpt);

    example.hotstart(g,lb,ub,lbA,ubA,nWSR,0);
    example.getPrimalSolution(xOpt);


    Eigen::Vector12d qp_q;
    for(int i=0;i<6;i++){
        qp_q(i) = xOpt[i]/hz_ + desired_q_not_compensated_(LF_BEGIN + i);
        qp_q(i+6) = xOpt[i+6]/hz_ + desired_q_not_compensated_(RF_BEGIN +i);

    }


//    if(walking_tick_ == 0){
////        cout<<"jacobian matrix : "<<endl<<jacobian_A<<endl;
//        //        cout<<"left jacobian " <<endl<<current_leg_jacobian_l_<<endl;
////        cout<<"right jacobian "<<endl<<current_leg_jacobian_r_<<endl;
//        cout<<"identity matrix "<<endl<<iden<<endl;
//    }
    Eigen::Vector6d constraint_test_l, constraint_test_r, q_sol_l, q_sol_r;
    for(int i=0;i<6;i++){
        q_sol_l(i) = xOpt[i];
        q_sol_r(i) = xOpt[i+6];
    }
    Eigen::Vector12d q_sol, constraint_test;

    for(int i=0;i<12;i++){
        q_sol(i)= xOpt[i];
    }
    constraint_test = jacobian_A*q_sol;

//    constraint_test_l = current_leg_jacobian_l_*q_sol_l;
//    constraint_test_r = current_leg_jacobian_r_*q_sol_r;
    constraint_test_l = jacobian_A.block<6,6>(0,0)*q_sol_l;
    constraint_test_r = jacobian_A.block<6,6>(6,6)*q_sol_r;

    Eigen::Vector12d inverse_test, q_temp;

    q_temp = jacobian_A.inverse()*y_input;

    inverse_test = jacobian_A*q_temp;

   file[6]<<walking_tick_<<"\t"<<y_input(0)<<"\t"<<y_input(1)<<"\t"<<y_input(2)<<"\t"<<y_input(3)<<"\t"<<y_input(4)<<"\t"<<y_input(5)
            <<"\t"<<y_input(6)<<"\t"<<y_input(7)<<"\t"<<y_input(8)<<"\t"<<y_input(9)<<"\t"<<y_input(10)<<"\t"<<y_input(11)
           <<"\t"<<constraint_test(0)<<"\t"<<constraint_test(1)<<"\t"<<constraint_test(2)<<"\t"<<constraint_test(3)<<"\t"<<constraint_test(4)<<"\t"<<constraint_test(5)
                       <<"\t"<<constraint_test(6)<<"\t"<<constraint_test(7)<<"\t"<<constraint_test(8)<<"\t"<<constraint_test(9)<<"\t"<<constraint_test(10)<<"\t"<<constraint_test(11)
         <<"\t"<<constraint_test_l(0)<<"\t"<<constraint_test_l(1)<<"\t"<<constraint_test_l(2)<<"\t"<<constraint_test_l(3)<<"\t"<<constraint_test_l(4)<<"\t"<<constraint_test_l(5)
        <<"\t"<<constraint_test_r(0)<<"\t"<<constraint_test_r(1)<<"\t"<<constraint_test_r(2)<<"\t"<<constraint_test_r(3)<<"\t"<<constraint_test_r(4)<<"\t"<<constraint_test_r(5)
       <<"\t"<<lp_(0)<<"\t"<<lp_(1)<<"\t"<<lp_(2)<<"\t"<<lp_(3)<<"\t"<<lp_(4)<<"\t"<<lp_(5)
      <<"\t"<<rp_(0)<<"\t"<<rp_(1)<<"\t"<<rp_(2)<<"\t"<<rp_(3)<<"\t"<<rp_(4)<<"\t"<<rp_(5)
     <<"\t"<<inverse_test(0)<<"\t"<<inverse_test(1)<<"\t"<<inverse_test(2)<<"\t"<<inverse_test(3)<<"\t"<<inverse_test(4)<<"\t"<<inverse_test(5)
       <<"\t"<<inverse_test(6)<<"\t"<<inverse_test(7)<<"\t"<<inverse_test(8)<<"\t"<<inverse_test(9)<<"\t"<<inverse_test(10)<<"\t"<<inverse_test(11)
     <<endl;

   file[7]<<walking_tick_
         <<"\t"<<xOpt[0]<<"\t"<<xOpt[1]<<"\t"<<xOpt[2]<<"\t"<<xOpt[3]<<"\t"<<xOpt[4]<<"\t"<<xOpt[5]
           <<"\t"<<xOpt[6]<<"\t"<<xOpt[7]<<"\t"<<xOpt[8]<<"\t"<<xOpt[9]<<"\t"<<xOpt[10]<<"\t"<<xOpt[11]
          <<"\t"<<yOpt[0]<<"\t"<<yOpt[1]<<"\t"<<yOpt[2]<<"\t"<<yOpt[3]<<"\t"<<yOpt[4]<<"\t"<<yOpt[5]
            <<"\t"<<yOpt[6]<<"\t"<<yOpt[7]<<"\t"<<yOpt[8]<<"\t"<<yOpt[9]<<"\t"<<yOpt[10]<<"\t"<<yOpt[11]
           <<"\t"<<yOpt[12]<<"\t"<<yOpt[13]<<"\t"<<yOpt[14]<<"\t"<<yOpt[15]<<"\t"<<yOpt[16]<<"\t"<<yOpt[17]
             <<"\t"<<yOpt[18]<<"\t"<<yOpt[19]<<"\t"<<yOpt[20]<<"\t"<<yOpt[21]<<"\t"<<yOpt[22]<<"\t"<<yOpt[23]
         <<"\t"<<qp_q[0]*RAD2DEG<<"\t"<<qp_q[1]*RAD2DEG<<"\t"<<qp_q[2]*RAD2DEG<<"\t"<<qp_q[3]*RAD2DEG<<"\t"<<qp_q[4]*RAD2DEG<<"\t"<<qp_q[5]*RAD2DEG
            <<"\t"<<qp_q[6]*RAD2DEG<<"\t"<<qp_q[7]*RAD2DEG<<"\t"<<qp_q[8]*RAD2DEG<<"\t"<<qp_q[9]*RAD2DEG<<"\t"<<qp_q[10]*RAD2DEG<<"\t"<<qp_q[11]*RAD2DEG<<endl;


   for(int i=0;i<12;i++){
       desired_q_(i) = qp_q(i);
//       desired_q_(i) = desired_q_not_compensated_(i) + inverse_test(i)/hz_;
   }
//   desired_q_.segment<12>(0) = qp_q;

   for(int i=0;i<12;i++){
       pre_q_dot_(i) = xOpt[i];
   }

}
void WalkingController::qpIK_test(){
    Eigen::MatrixXd Q_input;
    Q_input.resize(12,12);
    Q_input.setZero();

    Eigen::Matrix6d Iden_6;
    Iden_6.setIdentity();
    if(walking_tick_ == 0){
        cout<<endl<<" !!!!   Heel toe optimization  !!!!"<<endl<<endl;
    }
    Eigen::Matrix6d Q_jac_l, Q_jac_r;
    Q_jac_l = current_leg_jacobian_l_.transpose()*current_leg_jacobian_l_;
    Q_jac_r = current_leg_jacobian_r_.transpose()*current_leg_jacobian_r_;
//    Q_jac_l = current_left_toe_jacobian_.transpose()*current_left_toe_jacobian_;
//    Q_jac_r = current_right_toe_jacobian_.transpose()*current_right_toe_jacobian_;

    Q_input.block<6,6>(0,0) = Q_jac_l + 0.01*Iden_6;
    Q_input.block<6,6>(6,6) = Q_jac_r + 0.01*Iden_6;

//    Q_input.setIdentity();


//    lp_ = lp;
//    rp_ = rp;

    Eigen::Vector12d g_input;
    g_input.setZero();


    Eigen::Matrix6d Kp;
    Kp.setIdentity();
    Kp *= 200;

    Eigen::Vector6d g_r,g_l;

    Eigen::Matrix<double, 23,12> A_input_dsp1, A_input_landing;
    A_input_dsp1.setZero(); A_input_landing.setZero();

    Eigen::Matrix<double, 18, 12> A_input_lifting;
    A_input_lifting.setZero();

    Eigen::Matrix<double, 24,12> A_input_dsp2;
    A_input_dsp2.setZero();

    Eigen::VectorXd lbA_input_dsp1,ubA_input_dsp1, lbA_input_lifting, ubA_input_lifting, lbA_input_landing, ubA_input_landing, lbA_input_dsp2, ubA_input_dsp2;
    lbA_input_dsp1.resize(23); ubA_input_dsp1.resize(23);
    lbA_input_lifting.resize(18); ubA_input_lifting.resize(18);
    lbA_input_landing.resize(23); ubA_input_landing.resize(23);
    lbA_input_dsp2.resize(24); ubA_input_dsp2.resize(24);

//    getDesiredVelocity(lp_,rp_,ltoe_p_,rtoe_p_,lheel_p_,rheel_p_,lp_clik_,rp_clik_,ltoe_clik_, rtoe_clik_,lheel_clik_,rheel_clik_);

    GetConstraintMatrix(A_input_dsp1,A_input_lifting,A_input_landing,A_input_dsp2,
                        lbA_input_dsp1,ubA_input_dsp1,lbA_input_lifting,ubA_input_lifting,lbA_input_landing,ubA_input_landing,lbA_input_dsp2,ubA_input_dsp2);


//    g_l = current_leg_jacobian_l_.transpose()*(lp_ + 5*lp_clik_);
//    g_r = current_leg_jacobian_r_.transpose()*(rp_ + 5*rp_clik_);
    if(walking_tick_ == 0)
        pre_q_sol_.setZero();

    g_l = current_leg_jacobian_l_.transpose()*lp_ + 0.01*pre_q_sol_.segment<6>(0);
    g_r = current_leg_jacobian_r_.transpose()*rp_ + 0.01*pre_q_sol_.segment<6>(6);

//    g_l = current_left_toe_jacobian_.transpose()*ltoe_p_;
//    g_r = current_right_toe_jacobian_.transpose()*rtoe_p_;




//    g_l = Kp*lp_.transpose()*current_leg_jacobian_l_;
//    g_r = Kp*rp_.transpose()*current_leg_jacobian_r_;

    g_input.segment<6>(0) = -g_l;
    g_input.segment<6>(6) = -g_r;


    // consider constraint of toe trajectory
    Eigen::Matrix<double, 10, 12>Toe_Jaco_mat;
    Toe_Jaco_mat.setZero();
    Toe_Jaco_mat.block<4,6>(0,0) = current_left_toe_jacobian_.block<4,6>(0,0);
    Toe_Jaco_mat.block<1,6>(4,0) = current_left_toe_jacobian_.block<1,6>(5,0);
    Toe_Jaco_mat.block<4,6>(5,6) = current_right_toe_jacobian_.block<4,6>(0,0);
    Toe_Jaco_mat.block<1,6>(9,6) = current_right_toe_jacobian_.block<1,6>(5,0);
//    Toe_Jaco_mat.block<6,6>(0,0) = current_left_toe_jacobian_;
//    Toe_Jaco_mat.block<6,6>(6,6) = current_right_toe_jacobian_;



    Eigen::Matrix<double, 12,12> A_2;
    A_2.setIdentity();

/////// for test constraint matrix ////////////
/*

    QProblem test11(12,24);
    Options opt;

    opt.initialStatusBounds = ST_INACTIVE;
    opt.numRefinementSteps = 1;
    opt.enableCholeskyRefactorisation = 1;
    opt.printLevel = PL_NONE;

    int_t nWSR = 1000;

    real_t qOpt[12];
    test11.setOptions(opt);

    test11.init(H,g,A,lb,ub,lbA,ubA,nWSR,0);
    test11.getPrimalSolution(qOpt);
    test11.hotstart(g,lb,ub,lbA,ubA,nWSR,0);
    test11.getPrimalSolution(qOpt);

*/

    Eigen::Matrix<double, 24,12> A_input2;
    A_input2.setZero();
    A_input2.block<12,12>(0,0) = Q_input;
    A_input2.block<12,12>(12,0) = A_2;

    double left_leg_length, right_leg_length;

    left_leg_length = sqrt(pow(lfoot_trajectory_float_.translation()(0),2) +pow(lfoot_trajectory_float_.translation()(1),2) +pow(lfoot_trajectory_float_.translation()(2),2));
    right_leg_length = sqrt(pow(rfoot_trajectory_float_.translation()(0),2) +pow(rfoot_trajectory_float_.translation()(1),2) +pow(rfoot_trajectory_float_.translation()(2),2));

    double left_toe_length,right_toe_length;
    left_toe_length = sqrt(pow(ltoe_trajectory_float_.translation()(0),2) +pow(ltoe_trajectory_float_.translation()(1),2) +pow(ltoe_trajectory_float_.translation()(2),2));
    right_toe_length = sqrt(pow(rtoe_trajectory_float_.translation()(0),2) +pow(rtoe_trajectory_float_.translation()(1),2) +pow(rtoe_trajectory_float_.translation()(2),2));

//    file[28]<<walking_tick_<<"\t"<<left_leg_length<<"\t"<<right_leg_length<<"\t"<<left_toe_length<<"\t"<<right_toe_length<<endl;

//    real_t H[12*12], g[12],lb[12],ub[12], A[23*12], lbA[23],ubA[23] ;
    real_t H[12*12], g[12],  lb[12],ub[12],
            A_dsp1[23*12], A_lifting[18*12], A_landing[23*12], A_dsp2[24*12],
           lbA_dsp1[23],lbA_lifting[18],lbA_landing[23],lbA_dsp2[24],
           ubA_dsp1[23],ubA_lifting[18],ubA_landing[23],ubA_dsp2[24];

    for(int i=0;i<12;i++){
        for(int j=0;j<12;j++){
            H[12*i+j] = Q_input(i,j);           
        }
        g[i] = g_input(i);
        lb[i] = -10000;
        ub[i] = 10000;

//        lbA[i+11] = (q_leg_min_(i) - current_q_(i))*hz_;
//        ubA[i+11] = (q_leg_max_(i) - current_q_(i))*hz_;

//        lbA2[i+12] = (q_leg_min_(i) - current_q_(i))*hz_;
//        ubA2[i+12] = (q_leg_max_(i) - current_q_(i))*hz_;
    }

    for(int j=0;j<12;j++){
        for(int i=0;i<23;i++){
            A_dsp1[12*i + j] = A_input_dsp1(i,j);
            A_landing[12*i + j] = A_input_landing(i,j);
        }
        for(int i=0;i<18;i++){
            A_lifting[12*i + j] = A_input_lifting(i,j);
        }
        for(int i=0;i<24;i++){
            A_dsp2[12*i + j] = A_input_dsp2(i,j);
        }
    }

    for(int i=0;i<23;i++){
        lbA_dsp1[i] = lbA_input_dsp1(i);
        ubA_dsp1[i] = ubA_input_dsp1(i);

        lbA_landing[i] = lbA_input_landing(i);
        ubA_landing[i] = ubA_input_landing(i);
    }
    for(int i =0;i<18;i++){
        lbA_lifting[i] = lbA_input_lifting(i);
        ubA_lifting[i] = ubA_input_lifting(i);
    }

    for(int i=0;i<24;i++){
        lbA_dsp2[i] = lbA_input_dsp2(i);
        ubA_dsp2[i] = ubA_input_dsp2(i);
    }

//    }
//    for(int i=0;i<23;i++){
//        for(int j=0;j<12;j++){
//            A[12*i+j] = A_input(i,j);
//        }
////        lbA[i] = lbA_input(i);
////        ubA[i] = ubA_input(i);
//    }

//    for(int i=0;i<24;i++){
//        for(int j=0;j<12;j++){
//            A2[12*i+j] = A_input2(i,j);
//        }
//    }
//    for(int i=0;i<6;i++){
//        lbA2[i] = lp_(i) + 5*lp_clik_(i);
//        lbA2[i+6] = rp_(i)+5*rp_clik_(i);

//        ubA2[i] = lp_(i)+5*lp_clik_(i);
//        ubA2[i+6] = rp_(i) +5*rp_clik_(i);
//    }

//      cout<<"Hello test 2" <<endl;

    Options options;
    options.initialStatusBounds = ST_INACTIVE;
    options.numRefinementSteps = 1;
    options.enableCholeskyRefactorisation = 1;
    options.printLevel = PL_NONE;

    int_t nWSR = 1000;

    real_t qOpt[12];


//    if(walking_tick_ == t_start_ + t_double1_)
//        cout<<"start of litfing ,  "<<walking_tick_<<"  Left foot float angle : "<<lfoot_float_current_euler_(1)*RAD2DEG<<"\t"<<"Right foot float angle : "<<rfoot_float_current_euler_(1)*RAD2DEG<<endl;
//    if(walking_tick_ == t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0)
//        cout<<"start of landing ,  "<<walking_tick_<<"  Left foot float angle : "<<lfoot_float_current_euler_(1)*RAD2DEG<<"\t"<<"Right foot float angle : "<<rfoot_float_current_euler_(1)*RAD2DEG<<endl;
//    if(walking_tick_ == t_start_+t_total_-t_double2_-t_rest_last_)
//        cout<<"start of DSP ,  "<<walking_tick_<<"  Left foot float angle : "<<lfoot_float_current_euler_(1)*RAD2DEG<<"\t"<<"Right foot float angle : "<<rfoot_float_current_euler_(1)*RAD2DEG<<endl;

        if(walking_tick_< t_start_ + t_double1_){

            QProblem dsp1(12,23);
            dsp1.setOptions(options);

            dsp1.init(H,g,A_dsp1,lb,ub,lbA_dsp1,ubA_dsp1,nWSR);
            dsp1.getPrimalSolution(qOpt);
            dsp1.hotstart(g,lb,ub,lbA_dsp1,ubA_dsp1,nWSR);
            dsp1.getPrimalSolution(qOpt);
        }
        else if(walking_tick_ >= t_start_ + t_double1_ && walking_tick_ < t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0)// SSP (half of swing, toe off and lifting up the foot)
        {
            QProblem lifting(12,18);
            lifting.setOptions(options);

            lifting.init(H,g,A_lifting,lb,ub,lbA_lifting,ubA_lifting,nWSR);
            lifting.getPrimalSolution(qOpt);
            lifting.hotstart(g,lb,ub,lbA_lifting,ubA_lifting,nWSR);
            lifting.getPrimalSolution(qOpt);
        }
        else if(walking_tick_ >= t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0 && walking_tick_ < t_start_+t_total_-t_double2_-t_rest_last_)// SSP + DSP ( landing the foot ) and contact whole foot
        {
            QProblem landing(12,23);
            landing.setOptions(options);

            landing.init(H,g,A_landing,lb,ub,lbA_landing,ubA_landing,nWSR);
            landing.getPrimalSolution(qOpt);
            landing.hotstart(g,lb,ub,lbA_landing,ubA_landing,nWSR);
            landing.getPrimalSolution(qOpt);
        }
        else{
            QProblem dsp2(12,24);
            dsp2.setOptions(options);

            dsp2.init(H,g,A_dsp2,lb,ub,lbA_dsp2,ubA_dsp2,nWSR);
            dsp2.getPrimalSolution(qOpt);
            dsp2.hotstart(g,lb,ub,lbA_dsp2,ubA_dsp2,nWSR);
            dsp2.getPrimalSolution(qOpt);
        }

//    if(walking_tick_ >= t_start_ + t_double1_ +20 && walking_tick_ < t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0)// SSP (half of swing, toe off and lifting up the foot)
//    {
//         QProblem test(12,12);
//         test.setOptions(options);

//         test.init(H,g,A1,lb,ub,lbA1,ubA1,nWSR,0);
//         test.getPrimalSolution(qOpt);
//         test.hotstart(g,lb,ub,lbA1,ubA1,nWSR,0);
//         test.getPrimalSolution(qOpt);
//    }
//    else if(walking_tick_ >=t_start_ +t_total_ -t_double2_-t_rest_last_){
//        QProblem test1(12,24);
//        test1.setOptions(options);

//        test1.init(H,g,A2,lb,ub,lbA2,ubA2,nWSR);
//        test1.getPrimalSolution(qOpt);
//        test1.hotstart(g,lb,ub,lbA2,ubA2,nWSR,0);
//        test1.getPrimalSolution(qOpt);
//    }
//    else{
//    if(walking_tick_ > t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0)
//    {
//        QProblem test1(12,23);

//        test1.setOptions(options);

//        test1.init(H,g,A,lb,ub,lbA,ubA,nWSR);
//        test1.getPrimalSolution(qOpt);
//        test1.hotstart(g,lb,ub,lbA,ubA,nWSR,0);
//        test1.getPrimalSolution(qOpt);
//    }


    Eigen::Vector6d q_l_sol, q_r_sol;
    for(int i=0;i<6;i++){
        q_l_sol(i) = qOpt[i];
        q_r_sol(i) = qOpt[i+6];

        pre_q_sol_(i) = qOpt[i];
        pre_q_sol_(i+6) = qOpt[i+6];
    }
    ///// constraint check ////////////
    /// \brief check_l_const
    ///

    double state_l=0, state_r =0;
    Eigen::Vector6d check_l_const,check_r_const;

//    check_l_const = current_left_toe_jacobian_*q_l_sol;
//    check_r_const = current_right_toe_jacobian_*q_r_sol;

      Eigen::Vector12d lower_limit, upper_limit;
      lower_limit.setZero(); upper_limit.setZero();
    if(walking_tick_< t_start_ + t_double1_){

        if(foot_step_(current_step_num_,6) == 1){//left foot support
            lower_limit.segment<6>(0) = lbA_input_dsp1.segment<6>(0);
            lower_limit.segment<4>(6) = lbA_input_dsp1.segment<4>(6);
            lower_limit(11) = lbA_input_dsp1(10);

            upper_limit.segment<6>(0) = ubA_input_dsp1.segment<6>(0);
            upper_limit.segment<4>(6) = ubA_input_dsp1.segment<4>(6);
            upper_limit(11) = ubA_input_dsp1(10);

            check_l_const = A_input_dsp1.block<6,6>(0,0)*q_l_sol;
            check_r_const.segment<4>(0) = A_input_dsp1.block<4,6>(6,6)*q_r_sol;
            check_r_const(5) = A_input_dsp1.block<1,6>(10,6)*q_r_sol;

            state_l =0.1;
            state_r =0.1;
        }
        else{
            lower_limit.segment<4>(0) = lbA_input_dsp1.segment<4>(0);
            lower_limit(5) = lbA_input_dsp1(4);
            lower_limit.segment<6>(6) = lbA_input_dsp1.segment<6>(5);

            upper_limit.segment<4>(0) = ubA_input_dsp1.segment<4>(0);
            upper_limit(5) = ubA_input_dsp1(4);
            upper_limit.segment<6>(6) = ubA_input_dsp1.segment<6>(5);


            check_l_const.segment<4>(0) = A_input_dsp1.block<4,6>(0,0)*q_l_sol;
            check_l_const(5) = A_input_dsp1.block<1,6>(4,0) * q_l_sol;

            check_r_const = A_input_dsp1.block<6,6>(5,6) *q_r_sol;

            state_l =0.1;
            state_r =0.1;

        }

    }
    else if(walking_tick_ >= t_start_ + t_double1_ && walking_tick_ < t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0)// SSP (half of swing, toe off and lifting up the foot)
    {
        if(foot_step_(current_step_num_,6) == 1){// left support
            lower_limit.segment<6>(0) = lbA_input_lifting.segment<6>(0);
            upper_limit.segment<6>(0) = ubA_input_lifting.segment<6>(0);

            check_l_const = A_input_lifting.block<6,6>(0,0)*q_l_sol;

            state_l =0.1;

        }
        else{ //right support
            lower_limit.segment<6>(6) = lbA_input_lifting.segment<6>(0);
            upper_limit.segment<6>(6) = ubA_input_lifting.segment<6>(0);

            check_r_const = A_input_lifting.block<6,6>(0,6)*q_r_sol;


            state_r =0.1;
        }


    }
    else if(walking_tick_ >= t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0 && walking_tick_ < t_start_+t_total_-t_double2_-t_rest_last_)// SSP + DSP ( landing the foot ) and contact whole foot
    {
        if(foot_step_(current_step_num_,6) == 1){// left support
            lower_limit.segment<6>(0) = lbA_input_landing.segment<6>(0);
            lower_limit.segment<4>(6) = lbA_input_landing.segment<4>(6);
            lower_limit(11) = lbA_input_landing(10);

            upper_limit.segment<6>(0) = ubA_input_landing.segment<6>(0);
            upper_limit.segment<4>(6) = ubA_input_landing.segment<4>(6);
            upper_limit(11) = ubA_input_landing(10);

            check_l_const = A_input_landing.block<6,6>(0,0) *q_l_sol;

            check_r_const.segment<4>(0) = A_input_landing.block<4,6>(6,6)*q_r_sol;
            check_r_const(5) = A_input_landing.block<1,6>(10,6) *q_r_sol;

            state_l =0.1;
            state_r =0.1;
        }
        else{ //right support
            lower_limit.segment<4>(0) = lbA_input_landing.segment<4>(0);
            lower_limit(5) = lbA_input_landing(4);
            lower_limit.segment<6>(6) = lbA_input_landing.segment<6>(5);

            upper_limit.segment<4>(0) = ubA_input_landing.segment<4>(0);
            upper_limit(5) = ubA_input_landing(4);
            upper_limit.segment<6>(6) = ubA_input_landing.segment<6>(5);

            check_l_const.segment<4>(0) = A_input_landing.block<4,6>(0,0) *q_l_sol;
            check_l_const(5) = A_input_landing.block<1,6>(4,0) *q_l_sol;

            check_r_const = A_input_landing.block<6,6>(5,6) *q_r_sol;

            state_l =0.1;
            state_r =0.1;
        }

    }
    else
    {

        lower_limit = lbA_input_dsp2.segment<12>(0);
        upper_limit = ubA_input_dsp2.segment<12>(0);

        check_l_const = A_input_dsp2.block<6,6>(0,0)*q_l_sol;
        check_r_const = A_input_dsp2.block<6,6>(6,6) * q_r_sol;

        state_l =0.1;
        state_r =0.1;
    }


    file[7]<<walking_tick_<<"\t"<<state_l<<"\t"<<state_r
          <<"\t"<<check_l_const(0)<<"\t"<<check_l_const(1)<<"\t"<<check_l_const(2)<<"\t"<<check_l_const(3)<<"\t"<<check_l_const(4)<<"\t"<<check_l_const(5)
          <<"\t"<<check_r_const(0)<<"\t"<<check_r_const(1)<<"\t"<<check_r_const(2)<<"\t"<<check_r_const(3)<<"\t"<<check_r_const(4)<<"\t"<<check_r_const(5)
          <<"\t"<<lower_limit(0)<<"\t"<<lower_limit(1)<<"\t"<<lower_limit(2)<<"\t"<<lower_limit(3)<<"\t"<<lower_limit(4)<<"\t"<<lower_limit(5)
                <<"\t"<<lower_limit(6)<<"\t"<<lower_limit(7)<<"\t"<<lower_limit(8)<<"\t"<<lower_limit(9)<<"\t"<<lower_limit(10)<<"\t"<<lower_limit(11)
          <<"\t"<<upper_limit(0)<<"\t"<<upper_limit(1)<<"\t"<<upper_limit(2)<<"\t"<<upper_limit(3)<<"\t"<<upper_limit(4)<<"\t"<<upper_limit(5)
                 <<"\t"<<upper_limit(6)<<"\t"<<upper_limit(7)<<"\t"<<upper_limit(8)<<"\t"<<upper_limit(9)<<"\t"<<upper_limit(10)<<"\t"<<upper_limit(11)
         <<endl;

    file[20]<<walking_tick_<<"\t"<<qOpt[0]<<"\t"<<qOpt[1]<<"\t"<<qOpt[2]<<"\t"<<qOpt[3]<<"\t"<<qOpt[4]<<"\t"<<qOpt[5]<<"\t"<<qOpt[6]<<"\t"<<qOpt[7]<<"\t"<<qOpt[8]<<"\t"<<qOpt[9]<<"\t"<<qOpt[10]<<"\t"<<qOpt[11]<<endl;
    Eigen::Vector12d qp_q;

    if(walking_tick_ == 0)
        desired_q_not_compensated2_ = current_q_;


    for(int i=0;i<6;i++){
//        qp_q(i) = qOpt[i]/hz_ + desired_q_not_compensated_[LF_BEGIN+i];
//        qp_q(i+6) = qOpt[i+6]/hz_ + desired_q_not_compensated_[RF_BEGIN+i];
        desired_q_(i) = qOpt[i]/hz_ + desired_q_not_compensated2_[LF_BEGIN + i];
        desired_q_(i+6) = qOpt[i+6]/hz_ + desired_q_not_compensated2_[RF_BEGIN +i];
    }




//    file[20]<<walking_tick_
//           <<"\t"<<lp_(0)<<"\t"<<lp_(1)<<"\t"<<lp_(2)<<"\t"<<lp_(3)<<"\t"<<lp_(4)<<"\t"<<lp_(5)
//             <<"\t"<<rp_(0)<<"\t"<<rp_(1)<<"\t"<<rp_(2)<<"\t"<<rp_(3)<<"\t"<<rp_(4)<<"\t"<<rp_(5)
//            <<"\t"<<ltoe_p_(0)<<"\t"<<ltoe_p_(1)<<"\t"<<ltoe_p_(2)<<"\t"<<ltoe_p_(3)<<"\t"<<ltoe_p_(4)<<"\t"<<ltoe_p_(5)
//              <<"\t"<<rtoe_p_(0)<<"\t"<<rtoe_p_(1)<<"\t"<<rtoe_p_(2)<<"\t"<<rtoe_p_(3)<<"\t"<<rtoe_p_(4)<<"\t"<<rtoe_p_(5)
//           <<"\t"<<lbA[0]<<"\t"<<lbA[1]<<"\t"<<lbA[2]<<"\t"<<lbA[3]<<"\t"<<lbA[4]
//          <<"\t"<<lbA[5]<<"\t"<<lbA[6]<<"\t"<<lbA[7]<<"\t"<<lbA[8]<<"\t"<<lbA[9]
//         <<"\t"<<lbA[10]<<"\t"<<lbA[11]<<"\t"<<lbA[12]<<"\t"<<lbA[13]<<"\t"<<lbA[14]<<"\t"<<lbA[15]
//           <<"\t"<<lbA[16]<<"\t"<<lbA[17]<<"\t"<<lbA[18]<<"\t"<<lbA[19]<<"\t"<<lbA[20]<<"\t"<<lbA[21]
//          <<"\t"<<ubA[0]<<"\t"<<ubA[1]<<"\t"<<ubA[2]<<"\t"<<ubA[3]<<"\t"<<ubA[4]
//         <<"\t"<<ubA[5]<<"\t"<<ubA[6]<<"\t"<<ubA[7]<<"\t"<<ubA[8]<<"\t"<<ubA[9]
//        <<"\t"<<ubA[10]<<"\t"<<ubA[11]<<"\t"<<ubA[12]<<"\t"<<ubA[13]<<"\t"<<ubA[14]<<"\t"<<ubA[15]
//          <<"\t"<<ubA[16]<<"\t"<<ubA[17]<<"\t"<<ubA[18]<<"\t"<<ubA[19]<<"\t"<<ubA[20]<<"\t"<<ubA[21]
//            <<"\t"<<check_l_const(0)<<"\t"<<check_l_const(1)<<"\t"<<check_l_const(2)<<"\t"<<check_l_const(3)<<"\t"<<check_l_const(4)<<"\t"<<check_l_const(5)
//           <<"\t"<<check_r_const(0)<<"\t"<<check_r_const(1)<<"\t"<<check_r_const(2)<<"\t"<<check_r_const(3)<<"\t"<<check_r_const(4)<<"\t"<<check_r_const(5)<<endl;

//    file[7]<<walking_tick_
//          <<"\t"<<check_l_const(0)<<"\t"<<check_l_const(1)<<"\t"<<check_l_const(2)<<"\t"<<check_l_const(3)<<"\t"<<check_l_const(4)<<"\t"<<check_l_const(5)
//            <<"\t"<<check_r_const(0)<<"\t"<<check_r_const(1)<<"\t"<<check_r_const(2)<<"\t"<<check_r_const(3)<<"\t"<<check_r_const(4)<<"\t"<<check_r_const(5)
//           <<"\t"<<lbA_dsp1[0]<<"\t"<<lbA_dsp1[1]<<"\t"<<lbA_dsp1[2]<<"\t"<<lbA_dsp1[3]<<"\t"<<lbA_dsp1[4]<<"\t"<<lbA_dsp1[5]<<"\t"<<lbA_dsp1[6]<<"\t"<<lbA_dsp1[7]<<"\t"<<lbA_dsp1[8]<<"\t"<<lbA_dsp1[9]<<"\t"<<lbA_dsp1[10]
//             <<"\t"<<ubA_dsp1[0]<<"\t"<<ubA_dsp1[1]<<"\t"<<ubA_dsp1[2]<<"\t"<<ubA_dsp1[3]<<"\t"<<ubA_dsp1[4]<<"\t"<<ubA_dsp1[5]<<"\t"<<ubA_dsp1[6]<<"\t"<<ubA_dsp1[7]<<"\t"<<ubA_dsp1[8]<<"\t"<<ubA_dsp1[9]<<"\t"<<ubA_dsp1[10]
//             <<"\t"<<lbA_lifting[0]<<"\t"<<lbA_lifting[1]<<"\t"<<lbA_lifting[2]<<"\t"<<lbA_lifting[3]<<"\t"<<lbA_lifting[4]<<"\t"<<lbA_lifting[5]
//          <<"\t"<<ubA_lifting[0]<<"\t"<<ubA_lifting[1]<<"\t"<<ubA_lifting[2]<<"\t"<<ubA_lifting[3]<<"\t"<<ubA_lifting[4]<<"\t"<<ubA_lifting[5]
//          <<"\t"<<lbA_landing[0]<<"\t"<<lbA_landing[1]<<"\t"<<lbA_landing[2]<<"\t"<<lbA_landing[3]<<"\t"<<lbA_landing[4]<<"\t"<<lbA_landing[5]<<"\t"<<lbA_landing[6]<<"\t"<<lbA_landing[7]<<"\t"<<lbA_landing[8]<<"\t"<<lbA_landing[9]<<"\t"<<lbA_landing[10]
//          <<"\t"<<lbA_landing[0]<<"\t"<<lbA_landing[1]<<"\t"<<lbA_landing[2]<<"\t"<<lbA_landing[3]<<"\t"<<lbA_landing[4]<<"\t"<<lbA_landing[5]<<"\t"<<lbA_landing[6]<<"\t"<<lbA_landing[7]<<"\t"<<lbA_landing[8]<<"\t"<<lbA_landing[9]<<"\t"<<ubA_landing[10]
//            <<"\t"<<lbA_dsp2[0]<<"\t"<<lbA_dsp2[1]<<"\t"<<lbA_dsp2[2]<<"\t"<<lbA_dsp2[3]<<"\t"<<lbA_dsp2[4]<<"\t"<<lbA_dsp2[5]<<"\t"<<lbA_dsp2[6]<<"\t"<<lbA_dsp2[7]<<"\t"<<lbA_dsp2[8]<<"\t"<<lbA_dsp2[9]<<"\t"<<lbA_dsp2[10]<<"\t"<<lbA_dsp2[11]
//                    <<"\t"<<ubA_dsp2[0]<<"\t"<<ubA_dsp2[1]<<"\t"<<ubA_dsp2[2]<<"\t"<<ubA_dsp2[3]<<"\t"<<ubA_dsp2[4]<<"\t"<<ubA_dsp2[5]<<"\t"<<ubA_dsp2[6]<<"\t"<<ubA_dsp2[7]<<"\t"<<ubA_dsp2[8]<<"\t"<<ubA_dsp2[9]<<"\t"<<ubA_dsp2[10]<<"\t"<<ubA_dsp2[11]
//                   <<endl;
//          <<"\t"<<qOpt[0]<<"\t"<<qOpt[1]<<"\t"<<qOpt[2]<<"\t"<<qOpt[3]<<"\t"<<qOpt[4]<<"\t"<<qOpt[5]
//            <<"\t"<<qOpt[6]<<"\t"<<qOpt[7]<<"\t"<<qOpt[8]<<"\t"<<qOpt[9]<<"\t"<<qOpt[10]<<"\t"<<qOpt[11]
//          <<"\t"<<qp_q[0]*RAD2DEG<<"\t"<<qp_q[1]*RAD2DEG<<"\t"<<qp_q[2]*RAD2DEG<<"\t"<<qp_q[3]*RAD2DEG<<"\t"<<qp_q[4]*RAD2DEG<<"\t"<<qp_q[5]*RAD2DEG
//             <<"\t"<<qp_q[6]*RAD2DEG<<"\t"<<qp_q[7]*RAD2DEG<<"\t"<<qp_q[8]*RAD2DEG<<"\t"<<qp_q[9]*RAD2DEG<<"\t"<<qp_q[10]*RAD2DEG<<"\t"<<qp_q[11]*RAD2DEG<<endl;


    desired_q_not_compensated2_ = desired_q_;
}
void WalkingController::qp3(){
    //variable vector consists of only zerk possible one. using analytic method for jerk

    double dt = 1.0/hz_;

    int  NL= (int) 16*hz_/10;
    NL = 320;
    NL = (int) 20*t_total_/10;

    int N = 40;
    int interval = NL/N;

//    interval = 10;
//    N = NL/interval;

    if(MPC_Matrix_cal_ == false){
//        ObtainMatrix(NL,N,dt,interval);
        getMPCMatrix(NL,N,dt,interval);
        MPC_Matrix_cal_ = true;
    }


    if(current_step_num_ == 0)
      zmp_start_time_ = 0.0;
    else
      zmp_start_time_ = t_start_;


    Eigen::Vector3d x_0, y_0;
    x_0.setZero(); y_0.setZero();
    if(walking_tick_ - zmp_start_time_ == 0 && current_step_num_ ==0){
        x_0(0) = com_support_init_(0);//-0.03;
        y_0(0) = com_support_init_(1);
    }
    else {
        x_0 = x_p1_;
        y_0 = y_p1_;

//        x_0(0) = com_support_current_(0);
////        x_0(1) = com_support_vel_(0);
////        x_0(2) = com_support_acc_(0);

//        y_0(0) = com_support_current_(1);
//        y_0(1) = com_support_vel_(1);
//        y_0(2) = com_support_acc_(1);

    }

//    if(walking_tick_ == 920){
//        x_0(0) -= 0.03;
//        x_0(1) -= 0.03;
//    }

//    x_0(0) = com_support_current_(0);

//    Eigen::MatrixXd p_ref;
//    p_ref.resize(N,2);
    double start_time;

    if(current_step_num_ == 0)
      start_time = 0;
    else
      start_time = t_start_;

//    for(int i=0;i<N;i++){//column 0 = x, column 1 =y
//        p_ref(i,0) = ref_zmp_(walking_tick_ - start_time + interval*i,0); // x position of foot
//        p_ref(i,1) = ref_zmp_(walking_tick_ - start_time + interval*i,1); // y position of foot
//    }

//    zmp_desired_(0) = ref_zmp_(walking_tick_-start_time,0);
//    zmp_desired_(1) = ref_zmp_(walking_tick_-start_time,1);


//    file[17]<<walking_tick_<<"\t"<<p_ref(0,0)<<"\t"<<p_ref(0,1)<<endl;


    Eigen::VectorXd foot_x, foot_y;

    footReferenceGenerator(foot_x, foot_y);

    Eigen::VectorXd fx_ref, fy_ref;
    fx_ref.resize(N); fy_ref.resize(N);

//    fx_ref(0) = foot_x(walking_tick_-start_time);
//    fy_ref(0) = foot_y(walking_tick_-start_time);

//    for(int i=0;i<N-1;i++){
//        fx_ref(i+1) = foot_x(walking_tick_-start_time +1 +interval*i);
//        fy_ref(i+1) = foot_y(walking_tick_-start_time +1 +interval*i);
//    }
        for(int i=0;i<N;i++){
            fx_ref(i) = foot_x(walking_tick_-start_time +interval*i);
//            fy_ref(i) = foot_y(walking_tick_-start_time +interval*i);
            fy_ref(i) = ref_zmp_(walking_tick_ - start_time + interval*i,1); // y position of foot
        }
//    for(int i=0;i<N;i++){//column 0 = x, column 1 =y
//        fx_ref(i,0) = ref_zmp_(walking_tick_ - start_time + interval*i,0); // x position of foot
//        fy_ref(i,1) = ref_zmp_(walking_tick_ - start_time + interval*i,1); // y position of foot
//    }


//    file[21]<<walking_tick_<<"\t"<<fx_ref(0)<<"\t"<<fx_ref(1)<<"\t"<<fy_ref(0)<<"\t"<<fy_ref(1)<<endl;

    Eigen::MatrixXd support_x,support_y,tempx,tempy;
    support_x.resize(N,2);
    support_y.resize(N,2);

    GetSupportPolygon(tempx,tempy);

    for(int i=0;i<N;i++){
        support_x(i,0) = tempx(walking_tick_-start_time,0);
        support_x(i,1) = tempx(walking_tick_-start_time,1);

        support_y(i,0) = tempy(walking_tick_-start_time,0);
        support_y(i,1) = tempy(walking_tick_-start_time,1);
    }

    //file[26]<<walking_tick_<<"\t"<<support_x(0,0)<<"\t"<<support_x(0,1)<<"\t"<<support_y(0,0)<<"\t"<<support_y(0,1)<<endl;

//    if(walking_tick_ == t_start_){
//        file[26]<<walking_tick_;
//        for(int i=0;i<N;i++)
//            //file[26]<<"\t"<<fx_ref(i);
//            file[26]<<"\t"<<p_ref(i,0);
//        file[26]<<endl;

//        file[26]<<walking_tick_;
//        for(int i=0;i<N;i++)
//            //file[26]<<"\t"<<fy_ref(i);
//            file[26]<<"\t"<<p_ref(i,1);
//        file[26]<<endl;

//    }




    /////
    /// \brief px_x
    ///       zmp refernece = p_ref
    ///
    ///
    Eigen::MatrixXd px_x, px_x_to_foot, py_y, py_y_to_foot;
    px_x.resize(N,1); px_x_to_foot.resize(N,1); py_y.resize(N,1); py_y_to_foot.resize(N,1);
    px_x = Px_*x_0;
    py_y = Py_*y_0;

    px_x_to_foot = px_x - fx_ref;
    py_y_to_foot = py_y - fy_ref;

    Eigen::VectorXd jerk_x, jerk_y;
    jerk_x.resize(N); jerk_y.resize(N);
    //jerk = p_temp2*(Px*x_0-p_ref);
    //jerk = p_temp_*(px_x - p_ref);
    Eigen::VectorXd temp_x, temp_y;
    temp_x.resize(N);



    ///////////////////////////////////////////////////////////////////////////////
    ///                                                                   /////////
    /// for using qpoases,                                                ////////
    /// minimize zmp to foot position, velocity delta and jerk input          ////
    ///                                                                      /////
    ///////////////////////////////////////////////////////////////////////////////

    Eigen::MatrixXd temp_margin_x, temp_margin_y;
    calculateFootMargin2(temp_margin_x,temp_margin_y);

    Eigen::MatrixXd cst_x, cst_y;
    cst_x.resize(N,2); cst_y.resize(N,2);

//    cst_x(0,0) = temp_margin_x(walking_tick_-start_time,0);
//    cst_x(0,1) = temp_margin_x(walking_tick_-start_time,1);

//    cst_y(0,0) = temp_margin_y(walking_tick_-start_time,0);
//    cst_y(0,1) = temp_margin_y(walking_tick_-start_time,1);

//    for(int i=0;i<N;i++){
//        cst_x(i+1,0) = temp_margin_x(walking_tick_-start_time+interval*i+1,0);
//        cst_x(i+1,1) = temp_margin_x(walking_tick_-start_time+interval*i+1,1);

//        cst_y(i+1,0) = temp_margin_y(walking_tick_-start_time+interval*i+1,0);
//        cst_y(i+1,1) = temp_margin_y(walking_tick_-start_time+interval*i+1,1);
//    }

        for(int i=0;i<N;i++){
            cst_x(i,0) = temp_margin_x(walking_tick_-start_time+interval*i,0);
            cst_x(i,1) = temp_margin_x(walking_tick_-start_time+interval*i,1);

            cst_y(i,0) = temp_margin_y(walking_tick_-start_time+interval*i,0);
            cst_y(i,1) = temp_margin_y(walking_tick_-start_time+interval*i,1);
        }


    Eigen::MatrixXd Px_zmp, Px_vel, Py_zmp(N,1), Py_vel(N,1);
    Px_zmp.resize(N,1); Px_vel.resize(N,1);


    Eigen::MatrixXd selec_v_x, selec_v_y(N,1);
    selec_v_x.resize(N,1);

    selec_v_x = Selec_delta_v_reduced_*x_0;
    selec_v_y = Selec_delta_v_reduced_*y_0;
    Px_vel = Selec_B_vi_reduced_.transpose()*selec_v_x;
    Py_vel = Selec_B_vi_reduced_.transpose()*selec_v_y;

//    Eigen::MatrixXd selec_a_x(N,1), selec_a_y(N,1), Px_acc(N,1),Py_acc(N,1);
//    selec_a_x = A_ai_reduced_*x_0;
//    selec_a_y = A_ai_reduced_*y_0;

//    Px_acc = B_ai_reduced_.transpose()*selec_a_x;
//    Py_acc = B_ai_reduced_.transpose()*selec_a_y;

    Px_zmp = pu_.transpose()*px_x_to_foot;
    Py_zmp = pu_.transpose()*py_y_to_foot;


    Eigen::MatrixXd px_input, py_input;
    px_input.resize(N,1); py_input.resize(N,1);

    px_input = alpha_x_*Px_zmp + beta_x_*Px_vel; // for delta velocity
    py_input = alpha_y_*Py_zmp + beta_y_*Py_vel;

//    px_input = alpha_x_*Px_zmp + beta_x_*Px_acc; // for acceleration
//    py_input = alpha_y_*Py_zmp + beta_y_*Py_acc;

    Eigen::MatrixXd px_variance, py_variance;
    px_variance.resize(1,N); py_variance.resize(1,N);
    Eigen::MatrixXd tempx_var(N,1), tempy_var(N,1);


    Eigen::MatrixXd  gu(2*N,1);
    gu.setZero();


//    if(walking_tick_ ==0){
//        for(int i=0;i<2*N;i++){
//            for(int j=0;j<2*N;j++){
//                file[23]<<Qu_(i,j)<<"\t";
//            }
//            file[23]<<endl;
//        }
//    }



    gu.block(0,0,N,1) = px_input;
    gu.block(N,0,N,1) = py_input;


    real_t Qx_input[N*N], gx_input[N], lbx[N],ubx[N], Ax_input[N*N], lbAx[N],ubAx[N];
    real_t Qy_input[N*N], gy_input[N], lby[N],uby[N], Ay_input[N*N], lbAy[N],ubAy[N];

    for(int i=0;i<N;i++){
        for(int j=0;j<N;j++){
            Qx_input[j*N +i] = Qx_(i,j); // for delta velocity
            Ax_input[j*N+i] = pu_(i,j); // matrix for Jerk

//            Ay_input[j*N+i] = pu_(i,j);
//            Qy_input[j*N+i] = Qy_(i,j);

        }
         gx_input[i] = px_input(i);
//         gy_input[i] = py_input(i);

         lbx[i] = -10;
         ubx[i] = 10;

//         lby[i] = -10;
//         uby[i] = 10;


         lbAx[i] = cst_x(i,0) -px_x_to_foot(i);
         ubAx[i] = cst_x(i,1) - px_x_to_foot(i);

//         lbAy[i] = cst_y(i,0) -py_y_to_foot(i);
//         ubAy[i] = cst_y(i,1) - py_y_to_foot(i);
    }


    real_t Qu_input[2*N*2*N], gu_input[2*N], lbu[2*N],ubu[2*N],Au_input[2*N*2*N],lbAu[2*N],ubAu[2*N];


//    for(int i=0;i<2*N;i++){
//        for(int j=0;j<2*N;j++){
//            Qu_input[j*2*N+i] = Qu_(i,j);
//            Au_input[j*2*N+i] = Au_(i,j);
//        }
//        gu_input[i] = gu(i,0);
//        lbu[i] = -10;
//        ubu[i] = 10;
//    }
//    for(int i=0;i<N;i++){
//        lbAu[i] = cst_x(i,0) - px_x_to_foot(i);
//        ubAu[i] = cst_x(i,1) - px_x_to_foot(i);

//        lbAu[N+i] = cst_y(i,0) - py_y_to_foot(i);
//        ubAu[N+i] = cst_y(i,1) - py_y_to_foot(i);
//    }


    file[22]<<walking_tick_<<"\t"<<cst_x(0,0)<<"\t"<<cst_x(0,1)<<"\t"<<cst_y(0,0)<<"\t"<<cst_y(0,1)<<endl;

    /////////////////////////end of matrix for qpoases///////////////////////////////////////

    int_t nV;
    nV = N;
//    //QProblemB mpc(nV);
    QProblem mpc_x(nV,nV);
    Options op, op1;

    int_t nWSR = 10000;


    op.initialStatusBounds = ST_INACTIVE;
    op.numRefinementSteps = 1;
    op.enableCholeskyRefactorisation = 1;
//    op.setToMPC();
//    op.setToReliable();

    op.printLevel = PL_NONE;




    for(int i=0;i<N;i++){
        for(int j=0;j<N;j++){
//            Qx_input[j*N +i] = Qx_(i,j); // for delta velocity
//            Ax_input[j*N+i] = pu_(i,j); // matrix for Jerk

            Ay_input[j*N+i] = pu_(i,j);
            Qy_input[j*N+i] = Qy_(i,j);

        }
//         gx_input[i] = px_input(i);
         gy_input[i] = py_input(i);

//         lbx[i] = -10;
//         ubx[i] = 10;

         lby[i] = -10;
         uby[i] = 10;


//         lbAx[i] = cst_x(i,0) -px_x_to_foot(i);
//         ubAx[i] = cst_x(i,1) - px_x_to_foot(i);

         lbAy[i] = cst_y(i,0) -py_y_to_foot(i);
         ubAy[i] = cst_y(i,1) - py_y_to_foot(i);
    }

    op1.initialStatusBounds = ST_INACTIVE;
    op1.numRefinementSteps = 1;
    op1.enableCholeskyRefactorisation = 1;
//    op.setToMPC();
//    op.setToReliable();

    op1.printLevel = PL_NONE;
////    op.enableEqualities = BT_TRUE;
////    op.boundRelaxation = 1E-5;
///    

//        if(walking_tick_ == 0){
//            int_t nVu = 2*N;


//            QProblem mpc_u(nVu,nVu);

//            real_t xopt_u[nVu];

//            mpc_u.setOptions(op);
//            chrono::high_resolution_clock::time_point t_1 = std::chrono::high_resolution_clock::now();
//            mpc_u.init(Qu_input,gu_input,Au_input,lbu,ubu,lbAu,ubAu, nWSR);
//            mpc_u.getPrimalSolution(xopt_u);

//            mpc_u.hotstart(gu_input,lbu,ubu,lbAu,ubAu,nWSR);
//            returnValue qp_res;

//            qp_res = mpc_u.getPrimalSolution(xopt_u);
//            chrono::duration<double> t_2 = std::chrono::high_resolution_clock::now() - t_1;

//            x_d1_ = A_*x_0 + b1_*xopt_u[0];
//            y_d1_ = A_*y_0 + b1_*xopt_u[N];

//            opt_x_ = xopt_u[1];
//            opt_y_ = xopt_u[N+1];
//        }
//        else {
//            if(walking_tick_ % 2 == 0){
//                QProblem mpc_y(nV,nV);
//                real_t yopt[nV];

//                mpc_y.setOptions(op1);

//                chrono::high_resolution_clock::time_point t_12 = std::chrono::high_resolution_clock::now();
//                mpc_y.init(Qy_input,gy_input,Ay_input,lby,uby,lbAy,ubAy,nWSR);
//                mpc_y.getPrimalSolution(yopt);


//            ////    //mpc.hotstart(g_input,lb,ub,nWSR,0);
//                mpc_y.hotstart(gy_input,lby,uby,lbAy,ubAy,nWSR);
//                mpc_y.getPrimalSolution(yopt);

//                x_d1_ = A_*x_0 + b1_*opt_x_;
//                y_d1_ = A_*y_0 + b1_*yopt[0];

//                opt_y_ = yopt[1];
//            }
//            else
//            {
                mpc_x.setOptions(op);

                real_t xopt[nV];

                chrono::high_resolution_clock::time_point t_1 = std::chrono::high_resolution_clock::now();
                mpc_x.init(Qx_input,gx_input,Ax_input,lbx,ubx,lbAx,ubAx,nWSR);
                mpc_x.getPrimalSolution(xopt);

                //mpc.hotstart(g_input,lb,ub,nWSR,0);
                mpc_x.hotstart(gx_input,lbx,ubx,lbAx,ubAx,nWSR);
                mpc_x.getPrimalSolution(xopt);

                x_d1_ = A_*x_0 + b1_*xopt[0];
//                y_d1_ = A_*y_0 + b1_*opt_y_;

//                opt_x_ = xopt[1];
//            }

//        }







//        QProblem mpc_y(nV,nV);
//        real_t yopt[nV];

//        mpc_y.setOptions(op1);

////            chrono::high_resolution_clock::time_point t_12 = std::chrono::high_resolution_clock::now();
//        mpc_y.init(Qy_input,gy_input,Ay_input,lby,uby,lbAy,ubAy,nWSR);
//        mpc_y.getPrimalSolution(yopt);


//    ////    //mpc.hotstart(g_input,lb,ub,nWSR,0);
//        mpc_y.hotstart(gy_input,lby,uby,lbAy,ubAy,nWSR);
//        mpc_y.getPrimalSolution(yopt);


//        y_d1_ = A_*y_0 + b1_*yopt[0];


//        mpc_x.setOptions(op);

//        real_t xopt[nV];

//    //            chrono::high_resolution_clock::time_point t_1 = std::chrono::high_resolution_clock::now();
//        mpc_x.init(Qx_input,gx_input,Ax_input,lbx,ubx,lbAx,ubAx,nWSR);
//        mpc_x.getPrimalSolution(xopt);

//        //mpc.hotstart(g_input,lb,ub,nWSR,0);
//        mpc_x.hotstart(gx_input,lbx,ubx,lbAx,ubAx,nWSR);
//        mpc_x.getPrimalSolution(xopt);

//        x_d1_ = A_*x_0 + b1_*xopt[0];






//    x_d1_ = A_*x_0 + b1_*xopt[0];
//    y_d1_ = A_*y_0 + b1_*yopt[0];

//    chrono::duration<double> t_2 = std::chrono::high_resolution_clock::now() - t_1;



//    chrono::duration<double> t_22 = std::chrono::high_resolution_clock::now() - t_1;

// cout<<"computation time for x : "<<t_2.count()<<", for y : "<<t_22.count()<<endl;



//    if(qp_res==SUCCESSFUL_RETURN)
//    {
//        cout<<"qp solve success!"<<std::endl;
//    }
//    else
//    {
//        cout<<"qp solve failed!"<<std::endl;

//    }
//    cout<<"compute time : "<<t_2.count()<<endl;

//    x_d1_ = A_*x_0 + b1_*jerk_x(0);
//    y_d1_ = A_*y_0 + b1_*jerk_y(0);



    //    Eigen::VectorXd x_opt(N), y_opt(N);
    //    for(int i=0;i<N;i++){
    //        x_opt(i) = xopt_u[i];
    //        y_opt(i) = xopt_u[i+N];
    //    }

    Eigen::MatrixXd boundarycheck;
    boundarycheck.resize(N,1);

//    for(int i=0;i<N;i++)
//        boundarycheck(i) = xopt[i];

//    boundarycheck = pu_*boundarycheck;

//    if(walking_tick_ ==0||walking_tick_ ==t_start_){
//        for(int i=0;i<N;i++){
//            file[27]<<lbA[i]<<"\t"<<ubA[i]<<"\t"<<boundarycheck(i)<<endl;

//        }
//    }
///////////////////////////////////////////
    /// check
    ///
    Eigen::VectorXd temp1, temp2, temp3;
    temp1.resize(N,1); temp2.resize(N,1); temp3.resize(N,1);

//    temp1 = px_x_to_foot + pu_*jerk_x;
//    temp2 = selec_delta_v_*x_0 + selec_bv_*jerk_x;
//    temp3 = jerk_x;

    Eigen::Matrix<double, 1, 1> optvalue;

    //optvalue = 0.5*alpha_*temp1.transpose()*temp1 + 0.5*beta_*temp2.transpose()*temp2 + 0.5*gamma_*jerk_x.transpose()*jerk_x;

    Eigen::Matrix<double, 1, 3>c;
    c(0) = 1.0; c(1) = 0.0; c(2) = -zc_/GRAVITY;
    double zmp_x, zmp_y;



    zmp_x = c*x_d1_;
    zmp_y = c*y_d1_;

    ///////////////////////////////////

    x_p1_ = x_d1_;
    y_p1_ = y_d1_;



//    file[25]<<walking_tick_<<"\t"<<x_d1_(0)<<"\t"<<x_d1_(1)<<"\t"<<x_d1_(2)<<"\t"<<y_d1_(0)<<"\t"<<y_d1_(1)<<"\t"<<y_d1_(2)
////           <<"\t"<<jerk_x(0)<<"\t"<<jerk_y(0)<<endl;
//           <<"\t"<<xopt_u[0]<<"\t"<<xopt_u[N]<<"\t"<<qp_res<<endl;
////          <<"\t"<<pux<<"\t"<<puy<<endl;//"\t"<<zmp_x<<"\t"<<zmp_y<<endl;//<<"\t"<<pux<<"\t"<<puy<<"\t"<<xopt[0]<<"\t"<<yopt[0]<<"\t"<<xopt_u[0]<<"\t"<<xopt_u[N]<<endl;//"\t"<<yopt[0]<<endl;//<<"\t"<<mpc.getObjVal()<<"\t"<<optvalue<<"\t"<<fx_ref(0)<<  endl;//<<xOpt[0]<<endl;
///
    file[20]<<walking_tick_<<"\t"<<zmp_x<<"\t"<<zmp_y<<endl;//"\t"<<t_2.count()<<endl;//"\t"<<t_22.count()<<endl;

    SupportfootComUpdate(x_p1_,y_p1_,x_p1_,y_p1_);

    xs_ = x_p1_;
//    ys_ = y_p1_;

//    if(current_step_num_ != total_step_num_-1){
      if(com_control_mode_ == true)
      {
        com_desired_(0) = x_d1_(0);
//        com_desired_(1) = y_d1_(0);
        com_desired_(2) = DyrosMath::cubic(walking_tick_, t_start_, t_start_real_, pelv_support_init_.translation()(2), pelv_suppprt_start_.translation()(2), 0, 0);

        com_dot_desired_(0) = x_d1_(1);
//        com_dot_desired_(1) = y_d1_(1);
        com_dot_desired_(2) = DyrosMath::cubicDot(walking_tick_, t_start_, t_start_real_, pelv_support_init_.translation()(2), pelv_suppprt_start_.translation()(2), 0, 0, hz_);

      }
      else
      {
        com_desired_(0) = x_d1_(0);
//        com_desired_(1) = y_d1_(0);
        com_desired_(2) = DyrosMath::cubic(walking_tick_, t_start_, t_start_real_, pelv_support_init_.translation()(2), pelv_suppprt_start_.translation()(2), 0, 0);

        com_dot_desired_(0) = x_d1_(1);
//        com_dot_desired_(1) = y_d1_(1);
        com_dot_desired_(2) = DyrosMath::cubicDot(walking_tick_, t_start_, t_start_real_, pelv_support_init_.translation()(2), pelv_suppprt_start_.translation()(2), 0, 0, hz_);

      }

//    }


}
void WalkingController::getMPCMatrix(int full_n, int sampling_n, double dt, int interval){
    // dt = 1/200 = 0.005;

    cout<<"new get matrix func"<<endl;
    Eigen::Matrix3d A;
    A.setIdentity();
    A(0,1) = dt; A(0,2) = pow(dt,2)/2.0;
    A(1,2) = dt;
    A_ = A;
    Eigen::Vector3d b;
    b(0) = pow(dt,3)/6.0;
    b(1) = pow(dt,2)/2.0;
    b(2) = dt;
    b1_ = b;
    Eigen::Matrix<double, 1, 3>c;
    c(0) = 1.0; c(1) = 0.0; c(2) = -zc_/GRAVITY;

    Eigen::MatrixXd A_full(3*full_n,3);
    A_full.setZero();
    for(int i=0;i<full_n;i++){
        A_full(3*i,0) = 1.0;
        A_full(3*i,1) = (i+1)*dt;
        A_full(3*i,2) = (i+1)*(i+1)/2.0 * pow(dt,2);

        A_full(3*i+1,1) = 1.0;
        A_full(3*i+1,2) =(i+1)*dt;

        A_full(3*i+2,2) = 1.0;
    }
    Eigen::MatrixXd B_full(3*full_n,full_n);
    B_full.setZero();

    for(int i=0;i<full_n;i++){
        for(int j=0;j<i+1;j++){
            int nl = i-j;
            B_full(3*i,j) = (1+3*nl+3*nl*nl)*pow(dt,3)/6.0;
            B_full(3*i+1,j) = (1+2*nl)*pow(dt,2)/2.0;
            B_full(3*i+2,j) = dt;
        }
    }


    Eigen::MatrixXd px_full(full_n,3);
    Eigen::MatrixXd pu_full(full_n,full_n);
    px_full.setZero(); pu_full.setZero();

    for(int i=0;i<full_n;i++){
        px_full(i,0) = 1.0;
        px_full(i,1) = (i+1)*dt;
        px_full(i,2) = (i+1)*(i+1)*dt*dt*0.5-zc_/GRAVITY;

        for(int j=0;j<i;j++){
            int nl = i-j;
            pu_full(i,j) = (1+3*nl+3*nl*nl)*pow(dt,3)/6.0 - dt*zc_/GRAVITY;
        }
    }

    Eigen::MatrixXd A_vi(full_n,3);
    Eigen::MatrixXd B_vi(full_n,full_n);
    A_vi.setZero();
    for(int i=0;i<full_n;i++){
        A_vi.row(i) = A_full.row(3*i+1);
        B_vi.row(i) = B_full.row(3*i+1);
    }

    Selec_delta_.resize(full_n,full_n);
    Selec_delta_.setZero();
    Eigen::Matrix<double, 1,2> del_vel;
    del_vel(0,0) = -1.0; del_vel(0,1) = 1.0;

    Selec_delta_(0,0) = 1.0;
    for(int i=1;i<full_n;i++)
        Selec_delta_.block<1,2>(i,i-1) = del_vel;

    Eigen::MatrixXd Selec_v0(full_n,3);
    Selec_v0.setZero();
    Selec_v0(0,1) = 1.0;

    Eigen::MatrixXd Selec_A_vi(full_n,3), Selec_B_vi(full_n,full_n);
    Selec_A_vi = Selec_delta_*A_vi;
    Selec_B_vi = Selec_delta_*B_vi;


//cout<<"11111111111111"<<endl;
    /////////////////////// above are full size matrix////////////////
    /////////////////////// below are reduced size matrix////////////

    Eigen::MatrixXd A_reduced(3*sampling_n,3);

    for(int i=0;i<sampling_n;i++){
        A_reduced.block<3,3>(3*i,0) = A_full.block<3,3>(3*i*interval,0);
    }
//cout<<"aaaaaaaaaaaaaaaaaaaaaa"<<endl;
    Eigen::MatrixXd B_reduced(3*sampling_n,sampling_n);
    B_reduced.setZero();

    for(int i=0;i<sampling_n;i++){
        for(int j=0;j<i;j++){
            B_reduced.block<3,1>(3*i,j) = B_full.block<3,1>(3*i*interval,j*interval);
        }
    }
//cout<<"bbbbbbbbbbbbbbbbbbbbbbb"<<endl;
    Eigen::MatrixXd px_reduced(sampling_n,3), pu_reduced(sampling_n,sampling_n);
    px_reduced.setZero(); pu_reduced.setZero();

    for(int i= 0;i<sampling_n;i++){
        px_reduced.row(i) = px_full.row(i*interval);
        for(int j=0;j<i+1;j++){
            pu_reduced(i,j) = pu_full(interval*i,interval*j);
        }

    }
//cout<<"cccccccccccccccccccccccc"<<endl;
    Px_.resize(sampling_n,3); Py_.resize(sampling_n,3);
    Px_ = px_reduced;
    Py_ = Px_;

    pu_ = pu_reduced;

    Eigen::MatrixXd A_vi_reduced(sampling_n,3), B_vi_reduced(sampling_n,sampling_n);
    A_vi_reduced.setZero(); B_vi_reduced.setZero();

//cout<<"22222222222222222"<<endl;

    /////// setting the gain //////
    alpha_x_ = 1.0;  alpha_y_ = 1.0; // gain for zmp
    beta_x_ = 0.001; beta_y_ = 0.00001;//0.000001;
    gamma_x_ = 0.00001;    gamma_y_ = 0.00001;

    ////////// making for qp problem ///////////

    Eigen::MatrixXd pu_t_pu(sampling_n,sampling_n);

    pu_t_pu = pu_reduced.transpose() * pu_reduced;

    Selec_A_vi_reduced_.resize(sampling_n,3);       Selec_A_vi_reduced_.setZero();
    Selec_B_vi_reduced_.resize(sampling_n,sampling_n);     Selec_B_vi_reduced_.setZero();

    Eigen::MatrixXd Selec_v0_reduced(sampling_n,3);
    Selec_v0_reduced.setZero();
    for(int i=0;i<sampling_n;i++){
        Selec_A_vi_reduced_.row(i) = Selec_A_vi.row(i*interval);
        Selec_v0_reduced.row(i) = Selec_v0.row(i*interval);
        for(int j=0;j<i;j++){
            Selec_B_vi_reduced_(i,j) = Selec_B_vi(interval*i, interval*j);
        }
    }
//cout<<"3333333333333333333333"<<endl;
    Selec_delta_v_reduced_.resize(sampling_n,3);
    Selec_delta_v_reduced_.setZero();
    Selec_delta_v_reduced_ = Selec_A_vi_reduced_ - Selec_v0_reduced;


    Eigen::MatrixXd delta_velocity_cost(sampling_n,sampling_n);
    delta_velocity_cost.setZero();

    delta_velocity_cost = Selec_B_vi_reduced_.transpose()*Selec_B_vi_reduced_;


    Eigen::MatrixXd Iden_sampling_n(sampling_n,sampling_n);
    Iden_sampling_n.setIdentity();

//cout<<"444444444444444444444"<<endl;
    Qx_.resize(sampling_n,sampling_n);  Qy_.resize(sampling_n,sampling_n);
    Qx_.setZero();                      Qy_.setZero();

    Qx_ = alpha_x_*pu_t_pu + beta_x_*delta_velocity_cost + gamma_x_*Iden_sampling_n;
    Qy_ = alpha_y_*pu_t_pu + beta_y_*delta_velocity_cost + gamma_y_*Iden_sampling_n;

    Qx_inverse_ = Qx_.inverse();    Qy_inverse_ = Qy_.inverse();

    Qu_.resize(2*sampling_n,2*sampling_n);
    Qu_.setZero();

    Qu_.block(0,0,sampling_n,sampling_n) = Qx_;
    Qu_.block(sampling_n,sampling_n,sampling_n,sampling_n) = Qy_;

    Au_.resize(2*sampling_n, 2*sampling_n);
    Au_.block(0,0,sampling_n,sampling_n) = pu_;
    Au_.block(sampling_n,sampling_n,sampling_n,sampling_n) = pu_;


//    cout<<"Obtain mpc matric at count 0"<<endl;


}
void WalkingController::getMPCMatrix2(int full_n, int sampling_n, double dt, int interval){
    // test new matrix.
    // using ||a||^2 instead of ||delta V||^2
    // usin U vector (1, 2, 2+dt, 2+2*dt)// to use second value for next walking tick

    // dt = 1/200 = 0.005;

    cout<<"new get matrix func"<<endl;
    Eigen::Matrix3d A;
    A.setIdentity();
    A(0,1) = dt; A(0,2) = pow(dt,2)/2.0;
    A(1,2) = dt;
    A_ = A;
    Eigen::Vector3d b;
    b(0) = pow(dt,3)/6.0;
    b(1) = pow(dt,2)/2.0;
    b(2) = dt;
    b1_ = b;
    Eigen::Matrix<double, 1, 3>c;
    c(0) = 1.0; c(1) = 0.0; c(2) = -zc_/GRAVITY;

    Eigen::MatrixXd A_full(3*full_n,3);
    A_full.setZero();
    for(int i=0;i<full_n;i++){
        A_full(3*i,0) = 1.0;
        A_full(3*i,1) = (i+1)*dt;
        A_full(3*i,2) = (i+1)*(i+1)/2.0 * pow(dt,2);

        A_full(3*i+1,1) = 1.0;
        A_full(3*i+1,2) =(i+1)*dt;

        A_full(3*i+2,2) = 1.0;
    }
    Eigen::MatrixXd B_full(3*full_n,full_n);
    B_full.setZero();

    for(int i=0;i<full_n;i++){
        for(int j=0;j<i+1;j++){
            int nl = i-j;
            B_full(3*i,j) = (1+3*nl+3*nl*nl)*pow(dt,3)/6.0;
            B_full(3*i+1,j) = (1+2*nl)*pow(dt,2)/2.0;
            B_full(3*i+2,j) = dt;
        }
    }

    for(int i=0;i<3*full_n;i++){
        for(int j=0;j<full_n;j++){
            file[18]<<B_full(i,j)<<"\t";
        }
        file[18]<<endl;
    }

    Eigen::MatrixXd px_full(full_n,3);
    Eigen::MatrixXd pu_full(full_n,full_n);
    px_full.setZero(); pu_full.setZero();

    for(int i=0;i<full_n;i++){
        px_full(i,0) = 1.0;
        px_full(i,1) = (i+1)*dt;
        px_full(i,2) = (i+1)*(i+1)*dt*dt*0.5-zc_/GRAVITY;

        for(int j=0;j<i;j++){
            int nl = i-j;
            pu_full(i,j) = (1+3*nl+3*nl*nl)*pow(dt,3)/6.0 - dt*zc_/GRAVITY;
        }
    }

    Eigen::MatrixXd A_vi(full_n,3);
    Eigen::MatrixXd B_vi(full_n,full_n);
    A_vi.setZero();
    for(int i=0;i<full_n;i++){
        A_vi.row(i) = A_full.row(3*i+1);
        B_vi.row(i) = B_full.row(3*i+1);
    }

    Selec_delta_.resize(full_n,full_n);
    Selec_delta_.setZero();
    Eigen::Matrix<double, 1,2> del_vel;
    del_vel(0,0) = -1.0; del_vel(0,1) = 1.0;

    Selec_delta_(0,0) = 1.0;
    for(int i=1;i<full_n;i++)
        Selec_delta_.block<1,2>(i,i-1) = del_vel;

    Eigen::MatrixXd Selec_v0(full_n,3);
    Selec_v0.setZero();
    Selec_v0(0,1) = 1.0;

    Eigen::MatrixXd Selec_A_vi(full_n,3), Selec_B_vi(full_n,full_n);
    Selec_A_vi = Selec_delta_*A_vi;
    Selec_B_vi = Selec_delta_*B_vi;

    //////////
    Eigen::MatrixXd A_ai(full_n,3);
    Eigen::MatrixXd B_ai(full_n,full_n);

    // for selecting acceleration
//    for(int i=0;i<full_n;i++){
//        A_ai.row(i) = A_full.row(3*i+2);
//        B_ai.row(i) = B_full.row(3*i+2);
//    }

    // for selecting only velocity
    for(int i=0;i<full_n;i++){
        A_ai.row(i) = A_full.row(3*i+1);
        B_ai.row(i) = B_full.row(3*i+1);
    }



    /////////////////////// above are full size matrix////////////////
    /////////////////////// below are reduced size matrix////////////
 // using u (i+1, i+2, i+2+dt, ... )
    Eigen::MatrixXd A_reduced(3*sampling_n,3);

    A_reduced.block<3,3>(0,0) = A_full.block<3,3>(0,0);

//    for(int i=0;i<sampling_n-1;i++){
//        A_reduced.block<3,3>(3+3*i,0) = A_full.block<3,3>(3+3*i*interval,0);
//    }

//    for(int i=0;i<sampling_n;i++)
//        file[15]<<A_reduced(i,0)<<"\t"<<A_reduced(i,1)<<"\t"<<A_reduced(i,2)<<endl;

////cout<<"aaaaaaaaaaaaaaaaaaaaaa"<<endl;
//    Eigen::MatrixXd B_reduced(3*sampling_n,sampling_n);
//    B_reduced.setZero();

//    B_reduced.block<3,1>(0,0) = B_full.block<3,1>(0,0);
//    for(int i=0;i<sampling_n-1;i++){
//        B_reduced.block<3,1>(3+3*i,0) = B_full.block<3,1>(3+3*i*interval,0);
//        for(int j=0;j<i+1;j++){
//            B_reduced.block<3,1>(3+3*i,j+1) = B_full.block<3,1>(3+3*i*interval,1+j*interval);
//        }
//    }

//    for(int i=0;i<3*sampling_n;i++){
//        for(int j=0;j<sampling_n;j++)
//            file[19]<<B_reduced(i,j)<<"\t";

//        file[19]<<endl;
//    }

////cout<<"bbbbbbbbbbbbbbbbbbbbbbb"<<endl;
//    Eigen::MatrixXd px_reduced(sampling_n,3), pu_reduced(sampling_n,sampling_n);
//    px_reduced.setZero(); pu_reduced.setZero();

//    px_reduced.row(0) = px_full.row(0);
//    pu_reduced(0,0) = pu_full(0,0);
//    for(int i= 0;i<sampling_n-1;i++){
//        px_reduced.row(1+i) = px_full.row(1+i*interval);
//        pu_reduced(i+1,0) = pu_full(interval*i+1,0);
//        for(int j=0;j<i+1;j++){
//            pu_reduced(i+1,1+j) = pu_full(interval*i+1,1+interval*j);
//        }

//    }


    for(int i=0;i<sampling_n;i++){
        A_reduced.block<3,3>(3*i,0) = A_full.block<3,3>(3*i*interval,0);
    }
//cout<<"aaaaaaaaaaaaaaaaaaaaaa"<<endl;
    Eigen::MatrixXd B_reduced(3*sampling_n,sampling_n);
    B_reduced.setZero();

    for(int i=0;i<sampling_n;i++){
        for(int j=0;j<i;j++){
            B_reduced.block<3,1>(3*i,j) = B_full.block<3,1>(3*i*interval,j*interval);
        }
    }
//cout<<"bbbbbbbbbbbbbbbbbbbbbbb"<<endl;
    Eigen::MatrixXd px_reduced(sampling_n,3), pu_reduced(sampling_n,sampling_n);
    px_reduced.setZero(); pu_reduced.setZero();

    for(int i= 0;i<sampling_n;i++){
        px_reduced.row(i) = px_full.row(i*interval);
        for(int j=0;j<i+1;j++){
            pu_reduced(i,j) = pu_full(interval*i,interval*j);
        }

    }

//cout<<"cccccccccccccccccccccccc"<<endl;
    Px_.resize(sampling_n,3); Py_.resize(sampling_n,3);
    Px_ = px_reduced;
    Py_ = Px_;

    pu_ = pu_reduced;

    Eigen::MatrixXd A_vi_reduced(sampling_n,3), B_vi_reduced(sampling_n,sampling_n);
    A_vi_reduced.setZero(); B_vi_reduced.setZero();

//cout<<"22222222222222222"<<endl;

    /////// setting the gain //////
    alpha_x_ = 1.0;  alpha_y_ = 1.0; // gain for zmp
    beta_x_ = 1.0; beta_y_ = 1.0;//0.000001;
    gamma_x_ = 0.000001;    gamma_y_ = 0.000001;

    ////////// making for qp problem ///////////

    Eigen::MatrixXd pu_t_pu(sampling_n,sampling_n);

    pu_t_pu = pu_reduced.transpose() * pu_reduced;

    ////////////////////////// for delta V///////////////////////////
    //////////////////////////////////////////////////////////////////
    ///
    Selec_A_vi_reduced_.resize(sampling_n,3);       Selec_A_vi_reduced_.setZero();
    Selec_B_vi_reduced_.resize(sampling_n,sampling_n);     Selec_B_vi_reduced_.setZero();

    Eigen::MatrixXd Selec_v0_reduced(sampling_n,3);
    Selec_v0_reduced.setZero();
    for(int i=0;i<sampling_n;i++){
        Selec_A_vi_reduced_.row(i) = Selec_A_vi.row(i*interval);
        Selec_v0_reduced.row(i) = Selec_v0.row(i*interval);
        for(int j=0;j<i;j++){
            Selec_B_vi_reduced_(i,j) = Selec_B_vi(interval*i, interval*j);
        }
    }
cout<<"3333333333333333333333"<<endl;
    Selec_delta_v_reduced_.resize(sampling_n,3);
    Selec_delta_v_reduced_.setZero();
    Selec_delta_v_reduced_ = Selec_A_vi_reduced_ - Selec_v0_reduced;

    Eigen::MatrixXd delta_velocity_cost(sampling_n,sampling_n);
    delta_velocity_cost.setZero();

    delta_velocity_cost = Selec_B_vi_reduced_.transpose()*Selec_B_vi_reduced_;

    //////////////////////////////////////////////////////////////////
    ///
    /// for |a|^2//////////////////////////////////////////////////////
    ///
    ///
    Eigen::MatrixXd A_ai_reduced(sampling_n,3), B_ai_reduced(sampling_n,sampling_n);
    A_ai_reduced.setZero(); B_ai_reduced.setZero();

//cout<<"22222222222222222"<<endl;


    /////// setting the gain //////
    alpha_x_ = 1.0;  alpha_y_ = 1.0; // gain for zmp
    beta_x_ = 0.1; beta_y_ = 0.1;//0.000001;
    gamma_x_ = 0.000001;    gamma_y_ = 0.000001;

    ////////// making for qp problem ///////////

   /////////////////////////////////// working // to do
    A_ai_reduced_.resize(sampling_n,3);       A_ai_reduced_.setZero();
    B_ai_reduced_.resize(sampling_n,sampling_n);     B_ai_reduced_.setZero();

    Eigen::MatrixXd Selec_a0_reduced(sampling_n,3);
    Selec_v0_reduced.setZero();
    for(int i=0;i<sampling_n;i++){
        A_ai_reduced_.row(i) = A_ai.row(i*interval);

        for(int j=0;j<i;j++){
            B_ai_reduced_(i,j) = B_ai(interval*i, interval*j);
        }
    }



    Eigen::MatrixXd Ba_cost(sampling_n,sampling_n);
    Ba_cost.setZero();

    Ba_cost = B_ai_reduced_.transpose()*B_ai_reduced_;

    Eigen::MatrixXd Iden_sampling_n(sampling_n,sampling_n);
    Iden_sampling_n.setIdentity();

//cout<<"444444444444444444444"<<endl;
    Qx_.resize(sampling_n,sampling_n);  Qy_.resize(sampling_n,sampling_n);
    Qx_.setZero();                      Qy_.setZero();

    // for delta X
//    Qx_ = alpha_x_*pu_t_pu + beta_x_*delta_velocity_cost + gamma_x_*Iden_sampling_n;
//    Qy_ = alpha_y_*pu_t_pu + beta_y_*delta_velocity_cost + gamma_y_*Iden_sampling_n;

    // for |a|^2
    Qx_ = alpha_x_*pu_t_pu + beta_x_*Ba_cost + gamma_x_*Iden_sampling_n;
    Qy_ = alpha_y_*pu_t_pu + beta_y_*Ba_cost + gamma_y_*Iden_sampling_n;

    Qx_inverse_ = Qx_.inverse();    Qy_inverse_ = Qy_.inverse();


    Qx_inverse_ = Qx_.inverse();    Qy_inverse_ = Qy_.inverse();


    Qu_.resize(2*sampling_n,2*sampling_n);
    Qu_.setZero();

    Qu_.block(0,0,sampling_n,sampling_n) = Qx_;
    Qu_.block(sampling_n,sampling_n,sampling_n,sampling_n) = Qy_;

    Au_.resize(2*sampling_n, 2*sampling_n);
    Au_.block(0,0,sampling_n,sampling_n) = pu_;
    Au_.block(sampling_n,sampling_n,sampling_n,sampling_n) = pu_;


//    cout<<"Obtain mpc matric at count 0"<<endl;


}
void WalkingController::calculateFootMargin(Eigen::MatrixXd& margin_x, Eigen::MatrixXd& margin_y){
    unsigned int norm_size=0;
    if(current_step_num_ >= total_step_num_ - 3)
      norm_size = (t_last_-t_start_+1)*(total_step_num_-current_step_num_)+20*hz_;
    else
      norm_size = (t_last_-t_start_+1)*3;
    if(current_step_num_ == 0)
      norm_size = norm_size + t_temp_+1;

    margin_x.resize(norm_size,2); margin_y.resize(norm_size,2);

    //width of foot is 0.172, length of foot is 0.3

    //column 0 is lower bound column 1 is upper condition
    unsigned int index =0;
    if(current_step_num_ ==0){
      //  cout<<"hellot at foot margin  "<<walking_tick_<<endl;
        for(int i=0;i<t_temp_;i++){
            margin_x(i,0) = 0;
            margin_x(i,1) = 0.15; //original 0.05

            margin_y(i,0) = -0.075;
            margin_y(i,1) = 0.075;

            index++;
        }
    }    
    if(current_step_num_ >= total_step_num_-3){
        for(unsigned int i=current_step_num_;i<total_step_num_;i++){
            if(i ==0){
                for(unsigned int j=0;j<t_total_;j++){
//                    margin_x(index+j,0) = supportfoot_support_init_offset_(0);
//                    margin_x(index+j,1) = supportfoot_support_init_offset_(0)+0.15 ;

//                    margin_y(index+j,0) = supportfoot_support_init_offset_(1) - 0.05;
//                    margin_y(index+j,1) = supportfoot_support_init_offset_(1);
                  margin_x(index+j,0) = 0.0;
                  margin_x(index+j,1) = 0.15 ;

                  margin_y(index+j,0) = - 0.05;
                  margin_y(index+j,1) = 0.0;
                }
            }
//            else if(i == total_step_num_-1){
//              for(unsigned j=0;j<t_total_;j++){
//                margin_x(index+j,0) = -0.15;
//                margin_x(index+j,1) = 0.0 ;

//                margin_y(index+j,0) = - 0.05;
//                margin_y(index+j,1) = 0.0;
//                if(foot_step_(i,6) == 1)//left foot support
//                {
//                  margin_y(index+j,0) = -0.075;
//                  margin_y(index+j,1) = 0.0;
//                }
//                else //right foot support
//                {
//                  margin_y(index+j,0) = 0.0;
//                  margin_y(index+j,1) = 0.075;
//                }
//              }
//            }
            else{
                if(i >= total_step_num_-1){
                    for(unsigned int j=0;j<t_total_;j++){
                        margin_x(index+j,0) = 0.0;
                        margin_x(index+j,1) = 0.0;
//                        margin_x(index+j,0) = foot_step_support_frame_(i-1,0);
//                        margin_x(index+j,1) = foot_step_support_frame_(i-1,0);

                        margin_y(index+j,0) = -0.075;
                        margin_y(index+j,1) = 0.075;
//                        if(foot_step_(i,6) ==1)// left foot support
//                        {
//                            margin_y(index+j,0) = -0.05;
//                            margin_y(index+j,1) = +0.05;//0.02; //done
//    //                        margin_y(index+j,0) = -0.08;//-0.08;
//    //                        margin_y(index+j,1) = -0.02;//0.0;//-0.04;//0.02;
////                            margin_y(index+j,0) = foot_step_support_frame_(i-1,1) - 0.04;
////                            margin_y(index+j,1) = foot_step_support_frame_(i-1,1) - 0.02;
//                          /*margin_y(index+j,0) = foot_step_support_frame_(i-1,1) - 0.05;
//                          margin_y(index+j,1) = foot_step_support_frame_(i-1,1) */;

//                        }
//                        else //right foot support
//                        {
////                            margin_y(index+j,0) = 0.02;//-0.02;
////                            margin_y(index+j,1) = 0.04; //done
//    //                        margin_y(index+j,0) = 0.02;//0.0;//0.04;//-0.02;
//    //                        margin_y(index+j,1) = 0.08;//0.08;
////                            margin_y(index+j,0) = foot_step_support_frame_(i-1,1) + 0.02;
////                            margin_y(index+j,1) = foot_step_support_frame_(i-1,1) + 0.04;
//                          margin_y(index+j,0) = foot_step_support_frame_(i-1,1);
//                          margin_y(index+j,1) = foot_step_support_frame_(i-1,1) + 0.05;
//                        }
                    }
                }
                else {
                    for(unsigned int j=0;j<t_total_;j++){
                        margin_x(index+j,0) = -0.15; // original 0.1
                        margin_x(index+j,1) = 0.15;
//                        margin_x(index+j,0) = foot_step_support_frame_(i-1,0) - 0.15;
//                        margin_x(index+j,1) = foot_step_support_frame_(i-1,0) + 0.15;

                        margin_y(index+j,0) = -0.075;
                        margin_y(index+j,1) = 0.075;
//                        if(foot_step_(i,6) ==1)// left foot support
//                        {
////                            margin_y(index+j,0) = -0.04;//ok margin down -0.04 magin up -0.02
////                            margin_y(index+j,1) = -0.02;//0.02;
//    //                        margin_y(index+j,0) = -0.08;//-0.08;
//    //                        margin_y(index+j,1) = -0.02;//0.0;//-0.04;//0.02;
////                            margin_y(index+j,0) = foot_step_support_frame_(i-1,1) - 0.04;
////                            margin_y(index+j,1) = foot_step_support_frame_(i-1,1) - 0.02;
//                          margin_y(index+j,0) = foot_step_support_frame_(i-1,1) - 0.05;
//                          margin_y(index+j,1) = foot_step_support_frame_(i-1,1) ;
//                        }
//                        else //right foot support
//                        {
////                            margin_y(index+j,0) = 0.02;//-0.02;
////                            margin_y(index+j,1) = 0.04;//ok margin up 0.04 magin down 0.02
//    //                        margin_y(index+j,0) = 0.08;//0.04;//-0.02;
//    //                        margin_y(index+j,1) = 0.02;//0.08;
////                            margin_y(index+j,0) = foot_step_support_frame_(i-1,1) + 0.02;
////                            margin_y(index+j,1) = foot_step_support_frame_(i-1,1) + 0.04;
//                          margin_y(index+j,0) = foot_step_support_frame_(i-1,1);
//                          margin_y(index+j,1) = foot_step_support_frame_(i-1,1) + 0.05;
//                        }
                    }
                }
            }

            index = index+t_total_;
        }
        for(unsigned int j=0;j<20*hz_;j++){
            margin_x(index+j,0) = margin_x(index-1,0);
            margin_x(index+j,1) = margin_x(index-1,1);

            margin_y(index+j,0) = margin_y(index-1,0);
            margin_y(index+j,1) = margin_y(index-1,1);
        }
        index = index+20*hz_;
    }
    else {
//        if(current_step_num_==0)
//            cout<<"hellot at foot margin 22 "<<walking_tick_<<endl;

        for(unsigned int i=current_step_num_;i<current_step_num_+3;i++){
            if(i ==0){
                for(unsigned int j=0;j<t_total_;j++){
//                    margin_x(index+j,0) = supportfoot_support_init_offset_(0);
//                    margin_x(index+j,1) = supportfoot_support_init_offset_(0)+0.15 ;

//                    margin_y(index+j,0) = supportfoot_support_init_offset_(1) - 0.05;
//                    margin_y(index+j,1) = supportfoot_support_init_offset_(1);
                  margin_x(index+j,0) = 0.0;
                  margin_x(index+j,1) = +0.15 ;

                  margin_y(index+j,0) = - 0.075;
                  margin_y(index+j,1) = 0.075;
                }
            }
            else{
                for(unsigned int j=0;j<t_total_;j++){
                    margin_x(index+j,0) = -0.15;
                    margin_x(index+j,1) = 0.15; // original 0.1
//                    margin_x(index+j,0) = foot_step_support_frame_(i-1,0) - 0.15;
//                    margin_x(index+j,1) = foot_step_support_frame_(i-1,0) + 0.15;

                    margin_y(index+j,0) = -0.075;
                    margin_y(index+j,1) = 0.075;
//                    if(foot_step_(i,6) ==1)// left foot support
//                    {
////                        margin_y(index+j,0) = -0.04; //ok margin down -0.04 magin up -0.02
////                        margin_y(index+j,1) = -0.02;//0.02;
//    //                    margin_y(index+j,0) = -0.08;
//    //                    margin_y(index+j,1) = -0.02;//0.02;
////                        margin_y(index+j,0) = foot_step_support_frame_(i-1,1) - 0.04;
////                        margin_y(index+j,1) = foot_step_support_frame_(i-1,1) - 0.02;
//                      margin_y(index+j,0) = foot_step_support_frame_(i-1,1) - 0.05;
//                      margin_y(index+j,1) = foot_step_support_frame_(i-1,1) ;
//                    }
//                    else //right foot support
//                    {
////                        margin_y(index+j,0) = 0.02;//-0.02;
////                        margin_y(index+j,1) = 0.04;//ok margin up 0.04 magin down 0.02
//    //                    margin_y(index+j,0) = 0.02;//-0.02;
//    //                    margin_y(index+j,1) = 0.08;
////                        margin_y(index+j,0) = foot_step_support_frame_(i-1,1) + 0.02;
////                        margin_y(index+j,1) = foot_step_support_frame_(i-1,1) + 0.04;
//                      margin_y(index+j,0) = foot_step_support_frame_(i-1,1) ;
//                      margin_y(index+j,1) = foot_step_support_frame_(i-1,1) + 0.05;
//                    }
                }
            }

            index = index+ t_total_;
        }
    }
}
void WalkingController::calculateFootMargin2(Eigen::MatrixXd& margin_x, Eigen::MatrixXd& margin_y){
    //divide DSP and SSP
    unsigned int norm_size=0;
    if(current_step_num_ >= total_step_num_ - 3)
      norm_size = (t_last_-t_start_+1)*(total_step_num_-current_step_num_)+20*hz_;
    else
      norm_size = (t_last_-t_start_+1)*3;
    if(current_step_num_ == 0)
      norm_size = norm_size + t_temp_+1;

    margin_x.resize(norm_size,2); margin_y.resize(norm_size,2);

    //width of foot is 0.172, length of foot is 0.3

    //column 0 is lower bound column 1 is upper condition
    unsigned int index =0;
    if(current_step_num_ ==0){
      //  cout<<"hellot at foot margin  "<<walking_tick_<<endl;
        for(int i=0;i<t_temp_;i++){
            margin_x(i,0) = 0;
            margin_x(i,1) = 0.15; //original 0.05

            margin_y(i,0) = -0.075;
            margin_y(i,1) = 0.075;

            index++;
        }
    }
    if(current_step_num_ >= total_step_num_-3){
        for(unsigned int i=current_step_num_;i<total_step_num_;i++){
            if(i ==0){
                for(unsigned int j=0;j<(t_rest_init_+t_double1_);j++){
//                    margin_x(index+j,0) = supportfoot_support_init_offset_(0);
//                    margin_x(index+j,1) = supportfoot_support_init_offset_(0)+0.15 ;

//                    margin_y(index+j,0) = supportfoot_support_init_offset_(1) - 0.05;
//                    margin_y(index+j,1) = supportfoot_support_init_offset_(1);
                  margin_x(index+j,0) = 0.0;
                  margin_x(index+j,1) = 0.10 ;

//                  margin_y(index+j,0) = - 0.05;
//                  margin_y(index+j,1) = 0.02;

//                  margin_y(index+j,0) = -0.05;
//                  margin_y(index+j,1) = 0.0;
                  margin_y(index+j,0) = -0.075;
                  margin_y(index+j,1) = 0.075;

                }
                for(unsigned int j=(t_rest_init_+t_double1_);j<(t_total_-t_double2_-t_rest_last_);j++){
                    margin_x(index+j,0) = 0.0;
                    margin_x(index+j,1) = 0.10 ;

//                    margin_y(index+j,0) = - 0.05;
//                    margin_y(index+j,1) = 0.02;

//                    margin_y(index+j,0) = -0.0;
//                    margin_y(index+j,1) = 0.0;
                    margin_y(index+j,0) = -0.075;
                    margin_y(index+j,1) = 0.075;

                }
                for(unsigned int j=(t_total_-t_double2_-t_rest_last_);j<t_total_;j++){
                    margin_x(index+j,0) = 0.0;
                    margin_x(index+j,1) = 0.15;

//                    margin_y(index+j,0) = -2*0.127794;//supportfoot_support_init_offset_(1);
//                    margin_y(index+j,1) = 0.02;

//                    margin_y(index+j,0) = -0.05;
//                    margin_y(index+j,1) = 0.0;
                    margin_y(index+j,0) = -0.075;
                    margin_y(index+j,1) = 0.075;
                }
            }
            else{
                if(i >= total_step_num_-1){
                    for(unsigned int j=0;j<(t_rest_init_+t_double1_);j++){
                        margin_x(index+j,0) = 0.0;//-foot_step_support_frame_offset_(i-1,0);
                        margin_x(index+j,1) = 0.0;
//                        if(foot_step_(i,6) == 1)//left foot support
//                        {
//                            margin_y(index+j,0) = -2*0.127794;//foot_step_support_frame_offset_(i-1,1);
//                            margin_y(index+j,1) = 0.02;
//                        }
//                        else {//right foot support
//                            margin_y(index+j,0) = -0.02;
//                            margin_y(index+j,1) = 2*0.127794;//foot_step_support_frame_offset_(i-1,1);
//                        }

                        margin_y(index+j,0) = -0.075;
                        margin_y(index+j,1) = 0.075;


                    }
                    for(unsigned int j=(t_rest_init_+t_double1_);j<(t_total_-t_double2_-t_rest_last_);j++){
                        margin_x(index+j,0) = 0.0;//-0.15;
                        margin_x(index+j,1) = 0.0;//0.15;

                        margin_y(index+j,0) = -0.075;
                        margin_y(index+j,1) = 0.075;

//                        margin_y(index+j,0) = 0.0;
//                        margin_y(index+j,1) = 0.0;

                    }
                    for(unsigned int j=(t_total_-t_double2_-t_rest_last_);j<t_total_;j++){
                        margin_x(index+j,0) = 0.0;
                        margin_x(index+j,1) = 0.0;//(foot_step_support_frame_(i-1,0) + foot_step_support_frame_(i,0))/2.0;

//                        if(foot_step_(i,6) == 1)//left foot support
//                        {
//                            margin_y(index+j,0) = -2*0.127794;//foot_step_support_frame_offset_(i-1,1);
//                            margin_y(index+j,1) = 0.02;
//                        }
//                        else {//right foot support
//                            margin_y(index+j,0) = -0.02;
//                            margin_y(index+j,1) = 2*0.127794;//foot_step_support_frame_offset_(i-1,1);
//                        }
                        margin_y(index+j,0) = -0.075;
                        margin_y(index+j,1) = 0.075;

                    }

//                    for(unsigned int j=0;j<t_total_;j++){
//                        margin_x(index+j,0) = 0.0;
//                        margin_x(index+j,1) = 0.0;
////                        margin_x(index+j,0) = foot_step_support_frame_(i-1,0);
////                        margin_x(index+j,1) = foot_step_support_frame_(i-1,0);

//                        margin_y(index+j,0) = -0.075;
//                        margin_y(index+j,1) = 0.075;

//                    }
                }
                else {
                    for(unsigned int j=0;j<(t_rest_init_+t_double1_);j++){
                        margin_x(index+j,0) = -0.15;//-foot_step_support_frame_offset_(i-1,0);
                        margin_x(index+j,1) = 0.15;//0.0;
//                        if(foot_step_(i,6) == 1)//left foot support
//                        {
//                            margin_y(index+j,0) = -2*0.127794;//foot_step_support_frame_offset_(i-1,1);
//                            margin_y(index+j,1) = 0.02;
//                        }
//                        else {//right foot support
//                            margin_y(index+j,0) = -0.02;
//                            margin_y(index+j,1) = 2*0.127794;//foot_step_support_frame_offset_(i-1,1);
//                        }
                        margin_y(index+j,0) = -0.075;
                        margin_y(index+j,1) = 0.075;

                    }
                    for(unsigned int j=(t_rest_init_+t_double1_);j<(t_total_-t_double2_-t_rest_last_);j++){
                        margin_x(index+j,0) = -0.10;
                        margin_x(index+j,1) = 0.10;

                        margin_y(index+j,0) = -0.075;
                        margin_y(index+j,1) = 0.075;

//                        margin_y(index+j,0) = -0.0;
//                        margin_y(index+j,1) = 0.0;

//                        if(foot_step_(i,6) == 1)//left foot support
//                        {
//                            margin_y(index+j,0) = -0.02;
//                            margin_y(index+j,1) = 0.05;
//                        }
//                        else {//right foot support
//                            margin_y(index+j,0) = -0.05;
//                            margin_y(index+j,1) = 0.02;
//                        }

                    }
                    for(unsigned int j=(t_total_-t_double2_-t_rest_last_);j<t_total_;j++){
                        margin_x(index+j,0) = -0.15;//0.0;
                        margin_x(index+j,1) = 0.15;//(foot_step_support_frame_(i-1,0) + foot_step_support_frame_(i,0))/2.0;

//                        if(foot_step_(i,6) == 1)//left foot support
//                        {
//                            margin_y(index+j,0) = -2*0.127794;//foot_step_support_frame_offset_(i-1,1);
//                            margin_y(index+j,1) = 0.02;
//                        }
//                        else {//right foot support
//                            margin_y(index+j,0) = -0.02;
//                            margin_y(index+j,1) = 2*0.127794;//foot_step_support_frame_offset_(i-1,1);
//                        }
                        margin_y(index+j,0) = -0.075;
                        margin_y(index+j,1) = 0.075;

                    }
                }
            }

            index = index+t_total_;
        }
        for(unsigned int j=0;j<20*hz_;j++){
            margin_x(index+j,0) = margin_x(index-1,0);
            margin_x(index+j,1) = margin_x(index-1,1);

            margin_y(index+j,0) = margin_y(index-1,0);
            margin_y(index+j,1) = margin_y(index-1,1);
        }
        index = index+20*hz_;
    }
    else {
//        if(current_step_num_==0)
//            cout<<"hellot at foot margin 22 "<<walking_tick_<<endl;

        for(unsigned int i=current_step_num_;i<current_step_num_+3;i++){
            if(i ==0){
                for(unsigned int j=0;j<t_total_;j++){
//                    margin_x(index+j,0) = supportfoot_support_init_offset_(0);
//                    margin_x(index+j,1) = supportfoot_support_init_offset_(0)+0.15 ;

//                    margin_y(index+j,0) = supportfoot_support_init_offset_(1) - 0.05;
//                    margin_y(index+j,1) = supportfoot_support_init_offset_(1);
                  margin_x(index+j,0) = 0.0;
                  margin_x(index+j,1) = +0.15 ;

                  margin_y(index+j,0) = - 0.075;
                  margin_y(index+j,1) = 0.075;
                }
            }
            else {
                for(unsigned int j=0;j<(t_rest_init_+t_double1_);j++){
                    margin_x(index+j,0) = -(foot_step_support_frame_offset_(i,0)-foot_step_support_frame_offset_(i-1,0));
                    margin_x(index+j,1) = 0.0;//0.15;//0.0;
//                    margin_x(index+j,0) = -0.15;
//                    margin_x(index+j,1) = 0.15;
//                    if(foot_step_(i,6) == 1)//left foot support
//                    {
//                        margin_y(index+j,0) = -2*0.127794;//foot_step_support_frame_offset_(i-1,1);
//                        margin_y(index+j,1) = 0.02;
//                    }
//                    else {//right foot support
//                        margin_y(index+j,0) = -0.02;
//                        margin_y(index+j,1) = 2*0.127794;//foot_step_support_frame_offset_(i-1,1);
//                    }
                    margin_y(index+j,0) = -0.075;
                    margin_y(index+j,1) = 0.075;

                }
                for(unsigned int j=(t_rest_init_+t_double1_);j<(t_total_-t_double2_-t_rest_last_);j++){
                    margin_x(index+j,0) = -0.10;
                    margin_x(index+j,1) = 0.10;

                    margin_y(index+j,0) = -0.075;
                    margin_y(index+j,1) = 0.075;
//                    margin_y(index+j,0) = -0.0;
//                    margin_y(index+j,1) = 0.0;
//                    if(foot_step_(i,6) == 1)//left foot support
//                    {
//                        margin_y(index+j,0) = -0.02;
//                        margin_y(index+j,1) = 0.05;
//                    }
//                    else {//right foot support
//                        margin_y(index+j,0) = -0.05;
//                        margin_y(index+j,1) = 0.02;
//                    }

                }
                for(unsigned int j=(t_total_-t_double2_-t_rest_last_);j<t_total_;j++){
                    margin_x(index+j,0) = -0.15;//0.0;
                    margin_x(index+j,1) = (foot_step_support_frame_offset_(i+1,0) - foot_step_support_frame_offset_(i,0))/2.0;//0.15;//(foot_step_support_frame_(i-1,0) + foot_step_support_frame_(i,0))/2.0;
//                    margin_x(index+j,0) = -0.15;
//                    margin_x(index+j,1) = 0.15;

//                    if(foot_step_(i,6) == 1)//left foot support
//                    {
//                        margin_y(index+j,0) = -2*0.127794;//foot_step_support_frame_offset_(i-1,1);
//                        margin_y(index+j,1) = 0.02;
//                    }
//                    else {//right foot support
//                        margin_y(index+j,0) = -0.02;
//                        margin_y(index+j,1) = 2*0.127794;//foot_step_support_frame_offset_(i-1,1);
//                    }
                    margin_y(index+j,0) = -0.075;
                    margin_y(index+j,1) = 0.075;

                }
            }

            index = index+ t_total_;
        }
    }
}
void WalkingController::footReferenceGenerator(Eigen::VectorXd& foot_x, Eigen::VectorXd& foot_y){
    unsigned int norm_size=0;
    if(current_step_num_ >= total_step_num_ - 3)
      norm_size = (t_last_-t_start_+1)*(total_step_num_-current_step_num_)+20*hz_;
    else
      norm_size = (t_last_-t_start_+1)*3;
    if(current_step_num_ == 0)
      norm_size = norm_size + t_temp_+1;

    foot_x.resize(norm_size);
    foot_y.resize(norm_size);


    Eigen::VectorXd temp_f_x, temp_f_y;

    unsigned int index=0;
    if(current_step_num_ ==0){
        for(int i=0;i<t_temp_;i++){
            if(i<= 0.5*hz_){
                foot_x(i) = com_support_init_(0) + com_offset_(0);
                foot_y(i) = com_support_init_(1) + com_offset_(1);
            }
            else if(i<1.0*hz_){
                foot_x(i) = 0.0;
                foot_y(i) = com_support_init_(1) + com_offset_(1);
            }
            else{
                foot_x(i) = 0.0;
                foot_y(i) = com_support_init_(1) + com_offset_(1);
            }
            index++;
        }
    }
    if(current_step_num_ >= total_step_num_-3){
        for(unsigned int i=current_step_num_;i<total_step_num_;i++){
            onestepFoot(i,temp_f_x, temp_f_y);
            if(i == total_step_num_ -1){
              for(unsigned int j=0;j<t_total_-t_double2_;j++){
                  foot_x(index+j) = temp_f_x(j);
                  foot_y(index+j) = temp_f_y(j);
              }
              index = index+t_total_-t_double2_;
            }
            else{
              for(unsigned int j=0;j<t_total_;j++){
                  foot_x(index+j) = temp_f_x(j);
                  foot_y(index+j) = temp_f_y(j);
              }
              index = index+t_total_;
            }
        }
        for(unsigned int j=0;j<20*hz_;j++){
            foot_x(index+j) = 0.0;//foot_x(index-1);
//            foot_y(index+j) = foot_y(index-1);
            foot_y(index+j) = (foot_step_support_frame_(current_step_num_,1) + foot_step_support_frame_(current_step_num_-1,1))/2;
        }

        index = index+20*hz_;
    }
    else{
        for(unsigned int i=current_step_num_;i<current_step_num_+3;i++){
            onestepFoot(i,temp_f_x, temp_f_y);
            for(unsigned int j=0;j<t_total_;j++){
                foot_x(index+j) = temp_f_x(j);
                foot_y(index+j) = temp_f_y(j);
            }

            index = index+t_total_;
        }
    }
//    if(walking_tick_==t_start_){
//        cout<<"in qpotp controoler : "<<foot_step_support_frame_offset_(current_step_num_-1,1)<<endl;
//        if(current_step_num_ == total_step_num_-1)
//            cout<<"in qp end"<<(foot_step_support_frame_(current_step_num_,1) + foot_step_support_frame_(current_step_num_-1,1))/2;
//        file[27]<<walking_tick_;
//        for(int i=0;i<norm_size;i++){
//            file[27]<<"\t"<<foot_x(i);
//        }
//        file[27]<<endl;
//        file[27]<<current_step_num_;
//        for(int i=0;i<norm_size;i++){
//            file[27]<<"\t"<<foot_y(i);
//        }
//        file[27]<<endl;

//    }

}
void WalkingController::onestepFoot(unsigned int current_step_number, Eigen::VectorXd& temp_f_x, Eigen::VectorXd& temp_f_y){
    temp_f_x.resize((int)t_total_);
    temp_f_x.setZero();

    temp_f_y.resize((int)t_total_);
    temp_f_y.setZero();

    double left_foot_offset, right_foot_offset;
    left_foot_offset = 0.04;
    right_foot_offset = -0.04;

    if(current_step_number ==0){
        for(int i=0;i<t_total_;i++)
        {
            temp_f_x(i) = supportfoot_support_init_offset_(0);
            temp_f_y(i) = supportfoot_support_init_offset_(1);
            if(foot_step_(current_step_number,6) ==1) //left foot support
                temp_f_y(i) = supportfoot_support_init_offset_(1)-0.025 + left_foot_offset;
            else {
                temp_f_y(i) = supportfoot_support_init_offset_(1)+0.025 + right_foot_offset; //ok
            }
        }
    }
//    else if(current_step_number == total_step_num_-1){
//      for(int i=0;i<t_total_;i++){
//        temp_f_x(i) = foot_step_support_frame_offset_(current_step_number-1,0);
//        temp_f_y(i) = foot_step_support_frame_offset_(current_step_number-1,1);

//        if(foot_step_(current_step_number,6) ==1) //left foot support
//            temp_f_y(i) =foot_step_support_frame_offset_(current_step_number-1,1)+0.10;//-0.025;
//        else {
//            temp_f_y(i) = foot_step_support_frame_offset_(current_step_number-1,1)-0.10;//+0.025; // ok
//        }
//      }
//    }
    else if(current_step_number ==1){
      for(int i=0;i<t_total_;i++){
          temp_f_x(i) = foot_step_support_frame_offset_(current_step_number-1,0);
          temp_f_y(i) = foot_step_support_frame_offset_(current_step_number-1,1);
          if(foot_step_(current_step_number,6) ==1) //left foot support
              temp_f_y(i) =foot_step_support_frame_offset_(current_step_number-1,1)+0.025 + left_foot_offset;//-0.025;
          else {
              temp_f_y(i) = foot_step_support_frame_offset_(current_step_number-1,1)-0.025 + right_foot_offset;//+0.025; // ok
          }
      }
    }
    else if(current_step_number ==total_step_num_-1){
        for(int i=0;i<t_total_;i++){
            temp_f_x(i) = foot_step_support_frame_offset_(current_step_number-1,0);
            temp_f_y(i) = foot_step_support_frame_offset_(current_step_number-1,1);
            if(foot_step_(current_step_number,6) ==1) //left foot support
                temp_f_y(i) =foot_step_support_frame_offset_(current_step_number-1,1)+0.015 + left_foot_offset;//-0.025;
            else {
                temp_f_y(i) = foot_step_support_frame_offset_(current_step_number-1,1)-0.015 + right_foot_offset;//+0.025; // ok
            }
        }
    }
    else{
        for(int i=0;i<t_total_;i++){
            temp_f_x(i) = foot_step_support_frame_offset_(current_step_number-1,0);
            temp_f_y(i) = foot_step_support_frame_offset_(current_step_number-1,1);
            if(foot_step_(current_step_number,6) ==1) //left foot support
                temp_f_y(i) =foot_step_support_frame_offset_(current_step_number-1,1)+0.06+left_foot_offset;//-0.025;
            else {
                temp_f_y(i) = foot_step_support_frame_offset_(current_step_number-1,1)-0.06+right_foot_offset;//+0.025; // ok
            }
        }
    }

}
void WalkingController::ObtainMatrix(int NL, int N, double dt, int interval){
    Eigen::Matrix3d A;
    A.setIdentity();
    A(0,1) = dt; A(0,2) = pow(dt,2)/2.0;
    A(1,2) = dt;
    A_ = A;
    Eigen::Vector3d b;
    b(0) = pow(dt,3)/6.0;
    b(1) = pow(dt,2)/2.0;
    b(2) = dt;
    b1_ = b;
    Eigen::Matrix<double, 1, 3>c;
    c(0) = 1.0; c(1) = 0.0; c(2) = -zc_/GRAVITY;

    Eigen::MatrixXd px_full(NL,3);
    Eigen::MatrixXd pu_full(NL,NL);
    pu_full.setZero();

    //px matrix full size ///
    for(int i=0;i<NL;i++){
        px_full(i,0) = 1.0;
        px_full(i,1) = (i+1)*dt;
        px_full(i,2) = (i+1)*(i+1)*dt*dt*0.5-zc_/GRAVITY;
    }
     // pu matrix full size
    for(int i=0;i<NL;i++){
        for(int j=0;j<= i;j++){
            int nl = i - j;
            pu_full(i,j) = (1+3*nl+3*nl*nl)*pow(dt,3)/6.0 - dt*zc_/GRAVITY;
        }
    }

    //Eigen::MatrixXd Px;


    Px_.resize(N,3);
    Py_.resize(N,3);
    for(int i=0;i<N;i++){
        Px_(i,0) = 1.0;
        Px_(i,1) = (interval*i+1)*dt;
        Px_(i,2) = (interval*i+1)*(interval*i+1)*dt*dt*0.5-zc_/GRAVITY;
    }


//    for(int i=0;i<N;i++){
//        Px_.row(i) = px_full.row(i*interval);
//    }
    Py_ = Px_;

    Eigen::MatrixXd Pu;
    Pu.resize(N,N);
    Pu.setZero();

    for(int i=0;i<N;i++){
        for(int j=0;j<= i;j++){
            int nl = interval*i + 1-j;
            Pu(i,j) = (1+3*nl+3*nl*nl)*pow(dt,3)/6.0 - dt*zc_/GRAVITY;
        }
    }
//    for(int i=0;i<N;i++){
//        Pu.row(i) = pu_full.row(interval*i);
//    }
    pu_t_.resize(N,N);
    pu_t_ = Pu.transpose();

    double ratio=0.000001;
    Eigen::MatrixXd Iden_nl;
    Iden_nl.resize(N,N);
    Iden_nl.setIdentity();

    Eigen::MatrixXd p_temp1, p_temp2;
    p_temp1.resize(N,N); p_temp2.resize(N,N);

    Eigen::MatrixXd pu_transpose;
    pu_transpose.resize(N,N);
    pu_transpose =Pu.transpose();

    pu_.resize(N,N);
    pu_ = Pu;
    Eigen::MatrixXd pu_h;
    pu_h.resize(N,N);
    pu_h = Pu.transpose()*Pu + ratio*Iden_nl;

    p_temp1 = pu_h.inverse();
    p_temp2 = p_temp1*pu_transpose;

    p_temp_.resize(N,N);
    p_temp_ = p_temp2;

    Eigen::MatrixXd pu_t_pu;
    pu_t_pu.resize(N,N);
    pu_t_pu.setZero();

    pu_t_pu = pu_transpose*Pu;

    Eigen::MatrixXd bk_full(3*NL,NL);
    bk_full.setZero();
    for(int i=0;i<NL;i++){
        for(int j=0;j<=i;j++){
            int n=i-j;
            bk_full(3*i,j) = (1+3*n*(n+1))/6.0 * pow(dt,3);
            bk_full(3*i+1,j) = (2*n+1)/2.0 *pow(dt,2);
            bk_full(3*i+2,j) = dt;
        }
    }

    bk_.resize(3*N,N);
    bk_.setZero();
    for(int i=0;i<N;i++){
        for(int j=0;j<=i;j++){
            int n = i-j;
         bk_(3*i,j) = (1+3*n*(n+1))/6.0 * pow(dt,3);
         bk_(3*i+1,j) = (2*n+1)/2.0 *pow(dt,2);
         bk_(3*i+2,j) = dt;
        }
    }
//    for(int i=0;i<N;i++){
//        bk_.row(i) = bk_full.row(i*interval);
//    }

//    for(int i=0;i<3*N;i++){
//        for(int j=0;j<N;j++)
//            file[17]<<bk_(i,j)<<"\t";

//        file[17]<<endl;
//    }


    Eigen::MatrixXd Ak_full(3*NL,3);
    for(int i=0;i<NL;i++){
        Ak_full(3*i,0) = 1.0;
        Ak_full(3*i,1) = (i+1)*dt;
        Ak_full(3*i,2) = (i+1)*(i+1)/2.0 * pow(dt,2);

        Ak_full(3*i+1,1) = 1.0;
        Ak_full(3*i+1,2) =(i+1)*dt;

        Ak_full(3*i+2,2) = 1.0;
    }
    Ak_.resize(3*N,3);
    Ak_.setZero();
    for(int i=0;i<N;i++){
        Ak_(3*i,0) = 1.0;
        Ak_(3*i,1) = (i+1)*dt;
        Ak_(3*i,2) = (i+1)*(i+1)/2.0 * pow(dt,2);

        Ak_(3*i+1,1) = 1.0;
        Ak_(3*i+1,2) =(i+1)*dt;

        Ak_(3*i+2,2) = 1.0;
    }
//    for(int i=0;i<N;i++){
//        Ak_.row(i) = Ak_full.row(i*interval);
//    }
//    for(int i=0;i<3*N;i++){
//        for(int j=0;j<3;j++)
//            file[16]<<Ak_(i,j)<<"\t";

//        file[16]<<endl;
//    }


//    if(walking_tick_ ==0){
//        file[14]<<walking_tick_;
//        file[24]<<walking_tick_;
//        for(int i=0;i<N;i++){
////            file[14]<<"\t"<<px_full(i,0)<<"\t"<<px_full(i,1)<<"\t"<<px_full(i,2)<<endl;
//            file[14]<<"\t"<<Px_(i,0)<<"\t"<<Px_(i,1)<<"\t"<<Px_(i,2)<<endl;
//            for(int j=0;j<N;j++){
////                file[15]<<pu_full(i,j)<<"\t";
//                file[15]<<pu_t_(i,j)<<"\t";
//            }
//            file[15]<<endl;
//        }
//        //file[16]<<walking_tick_;
//        for(int i=0;i<3*NL;i++){
//            for(int j=0;j<NL;j++){
//          //      file[16]<<"\t"<<bk_full(i,j);
//            }
//            file[24]<<"\t"<<Ak_full(i,0)<<"\t"<<Ak_full(i,1)<<"\t"<<Ak_full(i,2)<<endl;
//            //file[16]<<endl;
//        }

//    }

    coef_inverse_.resize(N,N);
    Eigen::MatrixXd coef_temp;
    coef_temp.resize(N,N);

    double alp = 0.1;

    Eigen::MatrixXd selec;
    selec.resize(N,3*N);
    selec.setZero();

    Eigen::Matrix<double, 1, 3> selec_velocity;
    selec_velocity.setZero();
    selec_velocity(0,1) = 1.0;

    for(int i=0;i<N;i++)
        selec.block<1,3>(i,3*i) = selec_velocity;

    Eigen::MatrixXd temp_bk_s_t, temp_s_bk;
    temp_bk_s_t.resize(N,N); temp_s_bk.resize(N,N);

    temp_bk_s_t = bk_.transpose()*selec.transpose();
    temp_s_bk = selec*bk_;

    Eigen::MatrixXd temp_bk_s_s_bk;
    temp_bk_s_s_bk.resize(N,N);
    temp_bk_s_s_bk = temp_bk_s_t*temp_s_bk;

    coef_temp = pu_h + alp*temp_bk_s_s_bk;

    coef_inverse_ = coef_temp.inverse();

    coef_x_.resize(N,3*N);
    coef_x_ = alp*temp_bk_s_t*selec *Ak_;

    // for using delta_velocity
    Av_.resize(N,3);
    Av_.setZero();

    bv_.resize(N,N);
    bv_.setZero();

    Av0_.resize(N,3);
    Av0_.setZero();

    Av_ = selec*Ak_;
    bv_ = selec*bk_;

    Av0_(0,1) = 1.0;

    Selec_delta_.resize(N,N);
    Selec_delta_.setZero();
    Eigen::Matrix<double, 1, 2> del_vel;
    del_vel(0,0) = -1.0; del_vel(0,1) = 1.0;

    Selec_delta_(0,0) = 1.0;

    for(int i=1;i<N;i++)
        Selec_delta_.block<1,2>(i,i-1) =del_vel;

    Qx_inverse_.resize(N,N);    Qx_inverse_.setZero();
    Qy_inverse_.resize(N,N);    Qy_inverse_.setZero();

    beta_x_ = 1.0;//0.000001;
    beta_y_ = 0.000001;//0.000001;//0.000001;
    Eigen::MatrixXd sdelta_bv,Qx_temp, Q_v, Qy_temp;
    sdelta_bv.resize(N,N); bvt_sdeltat_.resize(N,N); Qx_temp.resize(N,N);Qy_temp.resize(N,N); Q_v.resize(N,N);

    sdelta_bv = Selec_delta_*bv_;
    bvt_sdeltat_ = sdelta_bv.transpose();

    selec_bv_.resize(N,N);
    selec_bv_.setZero();
    selec_bv_ = sdelta_bv;

    Q_v = bvt_sdeltat_*sdelta_bv;
    Qx_temp = pu_h + beta_x_*Q_v;
    //Qx_inverse_ = Qx_temp.inverse();
    Qx_inverse_ = p_temp1.inverse();

    Qy_temp = pu_h + beta_y_*Q_v;
    //Qy_inverse_ = Qy_temp.inverse();
    Qy_inverse_ = p_temp1.inverse();


    selec_delta_v_.resize(N,3);
    selec_delta_v_ = Selec_delta_*Av_ - Av0_;

    Velocity_delta_x_.resize(N,3);    Velocity_delta_y_.resize(N,3);

    Velocity_delta_x_ = beta_x_*bvt_sdeltat_*selec_delta_v_;
    Velocity_delta_y_ = beta_y_*bvt_sdeltat_*selec_delta_v_;


    ///// for using qp oases////


    alpha_x_ = 1.0; gamma_x_ = 0.0002; // alpha 1.0 beta 1.0 gamma 0.0002 for x :::: for y
    alpha_y_ = 1.0; gamma_y_ = 0.000001;//001; // :::: for y alpha 1.0 beta 0.000001 gamma 0.000001
    Qx_.resize(N,N);    Qx_.setZero();
    Qy_.resize(N,N);    Qy_.setZero();

    Qx_ = alpha_x_*pu_t_pu + beta_x_*Q_v + gamma_x_ * Iden_nl;
    Qy_ = alpha_y_*pu_t_pu + beta_y_*Q_v + gamma_y_ * Iden_nl;

//    for(int i=0;i<N;i++){
//        for(int j=0;j<3;j++)
//            file[27]<<Av_(i,j)<<"\t";
//        file[27]<<endl;
//    }
//    file[27]<<endl;

    //////////////////////// for using velocity variance /////////////////
    Av_variance_.resize(N,3);
    bv_variance_.resize(N,N);

    Eigen::MatrixXd selec_variance, Ones_nl;
    selec_variance.resize(N,N); Ones_nl.resize(N,N);
    Ones_nl.setOnes();
    selec_variance = Iden_nl-Ones_nl/N;
    Av_variance_ = selec_variance*Av_;
    bv_variance_ = selec_variance*bv_;

    Eigen::MatrixXd b_var_t_b_var;
    b_var_t_b_var.resize(N,N);
    b_var_t_b_var = bv_variance_.transpose()*bv_variance_;

    Qx_variance_.resize(N,N);
    Qx_variance_inverse_.resize(N,N);

    Qx_variance_ = alpha_x_*pu_t_pu + beta_x_*b_var_t_b_var/N +gamma_x_ *Iden_nl;
    Qx_variance_inverse_ = Qx_variance_.inverse();

    Qy_variance_.resize(N,N);
    Qy_variance_inverse_.resize(N,N);

    Qy_variance_ = alpha_y_*pu_t_pu + beta_y_*b_var_t_b_var/N +gamma_y_ *Iden_nl;
    Qy_variance_inverse_ = Qy_variance_.inverse();

}
void WalkingController::SupportfootComUpdate(Eigen::Vector3d x_p1,Eigen::Vector3d y_p1, Eigen::Vector3d& New_x_p1, Eigen::Vector3d& New_y_p1){

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

      //file[26]<<walking_tick_<<"\t"<<x_1_(0)<<"\t"<<x_1_(1)<<"\t"<<x_1_(2)<<endl;

      com_pos_prev(0) = x_p1(0);
      com_pos_prev(1) = y_p1(0);
      com_pos = temp_rot*(com_pos_prev - temp_pos);

      com_vel_prev(0) = x_p1(1);
      com_vel_prev(1) = y_p1(1);
      com_vel_prev(2) = 0.0;
      com_vel = temp_rot*com_vel_prev;

      com_acc_prev(0) = x_p1(2);
      com_acc_prev(1) = y_p1(2);
      com_acc_prev(2) = 0.0;
      com_acc = temp_rot*com_acc_prev;

      New_x_p1(0) = com_pos(0);
      New_x_p1(1) = com_vel(0);
      New_x_p1(2) = com_acc(0);

//      New_y_p1(0) = com_pos(1);
//      New_y_p1(1) = com_vel(1);
//      New_y_p1(2) = com_acc(1);

      //file[27]<<walking_tick_<<"\t"<<x_1_(0)<<"\t"<<x_1_(1)<<"\t"<<x_1_(2)<<endl;
    }
}
void WalkingController::GetSupportPolygon(Eigen::MatrixXd& sp_x, Eigen::MatrixXd& sp_y){

    unsigned int n_size;

    unsigned int step_number =3;


    if(current_step_num_ >= total_step_num_ - step_number)
      n_size = (t_last_-t_start_+1)*(total_step_num_-current_step_num_)+20*hz_;
    else
      n_size = (t_last_-t_start_+1)*(step_number);
    if(current_step_num_ == 0)
      n_size = n_size + t_temp_+1;


    sp_x.resize(n_size,2);    sp_y.resize(n_size,2);
    sp_x.setZero();    sp_y.setZero();

    Eigen::MatrixXd tempx, tempy;

    unsigned int index =0;
    if(current_step_num_ ==0){
        for(int i=0;i<t_temp_;i++){
            if(i<=0/5*hz_){
                sp_x(i,1) = com_support_init_(0) + com_offset_(0);
                sp_x(i,0) = 0.0;

                sp_y(i,0) = com_support_init_(1)+com_offset_(1);
                sp_y(i,1) = com_support_init_(1)+com_offset_(1);
            }
            else if(i<1.5*hz_){
                double delx = i-0.5*hz_;
                sp_x(i,1) = com_support_init_(0) + com_offset_(0) - delx*(com_support_init_(0) + com_offset_(0))/(1.0*hz_);
                sp_x(i,0) = 0.0;

                sp_y(i,0) = com_support_init_(1)+com_offset_(1);
                sp_y(i,1) = com_support_init_(1)+com_offset_(1);
            }
            else {
                sp_x(i,0) = 0.0;
                sp_x(i,1) = 0.0;

                sp_y(i,0) = com_support_init_(1)+com_offset_(1);
                sp_y(i,1) = com_support_init_(1)+com_offset_(1);
            }
            index ++;
        }
    }

    if(current_step_num_ >=total_step_num_-step_number){
        for(unsigned int i=current_step_num_;i<total_step_num_;i++){
            onestepSupportPolygon(i,tempx,tempy);
            for(unsigned int j=0;j<t_total_;j++){
                sp_x(index+j,0) = tempx(j,0);
                sp_x(index+j,1) = tempx(j,1);

                sp_y(index+j,0) = tempy(j,0);
                sp_y(index+j,1) = tempy(j,1);
            }
            index = index+t_total_;
        }
        for(unsigned int j=0;j<20*hz_;j++){
            sp_x(index+j,0) = sp_x(index-1,0);
            sp_x(index+j,1) = sp_x(index-1,1);

            sp_y(index+j,0) = sp_y(index-1,0);
            sp_y(index+j,1) = sp_y(index-1,1);
        }
        index = index+20*hz_;
    }
    else{
        for(unsigned int i=current_step_num_;i<current_step_num_+ step_number;i++){
            onestepSupportPolygon(i,tempx,tempy);
            for(unsigned int j=0;j<t_total_;j++){
                sp_x(index+j,0) = tempx(j,0);
                sp_x(index+j,1) = tempx(j,1);

                sp_y(index+j,0) = tempy(j,0);
                sp_y(index+j,1) = tempy(j,1);
            }
            index = index+t_total_;
        }
    }
//    if(walking_tick_==t_start_ || walking_tick_ ==0){
//        if(current_step_num_ ==0){
//            for(int i=0;i<n_size;i++)
//                file[18]<<walking_tick_<<"\t"<<current_step_num_<<"\t"<<sp_x(i,0)<<"\t"<<sp_x(i,1)<<"\t"<<sp_y(i,0)<<"\t"<<sp_y(i,1)<<endl;
//        }
//        else if(current_step_num_ ==1){
//            for(int i=0;i<n_size;i++)
//                file[19]<<walking_tick_<<"\t"<<current_step_num_<<"\t"<<sp_x(i,0)<<"\t"<<sp_x(i,1)<<"\t"<<sp_y(i,0)<<"\t"<<sp_y(i,1)<<endl;
//        }
//        else if(current_step_num_ ==2){
//            for(int i=0;i<n_size;i++)
//                file[20]<<walking_tick_<<"\t"<<current_step_num_<<"\t"<<sp_x(i,0)<<"\t"<<sp_x(i,1)<<"\t"<<sp_y(i,0)<<"\t"<<sp_y(i,1)<<endl;
//        }
//        else if(current_step_num_ ==3){
//            for(int i=0;i<n_size;i++)
//                file[21]<<walking_tick_<<"\t"<<current_step_num_<<"\t"<<sp_x(i,0)<<"\t"<<sp_x(i,1)<<"\t"<<sp_y(i,0)<<"\t"<<sp_y(i,1)<<endl;
//        }

//    }
}
void WalkingController::onestepSupportPolygon(unsigned int current_step_number, Eigen::MatrixXd& tempx, Eigen::MatrixXd& tempy){
    tempx.resize(t_total_,2);
    tempy.resize(t_total_,2);

    double x_length, y_length;
    x_length = 0.15; y_length = 0.1;

    if(current_step_number ==0){
        for(int i=0;i<t_total_;i++){
            if( i < t_rest_init_){
                tempx(i,0) = rfoot_support_init_.translation()(0) - x_length;
                tempx(i,1) = lfoot_support_init_.translation()(0) + x_length;;

                tempy(i,0) = rfoot_support_init_.translation()(1) -y_length;
                tempy(i,1) = lfoot_support_init_.translation()(1)+y_length;
            }
            else if (i>= t_rest_init_ && i <t_rest_init_+t_double1_) {
                tempx(i,0) = supportfoot_support_init_offset_(0) -x_length;;
                tempx(i,1) = supportfoot_support_init_offset_(0)+x_length;;

                tempy(i,0) = rfoot_support_init_.translation()(1)-y_length;
                tempy(i,1) = lfoot_support_init_.translation()(1)+y_length;
            }
            else if (i>= t_rest_init_+t_double1_ && i< t_total_-t_rest_last_-t_double2_) {
                tempx(i,0) = supportfoot_support_init_offset_(0)-x_length;;
                tempx(i,1) = supportfoot_support_init_offset_(0)+x_length;;

//                tempy(i,0) = supportfoot_support_init_offset_(1)-y_length;
//                tempy(i,1) = supportfoot_support_init_offset_(1)+y_length;
                tempy(i,0) = lfoot_support_init_.translation()(1)-y_length;
                tempy(i,1) = lfoot_support_init_.translation()(1)+y_length;
            }
            else{
                tempx(i,0) = supportfoot_support_init_offset_(0)-x_length;
                tempx(i,1) = foot_step_support_frame_(current_step_number,0) + x_length;
               // tempx(i,1) = (supportfoot_support_init_offset_(0) + foot_step_support_frame_(current_step_number,0))/2.0;

                //tempx(i,0) = (supportfoot_support_init_offset_(1) + foot_step_support_frame_(current_step_number,1))/2.0;
                tempy(i,0) = foot_step_support_frame_(current_step_number,1) - y_length;
                tempy(i,1) = lfoot_support_init_.translation()(1)+y_length;//supportfoot_support_init_offset_(1)+y_length;
            }
        }
    }
    else if(current_step_number==1){
        for(int i=0;i<t_total_;i++){
            if(i<t_rest_init_ + t_double1_){
                //tempx(i,0) = (supportfoot_support_init_(0)+foot_step_support_frame_(current_step_number-1,0))/2.0;
                tempx(i,0) = supportfoot_support_init_offset_(0) - x_length;
                tempx(i,1) = foot_step_support_frame_offset_(current_step_number-1,0) + x_length;

                if(foot_step_support_frame_(current_step_number-1,6)==0){//left foot swing , right foot support)
                    //tempy(i,1) = (supportfoot_support_init_offset_(1) + foot_step_support_frame_(current_step_number-1,1))/2.0;
                    tempy(i,1) = supportfoot_support_init_offset_(1) + y_length;
                    tempy(i,0) = foot_step_support_frame_offset_(current_step_number-1,1)-y_length;
                }
                else{
                    //tempy(i,0) = (supportfoot_support_init_offset_(1) + foot_step_support_frame_(current_step_number-1,1))/2.0;
                    tempy(i,0) = supportfoot_support_init_offset_(1) - y_length;
                    tempy(i,1) = foot_step_support_frame_offset_(current_step_number-1,1)+y_length;
                }
            }
            else if(i>= t_rest_init_ + t_double1_ && i<t_total_-t_rest_last_-t_double2_){

                tempx(i,0) = foot_step_support_frame_offset_(current_step_number-1,0)-x_length;
                tempx(i,1) = foot_step_support_frame_offset_(current_step_number-1,0)+x_length;

                tempy(i,0) = foot_step_support_frame_offset_(current_step_number-1,1)-y_length;
                tempy(i,1) = foot_step_support_frame_offset_(current_step_number-1,1)+y_length;

            }
            else {
                tempx(i,0) = foot_step_support_frame_offset_(current_step_number-1,0)-x_length;
                //tempx(i,1) = (foot_step_support_frame_(current_step_number-1,0)+foot_step_support_frame_(current_step_number,0))/2.0;
                tempx(i,1) = foot_step_support_frame_(current_step_number,0) + x_length;

                if(foot_step_support_frame_(current_step_number-1,6)==0){//left foot swing , right foot support)
                    tempy(i,0) = foot_step_support_frame_offset_(current_step_number-1,1) - y_length;
                    //tempy(i,1) = (foot_step_support_frame_(current_step_number,1)+foot_step_support_frame_(current_step_number-1,1))/2.0;
                    tempy(i,1) = foot_step_support_frame_(current_step_number,1) +y_length;
                }
                else{//right foot swing left foot support
                    //tempy(i,0) = (foot_step_support_frame_(current_step_number,1)+foot_step_support_frame_(current_step_number-1,1))/2.0;
                    tempy(i,0) = foot_step_support_frame_(current_step_number,1) - y_length;
                    tempy(i,1) = foot_step_support_frame_offset_(current_step_number-1,1) + y_length;
                }
            }
        }
    } // end of else if(current_step_number ==1)
    else{
        for(int i=0;i<t_total_;i++){
            if(i<t_rest_init_ + t_double1_){
                //tempx(i,0) = (foot_step_support_frame_(current_step_number-2,0)+foot_step_support_frame_(current_step_number-1,0))/2.0;
                tempx(i,0) = foot_step_support_frame_(current_step_number-2,0) - x_length;
                tempx(i,1) = foot_step_support_frame_offset_(current_step_number-1,0) + x_length;

                if(foot_step_support_frame_(current_step_number-1,6)==0){//left foot swing , right foot support)
                    tempy(i,1) = foot_step_support_frame_(current_step_number-2,1)+y_length;
                    tempy(i,0) = foot_step_support_frame_offset_(current_step_number-1,1)-y_length;
                }
                else{
                    //tempy(i,0) = (supportfoot_support_init_offset_(1) + foot_step_support_frame_(current_step_number-1,1))/2.0;
                    tempy(i,0) = foot_step_support_frame_(current_step_number-2,1) - y_length;
                    tempy(i,1) = foot_step_support_frame_offset_(current_step_number-1,1)+y_length;
                }
            }
            else if(i>= t_rest_init_ + t_double1_ && i<t_total_-t_rest_last_-t_double2_){

                tempx(i,0) = foot_step_support_frame_offset_(current_step_number-1,0)-x_length;
                tempx(i,1) = foot_step_support_frame_offset_(current_step_number-1,0)+x_length;

                tempy(i,0) = foot_step_support_frame_offset_(current_step_number-1,1)-y_length;
                tempy(i,1) = foot_step_support_frame_offset_(current_step_number-1,1)+y_length;

            }
            else {
                tempx(i,0) = foot_step_support_frame_offset_(current_step_number-1,0)-x_length;
                //tempx(i,1) = (foot_step_support_frame_(current_step_number-1,0)+foot_step_support_frame_(current_step_number,0))/2.0;
                tempx(i,1) = foot_step_support_frame_(current_step_number,0) + x_length;

                if(foot_step_support_frame_(current_step_number-1,6)==0){//left foot swing , right foot support)
                    tempy(i,0) = foot_step_support_frame_offset_(current_step_number-1,1) - y_length;
                    //tempy(i,1) = (foot_step_support_frame_(current_step_number,1)+foot_step_support_frame_(current_step_number-1,1))/2.0;
                    tempy(i,1) = foot_step_support_frame_(current_step_number,1) + y_length;
                }
                else{//right foot swing left foot support
                    //tempy(i,0) = (foot_step_support_frame_(current_step_number,1)+foot_step_support_frame_(current_step_number-1,1))/2.0;
                    tempy(i,0) = foot_step_support_frame_(current_step_number,1) -  y_length;
                    tempy(i,1) = foot_step_support_frame_offset_(current_step_number-1,1) + y_length;
                }
            }
        }
    }// end of else
}
void WalkingController::Obtain_com_pattern(ifstream& input_x,ifstream& input_y, ifstream& foot_input, Eigen::MatrixXd& com_x,Eigen::MatrixXd& com_y, Eigen::MatrixXd& input_foot ){
    com_x.resize(2157,13);
    com_x.setZero();

    com_y.resize(2545,7);
    com_y.setZero();
//    com_x.resize(2801,9);
//    com_x.setZero();

    input_foot.resize(2958,13);
    input_foot.setZero();

    cout<<"obatin pattern : "<<endl;

    for(int i=0;i<2958;i++){
        input_x >> com_x(i,0) >> com_x(i,1) >> com_x(i,2)>> com_x(i,3) >> com_x(i,4) >> com_x(i,5)>>com_x(i,6)>>com_x(i,7) >> com_x(i,8)>> com_x(i,9) >> com_x(i,10)>>com_x(i,11) >> com_x(i,12);// >> com_x(i,13)>>com_x(i,14);
        file[17]<<com_x(i,0)<<"\t"<<com_x(i,1)<<"\t"<<com_x(i,2)<<"\t"<<com_x(i,3)<<"\t"<<com_x(i,4)<<"\t"<<com_x(i,5)<<"\t"<<com_x(i,6)<<"\t"<<com_x(i,7)<<"\t"<<com_x(i,8)<<"\t"<<com_x(i,9)<<"\t"<<com_x(i,10)<<"\t"<<com_x(i,11)<<endl;
        //file[16]<<com_x(i,0)<<"\t"<<com_x(i,1)<<"\t"<<com_x(i,2)<<"\t"<<com_x(i,3)<<"\t"<<com_x(i,4)<<"\t"<<com_x(i,5)<<"\t"<<com_x(i,6)<<"\t"<<com_x(i,7)<<"\t"<<com_x(i,8)<<endl;

//        if(i == 804){
//            cout<<i<<"th value " <<endl;
//            for(int j=0;j<11;j++)
//                cout<<com_x(i,j)<<", ";

//            cout<<endl;
//        }
    }



    for(int i=0;i<2545;i++){
        input_y >> com_y(i,0) >> com_y(i,1) >> com_y(i,2)>> com_y(i,3) >> com_y(i,4) >> com_y(i,5)>>com_y(i,6);//com_y(i,7) >> com_y(i,8) >> com_y(i,9) >> com_y(i,10)>> com_y(i,11) >> com_y(i,12) >> com_y(i,13)>>com_y(i,14);

    }

//    for(int i=0;i<2801;i++){
//        input >> com_x(i,0) >> com_x(i,1) >> com_x(i,2)>> com_x(i,3) >> com_x(i,4) >> com_x(i,5)>>com_x(i,6)>>com_x(i,7) >> com_x(i,8);
//        file[17]<<com_x(i,0)<<"\t"<<com_x(i,1)<<"\t"<<com_x(i,2)<<endl;
//    }


    for(int i=0;i<2958;i++){
        foot_input >> input_foot(i,0) >> input_foot(i,1) >> input_foot(i,2)>> input_foot(i,3) >> input_foot(i,4) >> input_foot(i,5)>>input_foot(i,6)>>input_foot(i,7) >> input_foot(i,8) >> input_foot(i,9) >> input_foot(i,10)>> input_foot(i,11) >> input_foot(i,12);
        file[16]<<input_foot(i,0)<<"\t"<<input_foot(i,1)<<"\t"<<input_foot(i,2)<<"\t"<<input_foot(i,3)<<"\t"<<input_foot(i,4)<<"\t"<<input_foot(i,5)<<"\t"<<input_foot(i,6)<<"\t"<<input_foot(i,7)<<"\t"<<input_foot(i,8)<<"\t"<<input_foot(i,9)<<"\t"<<input_foot(i,10)<<"\t"<<input_foot(i,11)<<"\t"<<input_foot(i,12)<<endl;
    }
}

//void WalkingController::qp1(){
//    if(walking_tick_ ==0 )
//        cout<<"qp 1 test "<<endl;

//    int NL = 100;
//    double dt = 1.0/hz_;

//    Eigen::VectorXd p_ref;
//    p_ref.resize(NL);
//    double start_time;
//    if(current_step_num_ == 0)
//      start_time = 0;
//    else
//      start_time = t_start_;

//    for(int i=0;i<NL;i++)
//        p_ref(i) = ref_zmp_(walking_tick_ - start_time + i,0);


//    Eigen::Matrix3d a;
//    a.setIdentity();
//    a(0,1) = dt; a(0,2) = pow(dt,2)/2.0;
//    a(1,2) = dt;

//    Eigen::Matrix<double, 3, 1> b;
//    b(0,0) = pow(dt,3)/6.0; b(1,0) = pow(dt,2)/2.0; b(2,0) = dt;

//    //Eigen::Matrix3d::Identity()

//    Eigen::Matrix<double, 1,4> c;
//    c(0,0) = 0.0; c(0,1) = 1; c(0,2) = 0; c(0,3) = -zc_/GRAVITY;

//    Eigen::MatrixXd A_temp;
//    A_temp.resize(3*(NL-1),4*NL);
//    A_temp.setZero();
//    A_temp.block<3,1>(0,0) = -b;
//    A_temp.block<3,3>(0,1) = a+Eigen::Matrix3d::Identity();
//    A_temp.block<3,1>(0,4) = b;
//    A_temp.block<3,3>(0,5) = -Eigen::Matrix3d::Identity();

//    for(int i=1;i<NL-1;i++){
//        A_temp.block<3,3>(3*i,4*(i-1)+1) = -a;
//        A_temp.block<3,1>(3*i,4*i) = -b;
//        A_temp.block<3,3>(3*i,4*i+1) = a+Eigen::Matrix3d::Identity();
//        A_temp.block<3,1>(3*i,4*(i+1)) = b;
//        A_temp.block<3,3>(3*i,4*(i+1)+1) = -Eigen::Matrix3d::Identity();
//    }



////    if(walking_tick_ == 0){
////        cout<<"saving data : "<<endl;
////        for(int i=0;i<3*(NL-1);i++){
////            for(int j=0;j<4*NL;j++){
////                file[16]<<A_temp(i,j)<<"\t";
////            }
////            file[16]<<endl;
////        }
////    }

//    Eigen::MatrixXd c_temp;
//    c_temp.resize(NL,4*NL);

//    for(int i=0;i<NL;i++){
//        c_temp.block<1,4>(i,4*i) = c;
//    }

//    Eigen::Matrix<double, 1, 8> s;
//    s.setZero();
//    s(0,0) = -1.0; s(0,4) = 1.0;

//    Eigen::MatrixXd Selec_s;
//    Selec_s.resize(NL,4*NL);
//    Selec_s(0,0) = 1.0;
//    for(int i=1;i<NL;i++){
//        Selec_s.block<1,8>(i,4*(i-1)) = s;
//    }

////    if(walking_tick_ == 0){
////        cout<<"saving data : "<<endl;
////        for(int i=0;i<NL;i++){
////            for(int j=0;j<4*NL;j++){
////                file[16]<<Selec_s(i,j)<<"\t";
////            }
////            file[16]<<endl;
////        }
////    }

//    double r = 0.0000001;
//    Eigen::MatrixXd h_temp;
//    h_temp.resize(4*NL,4*NL);

//    Eigen::MatrixXd Q;
//    Q.resize(4*NL,4*NL);
//    Q.setIdentity();
//    h_temp = c_temp.transpose()*Q*c_temp + Selec_s.transpose() *r*Selec_s;


//    if(walking_tick_ ==0){

//    }

//    int FNL = 4*NL, NC = 3*(NL-1);
//    Eigen::EigenSolver<Eigen::MatrixXd>::EigenvectorsType z;


//    z = h_temp.eigenvalues();

//    Eigen::VectorXd z_real, z_img;

//    z_real = z.real();
//    z_img = z.imag();


//    if(walking_tick_ == 0){

//    }
//    Eigen::MatrixXd g_temp;
//    g_temp.resize(4*NL,1);
//    g_temp = -c.transpose()*p_ref;

//    Eigen::Vector3d init_x;
//    init_x.setZero();
//    if(walking_tick_ ==0){
//        init_x(0) = com_support_init_(0);
//    }
//    else {
//        init_x = x_1_;
//    }



//    Eigen::VectorXd b_temp;
//    b_temp.resize(NC);
//    b_temp.setZero();
//    b_temp.segment<3>(0) = a*init_x;


//    real_t H[FNL*FNL], g[FNL],A[NC*FNL], bA[NC],lb[FNL],ub[FNL];

//    for(int i=0;i<FNL;i++){
//        for(int j=0;j<FNL;j++){
//            H[j*FNL+i] = h_temp(i,j);
//        }
//        g[i] = g_temp(i,0);
//    }

//    for(int i=0;i<NC;i++){
//        for(int j=0;j<FNL;j++){
//            A[j*NC+i] = A_temp(i,j);
//        }
//        bA[i] = b_temp(i);
//    }

//    //SymSparseMat
//    real_t xOpt[FNL];
//    QProblem example(FNL,NC);
//    //QProblem example(FNL,NC,HST_SEMIDEF);
//    //SQProblem example(FNL,NC,HST_SEMIDEF);

//    Options myop;
//    myop.printLevel = PL_NONE;
//    example.setOptions(myop);

//    int_t nWSR = 1000;

//    for(int i=0;i<FNL;i++){
//        lb[i] = -100000000000;
//        ub[i] = -100000000000;
//    }

////    if(walking_tick_ ==0){
////        for(int i=0;i<NC;i++){
////            for(int j=0;j<FNL;j++){
////              file[18]<<A_temp(i,j)<<"\t";
////            }
////            file[18]<<endl;
////            file[19]<<b_temp(i,0)<<endl;
////        }
////        for(int i=0; i<FNL;i++){
////            file[17]<<g_temp(i,0)<<endl;
////            for(int j=0;j<FNL;j++){
////                file[16]<<h_temp(i,j)<<"\t";
////            }
////            file[16]<<endl;
////        }
////        for(int i=0;i<NL;i++)
////            file[20]<<p_ref(i)<<endl;

////        for(int i=0;i<NL;i++){
////            for(int j=0;j<4*NL;j++){
////                file[22]<<c_temp(i,j)<<"\t";
////                file[23]<<Selec_s(i,j)<<"\t";
////            }
////            file[22]<<endl;
////            file[23]<<endl;
////        }
////        for(int i=0;i<FNL;i++){
////            file[21]<<z_real(i)<<"\t"<<z_img(i)<<endl;
////        }

////    }
//    if(walking_tick_ == 0)
//        example.init(H,g,A,lb,ub,bA,bA,nWSR,0);

//    example.hotstart(g,lb,ub,bA,bA,nWSR,0);

//    example.getPrimalSolution(xOpt);

////    int_t getSimpleStatus(returnValue returnvalue);
////    cout<<"return value : "<<example.getStatus()<<endl;

//    x_1_ = a*init_x + b*xOpt[0];

////    if(walking_tick_ ==0 ){
////        for(int i=0;i<NL;i++){
////            file[16]<<xOpt[4*i+0]<<"\t"<<xOpt[4*i+1]<<"\t"<<xOpt[4*i+2]<<"\t"<<xOpt[4*i+3]<<"\t"<<endl;
////        }
////    }


//    file[15]<<walking_tick_<<"\t"<<xOpt[0]<<"\t"<<xOpt[1]<<"\t"<<xOpt[2]<<"\t"<<xOpt[3]<<"\t"<<xOpt[4]<<"\t"<<xOpt[5]<<"\t"<<xOpt[6]<<"\t"<<xOpt[7]
//           <<"\t"<<x_1_(0)<<"\t"<<x_1_(1)<<"\t"<<x_1_(2)<<endl;


//}
//void WalkingController::previewQP(){
//    if(walking_tick_ ==0)
//        cout<<"preview with QP solver and t_start time : "<<t_start_ <<endl;

//    int NL = 100;
//    NL = 320;
//    Eigen::Matrix3d a;
//    double dt = 1.0/hz_;

//    a.setZero();
//    a(0,0) = 1.0;    a(0,1) = dt;    a(0,2) = pow(dt,2)/2.0;
//    a(1,1) = 1.0;    a(1,2) = dt;
//    a(2,2) = 1.0;

//    Eigen::Matrix<double, 3,1> b;
//    b(0) = pow(dt,3)/6.0;
//    b(1) = pow(dt,2)/2.0;
//    b(2) = dt;

//    Eigen::Matrix<double, 1,4> c;
//    c.setZero();
//    c(1) = 1.0; c(3) = -zc_/GRAVITY;

//    Eigen::Matrix3d Iden;
//    Iden.setIdentity();

//    int FNL = 4*NL;

//    Eigen::MatrixXd A_temp;
//    A_temp.resize(FNL,FNL);
//    A_temp.setZero();
//    A_temp.block<3,1>(1,0) = b;
//    A_temp.block<3,3>(1,1) = -Iden;


//    for(int i=1;i<NL; i++){
//        A_temp.block<3,3>(4*i+1,4*i-3) = a;
//        A_temp.block<3,1>(4*i+1,4*i) = b;
//        A_temp.block<3,3>(4*i+1,4*i+1) = -Iden;
//    }

//    Eigen::Vector3d x_0;
//    x_0.setZero();

//    if(walking_tick_ == 0){
//        x_0(0)  = com_support_current_(0);
//        cout<<"support init at cnt 0 : "<<com_support_current_(0)<<endl;
//    }

//    else if(walking_tick_ == t_start_){
//        x_0(0)  = com_support_current_(0);
//    }
//    else{
//        x_0 = x_1_;// x_1_ is i-1 tick solution of x
//    }

////    else {
////        x_0 = x_1_;
////    }




//    Eigen::VectorXd b_temp;
//    b_temp.resize(FNL);
//    b_temp.setZero();
//    Eigen::Vector3d temp;
//    temp = a*x_0;
//    b_temp.segment<3>(1) =-temp;// -a*x_0;

//    if(walking_tick_ ==0){
//        cout<<"b vector ; "<<b_temp(0)<<", "<<b_temp(1)<<", "<<b_temp(2)<<", "<<b_temp(3)<<endl;
//    }


//    Eigen::Vector4d s;
//    s.setZero();
//    s(0) = 1.0;

////    Eigen::Matrix<double, 1, 8> s;
////    s.setZero();
////    s(0) = -1.0; s(4) = 1.0;


//    Eigen::MatrixXd Selec_M;
//    Selec_M.resize(NL,FNL);
//    Selec_M.setZero();
//    //Selec_M(0,0) = 1.0;
//    for(int i=0;i<NL;i++)
//        Selec_M.block<1,4>(i,4*i) = s;
//        //Selec_M.block<1,8>(i,4*(i-1)) = s;

////    if(walking_tick_ == 0){
////        cout<<"saving data : "<<endl;
////        for(int i=0;i<4*NL;i++){
////            for(int j=0;j<4*NL;j++){
////                file[16]<<Selec_M(i,j)<<"\t";
////            }
////            file[16]<<endl;
////        }
////    }

//    Eigen::MatrixXd C_temp;
//    C_temp.resize(NL,FNL);

//    for(int i=0;i<NL;i++)
//         C_temp.block<1,4>(i,4*i) = c;

//    Eigen::MatrixXd Q;
//    Q.resize(NL,NL);
//    Q.setIdentity();

//    Eigen::MatrixXd H_temp;
//    H_temp.resize(FNL,FNL);

//    double r = 0.0000001;
//    H_temp  = 100*C_temp.transpose()*C_temp;
//    H_temp += r*Selec_M.transpose()*Selec_M;


//    Eigen::VectorXd p_ref;
//    p_ref.resize(NL);
//    for(int i=0;i<NL;i++)
//        p_ref(i) = ref_zmp_(i,0);


////    if(walking_tick_ == 0){
////        file[16]<<walking_tick_;
////        for(int i=0;i<NL;i++)
////          file[16]<<"\t"<<p_ref(i);

////        file[16]<<endl;
////    }

//    Eigen::MatrixXd g_temp;
//    g_temp.resize(FNL,1);

//    g_temp = -C_temp.transpose()*p_ref;


//    real_t H[FNL*FNL], g[FNL],A[FNL*FNL],B[FNL], lb[FNL],ub[FNL];

//    for(int i=0;i<FNL;i++){
//        for(int j=0;j<FNL;j++){
//            H[j*FNL + i] = H_temp(i,j);
//            A[j*FNL +i] = A_temp(i,j);
//        }
//        g[i] = g_temp(i,0);
//        B[i] = b_temp(i);
//    }

//    real_t xOpt[FNL];

//    QProblem example(FNL,FNL);

//    Options myOptions;

//    myOptions.printLevel = PL_NONE;
//    example.setOptions(myOptions);

//    int_t nWSR = 1000;

//    for(int i=0;i<FNL;i++){
//        lb[i] = -10000000;
//        ub[i] = 10000000  ;
//    }

//    if(walking_tick_ ==0)
//        example.init(H,g,A,lb,ub,B,B,nWSR);

//    example.hotstart(g,lb,ub,B,B,nWSR);
//    example.getPrimalSolution(xOpt);

////    if(walking_tick_ ==0 ){
////        for(int i=0;i<NL;i++){
////            file[16]<<xOpt[4*i+0]<<"\t"<<xOpt[4*i+1]<<"\t"<<xOpt[4*i+2]<<"\t"<<xOpt[4*i+3]<<"\t"<<endl;
////        }
////    }

////     x_1_ =  a*x_0 + b*xOpt[0];
//////    x_1_(0) = xOpt[1];
//////    x_1_(1) = xOpt[2];
//////    x_1_(2) = xOpt[3];

////    file[15]<<walking_tick_<<"\t"<<xOpt[0]<<"\t"<<xOpt[1]<<"\t"<<xOpt[2]<<"\t"<<xOpt[3]<<"\t"<<xOpt[4]<<"\t"<<xOpt[5]<<"\t"<<xOpt[6]<<"\t"<<xOpt[7]
////           <<"\t"<<x_1_(0)<<"\t"<<x_1_(1)<<"\t"<<x_1_(2)<<endl;
//    //file[15]<<walking_tick_<<"\t"<<x_1_(0)<<"\t"<<x_1_(1)<<"\t"<<x_1_(2)<<endl;//<<"\t"<<x_1_(3)<<"\t"<<sol_x(0)<<"\t"<<sol_x(1)<<"\t"<<sol_x(2)<<"\t"<<sol_x(3)
////    if(walking_tick_ == 0){
////        file[15]<<walking_tick_;
////        for(int i=0;i<14;i++)
////            file[15]<<"\t"<<xOpt[4*i+1];
////        file[15]<<endl;
////    }

//}
//void WalkingController::MPCwQP(){
//    if(walking_tick_ ==0)
//        cout<<"sibalsibal"<<endl;
//////    int zmp_size;
//////    zmp_size = ref_zmp_.col(1).size();

//////    Eigen::VectorXd px_ref, py_ref;
//////    px_ref.resize(zmp_size);
//////    py_ref.resize(zmp_size);

//////    px_ref = ref_zmp_.col(0);
//////    py_ref = ref_zmp_.col(1);

////      cout<<"b"<<endl;
//    int NL = 16*hz_/10;
//    //NL = 320;

//    NL = 50;
//    Eigen::Matrix4d a;
//    Eigen::Matrix<double, 1,3>c;

//    double dt =1.0/hz_;

//    a.setIdentity();
//    a(0,1) = dt; a(0,2) = 0.5*pow(dt,2); a(0,3) = pow(dt,3)/6;
//    a(1,2) = dt; a(1,3) = pow(dt,2)/2;
//    a(2,3) = dt; a(3,3) = 0.0;

//    c(0,0) = 1; c(0,1) = 0; c(0,2) = -zc_/GRAVITY;

//////    Eigen::Vector4d b_bar;
//////    b_bar(0) = c*b;
//////    b_bar.segment(1,3) = b;

//////    Eigen::Matrix4x3d f_bar;
//////    f_bar.block<1,3>(0,0) = c*A;
//////    f_bar.block<3,3>(0,1) = A;

//////    Eigen::Matrix4d A_bar;
//////    A_bar.setZero();
//////    A_bar(0,0) = 1.0;
//////    A_bar.block<4,3>(0,1) = f_bar;

//    Eigen::MatrixXd extended_A;
//    extended_A.resize(4*NL,4*NL);
//    extended_A.setZero();
//    extended_A.block<4,4>(0,0).setIdentity();

//    for(int i=1;i<NL;i++){
//        extended_A.block<4,4>(4*i,4*(i-1)) = a;
//    }

//    Eigen::MatrixXd extended_c;
//    extended_c.resize(NL,4*NL);
//    extended_c.setZero();

//    for(int i=0;i<NL;i++){
//        extended_c.block<1,3>(i,4*i) = c;
//    }

//    Eigen::MatrixXd I_a;
//    I_a.resize(4*NL,4*NL);
//    I_a.setZero();
//    Eigen::Matrix4d iden3d;
//    iden3d.setIdentity();
//    iden3d(3,3) = 0.0;

//    for(int i=0;i<NL;i++)
//        I_a.block<4,4>(4*i,4*i) = iden3d;


//    Eigen::MatrixXd A_input;
//    A_input.resize(4*NL,4*NL);
//    A_input = extended_A - I_a; // for constraint A

//    A_input.block<3,3>(0,0).setIdentity();
//    A_input(3,3) = 0.0;

//    Eigen::VectorXd p_ref;
//    p_ref.resize(NL);

//    for(int i=0;i<NL;i++)
//        p_ref(i) = ref_zmp_(i,0);

////    if(walking_tick_ == 0){
////        cout<<"saving data : "<<endl;
////        for(int i=0;i<4*NL;i++){
////            for(int j=0;j<4*NL;j++){
////                file[16]<<A_input(i,j)<<"\t";
////            }
////            file[16]<<endl;
////        }
////    }
//    Eigen::MatrixXd S;
//    S.resize(NL,4*NL);
//    Eigen::MatrixXd sel_vector;
//    sel_vector.resize(1,8);
//    sel_vector.setZero();
//    sel_vector(0,3) = -1; sel_vector(0,7) = 1;
//    //sel_vector(0,3) = 1.0;
//    S(0,3) = 1.0;
//    for(int i=1;i<NL-1;i++)
//        S.block<1,8>(i,4*i) = sel_vector;


//    Eigen::MatrixXd h_temp;
//    h_temp.resize(4*NL,4*NL);
//    Eigen::Matrix<double, 1,1> r;
//    r(0,0) = 0.0001;

//    h_temp = 10*extended_c.transpose()*extended_c;
//    h_temp += S.transpose()*r*S;


//    Eigen::MatrixXd g_temp;
//    g_temp.resize(4*NL,1);

//    g_temp = - extended_c.transpose()*p_ref;



//    real_t H[4*NL*4*NL], g[4*NL], A[4*NL*4*NL];

//    for(int i=0;i<4*NL;i++){
//        for(int j=0;j<4*NL;j++){
//           // cout<<"test 4"<<endl;
//            H[j*4*NL + i] = h_temp(i,j);
//           // cout<<"test 5 "<<endl;
//            A[j*4*NL + i] = A_input(i,j);
//        }
//        //cout<<"test 6"<<endl;
//        g[i] = g_temp(i,0);
//    }



//    real_t xOpt[4*NL];

//    QProblem example(4*NL,4*NL);


//    Options myOptions;
//    myOptions.printLevel= PL_NONE;
//    example.setOptions(myOptions);

//    int_t nWSR = 1000;
//    real_t lbA[4*NL], lb[4*NL], ub[4*NL];
//    for(int i=0;i<4*NL;i++){
//        lbA[i] = 0.0;
//    }
//    Eigen::Vector3d x_0;
//    x_0.setZero();

//    if(walking_tick_ ==0){
//        x_0(0) = com_support_current_(0);
//    }
//    else {
//     x_0 = x_1_;
//    }
////    if(walking_tick_ == 0){
//        //lbA[0] = com_support_current_(0);
//    lbA[0] = x_0(0);
//    lbA[1] = x_0(1);
//    lbA[2] = x_0(2);

////    }
////    else if(walking_tick_ == t_start_){
//        //lbA[0] = com_support_current_(0);
////    }
////    else{

////    }
////    lbA[0] = x_1_(0);
////    lbA[1] = x_1_(1);
////    lbA[2] = x_1_(2);


////    if(walking_tick_-zmp_start_time_==0 && current_step_num_ == 0)
////        lbA[0] = com_support_init_(0);
////    else {
////        lbA[0] = x_1_(0);
////        lbA[1] = x_1_(1);
////        lbA[2] = x_1_(2);
////        lbA[3] = x_1_(3);
////    }

//    for(int i=0;i<NL;i++){
//        lb[4*i+3] = -10000000;
//        ub[4*i+3] =  10000000;
//    }

//    //xOpt[0] = com_support_current_(0);

//    if(walking_tick_ ==0)
//        example.init(H,g,A,lb,ub,lbA,lbA,nWSR);

//    example.hotstart(g,lb,ub,lbA,lbA,nWSR);

//    example.getPrimalSolution(xOpt);


////    if(walking_tick_ == 0){
////        file[16]<<walking_tick_;
////        for(int i=0;i<NL;i++)
////          file[16]<<"\t"<<p_ref(i);

////        file[16]<<endl;
////    }
////    if(walking_tick_ ==0 ){
////        for(int i=0;i<NL;i++){
////            file[16]<<xOpt[4*i+0]<<"\t"<<xOpt[4*i+1]<<"\t"<<xOpt[4*i+2]<<"\t"<<xOpt[4*i+3]<<"\t"<<endl;
////        }
////    }
//    Eigen::Vector4d sol_x;
//    for(int i=0;i<4;i++)
//        sol_x(i) = xOpt[i];

////    x_1_ = a.block<3,3>(0,0)*x_0 + a.block<3,1>(0,3)*xOpt[3];
////    //x_1_ = a*sol_x;

//////    file[15]<<walking_tick_<<"\t"<<x_1_(0)<<"\t"<<x_1_(1)<<"\t"<<x_1_(2)<<endl;//"\t"<<x_1_(3)<<"\t"<<sol_x(0)<<"\t"<<sol_x(1)<<"\t"<<sol_x(2)<<"\t"<<sol_x(3)
//////           <<"\t"<<xOpt[4]<<"\t"<<xOpt[5]<<"\t"<<xOpt[6]<<"\t"<<example.getObjVal()<<endl;

////   file[15]<<walking_tick_<<"\t"<<xOpt[0]<<"\t"<<xOpt[1]<<"\t"<<xOpt[2]<<"\t"<<xOpt[3]<<"\t"<<xOpt[4]<<"\t"<<xOpt[5]<<"\t"<<xOpt[6]<<"\t"<<xOpt[7]
////          <<"\t"<<x_1_(0)<<"\t"<<x_1_(1)<<"\t"<<x_1_(2)<<endl;

////   x_1_(0) = xOpt[4];
////    x_1_(1) = xOpt[5];
////    x_1_(2) = xOpt[6];
////    printf( "\nxOpt = [ %e, %e, %e, %e, %e, %e,%e, %e, %e];  objVal = %e\n\n",
////    xOpt_[0],xOpt_[1],xOpt_[2],xOpt_[3],xOpt_[4],xOpt_[5],xOpt_[6],xOpt_[7],xOpt_[8],example.getObjVal() );
////cout<<"test 7 "<<endl;

//}
void WalkingController::zmpPattern(unsigned int current_step_number, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py){
    if(walking_tick_ == 0)
        cout<<"zmp pattern "<<endl;

    temp_px.resize(t_total_);
    temp_py.resize(t_total_);
    temp_px.setZero();
    temp_py.setZero();

    Eigen::Matrix4d time_mat;
    Eigen::Vector4d coeff_vector;
    Eigen::Vector4d value_vector;

    double td = t_rest_init_ + t_double1_;
    double ts = (t_total_-t_rest_last_-t_double2_) - (t_rest_init_+t_double1_);
    double tf = t_total_;

    time_mat.setZero();
    time_mat.col(0).setOnes();
    for(int i=1;i<4;i++){
        time_mat(1,i) = pow(td,i);
        time_mat(2,i) = pow(ts,i);
        time_mat(3,i) = pow(tf,i);
    }
    double d= 0.5*(foot_step_(current_step_num_,0) - foot_step_(current_step_num_-1,0));

    if(current_step_number == 0){
       value_vector(0) = 0.0;
       value_vector(1) = supportfoot_support_init_offset_(0);
       value_vector(2) = supportfoot_support_init_offset_(0);
       value_vector(3) = (supportfoot_support_init_offset_(0) + foot_step_support_frame_(current_step_number,0))/2.0;
    }
    else if(current_step_number ==1){
        value_vector(0) = (foot_step_support_frame_(current_step_number-1,0)+supportfoot_support_init_(0))/2.0;
        value_vector(1) = foot_step_support_frame_offset_(current_step_number-1,0);
        value_vector(2) = foot_step_support_frame_offset_(current_step_number-1,0) +d;
        value_vector(3) = (foot_step_support_frame_(current_step_number,0)+foot_step_support_frame_(current_step_number-1,0))/2.0;
    }
    else if(current_step_number >1 && current_step_number<total_step_num_){
        value_vector(0) = (foot_step_support_frame_(current_step_number-1,0)+foot_step_support_frame_(current_step_number-2,0))/2.0;
        value_vector(1) = foot_step_support_frame_offset_(current_step_number-1,0)-d;
        value_vector(2) = foot_step_support_frame_offset_(current_step_number-1,0)+d;
        value_vector(3) =  (foot_step_support_frame_(current_step_number,0)+foot_step_support_frame_(current_step_number-1,0))/2.0;
    }
    else{
        value_vector(0) = (foot_step_support_frame_(current_step_number-1,0)+foot_step_support_frame_(current_step_number-2,0))/2.0;
        value_vector(1) = foot_step_support_frame_offset_(current_step_number-1,0);
        value_vector(2) = foot_step_support_frame_offset_(current_step_number-1,0);
        value_vector(3) = foot_step_support_frame_offset_(current_step_number-1,0);
    }


    coeff_vector = time_mat.inverse()*value_vector;

    for(int i=0;i<t_total_;i++){
        temp_px(i) = coeff_vector(0) + coeff_vector(1)*i + coeff_vector(2) * pow(i,2) + coeff_vector(3) * pow(i,3);
    }

}
//void WalkingController::supportToWaistPattern(){

//    waist_trajectory_support_ = pelv_trajectory_support_;
//    waist_trajectory_float_ = DyrosMath::inverseIsometry3d(waist_trajectory_support_)*waist_trajectory_support_;
//    lfoot_trajectory_waist_ = DyrosMath::inverseIsometry3d(waist_trajectory_support_)*lfoot_trajectory_support_;
//    rfoot_trajectory_waist_ = DyrosMath::inverseIsometry3d(waist_trajectory_support_)*rfoot_trajectory_support_;
//    lfoot_trajectory_euler_waist_=DyrosMath::rot2Euler(lfoot_trajectory_waist_.linear());
//    rfoot_trajectory_euler_waist_=DyrosMath::rot2Euler(rfoot_trajectory_waist_.linear());

//}
//void WalkingController::updateInitialStatefromWaist(){
//    lfoot_waist_init_ = DyrosMath::inverseIsometry3d(link_transform_[WA_LINK])*lfoot_float_init_;
//    rfoot_waist_init_ = DyrosMath::inverseIsometry3d(link_transform_[WA_LINK])*rfoot_float_init_;

//}
//void WalkingController::getRobotStatefromWaist(){

//    lfoot_waist_current_ = DyrosMath::inverseIsometry3d(link_transform_[WA_LINK])*lfoot_float_current_;
//    rfoot_waist_current_ = DyrosMath::inverseIsometry3d(link_transform_[WA_LINK])*rfoot_float_current_;
//    if(foot_step_(current_step_num_, 6) ==0)//rifht foot support
//        supportfoot_waist_current_ = rfoot_waist_current_;
//    else if(foot_step_(current_step_num_,6) ==1) // left foot support
//        supportfoot_waist_current_ = lfoot_waist_current_;

//    waist_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_waist_current_);

//    current_leg_waist_l_jacobian_.leftCols(1) = current_waist_jacobian_[0].leftCols(1);
//    current_leg_waist_l_jacobian_.block<6,6>(0,1) = current_leg_jacobian_l_;

//    current_leg_waist_r_jacobian_.leftCols(1) = current_waist_jacobian_[0].leftCols(1);
//    current_leg_waist_r_jacobian_.block<6,6>(0,1) = current_leg_jacobian_r_;

//    current_leg_waist_l_jacobian_.topRows(3) = link_transform_[WA_LINK].linear().transpose()*current_leg_waist_l_jacobian_.topRows(3);
//    current_leg_waist_l_jacobian_.bottomRows(3) = link_transform_[WA_LINK].linear().transpose()*current_leg_waist_l_jacobian_.bottomRows(3);

//    current_leg_waist_r_jacobian_.topRows(3) = link_transform_[WA_LINK].linear().transpose()*current_leg_waist_r_jacobian_.topRows(3);
//    current_leg_waist_r_jacobian_.bottomRows(3) = link_transform_[WA_LINK].linear().transpose()*current_leg_waist_r_jacobian_.bottomRows(3);

//}
//void WalkingController::computeWaistJacobianCtrl(Eigen::Isometry3d waist_lleg_transform, Eigen::Isometry3d waist_rleg_transform, Eigen::Vector3d waist_lleg_transform_euler, Eigen::Vector3d waist_rleg_transform_euler,Eigen::VectorXd& desired_q_dot){

//    if(walking_tick_ == 0)
//        cout<<"waist jacobian ctrl "<<endl;
//    Eigen::Matrix6d jacobian_temp_l, jacobian_temp_r;
//    double wl, wr, w0, lambda, a;
//    w0 = 0.001;
//    lambda = 0.05;
//    jacobian_temp_l = current_leg_waist_l_jacobian_*current_leg_waist_l_jacobian_.transpose();
//    jacobian_temp_r = current_leg_waist_r_jacobian_*current_leg_waist_r_jacobian_.transpose();
//    wl = sqrt(jacobian_temp_l.determinant());
//    wr = sqrt(jacobian_temp_r.determinant());
//    Eigen::Matrix<double, 7, 6> current_leg_waist_jacobian_l_inv, current_leg_waist_jacobian_r_inv;
//    Eigen::Matrix6d j_damped;

//    // inverse jacobian including waist joint
//    if(wl<= w0)
//    {//left jacobian including waist
//        a = lambda * pow(1-wl/w0,2);
//        j_damped = current_leg_waist_l_jacobian_*current_leg_waist_l_jacobian_.transpose() + a*Eigen::Matrix6d::Identity();
//        j_damped = j_damped.inverse();

//        cout<<"Singularity Regin of left leg : "<<wl<<endl;
//        current_leg_waist_jacobian_l_inv = current_leg_waist_l_jacobian_.transpose()*j_damped;
//    }
//    else {
//        current_leg_waist_jacobian_l_inv = current_leg_waist_l_jacobian_.transpose()*(current_leg_waist_l_jacobian_*current_leg_waist_l_jacobian_.transpose()).inverse();
//    }

//    if(wr<= w0)
//    {//left jacobian including waist
//        a = lambda * pow(1-wr/w0,2);
//        j_damped = current_leg_waist_r_jacobian_*current_leg_waist_r_jacobian_.transpose() + a*Eigen::Matrix6d::Identity();
//        j_damped = j_damped.inverse();

//        cout<<"Singularity Regin of right leg : "<<wr<<endl;
//        current_leg_waist_jacobian_r_inv = current_leg_waist_r_jacobian_.transpose()*j_damped;
//    }
//    else {
//        current_leg_waist_jacobian_r_inv = current_leg_waist_r_jacobian_.transpose()*(current_leg_waist_r_jacobian_*current_leg_waist_r_jacobian_.transpose()).inverse();
//    }

//    // inverse jacobian excluding waist jacobian at waist coordinate
//    jacobian_temp_l = current_leg_waist_l_jacobian_.block<6,6>(0,1)*current_leg_waist_l_jacobian_.block<6,6>(0,1).transpose();
//    jacobian_temp_r = current_leg_waist_r_jacobian_.block<6,6>(0,1)*current_leg_waist_r_jacobian_.block<6,6>(0,1).transpose();
//    wl = sqrt(jacobian_temp_l.determinant());
//    wr = sqrt(jacobian_temp_r.determinant());
//    Eigen::Matrix6d current_leg_jacobian_l_inv, current_leg_jacobian_r_inv;

//    if(wl<=w0){
//        a = lambda * pow(1-wl/w0,2);
//        j_damped = current_leg_waist_l_jacobian_.block<6,6>(0,1)*current_leg_waist_l_jacobian_.block<6,6>(0,1).transpose() + a*Eigen::Matrix6d::Identity();
//        j_damped = j_damped.inverse();

//        cout<<"Singularity Regin of left leg : "<<wl<<endl;
//        current_leg_jacobian_l_inv = current_leg_waist_l_jacobian_.block<6,6>(0,1).transpose()*j_damped;
//    }
//    else{
//        current_leg_jacobian_l_inv = current_leg_waist_l_jacobian_.block<6,6>(0,1).transpose()*(current_leg_waist_l_jacobian_.block<6,6>(0,1)*current_leg_waist_l_jacobian_.block<6,6>(0,1).transpose()).inverse();
//    }

//    if(wr<=w0){
//        a = lambda * pow(1-wr/w0,2);
//        j_damped = current_leg_waist_r_jacobian_.block<6,6>(0,1)*current_leg_waist_r_jacobian_.block<6,6>(0,1).transpose() + a*Eigen::Matrix6d::Identity();
//        j_damped = j_damped.inverse();

//        cout<<"Singularity Regin of right leg : "<<wr<<endl;
//        current_leg_jacobian_r_inv = current_leg_waist_r_jacobian_.block<6,6>(0,1).transpose()*j_damped;
//    }
//    else{
//        current_leg_jacobian_r_inv = current_leg_waist_r_jacobian_.block<6,6>(0,1).transpose()*(current_leg_waist_r_jacobian_.block<6,6>(0,1)*current_leg_waist_r_jacobian_.block<6,6>(0,1).transpose()).inverse();
//    }



////    current_leg_waist_l_jacobian_inv_ = current_leg_waist_jacobian_l_inv;
////    current_leg_waist_r_jacobian_inv_ = current_leg_waist_jacobian_r_inv;

//    Eigen::Matrix6d kp;
//    kp.setZero();
//    kp(0,0) = 200;
//    kp(1,1) = 200;
//    kp(2,2) = 200;
//    kp(3,3) = 200;
//    kp(4,4) = 200;
//    kp(5,5) = 200;


//    Eigen::Vector6d lp, rp, cubic_xr, cubic_xl;
//    lp.setZero(); rp.setZero(); cubic_xr.setZero(); cubic_xl.setZero();

//    if(walking_tick_ ==0){
//        lp.topRows(3) = (-lfoot_waist_current_.translation() + waist_lleg_transform.translation());
//        rp.topRows(3) = (-rfoot_waist_current_.translation() + waist_rleg_transform.translation());
//    }
//    else{
//        lp.topRows(3) = (-pre_lfoot_trajectory_waist_.translation() + waist_lleg_transform.translation());
//        rp.topRows(3) = (-pre_rfoot_trajectory_waist_.translation() + waist_rleg_transform.translation());
//    }


//    cubic_xl.topRows(3) = waist_lleg_transform.translation();
//    cubic_xl.bottomRows(3) = waist_lleg_transform_euler;

//    cubic_xr.topRows(3) = waist_rleg_transform.translation();
//    cubic_xr.bottomRows(3) = waist_rleg_transform_euler;

//    Eigen::Vector3d rleg_waist_phi, lleg_waist_phi;

//    Eigen::Matrix6d jacobian_check;
//    jacobian_check = current_leg_waist_l_jacobian_*current_leg_waist_jacobian_l_inv;

////    if(walking_tick_ ==0){
////        cout<<"lfoot position trajectory at waist "<<endl<<waist_lleg_transform.translation()<<endl;
////        cout<<"lfoot possition trajectory at waist "<<endl<<lfoot_waist_current_.translation()<<endl;
////        cout<<"lfoot position trajectory at pelvis :"<<endl<<lfoot_trajectory_float_.translation()<<endl;
////        cout<<"lf "<<lp<<endl;
////        cout<<"lfoot position current at pelvis : "<<endl<<lfoot_float_current_.translation()<<endl;
////        cout<<"jacobian check : "<<endl<<jacobian_check<<endl;
////    }

//    if(walking_tick_ == 0){
//        lleg_waist_phi = DyrosMath::legGetPhi(lfoot_waist_current_,lfoot_waist_init_, cubic_xl);
//        rleg_waist_phi = DyrosMath::legGetPhi(rfoot_waist_current_,rfoot_waist_init_, cubic_xr);
//    }
//    else{
//        lleg_waist_phi = DyrosMath::legGetPhi(pre_lfoot_trajectory_waist_,lfoot_waist_init_, cubic_xl);
//        rleg_waist_phi = DyrosMath::legGetPhi(pre_rfoot_trajectory_waist_,rfoot_waist_init_, cubic_xr);
//    }

////    if(walking_tick_ == 0){
////        lleg_waist_phi = DyrosMath::getPhi(lfoot_waist_current_.linear(), waist_lleg_transform.linear());
////        rleg_waist_phi = DyrosMath::getPhi(rfoot_waist_current_.linear(), waist_lleg_transform.linear());
////    }
////    else{
////        lleg_waist_phi = DyrosMath::getPhi(pre_lfoot_trajectory_waist_.linear(), waist_lleg_transform.linear());
////        rleg_waist_phi = DyrosMath::getPhi(pre_rfoot_trajectory_waist_.linear(), waist_lleg_transform.linear());
////    }


//    lp.bottomRows(3) = -lleg_waist_phi;
//    rp.bottomRows(3) = -rleg_waist_phi;

//    lp.bottomRows(3).setZero();
//    rp.bottomRows(3).setZero();
//    lp_ = lp;

//    Eigen::Vector6d q_foot_dot;
//    Eigen::Vector7d q_waist_foot_dot;

////    lp.setZero();
////    rp.setZero();
//    if(foot_step_(current_step_num_,6) == 1){// left foot support
////        lp.bottomRows(3) = -lleg_waist_phi;
////        rp.bottomRows(3) = rleg_waist_phi;

//        q_waist_foot_dot = current_leg_waist_jacobian_l_inv*(kp*lp);
//        q_foot_dot = current_leg_jacobian_r_inv*kp*rp;

//        desired_q_dot.segment(0,6) = q_waist_foot_dot.segment(1,6);
//        desired_q_dot.segment(6,6) = q_foot_dot;
//        desired_q_dot(12) = q_waist_foot_dot(0);

//    }
//    else{
////        lp.bottomRows(3) = lleg_waist_phi;
////        rp.bottomRows(3) = -rleg_waist_phi;

//        q_waist_foot_dot = current_leg_waist_jacobian_r_inv*(kp*rp);
//        q_foot_dot = current_leg_jacobian_l_inv*kp*lp;

//        desired_q_dot.segment(0,6) = q_foot_dot;
//        desired_q_dot.segment(6,6) = q_waist_foot_dot.segment(1,6);
//        desired_q_dot(12) = q_waist_foot_dot(0);


//    }
//    Eigen::Vector6d check;
//    check = current_leg_waist_l_jacobian_*q_waist_foot_dot;

//    file[8]<<walking_tick_;

//    for(int i=0;i<13;i++)
//        file[8]<<"\t"<<desired_q_dot (i)*RAD2DEG;

//    file[8]<<"\t"<<check(0)<<"\t"<<check(1)<<"\t"<<check(2)<<"\t"<<check(3)<<"\t"<<check(4)<<"\t"<<check(5);
//    file[8]<<endl;





//}
void WalkingController::getStateMatrix(int sampling_N, double dt, int interval){
    // for calculate Px, Pu matrix for state space.

    Eigen::Matrix3d A;
    A.setIdentity();
    A(0,1) = dt; A(0,2) = pow(dt,2)/2.0;
    A(1,2) = dt;
    A_ = A;
    Eigen::Vector3d b;
    b(0) = pow(dt,3)/6.0;
    b(1) = pow(dt,2)/2.0;
    b(2) = dt;
    B_ = b;
    Eigen::Matrix<double, 1, 3>c;
//    c(0) = 1.0; c(1) = 0.0; c(2) = -zc_/GRAVITY;
    c(0) = GRAVITY; c(1) = 0.0; c(2) = -zc_;

    Px_swing_.resize(sampling_N,3); Px_swing_0_.resize(sampling_N,3);
//    Px_.col(0).setOnes();
//    Px_(0,1) = dt; Px_(0,2) = dt*dt/2-zc_/GRAVITY;
    for(int i=0;i<sampling_N;i++){
        Px_swing_(i,0) = GRAVITY*1.0;
        Px_swing_(i,1) = GRAVITY*(i*interval +1)*dt;
        Px_swing_(i,2) = GRAVITY*(i*interval+1)*(i*interval+1)*dt*dt/2-zc_;
    }

    for(int i=0;i<sampling_N;i++){
        Px_swing_0_(i,0) = 1.0;
        Px_swing_0_(i,1) = (i*interval +1)*dt;
        Px_swing_0_(i,2) = (i*interval+1)*(i*interval+1)*dt*dt/2-zc_/GRAVITY;
    }

    Pu_swing_.resize(sampling_N,sampling_N); Pu_swing_0_.resize(sampling_N,sampling_N);
    Pu_swing_.setZero(); Pu_swing_0_.setZero();
    for(int i=0;i<sampling_N;i++){
        for(int j=0;j<i+1;j++){
            Pu_swing_(i,j) = GRAVITY*(1+3*(i-j)*interval+3*(i-j)*(i-j)*interval*interval)*dt*dt*dt/6-dt*zc_;
        }
    }

    for(int i=0;i<sampling_N;i++){
        for(int j=0;j<i+1;j++){
            Pu_swing_0_(i,j) = (1+3*(i-j)*interval+3*(i-j)*(i-j)*interval*interval)*dt*dt*dt/6-dt*zc_/GRAVITY;
        }
    }

//    for(int i=0;i<sampling_N;i++){
//        file[35]<<Px_swing_(i,0)<<"\t"<<Px_swing_(i,1)<<"\t"<<Px_swing_(i,2)<<endl;
//        for(int j=0;j<sampling_N;j++){
//            file[31]<<Pu_swing_(i,j)<<"\t";
//        }
//        file[31]<<endl;
//    }

    ///////////////////// for checking stability ////////////////////

    Eigen::MatrixXd eee;
    eee.resize(1,sampling_N);
    eee.setZero();
    eee(0,0) = 1.0;

    Eigen::MatrixXd PuTPu;
    PuTPu.resize(sampling_N,sampling_N);
    PuTPu = Pu_swing_0_.transpose()*Pu_swing_0_;

    Eigen::MatrixXd Iden;
    Iden.resize(sampling_N,sampling_N);
    Iden.setIdentity();

    double beta= 0.000001;
    Eigen::MatrixXd Pu_iden;
    Pu_iden.resize(sampling_N,sampling_N);
    Pu_iden = PuTPu + beta *Iden;

    Eigen::Matrix3d Stability_Matrix;
    Stability_Matrix  = A - b*eee*Pu_iden.inverse()*Pu_swing_0_.transpose()*Px_swing_0_;

    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            file[31]<<Stability_Matrix(i,j)<<"\t";
        }
        file[31]<<endl;
    }

    Eigen::MatrixXd A_vi;
    A_vi.resize(sampling_N,3);
    A_vi.setZero();
    for(int i=0;i<sampling_N;i++){
        A_vi(i,1) = 1.0;
        A_vi(i,2) = (i*interval +1)*dt;
    }

    Eigen::MatrixXd S_del_v;
    S_del_v.resize(sampling_N,sampling_N);
    S_del_v.setIdentity();
    for(int i=1;i<sampling_N;i++){
        S_del_v(i,i-1) = -1;
    }
    Eigen::MatrixXd A_del_v;
    A_del_v.resize(sampling_N,3);
    A_del_v = S_del_v*A_vi;

    Eigen::MatrixXd B_vi;
    B_vi.resize(sampling_N,sampling_N);

    for(int i=0;i<sampling_N;i++){
        for(int j=0;j<i+1;j++){
            B_vi(i,j) = (2*(i*interval -j*interval) +1)*dt*dt/2.0;
        }
    }

    Eigen::MatrixXd B_del_v;
    B_del_v.resize(sampling_N,sampling_N);

    B_del_v = S_del_v*B_vi;

    Eigen::MatrixXd S_v_0;
    S_v_0.resize(sampling_N,3);
    S_v_0.setZero();

    S_v_0(0,1) = 1.0;

    Eigen::MatrixXd B_del_t_B_del;
    B_del_t_B_del.resize(sampling_N,sampling_N);

    B_del_t_B_del = B_del_v.transpose()*B_del_v;

    double alpha, gamma;
    alpha= 1.0; beta = 0.001; gamma = 0.00001;

    Stability_Matrix = A- b*eee*(alpha*PuTPu+beta*B_del_t_B_del+gamma*Iden).inverse()*(alpha*Pu_swing_0_.transpose()*Px_swing_0_ + beta*B_del_v.transpose()*(A_del_v-S_v_0));

    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            file[35]<<Stability_Matrix(i,j)<<"\t";
        }
        file[35]<<endl;
    }
}
void WalkingController::getSwingLegMatrix(){
    unsigned int norm_size=0;
    unsigned int planning_step_numer =3;

    norm_size = t_total_*planning_step_numer;
    swing_foot_pos_.resize(norm_size,3);
    swing_foot_acc_.resize(norm_size,3);

    Eigen::Vector7d current_target_swing_foot, next_target_swing_foot;
    current_target_swing_foot = foot_step_support_frame_.row(current_step_num_);
    next_target_swing_foot = foot_step_support_frame_.row(current_step_num_+1);

    int index=0;
    double t_rest_temp = 0.05*hz_;

    Eigen::Vector3d first_swing_foot_init,second_swing_foot_init;

    if(foot_step_(current_step_num_,6) == 1){//left foot support
        for(int i=0;i<3;i++){
            first_swing_foot_init(i) = rfoot_support_init_.translation()(i);
            second_swing_foot_init(i) = lfoot_support_init_.translation()(i);
        }
    }
    else{//right foot support
        for(int i=0;i<3;i++){
            first_swing_foot_init(i) = lfoot_support_init_.translation()(i);
            second_swing_foot_init(i) = rfoot_support_init_.translation()(i);
        }
    }


    //if(current_step_num_ ==0 && walking_tick_<t_start_){
    if(current_step_num_<1){
        for(int i=0;i<t_temp_;i++){
             for(int j=0;j<3;j++){
                 swing_foot_pos_(index+i,j) = first_swing_foot_init(j);
                 swing_foot_acc_(index+i,j) = 0.0;
             }
         }
         cout<<"tick"<<walking_tick_<<" 11111 "<<endl;
         index = index+t_temp_;
    }
//    if(walking_tick_ < t_temp_){
//        for(int i=0;i<t_total_;i++){
//            for(int j=0;j<3;j++){
//                swing_foot_pos_(index+i,j) = first_swing_foot_init(j);
//                swing_foot_acc_(index+i,j) = 0.0;
//            }
//        }
//        cout<<"tick"<<walking_tick_<<" 11111 "<<endl;
//        index = index+t_total_;
//    }
    else{
        for(int i=0;i<t_total_;i++){
           if(i<t_rest_init_ + t_double1_){
               for(int j=0;j<3;j++){
                    swing_foot_pos_(index+i,j) = first_swing_foot_init(j);
                    swing_foot_acc_(index+i,j) = 0.0;
               }
            }
            else if(i>= t_rest_init_+t_double1_ && i<= t_total_-t_rest_last_-t_double2_){
               for(int j=0;j<2;j++){
                    swing_foot_pos_(index+i,j) = DyrosMath::cubic(i,t_rest_init_+t_double1_+2*t_rest_temp,t_total_-t_rest_last_-t_double2_-t_imp_-2*t_rest_temp,first_swing_foot_init(j),current_target_swing_foot(j),0.0,0.0);
                    swing_foot_acc_(index+i,j) = DyrosMath::cubic(i,t_rest_init_+t_double1_+2*t_rest_temp,t_total_-t_rest_last_-t_double2_-t_imp_-2*t_rest_temp,first_swing_foot_init(j),current_target_swing_foot(j),0.0,0.0);
               }
               if(i<t_rest_init_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0){
                    swing_foot_pos_(index+i,2) = DyrosMath::cubic(i,t_rest_init_+t_double1_+t_rest_temp,t_rest_init_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0.0,foot_height_,0.0,0.0);
                    swing_foot_acc_(index+i,2) = DyrosMath::cubic(i,t_rest_init_+t_double1_+t_rest_temp,t_rest_init_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0.0,foot_height_,0.0,0.0);
               }
               else {
                   swing_foot_pos_(index+i,2) = DyrosMath::cubic(i,t_rest_init_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,t_total_-t_rest_last_-t_double2_-t_imp_-t_rest_temp,foot_height_,current_target_swing_foot(2),0.0,0.0);
                   swing_foot_acc_(index+i,2) = DyrosMath::cubic(i,t_rest_init_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,t_total_-t_rest_last_-t_double2_-t_imp_-t_rest_temp,foot_height_,current_target_swing_foot(2),0.0,0.0);
               }
            }
            else {
               for(int j=0;j<3;j++){
                    swing_foot_pos_(index+i,j) = current_target_swing_foot(j);
                    swing_foot_acc_(index+i,j) = 0.0;
               }
            }
        }
        index += t_total_;
        for(int i=0;i<t_total_;i++){
           if(i<t_rest_init_ + t_double1_){
               for(int j=0;j<3;j++){
                    swing_foot_pos_(index+i,j) = second_swing_foot_init(j);
                    swing_foot_acc_(index+i,j) = 0.0;
               }
            }
            else if(i>= t_rest_init_+t_double1_ && i<= t_total_-t_rest_last_-t_double2_){
               for(int j=0;j<2;j++){
                    swing_foot_pos_(index+i,j) = DyrosMath::cubic(i,t_rest_init_+t_double1_+2*t_rest_temp,t_total_-t_rest_last_-t_double2_-t_imp_-2*t_rest_temp,second_swing_foot_init(j),next_target_swing_foot(j),0.0,0.0);
                    swing_foot_acc_(index+i,j) = DyrosMath::cubic(i,t_rest_init_+t_double1_+2*t_rest_temp,t_total_-t_rest_last_-t_double2_-t_imp_-2*t_rest_temp,second_swing_foot_init(j),next_target_swing_foot(j),0.0,0.0);
               }
               if(i<t_rest_init_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0){
                    swing_foot_pos_(index+i,2) = DyrosMath::cubic(i,t_rest_init_+t_double1_+t_rest_temp,t_rest_init_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0.0,foot_height_,0.0,0.0);
                    swing_foot_acc_(index+i,2) = DyrosMath::cubic(i,t_rest_init_+t_double1_+t_rest_temp,t_rest_init_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,0.0,foot_height_,0.0,0.0);
               }
               else {
                   swing_foot_pos_(index+i,2) = DyrosMath::cubic(i,t_rest_init_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,t_total_-t_rest_last_-t_double2_-t_imp_-t_rest_temp,foot_height_,next_target_swing_foot(2),0.0,0.0);
                   swing_foot_acc_(index+i,2) = DyrosMath::cubic(i,t_rest_init_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,t_total_-t_rest_last_-t_double2_-t_imp_-t_rest_temp,foot_height_,next_target_swing_foot(2),0.0,0.0);
               }
            }
            else {
               for(int j=0;j<3;j++){
                    swing_foot_pos_(index+i,j) = next_target_swing_foot(j);
                    swing_foot_acc_(index+i,j) = 0.0;
               }
            }
        }
        index+= t_total_;
    }


//    if(walking_tick_<t_temp_){
//        for(int i=0;i<t_temp_;i++){
//            file[15]<<walking_tick_<<"\t"<<swing_foot_pos_(i,0)<<"\t"<<swing_foot_pos_(i,1)<<"\t"<<swing_foot_pos_(i,2)<<"\t"<<swing_foot_acc_(i,0)<<"\t"<<swing_foot_acc_(i,1)<<"\t"<<swing_foot_acc_(i,2)<<endl;
//        }
//    }
//    else{
//        for(int i=0;i<t_total_;i++){
//            file[15]<<walking_tick_<<"\t"<<swing_foot_pos_(i,0)<<"\t"<<swing_foot_pos_(i,1)<<"\t"<<swing_foot_pos_(i,2)<<"\t"<<swing_foot_acc_(i,0)<<"\t"<<swing_foot_acc_(i,1)<<"\t"<<swing_foot_acc_(i,2)<<endl;
//        }
//    }


}
void WalkingController::getLegDynamicMatrix(int tick, int N, int interval,Eigen::MatrixXd& foot_pos, Eigen::MatrixXd& foot_acc, Eigen::MatrixXd& Ki_x,Eigen::MatrixXd& Di_x, Eigen::MatrixXd& Ki_y,Eigen::MatrixXd& Di_y){
    if(walking_tick_ == 0){
        Mass_sb_ =0;
        Mass_sl_ = 0;

        for(int i=0;i<6;i++)
            Mass_sl_ += link_mass_[i+i];

        Mass_sb_ = total_mass_- Mass_sl_;
    }

    Ki_x.resize(N,N); Ki_x.setZero();  Ki_y.resize(N,N); Ki_y.setZero();
    Di_x.resize(N,1); Di_x.setZero();  Di_y.resize(N,1); Di_y.setZero();

    for(int i=0;i<N;i++){
        Ki_x(i,i) = total_mass_/(Mass_sb_*GRAVITY + Mass_sl_*(foot_acc(interval*i+1+tick,2)+GRAVITY));
        Di_x(i) = Mass_sl_*((foot_acc(interval*i+1+tick,2) + GRAVITY)*foot_pos(interval*i+1+tick,0) -(foot_acc(interval*i+1+tick,0))*foot_pos(interval*i+1+tick,2))/(Mass_sb_*GRAVITY + Mass_sl_*(foot_acc(interval*(i-1)+1+tick,2)+GRAVITY));

        //Ki_y(i,i) = Mass_sb_/(Mass_sb_*GRAVITY + Mass_sl_*(foot_acc(interval*(i-1)+1+tick,2)+GRAVITY));
        Ki_y(i,i) = Ki_x(i,i);
        Di_y(i) = Mass_sl_*((foot_acc(interval*i+1+tick,2) + GRAVITY)*foot_pos(interval*i+1+tick,1) -(foot_acc(interval*i+1+tick,1))*foot_pos(interval*i+1+tick,2))/(Mass_sb_*GRAVITY + Mass_sl_*(foot_acc(interval*(i-1)+1+tick,2)+GRAVITY));
    }

    Eigen::MatrixXd gain_swing_leg;
    gain_swing_leg.resize(N,N);
    gain_swing_leg.setZero();
    for(int i=0;i<N;i++){
        gain_swing_leg(i,i) = Mass_sl_/(Mass_sb_*GRAVITY + Mass_sl_*(foot_acc(interval*(i-1)+1+tick,2)+GRAVITY));
    }
//    if(walking_tick_ ==0 || walking_tick_ == t_start_){
//        for(int i=0;i<N;i++){
//            file[21]<<Di_x(i)<<"\t";
//            for(int j=0;j<N;j++){
//                file[22]<<Ki_x_(i,j)<<"\t";
//            }
//            file[22]<<endl;

//        }
//        file[21]<<endl;
//        for(int i=0;i<N;i++){
//            for(int j=0;j<N;j++)
//                file[22]<<gain_swing_leg(i,j)<<"\t";
//            file[22]<<endl;
//        }

//    }

}

void WalkingController::QP_legdynamics(){

    double dt = 1.0/hz_;

    int NL = (int) 16*hz_/10;

    int N =20;
    int interval = NL/N;

    if(walking_tick_ == 0)
        getStateMatrix(N,dt,interval);

    if(walking_tick_ == 0 || walking_tick_ == t_start_){
        getSwingLegMatrix();
    }

    if(current_step_num_ == 0)
        zmp_start_time_ = 0.0;
    else {
        zmp_start_time_ = t_start_;
    }
    getLegDynamicMatrix(walking_tick_-zmp_start_time_,N,interval,swing_foot_pos_,swing_foot_acc_,Ki_x_,Di_x_,Ki_y_,Di_y_);

    Eigen::Vector3d x_0, y_0;
    x_0.setZero(); y_0.setZero();
    if(walking_tick_ - zmp_start_time_ == 0 && current_step_num_ ==0){
        x_0(0) = com_support_init_(0);//-0.03;
//        y_0(0) = com_support_init_(1);
        y_0(0) = com_support_init_(0);
    }
    else {
        x_0 = x_p1_;
        y_0 = y_p1_;

    }

    Eigen::VectorXd ref_zmp_x,ref_zmp_y;
    ref_zmp_x.resize(N); ref_zmp_y.resize(N);

    for(int i=0;i<N;i++){
        ref_zmp_x(i) = ref_zmp_(walking_tick_-zmp_start_time_+interval*i+1,0);
        ref_zmp_y(i) = ref_zmp_(walking_tick_-zmp_start_time_+interval*i+1,1);
    }

//    if(walking_tick_ ==0){
//        for(int i=0;i<N;i++){
//            file[25]<<ref_zmp_x(i)<<"\t";
//        }
//        file[25]<<endl;
//    }
    file[25]<<walking_tick_<<"\t"<<ref_zmp_x(0)<<"\t"<<ref_zmp_y(0)<<endl;
    double alpha, beta;
    alpha = 1.0;
    beta = 0.000001;

    Eigen::MatrixXd Qx,Qy, gx,gy;
    Qx.resize(N,N); Qy.resize(N,N); gx.resize(N,1); gy.resize(N,1);

    Eigen::MatrixXd temp_kPu, temp_kPu_T;
    temp_kPu.resize(N,N); temp_kPu_T.resize(N,N);

    Eigen::MatrixXd Identity_M;
    Identity_M.resize(N,N);
    Identity_M.setIdentity();

    //////////////////////////////////
    /// //// for checking Px, Pu matrix ///////
    ///

    if(current_step_num_==0){
        Eigen::MatrixXd PuuP;
        PuuP.resize(N,N);


        PuuP = Pu_swing_0_.transpose()*Pu_swing_0_;

        Qx = alpha*PuuP + beta*Identity_M;
        Eigen::MatrixXd px_x0;
        px_x0.resize(N,1);

        px_x0 = Px_swing_0_*x_0;
        gx = -alpha*Pu_swing_0_.transpose()*(px_x0-ref_zmp_x);

    }

    else{
        Eigen::MatrixXd puK_T_K_pu;
        puK_T_K_pu.resize(N,N);

        temp_kPu = Ki_x_*Pu_swing_;
        temp_kPu_T = temp_kPu.transpose();

        puK_T_K_pu = temp_kPu_T*temp_kPu;
        Qx = alpha*puK_T_K_pu + beta*Identity_M;
    //    Qx += beta*Identity_M;

        Eigen::MatrixXd temp_kPx, kPx_x0;
        temp_kPx.resize(N,3); kPx_x0.resize(N,1);

        temp_kPx = Ki_x_*Px_swing_;
        kPx_x0 =temp_kPx*x_0;

        Eigen::MatrixXd zmp_diff;
        zmp_diff.resize(N,1);
        zmp_diff = kPx_x0-ref_zmp_x + Di_x_;
       // gx = temp_kPu_T*gx;
        gx = -alpha*temp_kPu_T*zmp_diff;
    }



    Eigen::VectorXd u_d, ud1;
    u_d.resize(N); ud1.resize(N);

//    u_d = Qx.colPivHouseholderQr().solve(gx);
    u_d = Qx.colPivHouseholderQr().solve(gx);

    ud1 = Qx.inverse()*gx;

    Eigen::Vector3d x_dd, xdd1;

    x_dd = A_*x_0 +B_*u_d(0);

    xdd1 = A_*y_0 + B_*ud1(0);



    file[36]<<walking_tick_<<"\t"<<x_dd(0)<<"\t"<<x_dd(1)<<"\t"<<x_dd(2)<<"\t"<<u_d(0)
           <<"\t"<<xdd1(0)<<"\t"<<xdd1(1)<<"\t"<<xdd1(2)<<"\t"<<ud1(0)<<endl;

    x_p1_ = x_dd;
//    y_p1_ = y_d1_;
    y_p1_ = xdd1;


    SupportfootComUpdate(x_p1_,y_d1_,x_p1_,y_d1_);

    if(com_control_mode_ == true)
    {
      com_desired_(0) = x_dd(0);
//      com_desired_(1) = y_d1_(0);
      com_desired_(2) = DyrosMath::cubic(walking_tick_, t_start_, t_start_real_, pelv_support_init_.translation()(2), pelv_suppprt_start_.translation()(2), 0, 0);

      com_dot_desired_(0) = x_dd(1);
//      com_dot_desired_(1) = y_d1_(1);
      com_dot_desired_(2) = DyrosMath::cubicDot(walking_tick_, t_start_, t_start_real_, pelv_support_init_.translation()(2), pelv_suppprt_start_.translation()(2), 0, 0, hz_);

    }
    else
    {
      com_desired_(0) = x_dd(0);
//      com_desired_(1) = y_d1_(0);
      com_desired_(2) = DyrosMath::cubic(walking_tick_, t_start_, t_start_real_, pelv_support_init_.translation()(2), pelv_suppprt_start_.translation()(2), 0, 0);

      com_dot_desired_(0) = x_dd(1);
//      com_dot_desired_(1) = y_d1_(1);
      com_dot_desired_(2) = DyrosMath::cubicDot(walking_tick_, t_start_, t_start_real_, pelv_support_init_.translation()(2), pelv_suppprt_start_.translation()(2), 0, 0, hz_);

    }

}

void WalkingController::IntrisicMPC(){

    int sample_N;
    double sample_delta;
    double Time_horizon;
    Time_horizon = 1.6*t_total_;

    sample_N = 20;
    sample_delta = Time_horizon/sample_N;

    if(walking_tick_ == 0)
        iMPCparameter(sample_N,sample_delta);


    Eigen::Vector3d x_0, y_0; // com position, com velocity, zmp position
    x_0.setZero(); y_0.setZero();

    if(walking_tick_ - zmp_start_time_ == 0 && current_step_num_ ==0){
        x_0(0) = com_support_init_(0);//-0.03;
        y_0(0) = com_support_init_(1);
    }
    else {
        x_0 = x_p_temp_;
        y_0 = y_p_temp_;
    }
    double w;
    w = sqrt(GRAVITY/zc_);

    double lambda;
    lambda = exp(-w*sample_delta);

    double start_time;

    if(current_step_num_ == 0)
      start_time = 0;
    else
      start_time = t_start_;


    Eigen::VectorXd foot_x, foot_y;
    footReferenceGenerator(foot_x,foot_y);

    Eigen::VectorXd fx_ref, fy_ref;
    fx_ref.resize(sample_N); fy_ref.resize(sample_N);

    for(int i=0;i<sample_N;i++){
        fx_ref(i) = foot_x(walking_tick_-start_time +sample_delta*i);
        fy_ref(i) = foot_y(walking_tick_-start_time +sample_delta*i);
    }
//    if(walking_tick_ == t_start_){
//        for(int i=0;i<sample_N;i++){
//            file[12]<<fx_ref(i)<<"\t";
//        }
//        file[12]<<endl;
//        for(int i=0;i<sample_N;i++){
//            file[12]<<fy_ref(i)<<"\t";
//        }
//        file[12]<<endl;

//    }

    SupportFootReference(foot_x,foot_y,sample_N,sample_delta);


    Eigen::MatrixXd temp_margin_x, temp_margin_y;
    calculateFootMargin2(temp_margin_x,temp_margin_y);

    Eigen::MatrixXd cst_x, cst_y;
    cst_x.resize(sample_N,2); cst_y.resize(sample_N,2);


    for(int i=0;i<sample_N;i++){
        cst_x(i,0) = temp_margin_x(walking_tick_-start_time+sample_delta*i,0);
        cst_x(i,1) = temp_margin_x(walking_tick_-start_time+sample_delta*i,1);

        cst_y(i,0) = temp_margin_y(walking_tick_-start_time+sample_delta*i,0);
        cst_y(i,1) = temp_margin_y(walking_tick_-start_time+sample_delta*i,1);
    }

    Eigen::VectorXd zmp_min, zmp_max,foot_length,foot_width;
    zmp_min.resize(2*sample_N); zmp_max.resize(2*sample_N);
    foot_length.resize(sample_N); foot_width.resize(sample_N);

    foot_length.setOnes(); foot_length *=0.15;
    foot_width.setOnes();  foot_width *= 0.075;



//    zmp_min.segment(0,sample_N) = fx_ref + cst_x.col(1);
//    zmp_min.segment(sample_N,sample_N) = fy_ref +cst_y.col(1);

//    zmp_max.segment(0,sample_N) = fx_ref + cst_x.col(0);
//    zmp_max.segment(sample_N,sample_N) = fy_ref +cst_y.col(0);


    Eigen::MatrixXd A_temp(2*sample_N+2,2*sample_N);

    A_temp.setZero();
    A_temp.block(0,0,sample_N,sample_N) = P_matrix_;
    A_temp.block(sample_N,0,1,sample_N) = Lambda_vector_;
    A_temp.block(sample_N+1,sample_N,sample_N,sample_N) = P_matrix_;
    A_temp.block(2*sample_N+1,sample_N,1,sample_N) = Lambda_vector_;

//    if(walking_tick_ == 0){
//        for(int i=0;i<2*sample_N+2;i++){
//            for(int j=0;j<2*sample_N;j++){
//                file[12]<<A_temp(i,j)<<"\t";
//            }
//            file[12]<<endl;
//        }
//    }


    double x_stability_constraint, y_stability_constraint;
    x_stability_constraint = x_0(0) + x_0(1)/w - x_0(2);
    y_stability_constraint = y_0(0) + y_0(1)/w - y_0(2);

    Eigen::VectorXd Min_bound, Max_bound;
    Min_bound.resize(2*sample_N+2); Max_bound.resize(2*sample_N+2);

    Min_bound.segment(0,sample_N) = foot_x -foot_length;//zmp_min.segment(0,sample_N);
    Min_bound(sample_N) = x_stability_constraint;
    Min_bound.segment(sample_N+1,sample_N) = foot_y -foot_width;//zmp_min.segment(sample_N,sample_N);
    Min_bound(2*sample_N +1) = y_stability_constraint;

    Max_bound.segment(0,sample_N) = foot_x + foot_length;//zmp_max.segment(0,sample_N);
    Max_bound(sample_N) = x_stability_constraint;
    Max_bound.segment(sample_N+1,sample_N) = foot_y + foot_width;;//zmp_max.segment(sample_N,sample_N);
    Max_bound(2*sample_N +1) = y_stability_constraint;

    int_t nV;
    nV = (int)sample_N;

    QProblem iMPC(2*nV,2*nV+2);
    Options op;

    int_t nWSR = 1000;

    op.initialStatusBounds = ST_LOWER;
//    op.numRefinementSteps = 1;
//    op.enableCholeskyRefactorisation = 1;
    op.setToMPC();

//    op.printLevel = PL_NONE;

    real_t Q_input[2*nV*2*nV],lbA[2*nV+2],ubA[2*nV+2], A_input[2*nV*2*(nV+1)],lbx[2*nV],ubx[2*nV],g_input[2*nV] ;

    Eigen::MatrixXd Q_iden;
    Q_iden.resize(2*sample_N,2*sample_N);
    Q_iden.setIdentity();


    for(int i=0;i<2*sample_N;i++){
        for(int j=0;j<2*sample_N;j++){
            Q_input[j*(int) 2*sample_N +i] = Q_iden(i,j);

        }


        g_input[i] = 0.0;

        lbx[i] = -10;
        ubx[i] = 10;
    }



    for(int i=0;i<2*sample_N+2;i++){
        for(int j=0;j<2*sample_N;j++){
//            A_input[j*(int) 2*(sample_N+1) +i] = A_temp(i,j);
            A_input[j*(int) 2*(sample_N+1) +i] = 0.0;
        }
        lbA[i] = Min_bound(i);
        ubA[i] = Max_bound(i);
//        lbA[i] = 0.0;
//        ubA[i] = 0.0;

        if(walking_tick_ == 0){
            file[14]<<Min_bound(i)<<"\t"<<Max_bound(i)<<endl;
        }
    }
//    for(int i=0;i<sample_N;i++){
//        lbA[i] = zmp_min(i) - x_0(2);
//        lbA[i+(int)sample_N] = zmp_min(sample_N +i) - y_0(2);

//        ubA[i] = zmp_max(i) - x_0(2);
//        ubA[i+(int)sample_N] = zmp_max(sample_N +i) - y_0(2);
//    }

//    lbA[2*sample_N] = x_stability_constraint;
//    ubA[2*sample_N] = x_stability_constraint;

//    lbA[2*sample_N+1] = y_stability_constraint;
//    ubA[2*sample_N+1] = y_stability_constraint;

    real_t opt_u[2*nV];

    iMPC.setOptions(op);

    iMPC.init(Q_input,g_input,A_input,lbx,ubx,lbA,ubA,nWSR);
    iMPC.getPrimalSolution(opt_u);

    iMPC.hotstart(g_input,lbx,ubx,lbA,ubA,nWSR);
    iMPC.getPrimalSolution(opt_u);

//    QProblemB test(2*nV);
//    test.setOptions(op);

//    test.init(Q_input,g_input,lbx,ubx,nWSR,0);
//    test.getPrimalSolution(opt_u);

//    test.hotstart(g_input,lbx,ubx,nWSR,0);
//    test.getPrimalSolution(opt_u);



    Eigen::Vector3d X_dot,Y_dot,b_vec;
    Eigen::Matrix3d A_mat;
    A_mat.setZero();
    A_mat(0,1) = 1.0;
    A_mat(1,0) = pow(w,2); A_mat(1,2) = -pow(w,2);
    b_vec.setZero(); b_vec(2) = 1.0;

    X_dot = A_mat*x_0 + b_vec*opt_u[0];
    Y_dot = A_mat*y_0 + b_vec*opt_u[sample_N];

    x_p_temp_ = x_0 + X_dot*sample_delta/hz_;
    y_p_temp_ = y_0 + Y_dot*sample_delta/hz_;

//    file[22]<<walking_tick_<<"\t"<<x_p_temp_(0)<<"\t"<<x_p_temp_(1)<<"\t"<<x_p_temp_(2)<<"\t"<<y_p_temp_(0)<<"\t"<<y_p_temp_(1)<<"\t"<<y_p_temp_(2)<<"\t"<<opt_u[0]<<"\t"<<opt_u[sample_N]<<endl;
//    file[21]<<walking_tick_<<"\t"<<lbA[0]<<"\t"<<lbA[nV+1]<<"\t"<<ubA[0]<<"\t"<<ubA[nV+1]<<endl;
}
void WalkingController::iMPCparameter(int N, double delta){


//    cout<<"delta : "<<delta<<endl;

    p_vector_.resize(N);
    p_vector_.setOnes();

    P_matrix_.resize(N,N);
    P_matrix_.setOnes();
    P_matrix_*= delta;
    P_matrix_ = DyrosMath::LowerTriangularMatrix(P_matrix_);

    Lambda_vector_.resize(N);

    double w;
    w = sqrt(GRAVITY/zc_);

    double lambda;
    lambda = exp(-w*delta/hz_);

    for(int i=0;i<N;i++){
        Lambda_vector_(i) = pow(lambda,i);
    }
    double lambda_gain;
    lambda_gain = (1-lambda)/(w*(1-pow(lambda,N)));

    Lambda_vector_ *=lambda_gain;

//    cout<<"P matrix  "<<endl<<P_matrix_<<endl<<"p vextor "<<endl<<p_vector_<<endl;
}
void WalkingController::SupportFootReference(Eigen::VectorXd& support_x, Eigen::VectorXd& support_y, int N, double delta)
{
    unsigned int norm_size = 0;
    if(current_step_num_ >= total_step_num_ -3)
        norm_size = (t_last_-t_start_+1)*(total_step_num_-current_step_num_) +20*hz_;
    else
        norm_size = (t_last_ - t_start_ +1)*3;

    if(current_step_num_ == 0)
        norm_size = norm_size + t_temp_ +1;

//    support_x.resize(norm_size);
//    support_y.resize(norm_size);

    support_x.resize(N); support_y.resize(N);


    //// checking current support foot and next support foot ////
    int num_current_foot, num_next_foot, num_last_foot,support_num_max;

    support_num_max = t_total_/delta;
    num_current_foot = (t_last_- walking_tick_)/delta +1;

    if(current_step_num_ == 0){
      if(num_current_foot>N)
        num_current_foot = N;
    }
    else {
        if(num_current_foot>support_num_max)
            num_current_foot = support_num_max;
    }
    num_next_foot = N- num_current_foot;
    if(num_next_foot>support_num_max)
        num_next_foot = support_num_max;
    num_last_foot = N-num_next_foot-num_current_foot;
    if(num_last_foot <0)
        num_last_foot = 0;


    if(current_step_num_ ==0){
        for(int i=0;i<N;i++){
            if(i<num_current_foot){
                support_x(i) = com_support_init_(0) + com_offset_(0);
                support_y(i) = com_support_init_(1) + com_offset_(1);
            }
            else if(i<num_current_foot + num_next_foot){
                support_x(i) = supportfoot_support_init_offset_(0);
                support_y(i) = supportfoot_support_init_offset_(1);
            }
            else {
                support_x(i) = foot_step_support_frame_(0,0);
                support_y(i) = foot_step_support_frame_(0,1);
            }
        }
    }
    else{
        for(int i=0;i<N;i++){
            if(i<num_current_foot){
                support_x(i) = foot_step_support_frame_(current_step_num_-1,0);
                support_x(i) = foot_step_support_frame_(current_step_num_-1,1);
            }
            else if(i<num_current_foot + num_next_foot){
                support_x(i) = foot_step_support_frame_(current_step_num_,0);
                support_x(i) = foot_step_support_frame_(current_step_num_,1);
            }
            else{
                support_x(i) = foot_step_support_frame_(current_step_num_+1,0);
                support_x(i) = foot_step_support_frame_(current_step_num_+1,1);
            }

        }
    }

    file[12]<<walking_tick_;
    for(int i=0;i<N;i++)
        file[12]<<"\t"<<support_x(i);
    file[12]<<endl;
    file[12]<<walking_tick_;
    for(int i=0;i<N;i++)
        file[12]<<"\t"<<support_y(i);
    file[12]<<endl;

//    file[12]<<walking_tick_<<"\t"<<num_current_foot<<"\t"<<num_next_foot<<"\t"<<num_last_foot<<endl;
}
void WalkingController::GetConstraintMatrix(Eigen::Matrix<double, 23, 12> &A_dsp1, Eigen::Matrix<double, 18, 12> &A_lifting, Eigen::Matrix<double, 23, 12> &A_landing, Eigen::Matrix<double, 24, 12> &A_dsp2,
                                            Eigen::VectorXd &lbA_dsp1, Eigen::VectorXd &ubA_dsp1, Eigen::VectorXd &lbA_lifting, Eigen::VectorXd &ubA_lifting, Eigen::VectorXd &lbA_landing, Eigen::VectorXd &ubA_landing, Eigen::VectorXd &lbA_dsp2, Eigen::VectorXd &ubA_dsp2)
{

    A_dsp1.setZero(); A_lifting.setZero(); A_landing.setZero(); A_dsp2.setZero();

    Eigen::MatrixXd Iden_12;
    Iden_12.resize(12,12);
    Iden_12.setIdentity();

    double kv;
    kv = 0.05;

    kv = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,0.0,0.02,0.0,0.0);
    double torelance = 100;
    if(foot_step_(current_step_num_,6)== 1)//left foot support
    {
        if(l_ft_(4) <0)
            pushing_force_ = kv*l_ft_(4)/step_length_x_;
    }
    else{//right foot support
        if(r_ft_(4) <0)
            pushing_force_ = kv*r_ft_(4)/step_length_x_;
    }
    if(walking_tick_ <=600)// || walking_tick_ >= t_start_real_ + t_double1_
        pushing_force_ = 0;
    pushing_force_ = 0;

//    pushing_force_ = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_ + t_double1_,0.0,-0.4,0.0,0.0);
    /////////////////////////////////////////
    ///    constraint for task space      ///
    /////////////////////////////////////////

   if(foot_step_(current_step_num_,6) == 1)// left foot support
    {
       // constraint for toe contact ///
        A_dsp1.block<6,6>(0,0) = current_leg_jacobian_l_;

        A_dsp1.block<4,6>(6,6) = current_right_toe_jacobian_.block<4,6>(0,0);
        A_dsp1.block<1,6>(10,6) = current_right_toe_jacobian_.block<1,6>(5,0);


        lbA_dsp1.segment<6>(0) = lp_ + 5*lp_clik_;

        lbA_dsp1.segment<4>(6) = rtoe_p_.segment<4>(0) + 5*rtoe_clik_.segment<4>(0);
        lbA_dsp1(10) = rtoe_p_(5) + 5*rtoe_clik_(5);
//        lbA_dsp1(8) += pushing_force;
        lbA_dsp1(8) -= torelance;

        ubA_dsp1.segment<6>(0) = lp_ + 5*lp_clik_;

        ubA_dsp1.segment<4>(6) = rtoe_p_.segment<4>(0) + 5*rtoe_clik_.segment<4>(0);
        ubA_dsp1(10) = rtoe_p_(5) + 5*rtoe_clik_(5);

//        ubA_dsp1(8) += torelance;
        ubA_dsp1(8) += pushing_force_;


    }
    else if(foot_step_(current_step_num_,6) == 0)//right foot support
    {
       // constraint for toe contact ///
        A_dsp1.block<4,6>(0,0) = current_left_toe_jacobian_.block<4,6>(0,0);
        A_dsp1.block<1,6>(4,0) = current_left_toe_jacobian_.block<1,6>(5,0);

        A_dsp1.block<6,6>(5,6) = current_leg_jacobian_r_;

        lbA_dsp1.segment<4>(0) = ltoe_p_.segment<4>(0) + 5*ltoe_clik_.segment<4>(0);
        lbA_dsp1(4) = ltoe_p_(5) + 5*ltoe_clik_(5);
//        lbA_dsp1(2) += pushing_force;
        lbA_dsp1(2) -= torelance;

        lbA_dsp1.segment<6>(5) = rp_ + 5*rp_clik_;

        ubA_dsp1.segment<4>(0) = ltoe_p_.segment<4>(0) + 5*ltoe_clik_.segment<4>(0);
        ubA_dsp1(4) = ltoe_p_(5) + 5*ltoe_clik_(5);

//        ubA_dsp1(2) += torelance;
        ubA_dsp1(2) += pushing_force_;

        ubA_dsp1.segment<6>(5) = rp_ + 5*rp_clik_;
    }

   //constraint for support foot contact during lifting
    if(foot_step_(current_step_num_,6) == 1)// left foot support
    {
        A_lifting.block<6,6>(0,0) = current_leg_jacobian_l_;
        lbA_lifting.segment<6>(0) = lp_ + 5*lp_clik_;
        ubA_lifting.segment<6>(0) = lp_ + 5*lp_clik_;

    }
    else if(foot_step_(current_step_num_,6) == 0)
    {
        A_lifting.block<6,6>(0,6) = current_leg_jacobian_r_;

        lbA_lifting.segment<6>(0) = rp_ + 5*rp_clik_;
        ubA_lifting.segment<6>(0) = rp_ + 5*rp_clik_;
    }

    //constraint for heel
    int height_gain =10;
//    if(current_step_num_ >= total_step_num_-2)
//        height_gain = 5.0;
    if(foot_step_(current_step_num_,6) == 1)// left foot support
    {
        A_landing.block<6,6>(0,0) = current_leg_jacobian_l_;

        A_landing.block<4,6>(6,6) = current_right_heel_jacobian_.block<4,6>(0,0);
        A_landing.block<1,6>(10,6) = current_right_heel_jacobian_.block<1,6>(5,0);

        lbA_landing.segment<6>(0) = lp_ + 5*lp_clik_;
        lbA_landing.segment<4>(6) = rheel_p_.segment<4>(0) + 5*rheel_clik_.segment<4>(0);
        lbA_landing(8) = rheel_p_(2) + height_gain*rheel_clik_(2);
        lbA_landing(10) = rheel_p_(5) + 5*rheel_clik_(5);

        ubA_landing.segment<6>(0) = lp_ + 5*lp_clik_;

        ubA_landing.segment<4>(6) = rheel_p_.segment<4>(0) + 5*rheel_clik_.segment<4>(0);
        ubA_landing(8) = rheel_p_(2) + height_gain*rheel_clik_(2);
        ubA_landing(10) =  rheel_p_(5) + 5*rheel_clik_(5);

//        lbA_landing.segment<4>(6) = ubA_landing.segment<4>(6);
//        lbA_landing(10) = ubA_landing(10);
    }
    else if(foot_step_(current_step_num_,6) == 0)
    {
        A_landing.block<4,6>(0,0) = current_left_heel_jacobian_.block<4,6>(0,0);
        A_landing.block<1,6>(4,0) = current_left_heel_jacobian_.block<1,6>(5,0);

        A_landing.block<6,6>(5,6) = current_leg_jacobian_r_;

        lbA_landing.segment<4>(0) = lheel_p_.segment<4>(0) + 5*lheel_clik_.segment<4>(0);
        lbA_landing(2) = lheel_p_(2) + height_gain*lheel_clik_(2);
        lbA_landing(4) = lheel_p_(5) + 5*lheel_clik_(5);

        lbA_landing.segment<6>(5) = rp_ + 5*rp_clik_;

        ubA_landing.segment<4>(0) = lheel_p_.segment<4>(0) + 5*lheel_clik_.segment<4>(0);
        ubA_landing(2) = lheel_p_(2) + height_gain*lheel_clik_(2);
        ubA_landing(4) = lheel_p_(5) + 5*lheel_clik_(5);
        ubA_landing.segment<6>(5) = rp_ + 5*rp_clik_;
    }

    // constraint for full contact
    A_dsp2.block<6,6>(0,0) = current_leg_jacobian_l_;
    A_dsp2.block<6,6>(6,6) = current_leg_jacobian_r_;

    if(foot_step_(current_step_num_,6) == 1)//left foot support
    {
        lbA_dsp2.segment<6>(0) = lp_ + 3*lp_clik_;
        lbA_dsp2.segment<6>(6) = rp_ + 5*rp_clik_;

        ubA_dsp2.segment<6>(0) = lp_ + 3*lp_clik_;
        ubA_dsp2.segment<6>(6) = rp_ + 5*rp_clik_;
    }
    else if(foot_step_(current_step_num_,6) == 0){
        lbA_dsp2.segment<6>(0) = lp_ + 5*lp_clik_;
        lbA_dsp2.segment<6>(6) = rp_ + 3*rp_clik_;

        ubA_dsp2.segment<6>(0) = lp_ + 5*lp_clik_;
        ubA_dsp2.segment<6>(6) = rp_ + 3*rp_clik_;
    }



//    // test for heel control

//    A_dsp2.block<6,6>(0,0) = current_left_heel_jacobian_;
//    A_dsp2.block<6,6>(6,6) = current_right_heel_jacobian_;

//    lbA_dsp2.segment<6>(0) = lheel_p_ + 2*lheel_clik_;
//    lbA_dsp2.segment<6>(6) = rheel_p_ + 2*rheel_clik_;

//    ubA_dsp2.segment<6>(0) = lheel_p_ + 2*lheel_clik_;
//    ubA_dsp2.segment<6>(6) = rheel_p_ + 2*rheel_clik_;

    /////////////////////////////////////////
    ///    constraint for joint limit     ///
    /////////////////////////////////////////

    A_dsp1.block<12,12>(11,0) = Iden_12;

    for(int i=0;i<12;i++){
        lbA_dsp1(i+11) = (q_leg_min_(i) - desired_q_not_compensated_(i))*hz_;
        ubA_dsp1(i+11) = (q_leg_max_(i) - desired_q_not_compensated_(i))*hz_;
    }


    A_lifting.block<12,12>(6,0) = Iden_12;

    for(int i=0;i<12;i++){
        lbA_lifting(i+6) = (q_leg_min_(i) - desired_q_not_compensated_(i))*hz_;
        ubA_lifting(i+6) = (q_leg_max_(i) - desired_q_not_compensated_(i))*hz_;
    }

    A_landing.block<12,12>(11,0) = Iden_12;

    for(int i=0;i<12;i++){
        lbA_landing(i+11) = (q_leg_min_(i) - desired_q_not_compensated_(i))*hz_;
        ubA_landing(i+11) = (q_leg_max_(i) - desired_q_not_compensated_(i))*hz_;
    }

    A_dsp2.block<12,12>(12,0) = Iden_12;

    for(int i=0;i<12;i++){
        lbA_dsp2(i+12) = (q_leg_min_(i) - desired_q_not_compensated_(i))*hz_;
        ubA_dsp2(i+12) = (q_leg_max_(i) - desired_q_not_compensated_(i))*hz_;
    }



}
void WalkingController::getDesiredVelocity(Eigen::Vector6d &lp, Eigen::Vector6d &rp, Eigen::Vector6d &lp_toe, Eigen::Vector6d &rp_toe, Eigen::Vector6d &lp_heel, Eigen::Vector6d &rp_heel,
                                           Eigen::Vector6d &lp_clik, Eigen::Vector6d &rp_clik, Eigen::Vector6d &lp_toe_clik, Eigen::Vector6d &rp_toe_clik, Eigen::Vector6d &lp_heel_clik, Eigen::Vector6d &rp_heel_clik){

    Eigen::Vector6d cubic_xr, cubic_xl;
    Eigen::Vector3d r_leg_phi, l_leg_phi;

    lp.setZero(); rp.setZero(), cubic_xr.setZero(), cubic_xl.setZero();

    /////for calculating desired ankle velocity
    if(walking_tick_ == 0 || walking_tick_ == t_start_ || walking_tick_ == t_start_+t_total_-t_double2_-t_rest_last_  ){
        lp.topRows<3>() = (-lfoot_float_current_.translation()+lfoot_trajectory_float_.translation());
        rp.topRows<3>() = (-rfoot_float_current_.translation()+rfoot_trajectory_float_.translation());
    }
    else{
      lp.topRows<3>() = (-pre_lfoot_trajectory_float_.translation()+lfoot_trajectory_float_.translation());
      rp.topRows<3>() = (-pre_rfoot_trajectory_float_.translation()+rfoot_trajectory_float_.translation());
    }


    for(int i=0;i<3;i++)
    {
      cubic_xl(i) = lfoot_trajectory_float_.translation()(i);
      cubic_xl(i+3) = lfoot_trajectory_euler_float_(i);
    }

    for(int i=0;i<3;i++)
    {
      cubic_xr(i) = rfoot_trajectory_float_.translation()(i);
      cubic_xr(i+3) = rfoot_trajectory_euler_float_(i);
    }

    if(walking_tick_ ==0 || walking_tick_ == t_start_ ){
         l_leg_phi = DyrosMath::legGetPhi(lfoot_float_current_, lfoot_float_init_, cubic_xl);
         r_leg_phi = DyrosMath::legGetPhi(rfoot_float_current_, rfoot_float_init_, cubic_xr);
    }
//    else if(walking_tick_ == t_start_ + t_double1_){
//        l_leg_phi = DyrosMath::legGetPhi(lfoot_float_current_,lfoot_lifting_init_,cubic_xl);
//        r_leg_phi = DyrosMath::legGetPhi(rfoot_float_current_,rfoot_lifting_init_,cubic_xl);
//    }
//    else if(walking_tick_ > t_start_ + t_double1_ && walking_tick_ < t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0 ){
//        l_leg_phi = DyrosMath::legGetPhi(pre_lfoot_trajectory_float_,lfoot_lifting_init_,cubic_xl);
//        r_leg_phi = DyrosMath::legGetPhi(pre_rfoot_trajectory_float_,rfoot_lifting_init_,cubic_xl);
//    }
    else if(walking_tick_ >= t_start_ + t_double1_ && walking_tick_ < t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0)// SSP (half of swing, toe off and lifting up the foot)
    {
        if(walking_tick_ == t_start_ + t_double1_){
            if(foot_step_(current_step_num_,6) == 1)//left foot support
            {
                l_leg_phi = DyrosMath::legGetPhi(pre_lfoot_trajectory_float_, lfoot_float_init_, cubic_xl);
                r_leg_phi = DyrosMath::legGetPhi(rfoot_float_current_,rfoot_lifting_float_init_,cubic_xr);

            }
            else//right foot support
            {
                l_leg_phi = DyrosMath::legGetPhi(lfoot_float_current_,lfoot_lifting_float_init_,cubic_xl);
                r_leg_phi = DyrosMath::legGetPhi(pre_rfoot_trajectory_float_, rfoot_float_init_, cubic_xr);
            }
        }
        else{
            if(foot_step_(current_step_num_,6) == 1)//left foot support
            {
                l_leg_phi = DyrosMath::legGetPhi(pre_lfoot_trajectory_float_, lfoot_float_init_, cubic_xl);
                r_leg_phi = DyrosMath::legGetPhi(pre_rfoot_trajectory_float_,rfoot_lifting_float_init_,cubic_xr);
            }
            else//right foot support
            {
                l_leg_phi = DyrosMath::legGetPhi(pre_lfoot_trajectory_float_,lfoot_lifting_float_init_,cubic_xl);
                r_leg_phi = DyrosMath::legGetPhi(pre_rfoot_trajectory_float_, rfoot_float_init_, cubic_xr);
            }
        }
    }
    else if(walking_tick_ >= t_start_+t_total_-t_double2_-t_rest_last_)
    {
        if(walking_tick_ == t_start_+t_total_-t_double2_-t_rest_last_){
            if(foot_step_(current_step_num_,6) == 1)//left foot support
            {
                l_leg_phi = DyrosMath::legGetPhi(pre_lfoot_trajectory_float_, lfoot_float_init_, cubic_xl);
//                l_leg_phi = DyrosMath::legGetPhi(lfoot_float_current_,lfoot_DSP2_float_init_,cubic_xl);
                r_leg_phi = DyrosMath::legGetPhi(rfoot_float_current_,rfoot_DSP2_float_init_,cubic_xr);

            }
            else//right foot support
            {
                l_leg_phi = DyrosMath::legGetPhi(lfoot_float_current_,lfoot_DSP2_float_init_,cubic_xl);
//                r_leg_phi = DyrosMath::legGetPhi(rfoot_float_current_,rfoot_DSP2_float_init_,cubic_xr);
                r_leg_phi = DyrosMath::legGetPhi(pre_rfoot_trajectory_float_, rfoot_float_init_, cubic_xr);
            }
        }
        else{
            if(foot_step_(current_step_num_,6) == 1)//left foot support
            {
                l_leg_phi = DyrosMath::legGetPhi(pre_lfoot_trajectory_float_, lfoot_float_init_, cubic_xl);
//                l_leg_phi = DyrosMath::legGetPhi(pre_lfoot_trajectory_float_,lfoot_DSP2_float_init_,cubic_xr);
                r_leg_phi = DyrosMath::legGetPhi(pre_rfoot_trajectory_float_,rfoot_DSP2_float_init_,cubic_xr);
            }
            else//right foot support
            {
                l_leg_phi = DyrosMath::legGetPhi(pre_lfoot_trajectory_float_,lfoot_DSP2_float_init_,cubic_xl);
//                r_leg_phi = DyrosMath::legGetPhi(pre_rfoot_trajectory_float_,rfoot_DSP2_float_init_,cubic_xr);
                r_leg_phi = DyrosMath::legGetPhi(pre_rfoot_trajectory_float_, rfoot_float_init_, cubic_xr);
            }
        }
    }
    else{
        l_leg_phi = DyrosMath::legGetPhi(pre_lfoot_trajectory_float_, lfoot_float_init_, cubic_xl);
        r_leg_phi = DyrosMath::legGetPhi(pre_rfoot_trajectory_float_, rfoot_float_init_, cubic_xr);
    }

    lp.bottomRows<3>() = - l_leg_phi;
    rp.bottomRows<3>() = - r_leg_phi;



    // for calculating toe and heel desired velocity
    Eigen::Vector6d cubic_xr_toe, cubic_xl_toe, cubic_xl_heel, cubic_xr_heel;
    lp_toe.setZero(); rp_toe.setZero(); cubic_xl_toe.setZero(); cubic_xr_toe.setZero();
    lp_heel.setZero(); rp_heel.setZero(); cubic_xl_heel.setZero(); cubic_xr_heel.setZero();

    if(walking_tick_ == 0){
        lp_toe.topRows<3>() = (-ltoe_float_current_.translation() + ltoe_trajectory_float_.translation());
        rp_toe.topRows<3>() = (-rtoe_float_current_.translation() + rtoe_trajectory_float_.translation());

        lp_heel.topRows<3>() = (-lheel_float_current_.translation() + lheel_trajectory_float_.translation());
        rp_heel.topRows<3>() = (-rheel_float_current_.translation() + rheel_trajectory_float_.translation());
    }
    else{
        lp_toe.topRows<3>() = (-pre_ltoe_trajectory_.translation() + ltoe_trajectory_float_.translation());
        rp_toe.topRows<3>() = (-pre_rtoe_trajectory_.translation() + rtoe_trajectory_float_.translation());

        lp_heel.topRows<3>() = (-pre_lheel_trajectory_.translation() + lheel_trajectory_float_.translation());
        rp_heel.topRows<3>() = (-pre_rheel_trajectory_.translation() + rheel_trajectory_float_.translation());
    }

    for(int i =0;i<3;i++){
        cubic_xl_toe(i) = ltoe_trajectory_float_.translation()(i);
        cubic_xr_toe(i) = rtoe_trajectory_float_.translation()(i);

//        if(walking_tick_< t_start_+t_double1_){
//            cubic_xl_toe(i) =ltoe_float_init_.translation()(i);
//            cubic_xr_toe(i) =rtoe_float_init_.translation()(i);
//        }

        cubic_xl_toe(i+3) = ltoe_trajectory_euler_(i);
        cubic_xr_toe(i+3) = rtoe_trajectory_euler_(i);

//        cubic_xl_toe(i+3) = lfoot_float_euler_init_(i);
//        cubic_xr_toe(i+3) = rfoot_float_euler_init_(i);

        cubic_xl_heel(i) = lheel_trajectory_float_.translation()(i);
        cubic_xr_heel(i) = rheel_trajectory_float_.translation()(i);

        cubic_xl_heel(i+3) = lheel_trajectory_euler_(i);
        cubic_xr_heel(i+3) = rheel_trajectory_euler_(i);

    }

    Eigen::Vector3d l_toe_phi, r_toe_phi, l_heel_phi, r_heel_phi;
    if(walking_tick_ == 0){
        l_toe_phi  = DyrosMath::legGetPhi(ltoe_float_current_,ltoe_float_init_,cubic_xl_toe);
        r_toe_phi  = DyrosMath::legGetPhi(rtoe_float_current_,rtoe_float_init_,cubic_xr_toe);

        l_heel_phi = DyrosMath::legGetPhi(lheel_float_current_,lheel_float_init_,cubic_xl_heel);
        r_heel_phi = DyrosMath::legGetPhi(rheel_float_current_,rheel_float_init_,cubic_xr_heel);
    }
    else{
        l_toe_phi  = DyrosMath::legGetPhi(pre_ltoe_trajectory_,ltoe_float_init_,cubic_xl_toe);
        r_toe_phi  = DyrosMath::legGetPhi(pre_rtoe_trajectory_,rtoe_float_init_,cubic_xr_toe);

        l_heel_phi = DyrosMath::legGetPhi(pre_lheel_trajectory_,lheel_float_init_,cubic_xl_heel);
        r_heel_phi = DyrosMath::legGetPhi(pre_rheel_trajectory_,rheel_float_init_,cubic_xr_heel);
    }
    lp_toe.bottomRows<3>() = -l_toe_phi;
    rp_toe.bottomRows<3>() = -r_toe_phi;

    lp_heel.bottomRows<3>() = -l_heel_phi;
    rp_heel.bottomRows<3>() = -r_heel_phi;

    //////////////////////////////////////////////////
    /// for calculating CLIK parameter   //////
    /// /////////////////////////////////////////////


    lp_clik.topRows<3>() = -lfoot_float_current_.translation() + lfoot_trajectory_float_.translation();
    rp_clik.topRows<3>() = -rfoot_float_current_.translation() + rfoot_trajectory_float_.translation();

    for(int i=0;i<3;i++){
        cubic_xl(i) = lfoot_trajectory_float_.translation()(i);
        cubic_xl(i+3) = lfoot_trajectory_euler_float_(i);

        cubic_xr(i) = rfoot_trajectory_float_.translation()(i);
        cubic_xr(i+3) = rfoot_trajectory_euler_float_(i);
    }
    l_leg_phi = DyrosMath::legGetPhi(lfoot_float_current_,lfoot_float_init_,cubic_xl);
    r_leg_phi = DyrosMath::legGetPhi(rfoot_float_current_,rfoot_float_init_,cubic_xr);

    lp_clik.bottomRows<3>() = -l_leg_phi;
    rp_clik.bottomRows<3>() = -r_leg_phi;

    Eigen::Vector6d cubic_ltoe, cubic_rtoe, cubic_lheel, cubic_rheel;

    lp_toe_clik.topRows<3>() = -ltoe_float_current_.translation() + ltoe_trajectory_float_.translation();
    rp_toe_clik.topRows<3>() = -rtoe_float_current_.translation() + rtoe_trajectory_float_.translation();

    lp_heel_clik.topRows<3>() = -lheel_float_current_.translation() + lheel_trajectory_float_.translation();
    rp_heel_clik.topRows<3>() = -rheel_float_current_.translation() + rheel_trajectory_float_.translation();

    for(int i=0;i<3;i++){
        cubic_ltoe(i) = ltoe_trajectory_float_.translation()(i);
        cubic_ltoe(i+3) = ltoe_trajectory_euler_(i);

        cubic_rtoe(i) = rtoe_trajectory_float_.translation()(i);
        cubic_rtoe(i+3) = rtoe_trajectory_euler_(i);

        cubic_lheel(i) = lheel_trajectory_float_.translation()(i);
        cubic_lheel(i+3) = lheel_trajectory_euler_(i);

        cubic_rheel(i) = rheel_trajectory_float_.translation()(i);
        cubic_rheel(i+3) = rheel_trajectory_euler_(i);
    }

    l_toe_phi = DyrosMath::legGetPhi(ltoe_float_current_,ltoe_float_init_,cubic_ltoe);
    r_toe_phi = DyrosMath::legGetPhi(rtoe_float_current_,rtoe_float_init_,cubic_rtoe);

    l_heel_phi = DyrosMath::legGetPhi(lheel_float_current_,lheel_float_init_,cubic_lheel);
    r_heel_phi = DyrosMath::legGetPhi(rheel_float_current_,rheel_float_init_,cubic_rheel);

    lp_toe_clik.bottomRows<3>() = -l_toe_phi;
    rp_toe_clik.bottomRows<3>() = -r_toe_phi;

    lp_heel_clik.bottomRows<3>() = -l_heel_phi;
    rp_heel_clik.bottomRows<3>() = -r_heel_phi;


    lp *= 200;
    rp *= 200;

    lp_toe *= 200;
    rp_toe *= 200;

    lp_heel *= 200;
    rp_heel *= 200;


    pre_ltoe_trajectory_ = ltoe_trajectory_float_;
    pre_rtoe_trajectory_ = rtoe_trajectory_float_;

    pre_lheel_trajectory_ = lheel_trajectory_float_;
    pre_rheel_trajectory_ = rheel_trajectory_float_;
}
void WalkingController::getFoottrajectory_heel_toe(){
    Eigen::Vector6d target_swing_foot;

    if(walking_tick_==0)
        cout<<" Foot trajectory for Heel Toe "<<endl;

    for(int i=0;i<6;i++)
        target_swing_foot(i) = foot_step_support_frame_(current_step_num_,i);

    if(walking_tick_< t_start_ + t_double1_){// last half of DSP (toe of swing foot contact with ground)
        if(foot_step_(current_step_num_,6) == 1){// left foot support
            lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
            lfoot_trajectory_dot_support_.setZero();
    //        lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,lfoot_support_init_.translation()(2),0.0,0.0,0.0);
            lfoot_trajectory_support_.translation()(2) = 0.0;
            lfoot_trajectory_euler_support_(2) = lfoot_support_euler_init_(2);
            for(int i=0;i<2;i++)
                lfoot_trajectory_euler_support_(i) = DyrosMath::cubic(walking_tick_,t_start_,t_start_real_,lfoot_support_init_.translation()(i),0.0,0.0,0.0);

            rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
            rfoot_trajectory_support_.translation()(2) = 0.0;
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
            lfoot_trajectory_support_.translation()(2) = 0.0;
            lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;
            lfoot_trajectory_euler_support_(1) = 0.0;
        }

    }
    else if(walking_tick_ >= t_start_+t_double1_ && walking_tick_ < t_start_+t_total_-t_double2_-t_rest_last_) //lifting and landing of swing foot
    {
        if(foot_step_(current_step_num_,6) == 1) //Left foot support : Left foot is fixed at initial values, and Right foot is set to go target position
        {
          lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
          lfoot_trajectory_support_.translation()(2) = 0.0;
          lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;
          lfoot_trajectory_euler_support_.setZero();

          lfoot_trajectory_dot_support_.setZero();
          lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));

          // setting for Left supporting foot

          if(walking_tick_ < t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0) // the period for lifting the right foot
          {
            rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_+t_double1_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,0,foot_height_,0.0,0.0);
            rfoot_trajectory_dot_support_(2) = DyrosMath::cubicDot(walking_tick_,t_start_+t_double1_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,0,foot_height_,0.0,0.0,hz_);

            rfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_start_+t_double1_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,rfoot_lifting_support_euler_init_(1),0.0,0.0,0.0);

          } // the period for lifting the right foot
          else
          {
            rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_,foot_height_,target_swing_foot(2),0.0,0.0);
            rfoot_trajectory_dot_support_(2) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_,foot_height_,target_swing_foot(2),0.0,0.0,hz_);

//            rfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_start_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_-t_imp_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_,0.0,0.0,0.0,0.0);
            rfoot_trajectory_euler_support_(1) = 0.0;
          } // the period for putting the right foot

          rfoot_trajectory_euler_support_(0) = DyrosMath::cubic(walking_tick_,t_start_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_,0.0,target_swing_foot(0+3),0.0,0.0);
          rfoot_trajectory_dot_support_(0+3) = DyrosMath::cubicDot(walking_tick_,t_start_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_,0.0,target_swing_foot(0+3),0.0,0.0,hz_);

          for(int i=0; i<2; i++)
          {
            rfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_start_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_,rfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0);
            rfoot_trajectory_dot_support_(i) = DyrosMath::cubicDot(walking_tick_,t_start_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_,rfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0,hz_);
          }

          rfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_,t_start_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_,rfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0);
          rfoot_trajectory_dot_support_(5) = DyrosMath::cubicDot(walking_tick_,t_start_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_,rfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0,hz_);
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


          if(walking_tick_ < t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0)
          {

            lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_+t_double1_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,0,foot_height_,0.0,0.0);
            lfoot_trajectory_dot_support_(2) = DyrosMath::cubicDot(walking_tick_,t_start_+t_double1_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,0,foot_height_,0.0,0.0,hz_);


            lfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_start_+t_double1_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,lfoot_lifting_support_euler_init_(1),0.0,0.0,0.0);
//            lfoot_trajectory_dot_support_(4) = DyrosMath::cubicDot(walking_tick_,t_start_+t_double1_,t_start_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,0.0,ankle_temp,0.0,0.0,hz_);
          }
          else
          {
//            lfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_start_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_,0.0,0.0,0.0,0.0);
            lfoot_trajectory_euler_support_(1) = 0.0;
//            lfoot_trajectory_dot_support_(4) = DyrosMath::cubicDot(walking_tick_,t_start_+t_total_-t_rest_last_-t_double2_,t_start_+t_total_-t_rest_last_,ankle_temp,0.0,0.0,0.0,hz_);

            lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_,foot_height_,target_swing_foot(2),0.0,0.0);
            lfoot_trajectory_dot_support_(2) = DyrosMath::cubicDot(walking_tick_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_,foot_height_,target_swing_foot(2),0.0,0.0,hz_);
          }

          lfoot_trajectory_euler_support_(0) = DyrosMath::cubic(walking_tick_,t_start_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_,0.0,target_swing_foot(0+3),0.0,0.0);
          lfoot_trajectory_dot_support_(0+3) = DyrosMath::cubicDot(walking_tick_,t_start_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_,0.0,target_swing_foot(0+3),0.0,0.0,hz_);

          for(int i=0; i<2; i++)
          {
            lfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_start_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_,lfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0);
            lfoot_trajectory_dot_support_(i) = DyrosMath::cubicDot(walking_tick_,t_start_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_,lfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0,hz_);
          }

          lfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_,t_start_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_,lfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0);
          lfoot_trajectory_dot_support_(5) = DyrosMath::cubicDot(walking_tick_,t_start_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_,lfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0,hz_);
        }

    }
    else// first half of DSP (landing foot goes to full contact
    {
            if(foot_step_(current_step_num_,6) == 1) // left foot support
            {
              lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
              lfoot_trajectory_support_.translation()(2) = 0.0;
              lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;
              lfoot_trajectory_euler_support_(0) = 0.0;
              lfoot_trajectory_euler_support_(1) = 0.0;
        //      lfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_double2_-t_rest_last_,t_start_+t_total_-1,lfoot_support_euler_(1),lfoot_support_euler_init_(1),0.0,0.0);

              lfoot_trajectory_dot_support_.setZero();

              for(int i=0; i<3; i++)
              {
//                rfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
                  rfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_double2_-t_rest_last_,t_start_+t_total_-1,rfoot_DSP2_support_init_.translation()(i),target_swing_foot(i),0.0,0.0);
                rfoot_trajectory_euler_support_(i) = target_swing_foot(i+3);
              }
              rfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_double2_-t_rest_last_,t_start_+t_total_-1,rfoot_dsp2_support_euler_init_(1),0.0,0.0,0.0);
              rfoot_trajectory_dot_support_.setZero();

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

              for(int i=0; i<3; i++)
              {
//                lfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
                  lfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_double2_-t_rest_last_,t_start_+t_total_-1,lfoot_DSP2_support_init_.translation()(i),target_swing_foot(i),0.0,0.0);
                lfoot_trajectory_euler_support_(i) = target_swing_foot(i+3);
              }
              lfoot_trajectory_euler_support_(1) = DyrosMath::cubic(walking_tick_,t_start_+t_total_-t_double2_-t_rest_last_,t_start_+t_total_-1,lfoot_dsp2_support_euler_init_(1),0.0,0.0,0.0);
              lfoot_trajectory_dot_support_.setZero();
            }

    }

    lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
    rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
}

}




