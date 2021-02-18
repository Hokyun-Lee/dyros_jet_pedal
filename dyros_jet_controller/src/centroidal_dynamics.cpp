#include "dyros_jet_controller/dyros_jet_model.h"
#include "dyros_jet_controller/walking_controller.h"
#include "cvxgen_6_8_0/cvxgen/solver.h"
#include <chrono>
#include <stdio.h>


namespace dyros_jet_controller
{
void WalkingController::Relative_link_position(Eigen::Isometry3d* transform_matrix){
    ////////////////////////////
    //// calculating the distance between adjacent two joint according to tree structure
    ///

//    relative_distance_[RA_BEGIN] = transform_matrix[22].translation();//_T_RArm_global[0].translation();
//    relative_distance_[LA_BEGIN] = transform_matrix[15].translation();//_T_LArm_global[0].translation();
//    relative_distance_[RF_BEGIN] = transform_matrix[7].translation();
//    relative_distance_[LF_BEGIN] = transform_matrix[1].translation();
    relative_distance_[RA_BEGIN] = transform_matrix[RA_LINK].translation();//RA_BEGIN
    relative_distance_[LA_BEGIN] = transform_matrix[LA_LINK].translation();//LA_BEGIN
    relative_distance_[RF_BEGIN] = transform_matrix[RF_LINK].translation(); // RF_BEGIN
    relative_distance_[LF_BEGIN] = transform_matrix[LF_LINK].translation();//LF_BEGIN


    //cout<<"check relative distance  ra begin : "<<_relative_distance[RA_BEGIN]<<endl;
    for(int i=1;i<7;i++){
       // relative_distance_[RA_BEGIN+i] = transform_matrix[RA_LINK+i].translation() - relative_distance_[RA_BEGIN + i-1];//_T_RArm_global[i].translation() - relative_distance_[RA_BEGIN + i-1];//Right arm
       // relative_distance_[LA_BEGIN+i] = transform_matrix[LA_LINK+i].translation() - relative_distance_[LA_BEGIN + i-1];// Left arm
       relative_distance_[RA_BEGIN+i] = transform_matrix[RA_LINK+i].translation() - transform_matrix[RA_LINK + i-1].translation();
       relative_distance_[LA_BEGIN+i] = transform_matrix[LA_LINK+i].translation() - transform_matrix[LA_LINK + i-1].translation();
    }

    for(int i=1;i<6;i++){
//        relative_distance_[RF_BEGIN+i] = transform_matrix[RF_LINK+i].translation() - relative_distance_[RF_BEGIN + i-1];//Right foot
//        relative_distance_[LF_BEGIN+i] = transform_matrix[LF_LINK+i].translation() - relative_distance_[LF_BEGIN + i-1];//Left foot
        relative_distance_[RF_BEGIN +i] = transform_matrix[RF_LINK+i].translation() - transform_matrix[RF_LINK + i-1].translation();
        relative_distance_[LF_BEGIN +i] = transform_matrix[LF_LINK+i].translation() - transform_matrix[LF_LINK + i-1].translation();
    }

    relative_distance_[WA_BEGIN] = transform_matrix[WA_LINK].translation();
    relative_distance_[WA_BEGIN+1] = transform_matrix[WA_LINK+1].translation() - transform_matrix[WA_LINK].translation();

//    if(walking_tick_ ==0){
//        cout<<"relative distance "<<endl;
//        for(int i=0;i<29;i++)
//            cout<<i<<"th relative distance "<<endl<<relative_distance_[i]<<endl;
//    }

}
void WalkingController::relative_link_trans_matrix(Eigen::Isometry3d* transform_matrix){
    /// calculating transform matrix between adjecnt two joints
    ///
    ///     Based on the tree structure,
    ///     RA          LA
    ///        \       /
    ///            WA
    ///            |
    ///           Base
    ///           /   \\
    ///        RLEG   LLEG
    ///   relative_link_transform[i]  is transform matrix from [i-1] frame to [i] frame.
    ///   XX_BEGIN  is order of joint number and it starts from left hip yaw joint
    ///  XX_LINK is order of link number and it starts from pelvis link
    ///  relative_link_transform[i].linear()  = transform_matrix[i-1].linear().transpose() * transform_matrix[i].linear();
    ///  relative_link_transform[i].translation() = transform_matrix[i-1].linear().transpose()*transform_matrix[i].translation() - transform_matrix[i-1].linear().transpose()*transform_matrix[i-1].translation();



    relative_link_transform_[RF_BEGIN] = transform_matrix[RF_LINK];
    relative_link_transform_[LF_BEGIN] = transform_matrix[LF_LINK];
    relative_link_transform_[WA_BEGIN] = transform_matrix[WA_LINK];

    // for leg joint spatial transform
    for(int i=1;i<6;i++){
        relative_link_transform_[RF_BEGIN+i].linear() = (transform_matrix[RF_LINK+i-1].linear().transpose())*transform_matrix[RF_LINK+i].linear();
        relative_link_transform_[RF_BEGIN+i].translation() = transform_matrix[RF_LINK+i-1].linear().transpose()*transform_matrix[RF_LINK+i].translation() - transform_matrix[RF_LINK+i-1].linear().transpose()*transform_matrix[RF_LINK+i-1].translation();

        relative_link_transform_[LF_BEGIN+i].linear() = (transform_matrix[LF_LINK+i-1].linear().transpose())*transform_matrix[LF_LINK+i].linear();
        relative_link_transform_[LF_BEGIN+i].translation() = transform_matrix[LF_LINK+i-1].linear().transpose()*transform_matrix[LF_LINK+i].translation() - transform_matrix[LF_LINK+i-1].linear().transpose()*transform_matrix[LF_LINK+i-1].translation();
    }

    // for waist and arm first joint

    relative_link_transform_[WA_BEGIN+1].linear() = (transform_matrix[WA_LINK].linear().transpose())*transform_matrix[WA_LINK+1].linear();
    relative_link_transform_[WA_BEGIN+1].translation() = transform_matrix[WA_LINK].linear().transpose()*transform_matrix[WA_LINK+1].translation() - transform_matrix[WA_LINK].linear().transpose()*transform_matrix[WA_LINK].translation();

    // for both arm first joint

    relative_link_transform_[RA_BEGIN].linear() = (transform_matrix[WA_LINK+1].linear().transpose())*transform_matrix[RA_LINK].linear();
    relative_link_transform_[RA_BEGIN].translation() = transform_matrix[WA_LINK+1].linear().transpose()*transform_matrix[RA_LINK].translation() - transform_matrix[WA_LINK+1].linear().transpose()*transform_matrix[WA_LINK+1].translation();

    relative_link_transform_[LA_BEGIN].linear() = (transform_matrix[WA_LINK+1].linear().transpose())*transform_matrix[LA_LINK].linear();
    relative_link_transform_[LA_BEGIN].translation() = transform_matrix[WA_LINK+1].linear().transpose()*transform_matrix[LA_LINK].translation() - transform_matrix[WA_LINK+1].linear().transpose()*transform_matrix[WA_LINK+1].translation();

    for(int i=1;i<7;i++){
        relative_link_transform_[RA_BEGIN+i].linear() = (transform_matrix[RA_LINK+i-1].linear().transpose())*transform_matrix[RA_LINK+i].linear();
        relative_link_transform_[RA_BEGIN+i].translation() = transform_matrix[RA_LINK+i-1].linear().transpose()*transform_matrix[RA_LINK+i].translation() - transform_matrix[RA_LINK+i-1].linear().transpose()*transform_matrix[RA_LINK+i-1].translation();

        relative_link_transform_[LA_BEGIN+i].linear() = (transform_matrix[LA_LINK+i-1].linear().transpose())*transform_matrix[LA_LINK+i].linear();
        relative_link_transform_[LA_BEGIN+i].translation() = transform_matrix[LA_LINK+i-1].linear().transpose()*transform_matrix[LA_LINK+i].translation() - transform_matrix[LA_LINK+i-1].linear().transpose()*transform_matrix[LA_LINK+i-1].translation();
    }

    for(int i=0;i<28;i++){
        relative_rotation_[i] = relative_link_transform_[i].linear();
        relative_distance_[i] = relative_link_transform_[i].translation();
    }
//    file[14]<<walking_tick_;
//    for(int i=0;i<28;i++){
//        file[14]<<"\t"<<relative_distance_[i](0)<<"\t"<<relative_distance_[i](1)<<"\t"<<relative_distance_[i](2);
//    }
//    file[14]<<endl;

//    if(walking_tick_ ==0){
//        cout<<"relative distance in link trans matrix function "<<endl;
//        for(int i=0;i<29;i++)
//            cout<<i<<"th relative distance "<<endl<<relative_distance_[i]<<endl;

//        for(int i=0;i<20;i++)
//            cout<<i<<"th relative rotation " <<endl<<relative_rotation_[i]<<endl;
//    }

}
void WalkingController::relative_link_trans_matrix2(Eigen::Isometry3d* transform_matrix){
    /// calculating transform matrix between adjecnt two joints
    ///
    ///     Based on the tree structure,
    ///     RA          LA
    ///        \       /
    ///            WA
    ///            |
    ///           Base
    ///           /   \\
    ///        RLEG   LLEG
    ///   relative_link_transform[i]  is transform matrix from [i-1] frame to [i] frame.
    ///   XX_BEGIN  is order of joint number and it starts from left hip yaw joint
    ///  XX_LINK is order of link number and it starts from pelvis link
    ///  relative_link_transform[i].linear()  = transform_matrix[i-1].linear().transpose() * transform_matrix[i].linear();
    ///  relative_link_transform[i].translation() = transform_matrix[i-1].linear().transpose()*transform_matrix[i].translation() - transform_matrix[i-1].linear().transpose()*transform_matrix[i-1].translation();


    link_from_prior_transform_[BASE_LINK] = transform_matrix[BASE_LINK];

    link_from_prior_transform_[RF_LINK].linear() = transform_matrix[BASE_LINK].linear().transpose()*transform_matrix[RF_LINK].linear();
    link_from_prior_transform_[RF_LINK].translation() = transform_matrix[BASE_LINK].linear().transpose()*transform_matrix[RF_LINK].translation() - transform_matrix[BASE_LINK].linear().transpose()*transform_matrix[BASE_LINK].translation();

    link_from_prior_transform_[LF_LINK].linear() = transform_matrix[BASE_LINK].linear().transpose()*transform_matrix[LF_LINK].linear();
    link_from_prior_transform_[LF_LINK].translation() = transform_matrix[BASE_LINK].linear().transpose()*transform_matrix[LF_LINK].translation() - transform_matrix[BASE_LINK].linear().transpose()*transform_matrix[BASE_LINK].translation();

    link_from_prior_transform_[WA_LINK].linear() = transform_matrix[BASE_LINK].linear().transpose()*transform_matrix[WA_LINK].linear();
    link_from_prior_transform_[WA_LINK].translation() = transform_matrix[BASE_LINK].linear().transpose()*transform_matrix[WA_LINK].translation() - transform_matrix[BASE_LINK].linear().transpose()*transform_matrix[BASE_LINK].translation();


    // for leg joint spatial transform
    for(int i=1;i<6;i++){
        link_from_prior_transform_[RF_LINK+i].linear() = (transform_matrix[RF_LINK+i-1].linear().transpose())*transform_matrix[RF_LINK+i].linear();
        link_from_prior_transform_[RF_LINK+i].translation() = transform_matrix[RF_LINK+i-1].linear().transpose()*transform_matrix[RF_LINK+i].translation() - transform_matrix[RF_LINK+i-1].linear().transpose()*transform_matrix[RF_LINK+i-1].translation();

        link_from_prior_transform_[LF_LINK+i].linear() = (transform_matrix[LF_LINK+i-1].linear().transpose())*transform_matrix[LF_LINK+i].linear();
        link_from_prior_transform_[LF_LINK+i].translation() = transform_matrix[LF_LINK+i-1].linear().transpose()*transform_matrix[LF_LINK+i].translation() - transform_matrix[LF_LINK+i-1].linear().transpose()*transform_matrix[LF_LINK+i-1].translation();
    }

    // for waist and arm first joint

    link_from_prior_transform_[WA_LINK+1].linear() = (transform_matrix[WA_LINK].linear().transpose())*transform_matrix[WA_LINK+1].linear();
    link_from_prior_transform_[WA_LINK+1].translation() = transform_matrix[WA_LINK].linear().transpose()*transform_matrix[WA_LINK+1].translation() - transform_matrix[WA_LINK].linear().transpose()*transform_matrix[WA_LINK].translation();

    // for both arm first joint

    link_from_prior_transform_[RA_LINK].linear() = (transform_matrix[WA_LINK+1].linear().transpose())*transform_matrix[RA_LINK].linear();
    link_from_prior_transform_[RA_LINK].translation() = transform_matrix[WA_LINK+1].linear().transpose()*transform_matrix[RA_LINK].translation() - transform_matrix[WA_LINK+1].linear().transpose()*transform_matrix[WA_LINK+1].translation();

    link_from_prior_transform_[LA_LINK].linear() = (transform_matrix[WA_LINK+1].linear().transpose())*transform_matrix[LA_LINK].linear();
    link_from_prior_transform_[LA_LINK].translation() = transform_matrix[WA_LINK+1].linear().transpose()*transform_matrix[LA_LINK].translation() - transform_matrix[WA_LINK+1].linear().transpose()*transform_matrix[WA_LINK+1].translation();

    for(int i=1;i<7;i++){
        link_from_prior_transform_[RA_LINK+i].linear() = (transform_matrix[RA_LINK+i-1].linear().transpose())*transform_matrix[RA_LINK+i].linear();
        link_from_prior_transform_[RA_LINK+i].translation() = transform_matrix[RA_LINK+i-1].linear().transpose()*transform_matrix[RA_LINK+i].translation() - transform_matrix[RA_LINK+i-1].linear().transpose()*transform_matrix[RA_LINK+i-1].translation();

        link_from_prior_transform_[LA_LINK+i].linear() = (transform_matrix[LA_LINK+i-1].linear().transpose())*transform_matrix[LA_LINK+i].linear();
        link_from_prior_transform_[LA_LINK+i].translation() = transform_matrix[LA_LINK+i-1].linear().transpose()*transform_matrix[LA_LINK+i].translation() - transform_matrix[LA_LINK+i-1].linear().transpose()*transform_matrix[LA_LINK+i-1].translation();
    }

    for(int i=0;i<29;i++){
        relative_link_rotation_[i] = link_from_prior_transform_[i].linear();
        relative_link_distance_[i] = link_from_prior_transform_[i].translation();
    }
    //    file[14]<<walking_tick_;
//    for(int i=0;i<28;i++){
//        file[14]<<"\t"<<relative_distance_[i](0)<<"\t"<<relative_distance_[i](1)<<"\t"<<relative_distance_[i](2);
//    }
//    file[14]<<endl;

//    if(walking_tick_ ==0){
//        cout<<"relative distance in link trans matrix function "<<endl;
//        for(int i=0;i<29;i++)
//            cout<<i<<"th relative distance "<<endl<<relative_distance_[i]<<endl;

//        for(int i=0;i<20;i++)
//            cout<<i<<"th relative rotation " <<endl<<relative_rotation_[i]<<endl;
//    }

}
void WalkingController::Relative_link_rotation(Eigen::Isometry3d* transform_matrix){
    ////////////////////////////
    //// calculating the rotation matrix between adjacent two joint according to tree structure
    ///

////    cout<<"relative rotation : "<<_relative_rotation[WA_BEGIN]<<endl;
////    cout<<"global rotation : "<<_T_Waist_global[0].linear()<<endl;
////    cout<<"global rotation transpoise = "<<_relative_rotation[WA_BEGIN].transpose()*_T_Waist_global[0].linear()<<endl;


    relative_rotation_[WA_BEGIN] = transform_matrix[WA_LINK].linear();
    relative_rotation_[RA_BEGIN] = transform_matrix[RA_LINK].linear();
    relative_rotation_[LA_BEGIN] = transform_matrix[LA_LINK].linear();
    relative_rotation_[RF_BEGIN] = transform_matrix[RF_LINK].linear();
    relative_rotation_[LF_BEGIN] = transform_matrix[LF_LINK].linear();

//    cout<<"relative rotation : "<<relative_rotation_[WA_BEGIN]<<endl;
//    cout<<"global rotation : "<<_T_Waist_global[0].linear()<<endl;
//    cout<<"global rotation transpoise = "<<relative_rotation_[WA_BEGIN].transpose()*_T_Waist_global[0].linear()<<endl;
    for(int i=1;i<7;i++){
        //relative_rotation_[RA_BEGIN+i] = relative_rotation_[RA_BEGIN + i-1].transpose()*transform_matrix[RA_LINK+i].linear();
        //relative_rotation_[LA_BEGIN+i] = relative_rotation_[LA_BEGIN + i-1].transpose()*transform_matrix[LA_LINK+i].linear();
        relative_rotation_[RA_BEGIN +i] = transform_matrix[RA_LINK + i-1].linear().transpose() * transform_matrix[RA_LINK + i].linear();
        relative_rotation_[LA_BEGIN +i] = transform_matrix[LA_LINK + i-1].linear().transpose() * transform_matrix[LA_LINK + i].linear();
    }

    for(int i=1;i<6;i++){
//        relative_rotation_[RF_BEGIN+i] = relative_rotation_[RF_BEGIN + i-1].transpose()*transform_matrix[RF_LINK+i].linear();
//        relative_rotation_[LF_BEGIN+i] = relative_rotation_[LF_BEGIN + i-1].transpose()*transform_matrix[LF_LINK+i].linear();
        relative_rotation_[RF_BEGIN +i] = transform_matrix[RF_LINK + i-1].linear().transpose() * transform_matrix[RF_LINK + i].linear();
        relative_rotation_[LF_BEGIN +i] = transform_matrix[LF_LINK + i-1].linear().transpose() * transform_matrix[LF_LINK + i].linear();
    }

    relative_rotation_[WA_BEGIN+1] = transform_matrix[WA_LINK].linear().transpose()*transform_matrix[WA_LINK+1].linear();

//    cout<<"relative rotation wa : "<<_relative_rotation[WA_BEGIN]<<endl;
//    cout<<"relative rotation WA+1 : "<<_relative_rotation[WA_BEGIN+1]<<endl;

//    if(walking_tick_ == 0 ){
//        cout<<"relative rotation "<<endl;
//        for(int i=0;i<28;i++){
//            cout<<i<<"th rotation matrix"<<endl<<relative_rotation_[i]<<endl;
//        }

//        cout<<endl;
//    }
}

void WalkingController::Spatial_transform(){
    //////////////////////// calculating spatial matrix //////////////
    ///
    ///       order : linear velocity and angular velocity
    ///    i^X_p(i)     [ R    RS(p)^T]
    ///                 [ 0       R ]
    Eigen::Matrix<double, 3,3> Skew_temp, Rotation_temp;
    Skew_temp.setIdentity();
    Rotation_temp.setIdentity();

    if(walking_tick_==0){
        for(int i=0;i<28;i++){
            Spatial_Matrix_[i].setZero();
        }
    }

    for(int i=0;i<28;i++){
        Skew_temp = DyrosMath::skew(relative_distance_[i]);
        Rotation_temp = relative_rotation_[i]*(Skew_temp.transpose());


        Spatial_Matrix_[i].block<3,3>(0,0) = relative_rotation_[i];
        Spatial_Matrix_[i].block<3,3>(0,3) = Rotation_temp;
        Spatial_Matrix_[i].block<3,3>(3,3) = relative_rotation_[i];

//        if(walking_tick_ ==0)
//        cout<<i<<"th spatial matrix "<<endl<<Spatial_Matrix_[i]<<endl;

    }
//    for(int i=0;i<28;i++){
//        for(int j=0;j<6;j++){
////            file[36]<<COM_Projection_Matrix_[i](j,0)<<"\t"<<COM_Projection_Matrix_[i](j,1)<<"\t"<<COM_Projection_Matrix_[i](j,2)<<"\t"<<COM_Projection_Matrix_[i](j,3)<<"\t"<<COM_Projection_Matrix_[i](j,4)<<"\t"<<COM_Projection_Matrix_[i](j,5)<<endl;
//            file[36]<<Spatial_Matrix_[i](j,0)<<"\t"<<Spatial_Matrix_[i](j,1)<<"\t"<<Spatial_Matrix_[i](j,2)<<"\t"<<Spatial_Matrix_[i](j,3)<<"\t"<<Spatial_Matrix_[i](j,4)<<"\t"<<Spatial_Matrix_[i](j,5)<<endl;
//            if(walking_tick_== 0){
//                cout<<Spatial_Matrix_[i](j,0)<<"\t"<<Spatial_Matrix_[i](j,1)<<"\t"<<Spatial_Matrix_[i](j,2)<<"\t"<<Spatial_Matrix_[i](j,3)<<"\t"<<Spatial_Matrix_[i](j,4)<<"\t"<<Spatial_Matrix_[i](j,5)<<endl;
//            }
//        }
//    }
}
void WalkingController::Spatial_link_transform(){
    //////////////////////// calculating spatial matrix //////////////
    ///
    ///       order : linear velocity and angular velocity
    ///    i^X_p(i)     [ R    RS(p)^T]
    ///                 [ 0       R ]
    Eigen::Matrix<double, 3,3> Skew_temp, Rotation_temp;
    Skew_temp.setIdentity();
    Rotation_temp.setIdentity();

    if(walking_tick_==0){
        for(int i=0;i<29;i++){
            Spatial_link_transform_[i].setZero();
        }
    }

    for(int i=0;i<29;i++){
        Skew_temp = DyrosMath::skew(relative_link_distance_[i]);
        Rotation_temp = relative_link_rotation_[i]*(Skew_temp.transpose());


        Spatial_link_transform_[i].block<3,3>(0,0) = relative_link_rotation_[i];
        Spatial_link_transform_[i].block<3,3>(0,3) = Rotation_temp;
        Spatial_link_transform_[i].block<3,3>(3,3) = relative_link_rotation_[i];

//        if(walking_tick_ ==0)
//        cout<<i<<"th spatial matrix "<<endl<<Spatial_Matrix_[i]<<endl;
    }
//    Spatial_Matrix_[28].setIdentity();
}
void WalkingController::Projection_Spatial_COM(){
    Eigen::Matrix<double, 3, 3> Skew_temp, Rotation_temp, Iden_3d;
    Skew_temp.setIdentity();
    Rotation_temp.setIdentity();
    Iden_3d.setIdentity();

    //Skew_temp = DyrosMath::skew(com_support_current_);
    Skew_temp = DyrosMath::skew(com_float_current_);
    //_COM_real_support

    if(walking_tick_ == 0)
        cout<<"com real position "<<com_float_current_<<endl;

//    for(int j=0;j<3;j++){
//        for(int k=0;k<3;k++){
//            COM_Projection_Matrix_[28](j,k) = Skew_temp(j,k);
//        }
//        COM_Projection_Matrix_[28](j+3,j) = 1.0;
//        COM_Projection_Matrix_[28](j,j+3) = 1.0;
//    }
    COM_Projection_Matrix_[28].block<3,3>(0,0).setIdentity();// = Iden_3d;
    COM_Projection_Matrix_[28].block<3,3>(0,3) = Skew_temp;
    COM_Projection_Matrix_[28].block<3,3>(3,0).setZero();
    COM_Projection_Matrix_[28].block<3,3>(3,3).setIdentity();

    COM_Projection_Matrix_[RF_BEGIN] = Spatial_Matrix_[RF_BEGIN]*COM_Projection_Matrix_[28];
    COM_Projection_Matrix_[LF_BEGIN] = Spatial_Matrix_[LF_BEGIN]*COM_Projection_Matrix_[28];
    COM_Projection_Matrix_[WA_BEGIN] = Spatial_Matrix_[WA_BEGIN]*COM_Projection_Matrix_[28];
    COM_Projection_Matrix_[WA_BEGIN+1] = Spatial_Matrix_[WA_BEGIN+1]*COM_Projection_Matrix_[WA_BEGIN];

    COM_Projection_Matrix_[RA_BEGIN] = Spatial_Matrix_[RA_BEGIN]*COM_Projection_Matrix_[WA_BEGIN+1];
    COM_Projection_Matrix_[LA_BEGIN] = Spatial_Matrix_[LA_BEGIN]*COM_Projection_Matrix_[WA_BEGIN+1];


    for(int i=1;i<7;i++){
        COM_Projection_Matrix_[RA_BEGIN+i] = Spatial_Matrix_[RA_BEGIN+i]*COM_Projection_Matrix_[RA_BEGIN+i-1];
        COM_Projection_Matrix_[LA_BEGIN+i] = Spatial_Matrix_[LA_BEGIN+i]*COM_Projection_Matrix_[LA_BEGIN+i-1];
    }

    for(int i=1;i<6;i++){
        COM_Projection_Matrix_[RF_BEGIN+i] = Spatial_Matrix_[RF_BEGIN+i]*COM_Projection_Matrix_[RF_BEGIN+i-1];
        COM_Projection_Matrix_[LF_BEGIN+i] = Spatial_Matrix_[LF_BEGIN+i]*COM_Projection_Matrix_[LF_BEGIN+i-1];
    }

//    for(int i=0;i<28;i++){
//        for(int j=0;j<6;j++)
////            file[36]<<COM_Projection_Matrix_[i](j,0)<<"\t"<<COM_Projection_Matrix_[i](j,1)<<"\t"<<COM_Projection_Matrix_[i](j,2)<<"\t"<<COM_Projection_Matrix_[i](j,3)<<"\t"<<COM_Projection_Matrix_[i](j,4)<<"\t"<<COM_Projection_Matrix_[i](j,5)<<endl;
//            file[36]<<Spatial_Matrix_[i](j,0)<<"\t"<<Spatial_Matrix_[i](j,1)<<"\t"<<Spatial_Matrix_[i](j,2)<<"\t"<<Spatial_Matrix_[i](j,3)<<"\t"<<Spatial_Matrix_[i](j,4)<<"\t"<<Spatial_Matrix_[i](j,5)<<endl;

//    }

}
void WalkingController::Spatial_tranform_to_COM(){

    Eigen::Matrix6d Spatial_transform_pelv_to_COM;
    Eigen::Matrix<double, 3, 3> Skew_temp, Rotation_temp, Iden_3d;
    Skew_temp.setIdentity();
    Rotation_temp.setIdentity();
    Iden_3d.setIdentity();

    Skew_temp = DyrosMath::skew(com_float_current_);

    Spatial_transform_pelv_to_COM.setIdentity();
    Spatial_transform_pelv_to_COM.block<3,3>(0,3) = Skew_temp;

    Spatial_COM_transform_[BASE_LINK] = Spatial_link_transform_[BASE_LINK]*Spatial_transform_pelv_to_COM;

    Spatial_COM_transform_[LF_LINK] = Spatial_link_transform_[LF_LINK]*Spatial_COM_transform_[BASE_LINK];
    Spatial_COM_transform_[RF_LINK] = Spatial_link_transform_[RF_LINK]*Spatial_COM_transform_[BASE_LINK];
    Spatial_COM_transform_[WA_LINK] = Spatial_link_transform_[WA_LINK]*Spatial_COM_transform_[BASE_LINK];
    Spatial_COM_transform_[WA_LINK+1] = Spatial_link_transform_[WA_LINK+1]*Spatial_COM_transform_[WA_LINK];

    Spatial_COM_transform_[LA_LINK] = Spatial_link_transform_[LA_LINK]*Spatial_COM_transform_[WA_LINK+1];
    Spatial_COM_transform_[RA_LINK] = Spatial_link_transform_[RA_LINK]*Spatial_COM_transform_[WA_LINK+1];


    for(int i=1;i<7;i++){
        COM_Projection_Matrix_[RA_LINK+i] = Spatial_link_transform_[RA_LINK+i]*COM_Projection_Matrix_[RA_LINK+i-1];
        COM_Projection_Matrix_[LA_LINK+i] = Spatial_link_transform_[LA_LINK+i]*COM_Projection_Matrix_[LA_LINK+i-1];
    }

    for(int i=1;i<6;i++){
        COM_Projection_Matrix_[RF_LINK+i] = Spatial_link_transform_[RF_LINK+i]*COM_Projection_Matrix_[RF_LINK+i-1];
        COM_Projection_Matrix_[LF_LINK+i] = Spatial_link_transform_[LF_LINK+i]*COM_Projection_Matrix_[LF_LINK+i-1];
    }
}

void WalkingController::Relative_Inertia(){
    //////////////////////// calculating spatial inertia matrix //////////////
    ///
    ///       order : linear velocity and angular velocity
    ///    I_i     [ m_i          m_i*S(c_i)^T]
    ///            [ m_i*S(c_i)         bar_I ]
    ///
    ///    bar_I   = I^(cm)_i + m_i*S(c_i)*S(c_i)^T , bar_I  is rotational inertia(i)
    Eigen::Matrix<double, 3, 3> Skew_temp, rotational_inertia_temp;
    Skew_temp.setIdentity();
    rotational_inertia_temp.setIdentity();

//    if(walking_tick_ == 0){
//        cout<<"llims mass in relative inertia func "<<endl;
//        for(int i=0;i<29;i++)
//            cout<<link_mass_[i]<<endl;
//    }

    if(walking_tick_ == 0){
        for(int i=0;i<29;i++)
            Spatial_Inertia_[i].setZero();
    }
    for(int i=0;i<7;i++){// for arm link
        Skew_temp = DyrosMath::skew(link_local_com_position_[RA_LINK+i]);//right arm

        rotational_inertia_temp = link_inertia_[RA_LINK+i] + link_mass_[RA_LINK+i]*Skew_temp*Skew_temp.transpose();

        for(int j=0;j<3;j++){
            Spatial_Inertia_[RA_BEGIN+i](j,j) = link_mass_[RA_LINK+i];
        }
        Spatial_Inertia_[RA_BEGIN+i].block<3,3>(0,3) = link_mass_[RA_LINK+i]*Skew_temp.transpose();
        Spatial_Inertia_[RA_BEGIN+i].block<3,3>(3,0)  = link_mass_[RA_LINK+i]*Skew_temp;
        Spatial_Inertia_[RA_BEGIN+i].block<3,3>(3,3) = rotational_inertia_temp;

        Skew_temp = DyrosMath::skew(link_local_com_position_[LA_LINK+i]);//left arm
        rotational_inertia_temp = link_inertia_[LA_LINK+i] + link_mass_[LA_LINK+i]*Skew_temp*Skew_temp.transpose();

        for(int j=0;j<3;j++){
            Spatial_Inertia_[LA_BEGIN+i](j,j) = link_mass_[LA_LINK+i];
        }

        Spatial_Inertia_[LA_BEGIN+i].block<3,3>(0,3) = link_mass_[LA_LINK+i]*Skew_temp.transpose();
        Spatial_Inertia_[LA_BEGIN+i].block<3,3>(3,0)  = link_mass_[LA_LINK+i]*Skew_temp;
        Spatial_Inertia_[LA_BEGIN+i].block<3,3>(3,3) = rotational_inertia_temp;
    }

    for(int i=0;i<6;i++){// for foot link
        Skew_temp = DyrosMath::skew(link_local_com_position_[RF_LINK+i]);//right foot
        rotational_inertia_temp = link_inertia_[RF_LINK+i] + link_mass_[RF_LINK+i]*Skew_temp*Skew_temp.transpose();

        for(int j=0;j<3;j++){
            Spatial_Inertia_[RF_BEGIN+i](j,j) = link_mass_[RF_LINK+i];
        }

        Spatial_Inertia_[RF_BEGIN+i].block<3,3>(0,3) = link_mass_[RF_LINK+i]*Skew_temp.transpose();
        Spatial_Inertia_[RF_BEGIN+i].block<3,3>(3,0) = link_mass_[RF_LINK+i]*Skew_temp;
        Spatial_Inertia_[RF_BEGIN+i].block<3,3>(3,3) = rotational_inertia_temp;


        Skew_temp = DyrosMath::skew(link_local_com_position_[LF_LINK+i]);//left foot
        rotational_inertia_temp = link_inertia_[LF_LINK+i] + link_mass_[LF_LINK+i]*Skew_temp*Skew_temp.transpose();

        for(int j=0;j<3;j++){
            Spatial_Inertia_[LF_BEGIN+i](j,j) = link_mass_[LF_LINK+i];
        }

        Spatial_Inertia_[LF_BEGIN+i].block<3,3>(0,3) = link_mass_[LF_LINK+i]*Skew_temp.transpose();
        Spatial_Inertia_[LF_BEGIN+i].block<3,3>(3,0) = link_mass_[LF_LINK+i]*Skew_temp;
        Spatial_Inertia_[LF_BEGIN+i].block<3,3>(3,3) = rotational_inertia_temp;


    }

    Skew_temp = DyrosMath::skew(link_local_com_position_[WA_LINK]);//waist
    rotational_inertia_temp = link_inertia_[WA_LINK] + link_mass_[WA_LINK]*Skew_temp*Skew_temp.transpose();

    for(int j=0;j<3;j++){
        Spatial_Inertia_[WA_BEGIN](j,j) = link_mass_[WA_LINK];
    }

    Spatial_Inertia_[WA_BEGIN].block<3,3>(0,3)= link_mass_[WA_LINK]*Skew_temp.transpose();
    Spatial_Inertia_[WA_BEGIN].block<3,3>(3,0) = link_mass_[WA_LINK]*Skew_temp;
    Spatial_Inertia_[WA_BEGIN].block<3,3>(3,3) = rotational_inertia_temp;


    Skew_temp = DyrosMath::skew(link_local_com_position_[WA_LINK+1]);//waist +1
    rotational_inertia_temp = link_inertia_[WA_LINK+1] + link_mass_[WA_LINK+1]*Skew_temp*Skew_temp.transpose();

    for(int j=0;j<3;j++){
        Spatial_Inertia_[WA_BEGIN+1](j,j) = link_mass_[WA_LINK+1];
    }


    Spatial_Inertia_[WA_BEGIN+1].block<3,3>(0,3)= link_mass_[WA_LINK+1]*Skew_temp.transpose();
    Spatial_Inertia_[WA_BEGIN+1].block<3,3>(3,0) = link_mass_[WA_LINK+1]*Skew_temp;
    Spatial_Inertia_[WA_BEGIN+1].block<3,3>(3,3) = rotational_inertia_temp;

    Skew_temp = DyrosMath::skew(link_local_com_position_[0]);//mass of pelvis
    rotational_inertia_temp = link_inertia_[0] + link_mass_[0]*Skew_temp*(Skew_temp.transpose());

    for(int j=0;j<3;j++){
        Spatial_Inertia_[28](j,j+3) = link_mass_[0];
    }

    Spatial_Inertia_[28].block<3,3>(0,3) = link_mass_[0]*Skew_temp.transpose();
    Spatial_Inertia_[28].block<3,3>(3,0) = link_mass_[0]*Skew_temp;
    Spatial_Inertia_[28].block<3,3>(3,3) = rotational_inertia_temp;

}
void WalkingController::Joint_Spatial_Inertia()
{
    //////////////////////// calculating spatial inertia matrix //////////////
    ///
    ///       order : linear velocity and angular velocity
    ///    I_i     [ m_i          m_i*S(c_i)^T]
    ///            [ m_i*S(c_i)         bar_I ]
    ///
    ///    bar_I   = I^(cm)_i + m_i*S(c_i)*S(c_i)^T , bar_I  is rotational inertia(i)
    ///

    Eigen::Matrix<double, 3, 3> Skew_temp, rotational_inertia_link;
    Skew_temp.setZero(); rotational_inertia_link.setZero();

    Eigen::Matrix3d Iden_3d;
    Iden_3d.setIdentity();

    if(walking_tick_ == 0){
        for(int i=0;i<29;i++)
            Spatial_Inertia_[i].setZero();
    }

//    if(walking_tick_ == 0){
//        for(int i=0;i<29;i++){
//            cout<<i <<" link local com :"<<"\t"<<link_local_com_position_[i](0)<<"\t"<<link_local_com_position_[i](1)<<"\t"<<link_local_com_position_[i](2)<<endl;
//        }
//    }
    for(int i=0;i<28;i++){
        Skew_temp = DyrosMath::skew(link_local_com_position_[i+1]); // input link is one order high than ouput joint-begin

        rotational_inertia_link = link_inertia_[1+i] + link_mass_[i+1]*Skew_temp*(Skew_temp.transpose());

        Spatial_Inertia_[i].block<3,3>(0,0) = link_mass_[1+i]*Iden_3d;
        Spatial_Inertia_[i].block<3,3>(0,3) = link_mass_[1+i]*(Skew_temp.transpose());
        Spatial_Inertia_[i].block<3,3>(3,0) = link_mass_[1+i]*Skew_temp;
        Spatial_Inertia_[i].block<3,3>(3,3) = rotational_inertia_link;
    }

    Skew_temp = DyrosMath::skew(link_local_com_position_[0]);
//    Skew_temp  = DyrosMath::skew(com_float_current_);
    rotational_inertia_link = link_inertia_[0] + link_mass_[0]*Skew_temp*(Skew_temp.transpose());

    Spatial_Inertia_[28].block<3,3>(0,0) = link_mass_[0]*Iden_3d;
    Spatial_Inertia_[28].block<3,3>(0,3) = link_mass_[0]*(Skew_temp.transpose());
    Spatial_Inertia_[28].block<3,3>(3,0) = link_mass_[0]*Skew_temp;
    Spatial_Inertia_[28].block<3,3>(3,3) = rotational_inertia_link;

}
void WalkingController::Link_Spatial_Inertia()
{
    //////////////////////// calculating spatial inertia matrix //////////////
    ///
    ///       order : linear velocity and angular velocity
    ///    I_i     [ m_i          m_i*S(c_i)^T]
    ///            [ m_i*S(c_i)         bar_I ]
    ///
    ///    bar_I   = I^(cm)_i + m_i*S(c_i)*S(c_i)^T , bar_I  is rotational inertia(i)
    ///

    Eigen::Matrix<double, 3, 3> Skew_temp, rotational_inertia_link;
    Skew_temp.setZero(); rotational_inertia_link.setZero();

    Eigen::Matrix3d Iden_3d;
    Iden_3d.setIdentity();

    if(walking_tick_ == 0){
        for(int i=0;i<29;i++)
            Spatial_link_Inertia_[i].setZero();
    }

    if(walking_tick_ == 0){
        for(int i=0;i<29;i++){
            cout<<i <<" link local com :"<<"\t"<<link_local_com_position_[i](0)<<"\t"<<link_local_com_position_[i](1)<<"\t"<<link_local_com_position_[i](2)<<endl;
        }
    }
    for(int i=0;i<29;i++){
        Skew_temp = DyrosMath::skew(link_local_com_position_[i]); // input link is one order high than ouput joint-begin

        rotational_inertia_link = link_inertia_[i] + link_mass_[i]*Skew_temp*(Skew_temp.transpose());

        Spatial_link_Inertia_[i].block<3,3>(0,0) = link_mass_[i]*Iden_3d;
        Spatial_link_Inertia_[i].block<3,3>(0,3) = link_mass_[i]*(Skew_temp.transpose());
        Spatial_link_Inertia_[i].block<3,3>(3,0) = link_mass_[i]*Skew_temp;
        Spatial_link_Inertia_[i].block<3,3>(3,3) = rotational_inertia_link;
    }

}
void WalkingController::System_Spatial_Inertia(){

    //////////////////////// calculating spatial inertia matrix //////////////
    ///
    ///       order : linear velocity and angular velocity
    ///    I^c _p(i) = I^c _p(i) + X^i_p(i).transpose()*I^c_i*X^i_p(i)
    ///
    ///
    ///
    //cout<<"Spatial inertia function :"<<endl;

    for(int i=0;i<29;i++)
        System_Inertia_[i] = Spatial_Inertia_[i];


    for(int i=6; i==1; i--){
        System_Inertia_[RA_BEGIN+i-1] = System_Inertia_[RA_BEGIN+i-1] + Spatial_Matrix_[RA_BEGIN+i].transpose()*System_Inertia_[RA_BEGIN+i]*Spatial_Matrix_[RA_BEGIN+i];
        System_Inertia_[LA_BEGIN+i-1] = System_Inertia_[LA_BEGIN+i-1] + Spatial_Matrix_[LA_BEGIN+i].transpose()*System_Inertia_[LA_BEGIN+i]*Spatial_Matrix_[LA_BEGIN+i];
    }
    for(int i=5; i==1; i--){
        System_Inertia_[RF_BEGIN+i-1] = System_Inertia_[RF_BEGIN+i-1] + Spatial_Matrix_[RF_BEGIN+i].transpose()*System_Inertia_[RF_BEGIN+i]*Spatial_Matrix_[RF_BEGIN+i];
        System_Inertia_[LF_BEGIN+i-1] = System_Inertia_[LF_BEGIN+i-1] + Spatial_Matrix_[LF_BEGIN+i].transpose()*System_Inertia_[LF_BEGIN+i]*Spatial_Matrix_[LF_BEGIN+i];
    }
    System_Inertia_[WA_BEGIN+1] = System_Inertia_[WA_BEGIN+1] + Spatial_Matrix_[RA_BEGIN].transpose()*System_Inertia_[RA_BEGIN]*Spatial_Matrix_[RA_BEGIN] + Spatial_Matrix_[LA_BEGIN].transpose()*System_Inertia_[LA_BEGIN]*Spatial_Matrix_[LA_BEGIN];
    System_Inertia_[WA_BEGIN] = System_Inertia_[WA_BEGIN] +Spatial_Matrix_[WA_BEGIN+1].transpose()*System_Inertia_[WA_BEGIN+1]*Spatial_Matrix_[WA_BEGIN+1];

//    cout<<"spartial inertia RA_BEGIN : "<<Spatial_Inertia_[RA_BEGIN]<<endl;
//    cout<<"spartial inertia LA_BEGIN : "<<Spatial_Inertia_[LA_BEGIN]<<endl;
//    cout<<"spartial inertia WA_BEGIn : "<<Spatial_Inertia_[WA_BEGIN]<<endl;

    //Spatial_Inertia_[28] += Spatial_Matrix_[RA_BEGIN].transpose()*Spatial_Inertia_[RA_BEGIN]*Spatial_Matrix_[RA_BEGIN];
    //Spatial_Inertia_[28] += Spatial_Matrix_[LA_BEGIN].transpose()*Spatial_Inertia_[LA_BEGIN]*Spatial_Matrix_[LA_BEGIN];
//    System_Inertia_[28].setZero();

    System_Inertia_[28] += Spatial_Matrix_[RF_BEGIN].transpose()*System_Inertia_[RF_BEGIN]*Spatial_Matrix_[RF_BEGIN];
    System_Inertia_[28] += Spatial_Matrix_[LF_BEGIN].transpose()*System_Inertia_[LF_BEGIN]*Spatial_Matrix_[LF_BEGIN];
    System_Inertia_[28] += Spatial_Matrix_[WA_BEGIN].transpose()*System_Inertia_[WA_BEGIN]*Spatial_Matrix_[WA_BEGIN];


}
void WalkingController::Composite_Rigid_Body_Inertia(){

    //////////////////////// calculating spatial inertia matrix //////////////
    ///
    ///       order : linear velocity and angular velocity
    ///    I^c _p(i) = I^c _p(i) + X^i_p(i).transpose()*I^c_i*X^i_p(i)
    ///
    ///
    ///
    //cout<<"Spatial inertia function :"<<endl;

    for(int i=0;i<29;i++)
        CCRB_Inertia_[i] = Spatial_link_Inertia_[i];


    for(int i=6; i>0; i--){
        CCRB_Inertia_[RA_LINK+i-1] = CCRB_Inertia_[RA_LINK+i-1] + Spatial_link_transform_[RA_LINK+i].transpose()*CCRB_Inertia_[RA_LINK+i]*Spatial_link_transform_[RA_LINK+i];
        CCRB_Inertia_[LA_LINK+i-1] = CCRB_Inertia_[LA_LINK+i-1] + Spatial_link_transform_[LA_LINK+i].transpose()*CCRB_Inertia_[LA_LINK+i]*Spatial_link_transform_[LA_LINK+i];
    }
    for(int i=5; i>0; i--){
        CCRB_Inertia_[RF_LINK+i-1] = CCRB_Inertia_[RF_LINK+i-1] + Spatial_link_transform_[RF_LINK+i].transpose()*CCRB_Inertia_[RF_LINK+i]*Spatial_link_transform_[RF_LINK+i];
        CCRB_Inertia_[LF_LINK+i-1] = CCRB_Inertia_[LF_LINK+i-1] + Spatial_link_transform_[LF_LINK+i].transpose()*CCRB_Inertia_[LF_LINK+i]*Spatial_link_transform_[LF_LINK+i];
    }
    CCRB_Inertia_[WA_LINK+1] = CCRB_Inertia_[WA_LINK+1] + Spatial_link_transform_[RA_LINK].transpose()*CCRB_Inertia_[RA_LINK]*Spatial_link_transform_[RA_LINK] + Spatial_link_transform_[LA_LINK].transpose()*CCRB_Inertia_[LA_LINK]*Spatial_link_transform_[LA_LINK];
    CCRB_Inertia_[WA_LINK] = CCRB_Inertia_[WA_LINK] +Spatial_link_transform_[WA_LINK+1].transpose()*CCRB_Inertia_[WA_LINK+1]*Spatial_link_transform_[WA_LINK+1];

//    cout<<"spartial inertia RA_BEGIN : "<<Spatial_Inertia_[RA_BEGIN]<<endl;
//    cout<<"spartial inertia LA_BEGIN : "<<Spatial_Inertia_[LA_BEGIN]<<endl;
//    cout<<"spartial inertia WA_BEGIn : "<<Spatial_Inertia_[WA_BEGIN]<<endl;

    //Spatial_Inertia_[28] += Spatial_Matrix_[RA_BEGIN].transpose()*Spatial_Inertia_[RA_BEGIN]*Spatial_Matrix_[RA_BEGIN];
    //Spatial_Inertia_[28] += Spatial_Matrix_[LA_BEGIN].transpose()*Spatial_Inertia_[LA_BEGIN]*Spatial_Matrix_[LA_BEGIN];
//    System_Inertia_[28].setZero();

    CCRB_Inertia_[BASE_LINK] += Spatial_link_transform_[RF_LINK].transpose()*CCRB_Inertia_[RF_LINK]*Spatial_link_transform_[RF_LINK];
    CCRB_Inertia_[BASE_LINK] += Spatial_link_transform_[LF_LINK].transpose()*CCRB_Inertia_[LF_LINK]*Spatial_link_transform_[LF_LINK];
    CCRB_Inertia_[BASE_LINK] += Spatial_link_transform_[WA_LINK].transpose()*CCRB_Inertia_[WA_LINK]*Spatial_link_transform_[WA_LINK];

}
void WalkingController::Centroidal_Momentum_Matrix(){

    ///////////////////////////////////////////////////////////////
    /// calculating the centroidal momentum matrx A_G
    ///  A_G(i) = i_X_p(i).transpose*I_i_toCoM*joint_axis
    ///  Joint_axis is expressed in local coordinates, Joint_axis
    ///
    ///
    Eigen::Vector6d joint_axis;
    joint_axis.setZero();
    Eigen::Vector6d joint_axis_local[28];
    for(int i=0;i<28;i++)
        joint_axis_local[i].setZero();

    //left leg - right leg
    joint_axis_local[0](5) = -1; joint_axis_local[1](3) = 1;  joint_axis_local[2](4) = 1;  joint_axis_local[3](4) =  1; joint_axis_local[4](4) =  1;  joint_axis_local[5](3) =  1;
    joint_axis_local[6](5) = -1; joint_axis_local[7](3) = 1; joint_axis_local[8](4) = -1; joint_axis_local[9](4) = -1; joint_axis_local[10](4) = -1; joint_axis_local[11](3) = 1;

    //waist
    joint_axis_local[12](5) = 1; joint_axis_local[13](3) = -1;

    //left arm - right arm
    joint_axis_local[14](4) = 1;    joint_axis_local[15](3) = 1;    joint_axis_local[16](4) = 1;    joint_axis_local[17](3) = 1;    joint_axis_local[18](4) = 1;    joint_axis_local[19](3) = 1;    joint_axis_local[20](4) = 1;
    joint_axis_local[21](4) = -1;    joint_axis_local[22](3) = 1;    joint_axis_local[23](4) = -1;    joint_axis_local[24](3) = 1;    joint_axis_local[25](4) = -1;    joint_axis_local[26](3) = 1;    joint_axis_local[27](4) = -1;

//   // left leg - right leg
//    joint_axis_local[0](5) = 1; joint_axis_local[1](3) = 1;  joint_axis_local[2](4) = 1;  joint_axis_local[3](4) =  1; joint_axis_local[4](4) =  1;  joint_axis_local[5](3) =  1;
//    joint_axis_local[6](5) = 1; joint_axis_local[7](3) = 1; joint_axis_local[8](4) = 1; joint_axis_local[9](4) = 1; joint_axis_local[10](4) = 1; joint_axis_local[11](3) = 1;

//    //waist
//    joint_axis_local[12](5) = 1; joint_axis_local[13](3) = 1;

//    //left arm - right arm
//    joint_axis_local[14](4) = 1;    joint_axis_local[15](3) = 1;    joint_axis_local[16](4) = 1;    joint_axis_local[17](3) = 1;    joint_axis_local[18](4) = 1;    joint_axis_local[19](3) = 1;    joint_axis_local[20](4) = 1;
//    joint_axis_local[21](4) = 1;    joint_axis_local[22](3) = 1;    joint_axis_local[23](4) = 1;    joint_axis_local[24](3) = 1;    joint_axis_local[25](4) = 1;    joint_axis_local[26](3) = 1;    joint_axis_local[27](4) = 1;


    q_dot_2 = (p_q_ - current_q_)*hz_;

    //q_dot_ = current_q_dot_;
    q_dot_.setZero();
    q_dot_.segment<12>(0) = desired_leg_q_dot_;

    for(int i=0;i<7;i++){
        for(int j=3;j<6;j++){
            joint_axis(j)=current_arm_jacobian_r_(j,i);           
        }
        joint_axis.setZero();
        joint_axis = joint_axis_local[RA_BEGIN +i];
        //cout<<joint_axis<<endl;

        Centroidal_Momentum_Matrix_[RA_BEGIN+i] = COM_Projection_Matrix_[RA_BEGIN+i].transpose()*System_Inertia_[RA_BEGIN+i]*joint_axis;

        for(int j=3;j<6;j++){
            joint_axis(j)=current_arm_jacobian_l_(j,i);
        }
        joint_axis.setZero();
        joint_axis = joint_axis_local[LA_BEGIN +i];
        Centroidal_Momentum_Matrix_[LA_BEGIN+i] = COM_Projection_Matrix_[LA_BEGIN+i].transpose()*System_Inertia_[LA_BEGIN+i]*joint_axis;
    }

    for(int i=0;i<6;i++){
        for(int j=3;j<6;j++){
            joint_axis(j)=current_leg_jacobian_r_(j,i);
        }

        joint_axis.setZero();
        joint_axis = joint_axis_local[RF_BEGIN +i];
        Centroidal_Momentum_Matrix_[RF_BEGIN+i] = COM_Projection_Matrix_[RF_BEGIN+i].transpose()*System_Inertia_[RF_BEGIN+i]*joint_axis;

        for(int j=3;j<6;j++){
            joint_axis(j)=current_leg_jacobian_l_(j,i);
        }
        joint_axis.setZero();
        joint_axis = joint_axis_local[LF_BEGIN +i];
        Centroidal_Momentum_Matrix_[LF_BEGIN+i] = COM_Projection_Matrix_[LF_BEGIN+i].transpose()*System_Inertia_[LF_BEGIN+i]*joint_axis;
    }
    for(int i=0;i<2;i++){
        for(int j=3;j<6;j++){
            joint_axis(j) = current_waist_jacobian_[i](j,1);
        }


        joint_axis.setZero();
        joint_axis = joint_axis_local[WA_BEGIN +i];
       // cout<<"waist axis : "<<i<<" " <<joint_axis<<endl;
        Centroidal_Momentum_Matrix_[WA_BEGIN+i] = COM_Projection_Matrix_[WA_BEGIN+i].transpose()*System_Inertia_[WA_BEGIN+i]*joint_axis;
    }

//    for(int i=0;i<28;i++){
//        Centroidal_Momentum_Matrix_[i] = COM_Projection_Matrix_[i].transpose()*Spatial_Inertia_[i]*joint_axis_local[i];
//    }

//    file[36]<<walking_tick_<<"\t"<<COM_Projection_Matrix_[RF_BEGIN](3)<<"\t"<<COM_Projection_Matrix_[RF_BEGIN](4)<<"\t"<<COM_Projection_Matrix_[RF_BEGIN](5)//1,2~4
//                           <<"\t"<<COM_Projection_Matrix_[RF_BEGIN+1](3)<<"\t"<<COM_Projection_Matrix_[RF_BEGIN+1](4)<<"\t"<<COM_Projection_Matrix_[RF_BEGIN+1](5) //5~7
//                           <<"\t"<<COM_Projection_Matrix_[RF_BEGIN+2](3)<<"\t"<<COM_Projection_Matrix_[RF_BEGIN+2](4)<<"\t"<<COM_Projection_Matrix_[RF_BEGIN+2](5) //8~10
//                           <<"\t"<<COM_Projection_Matrix_[RF_BEGIN+3](3)<<"\t"<<COM_Projection_Matrix_[RF_BEGIN+3](4)<<"\t"<<COM_Projection_Matrix_[RF_BEGIN+3](5) //11~13
//                           <<"\t"<<COM_Projection_Matrix_[RF_BEGIN+4](3)<<"\t"<<COM_Projection_Matrix_[RF_BEGIN+4](4)<<"\t"<<COM_Projection_Matrix_[RF_BEGIN+4](5) //14~16
//                           <<"\t"<<COM_Projection_Matrix_[RF_BEGIN+5](3)<<"\t"<<COM_Projection_Matrix_[RF_BEGIN+5](4)<<"\t"<<COM_Projection_Matrix_[RF_BEGIN+5](5)  //17~19
//                           <<"\t"<<COM_Projection_Matrix_[LF_BEGIN](3)<<"\t"<<COM_Projection_Matrix_[LF_BEGIN](4)<<"\t"<<COM_Projection_Matrix_[LF_BEGIN](5) //20~22
//                           <<"\t"<<COM_Projection_Matrix_[LF_BEGIN+1](3)<<"\t"<<COM_Projection_Matrix_[LF_BEGIN+1](4)<<"\t"<<COM_Projection_Matrix_[LF_BEGIN+1](5) //23~25
//                           <<"\t"<<COM_Projection_Matrix_[LF_BEGIN+2](3)<<"\t"<<COM_Projection_Matrix_[LF_BEGIN+2](4)<<"\t"<<COM_Projection_Matrix_[LF_BEGIN+2](5)//26~28
//                           <<"\t"<<COM_Projection_Matrix_[LF_BEGIN+3](3)<<"\t"<<COM_Projection_Matrix_[LF_BEGIN+3](4)<<"\t"<<COM_Projection_Matrix_[LF_BEGIN+3](5)//29~31
//                           <<"\t"<<COM_Projection_Matrix_[LF_BEGIN+4](3)<<"\t"<<COM_Projection_Matrix_[LF_BEGIN+4](4)<<"\t"<<COM_Projection_Matrix_[LF_BEGIN+4](5)//32~35
//                           <<"\t"<<COM_Projection_Matrix_[LF_BEGIN+5](3)<<"\t"<<COM_Projection_Matrix_[LF_BEGIN+5](4)<<"\t"<<COM_Projection_Matrix_[LF_BEGIN+5](5)<<endl;//36~38

//    file[36]<<walking_tick_;
//    for(int i=0;i<6;i++){
//        for(int j=0;j<3;j++){
//            for(int k=0;k<3;k++){
//                file[36]<<"\t"<<COM_Projection_Matrix_[RF_BEGIN+i](j+3,k+3);
//            }
//        }
//    }
//    for(int i=0;i<6;i++){
//        for(int j=0;j<3;j++){
//            for(int k=0;k<3;k++){
//                file[36]<<"\t"<<COM_Projection_Matrix_[LF_BEGIN+i](j+3,k+3);
//            }
//        }
//    }
//    file[36]<<endl;

    if(walking_tick_ == 500){
        for(int i=0;i<29;i++){
            file[36]<<i<<"\t"<<Centroidal_Momentum_Matrix_[i](0)<<"\t"<<Centroidal_Momentum_Matrix_[i](1)<<"\t"<<Centroidal_Momentum_Matrix_[i](2)<<"\t"<<Centroidal_Momentum_Matrix_[i](3)<<"\t"<<Centroidal_Momentum_Matrix_[i](4)<<"\t"<<Centroidal_Momentum_Matrix_[i](5)<<endl;
        }
    }

    Centroidal_Momentum_[28].setZero();
    for(int i=0;i<28;i++){
////        cout<<"spatial inertia : "<<i <<"  "<<Spatial_Inertia_[i]<<endl;
////        cout<<"Spatial com transform : "<<i << "  " <<COM_Projection_Matrix_[i]<<endl;
////        cout<<"centorida momentum matrix "<<i<< "  " <<Centroidal_Momentum_Matrix_[i]<<endl;
        Centroidal_Momentum_[i] = Centroidal_Momentum_Matrix_[i] *current_q_dot_(i);
//        //Centroidal_Momentum_Matrix_[28] += Centroidal_Momentum_Matrix_[i]*q_dot_(i); //h_G (Centroidal momentum)
        Centroidal_Momentum_[28] += Centroidal_Momentum_[i];
    }

    Eigen::Matrix6d Inertia_G;

    Inertia_G.setZero();
    Inertia_G = COM_Projection_Matrix_[28].transpose()*System_Inertia_[28]*COM_Projection_Matrix_[28];

    Eigen::Vector6d G_vel;
    G_vel.setZero();

    G_vel = Inertia_G.inverse()*Centroidal_Momentum_[28];

    Centroidal_Momentum_leg_[0].setZero();
    Centroidal_Momentum_leg_[1].setZero();

    Centroidal_Momentum_Upper_.setZero();
    Centroidal_Momentum_total_.setZero();

    for(int i=0;i<6;i++){
        Centroidal_Momentum_leg_[0] += Centroidal_Momentum_Matrix_[LF_BEGIN+i]*current_q_dot_(i);
        Centroidal_Momentum_leg_[1] += Centroidal_Momentum_Matrix_[RF_BEGIN+i]*current_q_dot_(i+6);
    }

    for(int i=12;i<28;i++){
        Centroidal_Momentum_Upper_ += Centroidal_Momentum_Matrix_[i]*current_q_dot_(i);
    }
    Centroidal_Momentum_total_ = Centroidal_Momentum_leg_[0] + Centroidal_Momentum_leg_[1] + Centroidal_Momentum_Upper_;



    Eigen::Matrix6d CCM_LLeg, CCM_RLeg;
    CCM_LLeg.setZero(); CCM_RLeg.setZero();

    for(int i=0;i<6;i++){
        CCM_LLeg.col(i) = Centroidal_Momentum_Matrix_[i];
        CCM_RLeg.col(i) = Centroidal_Momentum_Matrix_[6+i];
    }
    Eigen::Matrix6d kp; // for setting CLIK gains
    kp.setZero();
    kp(0,0) = 200;
    kp(1,1) = 200;
    kp(2,2) = 200;
    kp(3,3) = 250;
    kp(4,4) = 250;
    kp(5,5) = 250;

    Eigen::Vector6d CM_lleg, CM_rleg;
    CM_lleg.setZero(); CM_rleg.setZero();

//    if(walking_tick_<t_start_real_+t_double1_ || walking_tick_>= t_start_+t_total_-t_double2_)
//    {
//        lp_.setZero();
//        rp_.setZero();
//    }

    CM_lleg = CCM_LLeg*current_leg_jacobian_l_inv_*kp*lp_;
    CM_rleg = CCM_RLeg*current_leg_jacobian_r_inv_*kp*rp_;


//    cout<<"Centroial momentum check : "<<endl<<"cm leg   "<<CM_lleg<<endl<<"CCM_lleg "<<CCM_LLeg<<endl;

//    Centroidal_Momentum_[28] = CM_lleg + CM_rleg;

    file[15]<<walking_tick_;
    for(int i=0;i<6;i++){
        file[15]<<"\t"<<Centroidal_Momentum_[28](i);
    }
    for(int i=0;i<6;i++){
        file[15]<<"\t"<<Centroidal_Momentum_total_(i);
    }

    Eigen::Vector6d CAM_at_pel;
    CAM_at_pel =  COM_Projection_Matrix_[28].transpose()*Centroidal_Momentum_[28];

    file[15]<<"\t"<<current_Angular_momentum_(0)<<"\t"<<current_Angular_momentum_(1)<<"\t"<<current_Angular_momentum_(2)<<"\t"<<CM_lleg(5)<<"\t"<<CM_rleg(5)
           <<"\t"<<G_vel(0)<<"\t"<<G_vel(1)<<"\t"<<G_vel(2)<<"\t"<<G_vel(3)<<"\t"<<G_vel(4)<<"\t"<<G_vel(5)
          <<"\t"<<CAM_at_pel(0)<<"\t"<<CAM_at_pel(1)<<"\t"<<CAM_at_pel(2)<<"\t"<<CAM_at_pel(3)<<"\t"<<CAM_at_pel(4)<<"\t"<<CAM_at_pel(5)<<endl;


}
void WalkingController::getCentroidal_Momentum_Matrix(){

    ///////////////////////////////////////////////////////////////
    /// calculating the centroidal momentum matrx A_G
    ///  A_G(i) = i_X_p(i).transpose*I_i_toCoM*joint_axis
    ///  Joint_axis is expressed in local coordinates, Joint_axis
    ///
    ///
    Eigen::Vector6d joint_axis;
    joint_axis.setZero();
    Eigen::Vector6d joint_axis_local[28];
    for(int i=0;i<28;i++)
        joint_axis_local[i].setZero();

    //left leg - right leg
    joint_axis_local[0](5) = -1; joint_axis_local[1](3) = 1;  joint_axis_local[2](4) = 1;  joint_axis_local[3](4) =  1; joint_axis_local[4](4) =  1;  joint_axis_local[5](3) =  1;
    joint_axis_local[6](5) = -1; joint_axis_local[7](3) = 1; joint_axis_local[8](4) = -1; joint_axis_local[9](4) = -1; joint_axis_local[10](4) = -1; joint_axis_local[11](3) = 1;

    //waist
    joint_axis_local[12](5) = 1; joint_axis_local[13](3) = -1;

    //left arm - right arm
    joint_axis_local[14](4) = 1;    joint_axis_local[15](3) = 1;    joint_axis_local[16](4) = 1;    joint_axis_local[17](3) = 1;    joint_axis_local[18](4) = 1;    joint_axis_local[19](3) = 1;    joint_axis_local[20](4) = 1;
    joint_axis_local[21](4) = -1;    joint_axis_local[22](3) = 1;    joint_axis_local[23](4) = -1;    joint_axis_local[24](3) = 1;    joint_axis_local[25](4) = -1;    joint_axis_local[26](3) = 1;    joint_axis_local[27](4) = -1;

//    //left leg - right leg
//    joint_axis_local[0](5) = 1; joint_axis_local[1](3) = 1;  joint_axis_local[2](4) = 1;  joint_axis_local[3](4) =  1; joint_axis_local[4](4) =  1;  joint_axis_local[5](3) =  1;
//    joint_axis_local[6](5) = 1; joint_axis_local[7](3) = 1; joint_axis_local[8](4) = 1; joint_axis_local[9](4) = 1; joint_axis_local[10](4) = 1; joint_axis_local[11](3) = 1;

//    //waist
//    joint_axis_local[12](5) = 1; joint_axis_local[13](3) = 1;

//    //left arm - right arm
//    joint_axis_local[14](4) = 1;    joint_axis_local[15](3) = 1;    joint_axis_local[16](4) = 1;    joint_axis_local[17](3) = 1;    joint_axis_local[18](4) = 1;    joint_axis_local[19](3) = 1;    joint_axis_local[20](4) = 1;
//    joint_axis_local[21](4) = 1;    joint_axis_local[22](3) = 1;    joint_axis_local[23](4) = 1;    joint_axis_local[24](3) = 1;    joint_axis_local[25](4) = 1;    joint_axis_local[26](3) = 1;    joint_axis_local[27](4) = 1;


    // CMM for pelvis is = because no joints

    // for leg
    Eigen::Vector6d virtual_joint;
    virtual_joint.setOnes();
    Centroidal_Momentum_Matrix_[BASE_LINK] = Spatial_COM_transform_[BASE_LINK].transpose()*CCRB_Inertia_[BASE_LINK]*virtual_joint;
    for(int i=0;i<6;i++){
        Centroidal_Momentum_Matrix_[LF_LINK+i] = Spatial_COM_transform_[LF_LINK+i].transpose()*CCRB_Inertia_[LF_LINK+i]*joint_axis_local[LF_BEGIN+i];
        Centroidal_Momentum_Matrix_[RF_LINK+i] = Spatial_COM_transform_[RF_LINK+i].transpose()*CCRB_Inertia_[RF_LINK+i]*joint_axis_local[RF_BEGIN+i];
    }
    // for arm
    for(int i=0;i<7;i++){
        Centroidal_Momentum_Matrix_[LA_LINK+i] = Spatial_COM_transform_[LA_LINK+i].transpose()*CCRB_Inertia_[LA_LINK+i]*joint_axis_local[LA_BEGIN+i];
        Centroidal_Momentum_Matrix_[RA_LINK+i] = Spatial_COM_transform_[RA_LINK+i].transpose()*CCRB_Inertia_[RA_LINK+i]*joint_axis_local[RA_BEGIN+i];
    }
    for(int i=0;i<2;i++){
        Centroidal_Momentum_Matrix_[WA_LINK+i] = Spatial_COM_transform_[WA_LINK+i].transpose()*CCRB_Inertia_[WA_LINK+i]*joint_axis_local[WA_BEGIN+i];
    }

//    for(int i=0;i<28;i++){
//        Centroidal_Momentum_Matrix_[i] = COM_Projection_Matrix_[i].transpose()*Spatial_Inertia_[i]*joint_axis_local[i];
//    }

//    file[36]<<walking_tick_<<"\t"<<COM_Projection_Matrix_[RF_BEGIN](3)<<"\t"<<COM_Projection_Matrix_[RF_BEGIN](4)<<"\t"<<COM_Projection_Matrix_[RF_BEGIN](5)//1,2~4
//                           <<"\t"<<COM_Projection_Matrix_[RF_BEGIN+1](3)<<"\t"<<COM_Projection_Matrix_[RF_BEGIN+1](4)<<"\t"<<COM_Projection_Matrix_[RF_BEGIN+1](5) //5~7
//                           <<"\t"<<COM_Projection_Matrix_[RF_BEGIN+2](3)<<"\t"<<COM_Projection_Matrix_[RF_BEGIN+2](4)<<"\t"<<COM_Projection_Matrix_[RF_BEGIN+2](5) //8~10
//                           <<"\t"<<COM_Projection_Matrix_[RF_BEGIN+3](3)<<"\t"<<COM_Projection_Matrix_[RF_BEGIN+3](4)<<"\t"<<COM_Projection_Matrix_[RF_BEGIN+3](5) //11~13
//                           <<"\t"<<COM_Projection_Matrix_[RF_BEGIN+4](3)<<"\t"<<COM_Projection_Matrix_[RF_BEGIN+4](4)<<"\t"<<COM_Projection_Matrix_[RF_BEGIN+4](5) //14~16
//                           <<"\t"<<COM_Projection_Matrix_[RF_BEGIN+5](3)<<"\t"<<COM_Projection_Matrix_[RF_BEGIN+5](4)<<"\t"<<COM_Projection_Matrix_[RF_BEGIN+5](5)  //17~19
//                           <<"\t"<<COM_Projection_Matrix_[LF_BEGIN](3)<<"\t"<<COM_Projection_Matrix_[LF_BEGIN](4)<<"\t"<<COM_Projection_Matrix_[LF_BEGIN](5) //20~22
//                           <<"\t"<<COM_Projection_Matrix_[LF_BEGIN+1](3)<<"\t"<<COM_Projection_Matrix_[LF_BEGIN+1](4)<<"\t"<<COM_Projection_Matrix_[LF_BEGIN+1](5) //23~25
//                           <<"\t"<<COM_Projection_Matrix_[LF_BEGIN+2](3)<<"\t"<<COM_Projection_Matrix_[LF_BEGIN+2](4)<<"\t"<<COM_Projection_Matrix_[LF_BEGIN+2](5)//26~28
//                           <<"\t"<<COM_Projection_Matrix_[LF_BEGIN+3](3)<<"\t"<<COM_Projection_Matrix_[LF_BEGIN+3](4)<<"\t"<<COM_Projection_Matrix_[LF_BEGIN+3](5)//29~31
//                           <<"\t"<<COM_Projection_Matrix_[LF_BEGIN+4](3)<<"\t"<<COM_Projection_Matrix_[LF_BEGIN+4](4)<<"\t"<<COM_Projection_Matrix_[LF_BEGIN+4](5)//32~35
//                           <<"\t"<<COM_Projection_Matrix_[LF_BEGIN+5](3)<<"\t"<<COM_Projection_Matrix_[LF_BEGIN+5](4)<<"\t"<<COM_Projection_Matrix_[LF_BEGIN+5](5)//36~38

//    file[36]<<walking_tick_;
//    for(int i=0;i<6;i++){
//        for(int j=0;j<3;j++){
//            for(int k=0;k<3;k++){
//                file[36]<<"\t"<<COM_Projection_Matrix_[RF_BEGIN+i](j+3,k+3);
//            }
//        }
//    }
//    for(int i=0;i<6;i++){
//        for(int j=0;j<3;j++){
//            for(int k=0;k<3;k++){
//                file[36]<<"\t"<<COM_Projection_Matrix_[LF_BEGIN+i](j+3,k+3);
//            }
//        }
//    }
//    file[36]<<endl;

//    if(walking_tick_ == 500){
//        for(int i=0;i<29;i++){
//            file[36]<<i<<"\t"<<Centroidal_Momentum_Matrix_[i](0)<<"\t"<<Centroidal_Momentum_Matrix_[i](1)<<"\t"<<Centroidal_Momentum_Matrix_[i](2)<<"\t"<<Centroidal_Momentum_Matrix_[i](3)<<"\t"<<Centroidal_Momentum_Matrix_[i](4)<<"\t"<<Centroidal_Momentum_Matrix_[i](5)<<endl;
//        }
//    }

//    Centroidal_Momentum_[28].setZero();
//    for(int i=0;i<28;i++){
//////        cout<<"spatial inertia : "<<i <<"  "<<Spatial_Inertia_[i]<<endl;
//////        cout<<"Spatial com transform : "<<i << "  " <<COM_Projection_Matrix_[i]<<endl;
//////        cout<<"centorida momentum matrix "<<i<< "  " <<Centroidal_Momentum_Matrix_[i]<<endl;
//        Centroidal_Momentum_[i] = Centroidal_Momentum_Matrix_[i] *current_q_dot_(i);
////        //Centroidal_Momentum_Matrix_[28] += Centroidal_Momentum_Matrix_[i]*q_dot_(i); //h_G (Centroidal momentum)
//        Centroidal_Momentum_[28] += Centroidal_Momentum_[i];
//    }

    Eigen::Vector6d Centroidal_Momentum;
    Centroidal_Momentum.setZero();

    for(int i=0;i<28;i++){
        Centroidal_Momentum += Centroidal_Momentum_Matrix_[BASE_LINK+i+1]*current_q_dot_(i);
    }


    Eigen::Matrix6d Inertia_G;

    Inertia_G.setZero();
    Inertia_G = COM_Projection_Matrix_[28].transpose()*System_Inertia_[28]*COM_Projection_Matrix_[28];

    Eigen::Vector6d G_vel;
    G_vel.setZero();

    G_vel = Inertia_G.inverse()*Centroidal_Momentum_[28];

    Centroidal_Momentum_leg_[0].setZero();
    Centroidal_Momentum_leg_[1].setZero();

    Centroidal_Momentum_Upper_.setZero();
    Centroidal_Momentum_total_.setZero();

    for(int i=0;i<6;i++){
        Centroidal_Momentum_leg_[0] += Centroidal_Momentum_Matrix_[LF_BEGIN+i]*current_q_dot_(i);
        Centroidal_Momentum_leg_[1] += Centroidal_Momentum_Matrix_[RF_BEGIN+i]*current_q_dot_(i+6);
    }

    for(int i=12;i<28;i++){
        Centroidal_Momentum_Upper_ += Centroidal_Momentum_Matrix_[i]*current_q_dot_(i);
    }
    Centroidal_Momentum_total_ = Centroidal_Momentum_leg_[0] + Centroidal_Momentum_leg_[1] + Centroidal_Momentum_Upper_;



    Eigen::Matrix6d CCM_LLeg, CCM_RLeg;
    CCM_LLeg.setZero(); CCM_RLeg.setZero();

    for(int i=0;i<6;i++){
        CCM_LLeg.col(i) = Centroidal_Momentum_Matrix_[i];
        CCM_RLeg.col(i) = Centroidal_Momentum_Matrix_[6+i];
    }
    Eigen::Matrix6d kp; // for setting CLIK gains
    kp.setZero();
    kp(0,0) = 200;
    kp(1,1) = 200;
    kp(2,2) = 200;
    kp(3,3) = 250;
    kp(4,4) = 250;
    kp(5,5) = 250;

    Eigen::Vector6d CM_lleg, CM_rleg;
    CM_lleg.setZero(); CM_rleg.setZero();

//    if(walking_tick_<t_start_real_+t_double1_ || walking_tick_>= t_start_+t_total_-t_double2_)
//    {
//        lp_.setZero();
//        rp_.setZero();
//    }

    CM_lleg = CCM_LLeg*current_leg_jacobian_l_inv_*lp_;
    CM_rleg = CCM_RLeg*current_leg_jacobian_r_inv_*rp_;


//    cout<<"Centroial momentum check : "<<endl<<"cm leg   "<<CM_lleg<<endl<<"CCM_lleg "<<CCM_LLeg<<endl;

//    Centroidal_Momentum_[28] = CM_lleg + CM_rleg;

    file[15]<<walking_tick_;
    for(int i=0;i<6;i++){
        file[15]<<"\t"<<Centroidal_Momentum(i);
    }


    Eigen::Vector6d CAM_at_pel;
    CAM_at_pel =  COM_Projection_Matrix_[28].transpose()*Centroidal_Momentum_[28];

    file[15]<<"\t"<<current_Angular_momentum_(0)<<"\t"<<current_Angular_momentum_(1)<<"\t"<<current_Angular_momentum_(2)<<"\t"<<CM_lleg(5)<<"\t"<<CM_rleg(5)<<"\t"<<CM_lleg(5)+CM_rleg(5)
           <<"\t"<<G_vel(0)<<"\t"<<G_vel(1)<<"\t"<<G_vel(2)<<"\t"<<G_vel(3)<<"\t"<<G_vel(4)<<"\t"<<G_vel(5)
          <<"\t"<<CAM_at_pel(0)<<"\t"<<CAM_at_pel(1)<<"\t"<<CAM_at_pel(2)<<"\t"<<CAM_at_pel(3)<<"\t"<<CAM_at_pel(4)<<"\t"<<CAM_at_pel(5)<<endl;


}
void WalkingController::Centroidal_Dynamics(){
 ////// refer Centroidal dynamics of Humanoid robot , David E. Orin, Ambarish Goswami, Sung-Hee Lee, 'Auton Robot, 2013'
 /// ' https://link.springer.com/content/pdf/10.1007/s10514-013-9341-4.pdf '
 ///

//    Relative_link_position(link_transform_);
//    Relative_link_rotation(link_transform_);





//    if(walking_tick_ == 0){
//        for(int i=0;i<29;i++){
//            file[29]<<i<<"\t"<<link_transform_[i].translation()(0)<<"\t"<<link_transform_[i].translation()(1)<<"\t"<<link_transform_[i].translation()(2)
//                   <<"\t"<<relative_distance_[i](0)<<"\t"<<relative_distance_[i](1)<<"\t"<<relative_distance_[i](2)<<endl;
//            file[28]<<i<<"\t"<<link_transform_[i].linear()(0,0)<<"\t"<<link_transform_[i].linear()(0,1)<<"\t"<<link_transform_[i].linear()(0,2)<<"\t"<<relative_rotation_[i](0,0)<<"\t"<<relative_rotation_[i](0,1)<<"\t"<<relative_rotation_[i](0,2)<<endl
//                   <<i<<"\t"<<link_transform_[i].linear()(1,0)<<"\t"<<link_transform_[i].linear()(1,1)<<"\t"<<link_transform_[i].linear()(1,2)<<"\t"<<relative_rotation_[i](1,0)<<"\t"<<relative_rotation_[i](1,1)<<"\t"<<relative_rotation_[i](1,2)<<endl
//                     <<i<<"\t"<<link_transform_[i].linear()(2,0)<<"\t"<<link_transform_[i].linear()(2,1)<<"\t"<<link_transform_[i].linear()(2,2)<<"\t"<<relative_rotation_[i](2,0)<<"\t"<<relative_rotation_[i](2,1)<<"\t"<<relative_rotation_[i](2,2)<<endl;
//        }
//    }


    if(walking_tick_ == 0){
        for(int i=0;i<29;i++){
            file[29]<<i<<"\t"<<link_transform_[i].translation()(0)<<"\t"<<link_transform_[i].translation()(1)<<"\t"<<link_transform_[i].translation()(2)
                   <<"\t"<<relative_link_distance_[i](0)<<"\t"<<relative_link_distance_[i](1)<<"\t"<<relative_link_distance_[i](2)<<endl;
            file[28]<<i<<"\t"<<link_transform_[i].linear()(0,0)<<"\t"<<link_transform_[i].linear()(0,1)<<"\t"<<link_transform_[i].linear()(0,2)<<"\t"<<relative_link_rotation_[i](0,0)<<"\t"<<relative_link_rotation_[i](0,1)<<"\t"<<relative_link_rotation_[i](0,2)<<endl
                   <<i<<"\t"<<link_transform_[i].linear()(1,0)<<"\t"<<link_transform_[i].linear()(1,1)<<"\t"<<link_transform_[i].linear()(1,2)<<"\t"<<relative_link_rotation_[i](1,0)<<"\t"<<relative_link_rotation_[i](1,1)<<"\t"<<relative_link_rotation_[i](1,2)<<endl
                     <<i<<"\t"<<link_transform_[i].linear()(2,0)<<"\t"<<link_transform_[i].linear()(2,1)<<"\t"<<link_transform_[i].linear()(2,2)<<"\t"<<relative_link_rotation_[i](2,0)<<"\t"<<relative_link_rotation_[i](2,1)<<"\t"<<relative_link_rotation_[i](2,2)<<endl;
        }
    }



//    Relative_Inertia();

//    if(walking_tick_ == 0){
//        for(int i=0;i<29;i++){
//            for(int j=0;j<6;j++){
//                for(int k=0;k<6;k++)
//                    file[15]<<Spatial_Inertia_[i](j,k)<<"\t";
//                file[15]<<endl;
//            }
//            file[15]<<i<<"\t"<<i<<"\t"<<i<<"\t"<<i<<"\t"<<i<<"\t"<<i<<"\t"<<endl;
//        }
//        file[15]<<endl<<endl;
//    }
    relative_link_trans_matrix(link_transform_);
    Spatial_transform();
    Joint_Spatial_Inertia();
    System_Spatial_Inertia();

    Projection_Spatial_COM();
    Centroidal_Momentum_Matrix();


//    if(walking_tick_ == 0){
//        for(int i=0;i<29;i++){
//            for(int j=0;j<6;j++){
//                for(int k=0;k<6;k++)
//                    file[15]<<Spatial_Inertia_[i](j,k)<<"\t";
//                file[15]<<endl;
//            }
//            file[15]<<i<<"\t"<<i<<"\t"<<i<<"\t"<<i<<"\t"<<i<<"\t"<<i<<"\t"<<endl;
//        }
//        file[15]<<endl<<endl;
//    }





    //if(_cnt==0)


    ////////////////////////////////
    ////  test for re trial    /////
    ////////////////////////////////
//    relative_link_trans_matrix2(link_transform_);
//    Spatial_link_transform();

//    Link_Spatial_Inertia();
//    Composite_Rigid_Body_Inertia();
//    Spatial_tranform_to_COM();
//    getCentroidal_Momentum_Matrix();

    ////////////////////////////////
    ////  test for re trial    /////
    ////////////////////////////////
    ///
    //cout<<"centroidal dynamics : "<<_m_R[6]<<endl;

    //cout<<"check relative_distance : "<<_relative_distance[2]<<", _T_arm_global "<<_T_RArm_global[2].translation()<<endl;
    Eigen::Vector6d Centroidal_Momentum_matrix_dot[29];
    Eigen::Vector6d Centroidal_Momentum_dot[29];

    for(int i=0;i<29;i++){
        Centroidal_Momentum_matrix_dot[i] = (Centroidal_Momentum_Matrix_[i] - pre_Centroidal_Momentum_Matrix_[i])/hz_; // last CMM_dot[27] is h_g_dot (whole centroidal momentum dot)
        Centroidal_Momentum_dot[i] = Centroidal_Momentum_[i] - pre_Centroidal_Momentum_[i];
    }

    CM_R_leg_.setZero(); CM_L_leg_.setZero();


//    for(int i=0;i<6;i++){
//        CM_L_leg_ += Centroidal_Momentum_Matrix_[LF_BEGIN+i] * desired_leg_q_dot_filtered_(i)/hz_;
//        file[15]<<"\t"<<Centroidal_Momentum_Matrix_[LF_BEGIN+i](5) * desired_leg_q_dot_filtered_(i)/hz_;
//    }
////    Eigen::Vector6d RF_CM;
////    RF_CM = COM_Projection_Matrix_[RF_BEGIN+5].transepose()**rp_;
//    for(int i=0;i<6;i++){
////        CMM_R_leg_dot += Centroidal_Momentum_matrix_dot[RF_BEGIN+i]*_q_RFoot_dot(i);
////        CMM_L_leg_dot += Centroidal_Momentum_matrix_dot[LF_BEGIN+i]*_q_LFoot_dot(i);
//        //CM_R_leg_ += Centroidal_Momentum_Matrix_[RF_BEGIN+i] * desired_leg_q_dot_(6+i);
//        //CM_L_leg_ += Centroidal_Momentum_Matrix_[LF_BEGIN+i] * desired_leg_q_dot_(i);
//        CM_R_leg_ += Centroidal_Momentum_Matrix_[RF_BEGIN+i] * desired_leg_q_dot_filtered_(6+i)/hz_;
//        file[15]<<"\t"<<Centroidal_Momentum_Matrix_[RF_BEGIN+i](5)* desired_leg_q_dot_filtered_(6+i)/hz_;

//        //CM_L_leg_ += Centroidal_Momentum_Matrix_[LF_BEGIN+i] * desired_leg_q_dot_filtered_(i);
//    }

//    for(int i=0;i<12;i++)
//        file[15]<<"\t"<<desired_leg_q_dot_filtered_(i);

//        file[15]<<endl;


    Eigen::Vector6d momentum_temp;
    momentum_temp.setZero();

    Eigen::Matrix<double, 6, 6> Gain;
    Gain.setIdentity();
    Gain *= 20.0*Gain;

    //momentum_temp = Gain*(-Centroidal_Momentum_Matrix_[28]) - CMM_R_leg_dot - CMM_L_leg_dot;
    //momentum_temp = Centroidal_Momentum_[28] -CMM_R_leg_dot - CMM_L_leg_dot;
    momentum_temp =  - CM_R_leg_ - CM_L_leg_;

    double temp_dot;
    //temp_dot = CMM_dot[WA_BEGIN](5);
    temp_dot = Centroidal_Momentum_Matrix_[WA_BEGIN](5);

    Eigen::Matrix3d temp_A;
    Eigen::Vector3d momentum_3_size;

    for(int i=0;i<3;i++){
        temp_A(i,0) = Centroidal_Momentum_Matrix_[WA_BEGIN](i+3);
        temp_A(i,1) = Centroidal_Momentum_Matrix_[RA_BEGIN](i+3);
        temp_A(i,2) = Centroidal_Momentum_Matrix_[LA_BEGIN](i+3);

        momentum_3_size(i) = momentum_temp(i+3);
    }
//    cout<<"waist momentum : "<<Centroidal_Momentum_Matrix_[WA_BEGIN]<<endl;// ---why the value is small ??
//    cout<<"Rarm momentum : "<<Centroidal_Momentum_Matrix_[RA_BEGIN]<<endl;
//    cout<<"Larm momentum : "<<Centroidal_Momentum_Matrix_[LA_BEGIN]<<endl;
//    cout<<"momentum temp : "<<momentum_temp<<endl;
//    cout<<"waist momentum dot : "<<CMM_dot[WA_BEGIN]<<endl;

    Eigen::Vector3d joint_dot_temp;

    joint_dot_temp = temp_A.inverse() * momentum_3_size;
//    q_dot_CMM_(0) = momentum_temp(5)/(temp_dot*100);

    q_dot_CMM_(0) = joint_dot_temp(0);
    q_dot_CMM_(1) = joint_dot_temp(1);
    q_dot_CMM_(2) = joint_dot_temp(2);

   // file[15]<<walking_tick_<<"\t"<<joint_dot_temp(0)<<"\t"<<joint_dot_temp(1)<<"\t"<<joint_dot_temp(2)<<endl;

//    file[18]<<_cnt<<"\t"<<q_dot_CMM_(0)/hz_<<"\t"<<1<<"\t"<<momentum_temp(5);
//    file[22]<<_cnt<<"\t"<<CM_R_leg_(0)<<"\t"<<CM_R_leg_(1)<<"\t"<<CM_R_leg_(2)<<"\t"<<CM_R_leg_(3)<<"\t"<<CM_R_leg_(4)<<"\t"<<CM_R_leg_(5)<<endl;
//    file[23]<<_cnt<<"\t"<<CM_L_leg_(0)<<"\t"<<CM_L_leg_(1)<<"\t"<<CM_L_leg_(2)<<"\t"<<CM_L_leg_(3)<<"\t"<<CM_L_leg_(4)<<"\t"<<CM_L_leg_(5)<<endl;
//    file[24]<<_cnt<<"\t"<<momentum_temp(0)<<"\t"<<momentum_temp(1)<<"\t"<<momentum_temp(2)<<"\t"<<momentum_temp(3)<<"\t"<<momentum_temp(4)<<"\t"<<momentum_temp(5)<<endl;
//    cout<<"waist yaw dot : "<<q_dot_CMM_<<endl;

    //calculate CCRBI(Centoridal composite rigid body inertia

    CCRBI_ = COM_Projection_Matrix_[28].transpose()*System_Inertia_[28]*COM_Projection_Matrix_[28];
    average_spatial_velocity_ = CCRBI_.inverse()*Centroidal_Momentum_[28];

}
void WalkingController::Jacobian_based_IK(){

}
void WalkingController::UpdateCentroidalMomentumMatrix(){
    Eigen::Matrix<double, DyrosJetModel::MODEL_WITH_VIRTUAL_DOF, 1> q_temp, qdot_temp;
    q_temp.setZero();
    qdot_temp.setZero();

    q_temp.segment<12>(6) = desired_q_not_compensated_.segment<12>(0);
    if(walking_tick_ == 0)
        q_temp.segment<28>(6) = current_q_.segment<28>(0);

    Eigen::Matrix<double, 3, 28> LMM_rbdl;
    Eigen::Matrix<double, 3, 28> AMM_rbdl;

    for(int i=0;i<28;i++){
        qdot_temp.setZero();
        qdot_temp(6+i) = 1.0;

        model_.updateKinematics(q_temp,qdot_temp);

        LMM_rbdl.col(i) = model_.getCurrentComLinearMomentum();
        AMM_rbdl.col(i) = model_.getCurrentComAngularMomentum();
    }

    Augmented_Centroidal_Momentum_Matrix_.block<3,28>(0,0) = LMM_rbdl;
    Augmented_Centroidal_Momentum_Matrix_.block<3,28>(3,0) = AMM_rbdl;

}
}
