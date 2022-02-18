#include <math.h>
#include "../include/ekf_slam/ekf_slam_solver.h"
using namespace Eigen;
using namespace std;
const double ZERO = 0.00001;
const double CLOSE_OBJECT = 0.2;
const double PI = 3.1415926535;

EkfSlamSolver::EkfSlamSolver()
{
     matrix_size = 3;
     object_size = 0;
     pose[0] = pose[1] = pose[2] = 0;
     pose_es = VectorXd::Zero(matrix_size);
     cov = MatrixXd::Identity(matrix_size,matrix_size)*0;
     // cov = MatrixXd::Zero(matrix_size,matrix_size);
    
     R << 0,0,0,
          0,0,0,
          0,0,0;
     Q << 0,0,
          0,0;        
}

EkfSlamSolver::EkfSlamSolver(const double & x, const double & y, const double & angle, const vector<double> & obstacle)
{
     if(obstacle.size()%2 == 0)
          object_size = obstacle.size()/2;
     else{
          cout<<"EkfSlamSolver: obstacle wrong input"<<endl;
          exit(0);
     }    
     // init pose estimation and covariance
     matrix_size = 3 + 2*object_size;
     pose_es = VectorXd::Zero(matrix_size);
     cov = MatrixXd::Identity(matrix_size,matrix_size)*0.001;
     // cov = MatrixXd::Zero(matrix_size,matrix_size);
        
     pose[0] = pose_es[0] = x;
     pose[1] = pose_es[1] = y;
     pose[2] = pose_es[2] = norm_angle(angle);
     for(size_t i=0; i<object_size; i++){
         pose_es[3+i*2] = obstacle[2*i];
         pose_es[3+i*2+1] = obstacle[2*i+1];
     }

     R << 0.0001,0,0,
          0,0.0001,0,
          0,0,0.001;
     Q << 0.0001,0,
          0,0.0001; 
}

void EkfSlamSolver::predict(const double & v, const double & w, const double & dt)
{
    Vector3d temp;
    MatrixXd F(matrix_size,3);
    Matrix3d temp_;
    MatrixXd G(matrix_size,matrix_size);

    F.setZero();
    temp_.setZero();
    G.setZero();

    temp[0] = v*dt*cos(pose[2]+0.5*w*dt);
    temp[1] = v*dt*sin(pose[2]+0.5*w*dt);
    temp[2] = w*dt;
    temp_(0,2) = -v*dt*sin(pose[2]+0.5*w*dt);
    temp_(1,2) = v*dt*cos(pose[2]+0.5*w*dt);

    F(0,0) = 1;
    F(1,1) = 1;
    F(2,2) = 1;
    pose_es =  pose_es + F*temp;
    pose_es[2] = norm_angle(pose_es[2]);
    pose[0] = pose_es[0]; pose[1] = pose_es[1]; pose[2] = pose_es[2];
    G = MatrixXd::Identity(matrix_size,matrix_size) + F*temp_*(F.transpose());
    cov = G*cov*(G.transpose()) + F*R*(F.transpose());
}

bool EkfSlamSolver::in_previous_observation(const double &x, const double &y, size_t & match_index)
{
     for(size_t i=0; i<object_size;i++){
          double distance = (pose_es[3+2*i]-x)*(pose_es[3+2*i]-x)+(pose_es[3+2*i+1]-y)*(pose_es[3+2*i+1]-y);
          if(distance<CLOSE_OBJECT*CLOSE_OBJECT){
               match_index = i;
               return 1;
          }
     }
     return 0;
}

void EkfSlamSolver::observe(PointSolver & point_solver)
{
     if(point_solver.new_point_cloud.size()==0)
          return;
     vector<PolarPoint> polar_objects;
     vector<CoordPoint> coord_objects;
     size_t object_number;
     point_solver.cluster_point_to_object(polar_objects);
     point_solver.polar_point_to_coord(pose[0],pose[1],pose[2],polar_objects,coord_objects);
     object_number = polar_objects.size();

     for(size_t i=0; i<object_number; i++){
          size_t match_index = 0;
          if(in_previous_observation(coord_objects[i].x,coord_objects[i].y,match_index)){
               observe_one(polar_objects[i].range,polar_objects[i].angle,match_index);
               // cout<<coord_objects[i].x<<" "<<coord_objects[i].y<<",match "<<match_index<<endl;
          }
          else{
               // cout<<"!!! cant recognize:"<<coord_objects[i].x<<" "<<coord_objects[i].y<<endl;
          }
     }
}

void EkfSlamSolver::observe_one(const double & r, const double & angle, const size_t & match_index)
{
     double delta_x = pose_es[3+2*match_index] - pose[0];
     double delta_y = pose_es[3+2*match_index+1] - pose[1];
     double q = delta_x*delta_x + delta_y*delta_y;
     double q_sqrt = pow(q,0.5);
     Vector2d z_obs,z_es;
     MatrixXd temp(2,5);
     MatrixXd F(5,matrix_size);
     MatrixXd H(2,matrix_size);
     MatrixXd K(matrix_size,2);

     if(q<ZERO){
          cout<<"Zero error, q is too small."<<endl;
          exit(0);
     }

     z_obs<<r, norm_angle(angle);
     z_es<<q_sqrt, norm_angle(atan2(delta_y,delta_x)-pose[2]);
     if((z_obs[1]-z_es[1])>PI)
          z_obs[1] -= 2*PI;
     if((z_obs[1]-z_es[1])<-PI)
          z_obs[1] += 2*PI;
     if((z_obs[1]-z_es[1])>PI || (z_obs[1]-z_es[1])<-PI)
          cout<<"angle error: "<<z_obs[1]<<" "<<z_es[1]<<endl;
 
     F.setZero();
     F(0,0) = F(1,1) = F(2,2) = F(3,3+2*match_index) = F(4,3+2*match_index+1) = 1;

     temp.setZero();
     temp<< -q_sqrt*delta_x, -q_sqrt*delta_y, 0, q_sqrt*delta_x, q_sqrt*delta_y,
            delta_y, -delta_x, -q, -delta_y, delta_x;
     H = (temp * (1/q)) * F;

     K = cov * H.transpose() * ((H*cov*H.transpose()+Q).inverse());
     pose_es = pose_es + K*(z_obs - z_es);
     cov = cov - K*H*cov;
     pose_es[2] = norm_angle(pose_es[2]);

     pose[0] = pose_es[0];
     pose[1] = pose_es[1];
     pose[2] = pose_es[2];

     return;
}

double EkfSlamSolver::norm_angle(const double & angle_)
{
     double angle = angle_;
     if(angle>PI){
          while(angle > PI){
               angle -= PI*2;
          }
          return angle;
     }
     if(angle<-PI){
          while(angle < -PI){
               angle += PI*2;
          }
          return angle;
     }
     return angle;
}