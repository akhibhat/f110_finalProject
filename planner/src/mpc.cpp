#include <ros/ros.h>
#include <Eigen/Dense>
#include "planner/mpc_tracker.h"
#include <unsupported/Eigen/MatrixFunctions>


MPC::MPC(ros::NodeHandle &nh): nh_(nh), listener(tfBuffer) 
{
	getParams(nh_);
}

void getParams(ros::NodeHandle& nh)
{
	nh.getParam("N", N);
	nh.getParam("dt", dt);
	nh.getParam("max_speed", max_speed);
	nh.getParam("max_steer", max_steer);
	// nh.getParam()
}


//This member function propagates the car state by one timestep using the dynamics
void propagate_Dynamics(Eigen::VectorXd& state(nx), Eigen::VectorXd& input(nu), Eigen::VectorXd& next_state(nx), double dt)
{
	VectorXd dynamics(state.size());
    dynamics(0) = input(0)*cos(state(2));
    dynamics(1) = input(0)*sin(state(2));
    dynamics(2) = tan(input(1))*input(0)/CAR_LENGTH;
    new_state = state + dynamics * dt;
}


//This function member will generate the control inputs for N timesteps in the horizon...
//...using the reference trajectory obtained from Dubin's Path generator
void MPC::getMPCpath()
{
	//define the Hessian and Constraint matrix
	Eigen::SparseMatrix<double> H_matrix((N+1)*(nx+nu), (N+1)*(nx+nu));
	Eigen::SparseMatrix<double> A_c(2*(N+1)*nx + (N+1)*nu);

	//define the gradient vector
	Eigen::VectorXd g((N+1)*(nx+nu));
	g.setZero();

	//the upper and lower bound constraint vectors
	Eigen::VectorXd lb(2*(N+1)*nx + (N+1)*nu);
	Eigen::VectorXd ub(2*(N+1)*nx + (N+1)*nu);

	//define the matrices(vectors) for state and control references at each time step
	Eigen::Matrix<double, nx, 1> x_ref;
	Eigen::Matrix<double, nu, 1> u_ref;

	//define the matrices for discrete dynamics
	Eigen::Matrix<double, nx, 1> hd;
	Eigen::Matrix<double, nx, nx> Ad;
	Eigen::Matrix<double, nx, nu> Bd;

	for(int i=0; i<N+1; i++)
	{
		x_ref = ref_trajectory_[i];

	}
	

}

void MPC::get_linear_dynamics(Matrix<double,nx,nx>& Ad, Matrix<double,nx, nu>& Bd, Matrix<double,nx,1>& hd, Matrix<double,nx,1>& x_op, Matrix<double,nu,1>& u_op)
{
	
}
