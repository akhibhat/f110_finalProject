#include <ros/ros.h>
#include <Eigen/Dense>
#include "planner/mpc_tracker.h"
#include <unsupported/Eigen/MatrixFunctions>


MPC::MPC(ros::NodeHandle &nh): nh_(nh), listener(tfBuffer) 
{
	get_params(nh_);
	//add publisher to drive
	//add subscriber to reference trajectory
}

void MPC::get_params(ros::NodeHandle& nh)
{
	nh.getParam("N", N);
	nh.getParam("Ts", Ts);
	nh.getParam("max_speed", max_speed);
	nh.getParam("max_steer", max_steer);
	nh.getparam("C_l", C_l);
	nh.getParam("q_x", q_x);
	nh.getParam("q_y", q_y);
	nh.getParam("q_yaw", q_yaw);
	nh.getParam("r_v", r_v);
	nh.getParam("r_steer", r_steer);
	nh.getParam("nx", nx);
	nh.getParam("nu", nu);

	Q.setZero(); R.setZero();
    Q.diagonal() << q_x, q_y, q_yaw;
    R.diagonal() << r_v, r_steer;
}


//This member function propagates the car state by one timestep using the dynamics
void MPC::propagate_Dynamics(Eigen::VectorXd& state(nx), Eigen::VectorXd& input(nu), Eigen::VectorXd& next_state(nx), double dt)
{
	Eigen::VectorXd dynamics(state.size());
    dynamics(0) = input(0)*cos(state(2));
    dynamics(1) = input(0)*sin(state(2));
    dynamics(2) = tan(input(1))*input(0)/C_l;
    next_state = state + dynamics * dt;
}


//This function member will generate the control inputs for N timesteps in the horizon...
//...using the reference trajectory obtained from Dubin's Path generator
void MPC::get_MPC_path(vector<Eigen::VectorXd>& ref_trajectory, vector<Eigen::VectorXd>& ref_input, double& current_speed)
{
	//define the Hessian and Constraint matrix
	Eigen::SparseMatrix<double> H_matrix((N+1)*(nx+nu), (N+1)*(nx+nu));
	Eigen::SparseMatrix<double> A_c((N+1)*nx + 2*(N+1) + (N+1)*nu, (N+1)*(nx+nu));

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

	double A11, A12, A21, A22, B11, B22;
	get_half_space_constraints(constraint_points, A11, A12, A21, A22, B11, B22);


	for(int i=0; i<N+1; i++)
	{
		x_ref = ref_trajectory[i]; //get the reference state at "i"th timestep
		u_ref = ref_input[i];
		get_linear_dynamics_car(Ad, Bd, hd, x_ref, u_ref); //get discretized dynamic matrices

		//fill the H_matrix with state cost Q for the first (N+1)*nx diagonal and input cost R along the next (N+1)*nu diagonal
		if (i > 0)
		{
			for(int row = 0; row < nx; row++)
			{
				H_matrix.insert(i*nx + row, i*nx + row) = Q(row, row);
			}

			for(int row = 0; row < nu; row++)
			{
				H_matrix.insert(((N+1)*nx) + (i*nu + row), ((N+1)*nx) + (i*nu + row)) = R(row, row);
			}

			//g matrix with first (N+1)*nx being -Qx_ref and remaining (N+1)*nu with -Ru_ref
			g.segment<nx>(i*nx) << -Q*x_ref;
			g.segment<nu>(((N+1)*nx)+i*nu) << -R*u_ref;
		}

		//fill the constraint matrix first with the dynamic constraint x_k+1 = Ad*x_k + Bd*u_k + hd
		if (i < N)
		{
			for(int row = 0; row < nx; row++)
			{
				for(int col = 0; col < nx; col++)
				{
					A_c.insert((i+1)*nx + row, i*nx + col) = Ad(row, col);
				}
			}

			for(int row = 0; row < nx; row++)
			{
				for(int col = 0; col < nu; col++)
				{
					A_c.insert((i+1)*nx + row, (N+1)*nx + i*nu + col) = Bd(row, col);
				}
			}

			lb.segment<nx>((i+1)*nx) = -hd;
			ub.segment<nx>((i+1)*nx) = -hd;
		}

		//fill identity for x_k+1
		for(int row = 0; row < nx; row++)
		{
			A_c.insert(i*nx + row, i*nx + row) = -1.0;
		}

		//fill Ax <= B
		A_c.insert(((N+1)*nx) + 2*i, (i*nx)) = A11;
		A_c.insert(((N+1)*nx) + 2*i, (i*nx) + 1) = A12;

		A_c.insert(((N + 1)*nx) + 2*i + 1, (i*nx)) = A21;
		A_c.insert(((N + 1)*nx) + 2*i + 1, (i*nx) + 1) = A22;

		lb(((N+1)*nx) + 2*i) = -OsqpEigen::INFTY;
		ub(((N+1)*nx) + 2*i) = B11;
		
		lb(((N + 1)*nx) + 2*i + 1) = -OsqpEigen::INFTY;
		ub(((N + 1)*nx) + 2*i + 1) = B12;

		//fill u_min < u < u_max in A_c
		for(int row = 0; row < nu; row++)
		{
			A_c.insert((N+1)*nx + 2*(N+1) + i*nu + row, (N+1)*nx + i*nu + row) = 1.0;
		}

		//fill u_min in lb and u_max in ub
		lb((N+1)*nx + 2*(N+1) + i*nu) = 0.0;
		ub((N+1)*nx + 2*(N+1) + i*nu) = max_speed;

		lb((N+1)*nx + 2*(N+1) + i*nu + 1) = -max_steer;
		ub((N+1)*nx + 2*(N+1) + i*nu + 1) = max_steer;

	}

	//fill initial condition in lb and ub
	lb.head(nx) = -ref_trajectory[0];
	ub.head(nx) = -ref_trajectory[0];
	lb((N+1)*nx + 2*(N+1)) = current_speed;
	ub((N+1)*nx + 2*(N+1)) = current_speed;

	SparseMatrix<double> H_matrix_T = H_matrix.transpose();
	SparseMatrix<double> sparse_I((N+1)*(nx+nu),(N+1)*(nx+nu));
    sparse_I.setIdentity();
	H_matrix = 0.5(H_matrix + H_matrix_T) + 0.0000001*sparse_I;


	//qsqp Eigen Solver from: https://robotology.github.io/osqp-eigen/doxygen/doc/html/index.html
	//instantiate the solver
	OsqpEigen::Solver solver;

	solver.settings()->setWarmStart(true);
	solver.data()->setNumberOfVariables((N+1)*(nx + nu));
	solver.data()->setNumberOfConstraints((N+1)*nx + 2*(N+1) + (N+1)*nu);

	if(!solver.data()->setHessianMatrix(hessian)) return 1;
	if(!solver.data()->setGradient(gradient)) return 1;
	if(!solver.data()->setLinearConstraintMatrix(linearMatrix)) return 1;
	if(!solver.data()->setLowerBound(lowerBound)) return 1;
	if(!solver.data()->setUpperBound(upperBound)) return 1;

	if(!solver.initSolver()) return 1;

	if(!solver.solve()) return 1;

	Eigen::VectorXd QPSolution = solver.getSolution();

	execute_MPC_path(QPSolution);	

}

void MPC::get_linear_dynamics_car(Eigen::Matrix<double,nx,nx>& Ad, Eigen::Matrix<double,nx, nu>& Bd, Eigen::Matrix<double,nx,1>& hd, Eigen::Matrix<double,nx,1>& state, Eigen::Matrix<double,nu,1>& input)
{
	double yaw = state(2);
	double v = input(0);
	double steer = input(1);

	Eigen::VectorXd dynamics(state.size());
    dynamics(0) = input(0)*cos(state(2));
    dynamics(1) = input(0)*sin(state(2));
    dynamics(2) = tan(input(1))*input(0)/C_l;

    Eigen::Matrix<double, nx, nx> Ak, M12;
    Eigen::Matrix<double, nx, nu> Bk;

    Ak << 0.0, 0.0, (-v*sin(yaw)), 0.0, 0.0, (v*cos(yaw)), 0.0, 0.0, 0.0;
    Bk << cos(yaw), 0.0, sin(yaw), 0.0, tan(steer)/C_l, v/(cos(steer)*cos(steer)*C_l);

    //from document: https://www.diva-portal.org/smash/get/diva2:1241535/FULLTEXT01.pdf, page 50
    Eigen::Matrix<double,nx+nx,nx+nx> aux, M;
    aux.setZero();
    aux.block<nx,nx>(0,0) << Ak;
    aux.block<nx,nx>(0, nx) << Eigen::Matrix3d::Identity();
    M = (aux*Ts).exp();
    M12 = M.block<nx,nx>(0,nx);

    Eigen::VectorXd hc(3);
    hc = dynamics - (Ak*state + Bk*input);

    //Discretize
    Ad = (Ak*Ts).exp();
    Bd = M12*Bk;
    hd = M12*hc;

}

void MPC::get_half_space_constraints(vector<vector<double>>& constraint_points, double& A11, double& A12, double& A21, double &A22, double& B11; double& B12)
{
	double xc1, xc2, xp1, xp2;
	double yc1, yc2, yp1, yp2;

	xc1 = constraint_points[0][0];
	yc1 = constraint_points[0][1];

	xc2 = constraint_points[1][0];
	yc2 = constraint_points[1][1];

	xp1 = constraint_points[2][0];
	yp1 = constraint_points[2][1];

	xp2 = constraint_points[3][0];
	yp2 = constraint_points[3][1];

	A11 = yc1 - yp1;
	A12 = xp1 - xc1;
	A21 = yp2 - yc2;
	A22 = xc2 - xp2;

	B11 = yc1*xp1 - yp1*xc1;
	B12 = yp2*xc2 - yc2*xp2;
}

void MPC::convert_waypoints_to_vector3d(vector<Waypoint>& waypoints)
{
	vector<Eigen::VectorXd> ref_trajectory;
	vector<Eigen::VectorXd> ref_input;
	for(int i=0; i < waypoints.size(); i++)
	{
		Eigen::VectorXd traj(nx);
		Eigen::VectorXd inp(nu);

		traj(0) = waypoints[i].x;
		traj(1) = waypoints[i].y;
		traj(2) = waypoints[i].yaw;

		ref_trajectory.push_back(traj);
	}

	return ref_trajectory;
}

void MPC::execute_MPC_path(Eigen::VectorXd& QPSolution)
{
	double speed = QPSolution((N+1)*nx);
	double steering_angle = QPSolution((N+1)*nx + 1);

	if (steering_angle > 0.413) {steering_angle = 0.413;}
	if (steering_angle < -0.413) {steering_angle = -0.413;}

	ackermann_msgs::AckermannDriveStamped ack_msg;
    ack_msg.drive.speed = speed;
    ack_msg.drive.steering_angle = steer;
    ack_msg.drive.steering_angle_velocity = 1.0;
    // drive_pub_.publish(ack_msg);
}
