/**
 * This is an auto-generated file by Optimization Engine (OpEn)
 * OpEn is a free open-source software - see doc.optimization-engine.xyz
 * dually licensed under the MIT and Apache v2 licences.
 *
 */
#include "ros/ros.h"
#include "open_nmpc_controller/OptimizationResult.h"
#include "open_nmpc_controller/OptimizationParameters.h"
#include "mpc_controller_bindings.hpp"
#include "open_optimizer.hpp"

#include <std_msgs/UInt8.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/bind.hpp>
#include <cmath>
#include <algorithm>

namespace open_nmpc_controller {
/**
 * Class open_nmpc_controller::OptimizationEngineManager manages the
 * exchange of data between the input and output topics
 * of this node
 */
class OptimizationEngineManager {
/**
 * Private fields and methods
 */
private:
    static const int NX = 6;
    static const int NY = 4;
    static const int NU = 2;
    static const int NP = 2;
    static const int NO = 90;
    static const int T = 10; // Horizon
    static const int N = 3; // Number of agents
    static constexpr double sampling_time = 0.1;
    static const int f = 8; // down sample factor
    
    double current_pos[NX] = {0};
    std::vector<std::pair<double, double>> robot_positions;
    std::vector<std::vector<std::tuple<double, double, double, double>>> robot_traj;

    double last_yaw;
    double accumulated_yaw;
    bool first_yaw_reading; // Flags to check if it's the first reading of yaw

    // Laser precessing
    double goal_x;
    double goal_y;
    struct Point2D {
        double x;
        double y;
    };
    std::vector<Point2D> processed_points;
    int N_0[N] = {0};
    double lidar_r = 0.024;

    std::vector<int> hierarchies;
    int pi_bar = N;

    /**
     * Optimization parameters announced on the corresponding
     * topic (open_nmpc_controller/parameters)
     */
    open_nmpc_controller::OptimizationParameters params;
    /**
     * Object containing the result (solution and solver
     * statistics), which will be announced on open_nmpc_controller/results
     */
    open_nmpc_controller::OptimizationResult results;
    /**
     * Vector of parameters (provided by the client on
     * open_nmpc_controller/parameters)
     */
    double p[MPC_CONTROLLER_NUM_PARAMETERS] = { 0 };
    /**
     * Solution vector
     */
    double u[MPC_CONTROLLER_NUM_DECISION_VARIABLES] = { 0 };
    /**
     * Vector of Lagrange multipliers (if any)
     */
    double *y = NULL;
    /**
     * Workspace variable used by the solver - initialised once
     */
    mpc_controllerCache* cache;
    /**
     * Initial guess for the penalty parameter
     */
    double init_penalty = ROS_NODE_MPC_CONTROLLER_DEFAULT_INITIAL_PENALTY;

    /**
     * Publish obtained results to output topic
     */
    void publishToTopic(ros::Publisher& publisher)
    {
        publisher.publish(results);
    }

    /**
     * Updates the input data based on the data that are posted
     * on /mpc/open_parameters (copies value from topic data to
     * local variables). This method is responsible for parsing
     * the data announced on the input topic.
     */
    void updateInputData()
    {
        init_penalty = (params.initial_penalty > 1.0)
            ? params.initial_penalty
            : ROS_NODE_MPC_CONTROLLER_DEFAULT_INITIAL_PENALTY;

        if (params.parameter.size() > 0) {
            for (size_t i = 0; i < MPC_CONTROLLER_NUM_PARAMETERS; ++i)
                p[i] = params.parameter[i];
        }

        if (params.initial_guess.size() == MPC_CONTROLLER_NUM_DECISION_VARIABLES) {
            for (size_t i = 0; i < MPC_CONTROLLER_NUM_DECISION_VARIABLES; ++i)
                u[i] = params.initial_guess[i];
        }

		if (params.initial_y.size() == MPC_CONTROLLER_N1) {
            for (size_t i = 0; i < MPC_CONTROLLER_N1; ++i)
                y[i] = params.initial_y[i];
		}
    }

    /**
     * Call OpEn to solve the problem
     */
    mpc_controllerSolverStatus solve()
    {
        return mpc_controller_solve(cache, u, p, y, &init_penalty);
    }
/**
 * Public fields and methods
 */
public:
    /**
     * Constructor of OptimizationEngineManager
     */
    OptimizationEngineManager()
    {
	    y = new double[MPC_CONTROLLER_N1];
        cache = mpc_controller_new();

        robot_traj.resize(N);
        hierarchies.resize(N);
        for (int i=0; i<N; i++){
            robot_traj[i].resize(T);
            hierarchies[i] = pi_bar;
        }
        robot_positions.resize(N, std::make_pair(0.0, 0.0));

        first_yaw_reading = true;

        processed_points.resize(NO, Point2D{1e4, 1e4});
    }

    /**
     * Destructor of OptimizationEngineManager
     */
    ~OptimizationEngineManager()
    {
		if (y!=NULL) delete[] y;
        mpc_controller_free(cache);
    }

    int getN() const {
        return N;
    }

    /**
     * Copies results from `status` to the local field `results`
     */
    void updateResults(mpc_controllerSolverStatus& status)
    {
        std::vector<double> sol(u, u + MPC_CONTROLLER_NUM_DECISION_VARIABLES);
        results.solution = sol;
        std::vector<double> y(status.lagrange, status.lagrange + MPC_CONTROLLER_N1);
        results.lagrange_multipliers = y;
        results.inner_iterations = status.num_inner_iterations;
        results.outer_iterations = status.num_outer_iterations;
        results.norm_fpr = status.last_problem_norm_fpr;
        results.cost = status.cost;
        results.penalty = status.penalty;
        results.status = (int)status.exit_status;
        results.solve_time_ms = (double)status.solve_time_ns / 1000000.0;
        results.infeasibility_f2 = status.f2_norm;
        results.infeasibility_f1 = status.delta_y_norm_over_c;
    }

    /**
     * Callback that obtains data from topic `/open_nmpc_controller/open_params`
     */
    void mpcReceiveRequestCallback(
        const open_nmpc_controller::OptimizationParameters::ConstPtr& msg)
    {
        params = *msg;
    }

    void solveAndPublish(ros::Publisher& publisher)
    {
        updateInputData(); /* get input data */
        mpc_controllerSolverStatus status = solve(); /* solve!  */
        updateResults(status); /* pack results into `results` */
        publishToTopic(publisher);
    }

    void trajCallback(const nav_msgs::Path::ConstPtr& msg, int robot_index){
        for (int i=0; i<T; i++){
            if (!msg->poses.empty()){
                double x_position = msg->poses[i].pose.position.x;
                double y_position = msg->poses[i].pose.position.y;
                // In this project, orientation msg is velocity in global frame
                double x_velocity = msg->poses[i].pose.orientation.x;
                double y_velocity = msg->poses[i].pose.orientation.y;

                robot_traj[robot_index][i] = std::make_tuple(x_position,
                                                                y_position,
                                                                x_velocity,
                                                                y_velocity);
            }
            else{
                ROS_WARN("traj msg is empty!");
            }
        }
        //ROS_INFO("neigh_x_%d[0]: %f",robot_index, std::get<0>(robot_traj[robot_index][0]));
    }

    void hierarchyCallback(const std_msgs::UInt8::ConstPtr& msg, int robot_index){
        hierarchies[robot_index] = msg->data;
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
        // Goal position in the robot frame
        //double x_g = (goal_x-current_pos[0])*cos(current_pos[2])+(goal_y-current_pos[1])*sin(current_pos[2]);
        //double y_g = (current_pos[0]-goal_x)*sin(current_pos[2])+(goal_y-current_pos[1])*cos(current_pos[2]);
        //double goal_angle = atan2(y_g, x_g);
        /*double theta_1 = goal_angle - M_PI/2;
        double theta_2 = goal_angle + M_PI/2;*/
        double theta_1 = -M_PI;
        double theta_2 = M_PI;

        // Clear the processed_points and resize to ensure it has NO elements initialized to {1e4, 1e4}
        processed_points.clear();
        processed_points.resize(NO, Point2D{1e4, 1e4});

        // Filter the scan data points within theta_1 and theta_2, replacing 'inf' with 1e4
        std::vector<std::pair<float, size_t>> filtered_ranges;
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float angle = msg->angle_min + i * msg->angle_increment;
            // Change to 1e4 if the value is inf or nan
            float range = std::isinf(msg->ranges[i]) || std::isnan(msg->ranges[i]) ? 1e4 : msg->ranges[i];
            if (angle >= theta_1 && angle <= theta_2) {
                filtered_ranges.push_back({range, i});
            }
        }

        // Apply downsampling on the filtered data
        int index = 0; // Index for placing points in processed_points
        for (size_t i = 0; i < filtered_ranges.size(); i += f) {
            if (index >= NO) {
                break; // Avoid exceeding the bounds of processed_points
            }
            auto min_it = std::min_element(filtered_ranges.begin() + i,
                                        std::min(filtered_ranges.begin() + i + f, filtered_ranges.end()),
                                        [](const std::pair<float, size_t>& a, const std::pair<float, size_t>& b) {
                                            return a.first < b.first;
                                        });

            // Calculate the angle for the point with the minimum range
            float angle = msg->angle_min + min_it->second * msg->angle_increment;

            // Convert this point to Cartesian coordinates in the robot frame
            Point2D point_in_robot_frame;
            point_in_robot_frame.x = min_it->first * cos(angle);
            point_in_robot_frame.y = min_it->first * sin(angle);

            // Transform the point to the global frame
            Point2D point_in_global_frame;
            point_in_global_frame.x = current_pos[0] + point_in_robot_frame.x * cos(current_pos[2]) - point_in_robot_frame.y * sin(current_pos[2]);
            point_in_global_frame.y = current_pos[1] + point_in_robot_frame.x * sin(current_pos[2]) + point_in_robot_frame.y * cos(current_pos[2]);

            // Exclude points that are within another detected robot
            bool within_another_robot = false;
            for (int j = 0; j < N; j++) {
                if (N_0[j] == 1) {
                    double dx_sqr = std::pow(point_in_global_frame.x - robot_positions[j].first, 2);
                    double dy_sqr = std::pow(point_in_global_frame.y - robot_positions[j].second, 2);
                    if (std::sqrt(dx_sqr + dy_sqr) <= lidar_r + 0.1) {
                        within_another_robot = true;
                        break;
                    }
                }
            }

            if (!within_another_robot) {
                processed_points[index++] = point_in_global_frame;
            }
        }
    }

    void hierarchyUpdate(int array[][T], int robot_index){
        int minElement = pi_bar;
        for (int i; i<N; i++){
            if (array[i][0]==1){
                minElement = std::min(minElement, hierarchies[i]);
            }
        }
        hierarchies[robot_index] = std::min(pi_bar, 1+minElement);
    }

    double dotProduct(double v_x, double v_y, double dx, double dy){
        return v_x*dx+v_y*dy;
    }

    void mocapCallback(const gazebo_msgs::ModelStates::ConstPtr& msg, int robot_index){
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i].find("husky_") != std::string::npos) {
                int index = std::stoi(msg->name[i].substr(6)); // Extracts the number after "husky_"
                if (index==robot_index){
                    double roll, pitch, yaw;
                    tf::Quaternion quat(msg->pose[i].orientation.x,
                                        msg->pose[i].orientation.y,
                                        msg->pose[i].orientation.z,
                                        msg->pose[i].orientation.w);
                    tf::Matrix3x3 m(quat);
                    m.getRPY(roll, pitch, yaw);

                    // If it's the first time reading the yaw, initialize the last_yaw and accumulated_yaw
                    if (first_yaw_reading) {
                        last_yaw = yaw;
                        accumulated_yaw = yaw;
                        first_yaw_reading = false;
                    } else {
                        // Compute the smallest difference between the new yaw and the last yaw
                        double yaw_diff = yaw - last_yaw;
                        // Adjust for overflow (e.g., from pi to -pi)
                        if (yaw_diff > M_PI) {
                            yaw_diff -= 2 * M_PI;
                        } else if (yaw_diff < -M_PI) {
                            yaw_diff += 2 * M_PI;
                        }

                        // Accumulate the yaw difference
                        accumulated_yaw += yaw_diff;

                        // Update the last yaw
                        last_yaw = yaw;
                    }

                    current_pos[0] = msg->pose[i].position.x;
                    current_pos[1] = msg->pose[i].position.y;
                    current_pos[2] = accumulated_yaw;
                    //ROS_INFO("yaw_%d: %f",i, yaw);
                    current_pos[3] = msg->twist[i].linear.x;
                    current_pos[4] = msg->twist[i].linear.y;
                    current_pos[5] = msg->twist[i].angular.z;

                    //ROS_INFO("current_pos_%d: %f",robot_index, std::get<0>(robot_traj[robot_index][0]));

                    robot_positions[index] = std::make_pair(current_pos[0], current_pos[1]);
                }
                else{
                    robot_positions[index] = std::make_pair(msg->pose[i].position.x, msg->pose[i].position.y);
                }
            }
        }
    }

    void solveAndPublishCmdVel(ros::Publisher& publisher_cmd, ros::Publisher& publisher_traj, ros::Publisher& publisher_hierarchy, int robot_index)
    {
        double current_par[MPC_CONTROLLER_NUM_PARAMETERS] = {0};
        double current_var[MPC_CONTROLLER_NUM_DECISION_VARIABLES] = {0};
        double lin_vel_cmd, ang_vel_cmd = 0;

        for (int i=0; i<NX; i++) {
            current_par[i] = current_pos[i];
        }

        // detection
        double detection_r = 6.0;
        int detection_array[N][T] = {{0}};
        /*for (int i=0; i<N; i++){
            if (i!=robot_index){
                double dx = robot_positions[i].first-robot_positions[robot_index].first;
                double dy = robot_positions[i].second-robot_positions[robot_index].second;
                //ROS_INFO("r: %f",std::pow(dx,2)+std::pow(dy,2) );
                if (std::pow(dx,2)+std::pow(dy,2) < std::pow(detection_r,2)){
                    detection_array[i] = 1;
                    //current_par[NX+NY*T+3+i] = 1; // N_t
                }
                else{
                    detection_array[i] = 0;
                    //current_par[NX+NY*T+3+i] = 0; // N_t
                }
            }
            else{
                detection_array[i] = 1;
                //current_par[NX+NY*T+3+i] = 0; // N_t
            }
        }*/
        for (int i=0; i<N; i++){
            if (i!=robot_index){
                for (int j=0; j<T; j++){
                    double dx = std::get<0>(robot_traj[i][j]) - std::get<0>(robot_traj[robot_index][j]);
                    double dy = std::get<1>(robot_traj[i][j]) - std::get<1>(robot_traj[robot_index][j]);
                    if (std::pow(dx,2)+std::pow(dy,2) < std::pow(detection_r,2)){
                        detection_array[i][j] = 1;
                        if (j==0){
                            N_0[i] = 1;
                        }
                    }
                    else{
                        if (j==0){
                            N_0[i] = 0;
                        }
                    }
                }
            }
            else {
                N_0[i] = 0;
            }
        }

        hierarchyUpdate(detection_array, robot_index);
        std_msgs::UInt8 pi_i;
        pi_i.data = hierarchies[robot_index];
        publisher_hierarchy.publish(pi_i);

        //double w_p[N] = {0.5, 0.25, 0.25};
        //double w_v = 0.33;
        double w_p[N][T] = {{0.0}};
        double w_v[N] = {0.0};
        for (int i=0; i<T; i++){
            for (int j=0; j<N; j++){
                double sum = 0;
                for (int k=0; k<N; k++){
                    if ((detection_array[k][i]==1) || (k==robot_index)){
                        sum+=std::pow(2,-hierarchies[k]);
                    }
                }
                w_p[j][i] = std::pow(2,-hierarchies[j])/sum;
            }
        }
        for (int i=0; i<N; i++){
            double dx = std::get<0>(robot_traj[i][0])-current_pos[0];
            double dy = std::get<1>(robot_traj[i][0])-current_pos[1];
            if (dotProduct(current_pos[3], current_pos[4], dx, dy)>=0){
                w_v[i] = 1;
            }
            else {
                w_v[i] = 0.5;
            }
        }

        double x_bar[T] = {0};
        double y_bar[T] = {0};
        double vx_bar[T] = {0};
        double vy_bar[T] = {0};

        for (int i=0; i<T; i++){
            for (int j=0; j<N;j++){
                x_bar[i] += w_p[j][i] * std::get<0>(robot_traj[j][i]);
                y_bar[i] += w_p[j][i] * std::get<1>(robot_traj[j][i]);
                vx_bar[i] += w_v[j] * std::get<2>(robot_traj[j][i]);
                vy_bar[i] += w_v[j] * std::get<3>(robot_traj[j][i]);
            }
            current_par[NX + NY * i] = x_bar[i];
            current_par[NX + NY * i + 1] = y_bar[i];
            current_par[NX + NY * i + 2] = vx_bar[i];
            current_par[NX + NY * i + 3] = vy_bar[i];
        }
        goal_x = x_bar[0];
        goal_y = y_bar[0];
        //ROS_INFO("%d x_bar: %f y_bar: %f x_0: %f",robot_index, x_bar[0], y_bar[0], std::get<0>(robot_traj[0][0]));

        double q_st = 0.5;
        double c = 10;
        current_par[NX+NY*T] = q_st/(1+c*(std::pow(current_pos[0]-x_bar[0],2)+std::pow(current_pos[1]-y_bar[0],2)));
        //current_par[NX+NY*T] = 0.0;
        // R
        current_par[NX+NY*T+1] = 0.005;
        current_par[NX+NY*T+2] = 0.025;
        // N_t
        /*for (int i=0; i<N;i++){
            if (i==robot_index){
                current_par[NX+NY*T+3+i] = 0;
            }
            else{
                current_par[NX+NY*T+3+i] = 1;
            }
        }*/
        int offset_N_t = NX+NY*T+3;
        int index = 0;
        for (int j=0; j<T; j++) {
            for (int i=0; i<N; i++) {
                current_par[offset_N_t+index++] = detection_array[i][j];
            }
        }
        // gamma
        double gamma[T] = {1.0, 1.0, 0.9, 0.9, 0.8, 0.8, 0.7, 0.7, 0.6, 0.6};
        for (int i=0; i<T; i++){
            current_par[NX+NY*T+3+N*T+i] = gamma[i];
        }
        // rho
        current_par[NX+NY*T+3+N*T+T] = 20.0;
        // r_min from the obstacles
        current_par[NX+NY*T+3+N*T+T+1] = 1.0;
        // p_t
        int offset_p_t = NX+NY*T+3+N*T+T+2;
        for (int i=0; i<T; ++i) {
            for (int j=0; j<N; ++j) {
                int index = 2 * (i * N + j);
                if (detection_array[j][i]==1){
                    current_par[offset_p_t+index] = std::get<0>(robot_traj[j][i]);
                    current_par[offset_p_t+index+1] = std::get<1>(robot_traj[j][i]);
                }
                else{
                    current_par[offset_p_t+index] = 1e4;
                    current_par[offset_p_t+index+1] = 1e4;
                }
            }
        } // NP*N*T elements in this loop
        // d_sep_t
        current_par[offset_p_t+NP*N*T] = 1.5;
        // obstacle
        int offset_obs = offset_p_t+NP*N*T+1;
        for (int i=0; i<NO; i++){
            current_par[offset_obs+NP*i]  = processed_points[i].x;
            current_par[offset_obs+NP*i+1]= processed_points[i].y;
        }

        /* solve                  */
        mpc_controllerSolverStatus status
            = mpc_controller_solve(cache, current_var, current_par, 0, &init_penalty);
        
        lin_vel_cmd = current_var[0];
        ang_vel_cmd = current_var[1];
    
        geometry_msgs::Twist twist;
        twist.linear.x = lin_vel_cmd;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = ang_vel_cmd;
        publisher_cmd.publish(twist);

        //params.initial_guess[0] = lin_vel_cmd;
        //params.initial_guess[1] = ang_vel_cmd;

        nav_msgs::Path traj;
        traj.poses.resize(T);

        traj.poses[0].pose.position.x = current_pos[0];
        traj.poses[0].pose.position.y = current_pos[1];
        // In this project, orientation msg is velocity in global frame
        traj.poses[0].pose.orientation.x = current_pos[3];
        traj.poses[0].pose.orientation.y = current_pos[4];

        robot_traj[robot_index][0] = std::make_tuple(current_pos[0],
                                                        current_pos[1],
                                                        current_pos[3],
                                                        current_pos[4]);
        std::vector<double> x_next;
        x_next.resize(NX);
        x_next = nextState(current_pos, lin_vel_cmd, ang_vel_cmd);
        for (int i = 1; i < T; i++){
            //param.initial_guess[i*NU] = current_var[i*NU];
            //param.initial_guess[i*NU+1] = current_var[i*NU+1];
            //params.initial_guess[i*NU] = current_var[i*NU];
            //params.initial_guess[i*NU+1] = current_var[i*NU+1];

            traj.poses[i].pose.position.x = x_next[0];
            traj.poses[i].pose.position.y = x_next[1];
            // In this project, orientation msg is velocity in global frame
            traj.poses[i].pose.orientation.x = x_next[3];
            traj.poses[i].pose.orientation.y = x_next[4];

            robot_traj[robot_index][i] = std::make_tuple(x_next[0],
                                                            x_next[1],
                                                            x_next[3],
                                                            x_next[4]);
            x_next = nextState(&x_next[0], current_var[i*NU], current_var[i*NU+1]);
        }
        
        publisher_traj.publish(traj);
        //ROS_INFO("x: %f, y: %f, yaw: %f", current_pos[0], current_pos[1], current_pos[2]);
        //ROS_INFO("Solve time: %f ms. I will send %f %f \n",
        //         (double)status.solve_time_ns / 1000000.0, lin_vel_cmd, ang_vel_cmd);
    }

    std::vector<double> nextState(double x[NX], double u_0, double u_1){
        std::vector<double> x_next(NX);
        x_next[0] = x[0] + sampling_time * u_0 * cos(x[2]);
        x_next[1] = x[1] + sampling_time * u_0 * sin(x[2]);
        x_next[2] = x[2] + sampling_time * u_1;
        x_next[3] = u_0 * cos(x[2]);
        x_next[4] = u_0 * sin(x[2]);
        x_next[5] = u_1;
        return x_next;
    }
}; /* end of class OptimizationEngineManager */
} /* end of namespace open_nmpc_controller */

/**
 * Main method
 *
 * This advertises a new (private) topic to which the optimizer
 * announces its solution and solution status and details. The
 * publisher topic is 'open_nmpc_controller/result'.
 *
 * It obtains inputs from 'open_nmpc_controller/parameters'.
 *
 */
int main(int argc, char** argv)
{
    std::string result_topic, params_topic;  /* parameter and result topics */
    double rate; /* rate of node (specified by parameter) */

    open_nmpc_controller::OptimizationEngineManager mng;
    ros::init(argc, argv, ROS_NODE_MPC_CONTROLLER_NODE_NAME);
    ros::NodeHandle nh, private_nh("~");

    /* obtain parameters from config/open_params.yaml file */
    private_nh.param("result_topic", result_topic,
                     std::string(ROS_NODE_MPC_CONTROLLER_RESULT_TOPIC));
    private_nh.param("params_topic", params_topic,
                     std::string(ROS_NODE_MPC_CONTROLLER_PARAMS_TOPIC));
    private_nh.param("rate", rate,
                     double(ROS_NODE_MPC_CONTROLLER_RATE));

    ros::Publisher mpc_pub
        = private_nh.advertise<open_nmpc_controller::OptimizationResult>(
            result_topic,
            ROS_NODE_MPC_CONTROLLER_RESULT_TOPIC_QUEUE_SIZE);
    /*ros::Subscriber sub
        = private_nh.subscribe(
            params_topic,
            ROS_NODE_MPC_CONTROLLER_PARAMS_TOPIC_QUEUE_SIZE,
            &open_nmpc_controller::OptimizationEngineManager::mpcReceiveRequestCallback,
            &mng);*/
    ros::Rate loop_rate(250);

    std::string id;
    int robot_id;
    if (argc >= 2){
        id = argv[1];
    }
    else {
        ROS_ERROR("Robot ID argument not provided. Exiting...");
        ros::shutdown();
        return 1;
    }
    robot_id = std::stoi(id);

    std::vector<ros::Subscriber> traj_subs;
    std::vector<ros::Subscriber> hierarchy_subs;
    static const int N = mng.getN();
    for (int i = 0; i < N; i++) {
        if (i != robot_id) {
            traj_subs.push_back(nh.subscribe<nav_msgs::Path>(
                "husky_" + std::to_string(i) + "/traj",
                10,
                boost::bind(&open_nmpc_controller::OptimizationEngineManager::trajCallback, &mng, _1, i)
            ));
            hierarchy_subs.push_back(nh.subscribe<std_msgs::UInt8>(
                "husky_" + std::to_string(i) + "/hierarchy",
                1,
                boost::bind(&open_nmpc_controller::OptimizationEngineManager::hierarchyCallback, &mng, _1, i)
            ));
        }
    }

    ros::Subscriber mocap_sub = nh.subscribe<gazebo_msgs::ModelStates>(
        "/gazebo/model_states",
        10,
        boost::bind(&open_nmpc_controller::OptimizationEngineManager::mocapCallback, &mng, _1, robot_id)
    );

    ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>(
        "husky_" + id + "/scan_filtered",
        10,
        boost::bind(&open_nmpc_controller::OptimizationEngineManager::scanCallback, &mng, _1)
    );

    ros::Publisher pub_twist_cmd = nh.advertise<geometry_msgs::Twist>("husky_" + id + "/cmd_vel", 10);

    ros::Publisher pub_traj = nh.advertise<nav_msgs::Path>("husky_" + id + "/traj", 10);

    ros::Publisher pub_hierarchy = nh.advertise<std_msgs::UInt8>("husky_" + id + "/hierarchy", 1);

    while (ros::ok()) {
        mng.solveAndPublishCmdVel(pub_twist_cmd, pub_traj, pub_hierarchy, robot_id);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}