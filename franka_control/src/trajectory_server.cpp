#include "rclcpp/rclcpp.hpp"
#include "franka_control_services/srv/trajectory_cartesian.hpp"
#include "franka_control_services/srv/obstacle_deletion.hpp"
#include "franka_control_services/srv/obstacle_insertion.hpp"
#include "franka_control_services/srv/trajectory_joint_space.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include <moveit/utils/moveit_error_code.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using moveit::planning_interface::MoveGroupInterface;
    
class FrankaControlServer : public rclcpp::Node 
{
public:
    FrankaControlServer() : Node("trajectory_control_server", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) 
    {
        RCLCPP_INFO(this->get_logger(),"*** Trajectory Server ***");

        this->compute_cartesian_trajectory_ = this->create_service<franka_control_services::srv::TrajectoryCartesian>("compute_cartesian_trajectory", 
        std::bind(&FrankaControlServer::callbackCartesianTrajectoryService, this, std::placeholders::_1, std::placeholders::_2));

        this->compute_cartesian_trajectory_linear_ = this->create_service<franka_control_services::srv::TrajectoryCartesian>("compute_cartesian_trajectory_linear", 
        std::bind(&FrankaControlServer::callbackCartesianLinearTrajectoryService, this, std::placeholders::_1, std::placeholders::_2));
        
        this->compute_joint_space_trajectory_ = this->create_service<franka_control_services::srv::TrajectoryJointSpace>("compute_joint_space_trajectory", 
        std::bind(&FrankaControlServer::callbackJointTrajectoryService, this, std::placeholders::_1, std::placeholders::_2));

        this->insert_obstacle_ = this->create_service<franka_control_services::srv::ObstacleInsertion>("insert_obstacle",
        std::bind(&FrankaControlServer::callbackObstacleAddService, this, std::placeholders::_1, std::placeholders::_2));

        this->delete_obstacle_  = this->create_service<franka_control_services::srv::ObstacleDeletion>("delete_obstacle",
        std::bind(&FrankaControlServer::callbackObstacleDelService, this, std::placeholders::_1, std::placeholders::_2));
    }
     
private:
    // 
    // Members
    // 
    rclcpp::Service<franka_control_services::srv::TrajectoryCartesian>::SharedPtr compute_cartesian_trajectory_;
    rclcpp::Service<franka_control_services::srv::TrajectoryCartesian>::SharedPtr compute_cartesian_trajectory_linear_;
    rclcpp::Service<franka_control_services::srv::TrajectoryJointSpace>::SharedPtr compute_joint_space_trajectory_;
    std::vector<std::string> obstacles;
    rclcpp::Service<franka_control_services::srv::ObstacleInsertion>::SharedPtr insert_obstacle_;
    rclcpp::Service<franka_control_services::srv::ObstacleDeletion>::SharedPtr delete_obstacle_;
    // 
    // Methods
    // 
    /** 
     * \brief Obstacle Add service callback.
     * \param[in] request  ObstacjeInterface request
     * \param[in] response ObstacjeInterface response
     * \return none. 
    */
    void callbackObstacleDelService(const franka_control_services::srv::ObstacleDeletion::Request::SharedPtr request,
                                 const franka_control_services::srv::ObstacleDeletion::Response::SharedPtr response){
        
        RCLCPP_INFO(this->get_logger(),"*** Delete All Obstacles ***");
        
        auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial");
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(move_group_node);
        std::thread([&executor]() { 
            executor.spin(); 
        }).detach();

        auto move_group_interface = MoveGroupInterface(move_group_node, "panda_arm");

        // Add the collision object to the scene
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        planning_scene_interface.removeCollisionObjects(this->obstacles);
        this->obstacles.clear();
        response->success = true;
        executor.cancel(); 
    }
    /** 
     * \brief Obstacle Add service callback.
     * \param[in] request  ObstacjeInterface request
     * \param[in] response ObstacjeInterface response
     * \return none. 
    */
    void callbackObstacleAddService(const franka_control_services::srv::ObstacleInsertion::Request::SharedPtr request,
                                 const franka_control_services::srv::ObstacleInsertion::Response::SharedPtr response){
        RCLCPP_INFO(this->get_logger(),"*** Insert An Obstacle ***");
        auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial");
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(move_group_node);
        std::thread([&executor]() { 
            executor.spin(); 
        }).detach();

        auto move_group_interface = MoveGroupInterface(move_group_node, "panda_arm");
        // Create collision object for the robot to avoid
        std::string name = "box_" + std::to_string(this->obstacles.size());
        auto const collision_object = [name, request, frame_id = move_group_interface.getPlanningFrame()] {
            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.header.frame_id = frame_id;
            collision_object.id = name;
            shape_msgs::msg::SolidPrimitive primitive;

            // Define the size of the box in meters
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[primitive.BOX_X] = request->box_width;
            primitive.dimensions[primitive.BOX_Y] = request->box_height;
            primitive.dimensions[primitive.BOX_Z] = request->box_depth;

            // Define the pose of the box (relative to the frame_id)
            geometry_msgs::msg::Pose box_pose;
            box_pose.orientation.w = 1.0;
            box_pose.position.x = request->x;
            box_pose.position.y = request->y;
            box_pose.position.z = request->z;

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(box_pose);
            collision_object.operation = collision_object.ADD;
            
            return collision_object;
        }();
        // Add the collision object to the scene
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        planning_scene_interface.applyCollisionObject(collision_object);
        this->obstacles.emplace_back(collision_object.id);
        response->status = true;
        executor.cancel();  
    }
    /** 
     * \brief Cartesian linear trajectory service callback.
     * \param[in] request CartesianTrajectoryInterface request
     * \param[in] response CartesianTrajectoryInterface response
     * \return none. 
    */
    void callbackCartesianLinearTrajectoryService(const franka_control_services::srv::TrajectoryCartesian::Request::SharedPtr request,
                                                  const franka_control_services::srv::TrajectoryCartesian::Response::SharedPtr response){
        RCLCPP_INFO(this->get_logger(),"*** Compute Cartesian Trajectory Linear ***");
        
        auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial");
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(move_group_node);
        std::thread([&executor]() { 
            executor.spin(); 
        }).detach();

        auto move_group_interface = MoveGroupInterface(move_group_node, "panda_arm");

        auto const target_pose = [=]
        {
            geometry_msgs::msg::Pose msg;
            msg.orientation.w = request->qw;
            msg.orientation.x = request->qx;
            msg.orientation.y = request->qy;
            msg.orientation.z = request->qz;
            msg.position.x = request->x;
            msg.position.y = request->y;
            msg.position.z = request->z;
            return msg;
        }();
        move_group_interface.setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        // Compute Cartesian path
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);
        // Define the intermediate waypoints for the Cartesian path
        geometry_msgs::msg::Pose middle_pose = target_pose;
        middle_pose.position.x += 0.1;
        middle_pose.position.y -= 0.1;
        middle_pose.position.z += 0.1;
        waypoints.push_back(middle_pose);

        move_group_interface.setNumPlanningAttempts(5);

        // Set the end effector trajectory constraints (optional)
        moveit_msgs::msg::Constraints trajectory_constraints;
        trajectory_constraints.name = "move_constraints";
        moveit_msgs::msg::PositionConstraint position_constraint;
        position_constraint.header.frame_id = "base_link";
        position_constraint.link_name = "end_effector";
        position_constraint.target_point_offset.x = 0.1;
        position_constraint.target_point_offset.y = 0.1;
        position_constraint.target_point_offset.z = 0.1;
        position_constraint.constraint_region.primitive_poses.push_back(target_pose);
        trajectory_constraints.position_constraints.push_back(position_constraint);
        move_group_interface.setPathConstraints(trajectory_constraints);

        // Compute the Cartesian path
        double eef_step = 0.01;  // Step size in meters
        double jump_threshold = 0.0;  // No jumping allowed between waypoints
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        
        moveit::core::MoveItErrorCode mgi_status;
        if (fraction >= 0.9)  // if the Cartesian path is 90% achievable
        {
            plan.trajectory = trajectory;
            response->status = true;
            mgi_status = move_group_interface.execute(plan);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
            response->status = false;
            executor.cancel();
        }
                if(mgi_status == moveit::core::MoveItErrorCode::SUCCESS){
            executor.cancel();
        }
    }
    /** 
     * \brief Joint trajectory service callback.
     * \param[in] request JointTrajectoryInterface request
     * \param[in] response JointTrajectoryInterface response
     * \return none. 
    */
    void callbackJointTrajectoryService(const franka_control_services::srv::TrajectoryJointSpace::Request::SharedPtr request, 
                                        const franka_control_services::srv::TrajectoryJointSpace::Response::SharedPtr response){
        RCLCPP_INFO(this->get_logger(),"*** Compute Joint Space Trajectory ***");
        
        auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial");
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(move_group_node);
        std::thread([&executor]() { 
            executor.spin(); 
        }).detach();

        auto move_group_interface = MoveGroupInterface(move_group_node, "panda_arm");                                

        std::vector<double> target_joint_values = {request->j1, request->j2, request->j3, request->j4, request->j5, request->j6, request->j7};

        move_group_interface.setJointValueTarget(target_joint_values);

        move_group_interface.setNumPlanningAttempts(5);

        auto const [success, plan] = [&move_group_interface]
        {
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(move_group_interface.plan(msg));
            return std::make_pair(ok, msg);
        }();
        moveit::core::MoveItErrorCode mgi_status;
        if (success)
        {
            response->status = true;
            mgi_status = move_group_interface.execute(plan);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
            response->status = false;
            executor.cancel();
        }
        if(mgi_status == moveit::core::MoveItErrorCode::SUCCESS){
            executor.cancel();
        }
    }
    /** 
     * \brief Cartesian trajectory service callback.
     * \param[in] request CartesianTrajectoryInterface request
     * \param[in] response CartesianTrajectoryInterface response
     * \return none. 
    */
    void callbackCartesianTrajectoryService(const franka_control_services::srv::TrajectoryCartesian::Request::SharedPtr request,
                                            const franka_control_services::srv::TrajectoryCartesian::Response::SharedPtr response){
        RCLCPP_INFO(this->get_logger(),"*** Compute Cartesian Trajectory ***");
        
        auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial");
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(move_group_node);
        std::thread([&executor]() { 
            executor.spin(); 
        }).detach();

        auto move_group_interface = MoveGroupInterface(move_group_node, "panda_arm");
        
        auto const target_pose = [=]
        {
            geometry_msgs::msg::Pose msg;
            msg.orientation.w = request->qw;
            msg.orientation.x = request->qx;
            msg.orientation.y = request->qy;
            msg.orientation.z = request->qz;
            msg.position.x = request->x;
            msg.position.y = request->y;
            msg.position.z = request->z;
            return msg;
        }();
        move_group_interface.setPoseTarget(target_pose);
        move_group_interface.setNumPlanningAttempts(5);
        auto const [success, plan] = [&move_group_interface]
        {
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(move_group_interface.plan(msg));
            return std::make_pair(ok, msg);
        }();
        moveit::core::MoveItErrorCode mgi_status;
        if (success)
        {
            response->status = true;
            mgi_status = move_group_interface.execute(plan);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
            response->status = false;
            executor.cancel();
        }
        if(mgi_status == moveit::core::MoveItErrorCode::SUCCESS){
            executor.cancel();
        }
    }
};
     
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrankaControlServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}