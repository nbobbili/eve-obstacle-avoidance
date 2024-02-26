#include "rclcpp/rclcpp.hpp"

// Include necessary message types from px4_msgs
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/distance_sensor.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using std::placeholders::_1;

// Define any required enums or structs to represent flight states or other constants
enum class FlightState {

};

class DroneControl : public rclcpp::Node {
public:
    void onInit(void);
    DroneControl() : Node("drone_control") {
        initializeWaypoints();
        onInit();
    }

    // Define methods for drone control, such as taking off, landing, waypoint navigation, etc.

private:
    std::vector<std::array<float, 3>> square_waypoints_;
    void initializeWaypoints() {
        // Initialize waypoints forming a square trajectory at the specified takeoff height
        square_waypoints_ = {
            {0.0, 0.0, -15.0}, // Takeoff position (assuming NED coordinates)
            {50.0, 0.0, -15.0}, // First waypoint
            {50.0, 50.0, -15.0}, // Second waypoint
            {0.0, 50.0, -15.0} // Third waypoint
        };
    }

    // Define member variables for publishers, subscribers, and other state variables

    //publishers
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectorySetpointPub;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicleCommandPub;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboardControlModePub;
    
    //subscribers
    rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::SharedPtr lidarSub;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr statusSub;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odomSub;

    //timer
    rclcpp::TimerBase::SharedPtr controlModeTimer;
    rclcpp::TimerBase::SharedPtr waypointTimer;

    //methods
    void lidarCallback(const px4_msgs::msg::DistanceSensor::SharedPtr msg);
    void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void vehicleCommandPublisher(uint16_t command, float param1 = 0.0, float param2 = 0.0,float param3 = 0.0);
    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void waypointPublisher();
    void offboardControlModePublisher();

    float obstacleDistance;
    double distToWaypoint;
    double desYaw;
    
    std::array<float, 3> currentPosition;
    
    int armCounter = 0;
    int avoidCounter = 0;
    int waypointNumber = 0;
    int lapNumber = 0;
    int totalLaps = 2;
    
    bool offboardModeActive = false;
    bool takeoffDone = false;
    bool vehicleNotMoving = false;
    bool avoidingObstacle = false;
    bool waypointChanged = false;
};

void DroneControl::lidarCallback(const px4_msgs::msg::DistanceSensor::SharedPtr msg) {

    if (msg->signal_quality > 10) {
        obstacleDistance = msg->current_distance;
        if (obstacleDistance <=5 ){
            avoidingObstacle = true;
        }
    }
    else {
        obstacleDistance = msg->max_distance;
    }
}

void DroneControl::odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {

    //store current position
    currentPosition = {msg->position[0], msg->position[1], msg->position[2]};
    
    //Find the distance to WP
    switch (waypointNumber) {
    case 0:
        distToWaypoint = sqrt(pow((square_waypoints_[0][0] - currentPosition[0]),2) +
                              pow((square_waypoints_[0][1] - currentPosition[1]),2) +
                              pow((square_waypoints_[0][2] - currentPosition[2]),2));                              
        break;
    case 1:
        distToWaypoint = sqrt(pow((square_waypoints_[1][0] - currentPosition[0]),2) +
                              pow((square_waypoints_[1][1] - currentPosition[1]),2) +
                              pow((square_waypoints_[1][2] - currentPosition[2]),2));
        break;
    case 2:
        distToWaypoint = sqrt(pow((square_waypoints_[2][0] - currentPosition[0]),2) +
                              pow((square_waypoints_[2][1] - currentPosition[1]),2) +
                              pow((square_waypoints_[2][2] - currentPosition[2]),2));
        break;
    case 3:
        distToWaypoint = sqrt(pow((square_waypoints_[3][0] - currentPosition[0]),2) +
                              pow((square_waypoints_[3][1] - currentPosition[1]),2) +
                              pow((square_waypoints_[3][2] - currentPosition[2]),2));
        break;                                                     
    default:
        RCLCPP_ERROR(this->get_logger(), "Waypoint number out of bounds!");
        break;
    }

    //Hack: Make yaw zero for takeoff, only works for altitude > 15m
    if (currentPosition[2] > -14) {   
        desYaw = M_PI / 2;
    }
    else {
    //Get heading for the next waypoint
    desYaw = atan2((square_waypoints_[waypointNumber][1] - currentPosition[1]),
                   (square_waypoints_[waypointNumber][0] - currentPosition[0]));
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(), 5000,
                         "Distance to waypoint: %f", distToWaypoint);
    
    // Change waypoint if position is within 1 meter of current position
    if (distToWaypoint < 1) {
        waypointNumber++;
        RCLCPP_INFO(this->get_logger(), "Going to  waypoint number: %d", waypointNumber);
    }
    // Reset counter when we reach last waypoint
    if (waypointNumber == 4) {
        waypointNumber = 0;
        avoidCounter = 0;
        lapNumber++;
    }

    //Check if moving
    vehicleNotMoving = std::abs(msg->velocity[0]) < 0.5 &&
                       std::abs(msg->velocity[1]) < 0.5 &&
                       std::abs(msg->velocity[2]) < 0.5;
}

void DroneControl::vehicleCommandPublisher(uint16_t command, float param1, float param2, float param3) {
    
    px4_msgs::msg::VehicleCommand modeMsg{};
    modeMsg.param1 = param1;
    modeMsg.param2 = param2;
    modeMsg.param7 = param3;
    modeMsg.command = command;
    modeMsg.target_system = 1;
    modeMsg.target_component = 1;
    modeMsg.source_component = 1;
    modeMsg.source_system = 1;
    modeMsg.from_external = true;
    modeMsg.timestamp = this->get_clock()->now().nanoseconds()/ 1000;
    vehicleCommandPub->publish(modeMsg);
}

void DroneControl::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {

    if (msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
        RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(), 10000, "Flight Mode: OFFBOARD");
    }
}
void DroneControl::offboardControlModePublisher() {
    
    px4_msgs::msg::OffboardControlMode msg;
    msg.position = true;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.direct_actuator = false;
    msg.thrust_and_torque = false;
    msg.velocity = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboardControlModePub->publish(msg);

    //OFFBOARD and ARM
    if (armCounter == 10) {
        //Set to Offboard
        this->vehicleCommandPublisher(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        // ARM
        this->vehicleCommandPublisher(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        RCLCPP_WARN(this->get_logger(), "Arming...");
    }

    if (armCounter < 11){
        armCounter++;
    }
}

void DroneControl::waypointPublisher() {

    px4_msgs::msg::TrajectorySetpoint msg;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    
    // General case
    if(obstacleDistance >= 5 && !avoidingObstacle) {
        if (lapNumber >= totalLaps) { // limits the number of laps
            msg.position = square_waypoints_[0];
            msg.yaw = desYaw;
            trajectorySetpointPub->publish(msg);
            return;
        }
        msg.position = square_waypoints_[waypointNumber];
        msg.yaw = desYaw;
        trajectorySetpointPub->publish(msg);
    } // Climb if we see obstacle
    else if (obstacleDistance <=5) {   
        avoidingObstacle = true;
        msg.position = {currentPosition[0], currentPosition [1], (currentPosition[2]-4)};
        msg.yaw = desYaw;
        RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(), 5000, "Obstacle ahead, climbing!");
        trajectorySetpointPub->publish(msg);
    } // Go straight for a seconds to clear the obstacle under us after we stop seeing it
    else if (obstacleDistance >=5 && avoidingObstacle) {
        msg.position = {(currentPosition[0]+5), currentPosition[1], currentPosition[2]};
        msg.yaw = desYaw;
        RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(), 5000, "Path clear. Going straight ahead.");
        trajectorySetpointPub->publish(msg);
        avoidCounter++;
        if (avoidCounter==10){
            avoidingObstacle = false;
        }
    }
}

void DroneControl::onInit(){

    RCLCPP_INFO(this->get_logger(), "Running offboard node");

    // // Get paramters
    // this->declare_parameter("lap_numbers", 2);
    // this->get_parameter("lap_numbers", totalLaps);
    
    // Initialize publishers and subscribers with appropriate topics and QoS settings
    //Publishers
    trajectorySetpointPub = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("fmu/in/trajectory_setpoint", 10);
    vehicleCommandPub = this->create_publisher<px4_msgs::msg::VehicleCommand>("fmu/in/vehicle_command", 10);
    offboardControlModePub = this->create_publisher<px4_msgs::msg::OffboardControlMode>("fmu/in/offboard_control_mode", 10);
    
    //QoS
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history,5), qos_profile);
    
    //Subscribers
    lidarSub = this->create_subscription<px4_msgs::msg::DistanceSensor>("fmu/out/distance_sensor",qos, 
                std::bind(&DroneControl::lidarCallback, this,_1));
    statusSub = this->create_subscription<px4_msgs::msg::VehicleStatus>("fmu/out/vehicle_status",qos, 
                std::bind(&DroneControl::vehicleStatusCallback, this,_1));
    odomSub = this->create_subscription<px4_msgs::msg::VehicleOdometry>("fmu/out/vehicle_odometry",qos, 
                std::bind(&DroneControl::odomCallback, this,_1));
    
    //OffboardControlMsg Spinner
    controlModeTimer = this->create_wall_timer(100ms,std::bind(&DroneControl::offboardControlModePublisher,this));
    waypointTimer = this->create_wall_timer(200ms,std::bind(&DroneControl::waypointPublisher,this));
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
