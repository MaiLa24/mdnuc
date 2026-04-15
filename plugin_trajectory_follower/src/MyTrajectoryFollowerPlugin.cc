/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

 #include <gz/msgs/boolean.pb.h>

 #include <mutex>
 #include <string>
 #include <vector>

 #include <fstream>  // To read the file
 #include <sstream>  // To convert the text to data
 
 #include <gz/common/Profiler.hh>
 #include <gz/math/Angle.hh>
 #include <gz/math/Helpers.hh>
 #include <gz/math/Pose3.hh>
 #include <gz/math/Vector2.hh>
 #include <gz/math/Vector3.hh>
 #include <gz/plugin/Register.hh>
 #include <gz/transport/Node.hh>
 #include <gz/transport/TopicUtils.hh>
 #include <sdf/sdf.hh>
 
 #include "gz/sim/components/Pose.hh"
 #include "gz/sim/components/AngularVelocityCmd.hh"
 #include "gz/sim/Link.hh"
 #include "gz/sim/Model.hh"
 #include "gz/sim/Util.hh"
 
 #include "MyTrajectoryFollowerPlugin.hh"

 #include "rclcpp/rclcpp.hpp"
 #include "std_msgs/msg/bool.hpp"

 
 using namespace gz;
 using namespace sim;
 using namespace systems;
 
 class gz::sim::systems::MyTrajectoryFollowerPrivate
 {
   /// \brief Initialize the plugin.
   /// \param[in] _ecm Immutable reference to the EntityComponentManager.
   /// \param[in] _sdf The SDF Element associated with this system plugin.
   public: void Load(const EntityComponentManager &_ecm,
                     const sdf::ElementPtr &_sdf);
 
   /// \brief Callback to pause/resume the behavior.
   /// \param[in] _paused True when the intention is to pause the trajectory
   /// follower behavior or false to continue the trajectory.
   public: void OnPause(const msgs::Boolean &_paused);
 
   /// \brief Load waypoints from a text file.
   /// \param[in] _filename The name of the text file.
   /// \return True if the waypoints were loaded successfully, false otherwise.
   public: bool LoadWaypointsFromFile(const std::string &filename);
 
   /// \brief Load doors from a text file.
    /// \param[in] _filename The name of the text file.
    /// \return True if the doors were loaded successfully, false otherwise.
   public: bool LoadDoorsFromFile(const std::string &filename);   

   /// \brief A mutex to protect the paused member.
   public: std::mutex mutex;
 
   /// \brief Gazebo transport node.
   public: transport::Node node;
 
   /// \brief Topic name to pause/resume the trajectory.
   public: std::string topic;
 
   /// \brief The link entity
   public: gz::sim::Link link;
 
   /// \brief Model interface
   public: Model model{kNullEntity};
 
   /// \brief The initial pose of the model relative to the world frame.
   public: gz::math::Pose3<double> modelPose;
 
   /// \brief True if the model should continue looping though the waypoints.
   public: bool loopForever = false;
 
   /// \brief Linear force to apply to the model in its X direction.
   public: double forceToApply = 60;
 
   /// \brief Torque to apply to the model to align it with the next goal.
   public: double torqueToApply = 50;
 
   /// \brief When the model is at this distance or closer we won't try to move.
   /// Units are in meters.
   public: double rangeTolerance = 0.5;
 
   /// \brief When the model is at this angle or closer we won't try to rotate.
   /// Units are in degrees.
   public: double bearingTolerance = 2.0;
 
   /// \brief Use to sample waypoints around a circle.
   public: unsigned int numSamples = 8u;
 
   /// \brief The next position to reach.
   public: gz::math::Vector3d nextGoal;
 
   /// \brief Vector containing waypoints as 3D vectors of doubles representing
   /// X Y, where X and Y are local (Gazebo) coordinates.
   public: std::vector<gz::math::Vector2d> localWaypoints;

   /// \brief Vector containing doors as booleans representing if there is a door (true) or not (false)
   /// at the corresponding waypoint in localWaypoints.
   public: std::vector<bool> isDoor;
 
   /// \brief Initialization flag.
   public: bool initialized{false};
 
   /// \brief Copy of the sdf configuration used for this plugin
   public: sdf::ElementPtr sdfConfig;
 
   /// \brief Whether the trajectory follower behavior should be paused or not.
   public: bool paused = false;
 
   /// \brief Angular velocity set to zero
   public: bool zeroAngVelSet = false;
 
   /// \brief Force angular velocity to be zero when bearing is reached
   public: bool forceZeroAngVel = false;

   // To define the publisher to send trigger to node pointcloud_saver to save the pointcloud when the last waypoint is reached
    public: std::shared_ptr<rclcpp::Node> rosNode;
    public: rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr save_pointcloud_publisher;

    public: rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr waypoint_achieved_publisher;
    public: rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr door_crossed_publisher;

    // Number of waypoints
    public: int waypointNum = 1;
 };
 

 //////////////////////////////////////////////////
 void MyTrajectoryFollowerPrivate::Load(const EntityComponentManager &_ecm,
     const sdf::ElementPtr &_sdf)
 {
   // Parse required elements.
   if (!_sdf->HasElement("link_name"))
   {
     gzerr << "No <link_name> specified" << std::endl;
     return;
   }

    // Initialize the ROS2 node
    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }
    rosNode = rclcpp::Node::make_shared("trajectory_follower_plugin");

    // Create the publisher to send a std_msgs::msg::Bool message. This is necessary for the pointcloud_saver node.
    save_pointcloud_publisher = rosNode->create_publisher<std_msgs::msg::Bool>("/save_pointcloud", 10);
    // Create publishers to indicate when a waypoint is achieved and when a door is crossed
    waypoint_achieved_publisher = rosNode->create_publisher<std_msgs::msg::Bool>("/waypoint_achieved", 10);
    door_crossed_publisher = rosNode->create_publisher<std_msgs::msg::Bool>("/door_crossed", 10);
   
 
   std::string linkName = _sdf->Get<std::string>("link_name");
   this->link = Link(this->model.LinkByName(_ecm, linkName));
   if (!this->link.Valid(_ecm))
   {
     gzerr << "Could not find link named [" << linkName
            << "] in model" << std::endl;
     return;
   }
 
   this->modelPose = gz::sim::worldPose(this->link.Entity(), _ecm);
   this->modelPose.Pos().Z() = 0;
 
   if (_sdf->HasElement("waypoints_file"))
    {
        std::string waypointsFile = _sdf->Get<std::string>("waypoints_file");
        if (!LoadWaypointsFromFile(waypointsFile))
        {
            gzerr << "Error loading waypoints from file." << std::endl;
            return;
        }
    }

    if (_sdf->HasElement("doors_file"))
    {
        std::string doorsFile = _sdf->Get<std::string>("doors_file");
        if (!LoadDoorsFromFile(doorsFile))
        {
            gzerr << "Doors file missing, assuming no doors." << std::endl;
            this->isDoor.assign(this->localWaypoints.size(), false);
        }
    }
 
   // Parse the optional <loop> element.
   if (_sdf->HasElement("loop"))
     this->loopForever = _sdf->Get<bool>("loop");
 
   // Parse the optional <force> element.
   if (_sdf->HasElement("force"))
     this->forceToApply = _sdf->Get<double>("force");
 
   // Parse the optional <torque> element.
   if (_sdf->HasElement("torque"))
     this->torqueToApply = _sdf->Get<double>("torque");
 
   // Parse the optional <range_tolerance> element.
   if (_sdf->HasElement("range_tolerance"))
     this->rangeTolerance = _sdf->Get<double>("range_tolerance");
 
   // Parse the optional <bearing_tolerance> element.
   if (_sdf->HasElement("bearing_tolerance"))
     this->bearingTolerance = _sdf->Get<double>("bearing_tolerance");
 
   // Parse the optional <zero_vel_on_bearing_reached> element.
   if (_sdf->HasElement("zero_vel_on_bearing_reached"))
     this->forceZeroAngVel = _sdf->Get<bool>("zero_vel_on_bearing_reached");
 
   // Parse the optional <topic> element.
   this->topic = "/model/" + this->model.Name(_ecm) +
     "/trajectory_follower/pause";
 
   if (_sdf->HasElement("topic"))
     this->topic = _sdf->Get<std::string>("topic");
 
   this->topic = transport::TopicUtils::AsValidTopic(this->topic);
 
   this->node.Subscribe(topic, &MyTrajectoryFollowerPrivate::OnPause, this);
 
   gzmsg << "MyTrajectoryFollower["
       << this->model.Name(_ecm) << "] subscribed "
       << "to pause messages on topic[" << this->topic << "]\n";
 
   // If we have waypoints to visit, read the first one.
   if (!this->localWaypoints.empty())
   {
     this->nextGoal =
       {this->localWaypoints.front().X(), this->localWaypoints.front().Y(), 0};
   }
 }

/////////////////////////////////////////////////
bool MyTrajectoryFollowerPrivate::LoadWaypointsFromFile(const std::string &filename)
{
    // We open the text file with the waypoints
    std::ifstream file(filename);
    
    // Verify that the file has opened correctly
    if (!file.is_open())
    {
        gzerr << "Error when opening the waypoints file: " << filename << std::endl;
        return false;
    }

    // Read each line
    std::string line;
    while (std::getline(file, line)) 
    {
        std::stringstream ss(line);
        double x, y;

        // Read coordinates (assuming x y format)
        ss >> x >> y;

        // Save the waypoint
        gz::math::Vector2d waypoint(x, y);
        this->localWaypoints.push_back(waypoint);

        // Debug message
        gzdbg << "Waypoint leído: X = " << x << ", Y = " << y << std::endl;
    }

    file.close();

    // Check that we have read at least one waypoint
    if (this->localWaypoints.empty())
    {
        gzerr << "No waypoints were read from the file." << std::endl;
    }

    std::cout << "Number of waypoints: " << this->localWaypoints.size() << std::endl;

    return true;
 }

/////////////////////////////////////////////////
bool MyTrajectoryFollowerPrivate::LoadDoorsFromFile(const std::string &filename)
{
    // We open the text file with the door information
    std::ifstream file(filename);
    
    // Verify that the file has opened correctly
    if (!file.is_open())
    {
        gzerr << "Error when opening the doors file: " << filename << std::endl;
        return false;
    }

    // Read each line
    std::string line;
    while (std::getline(file, line)) 
    {
        std::stringstream ss(line);
        bool value;

        int temp;
        ss >> temp;
        value = (temp != 0);

        this->isDoor.push_back(value);
    }

    file.close();

    // Check that we have read at least one door
    if (this->isDoor.empty())
    {
        gzerr << "No doors were read from the file." << std::endl;
    }

    return true;
 }
 /////////////////////////////////////////////////
 void MyTrajectoryFollowerPrivate::OnPause(const msgs::Boolean &_paused)
 {
   std::lock_guard<std::mutex> lock(this->mutex);
   this->paused = _paused.data();
 }
 
 //////////////////////////////////////////////////
 MyTrajectoryFollower::MyTrajectoryFollower()
   : dataPtr(std::make_unique<MyTrajectoryFollowerPrivate>())
 {
 }
 
 //////////////////////////////////////////////////
 void MyTrajectoryFollower::Configure(const Entity &_entity,
     const std::shared_ptr<const sdf::Element> &_sdf,
     EntityComponentManager &/*_ecm*/,
     EventManager &/*_eventMgr*/)
 {
   this->dataPtr->model = Model(_entity);
   this->dataPtr->sdfConfig = _sdf->Clone();
 }
 
 //////////////////////////////////////////////////
 void MyTrajectoryFollower::PreUpdate(
     const gz::sim::UpdateInfo &_info,
     gz::sim::EntityComponentManager &_ecm)
 {
   GZ_PROFILE("MyTrajectoryFollower::PreUpdate");
 
   {
     std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
     if (_info.paused || this->dataPtr->paused)
       return;
   }
 
   if (!this->dataPtr->initialized)
   {
     // We call Load here instead of Configure because we can't be guaranteed
     // that all entities have been created when Configure is called
     this->dataPtr->Load(_ecm, this->dataPtr->sdfConfig);
     enableComponent<components::WorldPose>(_ecm, this->dataPtr->link.Entity());
     this->dataPtr->initialized = true;
   }
 
   // Nothing to do.
   if (this->dataPtr->localWaypoints.empty())
     return;
 
   this->dataPtr->modelPose = gz::sim::worldPose(
     this->dataPtr->link.Entity(), _ecm);
   this->dataPtr->modelPose.Pos().Z() = 0;
 
   // Direction vector to the goal from the model.
   gz::math::Vector3d direction =
     this->dataPtr->nextGoal - this->dataPtr->modelPose.Pos();
 
   // Direction vector in the local frame of the model.
   gz::math::Vector3d directionLocalFrame =
     this->dataPtr->modelPose.Rot().RotateVectorReverse(direction);
 
   double range = directionLocalFrame.Length();
   gz::math::Angle bearing(
     atan2(directionLocalFrame.Y(), directionLocalFrame.X()));
   bearing.Normalize();
 
   // Waypoint reached!
   if (range <= this->dataPtr->rangeTolerance)
   {
     
     std::cout << "Waypoint " << this->dataPtr->waypointNum << " reached!!" << std::endl;

     std_msgs::msg::Bool msg;
     msg.data = true;  // Send `true` to indicate that a waypoint has been reached.

     // Publish the message in /waypoint_achieved topic
     this->dataPtr->waypoint_achieved_publisher->publish(msg);


     if (this->dataPtr->isDoor.front())
     {
         std::cout << "Door crossed!" << std::endl;
         // Publish the message in /door_crossed topic
         this->dataPtr->door_crossed_publisher->publish(msg);
     }

     // We always keep the last waypoint in the vector to keep the model
     // "alive" in case it moves away from its goal.
     if (this->dataPtr->localWaypoints.size() == 1)
     {


       std::cout << "Number of waypoints: " << this->dataPtr->localWaypoints.size() << std::endl;
       std_msgs::msg::Bool msg;
       msg.data = true;  // Send `true` to indicate that the last waypoint has been reached.
   
       // Publish the message in /save_pointcloud topic
       this->dataPtr->save_pointcloud_publisher->publish(msg);
       this->dataPtr->localWaypoints.clear();
       return;
     }

     if (this->dataPtr->loopForever)
     {
       // Rotate to the left.
       std::rotate(this->dataPtr->localWaypoints.begin(),
                   this->dataPtr->localWaypoints.begin() + 1,
                   this->dataPtr->localWaypoints.end());
       std::rotate(this->dataPtr->isDoor.begin(),  
                   this->dataPtr->isDoor.begin() + 1,
                   this->dataPtr->isDoor.end());
     }
     else
     {
       // Remove the first waypoint.
       this->dataPtr->localWaypoints.erase(
         this->dataPtr->localWaypoints.begin());
       this->dataPtr->isDoor.erase(this->dataPtr->isDoor.begin());
       this->dataPtr->waypointNum += 1;
     }
 
     this->dataPtr->nextGoal = {
       this->dataPtr->localWaypoints.front().X(),
       this->dataPtr->localWaypoints.front().Y(), 0};
 
     return;
   }
 
   // Transform from world to local frame.
   auto comPose = this->dataPtr->link.WorldInertialPose(_ecm);
   if (!comPose.has_value())
   {
     gzerr << "Failed to get CoM pose for link ["
            << this->dataPtr->link.Entity() << "]" << std::endl;
     return;
   }
 
   // Transform the force and torque to the world frame.
   // Move commands. The vehicle always move forward (X direction).
   gz::math::Vector3d forceWorld;
   if (std::abs(bearing.Degree()) <= this->dataPtr->bearingTolerance)
   {
     forceWorld = (*comPose).Rot().RotateVector(
       gz::math::Vector3d(this->dataPtr->forceToApply, 0, 0));
 
     // force angular velocity to be zero when bearing is reached
     if (this->dataPtr->forceZeroAngVel && !this->dataPtr->zeroAngVelSet &&
         math::equal (std::abs(bearing.Degree()), 0.0,
         this->dataPtr->bearingTolerance * 0.5))
     {
       this->dataPtr->link.SetAngularVelocity(_ecm, math::Vector3d::Zero);
       this->dataPtr->zeroAngVelSet = true;
     }
   }
   gz::math::Vector3d torqueWorld;
   if (std::abs(bearing.Degree()) > this->dataPtr->bearingTolerance)
   {
     // remove angular velocity component otherwise the physics system will set
     // the zero ang vel command every iteration
     if (this->dataPtr->forceZeroAngVel && this->dataPtr->zeroAngVelSet)
     {
       auto angVelCmdComp = _ecm.Component<components::AngularVelocityCmd>(
           this->dataPtr->link.Entity());
       if (angVelCmdComp)
       {
         _ecm.RemoveComponent<components::AngularVelocityCmd>(
           this->dataPtr->link.Entity());
         this->dataPtr->zeroAngVelSet = false;
       }
     }
 
     int sign = std::abs(bearing.Degree()) / bearing.Degree();
     torqueWorld = (*comPose).Rot().RotateVector(
        gz::math::Vector3d(0, 0, sign * this->dataPtr->torqueToApply));
   }
 
   // Apply the force and torque at COM.
   this->dataPtr->link.AddWorldWrench(_ecm, forceWorld, torqueWorld);
 }
 
 GZ_ADD_PLUGIN(MyTrajectoryFollower,
                     gz::sim::System,
                     MyTrajectoryFollower::ISystemConfigure,
                     MyTrajectoryFollower::ISystemPreUpdate)
 
 GZ_ADD_PLUGIN_ALIAS(MyTrajectoryFollower,
                           "gz::sim::systems::MyTrajectoryFollower")
 
 // TODO(CH3): Deprecated, remove on version 8
 GZ_ADD_PLUGIN_ALIAS(MyTrajectoryFollower,
                           "ignition::gazebo::systems::MyTrajectoryFollower")
