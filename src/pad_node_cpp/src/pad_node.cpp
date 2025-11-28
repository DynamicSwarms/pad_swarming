#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <vector>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <map> 
#include <future>
#include "pad_node_cpp/msg/node_broadcast.hpp"
#include "pad_node_cpp/msg/all_infos.hpp"
#include "pad_node_cpp/msg/tf_graph.hpp"
#include "pad_management_interfaces/msg/landing_interest.hpp"

#include <chrono>
#include "pad_management_interfaces/srv/landing_pad_information.hpp"
#include <string>
#include "pad_node_cpp/srv/lock_cluster.hpp"
#include "pad_management_interfaces/msg/airspace_lock.hpp"
#include "pad_management_interfaces/srv/pad_idle_target.hpp"
#include "pad_management_interfaces/srv/pad_right_release.hpp"
#include "pad_management_interfaces/srv/pad_right_acquire.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"  
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <iostream>
#include <vector>
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/string.hpp"

using pad_management_interfaces::srv::PadIdleTarget;
using pad_management_interfaces::srv::PadRightRelease;
using pad_management_interfaces::srv::PadRightAcquire;
using LockCluster = pad_node_cpp::srv::LockCluster;
using std::placeholders::_1;
using std::placeholders::_2;
#define PAD_IS_FREE 1
#define PAD_IS_OCCUPIED 2
#define PAD_HAS_REQEUEST 3
#define PAD_IS_LOCKED_BY_CLUSTER 4
#define PAD_HAS_DRONE 5
#define PAD_IS_RESERVED 6
#define total_reichweite 4
#define cluster_distance 4

struct NodeData {
    int id;
    double x;
    double y;
    double z;
    std::vector<int> known_nodes;
    std::chrono::system_clock::time_point timestamp;
};



class pad_node : public rclcpp::Node
{
public:
   pad_node(int id, float X, float Y, float Z) : 
    Node("pad_" + std::to_string(id)), 
    id_(id),
    status(PAD_IS_FREE),
    status_bevor_locking(PAD_IS_FREE),
    tf_buffer(get_clock()),
    tf_listener(tf_buffer)
{
    reqeuested=false;
    name = "pad_" + std::to_string(id_);
    X_=X;
    Y_=Y;
    Z_=Z;

    //===Callbackgroup===
    callback_group_airspace_lock_timer=this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_callback_group_landing_pad_information = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_callback_group_lock_cluster = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_callback_group_lock_cluster = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_callback_group_pad_right_acquire= this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_callback_group_pad_idle_target= this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_callback_group_pad_right_release= this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    publish_data_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_nodebroadcast = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_landing_interest = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    group_timer_publish= this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    group_timer_update= this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    //===Services===
    std::string service_name = "/" + name + "/shutdown";
    shutdown_srv_ = this->create_service<std_srvs::srv::Empty>(
        service_name,
        [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
               std::shared_ptr<std_srvs::srv::Empty::Response>) {
                delete_pad_lock_marker();
          RCLCPP_INFO(this->get_logger(), "Shutdown Service called, shutting down node...");
          rclcpp::shutdown();
        });
    service_name = "/" + name + "/new_drone";
    new_drone_service = this->create_service<PadRightRelease>(
        service_name,
        std::bind(&pad_node::handle_new_drone_service, this, _1, _2),
        rmw_qos_profile_services_default,
        service_callback_group_pad_right_acquire
    );
    service_name = "/" + name + "/pad_right_acquire";
    pad_right_aquire_service = this->create_service<PadRightAcquire>(
        service_name,
        std::bind(&pad_node::handle_aquire_landing_rigth_service, this, _1, _2),
        rmw_qos_profile_services_default,
        service_callback_group_pad_right_acquire
    );
    service_name = "/" + name + "/pad_idle_target";
    pad_idle_target_service = this->create_service<PadIdleTarget>(
        service_name,
        std::bind(&pad_node::handle_pad_idle_target_service, this, _1, _2),
        rmw_qos_profile_services_default,
        service_callback_group_pad_idle_target
    );
    lock_cluster_service = this->create_service<LockCluster>(
        "lock_cluster_Pad_" + std::to_string(id),
        std::bind(&pad_node::handle_lock_cluster_service,this, std::placeholders::_1,std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_callback_group_lock_cluster);
    ask_free_Pad_service = this->create_service<LockCluster>(
        "ask_Pad_free" + std::to_string(id),
        std::bind(&pad_node::handle_ask_free_Pad_service,this, std::placeholders::_1,std::placeholders::_2),
        rmw_qos_profile_services_default,
        service_callback_group_lock_cluster);
    service_name = "/" + name + "/pad_right_release";
    pad_right_release_service = this->create_service<PadRightRelease>(
        service_name,
        std::bind(&pad_node::handle_landing_complete_service, this, _1, _2),
        rmw_qos_profile_services_default,
        service_callback_group_pad_right_acquire 
    );

    // === PUBLISHER ===
    rclcpp::QoS qos_profile(10);
    qos_profile.transient_local();  // hält die letzte Nachricht
    pad_lock_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>(
        "pad_lock_marker", qos_profile);
    marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    publisher_broadcast = this->create_publisher<pad_node_cpp::msg::NodeBroadcast>("id_topic", 10);
    publisher_airspace_lock =  this->create_publisher<pad_management_interfaces::msg::AirspaceLock>("/airspace_lock", 10);

 
    // === SUBSCRIPTIONS ===
    rclcpp::SubscriptionOptions options_nodebroadcast;
    options_nodebroadcast.callback_group = callback_group_nodebroadcast;
    subscription_nodebroadcast_ = this->create_subscription<pad_node_cpp::msg::NodeBroadcast>(
        "id_topic",
        rclcpp::QoS(10),
        std::bind(&pad_node::receive_id, this, std::placeholders::_1),
        options_nodebroadcast);
    rclcpp::SubscriptionOptions options_landing_interest;
    options_landing_interest.callback_group = callback_group_landing_interest;
    need_pad_sub = this->create_subscription<pad_management_interfaces::msg::LandingInterest>(
        "/need_pad_topic",
        rclcpp::QoS(10),
        std::bind(&pad_node::handle_need_pad_request, this, std::placeholders::_1),
        options_landing_interest);
    subscription_landing_interest = this->create_subscription<pad_management_interfaces::msg::LandingInterest>(
        "/landing_interest_topic",
        rclcpp::QoS(10),
        std::bind(&pad_node::landing_request, this, std::placeholders::_1),
        options_landing_interest);

   
    // === TIMERS ===
    airspace_lock_timer = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&pad_node::publish_airspace_lock, this),
        callback_group_airspace_lock_timer
    );
    airspace_lock_timer->cancel();

    group_timer_publish = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&pad_node::publish_id, this),
        group_timer_publish
    );

    group_timer_update = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    update_timer_ = this->create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&pad_node::update_received_nodes, this),
        group_timer_update
    );

    group_timer_TF = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    
    update_coordinates_timer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&pad_node::getCoordinates, this),
        group_timer_TF
    );

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}




private:
    int id_;
    float X_;
    float Y_;
    float Z_;
    double qx_, qy_, qz_, qw_;
    std::string reserved_drone_name;
    std::string name;
    bool reqeuested;
    int status;
    int status_bevor_locking;
    std::string my_drone_name;
    std::vector<NodeData> non_cluster_nodes_;
    std::vector<NodeData> cluster_nodes_;
    std::set<std::string> active_locks;
    
    //====MUTEX====
    std::mutex status_mutex_;
    std::mutex pad_mutex;
    //====Callback Groups=====#
    rclcpp::CallbackGroup::SharedPtr group_timer_TF;
    rclcpp::CallbackGroup::SharedPtr group_timer_update;
    rclcpp::CallbackGroup::SharedPtr group_timer_publish;
    rclcpp::CallbackGroup::SharedPtr client_callback_group_landing_pad_information;
    rclcpp::CallbackGroup::SharedPtr client_callback_group_lock_cluster;
    rclcpp::CallbackGroup::SharedPtr service_callback_group_lock_cluster;
    rclcpp::CallbackGroup::SharedPtr service_callback_group_pad_right_acquire;
    rclcpp::CallbackGroup::SharedPtr service_callback_group_pad_idle_target;
    rclcpp::CallbackGroup::SharedPtr service_callback_group_pad_right_release;
    rclcpp::CallbackGroup::SharedPtr publish_data_callback_group;
    rclcpp::CallbackGroup::SharedPtr callback_group_nodebroadcast;
    rclcpp::CallbackGroup::SharedPtr callback_group_landing_interest;
    rclcpp::CallbackGroup::SharedPtr callback_group_airspace_lock_timer;
    //====TF====
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    //====Publisher=====
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pad_lock_marker_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    rclcpp::Publisher<pad_node_cpp::msg::NodeBroadcast>::SharedPtr publisher_broadcast;
    rclcpp::Publisher<pad_node_cpp::msg::AllInfos>::SharedPtr publisher_allinfos;
    rclcpp::Publisher<pad_management_interfaces::msg::AirspaceLock>::SharedPtr publisher_airspace_lock;
    //====Subsciption=====
    rclcpp::Subscription<pad_management_interfaces::msg::LandingInterest>::SharedPtr need_pad_sub;
    rclcpp::Subscription<pad_node_cpp::msg::NodeBroadcast>::SharedPtr subscription_nodebroadcast_;
    rclcpp::Subscription<pad_node_cpp::msg::TFGraph>::SharedPtr subscription_tfgraph_;
    rclcpp::Subscription<pad_management_interfaces::msg::LandingInterest>::SharedPtr subscription_landing_interest;
    //====Service=====
    rclcpp::Service<LockCluster>::SharedPtr ask_free_Pad_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr shutdown_srv_;
    rclcpp::Service<LockCluster>::SharedPtr lock_cluster_service ;
    rclcpp::Service<PadRightRelease>::SharedPtr pad_right_release_service;
    rclcpp::Service<PadIdleTarget>::SharedPtr pad_idle_target_service;
    rclcpp::Service<PadIdleTarget>::SharedPtr pad_idle_target;
    rclcpp::Service<PadRightAcquire>::SharedPtr pad_right_aquire_service;
    rclcpp::Service<PadRightRelease>::SharedPtr new_drone_service;
    //====Timer====
    rclcpp::TimerBase::SharedPtr update_timer_; 
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr draw_timer;
    rclcpp::TimerBase::SharedPtr tf_broadcast_timer_;
    rclcpp::TimerBase::SharedPtr airspace_lock_timer;
    rclcpp::TimerBase::SharedPtr update_coordinates_timer;
    rclcpp::TimerBase::SharedPtr request_timeout_timer_;
    //====Client====r
    rclcpp::Client<pad_management_interfaces::srv::LandingPadInformation>::SharedPtr landing_client_;
    rclcpp::Client<LockCluster>::SharedPtr lock_cluster_client;
    rclcpp::Client<pad_management_interfaces::srv::LandingPadInformation>::SharedPtr need_pad_client_;
    rclcpp::Client<LockCluster>::SharedPtr ask_free_client;


    void setStatus(int s){
        //set status with mutex and publish new status
        {std::lock_guard<std::mutex> lock(status_mutex_);
        status = s;
        }
        publish_pad_lock(s);
         if(s!=PAD_IS_OCCUPIED&&s!=PAD_HAS_DRONE){
            cleanup_markers_airspace_locks();
        }
    }

    int getStatus(){
        //get status with mutex
        {
        std::lock_guard<std::mutex> guard(status_mutex_);
        return status;
        }
    }

    void getCoordinates()
    {   
        //get Coordinates from TF
        if (!tf_buffer.canTransform("world", name,   tf2::TimePointZero)) {
                RCLCPP_INFO(this->get_logger(),"NO TRANSFORM!!!!!!!");
                return;
            }
        try {
            const geometry_msgs::msg::TransformStamped & transform = tf_buffer.lookupTransform(
               "world", name,    tf2::TimePointZero);
          
            X_ = transform.transform.translation.x;
            Y_ = transform.transform.translation.y;
            Z_ = transform.transform.translation.z;
            qx_ = transform.transform.rotation.x;
            qy_ = transform.transform.rotation.y;
            qz_ = transform.transform.rotation.z;
            qw_ = transform.transform.rotation.w;
            publish_pad_lock(getStatus());
        }
        catch (const tf2::TransformException & e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", e.what());
        }
    }

  


    void publish_pad_lock(int state){    
        //publish status and Position by using a marker            
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "pad_locks";
        marker.id = id_;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = X_;
        marker.pose.position.y = Y_;
        marker.pose.position.z = Z_;
        marker.pose.orientation.x = qx_;
        marker.pose.orientation.y = qy_;
        marker.pose.orientation.z = qz_;
        marker.pose.orientation.w = qw_;
        if (state == PAD_IS_RESERVED){
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0; 
        }else if(state==PAD_IS_FREE){
           marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;  
        }else if(state==PAD_HAS_REQEUEST){
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0; 
        }else if (state==PAD_IS_LOCKED_BY_CLUSTER){
            marker.color.r = 0.5;
            marker.color.g = 0.0;
            marker.color.b = 1.0; 
        }else if(state == PAD_IS_OCCUPIED || getStatus() == PAD_HAS_DRONE){
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0; 
        }else{
            RCLCPP_ERROR(this->get_logger(),"ERROR - Status: %d",state);
        }
        marker.color.a = 0.5;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.2;
        pad_lock_marker_pub->publish(marker);
    }

    void delete_pad_lock_marker() {
        //delet own padlock marker
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "pad_locks";  
        marker.id = id_;          
        marker.action = visualization_msgs::msg::Marker::DELETE;
        pad_lock_marker_pub->publish(marker);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }


    void publish_airspace_lock(){
        //publish own airspacelock 
        auto message = pad_management_interfaces::msg::AirspaceLock();
        message.name = name;
        message.radius = cluster_distance;
        send_airspace_lock_marker();
        publisher_airspace_lock->publish(message);
    }

    void send_airspace_lock_marker(){
        //publish own airspacelock as a marker in tf
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "airspace_locks";
        marker.id = id_;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = X_;
        marker.pose.position.y = Y_;
        marker.pose.position.z = Z_+1;
        marker.pose.orientation.w = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.2;
        marker.scale.x = cluster_distance;
        marker.scale.y = cluster_distance;
        marker.scale.z = 2.0;
        marker_pub->publish(marker);
    }


      void cleanup_markers_airspace_locks()
    {
        //delet the aispacelock marker from tf
        airspace_lock_timer->cancel();
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.header.frame_id = "world";
        delete_marker.header.stamp = this->get_clock()->now();
        delete_marker.ns = "airspace_locks";   
        delete_marker.id = id_;
        delete_marker.action = visualization_msgs::msg::Marker::DELETE;
        marker_pub->publish(delete_marker);
    }


    void handle_landing_complete_service(
        const std::shared_ptr<PadRightRelease::Request> request,
        std::shared_ptr<PadRightRelease::Response> response){

        //the padright get return, set either occupied or free and unlock cluster
        if(request->takeoff){
            setStatus(PAD_IS_FREE);
            reqeuested=false;
            my_drone_name = "";
        }else{
            setStatus(PAD_HAS_DRONE);
            my_drone_name = request->name;
        }
        

        lock_cluster_sync(false);
        response->success = true;
        cleanup_markers_airspace_locks();
        //if simpad (all ids smaler than 11), shut down
        if(id_ < 10){
            delete_pad_lock_marker();
            RCLCPP_INFO(this->get_logger(), "Shutdown SimPad, shutting down node...");
            rclcpp::shutdown();
        }
    }

    void handle_pad_idle_target_service(
        const std::shared_ptr<PadIdleTarget::Request> request,
        std::shared_ptr<PadIdleTarget::Response> response)
    {
        //get the drone position and send a idle target position
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "world";
        target_pose.pose.position.x = X_;
        target_pose.pose.position.y = Y_;
        target_pose.pose.position.z = Z_+ 1;
        target_pose.pose.orientation.w = 1.0;  

        //todo hier checken ob was im weg ist!!!!

        response->target = target_pose;
        return;

    }

    void handle_new_drone_service(
        const std::shared_ptr<PadRightRelease::Request> request,
        std::shared_ptr<PadRightRelease::Response> response)
    {
        //if informed about occupieing new drone
        setStatus(PAD_HAS_DRONE);
        my_drone_name=request->name;
        response->success = true;
    }

    void handle_aquire_landing_rigth_service(
        const std::shared_ptr<PadRightAcquire::Request> request,
        std::shared_ptr<PadRightAcquire::Response> response)
    {
        if(getStatus()==PAD_IS_OCCUPIED ){
            if(my_drone_name!=request->name){
                response->success = false;
                RCLCPP_INFO(this->get_logger(),"denied aquire IS Occupied");
                return;
            } 
        }

        if(getStatus()==PAD_HAS_DRONE ){
            if(my_drone_name!=request->name){
                response->success = false;
                RCLCPP_INFO(this->get_logger(),"denied aquire HAS Drone");
                return;
            } 
        }

        bool locked = lock_cluster_sync(true);
        if (!locked) {
            RCLCPP_WARN(this->get_logger(), "Locking Error %s",name.c_str());
            lock_cluster_sync(false);
            response->success = false;
            return;
        }
            setStatus(PAD_IS_OCCUPIED);
            airspace_lock_timer->reset();
            my_drone_name = request->name;
            //todo: timer starten, der false antwortet fals abgelaufen und wenn idle target == meine position, timer stoppen und landung erlauben
            RCLCPP_INFO(this->get_logger(),"Aquire success");
            response->success = true;        
    }

    void handle_ask_free_Pad_service(
        const std::shared_ptr<LockCluster::Request> request,
        std::shared_ptr<LockCluster::Response> response)
    {   
        //get ask if occupied, answer id free, reserve for drone for some time
        RCLCPP_INFO(this->get_logger(),"aks Free erhalten");
        if(getStatus()==PAD_IS_FREE){
            setStatus(PAD_IS_RESERVED);
            reserved_drone_name = request->name;
            //todo: starte timer ob wirklich gelandet wird, ansonsten wieder auf free und reserved auf leer
            response->success=true;
            RCLCPP_INFO(this->get_logger(),"antwort: bin frei");
            return;
        }
        RCLCPP_INFO(this->get_logger(),"antwort: bin besetzt");
        response->success=false;
    }


    
    void handle_lock_cluster_service(
        const std::shared_ptr<LockCluster::Request> request,
        std::shared_ptr<LockCluster::Response> response)
    {   
        //getting unlocked or locked
        const std::string& requester = request->name;
        if(request->locking){
            RCLCPP_INFO(this->get_logger(), "Lock empfangen: %s ",name.c_str());
            if(active_locks.empty()){
                status_bevor_locking=getStatus();
               
            }
            setStatus(PAD_IS_LOCKED_BY_CLUSTER);
            active_locks.insert(requester);
            response->success = true;
            RCLCPP_INFO(this->get_logger(), "Lock antwort: %s ",name.c_str());
            return;
            
        }else{
           RCLCPP_INFO(this->get_logger(), "Freigabe empfangen von: %s", requester.c_str());

            auto it = active_locks.find(requester);
            if (it != active_locks.end()) {
                active_locks.erase(it);
            } else {
                RCLCPP_WARN(this->get_logger(), "Unlock-Anfrage von %s, aber kein Lock vorhanden.", requester.c_str());
            }

            if (active_locks.empty()) {
                // from all unlocked, return to old status
                if (getStatus() == PAD_HAS_REQEUEST) {
                    setStatus(PAD_IS_FREE);
                } else {
                    setStatus(status_bevor_locking);
                }
          
            } else {
                setStatus(PAD_IS_LOCKED_BY_CLUSTER);  
            }
            response->success = true;
            return;
        }
    }




    void handle_need_pad_request(const pad_management_interfaces::msg::LandingInterest::SharedPtr msg){
        //a drone wants to get a pad assigne, if in the same area start searching for free pad
        float received_x = msg->x;
        float received_y = msg->y;
        float received_z = msg->z;
        //return if not in area
        if (distance(received_x, received_y, received_z) > total_reichweite){
            return;
        }
        RCLCPP_INFO(this->get_logger(),"Received need Pad: %s",msg->name.c_str());
        //todo: checken, das nur einer abfrägt!!!
        auto cluster_copy = non_cluster_nodes_;
        for (const auto& node : cluster_copy) {
            RCLCPP_INFO(this->get_logger(),"Asking: %d",node.id);
            if (ask_if_pad_free(node.id,msg->name)) {
                std::string service_name = "need_pad_information_" + msg->name;
                need_pad_client_ = this->create_client<pad_management_interfaces::srv::LandingPadInformation>(
                    service_name,
                    rmw_qos_profile_services_default,
                    client_callback_group_landing_pad_information);
                if (!need_pad_client_->wait_for_service(std::chrono::seconds(10))) {
                    RCLCPP_WARN(this->get_logger(), "Service not available need_pad_information");
                    return;
                }
                RCLCPP_INFO(this->get_logger(),"Drohne freies Pad mitgeteilt");
                auto request = std::make_shared<pad_management_interfaces::srv::LandingPadInformation::Request>();
                request->name = "pad_" + std::to_string(node.id);
                request->x = node.x;
                request->y = node.y;
                request->z = node.z;
                auto future =  need_pad_client_->async_send_request(request);
                return;
            }
        }
        
    }

    bool ask_if_pad_free(int node_id, std::string asking_padflie_name){
        //ask a certain pad if it is free 
        std::string service_name = "ask_Pad_free" + std::to_string(node_id);
        RCLCPP_INFO(this->get_logger(),"suche freies Pad");
        ask_free_client = this->create_client<LockCluster>(
        service_name,
        rmw_qos_profile_services_default,
        client_callback_group_lock_cluster);

        
        if (! ask_free_client->wait_for_service(std::chrono::seconds(3))) { 
            RCLCPP_WARN(this->get_logger(), "Service ask_free_client nicht verfügbar");
            return false;
        } 

        auto request = std::make_shared<LockCluster::Request>();
        request->name = asking_padflie_name;
        
        auto future =  ask_free_client->async_send_request(request);
        auto state = future.wait_for(std::chrono::milliseconds(300));
        
        if (state== std::future_status::ready) {
            auto response = future.get();
            if (response->success) { 
                return true; 
                RCLCPP_INFO(this->get_logger(),"freies Pad gefunden");
            } else {
                return false; 
            } 
            
        }
        return false;
    }

    void landing_request(const pad_management_interfaces::msg::LandingInterest::SharedPtr msg) {
        // a drone wants to land, check if in area and if free answer with your informations
        RCLCPP_INFO(this->get_logger(),"Landing Interest: %s",msg->name.c_str());
        std::lock_guard<std::mutex> lock(pad_mutex);
        {
        //do not answer if not free or has already a request running
        if (getStatus() != PAD_IS_FREE || reqeuested) {
            return;
        }
        
        int status_old=getStatus();
        float received_x = msg->x;
        float received_y = msg->y;
        float received_z = msg->z;

        //only send infromartion if in same area as drone
        if (distance(received_x, received_y, received_z) < total_reichweite) {
            reqeuested=true;
            my_drone_name = msg->name; 
                
            std::string service_name = "landing_pad_information_" + msg->name;

            landing_client_ = this->create_client<pad_management_interfaces::srv::LandingPadInformation>(
                service_name,
                rmw_qos_profile_services_default,
                client_callback_group_landing_pad_information);

            if (!landing_client_->wait_for_service(std::chrono::seconds(10))) {
                RCLCPP_WARN(this->get_logger(), "Service not available landing_pad_information");
                return;
            }

            auto request = std::make_shared<pad_management_interfaces::srv::LandingPadInformation::Request>();
            request->name = name;
            request->x = X_;
            request->y = Y_;
            request->z = Z_;

            auto future =  landing_client_->async_send_request(request);
            auto state = future.wait_for(std::chrono::seconds(10));
            
            if (state== std::future_status::ready) {
                auto response = future.get();
                if (response->success ) {
                    setStatus(PAD_IS_OCCUPIED);
                    my_drone_name = request->name;
                    return;
                } else {
                    reqeuested=false;
                    return;
                   
                }
                return;
            }else{
                reqeuested=false;
                
            }
        }  
    }          
    }

    bool lock_single_cluster_node(int node_id, bool lock) { 
        //try to lock or unlock a certain pad
        RCLCPP_INFO(this->get_logger(), "%s : Versuche %s Pad_%d ", name.c_str(), lock? "lock" : "freigabe", node_id);
        std::string service_name = "lock_cluster_Pad_" + std::to_string(node_id);


        lock_cluster_client = this->create_client<LockCluster>(
        service_name,
        rmw_qos_profile_services_default,
        client_callback_group_lock_cluster);

        auto cluster_copy = cluster_nodes_;
        if (! lock_cluster_client->wait_for_service(std::chrono::seconds(3))) { 
            RCLCPP_WARN(this->get_logger(), "Service Lock Single Cluster nicht verfügbar");
            return false;
        } 

        auto request = std::make_shared<LockCluster::Request>();
        request->name = name; 
        request->locking = lock;
        
        auto future =  lock_cluster_client->async_send_request(request);
        auto state = future.wait_for(std::chrono::milliseconds(300));
        
        if (state== std::future_status::ready) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Lock-Response von Pad_%d: success=%d", node_id, response->success);
            if (response->success) { 
                return true; 
            } else {
                RCLCPP_WARN(this->get_logger(), "Locking %s auf Pad_%d fehlgeschlagen", lock ? "aktivieren" : "freigeben", node_id);
                return false; 
            } 
            
        }
        RCLCPP_INFO(this->get_logger(), "Timout von lock single cluster");
        return false;
        
    }


    bool lock_cluster_sync(bool state) {
        //lock or unlock the cluster
        //do not lock if state is locked  
        if (getStatus() == PAD_IS_LOCKED_BY_CLUSTER && state) {
            RCLCPP_INFO(this->get_logger(), "Pad ist schon gelocked");
            return false;  
        }
        //while trying to lock start sending aispacelock
        send_airspace_lock_marker();
        if (cluster_nodes_.empty()) {
            RCLCPP_INFO(this->get_logger(), "Keine Cluster Pads");
            return true;
        }
        auto cluster_copy = cluster_nodes_;

        //lock or unlock
        if (state) {
            for (const auto& node : cluster_copy) {
                if (!lock_single_cluster_node(node.id, true)) {
                    return false;
                }
            }
            RCLCPP_INFO(this->get_logger(), "Lock SUCCESS");
            return true;
        } else {
            std::set<int> unlocked_ids;
            const int max_retries = 20;
            int attempts = 0;

            while (unlocked_ids.size() < cluster_copy.size() && attempts < max_retries) {
                for (const auto& node : cluster_copy) {
                    if (unlocked_ids.count(node.id)) {
                        continue; 
                    }
                    if (lock_single_cluster_node(node.id, false)) {
                        unlocked_ids.insert(node.id);
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Unlock failed for node %d, retrying...", node.id);
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }
                }
                attempts++;
            }
            
            if (unlocked_ids.size() < cluster_copy.size()) {
                RCLCPP_ERROR(this->get_logger(), "Unlock failed for some nodes after %d attempts.", max_retries);
                return false;
            }

            return true;
        }
    }


    void publish_id()
    {
        //publish id and coordinates in order to build clusters
        auto message = pad_node_cpp::msg::NodeBroadcast();
        message.id = id_;
        message.x = X_;
        message.y = Y_;
        message.z = Z_;
        for (const auto& node : cluster_nodes_) {
            message.known_ids.push_back(node.id);
        }
        for (const auto& node : non_cluster_nodes_) {
            message.known_ids.push_back(node.id);
        }
        publisher_broadcast->publish(message);
    }

    void update_received_nodes(){
        //upadte clusters with received ids and coordinates
        auto now = std::chrono::system_clock::now();
        auto threshold = now - std::chrono::seconds(3);
            
        non_cluster_nodes_.erase(
        std::remove_if(non_cluster_nodes_.begin(), non_cluster_nodes_.end(),
                    [threshold](const NodeData& node) {
                        return node.timestamp < threshold;
                    }),
        non_cluster_nodes_.end()
        );

        cluster_nodes_.erase(
            std::remove_if(cluster_nodes_.begin(), cluster_nodes_.end(),
                        [threshold](const NodeData& node) {
                            return node.timestamp < threshold;
                        }),
            cluster_nodes_.end()
        );

    }


    float distance(float x,float y,float z){
        //returns the distanz to a certain point
        return std::sqrt(
            std::pow(x - X_, 2) +
            std::pow(y - Y_, 2) +
            std::pow(z - Z_, 2)
        );
    }


    void receive_id(const pad_node_cpp::msg::NodeBroadcast::SharedPtr msg)
    {
        //received a id, coordinartes and known ids from another pad in the same area, update tables and clusters
        int received_id = msg->id;
        if (received_id == id_) return;

        float received_X = msg->x;
        float received_Y = msg->y;
        float received_Z = msg->z;

        std::vector<int> received_known_ids = msg->known_ids; // Original behalten

        if (distance(received_X, received_Y, received_Z) < cluster_distance) {
            auto it = std::find_if(
                cluster_nodes_.begin(), cluster_nodes_.end(),
                [received_id](const NodeData& data) { return data.id == received_id; }
            );

            if (it == cluster_nodes_.end()) {
                NodeData new_node{received_id, received_X, received_Y, received_Z,received_known_ids,std::chrono::system_clock::now()};
                cluster_nodes_.push_back(new_node); 
            } else {
                it->x = received_X;
                it->y = received_Y;
                it->z = received_Z;
                it->timestamp=std::chrono::system_clock::now();
                it->known_nodes=received_known_ids;
            }
        } else{ //if (distance(received_X, received_Y, received_Z) < total_reichweite) {
            auto it = std::find_if(
                non_cluster_nodes_.begin(), non_cluster_nodes_.end(),
                [received_id](const NodeData& data) { return data.id == received_id; }
            );

            if (it == non_cluster_nodes_.end()) {
                NodeData new_node{received_id, received_X, received_Y, received_Z,received_known_ids,std::chrono::system_clock::now()};
                non_cluster_nodes_.push_back(new_node); 
            } else {
                it->x = received_X;
                it->y = received_Y;
                it->z = received_Z;
                it->timestamp=std::chrono::system_clock::now();
                it->known_nodes=received_known_ids;
            }
        }
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 5) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "ID und Koordinaten erwartet! Beispiel: ros2 run dein_package dein_knoten 1 10 20 30");
        return 1;
    }

    int id = std::stoi(argv[1]);
    float X = std::stof(argv[2]);
    float Y = std::stof(argv[3]);
    float Z = std::stof(argv[4]);


    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<pad_node>(id, X, Y, Z);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

