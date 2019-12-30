/*
 * \file main.cpp
 * \author Pedro Vaz Teixeira
 * \brief Caesar service server
 */

#include <map>
#include <mutex>

#include <ros/ros.h>

#include <caesar_ros/AddFactor.h>
#include <caesar_ros/AddNode.h>
#include <caesar_ros/GetEstimate.h>

#include <graff/graff.hpp>

namespace {
class Server {
  // private:
  Server(){}; // can't touch this
public:
  Server(const std::string &address, const std::string &session)
      : session_(session) {

    // setup ROS side: advertise services
    add_node_srv_ =
        node_handle_.advertiseService("AddNode", &Server::addNode, this);
    add_factor_srv_ =
        node_handle_.advertiseService("AddFactor", &Server::addFactor, this);
    get_estimate_srv_ = node_handle_.advertiseService(
        "GetEstimate", &Server::getEstimate, this);

    // setup Caesar side
    endpoint_.Connect(address);

    // TODO: register robot, session
  };


  // addFactor.srv
  // REQUEST:
  // string id0
  // string id1
  // geometry_msgs/PoseWithCovariance measurement
  // ---
  // REPLY:
  // string status
  bool addFactor(caesar_ros::AddFactor::Request &req,
                 caesar_ros::AddFactor::Response &res) {
    geometry_msgs::Point zp = req.measurement.pose.position;
    geometry_msgs::Quaternion zo = req.measurement.pose.orientation;
    std::vector<double> mu = {zp.x, zp.y, zp.z, zo.x, zo.y, zo.z, zo.w};
    std::vector<double> cov(req.measurement.covariance.begin(),
                            req.measurement.covariance.end());
    graff::Normal *z = new graff::Normal(mu, cov);

    std::vector<std::string> nodes{req.id0, req.id1};
    graff::Factor factor("Pose3Pose3", nodes);
    factor.push_back(z);
    res.status = graff::AddFactor(endpoint_, session_, factor);
    return (true);
  }


  bool addNode(caesar_ros::AddNode::Request &req,
               caesar_ros::AddNode::Response &res) {
    graff::Variable pose(req.id, "Pose3");
    res.status = graff::AddVariable(endpoint_, session_, pose);
    return (true);
  }


  bool getEstimate(caesar_ros::GetEstimate::Request &req,
                   caesar_ros::GetEstimate::Response &res) {
    // retrieve and send estimate from latest solution
    auto rep = graff::GetVarMAPMean(endpoint_, session_, req.query);
    // TODO: some parsing to fill out response
    // (geometry_msgs::PoseWithCovariance)
    return (true);
  }

  void solver(void) {
    ros::Rate rate(1);
    while (ros::ok()) {
      // TODO fetch the latest estimates from the endpoint and cache them
      // locally
      rate.sleep();
    }
  }

private:
  // ros
  ros::NodeHandle node_handle_;
  ros::ServiceServer add_factor_srv_, add_node_srv_, get_estimate_srv_;

  // caesar
  graff::Endpoint endpoint_;
  graff::Session session_;
}; // class Server
}; // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "caesar_server");

  std::string ep_address("tcp://127.0.0.1:5555"), ep_session("ros-test");
  Server caesar_server(ep_address, ep_session );

  ros::Rate rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
}
