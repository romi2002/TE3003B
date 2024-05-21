#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "absl/strings/str_format.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>
#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <google/protobuf/empty.pb.h>
#include "RobotState.grpc.pb.h"

#include <boost/gil/extension/dynamic_image/any_image.hpp>
#include <boost/gil/extension/io/jpeg.hpp>
#include <boost/mpl/vector.hpp>


using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using robotonotos::RobotState;
using robotonotos::ImageReply;
using google::protobuf::Empty;
using namespace std::chrono_literals;

class BridgeSubscriber : public rclcpp::Node {
public:
  BridgeSubscriber() : Node("bridge_subscriber") {
    std::cout << "creating node" << std::endl;
    test_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "debug_topic", 10, [this](const std_msgs::msg::String::SharedPtr msg){
        std::cout << "SUB" << std::endl;
        std::lock_guard<std::mutex> lg(this->data_lock_);
        recv_value_ = msg->data;
      }
    );
    std::cout << "created sub" << std::endl;
  }

  std::string get_data(){
    std::lock_guard<std::mutex> lg(data_lock_);
    return recv_value_;
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr test_subscription_;
  std::mutex data_lock_;
  std::string recv_value_{"NA"};
};

class BridgeServiceImpl final : public RobotState::Service {
public:
  BridgeServiceImpl(const std::shared_ptr<BridgeSubscriber> &node){
    node_ = node;
  }

  Status GetImage(ServerContext* context, const Empty* empty_req,
                  ImageReply* reply) override {
    // Create (test image) and encode.
    boost::gil::rgb8_image_t image(800, 600);
    std::stringstream buffer(std::ios_base::out | std::ios_base::binary);
    boost::gil::write_view(buffer, view(image), boost::gil::jpeg_tag());
    reply->set_width(800);
    reply->set_height(600);
    *reply->mutable_img() = buffer.str();
    return Status::OK;
  }
private:
  std::shared_ptr<BridgeSubscriber> node_;
};

void RunServer(const std::shared_ptr<BridgeSubscriber> &node) {
  std::string server_address("0.0.0.0:50051");
  BridgeServiceImpl service(node);

  grpc::EnableDefaultHealthCheckService(true);
  grpc::reflection::InitProtoReflectionServerBuilderPlugin();
  ServerBuilder builder;
  // Listen on the given address without any authentication mechanism.
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  // Register "service" as the instance through which we'll communicate with
  // clients. In this case it corresponds to an *synchronous* service.
  builder.RegisterService(&service);
  // Finally assemble the server.
  std::unique_ptr<Server> server(builder.BuildAndStart());
  std::cout << "Server listening on " << server_address << std::endl;

  // Wait for the server to shutdown. Note that some other thread must be
  // responsible for shutting down the server for this call to ever return.
  server->Wait();
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BridgeSubscriber>();
  std::thread rpc_thread([node] { RunServer(node); } );

  rclcpp::spin(node);
  rclcpp::shutdown();
  rpc_thread.join();

  return 0;
}
