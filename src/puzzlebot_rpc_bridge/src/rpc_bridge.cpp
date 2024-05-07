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

#include "Test.grpc.pb.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using helloworld::HelloRequest;
using helloworld::HelloReply;
using helloworld::Greeter;
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

class BridgeServiceImpl final : public Greeter::Service {
public:
  BridgeServiceImpl(const std::shared_ptr<BridgeSubscriber> &node){
    node_ = node;
  }

  Status SayHello(ServerContext* context, const HelloRequest* request,
                  HelloReply* reply) override {
    std::string prefix(node_->get_data());
    reply->set_message(prefix + request->name());
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
