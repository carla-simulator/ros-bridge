#include <memory>
#include <string>
#include <functional>
#include <queue>
#include <utility>
#include <vector>

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "rclcpp/rclcpp.hpp"

#include "rosgraph_msgs/msg/clock.hpp"

using std::placeholders::_1;

class NetworkSimulatorBridge : public rclcpp::Node
{
  public:
    NetworkSimulatorBridge()
    : Node("ns3_ros_bridge")
    {
      // additional delay added to ns-3 (milliseconds)
      this->declare_parameter("delay_ms", 0);
      delay = this->get_parameter("delay_ms").as_int();

      // duration of simulation (seconds)
      this->declare_parameter("stoptime", -1);
      stopTime = this->get_parameter("stoptime").as_int();

      RCLCPP_INFO(this->get_logger(), "delay_ms=%d stoptime=%d", delay, stopTime);

      reset();

      //exitTopic = this->create_publisher<test_msg::msg::NodeExit>("exit", 10);
      //responseTopic = this->create_publisher<test_msg::msg::Response>("response", 10);
      //carTopic = this->create_subscription<test_msg::msg::Car>("car", 10, std::bind(&NS3Node::car_callback, this, _1));
      clockTopic = this->create_subscription<rosgraph_msgs::msg::Clock>("clock", 10, std::bind(&NetworkSimulatorBridge::clock_callback, this, _1));

      clock_ms = 0;
      terminate = false;
/*
      // TODO: initialized based on first received value
      currentResponse.last_received_bsm1 = 0;
      currentResponse.last_received_bsm2 = 0;
      currentResponse.last_received_bsm3 = 0;
*/
      setup_server();
    }

    ~NetworkSimulatorBridge()
    {
      shutdown(acceptedClient, SHUT_RDWR);
      close(acceptedClient);
    }

  private:
    void reset()
    {
      for(std::size_t i = 0; i < DATA_SIZE; i++) {
        dataUpdated[i] = false;
      }
    }

    void setup_server()
    {
      // Create a socket
      serverSocket = socket(AF_INET, SOCK_STREAM, 0);
      if (serverSocket == -1) {
        throw std::runtime_error("Failed to create socket.");
      }

      // Prepare the server address
      struct sockaddr_in serverAddress{};
      serverAddress.sin_family = AF_INET;
      serverAddress.sin_port = htons(1111);
      serverAddress.sin_addr.s_addr = INADDR_ANY;

      int reuse = 1;
      if (setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) == -1) {
        throw std::runtime_error("Failed to set socket options.");
      }

      // Bind the socket to the server address
      if (bind(serverSocket, reinterpret_cast<struct sockaddr*>(&serverAddress), sizeof(serverAddress)) == -1) {
        throw std::runtime_error("Failed to bind socket.");
      }

      // Listen for incoming connections
      if (listen(serverSocket, 1) == -1) {
        throw std::runtime_error("Failed to listen for connections.");
      }

      // Accept incoming connections
      struct sockaddr_in clientAddress{};
      socklen_t clientAddressSize = sizeof(clientAddress);
      acceptedClient = accept(serverSocket, reinterpret_cast<struct sockaddr*>(&clientAddress), &clientAddressSize);
      if (acceptedClient == -1) {
        throw std::runtime_error("Failed to accept connection.");
      }

      RCLCPP_INFO(this->get_logger(), "server_setup complete");
    }
/*
    void car_callback(const test_msg::msg::Car & msg)
    {
      RCLCPP_INFO(this->get_logger(), "Received: %s accel=%f x=%f", msg.id.c_str(), msg.accel, msg.x);

      std::size_t startIndex = 0;
      if (msg.id == "car1") {
        startIndex = 2;
      } else if (msg.id == "car2") {
        startIndex = 4;
      } else if (msg.id == "car3") {
        startIndex = 6;
      }

      if (dataUpdated[startIndex]) {
        RCLCPP_WARN(this->get_logger(), "received multiple car updates for %s", msg.id.c_str());
      }
      
      data[startIndex] = std::to_string(msg.x);
      data[startIndex+1] = std::to_string(msg.accel);
      dataUpdated[startIndex] = true;
      dataUpdated[startIndex+1] = true;

      update();
    }
*/
    void clock_callback(const rosgraph_msgs::msg::Clock & msg)
    {
      RCLCPP_INFO(this->get_logger(), "Received: clock %d s/%d ns", msg.clock.sec, msg.clock.nanosec);

      if (dataUpdated[0]) {
        RCLCPP_WARN(this->get_logger(), "received multiple clock updates: %s and %d", data[0].c_str(), msg.clock.sec);
      }
      data[0] = std::to_string(msg.clock.sec);
      data[1] = std::to_string(msg.clock.nanosec); 
      dataUpdated[0] = true;
      dataUpdated[1] = true;

      clock_ms = (msg.clock.sec * 1000) + (msg.clock.nanosec / 1000000);
      RCLCPP_DEBUG(this->get_logger(), "clock_ms = %ld", clock_ms);

      if (stopTime > 0 && msg.clock.sec >= stopTime) {
        terminate = true;
      }

      update();
    }

    void update()
    {
      if (terminate) {
        clean_exit(); // will exit
      }

      for (std::size_t i = 0; i < DATA_SIZE; i++) {
        if (!dataUpdated[i]) {
          return;
        }
      }

      // Send message to the client
      std::string dataString = createMessage();
      RCLCPP_INFO(this->get_logger(), "[ns-3] sending %s", dataString.c_str());
      if (send(acceptedClient, dataString.c_str(), dataString.size(), 0) == -1) {
        throw std::runtime_error("Failed to send data.");
      }

      // Receive response from the ns-3 client
      char incomingBuffer[4096];
      int bytesReceived = recv(acceptedClient, incomingBuffer, sizeof(incomingBuffer), 0);
      if (bytesReceived == -1) {
        throw std::runtime_error("Failed to read data from socket.");
      }
      RCLCPP_INFO(this->get_logger(), "[ns-3] received %s", incomingBuffer);
      handleMessage(incomingBuffer);
/*
      if (!responseQueue.empty()) {
        while (responseQueue.front().first <= clock_ms) {
          currentResponse = responseQueue.front().second;
          responseQueue.pop(); // some responses may be skipped
        }
      }
      RCLCPP_INFO(this->get_logger(), "[ros] published response %f %f %f @ %ld",
          currentResponse.last_received_bsm1,
          currentResponse.last_received_bsm2,
          currentResponse.last_received_bsm3,
          clock_ms);
      responseTopic->publish(currentResponse);
*/
      reset();
    }

    std::string createMessage() {
      std::string message = data[0];
      for (std::size_t i = 1; i < DATA_SIZE; i++) {
        message += "," + data[i];
      }
      message += "\n";
      return message;
    }

    void handleMessage(const char* message) {
      std::string messageString(message);
      std::stringstream s_stream(messageString);

      //parsing up data and packing in subVar
      std::vector<double> subVar;
      while(s_stream.good()){
          std::string substr;
          getline(s_stream,substr,',');
          subVar.push_back(static_cast<double>(atof(substr.c_str())));
      }

      if (subVar.size() != 3) {
        RCLCPP_ERROR(this->get_logger(), "[ns-3] received bad response from ns-3");
        return;
      }
/*
      // Forward response to ROS
      test_msg::msg::Response response;
      response.last_received_bsm1 = subVar[0];
      response.last_received_bsm2 = subVar[1];
      response.last_received_bsm3 = subVar[2];
      if (delay > 0) {
        responseQueue.push(std::make_pair(clock_ms + delay, response));
      } else { // TODO: handle negative values
        currentResponse = response;
      }
*/
    }

    void clean_exit()
    {
      // Send message to the client
      std::string dataString = "-1\n";
      RCLCPP_INFO(this->get_logger(), "[ns-3] sending %s", dataString.c_str());
      if (send(acceptedClient, dataString.c_str(), dataString.size(), 0) == -1) {
        throw std::runtime_error("Failed to send data.");
      }
/*
      test_msg::msg::NodeExit message;
      message.node_id = "ns3_node";
      exitTopic->publish(message);
*/
      exit(0); // destroy self
    }

    //rclcpp::Publisher<test_msg::msg::NodeExit>::SharedPtr exitTopic;
    //rclcpp::Publisher<test_msg::msg::Response>::SharedPtr responseTopic;
    //rclcpp::Subscription<test_msg::msg::Car>::SharedPtr carTopic;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clockTopic;

    static const std::size_t DATA_SIZE = 8;
    std::string data[DATA_SIZE]; // s ns a1 x1 a2 x2 a3 x3
    bool dataUpdated[DATA_SIZE];

    int serverSocket;
    int acceptedClient;

    int delay;
    int stopTime;

    uint64_t clock_ms;
    //test_msg::msg::Response currentResponse; // TODO: find out how to remove
    //std::queue< std::pair<uint64_t,test_msg::msg::Response> > responseQueue;

    bool terminate;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NetworkSimulatorBridge>());
  rclcpp::shutdown();
  return 0;
}

