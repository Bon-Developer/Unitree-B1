// 240724 code

#include <iostream>
#include "b1_base/base.hpp"

using namespace qre;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "base_node"); //node 이름: base_node
  ros::NodeHandle nh, p_nh("~");
  Base* base_driver; //Base 클래스의 포인터 변수 base_driver 선언
  UT::LoopFunc* control_loop; //UT::LoopFunc의 포인터 변수 control_loop 선언
  UT::InitEnvironment(); //UT 네임스페이스 내의 InitEnvironment() 함수 호출

  //모드를 선택하는 거긴 한데 어떤 기준으로 선택이 되는건지를 모르겠네
  if (argc >= 2) {
    if (strcmp(argv[1], "high_level") == 0) {
      std::cout << "Working mode is set to: " << argv[1] << std::endl;
      UT::HighCmd cmd;
      UT::HighState state;
      base_driver = new Base(cmd, state, &nh, &p_nh);
      control_loop = new UT::LoopFunc("control_loop", base_driver->dt, boost::bind(&Base::HighLevelControl, base_driver));
    }

    else if (strcmp(argv[1], "low_level") == 0) {
      std::cout << "Working mode is set to: " << argv[1] << std::endl;
      UT::LowCmd cmd;
      UT::LowState state;
      base_driver = new Base(cmd, state, &nh, &p_nh);
      control_loop = new UT::LoopFunc("control_loop", base_driver->dt, boost::bind(&Base::LowLevelControl, base_driver));

    else {
      std::cout << "Invalid working mode" << std::endl;
      std::cout << "Supported modes_ are: high_level and low_level" << std::endl;
      exit(-1);
    }
  }
  else {
    std::cout << "invalid mode not specified" << std::endl;
    std::cout << "Supported modes_ are: high_level and low_level" << std::endl;
    exit(-1);
  }
  UT::LoopFunc comm_bus_publisher("udp_send", base_driver->dt, 3, boost::bind(&Base::packetSend, base_driver);
  UT::LoopFunc comm_bus_subscriber("udp_recv", base_driver->dt, 3, boost::bind(&Base::packetReceive, base_driver));
  comm_bus_publisher.start();
  comm_bus_subscriber.start();
  control_loop->start();
  ros::Rate loop_rate(50);
  while (ros::ok()) {
    base_driver->publishStateMessages();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}







    
