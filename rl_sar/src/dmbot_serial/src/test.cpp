#include "dmbot_serial/protocol/damiao.h"
#include <csignal>

// Atomic flag for safe cross-thread modification
std::atomic<bool> running(true);

// Signal handler triggered by Ctrl+C
void signalHandler(int signum) {
    running = false;
    std::cerr << "\nInterrupt signal (" << signum << ") received.\n";
}

int main(int argc, char** argv)
{
  using clock = std::chrono::steady_clock;
  using duration = std::chrono::duration<double>;

  std::signal(SIGINT, signalHandler);

  try 
  {
      uint16_t canid1 = 0x01;
      uint16_t mstid1 = 0x11;
      uint16_t canid2 = 0x02;
      uint16_t mstid2 = 0x12;
      uint16_t canid3 = 0x03;
      uint16_t mstid3 = 0x13;
      uint16_t canid4 = 0x04;
      uint16_t mstid4 = 0x14;
      uint16_t canid5 = 0x05;
      uint16_t mstid5 = 0x15;
      uint16_t canid6 = 0x06;
      uint16_t mstid6 = 0x16;
      
      uint32_t nom_baud =1000000;
      uint32_t dat_baud =5000000;

      std::vector<damiao::DmActData> init_data;
      std::vector<damiao::DmActData> init_data2;
        init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
                                            .mode = damiao::MIT_MODE,
                                            .can_id=canid1,
                                            .mst_id=mstid1 });
        // init_data.push_back(damiao::DmActData{.motorType = damiao::DM6248P,
        //                                     .mode = damiao::MIT_MODE,
        //                                     .can_id=canid2,
        //                                     .mst_id=mstid2 });
        // init_data.push_back(damiao::DmActData{.motorType = damiao::DM6248P,
        //                                         .mode = damiao::MIT_MODE,
        //                                         .can_id=canid3,
        //                                         .mst_id=mstid3 });
        // init_data.push_back(damiao::DmActData{.motorType = damiao::DM6248P,
        //                                         .mode = damiao::MIT_MODE,
        //                                         .can_id=canid4,
        //                                         .mst_id=mstid4 });
        // init_data.push_back(damiao::DmActData{.motorType = damiao::DM6248P,
        //                                     .mode = damiao::MIT_MODE,
        //                                     .can_id=canid5,
        //                                     .mst_id=mstid5 });
        // init_data.push_back(damiao::DmActData{.motorType = damiao::DM6248P,
        //                                         .mode = damiao::MIT_MODE,
        //                                         .can_id=canid6,
        //                                         .mst_id=mstid6 });
        
        // init_data2.push_back(damiao::DmActData{.motorType = damiao::DM4310,
        //                                         .mode = damiao::POS_VEL_MODE,
        //                                         .can_id=canid1,
        //                                         .mst_id=mstid1 });
        init_data2.push_back(damiao::DmActData{.motorType = damiao::DM4310,
                                            .mode = damiao::MIT_MODE,
                                            .can_id=canid2,
                                            .mst_id=mstid2 });
        // init_data2.push_back(damiao::DmActData{.motorType = damiao::DM6248P,
        //                                         .mode = damiao::MIT_MODE,
        //                                         .can_id=canid3,
        //                                         .mst_id=mstid3 });
        // init_data2.push_back(damiao::DmActData{.motorType = damiao::DM6248P,
        //                                         .mode = damiao::MIT_MODE,
        //                                         .can_id=canid4,
        //                                         .mst_id=mstid4 });
        // init_data2.push_back(damiao::DmActData{.motorType = damiao::DM6248P,
        //                                     .mode = damiao::MIT_MODE,
        //                                     .can_id=canid5,
        //                                     .mst_id=mstid5 });
        // init_data2.push_back(damiao::DmActData{.motorType = damiao::DM6248P,
        //                                         .mode = damiao::MIT_MODE,
        //                                         .can_id=canid6,
        //                                         .mst_id=mstid6 });
     auto robot_ptr1_ = std::make_shared<damiao::Motor_Control>("can0",&init_data,damiao::canfd);
     
      auto robot_ptr2_ = std::make_shared<damiao::Motor_Control>("can1",&init_data2,damiao::canfd);
  //std::chrono::steady_clock::time_point last_time_;
      while (running) 
      {  auto last_time_ = std::chrono::steady_clock::now();
        const duration desired_duration(0.001); // Compute the desired period
        auto current_time = clock::now();

        // for (int i = 1; i < 7; i++)
        // {//front-left leg, front-right leg(Motor &DM_Motor, float kp, float kd, float q, float dq, float tau)
        //     robot_ptr1_->control_mit(*robot_ptr1_->getMotor(i), 0.0, 0.0, 0.0, 0.0, 0.0);  
        // }
        // for (int i = 1; i < 7; i++)
        // {//rear-left leg, rear-right leg
        //     robot_ptr2_->control_mit(*robot_ptr2_->getMotor(i), 0.0, 0.0, 0.0, 0.0, 0.0);  
        // }
        // for (int i = 1; i < 7; i++)
        // {//front-left leg, front-right leg(Motor &DM_Motor, float kp, float kd, float q, float dq, float tau)
        //     robot_ptr1_->set_zero_position(*robot_ptr1_->getMotor(i)); 
        // }
        // for (int i = 1; i < 7; i++)
        // {//rear-left leg, rear-right leg
        //     robot_ptr2_->set_zero_position(*robot_ptr2_->getMotor(i)); 
        // }
      //   std::cerr<<"robot_ptr1_: "<<std::endl;
      //   for(uint16_t id = 1;id<7;id++)
      //  {
      //    float pos=robot_ptr1_->getMotor(id)->Get_Position();
      //    float vel=robot_ptr1_->getMotor(id)->Get_Velocity();
      //    float tau=robot_ptr1_->getMotor(id)->Get_tau();
      //    double interval=robot_ptr1_->getMotor(id)->getTimeInterval() ;
      //    std::cerr<<"canid is: "<<id<<" pos: "<<pos<<" vel: "<<vel
      //            <<" effort: "<<tau<<" time(s): "<<interval<<std::endl;
      //  }

      //  std::cerr<<"robot_ptr2_: "<<std::endl;
      //   for(uint16_t id = 1;id<7;id++)
      //  {
      //    float pos=robot_ptr2_->getMotor(id)->Get_Position();
      //    float vel=robot_ptr2_->getMotor(id)->Get_Velocity();
      //    float tau=robot_ptr2_->getMotor(id)->Get_tau();
      //    double interval=robot_ptr2_->getMotor(id)->getTimeInterval() ;
      //    std::cerr<<"canid is: "<<id<<" pos: "<<pos<<" vel: "<<vel
      //            <<" effort: "<<tau<<" time(s): "<<interval<<std::endl;
      //  }
      robot_ptr1_->control_mit(*robot_ptr1_->getMotor(1), 0.0, 0.0, 0.0, 0.0, 0);
      robot_ptr2_->control_mit(*robot_ptr2_->getMotor(2), 1.0, 0.5, robot_ptr1_->getMotor(1)->Get_Position(), 0.0, 0);
      // robot_ptr1_->control_vel(*robot_ptr1_->getMotor(1),1.0);
      // robot_ptr2_->control_vel(*robot_ptr2_->getMotor(2),1.0);
      std::cerr<<"canid is: "<<1<<" pos: "<<robot_ptr1_->getMotor(1)->Get_Position()<<" time(s): "<<robot_ptr1_->getMotor(1)->getTimeInterval()<<std::endl;
      std::cerr<<"canid is: "<<2<<" pos: "<<robot_ptr2_->getMotor(2)->Get_Position()<<" time(s): "<<robot_ptr2_->getMotor(2)->getTimeInterval()<<std::endl;
//            auto now = std::chrono::steady_clock::now();
//     std::chrono::duration<double> dt = now - last_time_;

// std::cerr<<"dt: "<<dt.count()<<std::endl;
   // delta_time_ = dt.count(); // in seconds
        
        const auto sleep_till = current_time + std::chrono::duration_cast<clock::duration>(desired_duration);
        std::this_thread::sleep_until(sleep_till);    
      }

      std::cout <<  std::endl<<"The program exited safely." << std::endl<< std::endl;
  }
  catch (const std::exception& e) {
      std::cerr << "Error: hardware interface exception: " << e.what() << std::endl;
      return 1;
  }

  return 0;
}
