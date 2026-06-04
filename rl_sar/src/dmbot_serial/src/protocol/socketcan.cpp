/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 3/3/21.
//
#include "dmbot_serial/protocol/socketcan.h"
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <utility>
#include <linux/can.h>        
#include <linux/can/raw.h> 

namespace damiao
{
/* ref:
 * https://github.com/JCube001/socketcan-demo
 * http://blog.mbedded.ninja/programming/operating-systems/linux/how-to-use-socketcan-with-c-in-linux
 * https://github.com/linux-can/can-utils/blob/master/candump.c
 */

SocketCAN::~SocketCAN()
{
  if (this->isOpen())
    this->close();
}

void SocketCAN::log_throttled_error(const std::string& interface_name) const{
  static auto last_log_time = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed = now - last_log_time;

  if (elapsed.count() >= 5.0) {
      std::cerr << "Unable to write: Socket " << interface_name << " not open" << std::endl;
      last_log_time = now;
  }
}


bool SocketCAN::open(const std::string& interface, boost::function<void(const canfd_frame& frame)> handler,
                     int thread_priority)
{
  //在static void* socketcan_receiver_thread(void* argv)这个函数里
  //sock->reception_handler(rx_frame);
  reception_handler = std::move(handler);
  // Request a socket
  sock_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);//socket() 函数返回一个 socketcan 的句柄，后续的操作都是基于这个句柄的。
  if (sock_fd_ == -1)
  {
    //ROS_ERROR("Error: Unable to create a CAN socket");
    std::cerr << "[ERROR] Error: Unable to create a CAN socket" << std::endl;
    return false;
  }
  int enable_canfd = 1;
  if (setsockopt(sock_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) != 0)
  {
    std::cerr << "[ERROR] Failed to enable CAN FD support on socket" << std::endl;
    ::close(sock_fd_);
    sock_fd_ = -1;
    return false;
  }
  char name[16] = {};  // avoid stringop-truncation
  strncpy(name, interface.c_str(), interface.size());
  strncpy(interface_request_.ifr_name, name, IFNAMSIZ);
  // Get the index of the network interface
  if (ioctl(sock_fd_, SIOCGIFINDEX, &interface_request_) == -1)
  {
    //ROS_ERROR("Unable to select CAN interface %s: I/O control error", name);
    std::cerr << "[ERROR] Unable to select CAN interface " << name << ": I/O control error" << std::endl;
    // Invalidate unusable socket
    close();
    return false;
  }
  // Bind the socket to the network interface
  address_.can_family = AF_CAN;// 指定协议族
  address_.can_ifindex = interface_request_.ifr_ifindex; // 设备索引
  int rc = bind(sock_fd_, reinterpret_cast<struct sockaddr*>(&address_), sizeof(address_));
  if (rc == -1)
  {
    //ROS_ERROR("Failed to bind socket to %s network interface", name);
    std::cerr << "[ERROR] Failed to bind socket to " << name << " network interface" << std::endl;
    close();
    return false;
  }
  // Start a separate, event-driven thread for frame reception
  return startReceiverThread(thread_priority);
}

void SocketCAN::close()
{
  terminate_receiver_thread_ = true;
  while (receiver_thread_running_)
    ;

  if (!isOpen())
    return;
  // Close the file descriptor for our socket
  ::close(sock_fd_);
  sock_fd_ = -1;
}

bool SocketCAN::isOpen() const
{
  return (sock_fd_ != -1);
}

void SocketCAN::write(can_frame* frame) const
{
  if (!isOpen())
  {
    //ROS_ERROR_THROTTLE(5., "Unable to write: Socket %s not open", interface_request_.ifr_name);
    log_throttled_error(interface_request_.ifr_name);
    return;
  }
  if (::write(sock_fd_, frame, sizeof(can_frame)) == -1)
    //ROS_DEBUG_THROTTLE(5., "Unable to write: The %s tx buffer may be full", interface_request_.ifr_name);
    log_throttled_error(interface_request_.ifr_name);
}

void SocketCAN::write2(canfd_frame* frame) const
{
  if (!isOpen())
  {
    //ROS_ERROR_THROTTLE(5., "Unable to write: Socket %s not open", interface_request_.ifr_name);
    log_throttled_error(interface_request_.ifr_name);
    return;
  }
  if (::write(sock_fd_, frame, sizeof(canfd_frame)) == -1)
    //ROS_DEBUG_THROTTLE(5., "Unable to write: The %s tx buffer may be full", interface_request_.ifr_name);
    log_throttled_error(interface_request_.ifr_name);
}

static void* socketcan_receiver_thread(void* argv)
{
  /*
   * The first and only argument to this function
   * is the pointer to the object, which started the thread.
   */
  auto* sock = (SocketCAN*)argv;
  // Holds the set of descriptors, that 'select' shall monitor
  fd_set descriptors;
  // Highest file descriptor in set
  int maxfd = sock->sock_fd_;
  // How long 'select' shall wait before returning with timeout
  struct timeval timeout
  {
  };
  // Buffer to store incoming frame
  // 初始化接收CAN帧的缓冲区
  canfd_frame rx_frame{};// 使用CAN帧结构体的默认构造函数进行初始化
  // Run until termination signal received  
  sock->receiver_thread_running_ = true;// 标记接收线程正在运行
  // 循环运行，直到接收到终止信号  
  while (!sock->terminate_receiver_thread_)
  {// 每次循环都重新设置超时时间，确保不会无限期等待  
    timeout.tv_sec = 1.;  // Should be set each loop  
    // Clear descriptor set
    FD_ZERO(&descriptors);
    // Add socket descriptor
    FD_SET(sock->sock_fd_, &descriptors);
    // Wait until timeout or activity on any descriptor
    if (select(maxfd + 1, &descriptors, nullptr, nullptr, &timeout))
    {
      size_t len = read(sock->sock_fd_, &rx_frame, CANFD_MTU);
      if (len < 0)
        continue;
      if (sock->reception_handler != nullptr)
        sock->reception_handler(rx_frame);
    }
  }
   // 标记接收线程已停止运行 
  sock->receiver_thread_running_ = false;
  return nullptr;
}

bool SocketCAN::startReceiverThread(int thread_priority)
{
  // Frame reception is accomplished in a separate, event-driven thread.
  // See also: https://www.thegeekstuff.com/2012/04/create-threads-in-linux/
  // 标记为false，表示接收线程尚未被请求终止。  
  // 这个标志将在需要停止线程时被设置为true.
  terminate_receiver_thread_ = false;
  // 使用pthread_create函数创建一个新线程，该线程将执行socketcan_receiver_thread函数
   // receiver_thread_id_用于存储新创建的线程的标识符。
    // 第四个参数是传递给线程函数的参数，这里是当前SocketCAN类的实例指针（this）。  
  int rc = pthread_create(&receiver_thread_id_, nullptr, &socketcan_receiver_thread, this);
  if (rc != 0)
  {
    //ROS_ERROR("Unable to start receiver thread");
    std::cerr << "[ERROR] Unable to start receiver thread" << std::endl;
    return false;
  }
  //ROS_INFO("Successfully started receiver thread with ID %lu", receiver_thread_id_);
  std::cout << "[INFO] Successfully started receiver thread with ID " << receiver_thread_id_ << std::endl;
  sched_param sched{ .sched_priority = thread_priority };
  // 使用pthread_setschedparam函数设置接收线程的调度策略和优先级。  
  // 第一个参数是线程的标识符。  
  // 第二个参数是调度策略，这里使用SCHED_FIFO（先进先出调度策略）。  
  // 第三个参数是指向sched_param结构体的指针，包含具体的调度参数。  
  // 注意：设置线程的调度策略和优先级可能需要相应的权限（如root权限）。
  pthread_setschedparam(receiver_thread_id_, SCHED_FIFO, &sched);
  return true;
}

}  // namespace can
