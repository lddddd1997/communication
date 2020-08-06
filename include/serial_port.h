/** 
* @file     serial_port.h
* @brief    linux串口配置
* @details  
* @note
* @author   lddddd
*           Email: lddddd1997@gmail.com
* @date     2020.6.11 
* @version  1.0
* @par Edit history:          
*   1.0: lddddd, 2020.6.11, Serial work in Blocking mode.
*/

#ifndef COMMUNICATION_SERIAL_PORT_H_
#define COMMUNICATION_SERIAL_PORT_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string>
#include <numeric>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>

namespace lddddd
{
class Serial
{
public:
  // Serial(std::string _name, int _baud_rate);
  Serial();
  ~Serial();
  void ClosePort();
  bool DataWrite(const unsigned char* _buf, int _nWrite);
  bool DataRead(unsigned char* _buf, int _nRead);
protected:
  std::string port_name_;
  int baud_rate_;
private:
  int serial_;
};

// Serial::Serial(std::string _name, int _baud_rate) :
//   port_name_(_name), baud_rate_(_baud_rate)
Serial::Serial()
{
  serial_ = open(port_name_.c_str(), O_RDWR /*| O_NONBLOCK*/);
  if(serial_ == -1)
  {
    ROS_ERROR_STREAM("Failed to open serial port!");
    exit(0);
  }

  struct termios options;
  if(tcgetattr(serial_, &options) != 0){
    ROS_ERROR_STREAM("Can't get serial port sets!");
    exit(0);
  }
  tcflush(serial_, TCIFLUSH);//        TCIFLUSH      清空输入队列
                             //         TCOFLUSH     清空输出队列
                             //         TCIOFLUSH    同时清空输入和输出队列
  switch(baud_rate_)
  {
    case 921600:
      cfsetispeed(&options, B921600);
      cfsetospeed(&options, B921600);
      break;
    case 576000:
      cfsetispeed(&options, B576000);
      cfsetospeed(&options, B576000);
      break;
    case 500000:
      cfsetispeed(&options, B500000);
      cfsetospeed(&options, B500000);
      break;
    case 460800:
      cfsetispeed(&options, B460800);
      cfsetospeed(&options, B460800);
      break;
    case 230400:
      cfsetispeed(&options, B230400);
      cfsetospeed(&options, B230400);
      break;
    case 115200:
      cfsetispeed(&options, B115200);
      cfsetospeed(&options, B115200);
      break;
    case 57600:
      cfsetispeed(&options, B57600);
      cfsetospeed(&options, B57600);
      break;
    case 38400:
      cfsetispeed(&options, B38400);
      cfsetospeed(&options, B38400);
      break;
    case 19200:
      cfsetispeed(&options, B19200);
      cfsetospeed(&options, B19200);
      break;
    case 9600:
      cfsetispeed(&options, B9600);
      cfsetospeed(&options, B9600);
      break;
    case 4800:
      cfsetispeed(&options, B4800);
      cfsetospeed(&options, B4800);
      break;
    default:
      ROS_ERROR_STREAM("Unsupported baud rate!");
      exit(0);
  }
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  options.c_iflag &= ~(INLCR | ICRNL | IGNCR);
  options.c_oflag &= ~(ONLCR | OCRNL);

  // options.c_cc[VTIME] = 1;    //Set timeout of 5*0.1 seconds
  // options.c_cc[VMIN] = 0;    //超时则返回0，不设置时，非阻塞会返回-1

  if(tcsetattr(serial_, TCSANOW, &options) != 0){
    ROS_ERROR_STREAM("Can't set serial port options!");
    exit(0);
  }
}

Serial::~Serial()
{

}

bool Serial::DataRead(unsigned char* _buf, int _nRead)
{
  int total = 0, ret = 0;

  while(total != _nRead)
  {
    ret = read(serial_, _buf + total, (_nRead - total));
    if(ret < 0)    //Timeout
    {
      return false;
    }
    total += ret;
  }
  return true;
}

bool Serial::DataWrite(const unsigned char* _buf, int _nWrite)
{
  int total = 0, ret = 0;

  while(total != _nWrite)
  {
    ret = write(serial_, _buf + total, (_nWrite - total));
    if(ret < 0)    //Timeout
    {
      return false;
    }
    total += ret;
  }
  return true;
}

void Serial::ClosePort()
{
  close(serial_);
}
}


#endif
