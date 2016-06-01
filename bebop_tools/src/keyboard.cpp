/*
 * teleop_pr2_keyboard
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Kevin Watts

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>

#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_C 0x63
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77 
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_O 0x6f
#define KEYCODE_P 0x70

#define KEYCODE_Y 0x79
#define KEYCODE_H 0x6a

#define KEYCODE_F 0x66
#define KEYCODE_L 0x6c
#define KEYCODE_T 0x74

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57
#define KEYCODE_Q_CAP 0x51
#define KEYCODE_E_CAP 0x45
#define KEYCODE_P_CAP 0x50

#define KEYCODE_Y_CAP 0x59
#define KEYCODE_H_CAP 0x4a

class TeleopUAVKeyboard
{
private:
  double walk_vel, run_vel, yaw_rate, yaw_rate_run, vertical_vel;
  int rate_;
  geometry_msgs::Twist cmd;
  ros::NodeHandle n_;
  ros::Publisher vel_pub_, takeoff_pub_, land_pub_, flip_pub_;

public:
    void init()
    {
        cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;

        vel_pub_ = n_.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1);
        takeoff_pub_ = n_.advertise<std_msgs::Empty>("bebop/takeoff", 1);
        land_pub_ = n_.advertise<std_msgs::Empty>("bebop/land", 1);
        flip_pub_ = n_.advertise<std_msgs::UInt8>("bebop/flip", 1);

        ros::NodeHandle n_private("~");
        n_private.param("rate", rate_, 20);
        n_private.param("walk_vel", walk_vel, 0.2);
        n_private.param("run_vel", run_vel, 1.0);
        n_private.param("yaw_rate", yaw_rate, 0.2);
        n_private.param("yaw_run_rate", yaw_rate_run, 0.5);
        n_private.param("vertical_vel", vertical_vel, 0.1);
    }

    ~TeleopUAVKeyboard()   { }
    void keyboardLoop();

};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    exit(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "keyboard");

    TeleopUAVKeyboard tpk;
    tpk.init();

    signal(SIGINT,quit);

    tpk.keyboardLoop();

    return(0);
}

void TeleopUAVKeyboard::keyboardLoop()
{
  char c;
  bool dirty=false;
  bool dirtygripper=false;

  ros::Rate loop_rate(rate_);

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Press 'T' to takeoff");
  puts("Press 'L' to land");


  puts("Use 'WASD' to horizontal translate");
  puts("Use 'QE' to yaw");
  puts("Use 'YH' to up/down");
  puts("Press 'Shift' to run");

  puts("Press 'F' to flip");

  while (1)
    {
      // get the next event from the keyboard
      if(read(kfd, &c, 1) < 0)
        {
          perror("read():");
          exit(-1);
        }

      cmd.linear.x = cmd.linear.y = cmd.angular.z = cmd.linear.z = 0;

      std_msgs::Empty temp;
      std_msgs::UInt8 flip;
      switch(c)
        {
          // Walking
        case KEYCODE_T:
          takeoff_pub_.publish(temp);
          break;
        case KEYCODE_L:
          land_pub_.publish(temp);
          break;
        case KEYCODE_F:
          flip.data = 1;
          flip_pub_.publish(flip);
          break;
        case KEYCODE_W:
          cmd.linear.x = walk_vel;
          break;
        case KEYCODE_S:
          cmd.linear.x = - walk_vel;
          break;
        case KEYCODE_A:
          cmd.linear.y = walk_vel;
          break;
        case KEYCODE_D:
          cmd.linear.y = - walk_vel;
          break;
        case KEYCODE_Q:
          cmd.angular.z = yaw_rate;
          break;
        case KEYCODE_E:
          cmd.angular.z = - yaw_rate;
          break;
        case KEYCODE_Y:
          cmd.linear.z = vertical_vel;
          break;
        case KEYCODE_H:
          cmd.linear.z = - vertical_vel;
          break;

          // Running
        case KEYCODE_W_CAP:
          cmd.linear.x = run_vel;
          break;
        case KEYCODE_S_CAP:
          cmd.linear.x = - run_vel;
          break;
        case KEYCODE_A_CAP:
          cmd.linear.y = run_vel;
          break;
        case KEYCODE_D_CAP:
          cmd.linear.y = - run_vel;
          break;
        case KEYCODE_Q_CAP:
          cmd.angular.z = yaw_rate_run;
          break;
        case KEYCODE_E_CAP:
          cmd.angular.z = - yaw_rate_run;
          break;
        defualt:
          break;
        }

      vel_pub_.publish(cmd);
    }
}
