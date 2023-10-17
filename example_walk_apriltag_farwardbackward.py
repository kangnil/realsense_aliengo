#!/usr/bin/python

import sys
import time
import math
import dt_apriltags

sys.path.append('../lib/python/amd64')
import robot_interface as sdk

# high cmd
TARGET_PORT = 8082
LOCAL_PORT = 8081
TARGET_IP = "192.168.123.220"   # target IP address

HIGH_CMD_LENGTH = 113
HIGH_STATE_LENGTH = 244

import rospy
from std_msgs.msg import String
import threading   


def callback(data):
    global command
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    command = data.data

def thread_job():
    rospy.spin()
 
 

def listener():
 
    rospy.init_node('listener')
    add_thread = threading.Thread(target = thread_job)
    add_thread.start()
    
    rospy.Subscriber('/chatter', String, callback, queue_size=10)
    rospy.sleep(1)
 
if __name__ == '__main__':
    global command
    command = ""
    listener()
    HIGHLEVEL = 0x00
    LOWLEVEL  = 0xff

    # udp = sdk.UDP(8080, "192.168.123.161", 8082, 129, 1087, False, sdk.RecvEnum.nonBlock)
    # udp = sdk.UDP(HIGHLEVEL, 8080, "192.168.123.161", 8082)

    cmd = sdk.HighCmd()
    state = sdk.HighState()

    

    
    udp = sdk.UDP(LOCAL_PORT, TARGET_IP, TARGET_PORT, HIGH_CMD_LENGTH, HIGH_STATE_LENGTH, -1)

    udp.InitCmdData(cmd)

    motiontime = 0
    while True:
        time.sleep(0.002)
        motiontime = motiontime + 1

        # print(motiontime)
        # print(state.imu.rpy[0])
        
        
        udp.Recv()
        udp.GetRecv(state)
        

        # print(motiontime, state.motorState[0].q, state.motorState[1].q, state.motorState[2].q)
        print(state.imu.rpy[2])

        cmd.mode = 0      # 0:idle, default stand      1:forced stand     2:walk continuously
        cmd.gaitType = 0
        cmd.speedLevel = 0
        cmd.dFootRaiseHeight = 0
        cmd.dBodyHeight = 0
        cmd.rpy = [0, 0, 0]
        cmd.velocity = [0, 0]
        cmd.yawSpeed = 0.0
        cmd.reserve = 0

        # cmd.mode = 2
        # cmd.gaitType = 1
        # # cmd.position = [1, 0]
        # # cmd.position[0] = 2
        # cmd.velocity = [-0.2, 0] # -1  ~ +1
        # cmd.yawSpeed = 0
        # cmd.bodyHeight = 0.1

        
        if(motiontime > 1000 and motiontime < 2000):
            cmd.mode = 6
            print("mode 6")
            
        
        if(motiontime > 2000 and motiontime < 3000):
            cmd.mode = 1
            print("mode 1")
        
        if(motiontime > 3000 and motiontime < 40000):
            if command=="forward":
                cmd.mode = 2
                cmd.gaitType = 2
                cmd.velocity = [0.1, 0] # -1  ~ +1
                cmd.yawSpeed = 0
                cmd.dFootRaiseHeight = 0.1
                print("walk farward")
            elif command =="backward":
                cmd.mode = 2
                cmd.gaitType = 2
                cmd.velocity = [-0.1, 0] # -1  ~ +1
                cmd.dBodyHeight = 0.1
                print("walk backward")
            else:
                cmd.mode = 0
                cmd.velocity = [0, 0]
                print("not walking, mode 0")
        
        if(motiontime > 40000 and motiontime < 41000):
            cmd.mode = 0
            cmd.velocity = [0, 0]
            print("mode 0")
        
     
         
        if(motiontime > 41000 and motiontime < 42000):
            cmd.mode = 5  
            print("sit down")

        udp.SetSend(cmd)
        udp.Send()
