#!/usr/bin/env python3

import rf_msgs.msg
import rospy
import sensor_msgs
import rf_msgs
import sensor_msgs.msg


class Synchronizer:

    def __init__(self) -> None:
        self.wifi_sub = rospy.Subscriber("/csi", rf_msgs.msg.Wifi, self.wifi)
        self.imu_sub = rospy.Subscriber("/imu", sensor_msgs.msg.Imu, self.imu)
        self.gps_sub = rospy.Subscriber('/gps', sensor_msgs.msg.NavSatFix, self.gps)

        self.wifi_pub = rospy.Publisher("/sync/csi", rf_msgs.msg.Wifi, queue_size=10)
        self.imu_pub = rospy.Publisher("/sync/imu", sensor_msgs.msg.Imu, queue_size=10)
        self.gps_pub = rospy.Publisher("/sync/gps", sensor_msgs.msg.NavSatFix, queue_size=10)

        self.wifi_msg = None
        self.imu_msg = None
        self.gps_msg = None
    
    def wifi(self, msg):
        rospy.loginfo("Synchronized Data")
        self.wifi_msg = msg
        self.wifi_pub.publish(msg)
        self.imu_pub.publish(self.imu_msg)
        self.gps_pub.publish(self.gps_msg)
        return 
    
    def gps(self, msg):
       self.gps_msg = msg
       return
    
    def imu(self, msg):
        self.imu_msg = msg
        return

if __name__=="__main__":
    rospy.init_node('synchronizer_node', anonymous=True)
    synchronizer = Synchronizer()
    rospy.spin()


