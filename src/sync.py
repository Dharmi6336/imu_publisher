#!/usr/bin/env python3

import rf_msgs.msg
import rospy
import sensor_msgs
import rf_msgs
import sensor_msgs.msg


class Synchronizer:

    def __init__(self) -> None:
        self.wifi_sub = rospy.Subscriber("/csi", rf_msgs.msg.Wifi, self.wifi)
        #self.ap_sub = rospy.Subscriber("/csi_server/PUBLISHTOPIC", rf_msgs.msg.AccessPoints, self.ap)
        self.imu_sub = rospy.Subscriber("/imu/data", sensor_msgs.msg.Imu, self.imu)

        self.wifi_pub = rospy.Publisher("/sync/csi", rf_msgs.msg.Wifi)
        #self.ap_pub = rospy.Publisher("/sync/csi_server/PUBLISHTOPIC", rf_msgs.msg.AccessPoints)
        self.imu_pub = rospy.Publisher("/sync/imu", sensor_msgs.msg.Imu)

        self.wifi_msg = None
        #self.ap_msg = None
        self.imu_msg = None
    
    def wifi(self, msg):
        rospy.loginfo("Received Data")
        self.wifi_msg = msg
        self.wifi_pub.publish(msg)
        #self.ap_pub.publish(self.ap_msg)
        self.imu_pub.publish(self.imu_msg)
        return 
    
    #def ap(self, msg):
    #    self.ap_msg = msg
    #    return
    
    def imu(self, msg):
        self.imu_msg = msg
        return


if __name__=="__main__":
    rospy.init_node('synchronizer_node', anonymous=True)
    synchronizer = Synchronizer()
    rospy.spin()


