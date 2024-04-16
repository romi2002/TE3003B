#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

# Setup Variables to be used
tau = 0
def tau_cb(msg):
    global tau
    tau = msg.data

#wrap to pi function
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("SLM_Sim")

    #Get Parameters   
    k = 0.01
    m = 0.75
    l = 0.36
    a = l / 2
    g = 9.8
    tau = 0.0
    J = (4/3) * m * a * a

    x1 = 0.0
    x2 = 0.0
    x2_dot = 0.0
    dt = 1.0 / 100.0

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate", 100))

    joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rospy.Subscriber("tau", Float32, tau_cb)

    print("The SLM sim is Running")
    try:
        while not rospy.is_shutdown():
            x1 += x2 * dt
            x1 = wrap_to_Pi(x1)
            x2_dot = (1 / J) * (tau - m * g * a * np.cos(x1) - k * x2)
            x2 += x2_dot * dt

            # Publish joint states.
            state_msg = JointState()
            state_msg.header.stamp = rospy.Time.now()
            state_msg.name = ['joint2']
            state_msg.position = [x1]
            state_msg.velocity = [x2]
            joint_state_pub.publish(state_msg)

            #Wait and repeat
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node