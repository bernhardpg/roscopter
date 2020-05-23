#!/usr/bin/env python3
# license removed for brevity

import rospy
from rosflight_msgs.msg import NoroboCommand
from rosflight_msgs.msg import Command
from nav_msgs.msg import Odometry
from constants import *
from helpers import *
from state import *
from lqrController import LQRController
from trajectory_optimization import TrajectoryOptimization

# Global variables
# State = [phi, theta, gamma, p, q, r, u, v ,w, x, y, z]
# phi = roll, theta = pitch, gamma = yaw
default_loop_rate = None
command_pub = None
current_state = None
K, S = None, None  # Used in LQR
run_mode = RunState.KTH_NED
# controller = TrajectoryOptimization(time_steps=30, dt=control_rate)
controller = LQRController()
current_control_progress = 0


def arm():
    arm_pub = rospy.Publisher('norobo_command', NoroboCommand, queue_size=10)
    while (arm_pub.get_num_connections() == 0):
        default_loop_rate.sleep()
    rospy.loginfo("Sending arm")
    msg = NoroboCommand()
    msg.arm = True
    arm_pub.publish(msg)


def send_command(x, y, z, f):
    msg = Command()
    msg.header.stamp = rospy.Time.now()
    msg.mode = Command.MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE
    msg.ignore = Command.IGNORE_NONE
    msg.x = x
    msg.y = y
    msg.z = z
    msg.F = f

    command_pub.publish(msg)


def mode_state_callback(data):
    global current_state

    inertial_rotation_roll_pitch_yaw = quaternion_to_euler(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    current_state.inertial_position = Position(data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
    current_state.inertial_angle = Angle(inertial_rotation_roll_pitch_yaw[0], inertial_rotation_roll_pitch_yaw[1], inertial_rotation_roll_pitch_yaw[2])
    current_state.body_velocity = Velocity(data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z)
    current_state.body_angle_velocity = AngleVelocity(data.twist.twist.angular.x, data.twist.twist.angular.y, data.twist.twist.angular.z)

    current_state.initiated = True


def get_desired_state():
    return np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1])


def control_loop(timer):
    global current_state, K, m, g, current_control_progress

    if not current_state.initiated:
        print("waiting on current state")
        return
    # if len(controller.getU()) == current_control_progress:
    #     print("done with executing plan")
    print("current state = ", current_state.inertial_position.z)
    u = controller.getU(current_state, get_desired_state())

    # u = controller.getU()[current_control_progress]
    # u_before = u[3]
    # u[3] = solveForGain(u[3]/4)
    # print("u(b) = {}, u(a) = {}".format(u_before, u[3]))

    current_control_progress += 1

    send_command(u[0], u[1], u[2], u[3])
import time
def talker():
    global command_pub, default_loop_rate, current_state, control_rate, K, S
    #
    rospy.init_node('NoroboController', anonymous=True)
    default_loop_rate = rospy.Rate(10)  # 10hz
    command_pub = rospy.Publisher('command', Command, queue_size=10)

    current_state = State()
    print("Running trajectory planning...")
    start_time = time.time()
    # controller.generate_path()

    print("arming")
    arm()

    rospy.Timer(rospy.Duration(control_rate), control_loop)
    rospy.Subscriber("multirotor/truth/NED", Odometry, mode_state_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
