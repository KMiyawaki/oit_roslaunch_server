#!/usr/bin/env python
import os
from threading import Lock
import roslaunch
import rospy
import actionlib
from oit_roslaunch_server.msg import LaunchFromFileAction, LaunchFromFileResult


class OITRosLaunchServer(object):
    def __init__(self):
        self.launch_from_file_server = actionlib.SimpleActionServer(
            '~launch_from_file', LaunchFromFileAction, None, False)
        self.launch_from_file_server.start()
        self.launch_files = {}
        self.node_name = rospy.get_name()

    def launch_core(self, path):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [path])
        launch.start()
        return launch

    def launch(self, path):
        result = LaunchFromFileResult()
        result.succeeded = False
        if self.launch_files.has_key(path):
            result.message = path + " is already launched."
            return (None, result)

        try:
            launch_instance = self.launch_core(path)
            result.succeeded = True
            return (launch_instance, result)
        except Exception as e:
            result.message = str(e)
            return (None, result)

    def terminlate(self, path):
        result = LaunchFromFileResult()
        if self.launch_files.has_key(path) == False:
            result.succeeded = False
            result.message = path + " is not launched."
            return result
        try:
            self.launch_files[path].shutdown()
            result.succeeded = True
        except Exception as e:
            result.succeeded = False
            result.message = str(e)
        return result

    def show_goal(self, goal):
        rospy.loginfo("%s: Accept request. Type = %s, path = %s",
                      self.node_name, type(goal).__name__,  str(goal.path))

    def spin(self):
        if self.launch_from_file_server.is_new_goal_available() == False:
            return

        goal = self.launch_from_file_server.accept_new_goal()
        self.show_goal(goal)
        if goal.need_to_launch:
            (launch_instance, result) = self.launch(goal.path)
            if launch_instance:
                self.launch_files[goal.path] = launch_instance
                self.launch_from_file_server.set_succeeded(result)
                rospy.loginfo("%s: launched %s", self.node_name, goal.path)
            else:
                rospy.logerr("%s: %s", self.node_name, result.message)
                self.launch_from_file_server.set_aborted(result)
        else:
            result = self.terminlate(goal.path)
            if result.succeeded:
                self.launch_from_file_server.set_succeeded(result)
                rospy.loginfo("%s: terminalted %s", self.node_name, goal.path)
            else:
                rospy.logerr("%s: %s", self.node_name, result.message)
                self.launch_from_file_server.set_aborted(result)



def main():
    script_name = os.path.basename(__file__)
    node_name = os.path.splitext(script_name)[0]
    rospy.init_node(node_name)
    node = OITRosLaunchServer()
    process_rate = rospy.get_param("~process_rate", 1.0)
    rate = rospy.Rate(process_rate)
    rospy.loginfo("Start %s with process rate %f Hz",
                  node_name, process_rate)
    while not rospy.is_shutdown():
        node.spin()
        rate.sleep()
    rospy.loginfo("Exiting %s", node_name)


if __name__ == "__main__":
    main()
