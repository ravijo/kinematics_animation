#!/usr/bin/python

import os
import rospy
import numpy as np
from sensor_msgs.msg import JointState


class KinematicsAnimator(object):
    def __init__(self):
        freq = rospy.get_param('~freq', 50)
        rospy.loginfo('Publishing frequency = %f' % freq)

        fname = rospy.get_param('~file', None)
        if fname is None:
            rospy.logerr("Must provide private param '~file'")
            rospy.signal_shutdown('No CSV file provided')
        else:
            # Find out the absolute path if the input path is relative
            if not os.path.isabs(fname):
                fname = os.path.abspath(fname)
            rospy.loginfo('CSV file absolute path = %s' % fname)

        # load data from csv file
        self.data, file_header = self.read_csv(fname)
        joint_info = {idx: name for idx, name in enumerate(file_header)
                      if name.startswith('left_')
                      or name.startswith('right_')}

        joint_ids = joint_info.keys()
        self.data = self.data[:, joint_ids]
        self.joint_names = joint_info.values()

        self.row_number = 0
        self.base_time = rospy.Time.now()

        # create a joint_states publisher
        self.state_pub = rospy.Publisher('joint_states', JointState,
                                         latch=True, queue_size=3)

        # create a timer
        self.pbtimer = rospy.Timer(rospy.Duration(1 / float(freq)),
                                   self.timercb)

    def read_csv(self, file_name):
        # load the data files
        data = np.genfromtxt(file_name, delimiter=',', names=True)
        # get single header string
        header = data.dtype.names
        # convert structured array to numpy array
        data = data.view(np.float).reshape(data.shape + (-1,))
        return data, header

    def timercb(self, time_dat):
        t = (rospy.Time.now() - self.base_time).to_sec()
        q = self.data[self.row_number]
        self.row_number += 1
        if self.row_number >= self.data.shape[0]:
            rospy.loginfo('Animation complete! Replaying it.')
            self.row_number = 0
            self.base_time = rospy.Time.now()

        '''
        ' Provide all the joint names
        ' We are assuming here that the input csv file doesn't have head and finger information
        '''
        joint_names = ['head_pan', 'l_gripper_l_finger_joint',
                       'l_gripper_r_finger_joint',
                       'r_gripper_l_finger_joint',
                       'r_gripper_r_finger_joint'] + self.joint_names

        # Setting the head and finger joints to 0
        joint_position = [0] * 5 + q.tolist()
        js = JointState(name=joint_names, position=joint_position)
        js.header.stamp = rospy.Time.now()
        self.state_pub.publish(js)
        return


def main():
    rospy.init_node('animating_csv_files', log_level=rospy.INFO)

    try:
        animator = KinematicsAnimator()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()


if __name__ == '__main__':
    main()
