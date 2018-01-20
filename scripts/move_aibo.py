#!/usr/bin/env python
import rospy
import time
from math import pi, sin, cos, acos
import random
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
"""
Topics To Write on:
type: std_msgs/Float64
/aibo_tc/L_ear_joint_position_controller/command
/aibo_tc/L_ear_tilt_position_controller/command
/aibo_tc/R_ear_joint_position_controller/command
/aibo_tc/R_ear_tilt_position_controller/command
/aibo_tc/mouth_joint_position_controller/command
/aibo_tc/tailPan_position_controller/command
/aibo_tc/tailTilt_position_controller/command


/aibo_tc/legLB1_position_controller/command
/aibo_tc/legLB2_position_controller/command
/aibo_tc/legLB3_position_controller/command
/aibo_tc/legLF1_position_controller/command
/aibo_tc/legLF2_position_controller/command
/aibo_tc/legLF3_position_controller/command

/aibo_tc/legRB1_position_controller/command
/aibo_tc/legRB2_position_controller/command
/aibo_tc/legRB3_position_controller/command
/aibo_tc/legRF1_position_controller/command
/aibo_tc/legRF2_position_controller/command
/aibo_tc/legRF3_position_controller/command

/aibo_tc/headPan_position_controller/command
/aibo_tc/headTilt_position_controller/command
/aibo_tc/neck_joint_position_controller/command


"""

class AiboJointMover(object):

    def __init__(self):
        rospy.init_node('aibo_jointmover_demo', anonymous=True)
        rospy.loginfo("aibo JointMover Initialising...")

        # LEFT SIDE
        self.pub_legLB1_position = rospy.Publisher(
            '/aibo_tc/legLB1_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_legLB2_position = rospy.Publisher(
            '/aibo_tc/legLB2_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_legLB3_position = rospy.Publisher(
            '/aibo_tc/legLB3_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_legLF1_position = rospy.Publisher(
            '/aibo_tc/legLF1_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_legLF2_position = rospy.Publisher(
            '/aibo_tc/legLF2_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_legLF3_position = rospy.Publisher(
            '/aibo_tc/legLF3_position_controller/command',
            Float64,
            queue_size=1)

        # RIGHT SIDE
        self.pub_legRB1_position = rospy.Publisher(
            '/aibo_tc/legRB1_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_legRB2_position = rospy.Publisher(
            '/aibo_tc/legRB2_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_legRB3_position = rospy.Publisher(
            '/aibo_tc/legRB3_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_legRF1_position = rospy.Publisher(
            '/aibo_tc/legRF1_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_legRF2_position = rospy.Publisher(
            '/aibo_tc/legRF2_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_legRF3_position = rospy.Publisher(
            '/aibo_tc/legRF3_position_controller/command',
            Float64,
            queue_size=1)

        # Head
        self.pub_headPan_position = rospy.Publisher(
            '/aibo_tc/headPan_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_headTilt_position = rospy.Publisher(
            '/aibo_tc/headTilt_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_neck_joint_position = rospy.Publisher(
            '/aibo_tc/neck_joint_position_controller/command',
            Float64,
            queue_size=1)


        joint_states_topic_name = "/aibo_tc/joint_states"
        rospy.Subscriber(joint_states_topic_name, JointState, self.aibo_joints_callback)
        aibo_joints_data = None
        rate_try = rospy.Rate(1)
        while aibo_joints_data is None and not rospy.is_shutdown():
            try:
                aibo_joints_data = rospy.wait_for_message(joint_states_topic_name, JointState, timeout=1)
            except:
                rospy.logwarn("Time out " + str(joint_states_topic_name))
                rate_try.sleep()



        self.aibo_joint_dictionary = dict(zip(aibo_joints_data.name, aibo_joints_data.position))
        self.aibo_desired_joint_dictionary = dict(zip(aibo_joints_data.name, len(aibo_joints_data.name)*[0]))

        self.max_1 = 2.0
        self.middle_1 = 1.5
        self.small_1 = 0.7
        self.smallb_1 = 0.4
        self.smallc_1 = 0.3
        self.max_2 = 1.5
        self.middle_2 = 0.8
        self.smallb_2 = 0.3
        self.max_3 = 1.9
        self.middle_3 = 1.5
        self.small_3 = 0.7
        self.smallb_3 = 0.1

        self.fill_in_initpos1(joint_name_list=aibo_joints_data.name,
                              positions_list=aibo_joints_data.position)
        self.fill_in_initpos2(joint_name_list=aibo_joints_data.name,
                              positions_list=aibo_joints_data.position)
        self.fill_in_initpos3(joint_name_list=aibo_joints_data.name,
                              positions_list=aibo_joints_data.position)
        self.fill_in_initpos4(joint_name_list=aibo_joints_data.name,
                              positions_list=aibo_joints_data.position)
        self.fill_in_initpos5(joint_name_list=aibo_joints_data.name,
                              positions_list=aibo_joints_data.position)
        self.fill_in_initpos6(joint_name_list=aibo_joints_data.name,
                              positions_list=aibo_joints_data.position)
        self.fill_in_initpos7(joint_name_list=aibo_joints_data.name,
                              positions_list=aibo_joints_data.position)
        self.fill_in_initpos8(joint_name_list=aibo_joints_data.name,
                              positions_list=aibo_joints_data.position)
        self.fill_in_centerpos(joint_name_list=aibo_joints_data.name,
                              positions_list=aibo_joints_data.position)
        self.fill_in_walkinit_pos(joint_name_list=aibo_joints_data.name,
                              positions_list=aibo_joints_data.position)
        self.fill_in_walk_pos1(joint_name_list=aibo_joints_data.name,
                                  positions_list=aibo_joints_data.position)
        self.fill_in_walk_pos2(joint_name_list=aibo_joints_data.name,
                                  positions_list=aibo_joints_data.position)
        self.fill_in_walk_pos3(joint_name_list=aibo_joints_data.name,
                               positions_list=aibo_joints_data.position)
        self.fill_in_walk_pos4(joint_name_list=aibo_joints_data.name,
                               positions_list=aibo_joints_data.position)

    def fill_in_initpos1(self, joint_name_list, positions_list):
        """
        {'mouth_joint': 0,
        'tailPan': 0,
        'L_ear_joint': 0,
        'R_ear_tilt': 0,
        'tailTilt': 0,
        'headPan': 0,
        'headTilt': 0,
        'R_ear_joint': 0,
        L_ear_tilt': 0,
        'neck_joint': 0,
        'legRF2': 0,
        'legRF3': 0,
        'legRF1': 0,
        'legLF1': 0,
        'legLF2': 0,
        'legLF3': 0,
        'legLB1': 0,
        'legLB2': 0,
        'legLB3': 0,
        'legRB2': 0,
        'legRB3': 0,
        'legRB1': 0}
        :param joint_name_list:
        :param positions_list:
        :return:
        """
        self.INIT_POS_1 = dict(zip(joint_name_list, positions_list))


        init1_dict = {"legRF1": -self.max_1, "legRF2": 0, "legRF3": 0,
                      "legLF1": -self.max_1, "legLF2": 0, "legLF3": 0,
                      "legRB1": self.max_1, "legRB2": 0, "legRB3": 0,
                      "legLB1": self.max_1, "legLB2": 0, "legLB3": 0,
                      "headPan": -1.62, "headTilt": -0.87, "neck_joint": 0,
                      }
        for key, value in init1_dict.iteritems():
            self.INIT_POS_1[key] = value

    def fill_in_initpos2(self, joint_name_list, positions_list):

        self.INIT_POS_2 = dict(zip(joint_name_list, positions_list))

        init1_dict = {"legRF1": -self.max_1, "legRF2": self.middle_2, "legRF3": 0,
                      "legLF1": -self.max_1, "legLF2": self.middle_2, "legLF3": 0,
                      "legRB1": self.max_1, "legRB2": self.middle_2, "legRB3": 0,
                      "legLB1": self.max_1, "legLB2": self.middle_2, "legLB3": 0,
                      "headPan": 0, "headTilt": 0, "neck_joint": 0
                      }
        for key, value in init1_dict.iteritems():
            self.INIT_POS_2[key] = value

    def fill_in_initpos3(self, joint_name_list, positions_list):

        self.INIT_POS_3 = dict(zip(joint_name_list, positions_list))

        init1_dict = {"legRF1": -self.max_1, "legRF2": self.max_2, "legRF3": 0,
                      "legLF1": -self.max_1, "legLF2": self.max_2, "legLF3": 0,
                      "legRB1": self.max_1, "legRB2": self.max_2, "legRB3": 0,
                      "legLB1": self.max_1, "legLB2": self.max_2, "legLB3": 0,
                      "headPan": 0, "headTilt": 0, "neck_joint": 0
                      }
        for key, value in init1_dict.iteritems():
            self.INIT_POS_3[key] = value

    def fill_in_initpos4(self, joint_name_list, positions_list):

        self.INIT_POS_4 = dict(zip(joint_name_list, positions_list))

        init1_dict = {"legRF1": 0, "legRF2": self.max_2, "legRF3": 0,
                      "legLF1": 0, "legLF2": self.max_2, "legLF3": 0,
                      "legRB1": 0, "legRB2": self.max_2, "legRB3": 0,
                      "legLB1": 0, "legLB2": self.max_2, "legLB3": 0,
                      "headPan": 0, "headTilt": 0, "neck_joint": 0
                      }
        for key, value in init1_dict.iteritems():
            self.INIT_POS_4[key] = value

    def fill_in_initpos5(self, joint_name_list, positions_list):

        self.INIT_POS_5 = dict(zip(joint_name_list, positions_list))

        init1_dict = {"legRF1": -self.middle_1, "legRF2": 0, "legRF3": 0,
                      "legLF1": -self.middle_1, "legLF2": 0, "legLF3": 0,
                      "legRB1": -self.max_1, "legRB2": 0, "legRB3": self.max_3,
                      "legLB1": -self.max_1, "legLB2": 0, "legLB3": self.max_3,
                      "headPan": 0, "headTilt": 0, "neck_joint": 0
                      }
        for key, value in init1_dict.iteritems():
            self.INIT_POS_5[key] = value

    def fill_in_initpos6(self, joint_name_list, positions_list):

        self.INIT_POS_6 = dict(zip(joint_name_list, positions_list))

        init1_dict = {"legRF1": -self.middle_1, "legRF2": 0, "legRF3": 0,
                      "legLF1": -self.middle_1, "legLF2": 0, "legLF3": 0,
                      "legRB1": -self.middle_1, "legRB2": 0, "legRB3": self.middle_3,
                      "legLB1": -self.middle_1, "legLB2": 0, "legLB3": self.middle_3,
                      "headPan": 0, "headTilt": 0, "neck_joint": 0
                      }
        for key, value in init1_dict.iteritems():
            self.INIT_POS_6[key] = value

    def fill_in_initpos7(self, joint_name_list, positions_list):

        self.INIT_POS_7 = dict(zip(joint_name_list, positions_list))

        init1_dict = {"legRF1": -self.smallb_1, "legRF2": 0, "legRF3": 0,
                      "legLF1": -self.smallb_1, "legLF2": 0, "legLF3": 0,
                      "legRB1": -self.small_1, "legRB2": 0, "legRB3": self.middle_3,
                      "legLB1": -self.small_1, "legLB2": 0, "legLB3": self.middle_3,
                      "headPan": 0, "headTilt": 0, "neck_joint": 0
                      }
        for key, value in init1_dict.iteritems():
            self.INIT_POS_7[key] = value


    def fill_in_initpos8(self, joint_name_list, positions_list):

        self.INIT_POS_8 = dict(zip(joint_name_list, positions_list))

        init1_dict = {"legRF1": -self.smallb_1, "legRF2": 0, "legRF3": 0,
                      "legLF1": -self.smallb_1, "legLF2": 0, "legLF3": 0,
                      "legRB1": -self.smallb_1, "legRB2": 0, "legRB3": self.small_3,
                      "legLB1": -self.smallb_1, "legLB2": 0, "legLB3": self.small_3,
                      "headPan": 0, "headTilt": 0, "neck_joint": 0
                      }
        for key, value in init1_dict.iteritems():
            self.INIT_POS_8[key] = value

    def fill_in_centerpos(self, joint_name_list, positions_list):

        self.CENTER_POS = dict(zip(joint_name_list, positions_list))

        init1_dict = {"legRF1": 0, "legRF2": 0, "legRF3": 0,
                      "legLF1": 0, "legLF2": 0, "legLF3": 0,
                      "legRB1": 0, "legRB2": 0, "legRB3": 0,
                      "legLB1": 0, "legLB2": 0, "legLB3": 0,
                      "headPan": 0, "headTilt": 0, "neck_joint": 0
                      }
        for key, value in init1_dict.iteritems():
            self.CENTER_POS[key] = value

    def fill_in_walkinit_pos(self, joint_name_list, positions_list):

        self.WALK_INIT_POS_1 = dict(zip(joint_name_list, positions_list))

        init1_dict = {"legRF1": -self.smallc_1, "legRF2": 0, "legRF3": self.small_3,
                      "legLF1": -self.smallc_1, "legLF2": 0, "legLF3": self.small_3,
                      "legRB1": -self.smallc_1, "legRB2": 0, "legRB3": self.smallb_3,
                      "legLB1": -self.smallc_1, "legLB2": 0, "legLB3": self.smallb_3,
                      "headPan": 0, "headTilt": 0, "neck_joint": 0
                      }
        for key, value in init1_dict.iteritems():
            self.WALK_INIT_POS_1[key] = value

    def fill_in_walk_pos1(self, joint_name_list, positions_list):

        self.WALK_POS_1 = dict(zip(joint_name_list, positions_list))

        init1_dict = {"legRF1": -self.smallc_1, "legRF2": 0, "legRF3": -self.smallb_3,
                      "legLF1": -self.smallb_1, "legLF2": -self.smallb_2, "legLF3": -self.small_3,
                      "legRB1": -self.smallc_1, "legRB2": 0, "legRB3": self.smallb_3,
                      "legLB1": -self.smallc_1, "legLB2": 0, "legLB3": self.smallb_3,
                      "headPan": 0, "headTilt": 0, "neck_joint": 0
                      }
        for key, value in init1_dict.iteritems():
            self.WALK_POS_1[key] = value

    def fill_in_walk_pos2(self, joint_name_list, positions_list):

        self.WALK_POS_2 = dict(zip(joint_name_list, positions_list))

        init1_dict = {"legRF1": self.smallc_1, "legRF2": 0, "legRF3": -self.smallb_3,
                      "legLF1": -self.smallb_1, "legLF2": 0, "legLF3": self.smallb_3,
                      "legRB1": self.smallc_1, "legRB2": 0, "legRB3": self.smallb_3,
                      "legLB1": self.smallc_1, "legLB2": 0, "legLB3": self.smallb_3,
                      "headPan": 0, "headTilt": 0, "neck_joint": 0
                      }
        for key, value in init1_dict.iteritems():
            self.WALK_POS_2[key] = value

    def fill_in_walk_pos3(self, joint_name_list, positions_list):

        self.WALK_POS_3 = dict(zip(joint_name_list, positions_list))

        init1_dict = {"legRF1": self.smallb_1, "legRF2": 0, "legRF3": -self.smallb_3,
                      "legLF1": self.smallb_1, "legLF2": 0, "legLF3": self.smallb_3,
                      "legRB1": -self.small_1, "legRB2": 0, "legRB3": self.max_3,
                      "legLB1": self.smallb_1, "legLB2": 0, "legLB3": self.smallb_3,
                      "headPan": 0, "headTilt": 0, "neck_joint": 0
                      }
        for key, value in init1_dict.iteritems():
            self.WALK_POS_3[key] = value

    def fill_in_walk_pos4(self, joint_name_list, positions_list):

            self.WALK_POS_4 = dict(zip(joint_name_list, positions_list))

            init1_dict = {"legRF1": self.small_1, "legRF2": 0, "legRF3": -self.smallb_3,
                          "legLF1": self.small_1, "legLF2": 0, "legLF3": self.smallb_3,
                          "legRB1": -self.smallc_1, "legRB2": 0, "legRB3": self.middle_3,
                          "legLB1": self.smallb_1, "legLB2": 0, "legLB3": self.smallb_3,
                          "headPan": 0, "headTilt": 0, "neck_joint": 0
                          }
            for key, value in init1_dict.iteritems():
                self.WALK_POS_4[key] = value


    ##########################################


    def fill_in_turnleft_pos1(self, joint_name_list, positions_list):

        self.WALK_POS_1 = dict(zip(joint_name_list, positions_list))

        init1_dict = {"legRF1": -self.smallc_1, "legRF2": self.smallb_2, "legRF3": -self.smallb_3,
                      "legLF1": -self.smallb_1, "legLF2": self.smallb_2, "legLF3": -self.small_3,
                      "legRB1": -self.smallc_1, "legRB2": self.smallb_2, "legRB3": self.smallb_3,
                      "legLB1": -self.smallc_1, "legLB2": self.smallb_2, "legLB3": self.smallb_3,
                      "headPan": 0, "headTilt": 0, "neck_joint": 0
                      }
        for key, value in init1_dict.iteritems():
            self.WALK_POS_1[key] = value

    def fill_in_turnleft_pos2(self, joint_name_list, positions_list):

        self.WALK_POS_2 = dict(zip(joint_name_list, positions_list))

        init1_dict = {"legRF1": self.smallc_1, "legRF2": 0, "legRF3": -self.smallb_3,
                      "legLF1": -self.smallb_1, "legLF2": 0, "legLF3": -self.small_3,
                      "legRB1": self.smallc_1, "legRB2": 0, "legRB3": self.smallb_3,
                      "legLB1": self.smallc_1, "legLB2": 0, "legLB3": self.smallb_3,
                      "headPan": self.smallc_1, "headTilt": 0, "neck_joint": 0
                      }
        for key, value in init1_dict.iteritems():
            self.WALK_POS_2[key] = value

    def aibo_joints_callback(self, msg):
        """
        sensor_msgs/JointState
        std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
        string[] name
        float64[] position
        float64[] velocity
        float64[] effort

        :param msg:
        :return:
        """
        self.aibo_joint_dictionary = dict(zip(msg.name, msg.position))


    def convert_angle_to_unitary(self, angle):
        """
        Removes complete revolutions from angle and converts to positive equivalent
        if the angle is negative
        :param angle: Has to be in radians
        :return:
        """
        # Convert to angle between [0,360)
        complete_rev = 2 * pi
        mod_angle = int(angle / complete_rev)
        clean_angle = angle - mod_angle * complete_rev
        # Convert Negative angles to their corresponding positive values
        if clean_angle < 0:
            clean_angle += 2 * pi

        return clean_angle

    def assertAlmostEqualAngles(self, x, y,):
        c2 = (sin(x) - sin(y)) ** 2 + (cos(x) - cos(y)) ** 2
        angle_diff = acos((2.0 - c2) / 2.0)
        return angle_diff

    def aibo_check_continuous_joint_value(self, joint_name, value, error=0.1):
        """
        Check the joint by name 'base_waist_joint', 'body_head_joint', 'waist_body_joint is near the value given
        We have to convert the joint values removing whole revolutions and converting negative versions
        of the same angle
        :param joint_name:
        :param value:
        :param error: In radians
        :return:
        """
        joint_reading = self.aibo_joint_dictionary.get(joint_name)
        if not joint_reading:
            print "self.aibo_joint_dictionary="+str(self.aibo_joint_dictionary)
            print "joint_name===>"+str(joint_name)
            assert "There is no data about that joint"
        clean_joint_reading = self.convert_angle_to_unitary(angle=joint_reading)
        clean_value = self.convert_angle_to_unitary(angle=value)

        dif_angles = self.assertAlmostEqualAngles(clean_joint_reading, clean_value)
        similar = dif_angles <= error

        print "ACTUAL JOINT VALUE ="+str(joint_reading)
        print "DESIRED JOINT VALUE =" + str(value)
        print "ACTUAL JOINT clean="+str(clean_joint_reading)
        print "DESIRED JOINT clean="+str(clean_value)
        print "dif_angles="+str(dif_angles)
        print "similar=" + str(similar)

        return similar, dif_angles

    def move_aibo_leg_joints(self, position_dict):
        """
        evaluates for the moment only the legs
        init1_dict = {"legRF1": max_1, "legRF2": 0, "legRF3": 0,
                      "legLF1": max_1, "legLF2": 0, "legLF3": 0,
                      "legRB1": max_1, "legRB2": 0, "legRB3": 0,
                      "legLB1": max_1, "legLB2": 0, "legLB3": 0,
                      }
        :param position_dict:
        :return:
        """
        legLB1 = Float64()
        legLB1.data = position_dict["legLB1"]
        legLB2 = Float64()
        legLB2.data = position_dict["legLB2"]
        legLB3 = Float64()
        legLB3.data = position_dict["legLB3"]

        legLF1 = Float64()
        legLF1.data = position_dict["legLF1"]
        legLF2 = Float64()
        legLF2.data = position_dict["legLF2"]
        legLF3 = Float64()
        legLF3.data = position_dict["legLF3"]

        legRB1 = Float64()
        legRB1.data = position_dict["legRB1"]
        legRB2 = Float64()
        legRB2.data = position_dict["legRB2"]
        legRB3 = Float64()
        legRB3.data = position_dict["legRB3"]

        legRF1 = Float64()
        legRF1.data = position_dict["legRF1"]
        legRF2 = Float64()
        legRF2.data = position_dict["legRF2"]
        legRF3 = Float64()
        legRF3.data = position_dict["legRF3"]

        headPan = Float64()
        headPan.data = position_dict["headPan"]
        headTilt = Float64()
        headTilt.data = position_dict["headTilt"]
        neck_joint = Float64()
        neck_joint.data = position_dict["neck_joint"]

        self.pub_legLB1_position.publish(legLB1)
        self.pub_legLB2_position.publish(legLB2)
        self.pub_legLB3_position.publish(legLB3)
        self.pub_legLF1_position.publish(legLF1)
        self.pub_legLF2_position.publish(legLF2)
        self.pub_legLF3_position.publish(legLF3)

        # RIGHT SIDE
        self.pub_legRB1_position.publish(legRB1)
        self.pub_legRB2_position.publish(legRB2)
        self.pub_legRB3_position.publish(legRB3)
        self.pub_legRF1_position.publish(legRF1)
        self.pub_legRF2_position.publish(legRF2)
        self.pub_legRF3_position.publish(legRF3)

        # Head
        self.pub_headPan_position.publish(headPan)
        self.pub_headTilt_position.publish(headTilt)
        self.pub_neck_joint_position.publish(neck_joint)

    def generate_step_movement(self, movement_dictionary, angle_step):

        finished = True
        for key, value in movement_dictionary.iteritems():
            print "@@@   Joint ="+str(key)
            self.INIT_POS_7[key] = value
            """
            similar, dif_angles = self.aibo_check_continuous_joint_value(joint_name=key,
                                                                         value=value,
                                                                         error=0.1)
            """
            difference = value - self.aibo_joint_dictionary.get(key)
            print "des value = " + str(value)
            print "actual value = " + str(self.aibo_joint_dictionary.get(key))
            print "DIFFERENCE = "+str(difference)
            similar = abs(difference) <= 0.1
            if not similar:
                finished = False
                if difference >= 0:
                    print "DESIRED ANGLE IS BIGGER"
                    increment = angle_step
                else:
                    print "DESIRED ANGLE IS SMALLER"
                    increment = -angle_step
            else:
                print "SIMILAR NO INCREMENT"
                increment = 0

            final_value = self.aibo_joint_dictionary.get(key) + increment
            print "FINAL VALUE = "+str(final_value)
            self.aibo_desired_joint_dictionary[key] = final_value

        return self.aibo_desired_joint_dictionary, finished

    def aibo_movement_step(self, movement_dictionary, angle_step=0.1, step_frequency=10.0):
        """
        Move:
        :return:
        """
        rate_interval = rospy.Rate(step_frequency)
        movement_finished = False
        while not movement_finished:
            print "############ STEP ###############"
            step_movement, movement_finished = self.generate_step_movement(movement_dictionary, angle_step)
            if not movement_finished:
                self.move_aibo_leg_joints(step_movement)
            rate_interval.sleep()



    def aibo_init_pos_sequence(self):
        """
        UPPER limits lower="-1.55" upper="0.0"
        LOWER limits lower="-2.9" upper="1.5708"
        :return:
        """

        self.move_aibo_leg_joints(self.INIT_POS_1)
        print "INIT1 Waiting for 3 seconds.."
        time.sleep(3)

        self.move_aibo_leg_joints(self.INIT_POS_2)
        print "INIT2 Waiting for 3 seconds.."
        time.sleep(3)

        self.move_aibo_leg_joints(self.INIT_POS_3)
        print "INIT3 Waiting for 3 seconds.."
        time.sleep(3)

        self.move_aibo_leg_joints(self.INIT_POS_4)
        print "INIT4 Waiting for 3 seconds.."
        time.sleep(3)

        self.move_aibo_leg_joints(self.INIT_POS_5)
        print "INIT_POS_5 Waiting for 3 seconds.."
        time.sleep(3)
        self.move_aibo_leg_joints(self.INIT_POS_6)
        print "INIT_POS_6 Waiting for 3 seconds.."
        time.sleep(3)
        self.move_aibo_leg_joints(self.INIT_POS_7)
        print "INIT_POS_7 Waiting for 3 seconds.."
        time.sleep(3)
        self.move_aibo_leg_joints(self.INIT_POS_8)
        print "INIT_POS_8 Waiting for 3 seconds.."
        time.sleep(3)
        self.move_aibo_leg_joints(self.CENTER_POS)
        print "CENTER_POS Waiting for 3 seconds.."
        time.sleep(3)


    def aibo_walk_sequence(self):
        """
        UPPER limits lower="-1.55" upper="0.0"
        LOWER limits lower="-2.9" upper="1.5708"
        :return:
        """
        self.move_aibo_leg_joints(self.CENTER_POS)
        print "CENTER_POS Waiting for 3 seconds.."
        time.sleep(3)


        #self.move_aibo_leg_joints(self.WALK_INIT_POS_1)
        #print "INIT1 Waiting for 3 seconds.."
        #time.sleep(3)

        self.move_aibo_leg_joints(self.WALK_POS_1)
        print "INIT1 Waiting for 3 seconds.."
        time.sleep(3)

        self.move_aibo_leg_joints(self.WALK_POS_2)
        print "INIT1 Waiting for 3 seconds.."
        time.sleep(3)

        self.move_aibo_leg_joints(self.WALK_POS_3)
        print "INIT1 Waiting for 3 seconds.."
        time.sleep(3)

        self.move_aibo_leg_joints(self.WALK_POS_4)
        print "INIT1 Waiting for 3 seconds.."
        time.sleep(3)


    def movement_random_loop(self):
        """
        Executed movements in a random way
        :return:
        """
        rospy.loginfo("Start Moving aibo...")
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            r.sleep()
            print "Waiting..."
            self.aibo_walk_sequence()
    
    def aibo_init_pose(self):
        """
        Sets aibo in stable pose
        """
        rospy.loginfo("Starting Aibo...")
        self.move_aibo_leg_joints(self.CENTER_POS)
        print "CENTER_POS Waiting for 3 seconds.."
        time.sleep(3)

        self.move_aibo_leg_joints(self.CENTER_POS)
        print "CENTER_POS Waiting for 3 seconds.."
        time.sleep(3)
        rospy.loginfo("Aibo Ready...")

if __name__ == "__main__":
    aibo_jointmover_object = AiboJointMover()
    aibo_jointmover_object.movement_random_loop()

