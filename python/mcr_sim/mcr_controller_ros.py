import Sofa
import rospy
import os
import tf
import numpy as np

from std_msgs.msg import Float32
from visualization_msgs.msg import MarkerArray
from mag_msgs.msg import FieldStamped

from sofa_beam_rviz_markers_utils import *
from mcr_sim import mcr_mag_controller


class ControllerRos(Sofa.Core.Controller):
    '''
    A class that interfaces with ROS and with the magnetic field controller.
    The class subscribes to magnetic field and insertion speed topics and
    and publishes Rviz marker to visualze the magnetic instrument.

    :param root_node: The sofa root node
    :param e_mns: The object defining the eMNS
    :param instrument: The object defining the instrument
    :param environment: The object defining the environment
    :param T_sim_mns: The transform defining the pose of the sofa_sim frame center in Navion frame [x, y, z, qx, qy, qz, qw]
    :type T_sim_mns: list[float]
    :param length_init: The initial insertion length of the instrument (m)
    :type length_init: float
    '''

    def __init__(
            self,
            root_node,
            e_mns,
            instrument,
            environment,
            T_sim_mns,
            length_init,
            *args, **kwargs):

        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        # communicate with ROS
        rospy.init_node('sofa_sim', anonymous=True, log_level=rospy.INFO)

        self.root_node = root_node
        self.instrument = instrument
        self.e_mns = e_mns
        self.environment = environment
        self.T_sim_mns = T_sim_mns

        self.length_init = length_init
        self.field_des = np.array([0., 0., 0.])
        self.speed_des = 0.
        self.speed = 0.
        self.instrument_length = 0.

        self.time = 0.
        self.time_simu = 0.
        self.dt = root_node.dt.value

        # ros publisher
        self.rod_pub = rospy.Publisher(
            '/rod_markers',
            MarkerArray,
            queue_size=1)
        self.pos_pub = rospy.Publisher(
            '/rod_sim_insertion_length',
            Float32, queue_size=1)

        # ros subscriber
        self.ros_field_sub = rospy.Subscriber(
            "/backward_model/field",
            FieldStamped,
            self.ros_field_cb)
        self.ros_speed_sub = rospy.Subscriber(
            "/navion/target_velocity_mca",
            Float32,
            self.ros_speed_cb)

        # send magnetic field inputs to mag controller to apply the torques
        self.mag_controller = mcr_mag_controller.MagController(
            root_node=self.root_node,
            e_mns=self.e_mns,
            instrument=instrument,
            T_sim_mns=T_sim_mns,
            )
        self.root_node.addObject(self.mag_controller)

    def ros_field_cb(self, msg):
        ''' Subscribe to ros topic and update magnetic field '''

        field_x = msg.field.vector.x
        field_y = msg.field.vector.y
        field_z = msg.field.vector.z

        self.field_des = np.array([field_x, field_y, field_z])

    def ros_speed_cb(self, msg):
        ''' update length '''

        self.speed_des = msg.data

    def broadcast_transform(self):
        ''' broadcast transform between the simulation frame and the mns '''

        br = tf.TransformBroadcaster()

        br.sendTransform((
            self.T_sim_mns[0],
            self.T_sim_mns[1],
            self.T_sim_mns[2]),
            (self.T_sim_mns[3],
             self.T_sim_mns[4],
             self.T_sim_mns[5],
             self.T_sim_mns[6]),
            rospy.Time.now(),
            "sofa_sim",
            "mns")

        br.sendTransform((
            self.environment.T_env_sim[0],
            self.environment.T_env_sim[1],
            self.environment.T_env_sim[2]),
            (self.environment.T_env_sim[3],
             self.environment.T_env_sim[4],
             self.environment.T_env_sim[5],
             self.environment.T_env_sim[6]),
            rospy.Time.now(),
            self.environment.name_env,
            "sofa_sim")

    def shut_down(self):
        ''' Shut down SOFA. '''

        rospy.loginfo('shutting down SOFA')
        os._exit(1)

    def onAnimateBeginEvent(self, event):
        '''
        Upddate instrument insertion speed and publish Rviz markers at the
        beginning of the simulation step.
        '''

        # Shut down SOFA when ROS is shut down.
        rospy.on_shutdown(self.shut_down)

        # Update time
        self.time_prev = self.time
        self.time = rospy.get_time()
        self.time_simu = self.time_simu + self.dt
        tot_time = 4.0

        # during the first two seconds,
        # we put the catheter at its initial lenght
        if self.time <= tot_time:
            instrument_length = self.time/tot_time * self.length_init
            self.instrument.IRC.xtip = [instrument_length]
            return

        self.mag_controller.field_des = self.field_des

        # update catheter speed
        self.speed = self.speed_des/1000
        self.instrument.IRC.speed = self.speed * (self.time-self.time_prev)/self.dt

        # Publish beam markers for rviz display
        rod_msg = getBeamMarkers(
            self.instrument.MO_visu,
            beam_rad=self.instrument.outer_diam/2.,
            beam_opacity=self.instrument.color[3],
            beam_color=self.instrument.color[0:3],
            magnet_rad=1.1*self.instrument.outer_diam/2.,
            magnet_opacity=1.,
            magnet_color=self.instrument.magnets[0].color[0:3],
            magnets_indices=self.instrument.index_mag_visu,
            magnet_length=1.e-3,
            t=rospy.get_time())
        self.rod_pub.publish(rod_msg)

        pos_msg = Float32()
        pos_msg.data = (self.instrument.IRC.xtip[:][0] - self.length_init) * 1e3
        self.pos_pub.publish(pos_msg)

        # Broadcast transforms
        self.broadcast_transform()
