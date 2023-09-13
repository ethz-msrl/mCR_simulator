import Sofa
import numpy as np

from mcr_sim import mcr_mag_controller
from scipy.spatial.transform import Rotation as R


class ControllerSofa(Sofa.Core.Controller):
    '''
    A class that interfaces with the SOFA controller and with the magnetic
    field controller.
    On keyboard events, the desired magnetic field and insertion inputs are
    sent to the controllers.

    :param root_node: The sofa root node
    :param e_mns: The object defining the eMNS
    :param instrument: The object defining the instrument
    :param environment: The object defining the environment
    :param T_sim_mns: The transform defining the pose of the sofa_sim frame center in Navion frame [x, y, z, qx, qy, qz, qw]
    :type T_sim_mns: list[float]
    :param T_sim_mns: The inital magnetic field direction and magnitude (T)
    :type T_sim_mns: ndarray
    '''

    def __init__(
            self,
            root_node,
            e_mns,
            instrument,
            T_sim_mns,
            mag_field_init=np.array([0.01, 0.01, 0.]),
            *args, **kwargs):

        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.root_node = root_node
        self.e_mns = e_mns
        self.instrument = instrument
        self.T_sim_mns = T_sim_mns
        self.mag_field_init = mag_field_init

        self.dfield_angle = 0.

        self.mag_controller = mcr_mag_controller.MagController(
            name='mag_controller',
            root_node=self.root_node,
            e_mns=self.e_mns,
            instrument=self.instrument,
            T_sim_mns=self.T_sim_mns,
            )
        self.root_node.addObject(self.mag_controller)

        self.mag_controller.field_des = self.mag_field_init

    def onKeypressedEvent(self, event):
        ''' Send magnetic field and insertion inputs when keys are pressed.'''

        # Increment field angle in rad
        dfield_angle = 3.*np.pi/180
        # speed = 0.01

        key = event['key']
        # J key : z-rotation +
        if ord(key) == 76:

            r = R.from_rotvec(-dfield_angle * np.array([0, 0, 1]))
            self.mag_controller.field_des = r.apply(
                self.mag_controller.field_des)

        # L key : z-rotation -
        if ord(key) == 74:

            r = R.from_rotvec(dfield_angle * np.array([0, 0, 1]))
            self.mag_controller.field_des = r.apply(
                self.mag_controller.field_des)

        # I key : x-rotation +
        if ord(key) == 73:

            r = R.from_rotvec(-dfield_angle * np.array([1, 0, 0]))
            self.mag_controller.field_des = r.apply(
                self.mag_controller.field_des)

        # K key : x-rotation -
        if ord(key) == 75:

            r = R.from_rotvec(dfield_angle * np.array([1, 0, 0]))
            self.mag_controller.field_des = r.apply(
                self.mag_controller.field_des)
