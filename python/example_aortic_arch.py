from splib3.numerics import Quat, Vec3
from scipy.spatial.transform import Rotation as R

import sys
print(sys.path)

from mcr_sim import \
    mcr_environment, mcr_instrument, mcr_emns, mcr_simulator, \
    mcr_controller_sofa, mcr_magnet

# Calibration file for eMNS
cal_path = '../calib/Navion_2_Calibration_24-02-2020.yaml'

# Parameters instrument
young_modulus_body = 170e6  # (Pa)
young_modulus_tip = 21e6    # (Pa)
length_body = 0.5           # (m)
length_tip = 0.034          # (m)
outer_diam = 0.00133        # (m)
inner_diam = 0.0008         # (m)

length_init = .35

# Parameters environment
environment_stl = '../mesh/anatomies/J2-Naviworks.stl'

# Parameter magnet
magnet_length = 4e-3        # (m)
magnet_id = 0.86e-3         # (m)
magnet_od = 1.33e-3         # (m)
magnet_remanence = 1.45     # (T)

# Parameter for beams
nume_nodes_viz = 600
num_elem_body = 30
num_elem_tip = 3

# Transforms
# Sofa sim frame in Navion

# Transforms (translation , quat)
T_sim_mns = [
    0., 0., 0.,
    0., 0., 0., 1]

# Environment in sofa sim frame
rot_env_sim = [-0.7071068, 0, 0, 0.7071068]
transl_env_sim = [0., -0.45, 0.]

T_env_sim = [
    transl_env_sim[0],
    transl_env_sim[1],
    transl_env_sim[2],
    -0.7071068, 0, 0, 0.7071068]

# Starting pose in environment frame
T_start_env = [-.075, -.001, -.020, 0., -0.3826834, 0., 0.9238795]

trans_start_env = Vec3(
    T_start_env[0],
    T_start_env[1],
    T_start_env[2])
r = R.from_quat(rot_env_sim)
trans_start_env = r.apply(trans_start_env)

quat_start = Quat(rot_env_sim)
qrot = Quat(
    T_start_env[3],  T_start_env[4], T_start_env[5], T_start_env[6])
quat_start.rotateFromQuat(qrot)

# Starting pose sofa_sim frame
T_start_sim = [
    trans_start_env[0]+transl_env_sim[0],
    trans_start_env[1]+transl_env_sim[1],
    trans_start_env[2]+transl_env_sim[2],
    quat_start[0], quat_start[1], quat_start[2], quat_start[3]]


def createScene(root_node):
    ''' Build SOFA scene '''

    # simulator
    simulator = mcr_simulator.Simulator(
        root_node=root_node)

    # eMNS
    navion = mcr_emns.EMNS(
        name='Navion',
        calibration_path=cal_path)

    # environment
    environment = mcr_environment.Environment(
        root_node=root_node,
        environment_stl=environment_stl,
        name='aortic_arch',
        T_env_sim=T_env_sim,
        flip_normals=True,
        color=[1., 0., 0., 0.3])

    # magnet
    magnet = mcr_magnet.Magnet(
           length=magnet_length,
           outer_diam=magnet_od,
           inner_diam=magnet_id,
           remanence=magnet_remanence)

    # magnets on both ends of flexible segment
    magnets = [0. for i in range(num_elem_tip)]
    magnets[0] = magnet
    magnets[1] = magnet

    # instrument
    instrument = mcr_instrument.Instrument(
        name='mcr',
        root_node=root_node,
        length_body=length_body,
        length_tip=length_tip,
        outer_diam=outer_diam,
        inner_diam=inner_diam,
        young_modulus_body=young_modulus_body,
        young_modulus_tip=young_modulus_tip,
        magnets=magnets,
        num_elem_body=num_elem_body,
        num_elem_tip=num_elem_tip,
        nume_nodes_viz=nume_nodes_viz,
        T_start_sim=T_start_sim,
        color=[.2, .8, 1., 1.]
        )

    # sofa-based controller
    controller_sofa = mcr_controller_sofa.ControllerSofa(
        root_node=root_node,
        e_mns=navion,
        instrument=instrument,
        length_init=length_init,
        T_sim_mns=T_sim_mns,
    )
    root_node.addObject(controller_sofa)
