# A Simulation Framework for Magnetic Continuum Robots
 Roland Dreyfus, Quentin Boehler, Bradley J. Nelson

 ## Multimedia materials and source code

 ## Description
Magnetic continuum robot (m-CR) simulator is an object based Python implementation of SOFA to simulate magnetically actuated continuum robots. The package interfaces magnetic field models with the SOFA simulated mechanical model of the continuum robot. [SOFA](https://www.sofa-framework.org) (Simulation Open Framework Architecture) is an open-source framework primarily targeted at real-time simulation, with an emphasis on medical simulation.

 <table border = "0">
     <tr>
         <td>
         <img src="images/anatomy.png" alt="drawing" height="200"/>
         </td>
         <td>
         <img src="images/planar_model.png" alt="drawing" height="200"/>
         </td>
     </tr>
 </table>


 ## Installation

 This requires Ubuntu 20.04 with and Python 3.

 ### Install and build SOFA

 Follow the instructions [here](https://www.sofa-framework.org/community/doc/getting-started/build/linux/) to build SOFA with Linux.

 This code was tested with v.21.12 for SOFA and the external plugins.

 Build SOFA and include the following external plugins using [these instructions](https://www.sofa-framework.org/community/doc/plugins/build-a-plugin-from-sources/):
 * [BeamAdapter](https://github.com/sofa-framework/BeamAdapter)
 * [SoftRobots](https://project.inria.fr/softrobot/)
 * [STLIB](https://github.com/SofaDefrost/STLIB)

 Make sure to use the same version for SOFA and the plugins (these instructions were tested with v21.12), and to load these plugins in the plugin manager of runSofa.

 In CMake GUI, make sure to configure the path to the external plugins in the CMake variable ```SOFA_EXTERNAL_DIRECTORIES```, and check the following boxes:
 * ```PLUGIN_BEAMADAPTER```
 * ```PLUGIN_SOFAIMPLICITFIELD```
 * ```PLUGIN_SOFADISTANCEGRID```
 * ```PLUGIN_PLUGINEXAMPLE```
 * ```PLUGIN_STLIB```
 * ```PLUGIN_SOFTROBOTS```
 * ```SOFA_FETCH_SOFAPYTHON3``` (make sure to follow [these guidelines](https://sofapython3.readthedocs.io/en/latest/menu/Compilation.html) for a proper installation of SofaPython3, and make sure [you load SofaPython3 via the plugin manager of runSofa](https://sofapython3.readthedocs.io/en/latest/menu/SofaPlugin.html#within-runsofa))

 Make sure the box for ```PLUGIN_SOFAPYTHON``` is unchecked.

 ### Install the dependencies

 This repository has the following dependencies:
 * [mag_manip](https://pypi.org/project/mag-manip/): toolbox for electromagnetic navigation systems (eMNS) modelling.

 For installing mag_manip using pip:
 ```
 pip install mag-manip
 ```
 
 ## Usage simulator
 ### Run simulation using shell commands and SOFA user interface

 * Open a terminal and go to the `/bin` directory of SOFA

 ```
 ./runSofa
 ```

 * File/Open to open the scene (extentions *.scn or *.pyscn)
 * Press "Animate" to start the scene

 More information on how to use SOFA [here](https://www.sofa-framework.org/community/doc/).

 ### Manual navigation using keyboard commands. 

 * **Insertion/retraction**
     * CTRL + up/down: insertion/retraction of the m-CR

 * **Magnetic field**
     * CTRL + I/K: inclination angle
     * CTRL + J/L: azimuth angle
 
### Usage python scripts
An example of how to build a m-CR simulation scene, can be found in 
* [example_aortic_arch.py](python/example_aortic_arch.py)
* [example_flat.py](python/example_flat.py).


### Example with step by step explanation
Bellow is a step by step explanation of how to build a m-CR simulation scene. The explanation is based on [example_aortic_arch.py](python/example_aortic_arch.py).

Import the mcr_simulation modules
```python
from mcr_simulation import \
    mcr_environment, mcr_instrument, mcr_emns, mcr_simulator, \
    mcr_controller_sofa, mcr_magnet
```

Load or define the simulation parameters:
```python
# Calibration file for eMNS
cal_path = '../../calib/Navion_2_Calibration_24-02-2020.yaml'

# Parameters instrument
young_modulus_body = 170e6  # (Pa)
young_modulus_tip = 21e6    # (Pa)
length_body = 0.5           # (m)
length_tip = 0.034          # (m)
outer_diam = 0.00133        # (m)
inner_diam = 0.0008         # (m)

length_init = .35

# Parameters environment
environment_stl = '../../mesh/J2-Naviworks.stl'

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
# Sofa sim frame in eMNS

# Transforms (translation , quat)
T_sim_mns = [
    0., 0., 0.,
    0., 0., 0., 1]

# Environment in sofa sim frame
T_env_sim = [
    0., -0.45, 0.,
    -0.7071068, 0, 0, 0.7071068]

# Starting pose sofa_sim frame
T_start_sim = [
  -.075, -.001, -.020, 0.,
  -0.3826834, 0., 0.9238795]
```

In order to build an m-CR simulation, start by creating the SOFA scene:
```python
def createScene(root_node):
    ''' Build SOFA scene '''
```

Add the simulation physics and solver. The mcr_simulator object defines the solver settings and adds gravity, friction, constraints, etc.
```python
    # simulator
    simulator = mcr_simulator.Simulator(
        root_node=root_node)
```

Define the electromagnetic navigation system. The mcr_emns class uses [mag_manip](https://pypi.org/project/mag-manip/) to model the magnetic field and provides methodes to compute currents from magnetic fields (backwards model), and magnetic fields from currents (forward model).
```python
    # eMNS
    navion = mcr_emns.EMNS(
        name='Navion',
        calibration_path=cal_path)
```

Add the environment. The mcr_environment objects adds the mechanical,  collision and visual models of the envirnoment (e.g. anatomical model or collision objects) to the SOFA scene.
```python
    # environment
    environment = mcr_environment.Environment(
        root_node=root_node,
        environment_stl=environment_stl,
        name='aortic_arch',
        T_env_sim=T_env_sim,
        flip_normals=True,
        color=[1., 0., 0., 0.3])
```

Define the magnets that are embeded in the m-CR body. A magnet object can be created with the mcr_magnet class.
```python
    # magnet
    magnet = mcr_magnet.Magnet(
           length=magnet_length,
           outer_diam=magnet_od,
           inner_diam=magnet_id,
           remanence=magnet_remanence)
```

The magnetic elements of the instrument are defined by a list with the same length as the proximal section of the instrument num_elem_tip. The magnets should be stored at the index of the instrument element that is magnetic.
```python
    # magnets on both ends of flexible segment
    magnets = [0. for i in range(num_elem_tip)]
    magnets[0] = magnet
    magnets[1] = magnet
```

Create the magnetic instrument object. The mcr_instrument class adds the mechanical, collision and visual model to the SOFA scene. 
```python
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
```

Create the controller. The controller interfaces with the SOFA controller and with the magnetic field controller. On keyboard events, the desired magnetic field and insertion inputs are sent to the controllers.
```python
    # sofa-based controller
    controller_sofa = mcr_controller_sofa.ControllerSofa(
        root_node=root_node,
        e_mns=navion,
        instrument=instrument,
        length_init=length_init,
        T_sim_mns=T_sim_mns,
    )
    root_node.addObject(controller_sofa)
```

 ## Post-processing and figure generation
 
 ### Launch Jupyter notebook
 Data processing and figure generation was done in jupiter notebook.

 To launch Jupyter open a terminal, go to the directory `ral_22_sofa_magnetic`, and launch jupyter notebook. This will open a webbrowser window.

 ``` bash
 jupyter notebook notebook/ral_2022_mCR_data_viz.ipynb
 ```
 
 ## Tip tracker
 ### Run two-color tip tracker

 To run the tracker open a terminal, go to the directory `ral_22_sofa_magnetic`, and run the python script:

 ```
 python3 python/instrument_tracker.py -v video/MVI_9976.mp4
 ```


 Contact:
 Roland Dreyfus: dreyfusr@ethz.ch, 
 Quentin Boehler: qboehler@ethz.ch
