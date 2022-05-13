# ral_22_sofa_magnetic

# SOFA scenes for magnetic manipulation

## Description

Collection of SOFA scenes for magnetic manipulation. [SOFA](https://www.sofa-framework.org) (Simulation Open Framework Architecture) is an open-source framework primarily targeted at real-time simulation, with an emphasis on medical simulation

<table border = "0">
    <tr>
        <td>
        <img src="images/exple_simu_aortic.png" alt="drawing" height="200"/>
        </td>
        <td>
        <img src="images/exple_simu_flat.png" alt="drawing" height="200"/>
        </td>
    </tr>
</table>


## Installation

This requires Ubuntu 20.04 with ROS Noetic and Python 3.

### Install and build SOFA

Follow the instructions [here](https://www.sofa-framework.org/community/doc/getting-started/build/linux/) to build SOFA with Linux (tested for v21.06).

Build SOFA and include the following external plugins using [these instructions](https://www.sofa-framework.org/community/doc/plugins/build-a-plugin-from-sources/):
* [BeamAdapter](https://github.com/sofa-framework/BeamAdapter)
* [SoftRobots](https://project.inria.fr/softrobot/)
* [STLIB](https://github.com/SofaDefrost/STLIB)

In CMake GUI, make sure to configure the path to the external plugin, and check the following boxes:
* PLUGIN_BEAMADAPTER 
* PLUGIN_SOFAIMPLICITFIELD
* PLUGIN_SOFADISTANCEGRID
* PLUGIN_PLUGINEXAMPLE
* PLUGIN_STLIB
* PLUGIN_SOFTROBOTS
* SOFA_FETCH_SOFAPYTHON3 (make sure to follow [these guidelines](https://sofapython3.readthedocs.io/en/latest/menu/Compilation.html) for a proper installation of SofaPython3, and make sure [you load SofaPython3 via the plugin manager of runSofa](https://sofapython3.readthedocs.io/en/latest/menu/SofaPlugin.html#within-runsofa))

Make sure the box for PLUGIN_SOFAPYTHON is unchecked.

### Install the dependencies

This repository has the following dependencies:
* [Tesla](https://github.com/ethz-msrl/Tesla.git): collection of tools for performing magnetic manipulation.
* [Navion](https://github.com/ethz-msrl/Navion): packages for the control of the Navion electromagnetic navigation system.

These are interfaced using the ROS framework.

For installing ROS, *Tesla* and *Navion*, follow the instructions:
* ROS and Tesla: https://github.com/ethz-msrl/Tesla/wiki/Installation
* Navion: https://github.com/ethz-msrl/Navion

The next steps will assume that you have ROS and these dependencies installed, and a properly configured catkin workspace as described in the [Tesla instructions](https://github.com/ethz-msrl/Tesla/wiki/Installation). Scenes that do not depend on ROS can still be used without these dependencies (see the section on how to run scenes with the SOFA user interface).

### Clone repository

Clone the repository
```
cd ~/<workspace_name>/src 
git clone https://github.com/ethz-msrl/SOFA_magnetic.git
```

### ROS-SOFA linker package

Build the ROS package
```
cd ~/<workspace_name>
catkin build sofa_sim
source devel/setup.bash
```

## Run simulation using shell commands and SOFA user interface

* Open a terminal and go to the `/bin` directory of SOFA

```
./runSofa
```

* File/Open to open the scene (extentions *.scn or *.pyscn)
* Press "Animate" to start the scene

More information on how to use SOFA [here](https://www.sofa-framework.org/community/doc/).

## Run simulations using ROS

The ROS package [*sofa_sim*](/sofa_sim) provides a link between SOFA and ROS. This is required to use functionnalities from Tesla for the simulation of magnetic systems. The use of the package and related SOFA scenes is documented [here](/sofa_sim/README.md). 


## Tips and troubleshooting

### Code editor

We suggest to use **Visual Studio Code**. The native scene formats from SOFA \*.pyscn and \*.scn are not recognized by default. To provide the full functionnality of VSCode for these exentions:
* `File/Preferences/Settings`
* Search `files.association`
* Under *JSON: Schemas Associate schemas to JSON files in the current project*, clic *Edit in settings.json*
* Add the following rules for the files associations in the file:

```yaml
    "files.associations": {
        "*.scn": "xml",
        "*.pyscn": "python"
    }
```

Contact:
Roland Dreyfus: dreyfusr@ethz.ch
Quentin Boehler: qboehler@ethz.ch
