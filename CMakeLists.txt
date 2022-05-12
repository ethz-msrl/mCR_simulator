cmake_minimum_required(VERSION 3.0.2)
project(sofa_valid)

find_package(catkin_simple REQUIRED)

catkin_python_setup()

catkin_simple()

catkin_install_python(PROGRAMS 
    # scripts/example.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

cs_export()