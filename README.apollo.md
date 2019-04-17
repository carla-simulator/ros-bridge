# Use Carla Bridge within Apollo docker

## Prerequistes

Install libpng16

    git clone https://git.code.sf.net/p/libpng/code libpng-code
    cd libpng-code
    ./configure --prefix=/usr/
    sudo make install

Bring carla-python-api and carla-bridge into apollo directory

    cd <path-to-apollo-git>

    #previously extracted carla python api as described in carla autoware
    cp -r ~/carla-python .
  
    git clone -b feature/apollo https://github.com/carla-simulator/ros-bridge.git
    

## Execution

    export PYTHONPATH=$PYTHONPATH:/apollo/carla-python/carla/dist/carla-0.9.5-py2.7-linux-x86_64.egg:/apollo/carla-python/carla/:/apollo/ros-bridge/carla_ros_bridge/src/

    python -s /apollo/ros-bridge/carla_ros_bridge/src/carla_ros_bridge/client.py --binding /apollo/ros-bridge/carla_ros_bridge/src/carla_ros_bridge/binding/LogBinding.py 
