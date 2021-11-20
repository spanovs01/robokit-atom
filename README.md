# robokit-atom
Workspace for Robokit Atom robot. 

## Installation
 
 ```bash
    ./install_deps.sh
 ```

## Building

```bash 
    catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
    source devel/setup.sh
```

## Launching

To launch test vision script:

```bash
 roslaunch vision vision_test.launch
```
