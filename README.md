# Motion Planner Framework

***
## Prerequisites

You should have MoveIt2 source code available and set up the compilation environment before run below commands.

## Compile the source code

For planner_client, simply run below:

```
colcon build --packages-select motion_planning_hao
```

For planner_library, simply run below:

```
colcon build
```

After the planner library is generated, go to the root directory of moveit, and run:
```
source $(directory of planner_library)/install/local_setup.sh
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 4
```
To successfully run the above command, you need to add the dependency into CMakeLists.txt and package.xml for the corresponding package which rely on this library.


## Run the demo

First, run the servre application.

```
source install/setup.sh
ros2 launch moveit2_tutorials move_group.launch.py
```

Then, run the client application.

```
source $(directory of moveit_ws)/install/setup.sh
source ./install/local_setup.sh
ros2 launch motion_planning_hao motion_planning_hao.launch.py
```

For the detailed setup, please refer to [here](https://luhao2021.github.io/comp8604/).

## Authors and acknowledgment

This is developed by Hao Lu <hao.lu@anu.edu.au> for the COMP8604 research project which is supervised by Dr. Rahul Shome.

## License

Software License Agreement (BSD License)


Copyright (c) 2011, Rice University
Copyright (c) 2023, Australian National University
All rights reserved.
 
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
 
* Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above
 copyright notice, this list of conditions and the following
 disclaimer in the documentation and/or other materials provided
 with the distribution.
* Neither the name of the Rice University and the Australian
 National University nor the names of its contributors may be
 used to endorse or promote products derived from this software
 without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

## Project status

This project is on-going, and will be extended to support more usecases.
