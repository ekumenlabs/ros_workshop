## ROS Melodic guidelines

Base project docker image with ros melodic full distribution and a sample package.

### CI

CI relies on two Github Action packages that essentially configure the ROS
Melodic environment to build and test the packages. If extra dependencies are
required which cannot be handled by `rosdep` you must perform the custom
installation steps before the execution of `action-ros-ci`.

### Docker

#### Prerequisites

It is a requirement to have `docker engine` already installed in the host machine.

* See [Docker Installation Guide](https://docs.docker.com/engine/install/ubuntu/)

For NVIDIA GPU support, `nvidia-container-toolkit` should be installed. *Skip this step if you don't have an NVIDIA graphics card*


* Make sure you have the drivers installed:
  ```sh
  nvidia-smi
  ```
* See [NVIDIA Container Toolkit Installation Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

#### Building image and running container

- Build the docker image whose name is `ros_melodic`:

```sh
./docker/build.sh
```

You can also try to set a specific image name:

```sh
./docker/build.sh -i my_fancy_image_name
```

- Run a docker container from `ros_melodic` called `ros_melodic_container`:

```sh
./docker/run.sh
```

- **IMPORTANT**: If you are using nvidia drivers add the `--use_nvidia` flag:

```sh
./docker/run.sh --use_nvidia
```

You can also try to set specific image and container names:

```sh
./docker/run.sh -i my_fancy_image_name -c my_fancy_container_name
```

And a prompt in the docker image should appear at the root of the workspace:

```sh
$ pwd
/home/username/ws
$ ls -lash src/
total 12K
4.0K drwxr-xr-x 3 root         root         4.0K Nov 25 20:53 .
4.0K drwxr-xr-x 1 root         root         4.0K Nov 25 20:53 ..
4.0K drwxrwxr-x 4 agalbachicar agalbachicar 4.0K Nov 25 19:20 ekuabc
```

Note that the repository is mounted into a workspace. That is convenient if you
are working in a single repository project. Note that for multi-repository
workspace you should use another tool like vcs-tool to control via a `.repos`
file the repositories in your workspace.

### Prepare your workspace, build and test

- First of all, sync your submodules to bring the Noah bot repository running:

```sh
git submodule update --init --recursive
```

- To install dependencies via `rosdep`:

```sh
rosdep install -i -y --rosdistro melodic --from-paths src
```
- To build:

```sh
catkin_make
```

- To test:

```sh
catkin_make run_tests
```

## Try the code!

This is based on the [pub-sub Python tutorial](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
and the [pub-sub C++ tutorial](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)
so consider looking at the specific instructions in the page for all the details.

Once you have finished building and testing your workspace, make sure you try it.

- Open tmux:

```sh
tmux
```

- Open two panes by sequentially pressing `Ctrl-b` and then `"`.

- Source your development space:

```sh
source devel/setup.bash
```

- In one pane run:

```sh
rosrun py_pubsub talker
```

Alternatively you could try the C++ `talker` by doing:

```sh
rosrun cpp_pubsub talker
```

- And in the other (you can switch between panes by sequentially pressing
  `Ctrl-b` and the `up` and `down` arrow keys):

```sh
rosrun py_pubsub listener
```

Alternatively you could try the C++ `listener` by doing:

```sh
rosrun cpp_pubsub listener
```

You should see that in the `talker` pane you get logs every time a message is
sent with an increasing counter and in the `listener` pane you get the sent
message right after it was logged in the other one.

You can stop each node by pressing `Ctrl-c` and then exit each tmux pane by
`exit`ing the terminal session. You should return to the initial bash session
in the container.

## Running Noah bot simulation

After having built the repository, you can try launching Noah!

- Run:

```sh
source devel/setup.bash
roslaunch noah_bot_launcher noah_navigationless.launch
```
