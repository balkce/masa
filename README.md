# Multi-agent Auditory Scene Analysis
A framework for sound source localization, separation and classification, featuring the ability to provide feedback between agents (such as separation quality correcting localization errors, or source class establishing frequency search space). This results in simpler agents (and, in turn, faster response times) which may be prone to errors, but are corrected by multi-agent cooperation.

## Basis of operation

![Diagram of the whole system](/MASA.png?raw=true)

Each directory in this repository represents one agent:

- `demucs`: a speech enhancer that is trained with the output of the `phase` beamformer (of the [`beamform2`](https://github.com/balkce/beamform2) ROS2 node) to select the target speech source.
- `doaoptimizer`: a direction-of-arrival corrector by optimizing the `demucs` speech quality 
- `doa_plot`: a plotter of the outputs of both the `soundloc` and the `doaoptimizer` agents.
- `jack_control`: controls the start and end of the [`jackaudio`](https://jackaudio.org/) server, using the `jack_cmd` ROS2 service.
- `masacoord`: coordinates the start and end of all the agents using a [`terminator`](https://gnome-terminator.org/) split console.
- `online_sqa`: measures the speech quality in `demucs` output.
- `soundloc`: a multiple-sound-source direction-of-arrival estimator.


## To clone:

Before cloning this repository, you'll need GIT LFS:

    sudo apt install git-lfs

If your version of Ubuntu/Debian doesn't have the `git-lfs` package, install the following repository:

    curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash

Then run the following (only needs to be run once for each user account):

    git lfs install

Then you can clone this repository by running:

    git clone <url of this repository>


## Dependencies

### jackaudio

A properly configured [`jackaudio`](https://jackaudio.org/) server is required for real-time audio capture and reproduction. Follow its extensive [documentation](https://github.com/jackaudio/jackaudio.github.com/wiki) to install it and configure it.

It can be noted that `masacoord` can run `jackaudio` as if it were another agent, but it should be the first agent to be run (for obvious reasons). Frankly, it is preferable to have `jackaudio` running already before launching `masacoord`.

### beamform2

The [`beamform2`](https://github.com/balkce/beamform2) ROS2 node needs to be cloned in its own directory, and compiled as its own ROS2 package. It includes:

- The `phase` agent, whose beamformer output is `demucs` input.
- The `jack_write` agent that channels `demucs` output to `jackaudio`.
- The `jack_msgs` ROS2 message type, that is used by the `phase`, `demucs`, `online_sqa` and `jack_write` agents.
- The `ros2jack` utility that provides functions that bridge ROS2 and `jackaudio`.


### terminator

The [`terminator`](https://gnome-terminator.org/) console is used by `masacoord` to run all the agents in one window, by splitting it into different terminals (one per `masa` agent). To do this, it requires the `terminatorlib` API, which is not included in all versions of `terminator`. Thus, it is important to install its latest version from the developer's own repository:

```
sudo add-apt-repository ppa:mattrose/terminator
sudo apt-get update
sudo apt install terminator
```

## Installation

1. Configure the `beamform_config.yaml` file of [`beamform2`](https://github.com/balkce/beamform2) so that it matches your microphone setup.

2. Configure the `rosjack_config.yaml` file of [`beamform2`](https://github.com/balkce/beamform2) so that:

    - Its output is fed through a ROS2 topic: `output_type` should be either `0` or `2`.
    - Its sampling rate matches the one that `demucs` was trained with: `ros_output_sample_rate` should be `16000`.

    *Remember to run `colcon build` inside the `beamform2` directory so that all of these changes take effect.*

3. Install the python requirements of all the agents in `masa`:
    ```
    pip install -r requirements.txt
    ```

4. [Create a ROS2 package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html), and place all of the directories in this repository inside the package's `src` directory.

5. Specify which agents to launch in `src/masacoord/config/masacoord_config.yaml`.

6. Compile the newly created `masa` ROS2 package (this may take a while):
    ```
    colcon build
    ```

## To run

To run all of the agents in parallel, do:
```
ros2 launch masacoord masacoord.launch
```
You can change which agents are run by modifying `src/masacoord/config/masacoord_config.yaml`. However, for these changes to take effect, you'll need to re-compile the `masa` ROS2 package (since the change is minimal this time around, it shouldn't take as long as before).
