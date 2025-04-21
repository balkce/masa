# Multi-agent Auditory Scene Analysis
A framework for sound source localization, separation and classification, featuring the ability to provide feedback between agents (such as separation quality correcting localization errors, or source class establishing frequency search space). This results in simpler agents (and, in turn, faster response times) which may be prone to errors, but are corrected by multi-agent cooperation.

## Basis of operation

![Diagram of the whole system](/MASA.png?raw=true)

## Dependencies

## Before running

1. Install and configure [`jackaudio`](https://jackaudio.org/).

2. Clone and compile the [`beamform2`](https://github.com/balkce/beamform2) ROS2 node.

3. Configure the `beamform_config.yaml` of `beamform2` so that it matches your microphone setup.

4. Configure the `rosjack_config.yaml` of `beamform2` so that:

   - Its output is fed through a ROS2 topic: `output_type` should be either `0` or `2`.
   - Its sampling rate matches the one that `demucs` was trained with: `ros_output_sample_rate` should be `16000`.

5. Install the python requirements of all the nodes:

   `pip install -r requirements.txt`

6. [Create a ROS2 package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html), place the all of the directories in this repository inside the package's `src` directory, and run `colcon build`.
