# Multi-agent Auditory Scene Analysis
A framework for sound source localization, separation and classification, featuring the ability to provide feedback between agents (such as separation quality correcting localization errors, or source class establishing frequency search space). This results in simpler agents (and, in turn, faster response times) which may be prone to errors, but are corrected by multi-agent cooperation.

## Basis of operation

![Diagram of the whole system](/MASACurrent.png?raw=true)

The following directories in this repository represent one agent:

- `beamformphase`: a beamformer based on the phase difference between microphones. It provides two variations: `beamformer` that provides just the estimation of the source of interest; and `beamformermix` that also provides the estimation of the interference, in a multiplexed format. It also provides the `jack_write` agent that channels the output of the `demucs`/`demucsmix` agent to the JACK server.
- `demucs`: a speech enhancer that is trained with the output of the `beamformer` agent.
- `demucsmix`: a variation of `demucs` that uses the output of the `beamformermix` agent variation.
- `doaoptimizer`: a direction-of-arrival corrector by optimizing the `demucs` speech quality.
- `doa_plot`: a plotter of the outputs of both the `soundloc` and the `doaoptimizer` agents.
- `jack_control`: controls the start and end of the [`jackaudio`](https://jackaudio.org/) server.
- `masacoord`: coordinates the start and end of all the agents using a [`terminator`](https://gnome-terminator.org/) split console.
- `online_sqa`: measures the speech quality of the `demucs`/`demucsmix` output.
- `soundloc`: a multiple-sound-source direction-of-arrival estimator.

The rest of the directories define functions for communication protocols:
- `jack_msgs`: defines the ROS2 message type `JackAudio` with which audio data is passed from one module to another.
- `jack_cmd`: defines the ROS2 service `JACKcmd` with which the JACK audio server can be stopped and started with the `jack_control` agent.
- `rosjack`: provides the functionality of bridging the ROS2 audio-type topics and the JACK audio server.

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

It can be noted that `masacoord` can run `jackaudio` as if it were another agent (through the `jack_control` agent), but it should be the first agent to be run (for obvious reasons). Frankly, it is preferable to have `jackaudio` running already before launching `masacoord`.

### terminator

The [`terminator`](https://gnome-terminator.org/) console is used by `masacoord` to run all the agents in one window, by splitting it into different terminals (one for each agent). To do this, it requires the `terminatorlib` API, which is not included in all versions of `terminator`. Thus, it is important to install its latest version from the developer's own repository:

```
sudo add-apt-repository ppa:mattrose/terminator
sudo apt-get update
sudo apt install terminator
```

## Installation

1. Configure the `beamform_config.yaml` inside `beamformphase` so that it matches your microphone setup.

2. Configure the `rosjack_config.yaml` inside `beamformphase` so that:

    - Its output is fed through a ROS2 topic: `output_type` should be either `0` or `2`.
    - Its sampling rate matches the one that `demucs` and `demucsmix` were trained with: `ros_output_sample_rate` should be `16000`.

3. Install the python requirements of all the agents:
    ```
    pip install -r requirements.txt
    ```

4. [Create a ROS2 package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html), and place all of the directories in this repository inside the package's `src` directory. Preferably call this package `masa`.

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


## Agents sets

For ease of comparison between different MASA configurations, the following launch files are included to launch the follwing sets of agents:

- **Linear** set: no frequency selection and no DOA correction is carried out.
    - `beamformer`
    - `demucs`
    - `jack_write`
    - `soundloc`
    - `doaoptimizer_dummy` (a dummy version of `doaoptimizer` that does not do any optimization, just transfers the output of `soundloc` to `beamformer`)
    - `theta_plot`
    - `doa_plot`
        - To launch:
        ```
        ros2 launch masacoord Linear.launch
        ```
- **FrequencySelection** set: frequency selection is carried out, but no DOA correction is carried out.
    - `beamformer`
    - `demucs`
    - `jack_write`
    - `soundloc`
    - `doaoptimizer_dummy` (a dummy version of `doaoptimizer` that does not do any optimization, just transfers the output of `soundloc` to `beamformer`)
    - `freqselect`
    - `theta_plot`
    - `doa_plot`
        - To launch:
        ```
        ros2 launch masacoord FrequencySelection.launch
        ```
- **DOACorrection** set: both frequency selection and DOA correction are carried out.
    - `beamformer`
    - `demucs`
    - `jack_write`
    - `online_sqa`
    - `soundloc`
    - `doaoptimizer`
    - `freqselect`
    - `theta_plot`
    - `doa_plot`
    - `qual_plot`
        - To launch:
        ```
        ros2 launch masacoord DOACorrection.launch
        ```

Furthremore, there are two sets of agents that work with each other and may not be compatible with others. It depends if the `demucs` or the `demucsmix` variation is used, since the former requires both the estimations of the source of interest and the interference. Thus, they must be accompanied with their respective `beamformer`/`beamformermix` and `jack_write`/`jack_writemix` agents.

The **DOACorrection** set launches the agents that are compatible with the mono `demucs` variation. As for the `demucsmix` variation, the following set launches its compatible agents:

- **DOACorrectionMix** set: both frequency selection and DOA correction are carried out.
    - `beamformermix`
    - `demucsmix`
    - `jack_writemix`
    - `online_sqa`
    - `soundloc`
    - `doaoptimizer`
    - `freqselect`
    - `theta_plot`
    - `doa_plot`
    - `qual_plot`
        - To launch:
        ```
        ros2 launch masacoord DOACorrectionMix.launch
        ```
