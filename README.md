# bwi_speech
Speech functionality for BWI

# Installation and Setup

To install first clone this repo to your catkin workspace: 

```
$ git clone https://github.com/utexas-bwi/bwi_speech.git
```

Now download all files pertaining to the submodules of the repo:

```
$ cd catkin_ws/src/bwi_speech/
$ git submodule update --init --recursive
```

Now that it's downloaded, switch to the speech acquisition branch: 

```
$ git checkout speech_language_acquisition
```

Next, you'll need to install the other bwi packages following the 
instructions found here: https://github.com/utexas-bwi/bwi

Once that's installed, switch to the static fact query branch of bwi_common: 

```
$ cd catkin_ws/src/bwi_common/
$ git checkout jesse/static_fact_query
```

Finally, you'll have to install any dependencies which the Google
Speech API requires, this step may take some cleaning up of packages: 

```
$ cd catkin_ws/src/bwi_speech/ThirdParty/Samples/Google/python-docs-samples/speech/grpc/
$ sudo pip install -r requirements.txt
```

Before you can use the Google Speech API, you need to install the Google Cloud SDK using the
instructions found at: https://cloud.google.com/sdk/downloads#apt-get

After installing the SDK, you'll need to authenticate yourself to be able to call the API: 

```
$ gcloud beta auth application-default login
```

# Usage

In order to use, first source your workspace: 

```
$ source catkin_ws/devel/setup.bash
```

Then open a new terminal tab to run roscore:

```
$ roscore
```

Then in another tab either run the Segbot simulator: 

```
$ roslaunch bwi_launch simulation.launch
```

OR run the actual Segbot package for controlling one 
of the Segbots:

```
$ roslaunch bwi_launch segbot_v2.launch
$ roslaunch bwi_kr_execution bwi_kr_execution.launch
```

Finally, run the speech pipeline script: 

```
$ rosrun bwi_speech speech_pipeline.py
```
