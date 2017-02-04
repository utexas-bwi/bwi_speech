# bwi_speech
Speech functionality for BWI

# Installation

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
