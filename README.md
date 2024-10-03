# speech_input_server

This ROS 2 node implements speech recognition and wake word detection using the Microsoft Cognitive Speech Services and the *Microsoft Cognitive Services Speech SDK*.

A Microsoft Azure account is required to use this node.  After setting up the speech services for your account, specify these environment variables before running the elsabot_bt node.

export MS_COGNTIVE_SUB_KEY=[your key]

export MS_COGNTIVE_SUB_REGION=[your region]

Required to build package:

    sudo apt-get install libusb-1.0-0-dev