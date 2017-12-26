# Need for Speed SDC Capstone Project
This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://github.com/udacity/CarND-Capstone).

**Need for Speed** is comprised of the following engineers:

* Kiarie Ndegwa (u4742829@anu.edu.au)
* Joseph Zhou (zhouqi.joseph@gmail.com)
* Pramod BM (pmb@nvidia.com)
* Sidharth Varier (sidvarier@gmail.com)
* Mike Rzucidlo (mfrstatements@yahoo.com)

To build the environment needed to run the code in this repo, you should follow the instructions in the [original Udacity project instructions](https://github.com/udacity/CarND-Capstone).

# Building and running the project
Please find the models for traffic light classification [here](https://drive.google.com/open?id=1_Tth59EMFbogki_6tEUdMxMvloUF__Vz).
The models used in this exercise are based on the **Faster rcnn resnet 101 architecture**; pretrained on the Coco dataset and fine-tuned on the **Bosch traffic signal dataset**.

To use the models in the simulator you must:
* Download and extract the model from the above link. (`model.tar.gz` contains two models; one trained for the simulator and the other for the real world.)
* The respective files are labelled```frozen_inference_graph_real.pb``` and ```frozen_inference_graph_sim.pb```
* Copy both frozen models into ```ros/src/tl_detector/light_classification/model``` into your local repo

Once set up you need to go to the folder labeled ```NFS-Capstone/ros``` and type in the following and build commands:
* ```catkin_make```
* ```devel/setup.sh```
* ```roslaunch launch/styx.launch```
