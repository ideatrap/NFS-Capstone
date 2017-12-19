import rospy
from styx_msgs.msg import TrafficLight
import numpy as np
import os
import sys
import tensorflow as tf
from collections import defaultdict
from io import StringIO
from PIL import Image
import cv2


class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        sim = rospy.get_param("/use_simulator")
        #rospy.logwarn("sim {}".format(sim)) 
        curr_path = os.getcwd()
        classifier_path = curr_path+'/light_classification'
        if sim == True:
            rospy.logwarn("On Simulator")
            self.path_to_ckpt =  classifier_path + '/model/frozen_inference_graph_sim.pb'
        else:
            rospy.logwarn("On Site")
            self.path_to_ckpt =  classifier_path + '/model/frozen_inference_graph_real.pb'
        # List of the strings that is used to add correct label for each box.
        self.path_to_labels = classifier_path + '/model/model_label_map.pbtxt'
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.path_to_ckpt, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        self.detection_graph.as_default()
        self.sess = tf.Session(graph=self.detection_graph)
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        #TODO implement light color prediction
        # Definite input and output Tensors for detection_graph
        image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular object was detected.
        detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

        # the array based representation of the image will be used later in order to prepare the
        # result image with boxes and labels on it.
        image_np = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image_np, axis=0)
        # Actual detection.
        (boxes, scores, classes, num) = self.sess.run([detection_boxes, detection_scores, detection_classes, num_detections],feed_dict={image_tensor: image_np_expanded})
        classes_int = np.squeeze(classes).astype(np.int32)
        scores_sq = np.squeeze(scores)
        #rospy.logwarn("classify: {}, {}".format(classes_int[0], scores_sq[0]))
        if scores_sq[0] < 0.5:
            return TrafficLight.UNKNOWN
        elif classes_int[0] == 1:
            return TrafficLight.GREEN
        elif classes_int[0] == 2:
            return TrafficLight.RED
        elif classes_int[0] == 7:
            return TrafficLight.YELLOW
        else:
            return TrafficLight.UNKNOWN
