import tensorflow as tf
from tensorflow import keras
import cv2
import numpy as np
import struct
import os 
import sys

class NnChessDetector:
    def __init__(self):
        self.Width = 640
        self.Height = 480
        #self.model = tf.keras.models.load_model("/home/david/interbotix_ws/src/piece_detector/scripts/loc_to_servos/test1.h5")
        self.scale = 1/256
        self.conf_threshold = 0.5
        weights = "/home/david/interbotix_ws/src/piece_detector/scripts/loc_to_servos/yolov3-tiny_last.weights"
        cfg = "/home/david/interbotix_ws/src/piece_detector/scripts/loc_to_servos/yolov3-tiny.cfg"
        self.classes = ['white', 'black']
        self.net = cv2.dnn.readNet(weights, cfg)

    def get_output_layers(self, net):
        layer_names = net.getLayerNames()
        output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
        return output_layers

    def list_detected_pieces(self, image):
        blob = cv2.dnn.blobFromImage(image, self.scale, (self.Width, self.Height), (0, 0, 0), True, crop=True)
        self.net.setInput(blob)
        outs = self.net.forward(self.get_output_layers(self.net))
        max_confidence = self.conf_threshold
        card = None
        cards_found = 0
        pieces = []
        for out in outs:

            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                for objIndex in range(2):
                    confidence = scores[objIndex]

                    if confidence >  self.conf_threshold:
                        #print(detection)
                        center_x = int(detection[0] * self.Width)
                        center_y = int(detection[1] * self.Height)
                        pieces.append([objIndex, center_y, center_x])

        return pieces

