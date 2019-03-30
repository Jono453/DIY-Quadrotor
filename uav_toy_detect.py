#Tensorflow and opencv
#Real Time Winnie the Pooh Toy Detecto
#to be put onto Pi on UAV

# TensorFlow and tf.keras
import pandas as pd
import tensorflow as tf
from tensorflow import keras
import cv2
import time

# Helper libraries
import numpy as np
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

WP_Names = ['Pooh', 'Tigger','Eyore','Piglet']
cap = cv2.VideoCapture(1)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    #in each frame:
    #1. Load pre-trained model
    #2. Classify which character is seen

    # Display the resulting frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()




