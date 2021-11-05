#!/usr/bin/env python3
# Script to train and test a neural network with TF's Keras API for face detection

from math import inf
import os
import sys
import argparse
import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt
from train_face_detection import normalize_data_per_row, load_data_from_npz_file

def evaluate(input_features, target, model):
    """
    Helper function to evaluate model
    :param input_features: input tensor
    :param target: target tensor
    :param model: Keras model
    :return: list of TPR, list of FPR, list of thresholds, index of best threshold as an int
    """
    # normalize the inputs
    norm_input = normalize_data_per_row(input_features)

    # ... output model predictions on "prob" variable
    prob = model.predict(norm_input)


    # generate roc thresholds
    thresholds = [x/100.0 for x in range(0,100,2)]
    
    tpr = [] # list of true positive rate per threshold
    fpr = [] # list of false positive rate per threshold
    N = input_features.shape[0] # number of examples
  
    # compute the true positive rate and the false positive rate for each of the thresholds
    for t in thresholds:
  
        # turn predicted probabilities to 0-1 values based on the threshold
        prediction = np.zeros(prob.shape)
        prediction[prob > t] = 1
      
        # compute tpr and fpr based on the predictions and the target values from the dataset     
        # TO-DO. complete
        # print(sum(prediction and target), sum(target))
        # print(prediction.shape)
        current_tpr = sum(np.logical_and(prediction, target)) / sum(target)
        current_fpr = sum(np.logical_and(prediction, np.logical_not(target))) / sum(np.logical_not(target))

        tpr.append(current_tpr)
        fpr.append(current_fpr)
      
    # pick threshold that minimizes l2 distance to top-left corner of the graph (fpr = 0, tpr = 1)
    # TODO. Complete.
    # index of the threshold for which (fpr, tpr) get closest to (0,1) in the Euclidean sense
    tpr = np.array(tpr)
    fpr = np.array(fpr)

    diff = np.concatenate((fpr,tpr-1), axis=1)

    l2_err = np.sqrt(np.sum(np.power(diff, 2), axis=1))
    index = np.argmin(l2_err)

    return tpr, fpr, thresholds, index


def main(input_file, weights_file):
    """
    Evaluate the model on the given input data
    :param input_file: npz data
    :param weights_file: path to h5 file with model definition and weights
    """
    # load data
    input_features, target = load_data_from_npz_file(input_file)

    N = input_features.shape[0]

    # sanity check!
    assert N == target.shape[0], \
        "Error: The input and target arrays had different amounts of data ({} vs {})".format(N, target.shape[0])

    # load keras model from file
    model = tf.keras.models.load_model(weights_file) 
    
    tpr, fpr, thresholds, index = evaluate(input_features, target, model)

    print("Best threshold was: {} (TPR = {}, FPR = {})".format(thresholds[index], tpr[index], fpr[index]))

    # plot the ROC curve with matplotlib
    plt.plot(fpr, tpr)
    plt.scatter(fpr[index], tpr[index], s=20, c='r')
    plt.xlabel('False positive rate')
    plt.ylabel('True positive rate')
    plt.xlim([0,1])
    plt.ylim([0,1])
    plt.title('ROC Curve')
    plt.show()


if __name__ == "__main__":

    # script arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", help="input file (npz format)",
                        type=str, default='64x64_data.npz')
    parser.add_argument("--weights-path", help="path for the weights file",
                        type=str, default='weights.h5')
    args = parser.parse_args()
    
    # run the main function
    main(args.input, args.weights_path)
    sys.exit(0)
