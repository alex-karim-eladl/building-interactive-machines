#!/usr/bin/env python3
# Script to train and test a neural network with TF's Keras API for face detection

import os
import sys
import rospy
import train_utils
import datetime
import numpy as np
import tensorflow as tf
from sklearn.model_selection import train_test_split
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense

  
def compute_normalization_parameters(data):
    """
    Compute normalization parameters (mean, st. dev.)
    :param data: matrix with data organized by rows [N x num_variables]
    :return: mean and standard deviation per variable as row matrices of dimension [1 x num_variables]
    """

    mean = np.mean(data, axis=0)
    stdev = np.std(data, axis=0)

    # transpose mean and stdev in case they are (2,) arrays
    if len(mean.shape) == 1:
        mean = np.reshape(mean, (1,mean.shape[0]))
    if len(stdev.shape) == 1:
        stdev = np.reshape(stdev, (1,stdev.shape[0]))

    return mean, stdev


def normalize_data_per_row(data, mean, stdev):
    """
    Normalize a give matrix of data (samples must be organized per row)
    :param data: input data
    :param mean: mean for normalization
    :param stdev: standard deviation for normalization
    :return: whitened data, (data - mean) / stdev
    """

    # sanity checks!
    assert len(data.shape) == 2, "Expected the input data to be a 2D matrix"
    assert data.shape[1] == mean.shape[1], "Data - Mean size mismatch ({} vs {})".format(data.shape[1], mean.shape[1])
    assert data.shape[1] == stdev.shape[1], "Data - StDev size mismatch ({} vs {})".format(data.shape[1], stdev.shape[1])

    centered = data - np.tile(mean, (data.shape[0], 1))
    normalized_data = np.divide(centered, np.tile(stdev, (data.shape[0],1)))

    return normalized_data

   
def train_model(model, train_input, train_target, val_input, val_target, input_mean, input_stdev,
                epochs, learning_rate, batch_size):
    """
    Train the model on the given data
    :param model: Keras model
    :param train_input: train inputs
    :param train_target: train targets
    :param val_input: validation inputs
    :param val_target: validation targets
    :param input_mean: mean for the variables in the inputs (for normalization)
    :param input_stdev: st. dev. for the variables in the inputs (for normalization)
    :param epochs: epochs for gradient descent
    :param learning_rate: learning rate for gradient descent
    :param batch_size: batch size for training with gradient descent
    """
    # print(input_mean.shape)
    # print(input_stdev)
    python_file = open("../data/param.txt", "w")
    for i in range(5):
        python_file.write(str(input_mean[0,i]))
        python_file.write(' ')
        python_file.write(str(input_stdev[0,i]))
        python_file.write(' ')
    python_file.close()

    # rospy.set_param("~model", os.path.abspath(os.getcwd()) + "../src/best_imitation_weights.h5") 
    # rospy.set_param("~norm_params", os.path.abspath(os.getcwd()) + "../data/param.txt")

    # normalize
    norm_train_input = normalize_data_per_row(train_input, input_mean, input_stdev)
    norm_val_input = normalize_data_per_row(val_input, input_mean, input_stdev)

    # compile the model: define optimizer, loss, and metrics
    model.compile(optimizer=tf.keras.optimizers.Adam(lr=learning_rate), loss='mse', metrics=['mae'])

    # TODO - Create callbacks for saving checkpoints and visualizing loss on TensorBoard
     # tensorboard callback
    logs_dir = 'logs/log_{}'.format(datetime.datetime.now().strftime("%m-%d-%Y-%H-%M"))
    tbCallBack = tf.keras.callbacks.TensorBoard(log_dir=logs_dir, write_graph=True)

    # save checkpoint callback
    checkpointCallBack = tf.keras.callbacks.ModelCheckpoint(os.path.join(logs_dir,'best_imitation_weights.h5'),
                                                            monitor='mae',
                                                            verbose=0,
                                                            save_best_only=True,
                                                            save_weights_only=False,
                                                            mode='auto',
                                                            save_freq=1)

    # do training for the specified number of epochs and with the given batch size
    model.fit(norm_train_input, train_target, epochs=epochs, batch_size=batch_size,
            validation_data=(norm_val_input, val_target),
            callbacks=[tbCallBack, checkpointCallBack]) # add this extra parameter to the fit function


def main(input_path, batch_size, epochs, lr):
    """
    Main function that performs training and test on a validation set
    :param npz_data_file: npz input file with training data
    :param batch_size: batch size to use at training time
    :param epochs: number of epochs to train for
    :param lr: learning rate
    :param val: percentage of the training data to use as validation
    :param logs_dir: directory where to save logs and trained parameters/weights
    """

    input, target = train_utils.load_data(input_path)
    # print(input.shape)
    # print(target.shape)
    train_input, val_input, train_target, val_target = train_test_split(input, target, test_size=0.4)

    mean, stdev = compute_normalization_parameters(train_input)

    model = imitation_model()
    # build_fn = lambda: tf.keras.models.load_model("../src/best_imitation_weights.h5", compile=False)
    # model = build_fn()


    train_model(model, train_input, train_target, val_input, val_target, mean, stdev,
                epochs=epochs, learning_rate=lr, batch_size=batch_size)


def imitation_model():
    model = Sequential()    
    model.add(Dense(128, activation="relu", input_shape=(5,)))
    model.add(Dense(128, activation="relu"))
    model.add(Dense(2, activation="linear"))
    return model


if __name__ == "__main__":
    # script arguments
    # parser = argparse.ArgumentParser()
    # parser.add_argument("input_path", help="path to training data", type=str)
    # args = parser.parse_args()

    assert len(sys.argv) == 2,\
        "invalid number of arguments"

    # features, targets
    logs_dir = 'logs/log_{}'.format(datetime.datetime.now().strftime("%m-%d-%Y-%H-%M"))

    if not os.path.isdir(logs_dir):
        os.makedirs(logs_dir)

    # run the main function
    main(sys.argv[1], batch_size=64, epochs=1000, lr=1e-3)
    sys.exit(0)
