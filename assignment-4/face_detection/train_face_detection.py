#!/usr/bin/env python3
# Script to train and test a neural network with TF's Keras API for face detection

import os
import sys
import argparse
import datetime
import numpy as np
import tensorflow as tf
from sklearn.model_selection import train_test_split
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Conv2D, Flatten, MaxPooling2D, AveragePooling2D, Softmax, LeakyReLU


def load_data_from_npz_file(file_path):
    """
    Load data from npz file
    :param file_path: path to npz file with training data
    :return: input features and target data as numpy arrays
    """
    data = np.load(file_path)
    return data['input'], data['target']


def normalize_data_per_row(data):
    """
    Normalize a give matrix of data (samples must be organized per row)
    :param data: input data as a numpy array with dimensions NxHxWxC
    :return: normalized data with pixel values in [0,1] (array with same dimensions as input)
    """

    # sanity checks!
    assert len(data.shape) == 4, "Expected the input data to be a 4D matrix"

    return data.astype(float)/255.0

def train_model(model, train_input, train_target, val_input, val_target, logs_dir, epochs=20, learning_rate=0.01, batch_size=16):
    # compile the model: define optimizer, loss, and metrics
    model.compile(optimizer=tf.keras.optimizers.Adam(lr=learning_rate), loss='binary_crossentropy', metrics=['binary_accuracy'])

    # TODO - Create callbacks for saving checkpoints and visualizing loss on TensorBoard
     # tensorboard callback
    log_dir = logs_dir + '/face_log_{}'.format(datetime.datetime.now().strftime("%m-%d-%Y-%H-%M"))
    tbCallBack = tf.keras.callbacks.TensorBoard(log_dir=log_dir, write_graph=True)

    # save checkpoint callback
    checkpointCallBack = tf.keras.callbacks.ModelCheckpoint(os.path.join(logs_dir,'best_face_weights.h5'),
                                                            monitor='binary_accuracy',
                                                            verbose=0,
                                                            save_best_only=True,
                                                            save_weights_only=False,
                                                            mode='auto',
                                                            save_freq=1)

    # do training for the specified number of epochs and with the given batch size
    model.fit(train_input, train_target, epochs=epochs, batch_size=batch_size,
            validation_data=(val_input, val_target),
            callbacks=[tbCallBack, checkpointCallBack])

def main(npz_data_file, batch_size, epochs, lr, val, logs_dir):
    """
    Main function that performs training and test on a validation set
    :param npz_data_file: npz input file with training data
    :param batch_size: batch size to use at training time
    :param epochs: number of epochs to train for
    :param lr: learning rate
    :param val: percentage of the training data to use as validation
    :param logs_dir: directory where to save logs and trained parameters/weights
    """

    input, target = load_data_from_npz_file(npz_data_file)
    N = input.shape[0]
    assert N == target.shape[0], \
        "The input and target arrays had different amounts of data ({} vs {})".format(N, target.shape[0]) # sanity check!
    print("Loaded {} training examples.".format(N))

    # TODO. Complete. Implement code to train a network for image classification
    train_input, val_input, train_target, val_target = train_test_split(input, target, test_size=val)

    train_input = normalize_data_per_row(train_input)
    val_input = normalize_data_per_row(val_input)

    # print(train_input.shape)

    # build the model
    model = build_cnn_model(train_input)

    # train the model
    print("\n\nTRAINING...")
    train_model(model, train_input, train_target, val_input, val_target, logs_dir,
                epochs=epochs, learning_rate=lr, batch_size=batch_size)

def build_cnn_model(train_input):
    alpha = 0.3
    model = Sequential()    

    model.add(Conv2D(4, (3, 3), padding="same", input_shape=train_input.shape[1:]))
    model.add(LeakyReLU(alpha))
    model.add(MaxPooling2D(pool_size=(2,2))) # pool1: 64->32

    model.add(Conv2D(64, (3, 3), padding="same"))
    model.add(LeakyReLU(alpha))
    model.add(MaxPooling2D(pool_size=(2,2))) # pool2: 32->16

    model.add(Conv2D(16, (1, 1), padding="same"))
    model.add(LeakyReLU(alpha))
    model.add(Conv2D(128, (3, 3), padding="same"))
    model.add(LeakyReLU(alpha))
    model.add(Conv2D(16, (1, 1), padding="same"))
    model.add(LeakyReLU(alpha))
    model.add(Conv2D(128, (3, 3), padding="same"))
    model.add(LeakyReLU(alpha))
    model.add(MaxPooling2D(pool_size=(2,2))) # pool3: 16->8
    
    model.add(Conv2D(32, (1, 1), padding="same"))
    model.add(LeakyReLU(alpha))
    model.add(Conv2D(256, (3, 3), padding="same"))
    model.add(LeakyReLU(alpha))
    model.add(Conv2D(32, (1, 1), padding="same"))
    model.add(LeakyReLU(alpha))
    model.add(Conv2D(256, (3, 3), padding="same"))
    model.add(LeakyReLU(alpha))
    model.add(MaxPooling2D(pool_size=(2,2))) # pool4: 8->4

    model.add(Conv2D(64, (1, 1), padding="same"))
    model.add(LeakyReLU(alpha))
    model.add(Conv2D(512, (3, 3), padding="same"))
    model.add(LeakyReLU(alpha))
    model.add(Conv2D(64, (1, 1), padding="same"))
    model.add(LeakyReLU(alpha))
    model.add(Conv2D(512, (3, 3), padding="same"))
    model.add(LeakyReLU(alpha))
    model.add(Conv2D(128, (1, 1), padding="same"))
    model.add(LeakyReLU(alpha))
    model.add(Conv2D(1000, (1, 1), padding="same"))
    model.add(LeakyReLU(alpha))

    model.add(Flatten())
    model.add(Dense(1, activation="sigmoid"))


    return model


if __name__ == "__main__":
    # script arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--epochs", help="number of epochs for training", type=int, default=20)
    parser.add_argument("--batch_size", help="batch size used for training", type=int, default=100)
    parser.add_argument("--lr", help="learning rate for training", type=float, default=1e-3)
    parser.add_argument("--val", help="percent of training data to use for validation", type=float, default=0.2)
    parser.add_argument("--input", help="input file (npz format)", type=str, required=True)
    parser.add_argument("--logs_dir", help="logs directory", type=str, default="")
    args = parser.parse_args()

    if len(args.logs_dir) == 0: # parameter was not specified
        args.logs_dir = 'logs/log_{}'.format(datetime.datetime.now().strftime("%m-%d-%Y-%H-%M"))

    if not os.path.isdir(args.logs_dir):
        os.makedirs(args.logs_dir)

    # run the main function
    main(args.input, args.batch_size, args.epochs, args.lr, args.val, args.logs_dir)
    sys.exit(0)
