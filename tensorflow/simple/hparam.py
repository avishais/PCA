""" Auto Encoder Example.

Build a 2 layers auto-encoder with TensorFlow to compress images to a
lower latent space and then reconstruct them.

References:
    Y. LeCun, L. Bottou, Y. Bengio, and P. Haffner. "Gradient-based
    learning applied to document recognition." Proceedings of the IEEE,
    86(11):2278-2324, November 1998.

Links:
    [MNIST Dataset] http://yann.lecun.com/exdb/mnist/

Author: Aymeric Damien
Project: https://github.com/aymericdamien/TensorFlow-Examples/
"""
from __future__ import division, print_function, absolute_import

from utils_functions import *

import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import scipy.io as sio
import time
import random

def run_net(X, learning_rate, num_hidden_layer, k, activation, b):

    n_test = 20000
    n = X.shape[0]-n_test

    x_max = np.max(X,axis=0)
    x_min = np.min(X,axis=0)

    indices = [3, 4, 5, 6, 7, 8]#, 12]
    x_train = normz(X[0:n,0:3], x_max[0:3], x_min[0:3])
    y_train = normz(X[0:n, indices], x_max[indices], x_min[indices])
    x_test = normz(X[n:,0:3], x_max[0:3], x_min[0:3])
    y_test = normz(X[n:, indices], x_max[indices], x_min[indices])

    # Training Parameters
    num_steps = 10000
    batch_size = b

    display_step = 1000

    # Network Parameters
    num_classes = 6
    hidden_layers = [k]*num_hidden_layer
    hidden_layers.append(num_classes)
    num_input = 3 

    # tf Graph input 
    X = tf.placeholder("float", [None, num_input])
    Y = tf.placeholder("float", [None, num_classes])

    # Store layers weight & bias
    weights, biases = wNb(num_input, hidden_layers, num_classes)

    # Construct model
    prediction = neural_net(X, weights, biases, activation)

    # Define loss and optimizer, minimize the squared error
    cost = tf.reduce_mean(tf.square(prediction - Y))
    optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate).minimize(cost)

    # Initialize the variables (i.e. assign their default value)
    init = tf.global_variables_initializer()

    # Add ops to save and restore all the variables.
    saver = tf.train.Saver()

    # Start Training
    # Start a new TF session
    with tf.Session() as sess:

        # Run the initializer
        sess.run(init)

        # Training
        for i in range(1, num_steps+1):
            # Get the next batch 
            batch_x, batch_y = next_batch(batch_size, x_train, y_train)

            # Run optimization op (backprop) and cost op (to get loss value)
            _, c = sess.run([optimizer, cost], feed_dict={X: batch_x, Y: batch_y})
            # Display logs per step
            if i % display_step == 0 or i == 1:
                print('Step %i: Minibatch Loss: %f' % (i, c))

        # Save the variables to disk.
        saver.save(sess, "./models/hcp.ckpt")

        # Testing
        # Calculate cost for training data
        y_train_pred = sess.run(prediction, {X: x_train})
        training_cost = sess.run(cost, feed_dict={X: x_train, Y: y_train})
        # Calculate cost for test data
        y_pred = sess.run(prediction,{X: x_test})
        testing_cost = sess.run(cost, feed_dict={X: x_test, Y: y_test})
        
        return training_cost, testing_cost

def log2file(i, learning_rate, num_hidden_layer, k, activation, training_cost, testing_cost):
    f = open('./hparam.txt','a+')

    # f.write("-----------------------------\n")
    # f.write("Trial %d\n" % i)
    # f.write("Learning rate: %f\n" % learning_rate)
    # f.write("Number of hidden layers: %d\n" % num_hidden_layer)
    # f.write("Number of neurons in each layer: %d\n" % k)
    # f.write("Activation: %d" % activation)
    # f.write("Training cost: %f\n" % training_cost)
    # f.write("Testing cost: %f\n" % testing_cost)

    f.write("%d %.5f %d %d %d, %f %f\n" % (i, learning_rate, num_hidden_layer, k, activation, training_cost, testing_cost))

    f.close()


def main():
    #X = np.loadtxt('../data/ckc2d_10_samples_noJL.db')
    print('Loading training data...')
    X = np.loadtxt('samplesTangent.db')

    lr = [0.0001,	0.0005,	0.001,	0.002,	0.003,	0.004,	0.005,	0.006,	0.007,	0.008,	0.009,	0.01,	0.02,	0.0288888888888889,	0.0377777777777778,	0.0466666666666667,	0.0555555555555556,	0.0644444444444444,	0.0733333333333333,	0.0822222222222222,	0.0911111111111111,	0.1,	0.110000000000000,	0.120000000000000]
    a = [1, 3, 4]

    for i in range(100):
        learning_rate = random.choice(lr) #lr[random.randint(0,len(lr)-1)]
        num_hidden_layer = random.randint(1,7)
        k = random.randint(10,100)
        activation = random.choice(a)
        b = random.randint(40, 400)

        print("-----------------------------")
        print("Trial ", i)
        print("Learning rate: ", learning_rate)
        print("Number of hidden layers: ", num_hidden_layer)
        print("Number of neurons in each layer: ", k)
        print("Batch size: ", b)
        if activation==1 : 
            str = "sigmoid" 
        elif activation==2 : 
            str = "relu" 
        elif activation==3:
            str = "tanh"
        else: str = "elu"
        print("Activation function: ", str)

        training_cost, testing_cost = run_net(X, learning_rate, num_hidden_layer, k, activation, b)
        print("Training cost: ", training_cost)
        print("Testing cost: ", testing_cost)

        log2file(i, learning_rate, num_hidden_layer, k, activation, training_cost, testing_cost)

if __name__ == '__main__':
  main()
