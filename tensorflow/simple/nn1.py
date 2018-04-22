""" Neural Network.

A 2-Hidden Layers Fully Connected Neural Network (a.k.a Multilayer Perceptron)
implementation with TensorFlow. 


Author: Aymeric Damien
Project: https://github.com/aymericdamien/TensorFlow-Examples/
"""

from __future__ import print_function

from utils_functions import * # My utility functions

import tensorflow as tf
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
import time

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("-r", help="Retrain existing model", action="store_true")
parser.add_argument("-a", help="Plot trained models", action="store_true")
args = parser.parse_args()
if args.r:
    training = True
    retrain = True
elif args.a:
    training = False
    retrain = False
else:
    training = True
    retrain = False

print('Loading data...')
if training or retrain:
    X = np.loadtxt('samplesTangent.db')
else:
    X = np.loadtxt('samplesTangent_short.db')
n_test = int(X.shape[0]*0.2)
n = int(X.shape[0]-n_test)

x_max = np.max(X,axis=0)
x_min = np.min(X,axis=0)
indices = [3, 4, 5, 6, 7, 8, 12]
x_train = normz(X[0:n,0:3], x_max[0:3], x_min[0:3])
y_train = normz(X[0:n, indices], x_max[indices], x_min[indices])
x_test = normz(X[n:,0:3], x_max[0:3], x_min[0:3])
y_test = normz(X[n:, indices], x_max[indices], x_min[indices])

# plt.figure(0)
# ax = plt.axes(projection='3d')
# n_plot = int(1e4)
# ax.plot3D(x_train[:n_plot,0], x_train[:n_plot,1], x_train[:n_plot,2], 'ro')
# ax.plot3D(y_train[:n_plot,0], y_train[:n_plot,1], y_train[:n_plot,2], 'bo')
# for i in range(n_plot):
#     ax.plot3D([x_train[i,0], y_train[i,0]], [x_train[i,1], y_train[i,1]], [x_train[i,2], y_train[i,2]], 'k-')
# plt.show()
# exit()

# Parameters
learning_rate = 0.0005
num_steps = 500000
batch_size = 300
display_step = 100

# Network Parameters
hidden_layers = [60] * 6
num_input = 3
num_classes = len(indices) 
activation = 1

# tf Graph input
X = tf.placeholder("float", [None, num_input])
Y = tf.placeholder("float", [None, num_classes])

# Store layers weight & bias
weights, biases = wNb(num_input, hidden_layers, num_classes)

# Construct model
prediction = neural_net(X, weights, biases, activation)

# Define loss and optimizer
# cost = tf.reduce_mean(tf.pow(prediction-Y, 2))
cost = tf.reduce_mean(tf.square(prediction - Y))
optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate).minimize(cost)
# optimizer = tf.train.GradientDescentOptimizer(learning_rate=learning_rate).minimize(cost)

# Initialize the variables (i.e. assign their default value)
init = tf.global_variables_initializer()

# Add ops to save and restore all the variables.
saver = tf.train.Saver()

'''
'cp_abb.ckpt' - [20 20 20], a: 3, cost: square - output size: 4
'''

load_from = 'cp2.ckpt'
save_to = 'cp2.ckpt'

# Start training
COSTS = []	# for plotting
STEPS = []	# for plotting
start = time.time()
with tf.Session() as sess:

    if training:
        if  not retrain:
            # Run the initializer
            sess.run(init)
        else:
            # Restore variables from disk.
            saver.restore(sess, "./models/" + load_from)                
            print("Loaded saved model: %s" % "./models/" + load_from)

        for step in range(1, num_steps+1):
            batch_x, batch_y = next_batch(batch_size, x_train, y_train)

            # Run optimization op (backprop)
            _,c = sess.run([optimizer, cost] , feed_dict={X: batch_x, Y: batch_y})
            if step % display_step == 0 or step == 1:
                # Calculate batch loss and accuracy -JUST FOR PRINTING
                print("Step " + str(step) + ", Minibatch cost= " + "{:.4f}".format(c))
                save_path = saver.save(sess, "./models" + save_to)

                COSTS.append(c)
                STEPS.append(step)
        
        print("Optimization Finished!")
        # Save the variables to disk.
        save_path = saver.save(sess, "./models/" + save_to)
        print("Model saved in path: %s" % save_path)
        export_net(weights, biases, x_max, x_min, sess, './net.netxt')
        
        print('Training time: %.3f sec.' % (time.time()-start))

        plt.figure(2)
        plt.semilogy(STEPS, COSTS, 'k-')
        plt.xlabel('Step')
        plt.ylabel('Cost')
        plt.ylim([0, np.max(COSTS)])
        plt.grid(True)

    else:
        # Restore variables from disk.
        saver.restore(sess, "./models/" + load_from)


    # Calculate cost for training data
    y_train_pred = sess.run(prediction, {X: x_train})
    print("Training cost:", sess.run(cost, feed_dict={X: x_train, Y: y_train}))
    # Calculate cost for test data
    y_pred = sess.run(prediction,{X: x_test})
    print("Testing cost:", sess.run(cost, feed_dict={X: x_test, Y: y_test}))

    x_in = x_train[0]
    x_in = np.reshape(x_in, (1,3))
    print("x_in: ", x_in)
    print("train: ", y_train[0])
    print("pred: ", sess.run(prediction, {X: x_in}))
   
    

# fig = plt.figure(1)
# ax1 = plt.axes(projection='3d')
# ax1.plot3D(y_train[:n_plot,0], y_train[:n_plot,1], y_train[:n_plot,2], 'ro')
# ax1.plot3D(y_train_pred[:n_plot,0], y_train_pred[:n_plot,1], y_train_pred[:n_plot,2], 'bo')
# ax1.plot3D(y_pred[:n_plot,0], y_pred[:n_plot,1], y_pred[:n_plot,2], 'go')

# # ax1.plot(x_train, y_train, 'ro', label='Original')
# # ax1.plot(x_train, y_train_pred, 'go', label='Trained prediction')
# # ax1.plot(x_test, y_pred, 'bo', label='Tested prediction')
# # ax1.set_xlabel('x')
# # ax1.set_ylabel('y')
# # ax1.legend()



plt.show()
