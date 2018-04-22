
import tensorflow as tf
import numpy as np


# Export net to text file - this function must be called within the session
def export_net(W, B, x_max, x_min, sess, path_file = './net.netxt'):
    f = open(path_file,'w')

    k = W.keys()
    n = int(len(k))
    f.write(str(n-1) + ' ')
    
    # W's
    for i in range(n):
        sth = 'h' + str(i+1)
        w = sess.run(W[sth])
        f.write(str(w.shape[0]) + ' ' + str(w.shape[1]) + ' ')
        for j in range(w.shape[0]):
            for k in range(w.shape[1]):
                f.write(str(w[j,k]) + ' ')
    
    # b's
    for i in range(n):
        sth = 'b' + str(i+1)
        b = sess.run(B[sth])
        f.write(str(b.shape[0]) + ' ')
        for j in range(b.shape[0]):
                f.write(str(b[j]) + ' ')

    for i in range(len(x_max)):
        f.write(str(x_max[i]) + ' ')
    for i in range(len(x_max)):
        f.write(str(x_min[i]) + ' ')

    f.close()

def normz(x, x_max, x_min):
    
    n = x.shape[0]
    x = (x-np.tile(x_min,(n,1)))/np.tile(x_max-x_min,(n,1))

    return x

def denormz(x, x_max, x_min):

    n = x.shape[0]
    x = x*np.tile(x_max-x_min,(n,1)) + np.tile(x_min,(n,1))
    
    return x

# -----------------------------------------------------------------------

def next_batch(num, data, labels):
    '''
    Return a total of `num` random samples. 
    Similar to mnist.train.next_batch(num)
    '''
    idx = np.arange(0 , len(data))
    np.random.shuffle(idx)
    idx = idx[:num]
    data_shuffle = [data[ i] for i in idx]
    labels_shuffle = [labels[ i] for i in idx]

    return np.asarray(data_shuffle), np.asarray(labels_shuffle)


# Build weight and bias matrices
def wNb(num_input, hidden_layers, num_output):
    weights = {}
    biases = {}
    h = hidden_layers
    h = np.insert(h, 0, num_input)
    h = np.append(h, num_output)

    for i in range(len(h)-1):
        sth = 'h' + str(i+1)
        weights.update({sth: tf.Variable(tf.random_normal([h[i], h[i+1]]))})
        stb = 'b' + str(i+1)
        biases.update({stb: tf.Variable(tf.random_normal([h[i+1]]))})
    
   
    return weights, biases

def activF(x, activation_index):
    if activation_index==1:
        return tf.nn.sigmoid(x)
    if activation_index==2:
        return tf.nn.relu(x)
    if activation_index==3:
        return tf.nn.tanh(x)
    if activation_index==4:
        return tf.nn.elu(x)
    return x;

# Create model
# ReLU is added for non-linearity
def neural_net(x, weights, biases, activation_index=1):
    
    # First hidden fully connected layer 
    layer = activF(tf.add(tf.matmul(x, weights['h1']), biases['b1']), activation_index)

    # Remaining hidden fully connected layer 
    for i in range(2, len(weights)+1):
        sth = 'h' + str(i)
        stb = 'b' + str(i)
        layer = activF(tf.add(tf.matmul(layer, weights[sth]), biases[stb]), activation_index)

    # Output fully connected layer with a neuron for each class
    return layer

