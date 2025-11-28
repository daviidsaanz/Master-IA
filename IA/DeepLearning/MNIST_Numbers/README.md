# From-Scratch Neural Network for MNIST

This project implements a **neural network from scratch** in Python using only **NumPy**, without any deep learning libraries. The goal is to classify **MNIST** digit images (28x28 pixels).

---

## Dataset

- The **MNIST** dataset is used, with 60,000 training images and 10,000 test images.  
- Each image is flattened into a 784-element vector.  
- Labels are converted to **one-hot encoding** for training.

```python
X = X.reshape(X.shape[0], 28*28)
Y = np.zeros((train_labels.shape[0], 10))
for i, label in enumerate(train_labels):
    Y[i, label] = 1

batch_size = 500
num_samples = X.shape[0]

data = []
for i in range(0, num_samples, batch_size):
    x_batch = X[i:i + batch_size]
    y_batch = Y[i:i + batch_size]
    data.append((x_batch, y_batch))

model = Model([
    Deep(784, 100),           
    Activation(relu, relu_deriv),
    Deep(100, 10),
    Activation(softmax, softmax_deriv)
])

lr = 0.001
epochs = 15
epoch = 0

while epoch < epochs:
    print("Epoch:", epoch + 1)
    train_loop(model, data, lr)
    epoch += 1
```

---

## Classes

The model is organized in **from-scratch classes**:

- **Layer (abstract)**: defines the interface for all layers (**init**, **forward**, **backward**, **update**).  
- **Deep (dense layer)**: initializes **weights W** and **biases B**, computes **forward** and **backward**, and updates weights using the **Adam optimizer** with **momentum**, **RMSProp**, and **bias correction**.  
- **Activation**: applies activation functions (**ReLU**, **Softmax**) and their derivatives.  

Using **Adam** allowed the network to improve accuracy from ~15% (without Adam) to **~80%**.

---

## Implementation

The **training process** uses the following structure:

- **train_step**: computes predictions, calculates the error, backpropagates the gradients, updates the weights, and prints the batch accuracy.  
- **train_loop**: iterates over all batches in the dataset and calls `train_step` for each batch.  
- **accuracy**: computes the percentage of correctly predicted samples.

```python
def train_step(model, x, y, lr):
    y_pred = model.forward(x)
    error = y_pred - y 
    model.backward(error)
    model.update(lr)
    acc = accuracy(y_pred, y)
    print("Accuracy:", acc)

def train_loop(model, data, lr):
    for x_batch, y_batch in data:
        train_step(model, x_batch, y_batch, lr)

def accuracy(y_pred, y_real):
    goods = np.sum(np.argmax(y_pred, axis=1) == np.argmax(y_real, axis=1))
    acc = goods / len(y_real)
    return acc
```

---

## Test / Prediction

After training, the model can **predict single images**. The input image must be **flattened** to a 1D array (shape 784).

```python
def predict_single(model, image_flat):
    # reshape a single sample to (1, 784)
    x = image_flat.reshape(1, -1)
    y_pred = model.forward(x)
    predicted_class = np.argmax(y_pred, axis=1)[0]
    return predicted_class
```
