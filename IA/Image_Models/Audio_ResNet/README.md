# 🎵 ESC-50 Audio Classification with ResNet-50

## 📖 Description
This project demonstrates how to **load and classify the ESC-50 dataset** using images generated from audio and a **ResNet-50 model pretrained on ImageNet** with PyTorch.

The workflow includes:
- 🎧 Loading the ESC-50 dataset  
- 🛠 Creating a custom PyTorch Dataset  
- 📊 Splitting into training and test sets  
- 🚀 Training a ResNet-50 model  
- 📈 Evaluating performance using **accuracy** and **confusion matrix**  

---

## 🧠 Model
- Architecture: **ResNet-50 pretrained on ImageNet**  
- The final fully connected layer is modified to classify **50 classes**  
- Loss function: `CrossEntropyLoss`  
- Optimizer: `Adam`  

---

## 📂 Dataset
Official dataset:  
🔗 [ESC-50 GitHub](https://github.com/karolpiczak/ESC-50)  

You only need to download:
- `audio/` folder  
- `meta/` folder  

Expected project structure:
datasets/ESC-50/
- audio/
- meta/

---

## 🛠 Installation

Create a virtual environment (optional):
```bash
python -m venv venv
source venv/bin/activate   # Linux / macOS
venv\Scripts\activate      # Windows
```

Install dependencies:
```bash

pip install torch torchvision numpy matplotlib scikit-learn pillow seaborn librosa
```

---

## Running the project
Run the notebook or Python script to:
- Train the model  
- Evaluate test accuracy  
- Visualize the confusion matrix 