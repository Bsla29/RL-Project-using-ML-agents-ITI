# HummingBirds – Unity ML-Agents Project

A Unity-based reinforcement learning project where a hummingbird agent learns to collect nectar using ML-Agents (PPO).---

## 📁 Project Structure

Assets/ # Unity scenes & scripts
Packages/ # Unity package dependencies
ProjectSettings/ # Unity project settings

## 🛠️ Prerequisites
1. **Unity** (with ML-Agents package installed)  
2. **Anaconda/Miniconda** (for Python env)

conda create -n ml-env python=3.8 -y
conda activate ml-env
pip install mlagents tensorboard onnxruntime torch


🏁 Training the Agent

mlagents-learn trainer_config.yaml --run-id=hb_01 
mlagents-learn -help --> for the rest of the commadns
Open Unity and press ▶️ Play

Ensure Edit → Project Settings → Player → Run In Background is checked.



📊 Viewing Metrics with TensorBoard
In a new terminal (same folder):

tensorboard --logdir results --port 6006

Open in your browser:
http://localhost:6006
