"""
  Generated using Konverter: https://github.com/ShaneSmiskol/Konverter
"""

import numpy as np

wb = np.load('/home/gregor/openpilot/model_s_keras_weights.npz', allow_pickle=True)
w, b = wb['wb']

def predict(x):
  x = np.array(x, dtype=np.float32)
  l0 = np.dot(x, w[0]) + b[0]
  l0 = np.maximum(0, l0)
  l1 = np.dot(l0, w[1]) + b[1]
  l1 = np.maximum(0, l1)
  l2 = np.dot(l1, w[2]) + b[2]
  l2 = np.tanh(l2)
  return l2

if __name__ == '__main__':
  from tqdm import tqdm
  
  for i in tqdm(range(100000)):
    ipt = np.ones((270,))
    opt = predict(ipt)
  print(opt)
