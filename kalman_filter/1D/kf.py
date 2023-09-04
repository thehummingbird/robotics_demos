import numpy as np
from matplotlib import pyplot as plt


class KalmanFilter(object):
  """
  Kalman Filter using a constant acceleration motion model
  """

  def __init__(self, dt, u, sigma_a, sigma_z):
    self.dt = dt
    self.u = u
    self.sigma_a = sigma_a
    self.sigma_z = sigma_z
    self.A = np.matrix([[1, self.dt], [0, 1]])
    self.B = np.matrix([[(self.dt**2)/2], [self.dt]])
    self.R = np.matrix(
        [
            [(self.dt**4)/4, (self.dt**3)/2],
            [(self.dt**3)/2, self.dt**2]
        ]
    ) * self.sigma_a ** 2
    self.Q = sigma_z ** 2
    self.P = np.eye(self.A.shape[1])
    self.C = np.matrix([[1, 0]])
    self.x = np.matrix([[0], [0]])
    self.I = np.eye(self.C.shape[1])

  def predict(self):
    self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)
    self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.R
    return self.x

  def update(self, z):
    S = np.dot(self.C, np.dot(self.P, self.C.T)) + self.Q
    K = np.dot(self.P, np.dot(self.C.T, np.linalg.inv(S)))
    self.x = self.x + K*(z-np.dot(self.C, self.x))
    self.P = np.dot((self.I - (K*self.C)), self.P)


def main():
  dt = 0.1

  t = np.arange(0, 100, dt)

  # real
  real_x = 0.1*((t**2)-t)

  u = 1
  sigma_a = 0.25
  sigma_z = 2.0
  kf = KalmanFilter(dt, u, sigma_a, sigma_z)

  predictions = []
  measurements = []

  for x in real_x:
    z = kf.C.item(0)*x + np.random.normal(0, 50)

    measurements.append(z)
    x_kf = kf.predict().item(0)
    kf.update(z)

    predictions.append(x_kf)

  fig = plt.figure()

  plt.plot(t, measurements, label='Measurements', color='b', linewidth=0.5)
  plt.plot(t, np.array(real_x), label='Real Track',
           color='y', linewidth=1.5)
  plt.plot(t, predictions,
           label='Kalman Filter Prediction', color='r', linewidth=1.5)
  plt.show()


if __name__ == "__main__":
  main()
