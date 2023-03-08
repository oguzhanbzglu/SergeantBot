import numpy as np
import matplotlib.pyplot as plt

def ackermann_steering(L, T, R):
    inner_l_wheel_angle = np.arctan(L / (R-(T/2)))
    inner_r_wheel_angle = np.arctan(L / (R+(T/2)))
    return inner_l_wheel_angle, inner_r_wheel_angle

def plot_ackermann_steering(L, T):
    R = np.linspace(1, 10, 100)
    print(R)
    inner_l_angle_deg, inner_r_angle_deg = np.degrees(ackermann_steering(L, T, R))
    external_l_angle_deg = np.degrees(np.arctan(T / (R-(L/2))))
    external_r_angle_deg = np.degrees(np.arctan(T / (R+(L/2))))

    fig, ax = plt.subplots()
    ax.plot(R, inner_l_angle_deg, label='Inner Left Wheel')
    ax.plot(R, inner_r_angle_deg, label='Inner Right Wheel')
    ax.plot(R, external_l_angle_deg, label='External Left Wheel')
    ax.plot(R, external_r_angle_deg, label='External Right Wheel')
    ax.set_xlabel('Radius (m)')
    ax.set_ylabel('Angle (deg)')
    ax.legend()
    plt.show()

L = 0.4  # Distance between front and rear wheels
T = 0.15  # Distance between left and right wheels
plot_ackermann_steering(L, T)
