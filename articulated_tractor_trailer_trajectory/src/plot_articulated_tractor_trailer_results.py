from bagpy import bagreader
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

b = bagreader('../bags/topics_2021-09-09-11-10-20.bag')

# states
CURRENT_STATE = b.message_by_topic("/current_state")
df_current_state = pd.read_csv(CURRENT_STATE)

# controls
CURRENT_CONTROL = b.message_by_topic("/current_control")
df_current_control = pd.read_csv(CURRENT_CONTROL)

# reference
CURRENT_SETPOINT = b.message_by_topic("/current_setpoint")
df_current_setpoint  = pd.read_csv(CURRENT_SETPOINT)

# articulation angle
ARTICULATION_MSG = b.message_by_topic('/articulation_joint_controller/command')
df_articulation = pd.read_csv(ARTICULATION_MSG)

# steering angle
STEERING_MSG = b.message_by_topic('/ackerman_steering_joint_controller/command')
df_steering = pd.read_csv(STEERING_MSG)

# velocity control
VELOCITY_MSG = b.message_by_topic('/velocity_joint_controller/command')
df_velocity = pd.read_csv(VELOCITY_MSG)


pos_t_ = df_current_state['Time']
pos_x = df_current_state['data_0']
pos_y = df_current_state['data_1']
angle_gamma = df_current_state['data_3']
angle_phi = df_current_state['data_5']

ref_t_ = df_current_setpoint["Time"]
ref_x = df_current_setpoint['data_0']
ref_y = df_current_setpoint['data_1']

ctrl_t_ = df_current_control["Time"]
ctrl_vf = df_current_control['data_0']
ctrl_w1 = df_current_control['data_1']
ctrl_w2 = df_current_control['data_2']

# manipulate data time to plot data accordingly
pos_t = pos_t_[0:] - pos_t_[0]
ref_t = ref_t_[0:] - ref_t_[0]
ctrl_t = ctrl_t_[0:] - ctrl_t_[0]
art_t = df_articulation.Time[0:] - df_articulation.Time[0]
steer_t = df_steering.Time[0:] - df_steering.Time[0]
vel_t = df_velocity.Time[0:] - df_velocity.Time[0]

# plot states
plt.figure()
plt.plot(pos_t, pos_x, 'b-', label="$x_t$")
plt.xlabel("t [s]", fontsize = 14)
plt.ylabel("$x_t$ [m]", fontsize = 14)
# plt.legend(loc='upper right', bbox_to_anchor=(1.05, 1))
plt.grid()
plt.xlim(-1.0, 158)
plt.savefig('Figs/trailer_control/' + 'xt.pdf')

plt.figure()
plt.plot(pos_t, pos_y, 'b-', label="$y_t$")
plt.xlabel("t [s]", fontsize = 14)
plt.ylabel("$y_t$ [m]", fontsize = 14)
# plt.legend(loc='upper right', bbox_to_anchor=(1.05, 1))
plt.grid()
plt.xlim(-1.0, 158)
plt.savefig('Figs/trailer_control/' + 'yt.pdf')

plt.figure()
plt.plot(pos_t, angle_gamma, 'b-', label="$\gamma$")
plt.xlabel("t [s]", fontsize = 14)
plt.ylabel("$\gamma$ [rad]", fontsize = 14)
# plt.legend(loc='upper right', bbox_to_anchor=(1.05, 1))
plt.grid()
plt.xlim(-1.0, 158)
plt.savefig('Figs/trailer_control/' + 'gamma.pdf')

plt.figure()
plt.plot(pos_t, angle_phi, 'b-', label="$\phi$")
plt.xlabel("t [s]", fontsize = 14)
plt.ylabel("$\phi$ [rad]", fontsize = 14)
# plt.legend(loc='upper right', bbox_to_anchor=(1.05, 1))
plt.grid()
plt.xlim(-1.0, 158)
plt.savefig('Figs/trailer_control/' + 'phi.pdf')

# plot controls
wheel_radius = 0.2
plt.figure()
plt.step(vel_t, df_velocity.data_0[0:] * wheel_radius, 'b-', label="$v_f$")
plt.xlabel("t [s]", fontsize = 14)
plt.ylabel("$v_f$ [m/s]", fontsize = 14)
plt.xlim(-1.0, 158)
plt.grid()
plt.savefig('Figs/trailer_control/' + 'vf.pdf')

plt.figure()
plt.step(ctrl_t, ctrl_w1, 'b-', label="$\omega_1$")
plt.xlabel("t [s]", fontsize = 14)
plt.ylabel("$\omega_1$ [rad/s]", fontsize = 14)
plt.xlim(-1.0, 158)
plt.grid()
plt.savefig('Figs/trailer_control/' + 'omega1.pdf')

plt.figure()
plt.step(ctrl_t, ctrl_w2, 'b-', label="$\omega_2$")
plt.xlabel("t [s]", fontsize = 14)
plt.ylabel("$\omega_2$ [rad/s]", fontsize = 14)
plt.xlim(-1.0, 158)
plt.grid()
plt.savefig('Figs/trailer_control/' + 'omega2.pdf')

# plot xy trajectory
plt.figure()
plt.plot(ref_x, ref_y, 'r-', label="reference")
plt.plot(pos_x, pos_y, 'b--', label="trailer position")
plt.xlabel("$x$ [m]", fontsize = 14)
plt.ylabel("$y$ [m]", fontsize = 14)
plt.legend(loc='upper right', bbox_to_anchor=(1.05, 0.95))
plt.axis('equal')
plt.grid()
plt.savefig('Figs/trailer_control/' + 'xy_trajectory.pdf')

# plot x-error and y-error
plt.figure()
# plt.subplot(2, 1, 1)
plt.plot(pos_t, pos_x - ref_x, 'b', label="$x_t$-error")
plt.xlim(-1.0, 158)
# plt.legend()
plt.ylabel("$e_{x}$ [m]", fontsize = 14)
plt.xlabel("t [s]", fontsize = 14)
plt.grid()
plt.savefig('Figs/trailer_control/' + 'ex.pdf')

#plt.subplot(2, 1, 2)
plt.figure()
plt.plot(pos_t, pos_y - ref_y, 'b', label="$y_t$-error")
plt.xlim(-1.0, 158)
# plt.legend()
plt.ylabel("$e_y$ [m]", fontsize = 14)
plt.xlabel("t [s]", fontsize = 14)
plt.grid()
plt.savefig('Figs/trailer_control/' + 'ey.pdf')

plt.figure()
plt.step(pos_t, angle_gamma, 'b-', label="Articulation angle")
plt.step(pos_t, angle_phi, 'r--', label="Steering angle")
plt.ylabel("Angles [rad]", fontsize = 14)
plt.legend()
plt.xlim(-1.0, 158)
plt.xlabel("t [s]", fontsize = 14)
plt.grid()
plt.savefig('Figs/trailer_control/' + 'phi_gamma.pdf')



plt.show()