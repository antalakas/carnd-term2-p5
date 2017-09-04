#### The Model

The state consists of the x, y coordinates, the orientation angle, the velocity,as well as
the cross track error and the psi error (px, py, psi, v, cte, epsi as givn by the simulator).
The equations of the kinematic model are as follows (taken from the lesson):

![alt text](./eqns.png "Model Equations")

The **kinematic model** combines the state from the previous step (initially from the simulator)
as well as the actuations (delta: steering angle, a: throttle) to calculate the state of the
current timestep.

The **timestep length** is set to 10 and the dt (elapsed duration) to 0.1. The values used come
from the Q&A session, and represent a good tradeoff between the accuracy of the trajectory
followed and the time needed for caclulations in each step, actually value of N=20, 25 and
dt=0.05 were used leading to errors.

**Polynomial Fitting and MPC Preprocessing** : The waypoints are preprocessed in order to transform
them to the vehicles coordinate system (essentially rotating left by psi, main.cpp, lines 118-126).
In this way the calculations are simplified and the initial v = 0, psi = 0.

In order to handle **Model Predictive Control with Latency**, a step is introduced before the execution
of MPC to set the initial state to the expected state after 100ms (main.cpp, lines 144-149)


#### Further dicussion

A third order polynomial is used to fit the waypoints and its derivative is used in FG_eval to calculate
initial constraints for psi(s)

The solver is implemented as in the lesson, while in FG_Eval the constraints ar altered by multiplicating
them with appropriate values (also taken from Q&A session, with the exception of sequential actuations
for delta where value 400 is used: this enables to reach constant throttle of 0.6 without leaving the track)
Probably, cte has to be used in combination with a PID for the throttle in order to achieve better overall
performance (it is logicall to lower throttle inside turns or S's)

In line 187 of main.cpp the steer value is negated as suggested in tips and tricks, while the constraints
for the psi (25 degrees) are multiplied by Lf to trake into account the vehicle's turning radius.
(main.cpp, line 183)
