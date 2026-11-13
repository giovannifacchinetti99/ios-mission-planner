# Relative navigation

The guidance of an approach needs the position and the velocity of the
deputy with respect to the chief. No sensor measures them. The relative
sensor of {doc}`simulator` gives a distance and a direction, in the axes of
a body whose attitude is itself an estimate, with an error that changes by
orders of magnitude along the approach, and it gives no velocity at all.
The navigation is the function that turns that stream of imperfect
measurements into an estimate of the relative state, and into an honest
statement of how wrong that estimate may be. This chapter describes it.

## From range and direction to a position

The sensor sees the vector from itself to a target fixed on the chief:
$\rho\,\hat{\vect u}$ in the body axes of the deputy. The target is at a
known position $\vect p_t$ in the LVLH frame of the chief, and the sensor
at a known position $\vect s_b$ in the body of the deputy. Rotating the
measured vector to the LVLH frame with the estimated attitude gives the
relative position of the center of mass of the deputy,

$$
\vect z = \vect p_t - \hat R_{lb}\left(\rho\,\hat{\vect u} + \vect
s_b\right).
$$

This is a nonlinear function of what was actually measured, so its error is
not the same in every direction. Along the line of sight it is the range
error. Across the line of sight it is the angular error multiplied by the
distance, to which the error of the attitude estimate adds in the same way.
The covariance of the converted measurement keeps this shape,

$$
R_z = \hat R_{lb}\left[\sigma_\rho^2\,\hat{\vect u}\hat{\vect u}^{\mathsf
T} + \rho^2\left(\sigma_\theta^2 + \sigma_{att}^2\right)\left(I -
\hat{\vect u}\hat{\vect u}^{\mathsf T}\right)\right]\hat R_{lb}^{\mathsf
T},
$$

an ellipsoid elongated along the line of sight when the range error
dominates and flattened when it does not. Giving the filter this covariance
instead of a sphere lets it trust each direction as much as it deserves. At
5 km, with the sensor of the reference mission, the measurement is good to
a few metres across the line of sight and to tens of metres along it; at
20 m it is good to a centimetre in every direction.

## The filter

A single measurement is noisy and says nothing about the velocity. The
relative motion, on the other hand, follows known equations. A Kalman
filter combines the two: it propagates the estimate with the equations of
motion, and corrects it with each measurement by an amount that depends on
how much it trusts one and the other.

The model is the linear system of {doc}`../relative/hcw`, driven by the
acceleration that the thrusters were commanded to produce. For a command
held constant over the step $\Delta t$ the discretization is exact,

$$
\vect x_{k+1} = \Phi\,\vect x_k + \Gamma\,\vect u_k , \qquad
\Gamma = \int_0^{\Delta t}\Phi(s)\,B\,\mathrm ds ,
$$

with $\Phi$ the transition matrix over one step and $\Gamma$ available in
closed form. The filter carries an estimate $\hat{\vect x}$ and its
covariance $P$. Between measurements,

$$
\hat{\vect x}^- = \Phi\,\hat{\vect x} + \Gamma\,\vect u , \qquad
P^- = \Phi P\,\Phi^{\mathsf T} + Q ,
$$

and at each measurement of the position, with $H = [\,I\;\;0\,]$,

$$
K = P^- H^{\mathsf T}\left(H P^- H^{\mathsf T} + R_z\right)^{-1}, \qquad
\hat{\vect x} = \hat{\vect x}^- + K\left(\vect z - H\hat{\vect
x}^-\right), \qquad P = (I - KH)\,P^- .
$$

The velocity is never measured. It is observed through the model: a
velocity error makes the predicted position drift away from the measured
one, the correlation between position and velocity stored in $P$ grows,
and the gain uses it to correct the velocity from position residuals. The
covariance update is implemented in the Joseph form, algebraically
equivalent to the one above and guaranteed to stay symmetric and positive
definite in finite precision.

## Process noise: what the model does not know

The matrix $Q$ is where the filter is told how far to trust its model, and
it decides whether the covariance is honest. Three effects enter it.

The first is everything that perturbs the two spacecraft differently,
chiefly drag. It is small and is represented by a white acceleration of
constant spectral density.

The second is the error of the thrusters. The filter propagates with the
commanded acceleration, and the acceleration produced differs by a few
percent. The uncertainty added during a burn is therefore proportional to
the command, and enters through the input matrix $\Gamma$, because it is
constant over the step and not white.

The third is the model itself. The linear equations drop the second-order
terms of the gravity gradient, whose magnitude at a distance $\rho$ from
the chief, on an orbit of radius $r$, is of the order of

$$
a_{nl} \simeq \frac{3}{2}\,n^2\,\frac{\rho^2}{r}.
$$

At 5 km this is about $7\times10^{-6}$ m/s², larger than any perturbation,
and over the half hour of a transfer it displaces the deputy by metres with
respect to the prediction. It is a slowly varying error, not a noise. The
filter represents it as a white acceleration of the same magnitude with a
correlation time $1/n$, which gives a term of $Q$ that grows with the
fourth power of the distance and disappears near the chief. Without it the
filter becomes overconfident exactly where its model is worst: the estimate
is still accurate, but the covariance claims an accuracy that is not there,
and everything downstream that relies on the covariance is misled.

The test of an honest filter is statistical. The actual error must stay
inside three standard deviations of the filter almost all the time, and
must not sit far inside them either, which would mean that information is
being thrown away.

## Outliers and divergence

Before using a measurement the filter compares the residual with what it
expected. The normalized innovation squared,

$$
\epsilon = \left(\vect z - H\hat{\vect x}^-\right)^{\mathsf T}\left(H P^-
H^{\mathsf T} + R_z\right)^{-1}\left(\vect z - H\hat{\vect x}^-\right),
$$

is small when the measurement agrees with the prediction within their
combined uncertainty. A measurement whose $\epsilon$ exceeds a threshold is
rejected as an outlier.

This protection has a failure mode. If the deputy feels an acceleration
that the filter does not know about, a thruster stuck open for example, the
prediction moves away from the truth, every measurement looks like an
outlier, every measurement is rejected, and the estimate diverges further.
The measurements were right and the estimate was wrong. The filter
therefore counts consecutive rejections. One rejected measurement is an
outlier; three in a row mean that it is the estimate that should be
doubted, and at that point the filter enlarges its covariance to the size
of the disagreement and accepts the measurement. The estimate then catches
up within a few steps.

## The attitude

The attitude is estimated by the multiplicative filter of
{doc}`../attitude/estimation`, unchanged: the gyro propagates the
attitude, the star tracker corrects it, and the bias of the gyro is
estimated along the way. Its output enters the relative navigation twice,
in the rotation $\hat R_{lb}$ that converts the measurement and in the
rotation that turns the commanded body force into the acceleration used for
the prediction.

## One stated simplification

The LVLH frame in which all of this is expressed is built from the orbit of
the chief, and the simulation gives it to the on-board functions without
error. A real deputy would know it from the navigation data of the chief or
from its own absolute navigation, with errors of metres and of millimetres
per second, which would tilt the frame by a negligible angle. The orbit of
the deputy itself is never needed: everything is relative.

## In the code

`relative_position_from_measurement` in `gnc/navigation.py` converts a
measurement of the relative sensor into a position in the LVLH frame with
its covariance. `RelativeNavigationFilter` is the Kalman filter: `predict`
advances the estimate with the commanded acceleration and adds the three
terms of the process noise, and `update` applies a position measurement,
with the innovation gate and the recovery from divergence. The discrete
matrices come from `cw_discrete_matrices` in `dynamics/relative/cw.py`, in
closed form and checked against the matrix exponential.

The attitude filter is the one of `dynamics/attitude/filter.py`,
`mekf_predict` and `mekf_update`.

## In the notebook

{doc}`../examples/15_closed_loop_docking/closed_loop_docking` compares the
error of the estimated relative position with the three-sigma bound of the
filter along the whole approach, on a logarithmic scale that covers the
four orders of magnitude between the first hold point and the contact. It
also shows the convergence of the gyro bias estimate and the level of the
attitude knowledge error.

## References

1. Y. Bar-Shalom, X. R. Li, T. Kirubarajan, *Estimation with Applications
   to Tracking and Navigation*, Wiley, 2001.
2. W. Fehse, *Automated Rendezvous and Docking of Spacecraft*, Cambridge
   University Press, 2003.
