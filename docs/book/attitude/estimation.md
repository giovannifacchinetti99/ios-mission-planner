# Attitude determination and estimation

The control law of the previous chapter uses the attitude and the angular
velocity as if they were known. On a spacecraft they are not: they are
reconstructed from sensors, each of which measures something less than the
attitude and measures it with an error. This chapter covers the two steps
of that reconstruction. The first determines the attitude at one instant
from directions observed at that instant. The second combines those
determinations with a gyroscope over time, in a recursive filter that also
learns the bias of the gyroscope.

## What the sensors measure

Most attitude sensors measure a direction. A sun sensor gives the direction
of the Sun in body axes, a magnetometer the direction of the geomagnetic
field, a star tracker the directions of several stars. Each observation is
a unit vector $\vect b$ in the body frame, and the same direction is known
in the inertial frame, $\vect r$, from an ephemeris, a field model or a
star catalogue. If $R$ is the rotation matrix from inertial to body axes,
an ideal observation satisfies $\vect b = R\,\vect r$.

A single direction does not determine the attitude. It fixes two of the
three degrees of freedom and leaves free the rotation about the observed
direction itself. Two directions that are not parallel are the minimum.

## Two vectors: TRIAD

With two observations $(\vect b_1,\vect r_1)$ and $(\vect b_2,\vect r_2)$,
the TRIAD method builds one orthonormal triad from the two body vectors and
another, in the same way, from the two reference vectors:

$$
\hat{\vect t}_1 = \vect b_1, \qquad
\hat{\vect t}_2 = \frac{\vect b_1\times\vect b_2}{|\vect b_1\times\vect
b_2|}, \qquad
\hat{\vect t}_3 = \hat{\vect t}_1\times\hat{\vect t}_2 ,
$$

and likewise $\hat{\vect s}_1$, $\hat{\vect s}_2$, $\hat{\vect s}_3$ from
$\vect r_1$ and $\vect r_2$. The attitude is the rotation that takes one
triad into the other,

$$
R = \begin{bmatrix}\hat{\vect t}_1 & \hat{\vect t}_2 & \hat{\vect
t}_3\end{bmatrix}
\begin{bmatrix}\hat{\vect s}_1 & \hat{\vect s}_2 & \hat{\vect
s}_3\end{bmatrix}^{\mathsf T}.
$$

The method is exact when the observations are exact. With noise it treats
the two vectors differently: the first is reproduced exactly and the second
only fixes the rotation about the first, so the more accurate sensor should
come first.

## Many vectors: Wahba's problem

With more than two observations, using only two discards information. The
general problem, stated by Wahba in 1965, is to find the rotation that
minimizes the weighted sum of the squared residuals,

$$
L(R) = \frac12\sum_i w_i\,\left|\vect b_i - R\,\vect r_i\right|^2 ,
$$

with weights $w_i$ that express the confidence in each observation,
ideally the inverse of its variance. Expanding the square shows that
minimizing $L$ is the same as maximizing $\mathrm{tr}(R\,B^{\mathsf T})$,
with the *attitude profile matrix*

$$
B = \sum_i w_i\,\vect b_i\,\vect r_i^{\mathsf T}.
$$

The solution follows from the singular value decomposition $B = U\,\Sigma\,
V^{\mathsf T}$:

$$
R = U\,\mathrm{diag}\!\left(1,\,1,\,\det U\,\det V\right)V^{\mathsf T},
$$

where the last factor guarantees a proper rotation, with determinant plus
one. The well-known QUEST algorithm solves the same problem through the
largest eigenvalue of a $4\times4$ matrix built from $B$, a formulation
devised when a full decomposition was too expensive for an on-board
computer. The two give the same optimum.

The error of the estimate decreases as observations are added, roughly as
the inverse square root of their number for equal accuracies, and it
depends on the geometry: directions close to each other determine poorly
the rotation about their common direction.

## From rotation matrix to quaternion

The methods above return a rotation matrix, while the dynamics and the
control use a quaternion. The conversion extracts the four components from
the diagonal and the antisymmetric part of the matrix. Four algebraically
equivalent formulas exist, each dividing by a different component of the
quaternion; the numerically safe choice, due to Shepperd, is to use the one
that divides by the largest component, so that no division by a small
number ever occurs.

## Estimation over time: the gyroscope and its bias

Determining the attitude from scratch at every instant ignores what was
known a moment before. A gyroscope provides exactly that link: it measures
the angular velocity continuously, and integrating the kinematic equation
with it propagates the attitude between observations. Its measurement is
corrupted by noise and, more importantly, by a bias that changes slowly and
is not known:

$$
\tilde{\vect\omega} = \vect\omega + \vect\beta + \vect\eta_v, \qquad
\dot{\vect\beta} = \vect\eta_u ,
$$

with white noises $\vect\eta_v$ and $\vect\eta_u$. Integrating the
gyroscope alone accumulates the bias as an attitude error that grows
linearly with time, a drift of degrees within minutes for a typical small
sensor. Observed directions have no drift but are noisy and intermittent.
A Kalman filter combines the two and estimates the bias along the way.

## The multiplicative extended Kalman filter

A Kalman filter keeps an estimate of the state and the covariance of its
error. The quaternion poses a difficulty: its four components are tied by
the unit norm, so their $4\times4$ covariance would be singular, and adding
a correction to a quaternion would break the norm. The multiplicative
extended Kalman filter (MEKF) solves both problems by keeping two things
apart. The estimate is a full quaternion $\hat{\vect q}$. The error is a
small rotation, parametrized by three angles $\delta\vect\theta$, that
relates the estimate to the truth by a product,

$$
\vect q = \hat{\vect q}\otimes\delta\vect q(\delta\vect\theta), \qquad
\delta\vect q \approx \begin{bmatrix}1\\ \tfrac12\,\delta\vect\theta
\end{bmatrix}.
$$

The filter state has six components, the attitude error
$\delta\vect\theta$ and the bias error $\delta\vect\beta$, with a
$6\times6$ covariance $P$.

In the *prediction* step the estimate is propagated with the gyroscope
reading corrected by the estimated bias, $\hat{\vect\omega} =
\tilde{\vect\omega} - \hat{\vect\beta}$, using the exact solution of the
kinematic equation for a constant rate over the step. The error obeys the
linearized equations

$$
\delta\dot{\vect\theta} = -\hat{\vect\omega}\times\delta\vect\theta -
\delta\vect\beta - \vect\eta_v, \qquad
\delta\dot{\vect\beta} = \vect\eta_u ,
$$

in which the first term is the rotation of the error with the body, the
second is the bias error feeding the attitude error, and the noises drive
the growth of the covariance. Discretized over the step, they give the
transition matrix $F$ and the process noise $Q$ of the usual covariance
propagation, $P \leftarrow F\,P\,F^{\mathsf T} + Q$.

In the *update* step an attitude determined from observed directions,
$\vect q_m$, is compared with the estimate. The residual is the small
rotation between the two, expressed in body axes,

$$
\delta\vect q = \hat{\vect q}^{-1}\otimes\vect q_m, \qquad
\vect y = 2\,\delta\vect q_v ,
$$

and since it measures the attitude error directly, the measurement matrix
is $H = [\,\mathbf 1 \;\; \mathbf 0\,]$. The Kalman gain and the corrections
follow from the standard equations,

$$
K = P\,H^{\mathsf T}\left(H\,P\,H^{\mathsf T} + R_m\right)^{-1}, \qquad
\begin{bmatrix}\delta\hat{\vect\theta}\\ \delta\hat{\vect\beta}
\end{bmatrix} = K\,\vect y, \qquad
P \leftarrow (\mathbf 1 - K\,H)\,P ,
$$

with $R_m$ the covariance of the measured attitude. The bias is corrected
by addition. The attitude is corrected by multiplication, $\hat{\vect q}
\leftarrow \hat{\vect q}\otimes\delta\vect q(\delta\hat{\vect\theta})$,
followed by a normalization; this *reset* moves the correction into the
quaternion and leaves the error state at zero for the next cycle.

The bias is never measured. It becomes observable because, through the
error equations, a bias error produces an attitude error that grows in a
recognizable way, and the gain uses the correlation between the two that
the covariance accumulates during the prediction. One detail of the
implementation deserves attention: the residual must be formed in the same
frame in which the error is defined. With the error defined in body axes,
as here, the residual is $\hat{\vect q}^{-1}\otimes\vect q_m$ and the
correction multiplies on the right. Mixing the conventions gives a filter
that seems to work on one axis and fails on the others.

## In the code

The single-instant methods are in `dynamics/attitude/determination.py`.
`triad_attitude` and `wahba_svd_attitude` return the rotation matrix from
inertial to body axes, the first from two pairs of vectors and the second
from any number of pairs with optional weights.
`rotation_matrix_to_quaternion` converts the result to the quaternion
convention of the package with Shepperd's method, and
`quaternion_to_rotation_matrix` is its inverse.

The filter is in `dynamics/attitude/filter.py`. `MEKFState` holds the
estimated quaternion, the estimated bias and the covariance. `mekf_predict`
advances it by one step from a gyroscope reading and the two noise
densities, and `mekf_update` corrects it with a measured quaternion and its
covariance, including the reset.

## In the notebooks

{doc}`../examples/11_attitude_determination/attitude_determination`
recovers a known attitude exactly with TRIAD from two noiseless vectors,
then adds noise and shows how the error of the solution of Wahba's problem
decreases with the number of vectors and grows with the noise, and how the
weights let an accurate sensor prevail over a poor one.

{doc}`../examples/12_attitude_estimation_filter/attitude_estimation_filter`
simulates a gyroscope with an unknown bias and noisy vector observations.
Integrating the gyroscope alone, the attitude error grows to tens of
degrees within minutes; the filter, given the same data, stays within a
fraction of a degree and its estimate of the bias converges to the true
value on all three axes.

## References

1. F. L. Markley, J. L. Crassidis, *Fundamentals of Spacecraft Attitude
   Determination and Control*, Springer, 2014.
2. G. Wahba, "A least squares estimate of satellite attitude," *SIAM
   Review*, 7(3), 1965.
3. F. L. Markley, "Attitude determination using vector observations and
   the singular value decomposition," *Journal of the Astronautical
   Sciences*, 36(3), 1988.
4. E. J. Lefferts, F. L. Markley, M. D. Shuster, "Kalman filtering for
   spacecraft attitude estimation," *Journal of Guidance, Control, and
   Dynamics*, 5(5), 1982.
5. S. W. Shepperd, "Quaternion from rotation matrix," *Journal of
   Guidance and Control*, 1(3), 1978.
