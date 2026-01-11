# Camera calibration algorithm (planar target)

This document explains the calibration pipeline implemented in `calib_eigen_modern.cpp` for a planar calibration target.

---

## What calibration is doing, in one sentence
Calibration estimates the camera **intrinsic parameters** (mapping from normalized rays to pixels) and per-view **extrinsic parameters** (pose of the target relative to the camera) by fitting a geometric projection model to observed pixel coordinates.

---

## Closed-form planar initialization (homography + Zhang intrinsics + pose decomposition)

### 1) Why a homography is sufficient for a planar target
If the target is planar and you define target coordinates as \((X,Y,0)\), the mapping to image pixels \((u,v)\) is a 3×3 projective transform:

\[
\lambda \begin{bmatrix} u\\ v\\ 1 \end{bmatrix} =
H \begin{bmatrix} X\\ Y\\ 1 \end{bmatrix}
\]

Each view is summarized by a homography \(H\). With enough point correspondences, \(H\) is estimated with **DLT (Direct Linear Transform)**. The implementation uses **coordinate normalization** (centering and scaling so mean distance is \(\sqrt{2}\)) before building the DLT system, because this improves conditioning and reduces numerical error in the SVD solution.

### 2) Intrinsics from homographies (Zhang’s method)
Let the intrinsic matrix be:

\[
K =
\begin{bmatrix}
f_x & \gamma & c_x\\
0 & f_y & c_y\\
0 & 0 & 1
\end{bmatrix}
\]

For a planar target, the homography can be written (up to scale) as:

\[
H \sim K [r_1\ r_2\ t]
\]

where \(r_1,r_2\) are the first two columns of the rotation matrix \(R\) and \(t\) is translation. Since \(r_1\) and \(r_2\) are orthonormal, you have:

- \(r_1^\top r_2 = 0\)
- \(r_1^\top r_1 = r_2^\top r_2\)

Zhang rewrites these constraints as linear equations in the entries of the symmetric matrix:

\[
B = K^{-\top}K^{-1}
\]

Each view contributes two linear constraints on the 6 independent entries of \(B\). Stacking constraints across multiple views yields a homogeneous system \(Vb=0\), solved via SVD. From the recovered \(B\), one reconstructs \((f_x,f_y,c_x,c_y,\gamma)\) algebraically.

Why multiple views matter: each view adds only two constraints, and the constraints become informative only when the plane is observed under sufficiently different tilts and positions.

### 3) Per-view extrinsics from homography decomposition
Given \(K\) and \(H=[h_1\ h_2\ h_3]\), compute:

\[
\tilde r_1 = K^{-1}h_1,\quad \tilde r_2 = K^{-1}h_2,\quad \tilde t = K^{-1}h_3
\]

Choose a scale factor so that \(r_1, r_2\) have unit norm, set \(r_3 = r_1 \times r_2\), and form \(R=[r_1\ r_2\ r_3]\). Because noise makes \(R\) imperfect, the code projects it back to the nearest valid rotation matrix using SVD:

\[
R \leftarrow U V^\top
\]

and enforces \(\det(R)=+1\). This yields a consistent initial pose per view.

### Limits of the closed-form planar initialization
This stage fits projective geometry well but does not explicitly enforce a full lens model. In particular, if distortion is non-negligible or the measurements are noisy, the initialization will typically leave systematic residual structure that can be removed only by fitting a physical camera model directly.

---

## Nonlinear refinement via bundle adjustment with a distortion-aware camera model

### 1) Forward model used for refinement
For each target point \(\mathbf{X}\) on the plane:

1) Transform into the camera frame:
\[
\mathbf{X}_c = R\mathbf{X}+t
\]

2) Normalize:
\[
x = X_c/Z_c,\quad y = Y_c/Z_c
\]

3) Apply lens distortion (radial + tangential) with \(r^2=x^2+y^2\):
- radial (two coefficients \(k_1,k_2\)):
\[
x \leftarrow x(1+k_1r^2+k_2r^4),\quad
y \leftarrow y(1+k_1r^2+k_2r^4)
\]
- tangential (two coefficients \(p_1,p_2\)):
\[
x \leftarrow x + 2p_1xy + p_2(r^2+2x^2)
\]
\[
y \leftarrow y + p_1(r^2+2y^2) + 2p_2xy
\]

4) Map to pixels:
\[
u = f_x x + \gamma y + c_x,\quad
v = f_y y + c_y
\]

In the refinement stage, the code fixes \(\gamma=0\), which is standard for most modern sensors and reduces parameter coupling.

### 2) Objective: global reprojection error
The refinement minimizes summed squared reprojection error across all views and points:

\[
\min_{\theta}\sum_{v,i}\left\lVert
\mathbf{x}_{v,i} - \pi(\mathbf{X}_i;\theta)
\right\rVert^2
\]

where \(\theta\) includes:
- intrinsics \((f_x,f_y,c_x,c_y)\),
- distortion \((k_1,k_2,p_1,p_2)\),
- per-view poses \((R_v,t_v)\) represented as Rodrigues rotation vectors plus translations.

Optimizing all parameters jointly is the standard form of camera-calibration **bundle adjustment**.

### 3) Solver: Levenberg–Marquardt (LM)
The problem is nonlinear due to rotation and distortion. LM is used because it is robust for least-squares objectives of this type. At each iteration:

- evaluate residual vector \(r(\theta)\),
- form Jacobian \(J(\theta)\),
- solve the damped system:
\[
(J^\top J + \lambda I)\Delta = -J^\top r
\]
- test \(\theta'=\theta+\Delta\); accept if the cost decreases, otherwise increase \(\lambda\) and try again.

LM smoothly transitions between Gauss–Newton behavior (small \(\lambda\), fast near the optimum) and gradient-descent-like behavior (large \(\lambda\), safer far from the optimum).

In this implementation the Jacobian is computed by central finite differences (numeric Jacobian). That is computationally heavier than an analytic Jacobian, but straightforward and reliable for demonstration and testing.

### 4) Continuation strategy for parameter activation
To reduce early-stage parameter coupling, the code uses a continuation schedule:

- optimize intrinsics + poses with distortion held at zero,
- then release radial coefficients \((k_1,k_2)\),
- then release tangential coefficients \((p_1,p_2)\).

This prevents distortion terms from prematurely absorbing pose/intrinsic errors and typically improves convergence and final accuracy.

### 5) Practical safeguards for stability
The implementation includes additional stability measures commonly used in calibration pipelines:
- **parameter clamping** to keep focal lengths, principal point, and distortion coefficients within plausible bounds,
- **adaptive damping** (increase/decrease \(\lambda\) based on improvement),
- **bounded step size** to avoid destabilizing jumps,
- **rotation orthonormalization** during initialization to ensure valid starting poses.
