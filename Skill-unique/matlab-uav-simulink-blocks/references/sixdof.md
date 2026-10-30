# 6DOF Euler-Angle Plant

Use this reference only when the user requests a higher-fidelity rigid-body plant. Do not replace a working Guidance Model baseline automatically.

## 6DOF (Euler Angles) block contract

The fixed-mass block integrates body-axis force and moment inputs for a flat-Earth rigid body.

Core inputs:

- `Fxyz`: body-axis applied force `[Fx Fy Fz]`, N.
- `Mxyz`: body-axis applied moment `[Mx My Mz]`, N*m.

Configure metric units, mass, inertia tensor, initial earth-frame position and velocity, initial Euler angles, and initial body angular rates. Keep all of these in named model-workspace variables.

Relevant outputs include earth-frame velocity and position, Euler angles, direction cosine matrix, body-frame velocity, and body angular rates. R2025a and later removed several acceleration outputs, so do not copy a pre-R2025a port map without checking the installed release.

## Force-axis step test

For the simple one-axis demonstration:

1. Set gravity and trim forces consistently. Decide whether the test is free-flight, gravity-compensated, or perturbation about hover.
2. Apply a bounded step to one body-force axis only.
3. Hold moments at zero unless attitude stabilization is intentionally included.
4. Log body velocity, earth velocity, position, attitude, and body rates.
5. Explain cross-axis motion caused by attitude or frame transformation instead of assuming each body force maps to one earth position axis.

## Integration with the waypoint demo

A raw 6DOF plant is not a drop-in replacement for Guidance Model. It requires:

- an attitude controller producing moments;
- a thrust/force allocator;
- gravity compensation;
- actuator and saturation assumptions;
- state conversion back to the NED and quaternion signals expected by scenario blocks.

Keep this integration in a separate model until its trim and single-axis tests pass.

Official reference:

- https://www.mathworks.com/help/aeroblks/6dofeulerangles.html

