# rigid-body

**2D rigid body physics simulation** — impulse-based collision response, rotational dynamics, and gravitational integration.

A from-scratch implementation of Newtonian rigid body mechanics in 2D, covering linear and angular dynamics, box-shaped collision detection against static boundaries, and impulse-based collision resolution with restitution.

## Why It Matters

Rigid body dynamics is the foundation of game physics engines, robotics simulators, and mechanical CAD tools. Understanding the interplay of linear momentum ($m\mathbf{v}$), angular momentum ($I\omega$), and contact impulses is essential for any real-time simulation. This crate provides a minimal, readable implementation that maps directly to the governing equations — no magic numbers, no hidden solvers.

## How It Works

### State Representation

A rigid body is described by:

$$\mathbf{q} = (\mathbf{x}, \theta, \mathbf{v}, \omega, m, I)$$

where $\mathbf{x}$ is position, $\theta$ is orientation, $\mathbf{v}$ is linear velocity, $\omega$ is angular velocity, $m$ is mass, and $I$ is the moment of inertia.

### Moment of Inertia

For a rectangular box with width $w$ and height $h$:

$$I = \frac{m(w^2 + h^2)}{12}$$

This is the standard formula for a uniform-density rectangle rotating about its centroid.

### Time Integration (Semi-Implicit Euler)

Each timestep $\Delta t$:

1. Apply gravity: $v_y \mathrel{-}= g \cdot \Delta t$
2. Integrate position: $\mathbf{x} \mathrel{+=} \mathbf{v} \cdot \Delta t$
3. Integrate angle: $\theta \mathrel{+=} \omega \cdot \Delta t$
4. Check collisions and apply impulses

### Collision Detection

The body's 4 corners are computed via rotation:

$$\mathbf{c}_i = \mathbf{x} + R(\theta) \cdot \mathbf{l}_i$$

where $R(\theta)$ is the 2D rotation matrix and $\mathbf{l}_i$ are local-space corner offsets. Any corner with $y < y_{\text{floor}}$ triggers a collision response.

### Impulse-Based Collision Response

At each penetrating corner $\mathbf{c}$ with surface normal $\mathbf{n}$:

1. Compute contact point velocity: $\mathbf{v}_{\text{contact}} = \mathbf{v} + \omega \times \mathbf{r}$ where $\mathbf{r} = \mathbf{c} - \mathbf{x}$
2. Normal velocity: $v_n = \mathbf{v}_{\text{contact}} \cdot \mathbf{n}$
3. If $v_n < 0$ (approaching), apply impulse:

$$j = \frac{-(1 + e) v_n}{\frac{1}{m} + \frac{(\mathbf{r} \times \mathbf{n})^2}{I}}$$

where $e$ is the coefficient of restitution (0.4 in this implementation).

4. Update velocities: $\mathbf{v} \mathrel{+=} \frac{j \mathbf{n}}{m}$, $\omega \mathrel{+=} \frac{(\mathbf{r} \times j\mathbf{n})}{I}$

### Damping

Angular velocity is damped each step: $\omega \mathrel{*}= 0.999$ to prevent perpetual spin.

**Big-O complexity**: Per timestep is $O(1)$ — fixed 4 corners, constant work. For $N$ bodies, $O(N^2)$ for pairwise collision (not implemented here; single body vs. static floor).

## Quick Start

```bash
cargo run
```

Runs a 300-step simulation (3 seconds at $\Delta t = 0.01$s) of a 2×1 meter, 5 kg box dropped from height with initial rotation. Prints position, angle, and angular velocity every 50 steps.

## API

### `RigidBody`

```rust
let mut body = RigidBody::new(x, y, width, height, mass);
body.step(dt);                           // Advance one timestep
body.apply_impulse(&impulse, &contact);  // Apply impulse at world-space point
let corners: [Vec2; 4] = body.corners(); // Get world-space corners
```

### `Vec2`

2D vector with `dot`, `cross`, `add`, `sub`, `scale`, `len` operations. The 2D cross product returns a scalar: $\mathbf{a} \times \mathbf{b} = a_x b_y - a_y b_x$.

## Architecture Notes

The simulation follows the **γ + η = C** conservation pattern: γ (gravational energy input) combined with η (collision impulse dissipation) must conserve the system's total energy budget C. Energy is injected by gravity, dissipated by inelastic collisions ($e = 0.4$) and angular damping, and the remainder is the stable settled state.

The impulse-based approach is equivalent to solving a linear complementarity problem (LCP) at each contact. For multi-contact scenarios, a sequential impulse solver (like Box2D's) iterates over contacts. This implementation handles one contact at a time, which is sufficient for a single bouncing box.

## References

- **Chris Hecker**, "Rigid Body Dynamics" series, *Game Developer Magazine* (1996–1997)
- **Erin Catto**, "Iterative Methods with Rigid Body" GDC presentations (2005–2014)
- **Box2D** — Erin Catto's 2D physics engine, the industry reference implementation
- **Kenny Erleben**, *Stable, Robust, and Versatile Multibody Dynamics Animation*, PhD thesis (2004)

## License

MIT OR Apache-2.0
