# Rigid Body Dynamics

A **rigid body** is an idealized solid object whose deformation is negligible under applied forces. This crate simulates 2D rigid body physics — translation, rotation, impulse-based collisions, and gravitational integration — using Newton-Euler equations of motion.

## Why It Matters

Rigid body dynamics is the foundation of every game physics engine, robotics simulator, and CAD motion analysis tool. Whether you're building a platformer, simulating a robotic arm, or analyzing vehicle crash dynamics, you need to integrate forces, resolve collisions, and conserve angular momentum. This crate demonstrates the complete pipeline: Euler integration of linear and angular velocity, impulse-based collision response with restitution, and the moment of inertia tensor for rectangular bodies.

## How It Works

The state of a 2D rigid body is described by position **p**, velocity **v**, angle θ, and angular velocity ω. The equations of motion are Newton's second law for translation and its rotational analogue:

```
F = m·a        →    v₁ = v₀ + (F/m)·Δt
τ = I·α        →    ω₁ = ω₀ + (τ/I)·Δt
```

For a rectangular box of mass m, width w, and height h, the **moment of inertia** about the center is:

```
I = m·(w² + h²) / 12
```

**Integration** uses the semi-implicit Euler method: update velocities first, then positions. Gravity adds `−g·Δt` to the vertical velocity each step. This is O(1) per body per timestep.

**Collision response** uses impulse-based resolution. When a corner penetrates the floor, the crate computes the contact normal **n**, the relative velocity at the contact point (including angular contribution `v_contact = v + ω × r`), and applies an impulse:

```
j = −(1 + e) · (v·n) / (1/m + (r × n)² / I)
```

where e is the coefficient of restitution (0.4 here, meaning 40% energy returned). The angular velocity changes by `(r × j·n) / I`, causing the box to tumble realistically. Each timestep is O(n) in the number of corners checked.

## Quick Start

```rust
// Simulate a 2×1 meter box falling under gravity
fn main() {
    let mut body = rigid_body::RigidBody::new(5.0, 10.0, 2.0, 1.0, 5.0);
    for step in 0..300 {
        body.step(0.01); // 10ms timestep
    }
    println!("Final position: ({:.2}, {:.2})", body.pos.0, body.pos.1);
}
```

## API

| Type | Description |
|------|-------------|
| `Vec2` | 2D vector with dot, cross, add, sub, scale, len |
| `RigidBody` | Physical body with position, velocity, angle, angular velocity, mass, inertia |
| `RigidBody::new(x, y, w, h, mass)` | Create a box at (x,y) with dimensions w×h |
| `RigidBody::step(&mut self, dt)` | Advance simulation by dt seconds |
| `RigidBody::apply_impulse(&mut self, impulse, contact)` | Apply force impulse at a contact point |
| `RigidBody::corners(&self)` | World-space corner positions after rotation |

## Architecture Notes

This crate provides the physics layer for SuperInstance's spatial reasoning. Within the γ + η = C framework (grounding γ, elasticity η, coherence C), rigid body dynamics models **γ** — the physical grounding constraints that govern how virtual objects behave under realistic laws of motion. See [ARCHITECTURE.md](https://github.com/SuperInstance/SuperInstance/blob/main/ARCHITECTURE.md).

## References

1. Millington, I. *Game Physics Engine Development*. CRC Press, 2007.
2. Catto, E. "Iterative Dynamics with Temporal Coherence." GDC 2005.
3. Featherstone, R. *Rigid Body Dynamics Algorithms*. Springer, 2008.

## License

MIT
