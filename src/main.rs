/// 2D Rigid body physics: rotation, torque, collision with walls
use std::f64::consts::PI;

#[derive(Clone)]
struct Vec2(f64, f64);

impl Vec2 {
    fn zero() -> Self { Vec2(0.0, 0.0) }
    fn dot(&self, o: &Vec2) -> f64 { self.0 * o.0 + self.1 * o.1 }
    fn cross(&self, o: &Vec2) -> f64 { self.0 * o.1 - self.1 * o.0 }
    fn add(&self, o: &Vec2) -> Vec2 { Vec2(self.0 + o.0, self.1 + o.1) }
    fn sub(&self, o: &Vec2) -> Vec2 { Vec2(self.0 - o.0, self.1 - o.1) }
    fn scale(&self, s: f64) -> Vec2 { Vec2(self.0 * s, self.1 * s) }
    fn len(&self) -> f64 { self.dot(self).sqrt() }
}

struct RigidBody {
    pos: Vec2,
    vel: Vec2,
    angle: f64,
    ang_vel: f64,
    mass: f64,
    inertia: f64,
    half_ext: Vec2, // half-widths of box
}

impl RigidBody {
    fn new(x: f64, y: f64, w: f64, h: f64, mass: f64) -> Self {
        let inertia = mass * (w * w + h * h) / 12.0;
        RigidBody {
            pos: Vec2(x, y),
            vel: Vec2::zero(),
            angle: 0.3,
            ang_vel: 2.0,
            mass,
            inertia,
            half_ext: Vec2(w / 2.0, h / 2.0),
        }
    }

    fn corners(&self) -> [Vec2; 4] {
        let c = self.angle.cos();
        let s = self.angle.sin();
        let hw = self.half_ext.0;
        let hh = self.half_ext.1;
        let rot = |lx: f64, ly: f64| Vec2(
            self.pos.0 + c * lx - s * ly,
            self.pos.1 + s * lx + c * ly,
        );
        [rot(-hw, -hh), rot(hw, -hh), rot(hw, hh), rot(-hw, hh)]
    }

    fn apply_impulse(&mut self, impulse: &Vec2, contact: &Vec2) {
        let r = contact.sub(&self.pos);
        self.vel = self.vel.add(&impulse.scale(1.0 / self.mass));
        self.ang_vel += r.cross(impulse) / self.inertia;
    }

    fn step(&mut self, dt: f64) {
        // Gravity
        self.vel.1 -= 9.81 * dt;
        // Integrate
        self.pos = self.pos.add(&self.vel.scale(dt));
        self.angle += self.ang_vel * dt;
        // Simple floor collision
        let floor_y = 0.0;
        let corners = self.corners();
        for corner in &corners {
            if corner.1 < floor_y {
                let penetration = floor_y - corner.1;
                self.pos.1 += penetration;
                // Impulse at contact
                let normal = Vec2(0.0, 1.0);
                let vel_at = self.vel.add(&Vec2(-self.ang_vel * (corner.1 - self.pos.1), self.ang_vel * (corner.0 - self.pos.0)));
                let vn = vel_at.dot(&normal);
                if vn < 0.0 {
                    let r = corner.sub(&self.pos);
                    let rn = r.cross(&normal);
                    let denom = 1.0 / self.mass + rn * rn / self.inertia;
                    let j = -(1.0 + 0.4) * vn / denom;
                    self.apply_impulse(&normal.scale(j), corner);
                }
            }
        }
        // Damping
        self.ang_vel *= 0.999;
    }
}

fn main() {
    let mut body = RigidBody::new(5.0, 10.0, 2.0, 1.0, 5.0);

    for step in 0..300 {
        body.step(0.01);
        if step % 50 == 0 {
            let t = step as f64 * 0.01;
            println!("t={t:.2}s  pos=({:.2},{:.2}) angle={:.2}° ang_vel={:.3}",
                body.pos.0, body.pos.1, body.angle * 180.0 / PI, body.ang_vel);
        }
    }
    println!("Rigid body simulation complete.");
}
