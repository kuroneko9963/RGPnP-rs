use crate::bounds::RBound;
use crate::bounds::RBounds;
use crate::corres::Corres;
use glam::Mat3A;
use std::collections::BinaryHeap;

struct SatisfiedBranch<T> {
    lower_bound: u32,
    domain: T,
}

/// Branch-and-Bound (BB/BnB) method
pub fn bnb_rot(initial_rot: impl RBounds, corres: &[Corres], threshold: f32) -> Mat3A {
    let mut solution = SatisfiedBranch {
        lower_bound: 0,
        domain: initial_rot.rotation(),
    };

    let mut queue = BinaryHeap::from([initial_rot]);

    while let Some(bound) = queue.pop() {
        for mut divided in bound.subdivide() {
            divided.bounds(corres, threshold);
            queue.push(divided);
        }

        // update optimal solution
        for branch in queue.iter() {
            if solution.lower_bound < branch.lower() {
                solution.lower_bound = branch.lower();
                solution.domain = branch.rotation();
            }
        }

        queue.retain(|branch| solution.lower_bound <= branch.upper());

        if queue.peek().map(|bound| bound.upper()) == Some(solution.lower_bound) {
            return solution.domain;
        }
    }

    solution.domain
}

/// Branch-and-Bound (BB/BnB) method
pub fn bnb_rot2<R>(initial_rot: Vec<R>, threshold: f32) -> Vec<R>
where
    R: RBound,
{
    let mut solution = SatisfiedBranch {
        lower_bound: 0,
        domain: Mat3A::IDENTITY,
    };
    let mut queue = BinaryHeap::from(initial_rot);
    queue.reserve(4096);

    while let Some(bound) = queue.pop() {
        let mut push_pool = vec![];
        for mut divided in bound.subdivide() {
            divided.compute_bound(threshold);
            push_pool.push(divided);
        }
        queue.extend(push_pool);

        // update optimal solution
        for branch in &queue {
            if solution.lower_bound < branch.lower() {
                solution.lower_bound = branch.lower();
                solution.domain = branch.rotation();
            }
        }

        queue.retain(|branch| solution.lower_bound <= branch.upper());

        if queue.peek().map(|bound| bound.upper()) == Some(solution.lower_bound) {
            break;
        }
    }

    queue
        .into_iter()
        .filter(|bound| bound.upper() == solution.lower_bound)
        .collect()
}

/// Branch-and-Bound (BB/BnB) method
pub fn bnb_rot3(init: Vec<impl RBound>, threshold: f32) -> Mat3A {
    let mut solution = SatisfiedBranch {
        lower_bound: 0,
        domain: Mat3A::IDENTITY,
    };

    let mut queue = BinaryHeap::from(init);
    queue.reserve(4096);

    while let Some(bound) = queue.pop() {
        let mut push_pool = vec![];

        // println!(
        //     "queue={}, upper={}, lower={}",
        //     queue.len(),
        //     bound.upper(),
        //     bound.lower()
        // );

        // Branch op
        for mut divided in bound.subdivide() {
            divided.compute_bound(threshold);

            push_pool.push(divided);
        }
        queue.extend(push_pool);

        // update optimal solution
        for branch in queue.iter() {
            if solution.lower_bound < branch.lower() {
                solution.lower_bound = branch.lower();
                solution.domain = branch.rotation();
            }
        }

        // Bound op
        queue.retain(|branch| solution.lower_bound <= branch.upper());

        if queue.peek().map(|bound| bound.upper()) == Some(solution.lower_bound) {
            return queue.peek().unwrap().rotation();
        }
    }

    solution.domain
}
