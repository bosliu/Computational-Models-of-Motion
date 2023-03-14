//
// Created by Dongho Kang on 25.04.22.
//

#ifndef A5_RBSYMPLECTICENGINE_H
#define A5_RBSYMPLECTICENGINE_H

#include "sim/RBEngine.h"

namespace crl {

/**
 * simulation world implemented with symplectic integration scheme.
 */
class RBSymplecticEngine : public RBEngine {
public:
    RBSymplecticEngine() : RBEngine() {}

    ~RBSymplecticEngine() override = default;

    void step(double dt) override {
        // external force and torque
        updateForceForGravity();
        updateForceAndTorqueForSprings();

        // update states of rbs (integration)
        for (uint i = 0; i < rbs.size(); i++) {
            RB *rb = rbs[i];
            // retrieve saved force and tau
            V3D f = f_ext[i];
            V3D tau = tau_ext[i];

            // TODO: Ex.3 Stable Simulation
            // why our simulation is blown up? let's make it more stable using
            // symplectic Euler integration!

             rb->state.velocity += dt * f / rb->rbProps.mass;
            Matrix3x3 R = rb->state.orientation.toRotationMatrix();
            Matrix3x3 inertiaWorld = R * rb->rbProps.MOI_local * R.transpose();
            rb->state.angularVelocity += dt * inertiaWorld.inverse() * (tau - rb->state.angularVelocity.cross(V3D(inertiaWorld * rb->state.angularVelocity)));
           
            //rb->state.velocity = V3D();         // TODO: change this!
            //rb->state.angularVelocity = V3D();  // TODO: change this!

            if (simulateCollisions && rb->rbProps.collision)
                updateVelocityAfterCollision(rb);

            
           
            rb->state.pos = rb->state.pos + dt * rb->state.velocity;
            rb->state.orientation = updateRotationGivenAngularVelocity(rb->state.orientation, rb->state.angularVelocity, dt);
            // TODO: Ex.3 Stable Simulation
            // implement forward (explicit) Euler integration scheme for computing pose.
            //rb->state.pos = rb->state.pos + V3D() * dt;  // TODO: change this!
            //rb->state.orientation = updateRotationGivenAngularVelocity(
           //     rb->state.orientation, V3D(/*TODO: change this!*/), dt);
        }

        // clean up
        for (uint i = 0; i < f_ext.size(); i++) {
            f_ext[i] = V3D();
            tau_ext[i] = V3D();
        }
    }
};

}  // namespace crl

#endif  //A5_RBSYMPLECTICENGINE_H
