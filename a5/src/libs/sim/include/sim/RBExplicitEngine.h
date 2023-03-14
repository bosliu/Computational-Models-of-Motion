//
// Created by Dongho Kang on 25.04.22.
//

#ifndef A5_RBEXPLICITENGINE_H
#define A5_RBEXPLICITENGINE_H

#include "sim/RBEngine.h"

namespace crl {

/**
 * simulation world implemented with explicit integration scheme.
 */
class RBExplicitEngine : public RBEngine {
public:
    RBExplicitEngine() : RBEngine() {}

    ~RBExplicitEngine() override = default;

    void step(double dt) override {
        // update external force and torque
        updateForceForGravity();
        updateForceAndTorqueForSprings();

        // update states of rbs (integration)
        for (uint i = 0; i < rbs.size(); i++) {
            RB *rb = rbs[i];
            // retrieve saved force and tau
            V3D f = f_ext[i];
            V3D tau = tau_ext[i];

            // TODO: Ex.1 Numerical Integration
            // implement forward (explicit) Euler integration scheme for computing velocity.
            //
            // Hint:
            // - complete the function,
            // Quaternion updateRotationGivenAngularVelocity(const Quaternion &q, const V3D &angularVelocity, double dt)
            // in src/libs/sim/include/sim/RBEngine.h and use it for updating orientation of rigidbody.
            // - recall, you need to compute 3x3 moment of inertia matrix expressed in world frame.
            rb->state.pos = rb->state.pos + dt * rb->state.velocity;
            Matrix3x3 R = rb->state.orientation.toRotationMatrix();
            rb->state.orientation = updateRotationGivenAngularVelocity(rb->state.orientation, rb->state.angularVelocity, dt);
            rb->state.velocity += dt * f / rb->rbProps.mass;
            Matrix3x3 inertiaWorld = R * rb->rbProps.MOI_local * R.transpose();
            rb->state.angularVelocity += dt * inertiaWorld.inverse() * (tau - rb->state.angularVelocity.cross(V3D(inertiaWorld * rb->state.angularVelocity)));  // TODO: change this!

            if (simulateCollisions && rb->rbProps.collision)
                updateVelocityAfterCollision(rb);

            // TODO: Ex.1 Numerical Integration
            // implement forward (explicit) Euler integration scheme for computing pose.
            


            
            
            //rb->state.pos = rb->state.pos + V3D() * dt;  // TODO: change this!
            //rb->state.orientation = updateRotationGivenAngularVelocity(
               //rb->state.orientation, V3D(/*TODO: change this!*/), dt);
        }

        // clean up
        for (uint i = 0; i < f_ext.size(); i++) {
            f_ext[i] = V3D();
            tau_ext[i] = V3D();
        }
    }
};

}  // namespace crl

#endif  //A5_RBEXPLICITENGINE_H
