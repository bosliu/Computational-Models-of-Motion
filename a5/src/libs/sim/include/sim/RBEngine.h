#pragma once

#include <sim/RB.h>
#include <sim/RBRenderer.h>
#include <sim/RBSpring.h>
#include <sim/RBUtils.h>

namespace crl {

// subfunctions
Quaternion updateRotationGivenAngularVelocity(const Quaternion &q,
                                              const V3D &angularVelocity,
                                              double dt) {
    double angularVelocityMagnitude = angularVelocity.norm();
    // avoid divide by close to zero...
    if (angularVelocityMagnitude > 1e-10) {
        Quaternion qnew;

        qnew.w() = cos(dt * angularVelocityMagnitude/ 2);
        qnew.vec() = sin(dt * angularVelocityMagnitude /2) * angularVelocity / angularVelocityMagnitude;
        Quaternion q_p = qnew * q;


        // TODO: Ex.1 Integration
        // implement quaternion update logic
        // q_p = rot(w, dt) * q

        return q_p;
    }
    return q;
}

/**
 * "base" class of our simulation world governed by the rigid-body dynamics
 */
class RBEngine {
public:
    // constructor
    RBEngine() {}

    // desctructor
    virtual ~RBEngine() {
        // we are going to delete every rigid body when simulation world is
        // destroyed.
        for (uint i = 0; i < rbs.size(); i++) delete rbs[i];
        rbs.clear();
        for (uint i = 0; i < springs.size(); i++) delete springs[i];
        springs.clear();
    }

    /**
     * add rigid body to simulation world.
     * note that we assume this is a cube block with approx. 0.24 x 0.24 x 0.24. 
     */
    RB *addRigidBodyToEngine() {
        rbs.push_back(new RB());
        // we use default mass = 100 kg
        rbs.back()->rbProps.mass = 100;
        rbs.back()->rbProps.collision = false;
        rbs.back()->rbProps.id = rbs.size() - 1;
        // add force and torque
        f_ext.push_back(V3D());
        tau_ext.push_back(V3D());
        return rbs.back();
    }

    /**
     * add rigid body with collision to simulation world.
     * note that we assume this is a sphere with radius = 0.1. 
     */
    RB *addCollidingRigidBodyToEngine() {
        double i = 0.4 * 100 * 0.1 * 0.1;
        rbs.push_back(new RB());
        // we use default mass = 100 kg
        rbs.back()->rbProps.mass = 100;
        rbs.back()->rbProps.setMOI(i, i, i, 0, 0, 0);
        rbs.back()->rbProps.collision = true;
        rbs.back()->rbProps.id = rbs.size() - 1;
        // add force and torque
        f_ext.push_back(V3D());
        tau_ext.push_back(V3D());
        return rbs.back();
    }

    /**
     * add spring to simulation world.
     */
    RBSpring *addSpringToEngine(RB *parent, RB *child, P3D pJPos, P3D cJPos) {
        springs.push_back(new RBSpring());
        springs.back()->parent = parent;
        springs.back()->child = child;
        // local position of attached point from parent/child frames
        springs.back()->pJPos = pJPos;
        springs.back()->cJPos = cJPos;
        // we use default spring constant = 2000;
        springs.back()->k = 10000;

        // default rest length is the distance between attaching points when
        // the spring is added.
        if (parent == nullptr) {
            // TODO: Ex.2-1
            // implement your logic for a spring which is attached to world
            //


            // Hint:
            // - you can get world coordinates of local coordinate P3D p by
            // rb->state.getWorldCoordinates(p).

            //springs.back()->l0 = 0;
            P3D pJPosWorld = pJPos;
            P3D cJPosWorld = child ->state.getWorldCoordinates(cJPos);
            springs.back()->l0 = V3D(pJPosWorld - cJPosWorld).norm();
             // TODO: change this!
        } else {
            // TODO: Ex.2-2
            // implement your logic for a spring where both ends are attached
            // to rigid bodies
            //
            //
            // Hint:
            // - you can get world coordinates of local coordinate P3D p by
            // rb->state.getWorldCoordinates(p).

            //springs.back()->l0 = 0;  // TODO: change this!
            P3D pJPosWorld = parent->state.getWorldCoordinates(pJPos);
            P3D cJPosWorld = child->state.getWorldCoordinates(cJPos);
            springs.back()->l0 = V3D(pJPosWorld - cJPosWorld).norm();
        }
        return springs.back();
    }

    /**
     * apply external force (no spring force, no gravity. Force comes from 
     * third sources) to rigid body.
     */
    void applyForceTo(RB *rb, const V3D &f, const P3D &p) {
        // add force only if rb is in rbs
        for (uint i = 0; i < rbs.size(); i++) {
            if (rbs[i] == rb) {
                f_ext[i] += f;
                V3D r = rb->state.getWorldCoordinates(V3D(p));
                tau_ext[i] += r.cross(f);
            }
        }
    }

    /**
     * simulation stepping logic. advance one simulation timestep with dt.
     * the unit of dt is second.
     *
     * note that this function is "pure virtual" function. Actual implementation
     * of step function should be completed in the "derived" class.
     */
    virtual void step(double dt) = 0;

    /**
     * draw every rigid body belongs to world.
     */
    inline void draw(const gui::Shader &rbShader) {
        // draw moi boxes
        for (uint i = 0; i < this->rbs.size(); i++) {
            if (!this->rbs[i]->rbProps.fixed) {
                if (this->rbs[i]->rbProps.collision)
                    crl::RBRenderer::drawCollisionRB(this->rbs[i], rbShader);
                else
                    crl::RBRenderer::drawMOI(this->rbs[i], rbShader);
            }
        }

        // draw springs
        for (uint i = 0; i < this->springs.size(); i++) {
            P3D start, end;
            if (this->springs[i]->parent == nullptr) {
                start = this->springs[i]->pJPos;
                end = this->springs[i]->child->state.getWorldCoordinates(
                    this->springs[i]->cJPos);
            } else {
                start = this->springs[i]->parent->state.getWorldCoordinates(
                    this->springs[i]->pJPos);
                end = this->springs[i]->child->state.getWorldCoordinates(
                    this->springs[i]->cJPos);
            }
            drawCylinder(start, end, 0.05, rbShader);
        }

        // and now coordinate frames
        if (showCoordFrame) {
            for (uint i = 0; i < this->rbs.size(); i++)
                crl::RBRenderer::drawCoordFrame(this->rbs[i], rbShader);
        }
    }

    /**
     * returns NULL if no RBs are hit by the ray...
     */
    RB *getFirstRBHitByRay(const Ray &ray, P3D &intersectionPoint) {
        RB *selectedRB = nullptr;
        double t = DBL_MAX;
        P3D tmpIntersectionPoint = P3D(0, 0, 0);

        for (uint i = 0; i < rbs.size(); i++) {
            if (rbs[i]->getRayIntersectionPoint(ray, tmpIntersectionPoint)) {
                double tTmp = ray.getRayParameterFor(tmpIntersectionPoint);
                if (tTmp < t) {
                    selectedRB = rbs[i];
                    t = tTmp;
                    intersectionPoint = tmpIntersectionPoint;
                }
            }
        }
        return selectedRB;
    }

protected:
    void updateForceForGravity() {
        for (uint i = 0; i < rbs.size(); i++) {
            // force and torque by gravity
            f_ext[i] += rbs[i]->rbProps.mass * V3D(0, RBGlobals::g, 0);
        }
    }

    void updateForceAndTorqueForSprings() {
        // force and torque by springs
        for (RBSpring *spring : springs) {
            // TODO: Ex.2 Spring force
            // compute spring force f_spring = -kx and torque tau_spring and
            // add them to f_ext and tau_ext
            //
            // Hint:
            // - spring->l0 is the rest length and spring->k is the spring contant
            // - you can retrieve index of rb in this->rbs list from rb->rbProps.id

            if (spring->parent == nullptr) {
                // TODO: Ex.2-1
                // implement your logic for a spring which is attached to world

                // force
                V3D pJPosWorld = V3D(spring->pJPos);
                V3D cJPosWorld = V3D(spring->child->state.getWorldCoordinates(spring->cJPos));
                V3D vPos = cJPosWorld - pJPosWorld;
                V3D fSpring = -spring->k *(vPos - vPos.normalized() * spring->l0);
                V3D tauSpring = (cJPosWorld - V3D(spring->child->state.pos)).cross(fSpring);
                auto it = find(rbs.begin(), rbs.end(), spring->child);
                int idx = it - rbs.begin();
                //V3D f(0, 0, 0);  // TODO: change this!
                f_ext[idx] += fSpring;
                tau_ext[idx] += tauSpring;

                // torque
                //V3D tau(0, 0, 0);  // TODO: change this!
                //tau_ext[spring->child->rbProps.id] += tau;
            } else {
                // TODO: Ex.2-2
                // implement your logic for a spring where both ends are attached
                // to rigid bodies.

                // force
                V3D pJPosWorld = V3D(spring->parent->state.getWorldCoordinates(spring->pJPos));
                V3D cJPosWorld = V3D(spring->child->state.getWorldCoordinates(spring->cJPos));
                V3D vPos = cJPosWorld - pJPosWorld;
                V3D fSpring = -spring->k * (vPos - vPos.normalized() * spring->l0);
                V3D tauSpringP = (pJPosWorld - V3D(spring->parent->state.pos)).cross(fSpring);
                V3D tauSpringC = (cJPosWorld - V3D(spring->child->state.pos)).cross(fSpring);
                auto pIt = find(rbs.begin(), rbs.end(), spring->parent);
                int pIdx = pIt - rbs.begin();
                f_ext[pIdx] -= fSpring;
                tau_ext[pIdx] -= tauSpringP;
                auto cIt = find(rbs.begin(), rbs.end(), spring->child);
                int cIdx = cIt - rbs.begin();
                f_ext[cIdx] += fSpring;
                tau_ext[cIdx]+= tauSpringC;
                //V3D f(0, 0, 0);  // TODO: change this!
                //f_ext[spring->parent->rbProps.id] -= f;
                //f_ext[spring->child->rbProps.id] += f;

                // torque
                //V3D tau1(0, 0, 0);  // TODO: change this!
                //V3D tau2(0, 0, 0);  // TODO: change this!
                //tau_ext[spring->parent->rbProps.id] += tau1;
                //tau_ext[spring->child->rbProps.id] += tau2;
            }
        }
    }

    void updateVelocityAfterCollision(RB *rb) const {
        // TODO: Ex.4 Impulse-based Collisions
        // we will simulate collisions between a spherical rigidbody and
        // the ground plane. implement impulse-based collisions here. use
        // coefficient of restituation "epsilon". (it's a member variable
        // of this class).
        // we only implement collisions between ground and spheres.
        //
        // Steps:
        // 0. read the material "ImpulseBasedCollisions" on CMM21 website
        // carefully.
        // 1. compute impulse
        // 2. update linear and angular velocity with an impulse
        //
        // Hint:
        // - the radius of the sphere is 0.1 m
        // - detect collision if 1) the y coordinate of the point at the
        // bottom of the sphere < 0 and 2) the y component of linear
        // velocity of the point at the botton < 0.
        // - we will assume that a collision only happens at the bottom
        // points.
        // - we will assume there's only one contact between a sphere
        // and the ground

        
       P3D bottomPosRel = P3D(0.0, -0.1, 0.0);
       P3D bottomPos = rb->state.getWorldCoordinates(bottomPosRel);
       V3D bottomVel = rb->state.getVelocityForPoint_global(bottomPos);// TODO: change this!
        //bool collisionDetected = (bottomPos.y<0) && (bottomVel.y()<0);  
        if (bottomPos.y < 0 && bottomVel.y() < 0) {
            V3D impulse(0, 0, 0);
            Matrix3x3 K_ground = Matrix3x3::Zero();
            Matrix3x3 R = rb->state.orientation.toRotationMatrix();
            Matrix3x3 inertiaWorld = R * rb->rbProps.MOI_local * R.transpose();
            P3D rB = bottomPos - rb->state.pos;
            Matrix3x3 rBSkew = getSkewSymmetricMatrix(V3D(rB));
            Matrix3x3 K_rb = (Matrix3x3::Identity() / rb->rbProps.mass - rBSkew * inertiaWorld.inverse() * rBSkew);
            Matrix3x3 K_T = K_ground + K_rb;
            V3D N = V3D(0, 1, 0);
            V3D u_rel = bottomVel;
            if (frictionalCollision) {
            
        
            impulse = K_T.inverse() * (-u_rel - eps * u_rel.dot(N) * N.normalized());
             // TODO: compute infinite friction collision impulse
            } else {
            impulse = (-(1+eps)*u_rel.dot(N))/(N.transpose()*K_T*N)*N;
                // TODO: compute frictionless collision impulse
            }

            // update velocity by impulse
            rb->state.velocity += impulse / rb->rbProps.mass;
            rb->state.angularVelocity += inertiaWorld.inverse() * rBSkew * impulse;
              // TODO: change this!
        }
        
    }

public:
    // this is a list of all rigid bodies and springs belong to the world.
    std::vector<RB *> rbs;
    std::vector<RBSpring *> springs;

    // coefficients
    float eps = 0.0;  // restitution

    // drawing flags
    bool showCoordFrame = true;

    // options
    bool simulateCollisions = false;
    bool frictionalCollision = false;

protected:
    // list of force and tau applied to rigid bodies
    std::vector<V3D> f_ext;
    std::vector<V3D> tau_ext;
};
}  // namespace crl