#include "custom_dynamics_world.h"

#include <BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h>

#include <vector>
#include <iostream>

// Print 3x3 Matrix to console with message
void printMatrix(btMatrix3x3 matrix, char* message) {
    //std::cout<< message << std::endl;
    //for (int i = 0; i < 3; ++i)
    //{
       // for (int j = 0; j < 3; ++j)
       // {
        //    std::cout<<matrix[i][j]<<" ";
        //}
        //std::cout<<std::endl;
    //}
    //std::cout<<"------------"<<std::endl;
    printf("%s,%f %f %f\n%f %f %f\n%f %f %f\n---------------\n",message,matrix[0][0],matrix[0][1],matrix[0][2],matrix[1][0],matrix[1][1],matrix[1][2],matrix[2][0],matrix[2][2],matrix[2][2]);
}
// Print Vector 3 to console with message
void printVector(btVector3 vector, char* message){
    //std::cout<< message <<std::endl;
    //std::cout<< vector.getX() << " " << vector.getY() << " " << vector.getZ() << " " <<std::endl;
    printf("%s\n%f %f %f\n",message,vector.getX(),vector.getY(),vector.getZ());
}


//----------------------------------------------------------------------------------------//
void CustomDynamicsWorld::internalSingleStepSimulation(btScalar timeStep)
{
    // TODO: Enable this? Don't think it's so important for our purposes here though.
    // BT_PROFILE("internalSingleStepSimulation");

    if (0 != m_internalPreTickCallback)
    {
        (*m_internalPreTickCallback)(this, timeStep);
    }

    btDispatcherInfo& dispatchInfo = getDispatchInfo();

    dispatchInfo.m_timeStep = timeStep;
    dispatchInfo.m_stepCount = 0;
    dispatchInfo.m_debugDraw = getDebugDrawer();

    getSolverInfo().m_timeStep = timeStep;

    integrateConstrainedBodiesWithCustomPhysics(timeStep);

    ///update vehicle simulation
    updateActions(timeStep);

    updateActivationState(timeStep);

    if (0 != m_internalTickCallback)
    {
        (*m_internalTickCallback)(this, timeStep);
    }
}

std::vector<btRigidBody *> collectBodies(btCollisionObjectArray & objects,
                                          std::vector<btRigidBody *> && buffer = std::vector<btRigidBody *>())
{
    const auto num_obj = objects.size();
    for (int i = 0; i < num_obj; ++i)
    {
        const auto & obj = objects[i];
        const auto body = btRigidBody::upcast(obj);

        if (body)
        {
            buffer.push_back(body);
        }
    }

    return buffer;
}

// Get contraint manifolds, which contains information about interpenetrations between two rigid bodies.
std::vector<btPersistentManifold *> fetchManifolds(CustomDynamicsWorld & world) 
{
    const auto num_manifolds = world.getDispatcher()->getNumManifolds();
    std::vector<btPersistentManifold *> manifolds;
    for (int i = 0; i < num_manifolds; ++i) 
    {
        manifolds.push_back(world.getDispatcher()->getManifoldByIndexInternal(i));
        auto manifold = manifolds[i];
        const auto num_contacts = manifold->getNumContacts();
        for (int c = 0; c < num_contacts; c++) 
        {
            auto & contact = manifold->getContactPoint(c);
            contact.m_appliedImpulse = 0.0f;
            contact.m_appliedImpulseLateral1 = 0.0f;
            contact.m_appliedImpulseLateral2 = 0.0f;
        }
    }
    return manifolds;

}

void CustomDynamicsWorld::integrateConstrainedBodiesWithCustomPhysics(btScalar timeStep) {
    // We typically perform collision detection at the beginning of the step
    performDiscreteCollisionDetection();

    // Fetch list of current manifolds in CustomDynamicWorld this
    auto manifolds = fetchManifolds(*this);
    auto num_manifolds = manifolds.size();

    // Collect all instances of btRigidBody (as pointers) in the current simulation. Note: there may also be
    // collision objects which are not rigid bodies.
    auto bodies = collectBodies(getCollisionObjectArray());

    // The below does nothing, it just touches the center of mass transforms of each body
    // (turns out that not doing this somehow causes some strange issues in Godot).
    // You can use this as a starting point for unconstrained time integration.
    for (auto body : bodies)
    {
        btVector3 velocity, position, acceleration;
        auto x = body->getCenterOfMassPosition();
        auto q = body->getOrientation();

        //Get current velocity and position
        velocity = body->getLinearVelocity();
        position = x;
        //Calculate acceleration
        acceleration = body->getTotalForce() * body->getInvMass();
        //Semi-explicit Euler
        velocity += acceleration * timeStep;
        body->setLinearVelocity(velocity);
        // position += velocity * timeStep;
        // // Update Rigidbody rotation and position
        // body->setCenterOfMassTransform(btTransform(q, position));  
    }

    // Get ball joints and add them to point_constraints
    const auto num_constraints = this->getNumConstraints();
    std::vector<btPoint2PointConstraint *> point_constraints;

    for (int i = 0; i < num_constraints; ++i)
    {
        auto c = this->getConstraint(i);
        if (c->getConstraintType() == POINT2POINT_CONSTRAINT_TYPE) {
            point_constraints.push_back(dynamic_cast<btPoint2PointConstraint *>(c));
        }
    }

    // When looping over your constraints, you can use getConstraintIterations() to obtain the number of
    // constraint iterations configured in the Project Settings:printf("Contact\n");
    for (int conIt = 0; conIt < getConstraintIterations(); ++conIt)
    {   
        // Apply correctional impulses for joints
        for (auto c:point_constraints)
        {
            auto &bodyA = c->getRigidBodyA();
            auto &bodyB = c->getRigidBodyB();

            auto pivotA = c->getPivotInA();
            auto pivotB = c->getPivotInB();

            /// Calculate S
            auto identity_matrix = btMatrix3x3::getIdentity();
            auto massA = identity_matrix*bodyA.getInvMass();
            auto massB = identity_matrix*bodyB.getInvMass();
            auto Riri = bodyA.getWorldTransform().getBasis()*pivotA;
            auto Rjrj = bodyB.getWorldTransform().getBasis()*pivotB;
            auto Ji = bodyA.getInvInertiaTensorWorld();
            auto Jj = bodyB.getInvInertiaTensorWorld();
            auto Ki = btMatrix3x3(0.0f, -Riri.getZ(), Riri.getY(), Riri.getZ(), 0.0f, -Riri.getX(), -Riri.getY(), Riri.getX(), 0.0f);
            auto Kj = btMatrix3x3(0.0f, -Rjrj.getZ(), Rjrj.getY(), Rjrj.getZ(), 0.0f, -Rjrj.getX(), -Rjrj.getY(), Rjrj.getX(), 0.0f);

            auto inertia_i = Ki.transpose()*Ji*Ki;
            auto inertia_j = Kj*Jj*Kj.transpose();

            auto s = identity_matrix*massA + identity_matrix*massB + inertia_i + inertia_j;

            // Calculate deltaLamda
            auto vi = bodyA.getLinearVelocity();
            auto vj = bodyB.getLinearVelocity();
            auto wi = bodyA.getAngularVelocity();
            auto wj = bodyB.getAngularVelocity();
            auto gu = vi - Ki*wi - vj + Kj*wj;

            // Fixing drift
            auto pivotA_world = bodyA.getCenterOfMassPosition() + bodyA.getWorldTransform().getBasis()*pivotA;
            auto pivotB_world = bodyB.getCenterOfMassPosition() + bodyB.getWorldTransform().getBasis()*pivotB;
            auto targetVelocity = (pivotB_world - pivotA_world)/timeStep;
            auto gamma = 1.0f;
            auto deltaLamda = (targetVelocity*gamma-gu)*s.inverse();

            // Apply correctional impulses
            auto correctiveLinearVelA = identity_matrix*massA*deltaLamda;
            bodyA.setLinearVelocity(vi + correctiveLinearVelA);

            auto correctiveAngularVelA = Ji*Ki.transpose()*deltaLamda;
            bodyA.setAngularVelocity(wi - correctiveAngularVelA);

            auto correctiveLinearVelB = identity_matrix*massB*deltaLamda;
            bodyB.setLinearVelocity(vj - correctiveLinearVelB);
            
            auto correctiveAngularVelB = Jj*Kj.transpose()*deltaLamda;
            bodyB.setAngularVelocity(wj + correctiveAngularVelB);
        }

        // Apply correctional impulses for contacts
        for (size_t i = 0; i < num_manifolds; ++i)
        {
            auto manifold = manifolds[i];

            auto bodyA = btRigidBody::upcast(const_cast<btCollisionObject *>(manifold->getBody0()));
            auto bodyB = btRigidBody::upcast(const_cast<btCollisionObject *>(manifold->getBody1()));

            const auto is_rigidA = bodyA != nullptr;
            const auto is_rigidB = bodyB != nullptr;
            if (!is_rigidA || !is_rigidB) 
            {
                continue;
            }
            const auto num_contacts = manifold->getNumContacts();
            for (int c = 0; c < num_contacts; ++c) 
            {
                auto & contact = manifold->getContactPoint(c);
                const auto n = contact.m_normalWorldOnB;
                const auto r_a = contact.m_localPointA;
                const auto r_b = contact.m_localPointB;

                // Calculate S
                auto identity_matrix = btMatrix3x3::getIdentity();
                auto massA = identity_matrix*bodyA->getInvMass();
                auto massB = identity_matrix*bodyB->getInvMass();
                auto Riri = bodyA->getWorldTransform().getBasis()*r_a;
                auto Rjrj = bodyB->getWorldTransform().getBasis()*r_b;
                auto Ji = bodyA->getInvInertiaTensorWorld();
                auto Jj = bodyB->getInvInertiaTensorWorld();
                auto Ki = btMatrix3x3(0.0f, -Riri.getZ(), Riri.getY(), Riri.getZ(), 0.0f, -Riri.getX(), -Riri.getY(), Riri.getX(), 0.0f);
                auto Kj = btMatrix3x3(0.0f, -Rjrj.getZ(), Rjrj.getY(), Rjrj.getZ(), 0.0f, -Rjrj.getX(), -Rjrj.getY(), Rjrj.getX(), 0.0f);
                auto inertia_i = n.dot(Ki*Ji*Ki.transpose()*n);
                auto inertia_j = n.dot(Kj*Jj*Kj.transpose()*n);

                auto s = bodyA->getInvMass() + bodyB->getInvMass() + inertia_i + inertia_j;

                // Calculate deltaLamda
                auto vi = bodyA->getLinearVelocity();
                auto vj = bodyB->getLinearVelocity();
                auto wi = bodyA->getAngularVelocity();
                auto wj = bodyB->getAngularVelocity();
                auto gu = n.dot(vi - Ki*wi - vj + Kj*wj);

                // Fixing drift
                auto ra_world = bodyA->getCenterOfMassPosition() + bodyA->getWorldTransform().getBasis()*r_a;
                auto rb_world = bodyB->getCenterOfMassPosition() + bodyB->getWorldTransform().getBasis()*r_b;
                auto cn = n.dot(ra_world - rb_world);
                auto targetVelocity = cn/timeStep;
                auto gamma = 0.1f;
                btScalar deltaLamda;

                //Restitution
                //auto dc = n.dot(bodyA->getLinearVelocity()-bodyB->getLinearVelocity());
                //auto epsilon = contact.m_combinedRestitution;
                //auto c_target = btMax(targetVelocity*-gamma,-epsilon*dc);
                auto c_target = targetVelocity*-gamma;
                if (cn < 0)
                {
                    deltaLamda = (c_target-gu)/s;
                }
                else
                {
                    deltaLamda = -gu/s;
                }

                // Stack fix
                auto & total_dn = contact.m_appliedImpulse;
                if (total_dn + deltaLamda < 0)
                {
                    deltaLamda = -total_dn; 
                }
                total_dn += deltaLamda;
                // Apply correctional impulses
                auto correctiveLinearVelA = identity_matrix*n*massA*deltaLamda;
                auto correctiveAngularVelA = Ji*Ki.transpose()*n*deltaLamda;
                auto correctiveLinearVelB = identity_matrix*n*massB*deltaLamda;
                auto correctiveAngularVelB = Jj*Kj.transpose()*n*deltaLamda;
                if(true){
                bodyA->setLinearVelocity(vi + correctiveLinearVelA);
                bodyA->setAngularVelocity(wi - correctiveAngularVelA);
                bodyB->setLinearVelocity(vj - correctiveLinearVelB);
                bodyB->setAngularVelocity(wj + correctiveAngularVelB);
                }

                auto v1 = bodyA->getLinearVelocity();
                auto v2 = bodyB->getLinearVelocity();
                auto w1 = bodyA->getAngularVelocity();
                auto w2 = bodyB->getAngularVelocity();

                Ji = bodyA->getInvInertiaTensorWorld();
                Jj = bodyB->getInvInertiaTensorWorld();
                auto v_r = v1-(Ki*w1)-v2+(Kj*w2);
                //Compute tangential velocity by Gram-Schmidt-Projection
                auto v_T = v_r-((n.dot(v_r))/n.length2())*n;
                contact.m_lateralFrictionDir1 = v_T.safeNormalize();
                auto v_T2 = n.cross(contact.m_lateralFrictionDir1);
                contact.m_lateralFrictionDir2 = v_T2.safeNormalize();

                auto & alpha_1 = contact.m_appliedImpulseLateral1;
                auto & alpha_2 = contact.m_appliedImpulseLateral2;
                auto & t_1 = contact.m_lateralFrictionDir1;
                auto & t_2 = contact.m_lateralFrictionDir2;

                //Use Matrix first row as t_1T, therefore select first entry at the end
                //auto t_1T = btMatrix3x3(t_1.getX(),t_1.getY(),t_1.getZ(),0,0,0,0,0,0);
                auto inertia1 = t_1.dot(Ki*Ji*Ki.transpose()*t_1);
                auto inertia2 = t_1.dot(Kj*Jj*Kj.transpose()*t_1);
                auto scalar_s = bodyA->getInvMass()+bodyB->getInvMass()+inertia1+inertia2;
                
                //Apply G_1*u
                auto gb = v1-(Ki*w1)-v2+(Kj*w2);
                auto scalar_g = t_1.dot(gb);
                //printf("%f,%f\n", scalar_g2,t_2.dot(gb));
                //Solve da
                auto da_1 = -scalar_g/scalar_s;
                auto mu = contact.m_combinedFriction;
                auto & dn = contact.m_appliedImpulse;
                auto boundary = mu*dn;
                if(alpha_1+da_1 < -boundary)
                {
                    da_1 = (-boundary) - alpha_1;
                }
                else if(alpha_1+da_1 > boundary)
                {
                    da_1 = boundary - alpha_1;
                }

                alpha_1 += da_1;
                //Apply impulses
                correctiveLinearVelA = massA*t_1*da_1;
                bodyA->setLinearVelocity(v1 + correctiveLinearVelA);
                
                correctiveAngularVelA = Ji*Ki.transpose()*t_1*da_1;
                bodyA->setAngularVelocity(w1 - correctiveAngularVelA);

                correctiveLinearVelB = massB*t_1*da_1;
                bodyB->setLinearVelocity(v2 - correctiveLinearVelB);

                correctiveAngularVelB = Jj*Kj.transpose()*t_1*da_1;
                bodyB->setAngularVelocity(w2 + correctiveAngularVelB);

                //Analagous to t_1T
                //auto t_2T = btMatrix3x3(t_2.getX(),t_2.getY(),t_2.getZ(),0,0,0,0,0,0);
                inertia1 = t_2.dot(Ki*Ji*Ki.transpose()*t_2);
                inertia2 = t_2.dot(Kj*Jj*Kj.transpose()*t_2);
                auto scalar_s2 = bodyA->getInvMass()+bodyB->getInvMass()+inertia1+inertia2;

                //Apply G_1*u
                v1 = bodyA->getLinearVelocity();
                v2 = bodyB->getLinearVelocity();
                w1 = bodyA->getAngularVelocity();
                w2 = bodyB->getAngularVelocity();
                gb = v1-(Ki*w1)-v2+(Kj*w2);
                auto scalar_g2 = t_2.dot(gb);
                auto da_2 = -scalar_g2/scalar_s2;
                if(alpha_2+da_2 < -boundary)
                {
                    da_2 = (-boundary) - alpha_2;
                }
                else if(alpha_2+da_2 > boundary)
                {
                    da_2 = boundary - alpha_2;
                }
                alpha_2 += da_2;
                correctiveLinearVelA = massA*t_2*da_2;
                bodyA->setLinearVelocity(v1 + correctiveLinearVelA);
                
                correctiveAngularVelA =Ji*Ki.transpose()*t_2*da_2;
                bodyA->setAngularVelocity(w1 - correctiveAngularVelA);

                correctiveLinearVelB =massB*t_2*da_2;
                bodyB->setLinearVelocity(v2 - correctiveLinearVelB);

                correctiveAngularVelB =Jj*Kj.transpose()*t_2*da_2;
                bodyB->setAngularVelocity(w2 + correctiveAngularVelB);
            }
        }
    }

    for (auto body : bodies)
    {   
        auto x = body->getCenterOfMassPosition();
        auto q = body->getOrientation();

        auto omega = body->getAngularVelocity();
        auto omega_quat = btQuaternion(omega.x(), omega.y(), omega.z(), 0.0f);
        q += omega_quat*q*timeStep/2;
        q.safeNormalize();
        auto velocity = body->getLinearVelocity();
        auto position = x + velocity*timeStep;
        body->setCenterOfMassTransform(btTransform(q, position));
    }   

    // Testing
    // Important types that you may wish to use:
    //  btVector3 (3-element vector)
    //  btMatrix3 (3x3 matrix)
    //  btTransform (encapsulates rotation transform and translation)

    // Matrix/vector types support "obvious" functionality, such as:
    //  v.cross(u)
    //  v.dot(u)
    //  a * v (matrix-vector product)
    //  a * b (matrix-matrix product)
    //  a * alpha (scaling matrix by scalar)

    // Below we have included some examples of things you may want to do
    // Given a rigid body `body`:
    //  const auto x = body->getCenterOfMassPosition();
    //  const auto q = body->getOrientation();
    //  const auto v = body->getLinearVelocity();
    //  const auto omega = body->getAngularVelocity();
    //  const auto m_inv = body->getInvMass();
    //  const auto I_inv = body->getInvInertiaTensorWorld();
    //  const auto f = body->getTotalForce();
    //  const auto tau = body->getTotalTorque();

    // Create quaternion from angular velocity (with 0 "w" component)
    //  const auto omega_quat = btQuaternion(omega.x(), omega.y(), omega.z(), 0);

    // Normalize a quaternion:
    //  q.safeNormalize();

    // Update transform (generalized position) of rigid body with new values q and x
    // (note: implicitly also updates "world" inertia tensor)
    //  body->setCenterOfMassTransform(btTransform(q, x));

    // Get rotation matrix for a body
    // body->getWorldTransform().getBasis()
}
