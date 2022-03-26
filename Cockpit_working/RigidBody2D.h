

struct RigidBody {

  RigidBody(float m) : fMass{m} {}
  
  float fThrust;    // total thrust
  float fMass;      // total mass (constant)
  double vVelocity;
  float fC;   // drag coefficient
  float S;    // displacement
  

  float fSpeed;   // speed
  float vForces;    // total force on body
  float vMoments;   // total moment (torque) on body

  float thrustForce;    // Magnitude of the thrust force


};
