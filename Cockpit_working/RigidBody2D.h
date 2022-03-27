

struct RigidBody {

  RigidBody(float m, Engine& e1, Engine& e2, Fuel_tank& t) 
    : fMass{m}, engine1{e1}, engine2{e2}, tank{t} {}
  
  float fThrust;    // total thrust
  
  double vVelocity;
  float fC;   // drag coefficient
  float S;    // displacement
  

  float fSpeed;   // speed
  float vForces;    // total force on body
  float vMoments;   // total moment (torque) on body

  float thrustForce;    // Magnitude of the thrust force

  const float get_fMass() const {return fMass;}

  Engine& engine1;
  Engine& engine2;
  Fuel_tank& tank;

  private:
  float fMass;      // total mass (constant)
  
  


};
