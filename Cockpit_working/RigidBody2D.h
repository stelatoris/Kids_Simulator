#include "vector.h"

struct Drag_Forces
{
  Drag_Forces(String n, float d)
    : f_name{n}, cD{d}, f_status{false} {}
    Drag_Forces() : f_name{String()}, cD{0.0}, f_status{false} {}
  
  String f_name;
  float cD;
  bool f_status;  
};

//-----------------------------------------------------

struct RigidBody {

  RigidBody(float m, Engine& e1, Engine& e2, Fuel_tank& t) 
    : fMass{m}, engine1{e1}, engine2{e2}, tank{t} {}
  
  float total_Thrust();
  float total_fC() {return fC+gears_fC;}
  void set_fC(float d) {fC=d;}
  void set_gears_fC(float d) {gears_fC=d;}
  void add_drag_force(String n, float d);
  
  double vVelocity;
  
  float S;    // displacement
  

  float fSpeed;   // speed
  float vForces;    // total force on body
  //float vMoments;   // total moment (torque) on body

  //float thrustForce;    // Magnitude of the thrust force

  const float get_fMass() const {return fMass+tank.get_quantity();}

  Engine& engine1;
  Engine& engine2;
  Fuel_tank& tank;

  private:
  float fMass;      // total mass (constant)
  float fThrust;    // total thrust
  float fC;   // drag coefficient
  float gears_fC;   // drag coefficient
  vector<Drag_Forces> drag;

};

void RigidBody::add_drag_force(String n, float d)
{
  Drag_Forces temp{n,d};
  drag.push_back(temp);
}
