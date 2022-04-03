#include "vector.h"

struct Drag_Forces
{
  Drag_Forces(String n, float d, bool s)
    : f_name{n}, cD{d}, f_status{s} {}
    
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
  float total_fC();
  //void set_fC(float d) {fC=d;}
  void set_gears_fC(float d) {gears_fC=d;}
  void add_drag_force(String n, float d, bool s);
  
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
  vector<Drag_Forces> drag;
  private:
  float fMass;      // total mass (constant)
  float fThrust;    // total thrust
  //float fC;   // drag coefficient
  float gears_fC;   // drag coefficient
  

};

void RigidBody::add_drag_force(String n, float d, bool s)
{
  //Drag_Forces temp{n,d, s};
  drag.push_back(Drag_Forces{n,d,s});
}

float RigidBody::total_fC() 
{
  float total{0};
  for(int i=0; i<drag.size();++i){
    if(drag[i].f_status==true) {
      total+=drag[i].cD;
    }
  }
  return total;
}
