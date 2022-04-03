#include "tools.h"

void Fuel_tank::refuel()
{
    qty += 1000; //lbs
    if (qty > cpcty) qty = cpcty;
}

double Engine::get_throttle()
{
    double angle = floatMap(throttle, 0, 1023, 0, 100);
    return angle;
    //return throttle_value * 100.0 / 1023.0;
}

double Engine::rpm()
{
    double flow = fuel_flow();
    if (flow > 0.0) {               //spools RPM
        if (eng_rpm < 100.0 * flow) {
            eng_rpm += 2.5;             // spools up
        }
        if (eng_rpm > 100.0 * flow) {
            eng_rpm -= 2.0;             // spools down
        }
    }

    if (eng_rpm > 0 && eng_sw_state) eng_ON = true;
    else eng_ON == false;

    if (!eng_sw_state || flow <= 0) {
        eng_rpm = 0.0;
        eng_ON = false;
    }

    return eng_rpm;
}

bool Engine::fuel_pump()
{
    if (f_pump_sw_state && pwr_sw_state) {
        digitalWrite(f_pump_LED, HIGH);
        return true;
    }
    else {
        digitalWrite(f_pump_LED, LOW);
        return false;
    }
}

double Engine::fuel_flow()
{
    if (0 < tank.get_quantity() && eng_sw_state && fuel_pump()) {
        f_flow = get_throttle() * gps / 100.0; // gallons
        if (timer_second()) tank.consume(f_flow); // fuel is depleted from tank
        if (tank.get_quantity() <= 0) tank.set_quantity(0.0);
    }
    else { f_flow = 0.0; }
    return f_flow;
}

//---------------------------------