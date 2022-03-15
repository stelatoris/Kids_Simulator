#pragma once

//---------------------------------

double floatMap(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

struct Fuel_tank {

    Fuel_tank(double cap, double quant) : cpcty{ cap }, qty{ quant } {}
    Fuel_tank() : cpcty{ 10000 }, qty{ 5000 } {}

    double get_quantity() const { return qty; }
    void set_quantity(double q) { qty = q; }
    void consume(double q) { qty -= q; }
    void set_capacity(double c) { cpcty = c; }
    double get_capacity() const { return cpcty; }

    void refuel();
private:
    double cpcty;
    double qty;
};

//---------------------------------

struct Engine {
    Engine(Fuel_tank& f) : tank{ f } { thrtl_flip = false; }

    double rpm();
    double fuel_flow();
    double get_throttle();
    bool fuel_pump();
    bool engineON() { return eng_ON; }
    void set_throttle(double t) { throttle = t; }
    void flip_throttle(bool b) { thrtl_flip = b; } // flips rotation on potentiometer
    bool get_throttle_axis() const { return thrtl_flip; }

    //int eng_num;

private:
    Fuel_tank& tank;
    double throttle;
    double eng_rpm;
    double gps = 1.0; // gives gallons per second at 100% throttle
    double f_flow;
    bool eng_ON;
    bool thrtl_flip;
};