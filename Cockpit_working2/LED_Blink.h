#pragma once

class LED_timer {
  public:
    LED_timer() : prev_time{0} {}

    bool blink_LED( int on, int off, long total);
    void end();

    
    int on_time;
    int off_time;
    long total_interval;
    
    long prev_on_time;
    long prev_off_time;
    long prev_time;
    
    bool blink_on{false};
    bool on_interval;
    bool off_interval;
    bool switch_ON;
    bool sequence_done=false;
};
