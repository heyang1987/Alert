module BlinkC
{
  uses interface Timer<TMilli> as Timer0;
  uses interface Leds;
  uses interface Boot;
}

implementation
{
  event void Boot.booted()
  {
    call Timer0.startPeriodic( 500 );
  }

  event void Timer0.fired()
  {
    dbg("BlinkC", "Timer 0 fired @ %s.\n", sim_time_string());
#ifndef BLINK_REVERSE
    call Leds.led2Toggle();
    //call Leds.led1Toggle();
    call Leds.led0Toggle();
#else
    call Leds.led2Toggle();
#endif
  }
}
