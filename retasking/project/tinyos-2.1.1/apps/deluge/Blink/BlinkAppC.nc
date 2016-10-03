configuration BlinkAppC { }

implementation
{
  components MainC, BlinkC, LedsC;
  components new TimerMilliC() as Timer0;
  components DelugeC;

  DelugeC.Leds -> LedsC;

  BlinkC -> MainC.Boot;
  BlinkC.Timer0 -> Timer0;
  BlinkC.Leds -> LedsC;
}
