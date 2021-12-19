// Modified Version of
// Fire2012 by Mark Kriegsman, July 2012
// as part of "Five Elements" shown here: http://youtu.be/knWiGsmgycY

// COOLING: How much does the air cool as it rises?
// Less cooling = taller flames.  More cooling = shorter flames.
// Default 50, suggested range 20-100 
#define COOLING  1

// SPARKING: What chance (out of 255) is there that a new spark will be lit?
// Higher chance = more roaring fire.  Lower chance = more flickery fire.
// Default 120, suggested range 50-200.
#define SPARKING 60
void Fire()
{
// Array of temperature readings at each simulation cell
  static byte heat[numLEDsBottom];
  static byte heatDrift[numLEDsBottom];

  // Step 1.  Cool down every cell a little
    for( int i = 0; i < numLEDsBottom; i++) {
      heat[i] = qsub8( heat[i],  random8(0, (COOLING / numLEDsBottom) + 2));
    }
    
    // Alternative Step2: Heat drifts left and right
    for (int k = 0; k < numLEDsBottom-1; k++)
    {
      heatDrift[k] = (heat[(k+numLEDsBottom-1)%numLEDsBottom] + heat[(k+numLEDsBottom-2)%numLEDsBottom] + heat[(k+1)%numLEDsBottom] + heat[(k+2)%numLEDsBottom]) / 4;
    }
    for (int k = 0; k < numLEDsBottom-1; k++)
    {
      heat[k] = heatDrift[k];
    }

    // Alternative Step3: Ignite all over the strip
    if( random8() < SPARKING ) {
      uint8_t y = random8(numLEDsBottom);
      uint8_t intensity = random8(ledBrightnessBottom/2,ledBrightnessBottom);
      heat[(y+numLEDsBottom-1)%numLEDsBottom] = qadd8(heat[(y+numLEDsBottom-1)%numLEDsBottom], intensity/3);
      heat[(y+1)%numLEDsBottom] = qadd8(heat[(y+1)%numLEDsBottom], intensity/3);
      heat[y] = qadd8( heat[y], intensity );
    }

    // Step 4.  Map from heat cells to LED colors
    for( int j = 0; j < numLEDsBottom; j++) {
      CRGB color = HeatColor( heat[j]);
      ledsBottom[j] = color;
    }
}

void Pulse()
{
  static float phi = 0.0;

  uint8_t brightness = (sin(phi)+1.0)/2.0*((float)ledBrightnessBottom);
  
  fill_solid(ledsBottom, 
                 numLEDsBottom/*led count*/, 
                 CRGB(0, 0, brightness));
  phi += 0.015;
  if (phi > 360.0)
    phi = 0.0;
}
