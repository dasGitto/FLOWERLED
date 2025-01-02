#include <Adafruit_NeoPixel.h>
#include <esp_sleep.h>

#define BUTTON_PIN D0
#define PIN D10
#define NUM_LEDS 64
#define LEDS_ONE_SIDE 32
#define DEEP_SLEEP_TIME 4000 // Time in milliseconds for deep sleep trigger

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);

enum Mode { FIRE, METEOR, STROBE, CYLON, RUN_LIGHTS, BOUNCE_BALLS, METEOR_ORIG, BLUEFIRE };
Mode currentMode = FIRE;

volatile bool buttonPressed = false;
unsigned long buttonPressTime = 0;

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), onButtonPress, FALLING);

  strip.begin();
  strip.show();

  esp_sleep_enable_gpio_wakeup();

  // Handle waking from deep sleep
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO) {
    Serial.begin(115200);
    Serial.println("Woke up from deep sleep due to button press.");
  }
}

void loop() {
  // Handle button press
  if (buttonPressed) {
    buttonPressed = false;

    unsigned long pressDuration = millis() - buttonPressTime;

    if (pressDuration >= DEEP_SLEEP_TIME) {
      Serial.println("Entering deep sleep...");
      enterDeepSleep();
    } else {
      Serial.println("Cycling modes...");
      cycleModes();
    }
  }

  // Execute the current mode's effect
  switch (currentMode) {
    case FIRE: Fire(55, 120, 15); break;
    case BLUEFIRE: blueFire(55, 120, 15); break;
    case METEOR: meteorRain(0xff, 0xff, 0xff, 4, 64, true, 45); break;
    case STROBE: Strobe(0xff, 0xff, 0xff, 10, 25, 0); break;
    case METEOR_ORIG: meteorRainOriginal(0xff, 0xff, 0xff, 10, 64, true, 30); break;
    case CYLON: CylonBounce(0, 0xff, 0, 4, 10, 50); break;
    case RUN_LIGHTS: RunningLights(0xff, 0xff, 0x00, 50); break;
    case BOUNCE_BALLS: BouncingBalls(0x9b, 0x93, 0xf0, 3); break;
  }
}

// Interrupt Service Routine for button press
void onButtonPress() {
  // Record the time of button press
  if (!buttonPressed) {
    buttonPressed = true;
    buttonPressTime = millis();
  }
}

void enterDeepSleep() {
  detachInterrupt(digitalPinToInterrupt(BUTTON_PIN));
  esp_sleep_enable_gpio_wakeup();
  esp_deep_sleep_start();
}

void cycleModes() {
  currentMode = static_cast<Mode>((currentMode + 1) % 8);
  delay(200); // Debounce delay
}


void Fire(int Cooling, int Sparking, int SpeedDelay) {
  static byte heat[NUM_LEDS];
  int cooldown;
  
  // Step 1.  Cool down every cell a little
  for( int i = 0; i < LEDS_ONE_SIDE; i++) {
    cooldown = random(0, ((Cooling * 10) / LEDS_ONE_SIDE) + 2);
    
    if(cooldown>heat[i]) {
      heat[i]=0;
    } else {
      heat[i]=heat[i]-cooldown;
    }
  }
  
  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for( int k= LEDS_ONE_SIDE - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  }
    
  // Step 3.  Randomly ignite new 'sparks' near the bottom
  if( random(255) < Sparking ) {
    int y = random(7);
    heat[y] = heat[y] + random(160,255);
    //heat[y] = random(160,255);
  }

  // Step 4.  Convert heat to LED colors
  for( int j = 0; j < LEDS_ONE_SIDE; j++) {
    setPixelHeatColor(j, heat[j] );
    setPixelHeatColor(NUM_LEDS - j, heat[j] );
  }

  showStrip();
  // Check if button was pressed during the effect
  if (buttonPressed) {
    return; // Exit the function to handle button press
  }
  delay(SpeedDelay);
}

void setPixelHeatColor (int Pixel, byte temperature) {
  // Scale 'heat' down from 0-255 to 0-191
  byte t192 = round((temperature/255.0)*191);
 
  // calculate ramp up from
  byte heatramp = t192 & 0x3F; // 0..63
  heatramp <<= 2; // scale up to 0..252
 
  // figure out which third of the spectrum we're in:
  if( t192 > 0x80) {                     // hottest
    setPixel(Pixel, 255, 255, heatramp);
  } else if( t192 > 0x40 ) {             // middle
    setPixel(Pixel, 255, heatramp, 0);
  } else {                               // coolest
    setPixel(Pixel, heatramp, 0, 0);
  }
}
// *** REPLACE TO HERE ***

void showStrip() {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   strip.show();
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H
   // FastLED
   FastLED.show();
 #endif
}

void setPixel(int Pixel, byte red, byte green, byte blue) {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   strip.setPixelColor(Pixel, strip.Color(red, green, blue));
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H 
   // FastLED
   leds[Pixel].r = red;
   leds[Pixel].g = green;
   leds[Pixel].b = blue;
 #endif
}

void setAll(byte red, byte green, byte blue) {
  for(int i = 0; i < NUM_LEDS; i++ ) {
    setPixel(i, red, green, blue); 
  }
  showStrip();
}


void blueFire(int Cooling, int Sparking, int SpeedDelay) {
  static byte heat[NUM_LEDS];
  int cooldown;
  
  // Step 1. Cool down every cell a little
  for (int i = 0; i < LEDS_ONE_SIDE; i++) {
    cooldown = random(0, ((Cooling * 10) / LEDS_ONE_SIDE) + 2);
    
    if (cooldown > heat[i]) {
      heat[i] = 0;
    } else {
      heat[i] = heat[i] - cooldown;
    }
  }
  
  // Step 2. Heat from each cell drifts 'up' and diffuses a little
  for (int k = LEDS_ONE_SIDE - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  }
    
  // Step 3. Randomly ignite new 'sparks' near the bottom
  if (random(255) < Sparking) {
    int y = random(7);
    heat[y] = heat[y] + random(160, 255);
  }

  // Step 4. Convert heat to LED colors
  for (int j = 0; j < LEDS_ONE_SIDE; j++) {
    bsetPixelHeatColor(j, heat[j]);
    bsetPixelHeatColor(NUM_LEDS - j, heat[j]);
  }

  showStrip();
  // Check if button was pressed during the effect
  if (buttonPressed) {
    return; // Exit the function to handle button press
  }
  delay(SpeedDelay);
}

void bsetPixelHeatColor(int Pixel, byte temperature) {
  // Scale 'heat' down from 0-255 to 0-191
  byte t192 = round((temperature / 255.0) * 191);
 
  // Calculate ramp up from 0 to 63
  byte heatramp = t192 & 0x3F;
  heatramp <<= 2; // Scale up to 0-252
 
  // Figure out which third of the spectrum we're in:
  if (t192 > 0x80) {                     // Hottest (white)
    setPixel(Pixel, 255, 255, 255);
  } else if (t192 > 0x40) {              // Middle (icy blue + white mix)
    setPixel(Pixel, 0x80, 0x80, 255);
  } else {                               // Coolest (icy blue)
    setPixel(Pixel, 0, 0, heatramp + 0x80);
  }
}

//meteor code

void meteorRain(byte green, byte red, byte blue, byte meteorSize, byte meteorTrailDecay, boolean meteorRandomDecay, int SpeedDelay) {  
  setAll(0,0,0);
 
  for(int i = 0; i < NUM_LEDS; i++) {
   
   
    // fade brightness all LEDs one step
    for(int j=0; j<LEDS_ONE_SIDE; j++) {
      if( (!meteorRandomDecay) || (random(30)>5) ) {
        fadeToBlack(j, meteorTrailDecay );        
        fadeToBlack(LEDS_ONE_SIDE+j, meteorTrailDecay );        
      }
    }
   
    // draw meteor
    for(int j = 0; j < meteorSize; j++) {
      if( ( i-j <LEDS_ONE_SIDE) && (i-j>=0) ) {
        setPixel(i-j, red, green, blue);
        setPixel(LEDS_ONE_SIDE+i-j, red, green, blue);
      }
    }
   
    showStrip();
    
    // Check if button was pressed during the effect
    if (buttonPressed) {
      return; // Exit the function to handle button press
    }
    delay(SpeedDelay);
  }
}

void fadeToBlack(int ledNo, byte fadeValue) {
 #ifdef ADAFRUIT_NEOPIXEL_H
    // NeoPixel
    uint32_t oldColor;
    uint8_t r, g, b;
    int value;
   
    oldColor = strip.getPixelColor(ledNo);
    r = (oldColor & 0x00ff0000UL) >> 16;
    g = (oldColor & 0x0000ff00UL) >> 8;
    b = (oldColor & 0x000000ffUL);

    r=(r<=10)? 0 : (int) r-(r*fadeValue/256);
    g=(g<=10)? 0 : (int) g-(g*fadeValue/256);
    b=(b<=10)? 0 : (int) b-(b*fadeValue/256);
   
    strip.setPixelColor(ledNo, r,g,b);
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H
   // FastLED
   leds[ledNo].fadeToBlackBy( fadeValue );
 #endif  
}


void Strobe(byte red, byte green, byte blue, int StrobeCount, int FlashDelay, int EndPause){
  for(int j = 0; j < StrobeCount; j++) {
    setAll(red,green,blue);
    showStrip();
    delay(FlashDelay);
    setAll(0,0,0);
    showStrip();
    delay(FlashDelay);
  }
 
 // Check if button was pressed during the effect
  if (buttonPressed) {
    return; // Exit the function to handle button press
  }
 delay(EndPause);
}


void CylonBounce(byte green, byte red, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay){

  for(int i = 0; i < NUM_LEDS-EyeSize-2; i++) {
    setAll(0,0,0);
    setPixel(i, red/10, green/10, blue/10);
    for(int j = 1; j <= EyeSize; j++) {
      setPixel(i+j, red, green, blue); 
    }
    setPixel(i+EyeSize+1, red/10, green/10, blue/10);
    showStrip();
    // Check if button was pressed during the effect
    if (buttonPressed) {
      return; // Exit the function to handle button press
    }
    delay(SpeedDelay);
  }

  delay(ReturnDelay);

  for(int i = NUM_LEDS-EyeSize-2; i > 0; i--) {
    setAll(0,0,0);
    setPixel(i, red/10, green/10, blue/10);
    for(int j = 1; j <= EyeSize; j++) {
      setPixel(i+j, red, green, blue); 
    }
    setPixel(i+EyeSize+1, red/10, green/10, blue/10);
    showStrip();
    if (buttonPressed) {
      return; // Exit the function to handle button press
    }
    delay(SpeedDelay);
  }
  
  delay(ReturnDelay);
}


void RunningLights(byte green, byte red, byte blue, int WaveDelay) {
  int Position=0;
  
  for(int i=0; i<NUM_LEDS*2; i++)
  {
      Position++; // = 0; //Position + Rate;
      for(int i=0; i<NUM_LEDS; i++) {
        // sine wave, 3 offset waves make a rainbow!
        //float level = sin(i+Position) * 127 + 128;
        //setPixel(i,level,0,0);
        //float level = sin(i+Position) * 127 + 128;
        setPixel(i,((sin(i+Position) * 127 + 128)/255)*red,
                   ((sin(i+Position) * 127 + 128)/255)*green,
                   ((sin(i+Position) * 127 + 128)/255)*blue);
      }
      
      showStrip();
      if (buttonPressed) {
        return; // Exit the function to handle button press
    }
      delay(WaveDelay);
  }
}


void BouncingBalls(byte green, byte red, byte blue, int BallCount) {
  float Gravity = -9.81;
  int StartHeight = 1;
  
  float Height[BallCount];
  float ImpactVelocityStart = sqrt( -2 * Gravity * StartHeight );
  float ImpactVelocity[BallCount];
  float TimeSinceLastBounce[BallCount];
  int   Position[BallCount];
  long  ClockTimeSinceLastBounce[BallCount];
  float Dampening[BallCount];
  
  for (int i = 0 ; i < BallCount ; i++) {   
    ClockTimeSinceLastBounce[i] = millis();
    Height[i] = StartHeight;
    Position[i] = 0; 
    ImpactVelocity[i] = ImpactVelocityStart;
    TimeSinceLastBounce[i] = 0;
    Dampening[i] = 0.90 - float(i)/pow(BallCount,2); 
  }

  while (true) {
    for (int i = 0 ; i < BallCount ; i++) {
      TimeSinceLastBounce[i] =  millis() - ClockTimeSinceLastBounce[i];
      Height[i] = 0.5 * Gravity * pow( TimeSinceLastBounce[i]/1000 , 2.0 ) + ImpactVelocity[i] * TimeSinceLastBounce[i]/1000;
  
      if ( Height[i] < 0 ) {                      
        Height[i] = 0;
        ImpactVelocity[i] = Dampening[i] * ImpactVelocity[i];
        ClockTimeSinceLastBounce[i] = millis();
  
        if ( ImpactVelocity[i] < 0.01 ) {
          ImpactVelocity[i] = ImpactVelocityStart;
        }
      }
      Position[i] = round( Height[i] * (LEDS_ONE_SIDE - 1) / StartHeight);
    }
  
    for (int i = 0 ; i < BallCount ; i++) {
      setPixel(Position[i],red,green,blue);
      setPixel(NUM_LEDS-Position[i],red,green,blue);
    }
    
    showStrip();
    if (buttonPressed) {
        return; // Exit the function to handle button press
    }
    setAll(0,0,0);
  }
}

void meteorRainOriginal(byte red, byte green, byte blue, byte meteorSize, byte meteorTrailDecay, boolean meteorRandomDecay, int SpeedDelay) {  
  setAll(0,0,0);
 
  for(int i = 0; i < NUM_LEDS+NUM_LEDS; i++) {
   
   
    // fade brightness all LEDs one step
    for(int j=0; j<NUM_LEDS; j++) {
      if( (!meteorRandomDecay) || (random(10)>5) ) {
        fadeToBlack(j, meteorTrailDecay );      
      }
    }
   
    // draw meteor
    for(int j = 0; j < meteorSize; j++) {
      if( ( i-j <NUM_LEDS) && (i-j>=0) ) {
        setPixel(i-j, red, green, blue);
      }
    }
   
    showStrip();
    delay(SpeedDelay);
  }
}