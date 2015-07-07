#ifndef LED_H
#define LED_H

struct Colour
{
  Colour()
      : Colour(0, 0, 0)
  {
  }
  Colour(uint8_t red, uint8_t green, uint8_t blue)
  {
    r = red;
    g = green;
    b = blue;
  }
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
};

enum LedState
{
  off = 0,
  low,
  standard,
  high
};

class LED
{
  public:
  LED()
      : LED(NULL, NULL, 0, 0, 0, NULL, NULL, NULL, NULL)
  {
  }

  LED(Colour* theLed, int16_t* refVal, int16_t offThresh, int16_t loThresh,
      int16_t hiThresh, Colour* ofColour, Colour* stdColour, Colour* loColour,
      Colour* hiColour)
      : LED(theLed)
  {
    setLed(theLed);
    setReferenceValue(refVal);
    setOffThreshold(offThresh);
    setLowThreshold(loThresh);
    setHighThreshold(hiThresh);
    setOffColour(ofColour);
    setStandardColour(stdColour);
    setLowColour(loColour);
    setHighColour(hiColour);
  }

  LED(Colour* theLed)
  {
    led = theLed;
    currentState = off;
    lastUpdate = millis();
    deltaTime = 1000;
  }
  ~LED() {}

  void setLed(Colour* colour) { led = colour; }

  void setReferenceValue(int16_t* refVal) { referenceValue = refVal; }

  void setOffThreshold(int16_t thresh) { offThreshold = thresh; }

  void setHighThreshold(int16_t thresh) { highThreshold = thresh; }

  void setLowThreshold(int16_t thresh) { lowThreshold = thresh; }

  void setOffColour(Colour* col) { offColour = col; }

  void setHighColour(Colour* col) { highColour = col; }

  void setLowColour(Colour* col) { lowColour = col; }

  void setStandardColour(Colour* col) { standardColour = col; }

  void setDeltaTime(unsigned long deltaT) { deltaTime = deltaT; }

  void checkAndUpdate()
  {
    LedState newState = getNewState();
    if (newState != currentState)
    {
      if ((millis() - lastUpdate) > deltaTime)
      {
        currentState = newState;
        lastUpdate = millis();
        switch (currentState)
        {
        case off:
          led->r = offColour->r;
          led->g = offColour->g;
          led->b = offColour->b;
          break;
        case low:
          led->r = lowColour->r;
          led->g = lowColour->g;
          led->b = lowColour->b;
          break;
        case standard:
          led->r = standardColour->r;
          led->g = standardColour->g;
          led->b = standardColour->b;
          break;
        case high:
          led->r = highColour->r;
          led->g = highColour->g;
          led->b = highColour->b;
          break;
        default:
          break;
        }
      }
    }
  }

  private:
  Colour* led;
  int16_t* referenceValue;
  int16_t offThreshold;
  int16_t lowThreshold;
  int16_t highThreshold;
  Colour* offColour;
  Colour* lowColour;
  Colour* highColour;
  Colour* standardColour;
  LedState currentState;
  unsigned long lastUpdate;
  unsigned long deltaTime;

  LedState getNewState()
  {
    LedState state = standard;
    if (*referenceValue <= offThreshold)
    {
      state = off;
    }
    else if (*referenceValue <= lowThreshold)
    {
      state = low;
    }
    else if (*referenceValue >= highThreshold)
    {
      state = high;
    }
    else
    {
      // Return state is already standard if none of the above are a match
    }
    return state;
  }
};

#endif // LED_H
