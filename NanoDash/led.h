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

class LED
{
  public:
  LED()
      : LED(nullptr, 0, 0, nullptr, nullptr, nullptr)
  {
  }
  LED(Colour* theLed, int16_t lowThresh, int16_t highThresh, Colour* lowColour,
      Colour* highColour, Colour* standardColour)
      : LED(theLed)
  {
    setLed(theLed);
    setLowThreshold(lowThresh);
    setHighThreshold(highThresh);
    setLowColour(lowColour);
    setHighColour(highColour);
    setStandardColour(standardColour);
  }
  LED(Colour* theLed) { led = theLed; }
  ~LED();
  void setLed(Colour* colour) { led = colour; }
  void setHighThreshold(int16_t thresh) { highThreshold = thresh; }
  void setLowThreshold(int16_t thresh) { lowThreshold = thresh; }
  void setHighColour(Colour* col) { highColour = col; }
  void setLowColour(Colour* col) { lowColour = col; }
  void setStandardColour(Colour* col) { standardColour = col; }
  void checkAndUpdate()
  {
    // TODO
  }

  private:
  Colour* led;
  int16_t lowThreshold;
  int16_t highThreshold;
  Colour* lowColour;
  Colour* highColour;
  Colour* standardColour;
  unsigned long lastUpdate;
  unsigned long deltaTime;
};

#endif // LED_H