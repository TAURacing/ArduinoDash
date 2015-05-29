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
  LED(Colour* ledCol);
  ~LED();
  void setHighThreshold(int16_t thresh);
  void setLowThreshold(int16_t thresh);
  void setHighColour(Colour* col);
  void setLowColour(Colour* col);
  void setStandardColour(Colour* col);

  private:
  int* ledCol;
  int16_t lowThreshold;
  int16_t highThreshold;
  Colour* lowColour;
  Colour* highColour;
  Colour* standardColour;
};

#endif // LED_H