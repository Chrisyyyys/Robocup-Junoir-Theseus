// Simulator version of the LiquidCrystal (HD44780) library. Text written to the
// LCD shows up in the simulator log.
#pragma once
#include <Arduino.h>

#define LCD_5x8DOTS 0x00
#define LCD_5x10DOTS 0x04

class LiquidCrystal : public Print {
 public:
  LiquidCrystal(uint8_t rs, uint8_t enable, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
  LiquidCrystal(uint8_t rs, uint8_t rw, uint8_t enable, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
  void begin(uint8_t cols, uint8_t rows, uint8_t charsize = LCD_5x8DOTS);
  void clear();
  void home();
  void setCursor(uint8_t col, uint8_t row);
  void noDisplay() {}
  void display() {}
  void noBlink() {}
  void blink() {}
  void noCursor() {}
  void cursor() {}
  size_t write(uint8_t c) override;
  using Print::write;

 private:
  uint8_t cols_ = 16, rows_ = 2, col_ = 0, row_ = 0;
  char text_[4][41];
};
