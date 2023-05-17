#include <Arduino.h>
#include <unity.h>


#include "Ramp.hpp"
#include "defines.hpp"


float currentSpeed = 0;
uint32_t current_time;
Ramp* rampLinear = NULL;

// runs before each test
void setUp(void)
{
}

// runs after each test
void tearDown(void)
{
}

void test_ramp_update()
{
  current_time = micros();

  TEST_ASSERT_NOT_NULL(rampLinear);

  currentSpeed = rampLinear->updateRamp(current_time);

  // TEST_ASSERT_EQUAL(13, LED_BUILTIN);
  // TEST_ASSERT_NOT_EQUAL(12, LED_BUILTIN);
}


void test_smth() {
  TEST_PASS_MESSAGE("Noice");
  TEST_FAIL_MESSAGE("C'est con");
  TEST_IGNORE_MESSAGE("Osef");

  TEST_MESSAGE("Coucou");
}

void setup()
{
  // NOTE!!! Wait for >2 secs
  // if board doesn't support software reset via Serial.DTR/RTS
  delay(2000);

  Ramp* rampLinear = new Ramp(DEFAULT_LINEAR_ACCEL_PARAM);

  uint32_t t0 = micros();
  rampLinear->beginRamp(t0, MAX_LINEAR_GOAL_SPEED);


  UNITY_BEGIN(); // begin unit testing
}

uint8_t i = 0;
uint8_t max_loop_nb = 5;



void loop()
{

  current_time = micros();

  currentSpeed = rampLinear->updateRamp(current_time);
    
  if (i < max_loop_nb)
  {
    RUN_TEST(test_ramp_update);
    delay(100);
    i++;
  }
  else if (i == max_loop_nb)
  {
    UNITY_END(); // stop unit testing
  }
}