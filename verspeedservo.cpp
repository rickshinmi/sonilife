#include <Arduino.h>
#include "VarSpeedServo.h"

#define usToTicks(_us)    ((clockCyclesPerMicrosecond() * _us) / 8)
#define ticksToUs(_ticks) ((unsigned)(_ticks * 8) / clockCyclesPerMicrosecond())

#define TRIM_DURATION       2

#define MAX_SERVOS          12  // Adjust this value based on your requirements
#define SERVOS_PER_TIMER    12  // Adjust this value based on your requirements
#define REFRESH_INTERVAL    20000  // 20 ms


servo_t servos[MAX_SERVOS];
static volatile int8_t Channel[_Nbr_16timers];

uint8_t ServoCount = 0;

servoSequencePoint initSeq[] = {{0, 100}, {45, 100}};

#define SERVO_INDEX_TO_TIMER(_servo_nbr) ((timer16_Sequence_t)(_servo_nbr / SERVOS_PER_TIMER))
#define SERVO_INDEX_TO_CHANNEL(_servo_nbr) (_servo_nbr % SERVOS_PER_TIMER)
#define SERVO_INDEX(_timer, _channel) ((_timer * SERVOS_PER_TIMER) + _channel)
#define SERVO(_timer, _channel) (servos[SERVO_INDEX(_timer, _channel)])
#define SERVO_MIN() (MIN_PULSE_WIDTH - this->min * 4)  // minimum value in uS for this servo
#define SERVO_MAX() (MAX_PULSE_WIDTH - this->max * 4)  // maximum value in uS for this servo
// Function prototypes
static void handle_interrupts(timer16_Sequence_t timer, volatile uint16_t *TCNTn, volatile uint16_t *OCRnA);
static void initISR(timer16_Sequence_t timer);
static void finISR(timer16_Sequence_t timer);
static boolean isTimerActive(timer16_Sequence_t timer);

// Other functions remain unchanged

// Update ISR functions for ESP32
#if defined(ESP32)

hw_timer_t *timer1 = NULL;
hw_timer_t *timer3 = NULL;
hw_timer_t *timer4 = NULL;
hw_timer_t *timer5 = NULL;

static void IRAM_ATTR onTimer1() {
  handle_interrupts(_timer1, &timer1->count, &timer1->alarm_high);
}

static void IRAM_ATTR onTimer3() {
  handle_interrupts(_timer3, &timer3->count, &timer3->alarm_high);
}

static void IRAM_ATTR onTimer4() {
  handle_interrupts(_timer4, &timer4->count, &timer4->alarm_high);
}

static void IRAM_ATTR onTimer5() {
  handle_interrupts(_timer5, &timer5->count, &timer5->alarm_high);
}

static void initISR(timer16_Sequence_t timer) {
  switch (timer) {
    case _timer1:
      if (!timer1) {
        timer1 = timerBegin(1, 80, true);
        timerAttachInterrupt(timer1, &onTimer1, true);
        timerAlarmWrite(timer1, REFRESH_INTERVAL, true);
        timerAlarmEnable(timer1);
      }
      break;

    case _timer3:
      if (!timer3) {
        timer3 = timerBegin(3, 80, true);
        timerAttachInterrupt(timer3, &onTimer3, true);
        timerAlarmWrite(timer3, REFRESH_INTERVAL, true);
        timerAlarmEnable(timer3);
      }
      break;

    case _timer4:
      if (!timer4) {
        timer4 = timerBegin(4, 80, true);
        timerAttachInterrupt(timer4, &onTimer4, true);
        timerAlarmWrite(timer4, REFRESH_INTERVAL, true);
        timerAlarmEnable(timer4);
      }
      break;

    case _timer5:
      if (!timer5) {
        timer5 = timerBegin(5, 80, true);
        timerAttachInterrupt(timer5, &onTimer5, true);
        timerAlarmWrite(timer5, REFRESH_INTERVAL, true);
        timerAlarmEnable(timer5);
      }
      break;

    default:
      break;
  }
}

static void finISR(timer16_Sequence_t timer) {
  switch (timer) {
    case _timer1:
      if (timer1) {
        timerAlarmDisable(timer1);
        timer1 = NULL;
      }
      break;

    case _timer3:
      if (timer3) {
        timerAlarmDisable(timer3);
        timer3 = NULL;
      }
      break;

    case _timer4:
      if (timer4) {
        timerAlarmDisable(timer4);
        timer4 = NULL;
      }
      break;

    case _timer5:
      if (timer5) {
        timerAlarmDisable(timer5);
        timer5 = NULL;
      }
      break;

    default:
      break;
  }
}

#endif  // ESP32


VarSpeedServo::VarSpeedServo()
{
  if( ServoCount < MAX_SERVOS) {
    this->servoIndex = ServoCount++;                    // assign a servo index to this instance
	  servos[this->servoIndex].ticks = usToTicks(DEFAULT_PULSE_WIDTH);   // store default values  - 12 Aug 2009
    this->curSeqPosition = 0;
    this->curSequence = initSeq;
  }
  else
    this->servoIndex = INVALID_SERVO ;  // too many servos
}

uint8_t VarSpeedServo::attach(int pin)
{
  return this->attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

uint8_t VarSpeedServo::attach(int pin, int min, int max)
{
  if(this->servoIndex < MAX_SERVOS ) {
    pinMode( pin, OUTPUT) ;                                   // set servo pin to output
    servos[this->servoIndex].Pin.nbr = pin;
    // todo min/max check: abs(min - MIN_PULSE_WIDTH) /4 < 128
    this->min  = (MIN_PULSE_WIDTH - min)/4; //resolution of min/max is 4 uS
    this->max  = (MAX_PULSE_WIDTH - max)/4;
    // initialize the timer if it has not already been initialized
    timer16_Sequence_t timer = SERVO_INDEX_TO_TIMER(servoIndex);
    if(isTimerActive(timer) == false)
      initISR(timer);
    servos[this->servoIndex].Pin.isActive = true;  // this must be set after the check for isTimerActive
  }
  return this->servoIndex ;
}

void VarSpeedServo::detach()
{
  servos[this->servoIndex].Pin.isActive = false;
  timer16_Sequence_t timer = SERVO_INDEX_TO_TIMER(servoIndex);
  if(isTimerActive(timer) == false) {
    finISR(timer);
  }
}

void VarSpeedServo::write(int value)
{

  byte channel = this->servoIndex;
  servos[channel].value = value;

  if(value < MIN_PULSE_WIDTH)
  {  // treat values less than 544 as angles in degrees (valid values in microseconds are handled as microseconds)
    // updated to use constrain() instead of if(), pva
    value = constrain(value, 0, 180);
    value = map(value, 0, 180, SERVO_MIN(),  SERVO_MAX());
  }
  this->writeMicroseconds(value);
}

void VarSpeedServo::writeMicroseconds(int value)
{
  // calculate and store the values for the given channel
  byte channel = this->servoIndex;
  servos[channel].value = value;

  if( (channel >= 0) && (channel < MAX_SERVOS) )   // ensure channel is valid
  {
    if( value < SERVO_MIN() )          // ensure pulse width is valid
      value = SERVO_MIN();
    else if( value > SERVO_MAX() )
      value = SERVO_MAX();

  	value -= TRIM_DURATION;
    value = usToTicks(value);  // convert to ticks after compensating for interrupt overhead - 12 Aug 2009

    uint8_t oldSREG = SREG;
    cli();
    servos[channel].ticks = value;
    SREG = oldSREG;

	// Extension for slowmove
	// Disable slowmove logic.
	servos[channel].speed = 0;
	// End of Extension for slowmove
  }
}

// Extension for slowmove
/*
  write(value, speed) - Just like write but at reduced speed.

  value - Target position for the servo. Identical use as value of the function write.
  speed - Speed at which to move the servo.
          speed=0 - Full speed, identical to write
          speed=1 - Minimum speed
          speed=255 - Maximum speed
*/
void VarSpeedServo::write(int value, uint8_t speed) {
	// This fuction is a copy of write and writeMicroseconds but value will be saved
	// in target instead of in ticks in the servo structure and speed will be save
	// there too.

  byte channel = this->servoIndex;
  servos[channel].value = value;

	if (speed) {

		if (value < MIN_PULSE_WIDTH) {
			// treat values less than 544 as angles in degrees (valid values in microseconds are handled as microseconds)
			// updated to use constrain instead of if, pva
			value = constrain(value, 0, 180);
			value = map(value, 0, 180, SERVO_MIN(),  SERVO_MAX());
		}

		// calculate and store the values for the given channel
		if( (channel >= 0) && (channel < MAX_SERVOS) ) {   // ensure channel is valid
			// updated to use constrain instead of if, pva
			value = constrain(value, SERVO_MIN(), SERVO_MAX());

			value = value - TRIM_DURATION;
			value = usToTicks(value);  // convert to ticks after compensating for interrupt overhead - 12 Aug 2009

			// Set speed and direction
			uint8_t oldSREG = SREG;
			cli();
			servos[channel].target = value;
			servos[channel].speed = speed;
			SREG = oldSREG;
		}
	}
	else {
		write (value);
	}
}

void VarSpeedServo::write(int value, uint8_t speed, bool wait) {
  write(value, speed);

  if (wait) { // block until the servo is at its new position
    if (value < MIN_PULSE_WIDTH) {
      while (read() != value) {
        delay(5);
      }
    } else {
      while (readMicroseconds() != value) {
        delay(5);
      }
    }
  }
}

void VarSpeedServo::stop() {
  write(read());
}

void VarSpeedServo::slowmove(int value, uint8_t speed) {
  // legacy function to support original version of VarSpeedServo
  write(value, speed);
}

// End of Extension for slowmove


int VarSpeedServo::read() // return the value as degrees
{
  return  map( this->readMicroseconds()+1, SERVO_MIN(), SERVO_MAX(), 0, 180);
}

int VarSpeedServo::readMicroseconds()
{
  unsigned int pulsewidth;
  if( this->servoIndex != INVALID_SERVO )
    pulsewidth = ticksToUs(servos[this->servoIndex].ticks)  + TRIM_DURATION ;   // 12 aug 2009
  else
    pulsewidth  = 0;

  return pulsewidth;
}

bool VarSpeedServo::attached()
{
  return servos[this->servoIndex].Pin.isActive ;
}

uint8_t VarSpeedServo::sequencePlay(servoSequencePoint sequenceIn[], uint8_t numPositions, bool loop, uint8_t startPos) {
  uint8_t oldSeqPosition = this->curSeqPosition;

  if( this->curSequence != sequenceIn) {
    //Serial.println("newSeq");
    this->curSequence = sequenceIn;
    this->curSeqPosition = startPos;
    oldSeqPosition = 255;
  }

  if (read() == sequenceIn[this->curSeqPosition].position && this->curSeqPosition != CURRENT_SEQUENCE_STOP) {
    this->curSeqPosition++;

    if (this->curSeqPosition >= numPositions) { // at the end of the loop
      if (loop) { // reset to the beginning of the loop
        this->curSeqPosition = 0;
      } else { // stop the loop
        this->curSeqPosition = CURRENT_SEQUENCE_STOP;
      }
    }
  }

  if (this->curSeqPosition != oldSeqPosition && this->curSeqPosition != CURRENT_SEQUENCE_STOP) {
    // CURRENT_SEQUENCE_STOP position means the animation has ended, and should no longer be played
    // otherwise move to the next position
    write(sequenceIn[this->curSeqPosition].position, sequenceIn[this->curSeqPosition].speed);
    //Serial.println(this->seqCurPosition);
  }

  return this->curSeqPosition;
}

uint8_t VarSpeedServo::sequencePlay(servoSequencePoint sequenceIn[], uint8_t numPositions) {
  return sequencePlay(sequenceIn, numPositions, true, 0);
}

void VarSpeedServo::sequenceStop() {
  write(read());
  this->curSeqPosition = CURRENT_SEQUENCE_STOP;
}

// to be used only with "write(value, speed)"
void VarSpeedServo::wait() {
  byte channel = this->servoIndex;
  int value = servos[channel].value;

  // wait until is done
  if (value < MIN_PULSE_WIDTH) {
    while (read() != value) {
      delay(5);
    }
  } else {
    while (readMicroseconds() != value) {
      delay(5);
    }
  }
}

bool VarSpeedServo::isMoving() {
  byte channel = this->servoIndex;
  int value = servos[channel].value;

  if (value < MIN_PULSE_WIDTH) {
    if (read() != value) {
      return true;
    }
  } else {
    if (readMicroseconds() != value) {
      return true;
    }
  }
  return false;
}