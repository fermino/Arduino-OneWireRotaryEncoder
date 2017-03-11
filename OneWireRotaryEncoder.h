/**
 * Rotary encoder library for Arduino.
 */

#ifndef OneWireRotaryEncoder_h
#define OneWireRotaryEncoder_h

	#include <Arduino.h>

	// Enable this to emit codes twice per step.
	//#define HALF_STEP

	// Values returned by 'process'
	// No complete step yet.
	#define DIR_NONE 0x0
	// Clockwise step.
	#define DIR_CW 0x10
	// Anti-clockwise step.
	#define DIR_CCW 0x20

	#define R_START 0x0

	#ifdef HALF_STEP
		// Use the half-step state table (emits a code at 00 and 11)
		#define R_CCW_BEGIN 0x1
		#define R_CW_BEGIN 0x2
		#define R_START_M 0x3
		#define R_CW_BEGIN_M 0x4
		#define R_CCW_BEGIN_M 0x5
	#else
		// Use the full-step state table (emits a code at 00 only)
		#define R_CW_FINAL 0x1
		#define R_CW_BEGIN 0x2
		#define R_CW_NEXT 0x3
		#define R_CCW_BEGIN 0x4
		#define R_CCW_FINAL 0x5
		#define R_CCW_NEXT 0x6
	#endif

	template <uint8_t INPUT_PIN>
	class OneWireRotaryEncoder
	{
		private:
			struct
			{
				uint16_t A = 0;
				uint16_t B = 0;
				uint16_t AB = 0;

				uint16_t Button = 0;
				uint16_t ButtonA = 0;
				uint16_t ButtonB = 0;
				uint16_t ButtonAB = 0;
			} ExpectedValues;

			uint8_t ReadTolerance;

			/**
			 * The below state table has, for each state (row), the new state
			 * to set based on the next encoder output. From left to right in,
			 * the table, the encoder outputs are 00, 01, 10, 11, and the value
			 * in that position is the new state to set.
			 */

			#ifdef HALF_STEP
				// Use the half-step state table (emits a code at 00 and 11)
				const unsigned char StateTable[6][4] = {
					// R_START (00)
					{R_START_M,						R_CW_BEGIN,		 R_CCW_BEGIN,	R_START},
					// R_CCW_BEGIN
					{R_START_M | DIR_CCW, R_START,				R_CCW_BEGIN,	R_START},
					// R_CW_BEGIN
					{R_START_M | DIR_CW,	R_CW_BEGIN,		 R_START,			R_START},
					// R_START_M (11)
					{R_START_M,						R_CCW_BEGIN_M,	R_CW_BEGIN_M, R_START},
					// R_CW_BEGIN_M
					{R_START_M,						R_START_M,			R_CW_BEGIN_M, R_START | DIR_CW},
					// R_CCW_BEGIN_M
					{R_START_M,						R_CCW_BEGIN_M,	R_START_M,		R_START | DIR_CCW},
				};
			#else
			// Use the full-step state table (emits a code at 00 only)
				const unsigned char StateTable[7][4] = {
					// R_START
					{R_START,		R_CW_BEGIN,	R_CCW_BEGIN, R_START},
					// R_CW_FINAL
					{R_CW_NEXT,	R_START,		 R_CW_FINAL,	R_START | DIR_CW},
					// R_CW_BEGIN
					{R_CW_NEXT,	R_CW_BEGIN,	R_START,		 R_START},
					// R_CW_NEXT
					{R_CW_NEXT,	R_CW_BEGIN,	R_CW_FINAL,	R_START},
					// R_CCW_BEGIN
					{R_CCW_NEXT, R_START,		 R_CCW_BEGIN, R_START},
					// R_CCW_FINAL
					{R_CCW_NEXT, R_CCW_FINAL, R_START,		 R_START | DIR_CCW},
					// R_CCW_NEXT
					{R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
				};
			#endif

			uint8_t State = R_START;
			bool ButtonPressed = false;

			inline bool valueIsBetween(uint16_t Value, uint16_t Min, uint16_t Max)
			{ return (Value >= Min && Value <= Max); }
		public:
			OneWireRotaryEncoder(uint32_t R2, uint32_t REncoderA, uint32_t REncoderB, uint32_t REncoderButton = 0, uint8_t ReadTolerance = 25);

			uint8_t process();
			inline bool buttonPressed()
			{ return ButtonPressed; }
	};

	template <uint8_t INPUT_PIN>
	OneWireRotaryEncoder<INPUT_PIN>::OneWireRotaryEncoder(uint32_t R2, uint32_t REncoderA, uint32_t REncoderB, uint32_t REncoderButton = 0, uint8_t ReadTolerance = 25)
	{
		ExpectedValues.A = ((float) R2 / ((float) R2 + (float) REncoderA)) * 1023 - ReadTolerance;
		ExpectedValues.B = ((float) R2 / ((float) R2 + (float) REncoderB)) * 1023 - ReadTolerance;
		ExpectedValues.AB = ((float) R2 / ((float) R2 + 1.f / (1.f / (float) REncoderA + 1.f / (float) REncoderB))) * 1023 - ReadTolerance;

		if(REncoderButton != 0)
		{
			ExpectedValues.Button = ((float) R2 / ((float) R2 + (float) REncoderButton)) * 1023 - ReadTolerance;

			ExpectedValues.ButtonA = ((float) R2 / ((float) R2 + 1.f / (1.f / (float) REncoderButton + 1.f / (float) REncoderA))) * 1023 - ReadTolerance;
			ExpectedValues.ButtonB = ((float) R2 / ((float) R2 + 1.f / (1.f / (float) REncoderButton + 1.f / (float) REncoderB))) * 1023 - ReadTolerance;
			ExpectedValues.ButtonAB = ((float) R2 / ((float) R2 + 1.f / (1.f / (float) REncoderButton + 1.f / (float) REncoderA + 1.f / (float) REncoderB))) * 1023 - ReadTolerance;
		}

		/*Serial.println(ExpectedValues.A);
		Serial.println(ExpectedValues.AB);
		Serial.println(ExpectedValues.B);*/

		this->ReadTolerance = ReadTolerance * 2;
	}

	template <uint8_t INPUT_PIN>
	uint8_t OneWireRotaryEncoder<INPUT_PIN>::process()
	{
		uint16_t Reading = analogRead(INPUT_PIN);

		bool ButtonAState = valueIsBetween(Reading, ExpectedValues.ButtonA, ExpectedValues.ButtonA + ReadTolerance);
		bool ButtonBState = valueIsBetween(Reading, ExpectedValues.ButtonB, ExpectedValues.ButtonB + ReadTolerance);
		bool ButtonABState = valueIsBetween(Reading, ExpectedValues.ButtonAB, ExpectedValues.ButtonAB + ReadTolerance);

		bool PinABState = valueIsBetween(Reading, ExpectedValues.AB, ExpectedValues.AB + ReadTolerance);
		bool PinAState = PinABState || ButtonABState || ButtonAState || valueIsBetween(Reading, ExpectedValues.A, ExpectedValues.A + ReadTolerance);
		bool PinBState = PinABState || ButtonABState || ButtonBState || valueIsBetween(Reading, ExpectedValues.B, ExpectedValues.B + ReadTolerance);

		ButtonPressed = ButtonAState || ButtonBState || ButtonABState || valueIsBetween(Reading, ExpectedValues.Button, ExpectedValues.Button + ReadTolerance);

		/*Serial.print(Reading);
		Serial.print("\t");
		Serial.print(PinAState);
		Serial.print("\t");
		Serial.print(PinBState);
		Serial.print("\t");*/

		// Determine new state from the pins and state table.
		State = StateTable[State & 0xf][((PinAState << 1) | PinBState)];

		/*Serial.print(State & 0x30);
		Serial.print("\t");
		Serial.println();*/

		// Return emit bits, ie the generated event.
		return State & 0x30;
	}
#endif