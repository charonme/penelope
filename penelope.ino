#define DEBUG_MODE 0
/*
 * penelope.ino		- step sequencer functionality
 * nlpcore.ino		- Novation Launchpad button detection and LED lighting functionality
 * initconstants.h	- pre-calculated constants put in PROGMEM to save RAM and CPU cycles
 * 
 */



/* TODO:
 * - !!!! implement midi running status !!!!
 * - send note off when a seq is deleted
 * - recalculate seqStepValueToOff when changing base note - see setSeqBaseNoteAndChannel()
 * - note off when seq is paused/stopped/muted
 * - use seqPlayingNote for offing notes easily
 * 
 * - allow seq config to specify how many bits from the step data byte are to be used and interpreted in what way
 *		- for example a few bits for note height and a few for velocity value
 * 
 * - press and hold user 1: select active seq for editing using right arrows (including not assigned slots yet)
 *		- release user 1: enter into editing mode for selected active seq
 *			- after exiting edit mode for an unassigned slot wait for user to pick start/end points
 * - generalized state: when an unassigned slot is selected as "the active seq" (perhaps the seq is already created, but is in defining/not playing state)
 *  pressing and releasing a single pad toggles a step, but pressing and releasing a pad while another one
 *  is pressed will define the start/end points and run/start the seq.
 * - pressing the right arrow slot pad for an active sequence currently stops that sequence.
 *		If there is no seq defined for that slot, create it the same way as described in the process above
 *		and proceed directly to start/end points definition
 * 
 * - right arrows in seq run view:
 *		- select, create, mute, unmute, pause, stop, resume/start
 *			- different functions accessed maybe with a combination of up, down, left, right buttons?
 * 
 * - control view of all sequences 8 pages on 8 rows, max 8 seq per page = 8 cols
 *		- pads have the same functionality as right arrows in seq run view
 * 
 * - current page indicator
 * - visual indication when attempting to create too many sequences
 * 
 * - possibility to send configured MIDI Control Change values instead of note-ons
 * 
 * - more ways (pad combinations) to switch to the scale picker
 *		- pressing both user1+user2 in any order should activate seq scale configurator
 *		- unlatching user1 in seq scale configurator should activate seq scale runner mode
 * - scale runner: bottom row (horizontal scrollbar) could be used for runner indicator too
 * 
 * - manual BPM setting
 *		- pad tapping
 *		- increment/decrement or numeric input
 * - start and stop for all seqs depending on MIDI sync msgs
 * - MIDI input:
 *		- scale setup
 *		- seq base note select
 *		- live note recording (quantized to steps)
 * 
 * - optimize MIDI data sending, I suspect using serial write to send a single byte is too slow
 * 
 * - features documentation
 */

/*
 * 
 * Step toggle screen:
 * 
 *		[up] - select previous (older) sequence
 *		[down] - select next (newer) sequence
 *		[left] - previous page
 *		[right] - next page
 *		[session] -> edit start and end point of active sequence (session+mixer = delete sequence)
 *		[user 1] -> pick note screen (for active sequence)
 *		[user 2] -> melody screen (for active sequence)
 *		[mixer] -> BPM setting screen
 *		[arrows] - create a new sequence or stop/start an existing sequence
 *		[pads] - toggle step
 * 
 * 
 * Pick note screen:
 * 
 *		[up] - scroll to higher octave
 *		[down] - scroll to lower octave
 *		[left] - 
 *		[right] - 
 *		[session] -
 *		[user 1] -> back to step toggle screen
 *		[user 2] -> scale editor screen (for active sequence)
 *		[mixer] -
 *		[arrows] - TODO: fast scrolling between octaves (-2..8)
 *		[pads] - select base note / select midi channel / select timing measure
 * 
 * 
 * 
 *  
 */



#include <Arduino.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "initconstants.h"

extern HardwareSerial Serial;

#define LED_ON 7 //3bits B111
#define LED_DIM 3 //3bits B011
//LED values: RRRGGG00
#define LED_RED 224 //B11100000
#define LED_GREEN 28 //B00011100
#define LED_AMBER 252 //B11111100
#define LED_ORANGE 236 //B11101100
#define LED_YELLOW 108 //B01101100

#define ARROW_SEQ_PLAYING	LED_GREEN
#define ARROW_SEQ_PAUSED	LED_RED
#define ARROW_SEQ_ARMED		LED_AMBER
#define ARROW_SEQ_ACTIVE_PLAYING	LED_ORANGE

#define PAGE_COUNT 8
#define STEP_COUNT 64
#define SEQ_COUNT 32
#define PLAY_BUFFER_SIZE 64

#define PAD_SESSION 35
#define PAD_MIXER 8
#define PAD_UP 71
#define PAD_DOWN 62
#define PAD_LEFT 53
#define PAD_RIGHT 44
#define PAD_USER_1 26
#define PAD_USER_2 17

#define LED_MODE_SEQ_RUN 0
#define LED_MODE_NOTE_PICKER 1
#define LED_MODE_SCALE_PICKER 5 //see PAD_MODE_SCALE_PICKER
#define LED_MODE_SCALE_RUN 6 //see PAD_MODE_SCALE_RUN (one sequence displayed, each row as a different note, each column as a different step)

#define PAD_MODE_STEP_TOGGLE 0 //default mode
#define PAD_MODE_ADD_SEQ 1 //adding seq with SESSION pad (pick start and end)
#define PAD_MODE_NOTE_PICKER 2 //set base note for seq with USER1 pad
#define PAD_MODE_INIT_SEQ 3 //add new seq with right arrow
#define PAD_MODE_EDIT_SEQ 4 //edit seq start and end points
#define PAD_MODE_SCALE_PICKER 5 //selecting multiple notes (in one octave) inside regular note picker
#define PAD_MODE_SCALE_RUN 6 //toggling notes for one sequence configured with scale picker
#define PAD_MODE_SELECT_SEQ 7 //while user1 is pressed down select a slot using right arrows
#define PAD_MODE_SETTING_STEPS 8 //first pad pressed while defining start/end for a seq SEQ_STATE_DEFINED_WITHOUT_STEPS

#define MIDI_WAITING_FOR_SYSEX -1

#define MIDI_BUFFERED_COMMAND_MASK 240	//B1111 0000
#define MIDI_BUFFERED_CHANNEL_MASK 31	//B0000 1111
#define MIDI_BUFFERED_BEAT_CLOCK 240	//B1111 0000
#define MIDI_BUFFERED_BEAT_START 224	//B1110 0000
#define MIDI_BUFFERED_BEAT_STOP 192		//B1100 0000
#define MIDI_BUFFERED_NOTE_ON 144		//B1001 0000
#define MIDI_BUFFERED_NOTE_OFF 128		//B1000 0000
#define MIDI_NOTE_ON 144	//B10010000
#define MIDI_BEAT_CLOCK 248	//B11111000
#define MIDI_BEAT_START 250	//B11111010
#define MIDI_BEAT_STOP 252	//B11111100

#define MIDI_BEAT_STOPPED 0
#define MIDI_BEAT_ARMED 1
#define MIDI_BEAT_PLAYING 2
uint8_t midiBeatState = MIDI_BEAT_STOPPED;

#define METRONOME_LENGTH 48
#define METRONOME_HALF_LENGTH 24

int8_t activeSeqIndex = -1;
int8_t applyNotePickerToSeqIndex = -1;
uint8_t metronomePosition = 0;
unsigned long metronomeLastStepTime;
//unsigned long lastMetronomeBeatTime;

uint8_t playBufferStart = 0;
uint8_t playBufferEnd = 0;
uint8_t playBufferData[PLAY_BUFFER_SIZE];
uint8_t playBufferCommand[PLAY_BUFFER_SIZE];

int8_t activePage = 0;
#define SLOT_SEQS_NO_SEQ -1
int8_t slotSeqs[PAGE_COUNT][8]; //TODO: don't allow more than SEQ_COUNT seqs
int8_t queuedProcessSeqIndex = SEQ_COUNT; //0..SEQ_COUNT-1 = process this seq and increment, setting to 0 will start processing seqs

#define MAX_SCALE_SIZE 24
#define DEFAULT_SCALE_LO 65535 //lower 16 bits
uint16_t seqScaleLo[SEQ_COUNT]; //16 lower bits for scale config
#define DEFAULT_SCALE_HI 255 //higher 8 bits
uint8_t seqScaleHi[SEQ_COUNT]; //8 more bits for two-octave scales

#define SEQ_PAGE_MASK 248 //B11111000
#define SEQ_PAGE_SHIFT 3
#define SEQ_SLOT_MASK 7   //B00000111
uint8_t seqPageSlot[SEQ_COUNT]; //5 MSBits for page number, 3 LSBits for slot number
//uint8_t seqPage[SEQ_COUNT]; //0..PAGE_COUNT-1
//int8_t seqSlot[SEQ_COUNT]; //-1=none, 0..7 (row indicated by right arrows) should be unique for every page

uint8_t seqStart[SEQ_COUNT];	//0..63  (6 bits)
uint8_t seqEnd[SEQ_COUNT];		//0..63  (6 bits)
uint8_t seqPosition[SEQ_COUNT]; //0..63  (6 bits)
uint8_t seqBaseNote[SEQ_COUNT]; //0..127 (7 bits)
uint8_t seqChannel[SEQ_COUNT];	//0..15  (4 bits) //29 total = 4 bytes and 3 bits still left

#define SEQ_STATE_OFF 0 //seq not defined
#define SEQ_STATE_SET 1 //false = seq not defined
#define SEQ_STATE_ARMED 2 //true = waiting for metronome beat clock 0 to start playing
#define SEQ_STATE_PLAYING 4 //false = paused/stopped (not playing)
#define SEQ_STATE_HIGHLIGHTED 128
#define SEQ_STATE_DEFINED_WITHOUT_STEPS 8
uint8_t seqState[SEQ_COUNT];

//playing note waiting to be NOTE-OFF-ed
#define NO_NOTE_TO_OFF 0
int8_t seqStepValueToOff[SEQ_COUNT];

#define NO_PLAYING_NOTE -1
int8_t seqPlayingNote[SEQ_COUNT];

int8_t seqMeasure[SEQ_COUNT]; //number of beat clocks per step
int8_t seqBeatClocksSinceLastStep[SEQ_COUNT]; //always lower than seqMeasure

uint16_t cbpm = 16000; //16000 cbpm = 160bpm; 65535 cbpm = 655.35 bpm
#define CBPM_STEP_MEASURE_RATIO 750000000
unsigned long microsPerMetronomeStep;
unsigned long currentTime;
//const unsigned long MAX_TIME = 4294967295; //+1 overflows to 0
/*
#define MAX_TAP_QUEUE 3
#define LATEST_FIRST_TAP 6000000
#define MAX_TAP_DISCREPANCY_RATIO 3
unsigned long tapTime[MAX_TAP_QUEUE];
uint8_t tapCount = 0;
*/
#define EMPTY_STEP 0 //just the note value, other settings remain set
#define BASE_STEP 1
#define STEPS_NOTE_MASK 31  //B00011111
#define STEPS_ACC 32        //B00100000
#define STEPS_OFF_MASK 192  //B11000000
#define STEPS_OFF_TIE 192   //B11000000
#define STEPS_OFF_NONE 0    //B00000000
#define STEPS_OFF_NEXT 64   //B01000000
#define STEPS_OFF_ONE 128   //B10000000

#define STEPS_OFF_NEXT_VALUE 34
#define STEPS_OFF_ONE_VALUE 33
#define STEPS_ACC_VALUE 32

int8_t steps[PAGE_COUNT][STEP_COUNT];
#define DEFAULT_STEP STEPS_ACC | STEPS_OFF_ONE

//int8_t notePickerBottomOctave = -1; //-2..5 (max octave = 8, 4 are displayed)
uint8_t notePickerCurrentNote = 40; //0..127
//uint8_t notePickerCurrentChannel = 0;
int8_t notePickerCurrentMeasure = 6;
#define NOTE_PICKER_STATE_MASK_CHANNEL 15 //B00001111
#define NOTE_PICKER_STATE_MASK_BOTTOM 112 //B01110000 //bottom octave: max octave = 8, 4 are displayed, 0=-2, 8=5
#define NOTE_PICKER_STATE_SHIFT_BOTTOM 4
#define NOTE_PICKER_STATE_MASK_PREVIEW 128 //B10000000
#define NOTE_PICKER_STATE_SHIFT_PREVIEW 7
uint8_t notePickerState = 1 << NOTE_PICKER_STATE_SHIFT_BOTTOM;
#define NOTE_PICKER_PREVIEW_PAD 9

#define MAX_SCALE_PLAYER_POSITION 120
uint8_t scalePlayerPosition = 0; //leftmost time position (displaying 8 steps in columns from position to pos.+7)
#define MAX_SCALE_PLAYER_OFFSET 28
uint8_t scalePlayerOffset = 0; //bottom note/scale offset value (displaying 8 values in rows from offset to off.+7)

#define SCALE_STEP_FLAG 192 //B11000000
#define SCALE_STEP_ABOVE 128 //B10000000
#define SCALE_STEP_BELOW 64 //B01000000
#define SCALE_STEP_VALUE 63 //B00111111
#define SCALE_STEP_EMPTY 192 //B11000000

const boolean sharpTone[13] = {1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0};
const int8_t tonePickerOffset[13] = {0, -8, 1, -7, 2, 3, -5, 4, -4, 5, -3, 6, 7};
//const int8_t tonePickerOffset[13] = {0, -7, 1, -6, 2, 3, -4, 4, -3, 5, -2, 6, 7};

uint8_t ledMode = LED_MODE_SEQ_RUN;
uint8_t padMode = PAD_MODE_STEP_TOGGLE;

boolean sendMidi = !DEBUG_MODE;
boolean midiBeatMaster = true; //false = slave, receive beat clock, don't send our own
int8_t midiWaitingForIncoming = 0; //MIDI_WAITING_FOR_SYSEX = -1
int8_t midiPassThruFilterOut = 0;

#define OTHER_STEP_NONE -2 //no other pad (nor step) was pressed
#define OTHER_STEP_WRONG -1 //wrong other pad (not a step) or more than 2 pads were pressed


// **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **
//   **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **
// **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **

void loop() {
	currentTime = micros();
	
	if (midiBeatMaster) processMidiMasterMetronome(); //processes sequences too
	if (!midiBeatMaster && isItTime(metronomeLastStepTime + 200000)) { //no incoming clock for 0.2 sec, turn off midi beat slave
		midiBeatMaster = true;
		setLedRed(PAD_MIXER, 0);
		setLedGreen(PAD_MIXER, LED_ON);
	}
	nlpCoreScanBank(); //***

	if (queuedProcessSeqIndex < SEQ_COUNT) queuedProcessSeq();
	nlpCoreScanBank(); //***

	midiProcessBufferedOutput();
	nlpCoreScanBank(); //***

	midiProcessInput();
	nlpCoreScanBank(); //***
}

// **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **
//   **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **
// **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **

void setSeqState(uint8_t si, uint8_t flag, boolean value) {
	if (value) {
		seqState[si] |= flag;
	} else {
		seqState[si] &= ~flag;
	}
}
boolean getSeqState(uint8_t si, uint8_t flag) {
	return 0 != (seqState[si] & flag);
}

void initSeq(int8_t si) {
	seqState[si] = 0;
	//seqPage[si] = 0;
	seqMeasure[si] = 6;
	//seqLastUnprocessedStep[si] = -1;
	seqChannel[si] = 0;
	seqStart[si] = 0;
	seqEnd[si] = 0;
	seqBaseNote[si] = si + 40;
	seqStepValueToOff[si] = NO_NOTE_TO_OFF;
	seqPlayingNote[si] = NO_PLAYING_NOTE;
	seqScaleLo[si] = DEFAULT_SCALE_LO;
	seqScaleHi[si] = DEFAULT_SCALE_HI;
	seqPageSlot[si] = ((si/8)<<SEQ_PAGE_SHIFT) | (si%8);
	//seqSlot[si] = -1;
}


void setup() {
	if (DEBUG_MODE) {
		Serial.begin(9600); //debug serial output
		Serial.println("debug mode");
	} else {
		Serial.begin(31250); //MIDI
	}
	nlpCoreInit();
	
	analogWrite(14,255); //enable MIDI out
	analogWrite(15,255); //test
	analogWrite(16,255); //test
	
	for (uint8_t pageIndex = 0; pageIndex < PAGE_COUNT; pageIndex++) {
		for (uint8_t stepIndex = 0; stepIndex < STEP_COUNT; stepIndex++) {
			steps[pageIndex][stepIndex] = DEFAULT_STEP;
		}
		for (uint8_t slotIndex = 0; slotIndex < 8; slotIndex++) {
			slotSeqs[pageIndex][slotIndex] = SLOT_SEQS_NO_SEQ;
		}
	}
	currentTime = micros();
	metronomeLastStepTime = currentTime;
	//lastMetronomeBeatTime = currentTime;
	microsPerMetronomeStep = CBPM_STEP_MEASURE_RATIO / (cbpm * METRONOME_LENGTH);

	for (uint8_t si = 0; si < SEQ_COUNT; si++) {
		initSeq(si);
	}

}
//------------------------------------------------------------------------------

unsigned long timeDiff(unsigned long recentTime, unsigned long pastTime) {
	//TODO: overflow handling
	if (recentTime < pastTime) {
		return 0;
	} else {
		return recentTime - pastTime;
	}
}

boolean isItTime(long timeWeAreWaitingFor) {
	if (300 >= timeDiff(timeWeAreWaitingFor, currentTime)) { //300 = latency compensation test
		return true;
	}
	return false;
}
unsigned long getMicrosPerStep(uint8_t measure) {
	return microsPerMetronomeStep * measure;
}

void midiProcessInput() {
	uint8_t processedBytes = 0;
	while (Serial.available() && processedBytes < 4) {
		uint8_t midiDataByte = Serial.read();
		
		if (midiWaitingForIncoming == MIDI_WAITING_FOR_SYSEX && (midiDataByte & 128) != 0) {
			midiWaitingForIncoming = 0;
		}
		
		if (midiWaitingForIncoming == MIDI_WAITING_FOR_SYSEX && (midiDataByte & 128) == 0) {
			
			//NOP, just pass the sysex data
			
		} else if (midiWaitingForIncoming == 0) {
		
			if (midiDataByte == 240) {
				//processMidiOut = false;
				midiWaitingForIncoming = MIDI_WAITING_FOR_SYSEX;
			} else if (midiDataByte & 240 == 128) { //note off
				midiWaitingForIncoming = 2;
			} else if (midiDataByte & 240 == 144) { //note on
				midiWaitingForIncoming = 2;
			} else if (midiDataByte & 240 == 160) { //PKP aftertouch
				midiWaitingForIncoming = 2;
			} else if (midiDataByte & 240 == 176) { //CC
				midiWaitingForIncoming = 2;
			} else if (midiDataByte & 240 == 192) { //prog chng
				midiWaitingForIncoming = 1;
			} else if (midiDataByte & 240 == 208) { //chan aftertouch
				midiWaitingForIncoming = 1;
			} else if (midiDataByte & 240 == 224) { //Pitch Wheel Change
				midiWaitingForIncoming = 2;
			} else if (midiDataByte == 241) { //time code
				midiWaitingForIncoming = 1;
			} else if (midiDataByte == 242) { //song position
				midiWaitingForIncoming = 2;
			} else if (midiDataByte == 243) { //song select
				midiWaitingForIncoming = 1;
			} else if (midiDataByte == 248) { //beat clock
				
				if (midiBeatMaster) {
					midiBeatMaster = false;
					setLedRed(PAD_MIXER, LED_ON);
					setLedGreen(PAD_MIXER, 0);
				}
				
				if (midiBeatMaster) {
					midiPassThruFilterOut++;
				} else {
					metronomeLastStepTime = currentTime;
					processMetronomeStep();
				}
				midiWaitingForIncoming = 0;
			} else if (midiDataByte == 250) { //start
				midiWaitingForIncoming = 0;
			} else if (midiDataByte == 251) { //continue
				midiWaitingForIncoming = 0;
			} else if (midiDataByte == 252) { //stop
				midiWaitingForIncoming = 0;
			} else if (midiDataByte == 247) { //sysex end
				midiWaitingForIncoming = 0;
			} else {
				midiWaitingForIncoming = 0;
			}
		
		} else if (midiWaitingForIncoming > 0) {
			
			midiWaitingForIncoming--;
		}
		if (midiPassThruFilterOut==0) {
			Serial.write(midiDataByte);
		} else {
			midiPassThruFilterOut--;
		}
		
		/*if (midiWaitingForIncoming == 0) {
			processMidiOut = true;
		} else {
			processMidiOut = false;
		}*/
		
		processedBytes++;
	}
}

	/* * /
	if (getPad(PAD_MIXER)) {
		padMixerPressedTenthSeconds++;
		if (padMixerPressedTenthSeconds > 12) {
			if (midiBeatMaster) {
				midiBeatMaster = false;
				setLedRed(PAD_MIXER, LED_ON);
				setLedGreen(PAD_MIXER, 0);
			} else {
				midiBeatMaster = true;
				setLedRed(PAD_MIXER, 0);
				setLedGreen(PAD_MIXER, LED_ON);
			}
			padMixerPressedTenthSeconds = 0;
		}
	}
	/* */


void processMidiMasterMetronome() {
	if (isItTime(metronomeLastStepTime + microsPerMetronomeStep)) { //metronome
		metronomeLastStepTime += microsPerMetronomeStep;
		processMetronomeStep();
	}
}
void processMetronomeStep() {
		
	if (0 == metronomePosition) {
		//lastMetronomeBeatTime = metronomeLastStepTime;
		
		for (uint8_t i=0; i<SEQ_COUNT; i++) {
			if (getSeqState(i, SEQ_STATE_ARMED)) {
				setSeqState(i, SEQ_STATE_ARMED, false);
				setSeqState(i, SEQ_STATE_PLAYING, true);
				if (((seqPageSlot[i] && SEQ_PAGE_MASK) >> SEQ_PAGE_SHIFT) == activePage) {
				//if (seqPage[i] == activePage && seqSlot[i]>=0 && seqSlot[i]<=7) {
					if (i == activeSeqIndex) {
						setLed(SEQ_SLOT_MASK & seqPageSlot[i],ARROW_SEQ_ACTIVE_PLAYING);
					} else {
						setLed(SEQ_SLOT_MASK & seqPageSlot[i],ARROW_SEQ_PLAYING);
					}
				}
			}
		}

		if (midiBeatMaster) {
			setLed(PAD_MIXER,LED_AMBER);
			//setLedGreen(PAD_MIXER, LED_ON);
			//setLedRed(PAD_MIXER, LED_ON);
		} else {
			setLed(PAD_MIXER,LED_ORANGE);
			//setLedGreen(PAD_MIXER, LED_DIM);
			//setLedRed(PAD_MIXER, LED_ON);
		}
	} else if (METRONOME_HALF_LENGTH == metronomePosition) {
		if (midiBeatMaster) {
			setLed(PAD_MIXER, LED_GREEN);
			//setLedGreen(PAD_MIXER, LED_ON);
			//setLedRed(PAD_MIXER, 0);
		} else {
			setLed(PAD_MIXER, LED_RED);
			//setLedRed(PAD_MIXER, LED_ON);
		}
	} else {
		setLed(PAD_MIXER, 0);
		//setLedGreen(PAD_MIXER, 0);
		//setLedRed(PAD_MIXER, 0);
	}
	metronomePosition = ((metronomePosition + 1) % METRONOME_LENGTH);

	midiBeatClock();
	queuedProcessSeqIndex = 0; //start processing sequences
}

void setSeqStep(uint8_t seqIndex, uint8_t stepIndex, boolean isCurrentlyPlaying) {
	if (((seqPageSlot[seqIndex] & SEQ_PAGE_MASK)>> SEQ_PAGE_SHIFT) == activePage) {
		if (isCurrentlyPlaying) { //playing current step
			
			if (EMPTY_STEP != (steps[activePage][stepIndex] & STEPS_NOTE_MASK)) { //toggled step
				if (seqIndex == activeSeqIndex) {
					if (getSeqState(seqIndex, SEQ_STATE_HIGHLIGHTED)) {
						//highlighted toggled
						setStepLed(stepIndex,LED_ORANGE);
					} else { //active toggled
						setStepLed(stepIndex,LED_ORANGE);
					}
				} else { //regular toggled step
					setStepLed(stepIndex,LED_AMBER);
				}
			} else { //empty step (no note)
				if (seqIndex == activeSeqIndex) {
					//active empty (both highlighted or not)
					setStepLed(stepIndex,LED_AMBER);
				} else { //regular empty
					setStepLed(stepIndex,LED_GREEN);
				}
			}
			
		} else { //other non-playing steps (turn off step not playing any more)
			
			if (EMPTY_STEP != (steps[activePage][stepIndex] & STEPS_NOTE_MASK)) { //toggled step
				if (seqIndex == activeSeqIndex) {
					if (getSeqState(seqIndex, SEQ_STATE_HIGHLIGHTED)) {
						//highlighted toggled
						setStepLed(stepIndex,LED_AMBER);
					} else { //active toggled
						setStepLed(stepIndex,LED_RED);
					}
				} else { //regular toggled step
					setStepLed(stepIndex,LED_RED);
				}
			} else { //empty step (no note)
				if (seqIndex == activeSeqIndex) {
					if (getSeqState(seqIndex, SEQ_STATE_HIGHLIGHTED)) {
						//highlighted empty
						setStepLed(stepIndex,LED_GREEN);
					} else { //active empty
						setStepLed(stepIndex,0);
					}
				} else { //regular empty
					setStepLed(stepIndex, 0);
				}
			}
		}
	}
}

void checkNoteOff(int8_t seqIndex, uint8_t checkOffType) { //STEPS_OFF_ONE or STEPS_OFF_NEXT
	if (
		//EMPTY_STEP != (seqStepValueToOff[seqIndex] & STEPS_NOTE_MASK) &&
		(seqStepValueToOff[seqIndex] & STEPS_OFF_MASK) == checkOffType
	) {
		int8_t oldSeqNote = getSeqNote(seqIndex, seqStepValueToOff[seqIndex]);
		midiNoteOff(oldSeqNote, seqChannel[seqIndex]);
		seqStepValueToOff[seqIndex] = NO_NOTE_TO_OFF;
		seqPlayingNote[seqIndex] = NO_PLAYING_NOTE;
	}
}


void queuedProcessSeq() {	
	uint8_t si = queuedProcessSeqIndex;
	
	if (getSeqState(si, SEQ_STATE_PLAYING)) { //TODO: clear previous led even when seq just stopped playing
		seqBeatClocksSinceLastStep[si]++;
		if (seqBeatClocksSinceLastStep[si] >= seqMeasure[si]) {
			seqBeatClocksSinceLastStep[si] = 0;

			//note off
			if (seqStepValueToOff[si] != NO_NOTE_TO_OFF) {
				checkNoteOff(si, STEPS_OFF_ONE);
			}

			//clear previous step LED
			redrawPreviousStep(si);


			seqPosition[si]++;
			if (seqPosition[si] > seqEnd[si]) {
				seqPosition[si] = seqStart[si];
			}
			if (midiBeatMaster && midiBeatState == MIDI_BEAT_ARMED && seqPosition[si] == seqStart[si]) {
				midiBeatState = MIDI_BEAT_PLAYING;
				midiBeatStart();
			}
			processStep(si);
		}
	}
	queuedProcessSeqIndex++;
}

int8_t getSeqNote(int8_t seqIndex, int8_t stepValue) { //used in checkNoteOff and processStep
	/* */
	uint8_t scaleSize = 0;
	uint8_t scaleValues[MAX_SCALE_SIZE];
	uint8_t scaleOctaves = 1;
	for (int8_t i=0; i<MAX_SCALE_SIZE; i++) {
		if (i<16 && 0 != (seqScaleLo[seqIndex] & (1<<i))) {
			scaleValues[scaleSize] = i;
			scaleSize++;
		} else if (i>=16 && 0 != (seqScaleHi[seqIndex] & (1<<(i-16)))) {
			scaleValues[scaleSize] = i;
			scaleSize++;
			scaleOctaves = 2;
		}
	}
	uint8_t stepNoteValue = (stepValue & STEPS_NOTE_MASK) - BASE_STEP;
	//TODO: light up note in scale editor?
	return seqBaseNote[seqIndex] + scaleValues[stepNoteValue % scaleSize] + ((int)(stepNoteValue/scaleSize))*12*scaleOctaves;
	/* */
	//return 0;
}

void processStep(int8_t seqIndex) {
	int8_t newSeqNote = -1;
	if (EMPTY_STEP != (steps[((seqPageSlot[seqIndex] & SEQ_PAGE_MASK)>> SEQ_PAGE_SHIFT)][seqPosition[seqIndex]] & STEPS_NOTE_MASK)) {
		
		if (seqStepValueToOff[seqIndex] != NO_NOTE_TO_OFF) checkNoteOff(seqIndex, STEPS_OFF_NEXT);
		
		newSeqNote = getSeqNote(seqIndex, steps[((seqPageSlot[seqIndex] & SEQ_PAGE_MASK)>> SEQ_PAGE_SHIFT)][seqPosition[seqIndex]]);
		midiPlayNote(newSeqNote, seqChannel[seqIndex]);
		
		if (seqStepValueToOff[seqIndex] != NO_NOTE_TO_OFF) checkNoteOff(seqIndex, STEPS_OFF_TIE);
		
		seqStepValueToOff[seqIndex] = steps[((seqPageSlot[seqIndex] & SEQ_PAGE_MASK)>> SEQ_PAGE_SHIFT)][seqPosition[seqIndex]];
		seqPlayingNote[seqIndex] = newSeqNote;
	}
	drawCurrentStep(seqIndex,newSeqNote); //originally was before non-empty step processing
}


void midiProcessBufferedOutput() {
	uint8_t bufferOffset = 0;
	while (midiWaitingForIncoming == 0 && playBufferStart != playBufferEnd && bufferOffset < 4) { //play max 4 notes at a time
		switch (MIDI_BUFFERED_COMMAND_MASK & playBufferCommand[playBufferStart]) {
			case MIDI_BUFFERED_NOTE_ON:
				Serial.write(MIDI_NOTE_ON | (MIDI_BUFFERED_CHANNEL_MASK & playBufferCommand[playBufferStart])); //note on B10010000 | 4bit channel 
				Serial.write(playBufferData[playBufferStart]);
				Serial.write(127);
				break;
			case MIDI_BUFFERED_NOTE_OFF:
				Serial.write(MIDI_NOTE_ON | (MIDI_BUFFERED_CHANNEL_MASK & playBufferCommand[playBufferStart])); //note on B10010000 | 4bit channel 
				Serial.write(playBufferData[playBufferStart]);
				Serial.write(0);
				break;
			case MIDI_BUFFERED_BEAT_CLOCK:
				Serial.write(MIDI_BEAT_CLOCK);
				break;
			case MIDI_BUFFERED_BEAT_START:
				Serial.write(MIDI_BEAT_START);
				break;
			case MIDI_BUFFERED_BEAT_STOP:
				Serial.write(MIDI_BEAT_STOP);
				break;
		}
		bufferOffset++;
		playBufferStart = (playBufferStart + 1) % PLAY_BUFFER_SIZE;
	}
}

void midiBufferCommand(uint8_t command, uint8_t channel, uint8_t data) {
	if (sendMidi) {
		if (((playBufferEnd + 1) % PLAY_BUFFER_SIZE) == playBufferStart) { //buffer full, try to empty
			midiProcessBufferedOutput();
		}
		playBufferCommand[playBufferEnd] = command | channel;
		playBufferData[playBufferEnd] = data;
		playBufferEnd = (playBufferEnd + 1) % PLAY_BUFFER_SIZE;
	}
}

void midiBeatStart() {
	if (midiBeatMaster) {
		midiBufferCommand(MIDI_BUFFERED_BEAT_START, 0, 0);
	}
}
void midiBeatStop() {
	if (midiBeatMaster) {
		midiBufferCommand(MIDI_BUFFERED_BEAT_STOP, 0, 0);
	}
}

void midiBeatClock() {
	if (midiBeatMaster) {
		midiBufferCommand(MIDI_BUFFERED_BEAT_CLOCK, 0, 0);
	}
}

void midiPlayNote(int8_t midiNote, uint8_t midiChannel) {
	if (sendMidi && midiNote < 128) {
		midiBufferCommand(MIDI_BUFFERED_NOTE_ON, midiChannel, midiNote);
	}

}

void midiNoteOff(int8_t midiNote, uint8_t midiChannel) {
	if (sendMidi && midiNote < 128) {
		midiBufferCommand(MIDI_BUFFERED_NOTE_OFF, midiChannel, midiNote);
	}
}

void addSeq(int8_t seqStartIndex, int8_t seqEndIndex, int8_t measure, int8_t slotIndex) {
	int8_t si = 0;
	while (si < SEQ_COUNT && seqState[si] != SEQ_STATE_OFF) si++; //allocate new sequence index
	
	for (int8_t slotOffset=0; slotOffset<8 && slotSeqs[activePage][slotIndex] >= 0; slotOffset++) 
		slotIndex = (slotIndex+1)%8;
	
	if (
		si < SEQ_COUNT && seqState[si] == SEQ_STATE_OFF &&
		slotIndex < 8 && slotIndex >= 0 &&
		slotSeqs[activePage][slotIndex] == SLOT_SEQS_NO_SEQ //if slot is not free, do nothing
	) {
		if (activeSeqIndex >= 0) setSeqState(activeSeqIndex, SEQ_STATE_HIGHLIGHTED, false); //unhighlight previously selected seq
		
		//seqPage[si] = activePage;
		seqEnd[si] = seqEndIndex;
		seqStart[si] = seqStartIndex;
		slotSeqs[activePage][slotIndex] = si;
		seqPageSlot[si] = (activePage << SEQ_PAGE_SHIFT) | slotIndex;
		//seqSlot[si] = slotIndex;
		
		setLed(slotIndex,ARROW_SEQ_ARMED); //seq in slot armed
		
		seqBaseNote[si] = notePickerCurrentNote;
		seqChannel[si] = (notePickerState & NOTE_PICKER_STATE_MASK_CHANNEL);

		if (measure <= 0) {
			seqMeasure[si] = 6;
		} else {
			seqMeasure[si] = measure;
		}
		
		if (seqStartIndex == 0 && seqEndIndex == 0) {
			
			seqState[si] = SEQ_STATE_DEFINED_WITHOUT_STEPS;
			//TODO: setup seq almost like stopped/paused ?
			
		} else {
			
			setSeqState(si, SEQ_STATE_SET, true);
			setSeqState(si, SEQ_STATE_ARMED, true);
			setSeqState(si, SEQ_STATE_PLAYING, false);
		}
		
		//TODO: do this other stuff also for defined_without_steps ???
		
		seqBeatClocksSinceLastStep[si] = seqMeasure[si];

		seqPosition[si] = seqEnd[si];
		activeSeqIndex = si;

		if (midiBeatMaster && midiBeatState == MIDI_BEAT_STOPPED) {
			boolean isFirstSeq = true;
			for (uint8_t i=0; i<SEQ_COUNT; i++) if (si!=i && seqState[si] != SEQ_STATE_OFF)	isFirstSeq = false;
			if (isFirstSeq) midiBeatState = MIDI_BEAT_ARMED;
		}
		
	}
}

void deleteSeq(int8_t si) {
	seqEnd[si] = 0;
	seqStart[si] = 0;
	seqState[si] = SEQ_STATE_OFF; //this will also unhighlight the seq
	seqBeatClocksSinceLastStep[si] = 0;
	
	slotSeqs[(seqPageSlot[si] & SEQ_PAGE_MASK)>> SEQ_PAGE_SHIFT][seqPageSlot[si] & SEQ_SLOT_MASK] = SLOT_SEQS_NO_SEQ;
	//slotSeqs[seqPage[si]][seqSlot[si]] = SLOT_SEQS_NO_SEQ;
	
	//seqSlot[si] = -1;

	if (si == activeSeqIndex) {
		do {
			if (activeSeqIndex==0) {
				activeSeqIndex = SEQ_COUNT - 1;
			} else {
				activeSeqIndex--;
			}
		} while (seqState[activeSeqIndex] == SEQ_STATE_OFF && si != activeSeqIndex);
		if (si == activeSeqIndex) {
			activeSeqIndex = -1;
			
			if (midiBeatMaster && midiBeatState == MIDI_BEAT_PLAYING) {
				midiBeatStop();
				midiBeatState = MIDI_BEAT_STOPPED;
			}
			
			
		} else if (activePage != ((seqPageSlot[activeSeqIndex] & SEQ_PAGE_MASK)>> SEQ_PAGE_SHIFT)) {
			activePage = ((seqPageSlot[activeSeqIndex] & SEQ_PAGE_MASK)>> SEQ_PAGE_SHIFT);
			redrawSeqPlayer();
		}
	}
}

void setStepRed(int8_t stepIndex, int8_t ledValue) {
	if (stepIndex>=0 && stepIndex < STEP_COUNT) {
		setLedRed((int8_t) pgm_read_byte(&(stepToLed[stepIndex])), ledValue);
	}
}
void setStepGreen(int8_t stepIndex, int8_t ledValue) {
	if (stepIndex>=0 && stepIndex < STEP_COUNT) {
		setLedGreen((int8_t) pgm_read_byte(&(stepToLed[stepIndex])), ledValue);
	}
}
void setStepLed(int8_t stepIndex, int8_t ledValue) {
	if (stepIndex>=0 && stepIndex < STEP_COUNT) {
		setLed((int8_t) pgm_read_byte(&(stepToLed[stepIndex])), ledValue);
	}
}

void setSeqBaseNoteAndChannel(int8_t seqIndex, int8_t newNote, int8_t newChannel) { //TODO: correct int type
	//TODO: recalc seqStepValueToOff when changing base note
	seqBaseNote[seqIndex] = newNote;
	//TODO: turn off playing note before changing channel
	seqChannel[seqIndex] = newChannel;
}

void setNotePicker(uint8_t stepIndex) {
	int8_t toneOffset = (int8_t) pgm_read_byte(&(stepToToneOffset[stepIndex]));
	if (toneOffset >= 0) {
		int8_t newNote = 12 * ((notePickerState & NOTE_PICKER_STATE_MASK_BOTTOM)>> NOTE_PICKER_STATE_SHIFT_BOTTOM) + toneOffset;
		if (newNote < 128) {
			if (applyNotePickerToSeqIndex >= 0) {
				
				setSeqBaseNoteAndChannel(applyNotePickerToSeqIndex, newNote, (notePickerState & NOTE_PICKER_STATE_MASK_CHANNEL));
				
				//TODO: when changing measure, reset and/or rearm timing?
				seqMeasure[applyNotePickerToSeqIndex] = notePickerCurrentMeasure;
				
			} else if (activeSeqIndex < 0 || seqState[activeSeqIndex] == SEQ_STATE_OFF) {
				midiNoteOff(notePickerCurrentNote, (notePickerState & NOTE_PICKER_STATE_MASK_CHANNEL));
				midiPlayNote(newNote, (notePickerState & NOTE_PICKER_STATE_MASK_CHANNEL));
			}
			if (notePickerCurrentNote != newNote) {
				notePickerCurrentNote = newNote;
				redrawNotePicker();
			}
		}
	}
}

void setNotePickerChannel(uint8_t newChannel) {
	if (applyNotePickerToSeqIndex >= 0) {
		seqChannel[applyNotePickerToSeqIndex] = newChannel;
	}
	redrawNotePicker();
}

void clearStepLeds() {
	for (int8_t stepIndex = 0; stepIndex < STEP_COUNT; stepIndex++) {
		setStepLed(stepIndex, 0);
	}
}

void highlightActiveSeq() {
	setSeqState(activeSeqIndex, SEQ_STATE_HIGHLIGHTED, true);
	for (uint8_t stepSeqMarkIndex = seqStart[activeSeqIndex]; stepSeqMarkIndex <= seqEnd[activeSeqIndex]; stepSeqMarkIndex++) {
		setSeqStep(activeSeqIndex,stepSeqMarkIndex,stepSeqMarkIndex == seqPosition[activeSeqIndex]);
	}
}


void redrawPreviousStep(int8_t seqIndex) { //turn off drawn previously played step
	if (
		ledMode == LED_MODE_SEQ_RUN && 
		((seqPageSlot[seqIndex] & SEQ_PAGE_MASK)>> SEQ_PAGE_SHIFT) == activePage && 
		seqPosition[seqIndex]>=0 && seqPosition[seqIndex] < STEP_COUNT
	) { 
		setSeqStep(seqIndex,seqPosition[seqIndex],false);
	} else if (ledMode == LED_MODE_SCALE_PICKER && seqIndex == activeSeqIndex) {
		
		//TODO
		redrawNotePicker();
		
	} else if (ledMode == LED_MODE_SCALE_RUN && seqIndex == activeSeqIndex) {
		drawScaleStep(seqPosition[seqIndex],false);
	}
}
void drawCurrentStep(int8_t seqIndex, int8_t seqNote) {
	if (ledMode == LED_MODE_SEQ_RUN) { //and if (((seqPageSlot[seqIndex] & SEQ_PAGE_MASK)>> SEQ_PAGE_SHIFT) == activePage)
		
		setSeqStep(seqIndex,seqPosition[seqIndex],true);
		
	} else if (ledMode == LED_MODE_SCALE_PICKER && seqIndex == activeSeqIndex && seqNote>=seqBaseNote[seqIndex]) {
		
		setStepLed(56 + tonePickerOffset[(seqNote-seqBaseNote[seqIndex]) % 12], LED_GREEN);
		//setStepLed(56 + seqNote-seqBaseNote[seqIndex], LED_GREEN);
		//setStepLed(56 + tonePickerOffset[0], LED_GREEN);
		
	} else if (ledMode == LED_MODE_SCALE_RUN && seqIndex == activeSeqIndex) {
		
		drawScaleStep(seqPosition[seqIndex],true);
		redrawScalePlayerScrollbar(seqPosition[seqIndex]-seqStart[seqIndex]);
		
	}
}

//TODO: better gui
int8_t setMeasureFromStepPadIndex(int8_t stepPadIndex) {
	//notePickerCurrentMeasure = stepPadIndex+1;
	/* */
	uint8_t measureModulo = 8;
	while (measureModulo>1 && (notePickerCurrentMeasure < measureModulo || (notePickerCurrentMeasure % measureModulo)!=0)) {
		measureModulo--;
	}
	if (stepPadIndex>=8 && stepPadIndex<=15) {
		notePickerCurrentMeasure = notePickerCurrentMeasure/measureModulo*(stepPadIndex-7);
	} else if (stepPadIndex>=0 && stepPadIndex<=7) {
		notePickerCurrentMeasure = measureModulo*(stepPadIndex+1);
	}
	/* */
}
void redrawMeasurePicker() {
	/* * /
	for (uint8_t measureIndicator = 0; measureIndicator < 16; measureIndicator++) {
		if (measureIndicator < notePickerCurrentMeasure) {
			setStepLed(measureIndicator, LED_ORANGE);
		} else {
			setStepLed(measureIndicator, LED_YELLOW);
		}
	}
	/* */
	uint8_t measureModulo = 8;
	while (measureModulo>1 && (notePickerCurrentMeasure < measureModulo || (notePickerCurrentMeasure % measureModulo)!=0)) {
		measureModulo--;
		setStepLed(measureModulo+7,0);
	}
	for (uint8_t measureIndicator = 1; measureIndicator <= measureModulo; measureIndicator++)
		setStepLed(measureIndicator+7,LED_YELLOW);
	for (uint8_t measureIndicator = 1; measureIndicator <= notePickerCurrentMeasure/measureModulo; measureIndicator++)
		setStepLed(measureIndicator-1,LED_ORANGE);
	for (uint8_t measureIndicator = 1+notePickerCurrentMeasure/measureModulo; measureIndicator<=8; measureIndicator++)
		setStepLed(measureIndicator-1,0);
	/* */
}
//------------------------

void redrawNotePicker() {
	clearStepLeds();
	uint8_t octaveStartStep = 56;
	int8_t currentNoteOctave = notePickerCurrentNote / 12 - 2; //-2 .. 8
	uint8_t currentNoteTone = notePickerCurrentNote % 12; //0..11
	if (padMode == PAD_MODE_NOTE_PICKER) {
		for (uint8_t octaveDrawingIndex = 0; octaveDrawingIndex < 4; octaveDrawingIndex++) {
			setStepRed(octaveStartStep - 8, LED_ON); //C#
			setStepRed(octaveStartStep - 7, LED_ON); //D#
			setStepRed(octaveStartStep - 5, LED_ON); //F#
			setStepRed(octaveStartStep - 4, LED_ON); //G#
			setStepRed(octaveStartStep - 3, LED_ON); //A#

			if (activeSeqIndex < 0 || padMode != PAD_MODE_SCALE_PICKER) {

				if (((notePickerState & NOTE_PICKER_STATE_MASK_BOTTOM)>> NOTE_PICKER_STATE_SHIFT_BOTTOM)-2 + octaveDrawingIndex > 0) {
					for (
						uint8_t octaveMarker = 0; 
						octaveMarker < ((notePickerState & NOTE_PICKER_STATE_MASK_BOTTOM)>> NOTE_PICKER_STATE_SHIFT_BOTTOM)-2 + octaveDrawingIndex;
						octaveMarker++
					) {
						setStepGreen(octaveStartStep + octaveMarker, LED_ON);
					}
				} else if (((notePickerState & NOTE_PICKER_STATE_MASK_BOTTOM)>> NOTE_PICKER_STATE_SHIFT_BOTTOM)-2 + octaveDrawingIndex == -1) { //negative octaves
					setStepGreen(octaveStartStep, LED_ON);
					setStepRed(octaveStartStep, LED_ON);
				} else if (((notePickerState & NOTE_PICKER_STATE_MASK_BOTTOM)>> NOTE_PICKER_STATE_SHIFT_BOTTOM)-2 + octaveDrawingIndex == -2) {
					setStepGreen(octaveStartStep, LED_ON);
					setStepRed(octaveStartStep, LED_ON);
					setStepGreen(octaveStartStep + 1, LED_ON);
					setStepRed(octaveStartStep + 1, LED_ON);
				}

				//currently selected note
				if (currentNoteOctave == ((notePickerState & NOTE_PICKER_STATE_MASK_BOTTOM)>> NOTE_PICKER_STATE_SHIFT_BOTTOM)-2 + octaveDrawingIndex) { 
					setStepLed(octaveStartStep + tonePickerOffset[currentNoteTone], LED_ORANGE);
				}
			}

			octaveStartStep -= 16;
		}
		//toggle selected note preview midi send
		if (0 != (notePickerState & NOTE_PICKER_STATE_MASK_PREVIEW)) {
			setLed(NOTE_PICKER_PREVIEW_PAD, LED_AMBER);
		} else {
			setLed(NOTE_PICKER_PREVIEW_PAD, LED_RED);
		}
		//channel picker
		setLed(16, ((1 & (notePickerState & NOTE_PICKER_STATE_MASK_CHANNEL)) > 0)?LED_AMBER:0);
		setLed(15, ((2 & (notePickerState & NOTE_PICKER_STATE_MASK_CHANNEL)) > 0)?LED_AMBER:0);
		setLed(14, ((4 & (notePickerState & NOTE_PICKER_STATE_MASK_CHANNEL)) > 0)?LED_AMBER:0);
		setLed(13, ((8 & (notePickerState & NOTE_PICKER_STATE_MASK_CHANNEL)) > 0)?LED_AMBER:0);
		
	} else { //"scale picker" mode // if (padMode == PAD_MODE_SCALE_PICKER) {
		
		if (activeSeqIndex >=0) {
			for (uint8_t i=0; i<MAX_SCALE_SIZE; i++) {

				if (i<16) {
					if (0 != (seqScaleLo[activeSeqIndex] & (1 << i))) 
						setStepLed((uint8_t) pgm_read_byte(&(scaleBitToStep[i])), LED_ORANGE);
				} else {
					if (0 != (seqScaleHi[activeSeqIndex] & (1 << (i-16)))) 
						setStepLed((uint8_t) pgm_read_byte(&(scaleBitToStep[i])), LED_ORANGE);
				}

			}
		}
		//measure picker (moved from "note picker" into the "scale picker" mode)
		redrawMeasurePicker();
		
	}
}

//void lightScaleOffsetStep(uint8_t seqIndex, uint8_t) {
//}
int8_t seqPosToScaleStep(int8_t seqPos) {
	int8_t noteValue = (steps[activePage][seqPos] & STEPS_NOTE_MASK) - BASE_STEP;
	if (noteValue == EMPTY_STEP - BASE_STEP) {
		//no note defined on step, indicate bottom row position for seq runner
		return SCALE_STEP_EMPTY + 48 + (seqPos - seqStart[activeSeqIndex] - scalePlayerPosition);
		
	} else if (noteValue < scalePlayerOffset) { //toggled step is under viewing window

		return SCALE_STEP_BELOW + 48 + (seqPos - seqStart[activeSeqIndex] - scalePlayerPosition);

	} else if (noteValue > scalePlayerOffset + 6) { //toggled step is above viewing window (7 rows: 0..6, the 8th row is scrollbar)

		return SCALE_STEP_ABOVE + (seqPos - seqStart[activeSeqIndex] - scalePlayerPosition);

	} else {
		return (seqPos - seqStart[activeSeqIndex] - scalePlayerPosition)
		+ 8 * (6 - noteValue + scalePlayerOffset);
	}
}

void drawScaleStep(int8_t seqStepIndex, boolean isCurrentlyPlaying) {

	if (
		seqStepIndex - seqStart[activeSeqIndex] >= scalePlayerPosition &&
		seqStepIndex - seqStart[activeSeqIndex] <= scalePlayerPosition + 7 //only if it is in current viewing window
	) {
		int8_t scaleStepIndex = seqPosToScaleStep(seqStepIndex);
		switch (scaleStepIndex & SCALE_STEP_FLAG) {

			case SCALE_STEP_BELOW:
				if (seqPosition[activeSeqIndex] == seqStepIndex && isCurrentlyPlaying) {
					setStepLed(scaleStepIndex & SCALE_STEP_VALUE, LED_AMBER);
				} else {
					setStepLed(scaleStepIndex & SCALE_STEP_VALUE, LED_GREEN);
				}
				break;
			case SCALE_STEP_ABOVE:
				setStepLed(scaleStepIndex & SCALE_STEP_VALUE, LED_GREEN);
				break;
			case SCALE_STEP_EMPTY:
				if (seqPosition[activeSeqIndex] == seqStepIndex && isCurrentlyPlaying) {
					setStepLed(scaleStepIndex & SCALE_STEP_VALUE, LED_AMBER);
				} else {
					setStepLed(scaleStepIndex & SCALE_STEP_VALUE, 0);
				}
				break;
			default:
				if (seqPosition[activeSeqIndex] == seqStepIndex && isCurrentlyPlaying) {
					setStepLed(scaleStepIndex, LED_RED);
				} else {
					setStepLed(scaleStepIndex, LED_ORANGE);
				}
		}
	}

}
void redrawScalePlayerScrollbar(int8_t seqPosOffset) {
	for (uint8_t i=0; i<=(seqEnd[activeSeqIndex]-seqStart[activeSeqIndex])/8; i++) {
		if (i == scalePlayerPosition/8) { //current window position
			//processing step in current window
			if (seqPosOffset >= 0 && (seqPosOffset/8) >= i && (seqPosOffset/8) < i+1) { 
				setStepLed(i+56,LED_ORANGE);
			} else {
				setStepLed(i+56,LED_RED);
			}
		} else {
			//processing step in some other position
			if (seqPosOffset >= 0 && (seqPosOffset/8) >= i && (seqPosOffset/8) < i+1) { 
				setStepLed(i+56,LED_AMBER);
			} else {
				setStepLed(i+56,LED_GREEN);
			}
		}
	}
		
	
}
void redrawScalePlayer() {
	clearStepLeds();
	if (activeSeqIndex >= 0 && seqState[activeSeqIndex] != SEQ_STATE_OFF) {

		if (scalePlayerPosition == 0) {
			setLed(PAD_LEFT,0);
		} else {
			setLed(PAD_LEFT,LED_GREEN);
		}

		if (scalePlayerOffset > 0) {
			setLed(PAD_DOWN,LED_GREEN);
		} else {
			setLed(PAD_DOWN,0);
		}
		
		for (uint8_t winPos=0; winPos<8 && seqStart[activeSeqIndex]+winPos+scalePlayerPosition <= seqEnd[activeSeqIndex]; winPos++) {
			drawScaleStep(seqStart[activeSeqIndex]+winPos+scalePlayerPosition, true);
			
			if (scalePlayerOffset >= STEPS_ACC_VALUE-6 && scalePlayerOffset <= STEPS_ACC_VALUE+6) { //26 = 32-6, 38 = 32+6
				if (0 != (steps[activePage][seqStart[activeSeqIndex]+winPos+scalePlayerPosition] & STEPS_ACC)) {
					setStepLed(winPos + (scalePlayerOffset+6-STEPS_ACC_VALUE)*8, LED_RED);
				} else {
					setStepLed(winPos + (scalePlayerOffset+6-STEPS_ACC_VALUE)*8, LED_GREEN);
				}
			}
			if (
				scalePlayerOffset >= STEPS_OFF_ONE_VALUE-6 && scalePlayerOffset <= STEPS_OFF_ONE_VALUE+6 && //27 = 33-6, 39 = 33+6
				0 != (steps[activePage][seqStart[activeSeqIndex]+winPos+scalePlayerPosition] & STEPS_OFF_ONE)
			) {
				setStepLed(winPos + (scalePlayerOffset+6-STEPS_OFF_ONE_VALUE)*8, LED_AMBER);
			}
			if (scalePlayerOffset >= STEPS_OFF_NEXT_VALUE-6 && scalePlayerOffset <= STEPS_OFF_NEXT_VALUE+6) { //28 = 34-6, 40 = 34+6
				if (0 != (steps[activePage][seqStart[activeSeqIndex]+winPos+scalePlayerPosition] & STEPS_OFF_NEXT)) {
					setStepLed(winPos + (scalePlayerOffset+6-STEPS_OFF_NEXT_VALUE)*8, LED_AMBER);
				//} else {
				//	setStepLed(winPos + (scalePlayerOffset+6-STEPS_OFF_NEXT_VALUE)*8, LED_GREEN);
				}
			}
		}
		redrawScalePlayerScrollbar(-1);
		for (uint8_t i=0; i<3; i++) setLed(i,0);
		for (uint8_t i=3; i<8; i++) setLed(i,LED_GREEN);
		setLed(7-(scalePlayerOffset/7),LED_RED); //scrollbar position
		
	}
}


void redrawSeqPlayer() {
	clearStepLeds();
	
	setLed(PAD_UP,0);
	setLed(PAD_DOWN,0);
	setLed(PAD_LEFT,0);
	setLed(PAD_RIGHT,0);
	
	for (uint8_t stepIndex = 0; stepIndex < STEP_COUNT; stepIndex++) {
		if (EMPTY_STEP != (steps[activePage][stepIndex] & STEPS_NOTE_MASK)) {
			setStepLed(stepIndex, LED_RED);
		}
	}
	
	for (uint8_t si = 0; si < SEQ_COUNT; si++) {
		if (
			((seqPageSlot[si] & SEQ_PAGE_MASK)>> SEQ_PAGE_SHIFT) == activePage && 
			seqState[si] != SEQ_STATE_OFF && seqPosition[si] >= 0
		) {
			
			if (si == activeSeqIndex) { //active seq
				for (uint8_t actStpIndx = seqStart[si]; actStpIndx<=seqEnd[si]; actStpIndx++) {
					setSeqStep(si, actStpIndx, actStpIndx == seqPosition[si]);
				}
			} else {
				setSeqStep(si, seqPosition[si], true);
			}
		}
	}
	for (int8_t slotIndex = 0; slotIndex < 8; slotIndex++) { //right arrows
		setLed(slotIndex,0);
		int8_t si = slotSeqs[activePage][slotIndex];
		if (si >= 0 && seqState[si] != SEQ_STATE_OFF) {
			if (getSeqState(si, SEQ_STATE_ARMED)) {
				setLed(slotIndex,ARROW_SEQ_ARMED);
			} else if (getSeqState(si, SEQ_STATE_PLAYING)) {
				if (si == activeSeqIndex) {
					setLed(slotIndex,ARROW_SEQ_ACTIVE_PLAYING);
				} else {
					setLed(slotIndex,ARROW_SEQ_PLAYING);
				}
			} else {
				setLed(slotIndex,ARROW_SEQ_PAUSED);
			}
		}
	}
}

void switchLedMode(uint8_t newLedMode) {
	if (newLedMode == LED_MODE_SEQ_RUN) {
		redrawSeqPlayer();
		ledMode = LED_MODE_SEQ_RUN;
	} else if (newLedMode == LED_MODE_NOTE_PICKER && ledMode != newLedMode) {
		redrawNotePicker();
		ledMode = LED_MODE_NOTE_PICKER;
	} else if (newLedMode == LED_MODE_SCALE_PICKER) {
		ledMode = LED_MODE_SCALE_PICKER;
		redrawNotePicker();
	} else if (newLedMode == LED_MODE_SCALE_RUN) {
		redrawScalePlayer();
		ledMode = LED_MODE_SCALE_RUN;
	}
}

int8_t addSeqHandlingStepStart = -1;
//---------------------------------------------------------------------------------------------------------------

void setActiveSeqStartEnd(int8_t startStepIndex,int8_t endStepIndex) {
						
	seqStart[activeSeqIndex] = startStepIndex;
	seqEnd[activeSeqIndex] = endStepIndex;
	seqState[activeSeqIndex] = 0; //setSeqState(activeSeqIndex, SEQ_STATE_DEFINED_WITHOUT_STEPS, false);
	setSeqState(activeSeqIndex, SEQ_STATE_SET, true);
	setSeqState(activeSeqIndex, SEQ_STATE_ARMED, true);
	setSeqState(activeSeqIndex, SEQ_STATE_PLAYING, false);
	seqBeatClocksSinceLastStep[activeSeqIndex] = seqMeasure[activeSeqIndex];
	seqPosition[activeSeqIndex] = seqEnd[activeSeqIndex];
	if (midiBeatMaster && midiBeatState == MIDI_BEAT_STOPPED) {
		boolean isFirstSeq = true;
		for (uint8_t i=0; i<SEQ_COUNT; i++) 
			if (activeSeqIndex != i && seqState[activeSeqIndex] != SEQ_STATE_OFF)	
				isFirstSeq = false;
		if (isFirstSeq) midiBeatState = MIDI_BEAT_ARMED;
	}
}

						
int8_t getOtherPressedStep(int8_t padIndex, int8_t ignorePadIndex) {
	int8_t otherPressedStep = OTHER_STEP_NONE;
					
	for (int8_t i = 0; i<80; i++) {
		if (getPad(i) && i != padIndex && (ignorePadIndex<0 || ignorePadIndex!=i)) {
			if (otherPressedStep == OTHER_STEP_NONE) {
				otherPressedStep = (int8_t)pgm_read_byte(&(padToStep[i]));
				if (otherPressedStep == OTHER_STEP_WRONG) break;
			} else {
				otherPressedStep = OTHER_STEP_WRONG;
				break;
			}
		}
	}
	return otherPressedStep;
}


//---------------------------------------------------------------------------------------------------------------

void padOnHandler(uint8_t padIndex) {
	
	int8_t stepPadIndex = (int8_t) pgm_read_byte(&(padToStep[padIndex]));
	
	if (padMode == PAD_MODE_NOTE_PICKER) {
		
		if (padIndex == PAD_UP && ((notePickerState & NOTE_PICKER_STATE_MASK_BOTTOM)>> NOTE_PICKER_STATE_SHIFT_BOTTOM) < 7) { //note picker octave switching
			notePickerState = (notePickerState & ~NOTE_PICKER_STATE_MASK_BOTTOM) |
				((((notePickerState & NOTE_PICKER_STATE_MASK_BOTTOM)>> NOTE_PICKER_STATE_SHIFT_BOTTOM)+1)<<NOTE_PICKER_STATE_SHIFT_BOTTOM);
			//notePickerBottomOctave++;
			redrawNotePicker();
		} else if (padIndex == PAD_DOWN && ((notePickerState & NOTE_PICKER_STATE_MASK_BOTTOM)>> NOTE_PICKER_STATE_SHIFT_BOTTOM) > 0) { //midi octaves go from -2 (c-2=0) to 8 (g+8=127)
			notePickerState = (notePickerState & ~NOTE_PICKER_STATE_MASK_BOTTOM) |
				((((notePickerState & NOTE_PICKER_STATE_MASK_BOTTOM)>> NOTE_PICKER_STATE_SHIFT_BOTTOM)-1)<<NOTE_PICKER_STATE_SHIFT_BOTTOM);
			//notePickerBottomOctave--;
			redrawNotePicker();

		} else if (padIndex == NOTE_PICKER_PREVIEW_PAD) { //toggle note preview
			
			notePickerState = notePickerState ^ NOTE_PICKER_STATE_MASK_PREVIEW;
			redrawNotePicker();
			
		} else if (padIndex >= 13 && padIndex <= 16) { //select channel
			
			//TODO: note off before switching channel
			if (padIndex == 16) notePickerState = notePickerState ^ 1;
			if (padIndex == 15) notePickerState = notePickerState ^ 2;
			if (padIndex == 14) notePickerState = notePickerState ^ 4;
			if (padIndex == 13) notePickerState = notePickerState ^ 8;
			setNotePickerChannel(notePickerState & NOTE_PICKER_STATE_MASK_CHANNEL);

		} else if (stepPadIndex >= 0 && stepPadIndex < STEP_COUNT) {

			setNotePicker(stepPadIndex);

		} else if (padIndex == PAD_MIXER && getPad(PAD_SESSION) && activeSeqIndex >= 0 && seqState[activeSeqIndex] != SEQ_STATE_OFF) { 
			//delete seq
			//TODO: note off before deleting
			deleteSeq(activeSeqIndex);
			applyNotePickerToSeqIndex = -1;
			switchLedMode(LED_MODE_SEQ_RUN);
			padMode = PAD_MODE_STEP_TOGGLE;
			setLedGreen(PAD_SESSION, 0);
			setLedGreen(PAD_USER_1, 0);
			setLedRed(PAD_SESSION, 0);
			setLedRed(PAD_USER_1, 0);

		} else if (padIndex == PAD_USER_1) { //pressed again, turn off interactive note picker for running sequences
			//midiNoteOff(notePickerCurrentNote, (notePickerState & NOTE_PICKER_STATE_MASK_CHANNEL));
			if (activeSeqIndex >= 0 && seqState[activeSeqIndex] != SEQ_STATE_OFF) {
				setSeqBaseNoteAndChannel(activeSeqIndex, notePickerCurrentNote, (notePickerState & NOTE_PICKER_STATE_MASK_CHANNEL));
			}
			applyNotePickerToSeqIndex = -1;
			padMode = PAD_MODE_STEP_TOGGLE;
			switchLedMode(LED_MODE_SEQ_RUN);
			setLedGreen(PAD_SESSION, 0);
			setLedGreen(PAD_USER_1, 0);
			
		} else if (padIndex == PAD_USER_2) { //enable scale picker
			
			padMode = PAD_MODE_SCALE_PICKER;
			switchLedMode(LED_MODE_SCALE_PICKER);//calls redrawNotePicker();
			setLed(PAD_USER_2, LED_ORANGE);
			
		}
		
	} else if (padMode == PAD_MODE_SCALE_PICKER) {
		
		if (padIndex == PAD_USER_2) { //pressed again, disable scale picker
			
			padMode = PAD_MODE_NOTE_PICKER;
			switchLedMode(LED_MODE_NOTE_PICKER);
			setLed(PAD_USER_2, 0);
			
		} else if (stepPadIndex >= 0 && stepPadIndex < 16) { //measure selector
			
			setMeasureFromStepPadIndex(stepPadIndex); //sets notePickerCurrentMeasure
			
			if (applyNotePickerToSeqIndex >= 0) { //apply measure change
				//TODO: when changing measure, reset and/or rearm timing
				seqMeasure[applyNotePickerToSeqIndex] = notePickerCurrentMeasure;
			}
			
			redrawNotePicker();
			
		} else { //scale bits toggle
			
			int8_t scaleBitIndex = (int8_t) pgm_read_byte(&(stepToScaleBit[stepPadIndex]));
			
			if (scaleBitIndex >= 0) {
				if (scaleBitIndex < 16) {
					seqScaleLo[activeSeqIndex] ^= (1 << (scaleBitIndex));
				} else {
					seqScaleHi[activeSeqIndex] ^= (1 << (scaleBitIndex-16));
				}
				redrawNotePicker();
			}
			
		}

		
	} else if (padMode == PAD_MODE_SETTING_STEPS) { //one step pad pressed, waiting for second step pad
		
		if (stepPadIndex >= 0 && stepPadIndex < STEP_COUNT) { //step pads
			
			int8_t otherPressedStep = OTHER_STEP_NONE;
			
			if (
				activeSeqIndex >= 0 && 
				activePage == ((seqPageSlot[activeSeqIndex] & SEQ_PAGE_MASK)>> SEQ_PAGE_SHIFT) &&
				getSeqState(activeSeqIndex,SEQ_STATE_DEFINED_WITHOUT_STEPS)
			) {

				otherPressedStep = getOtherPressedStep(padIndex,-1);
				
				//ignore otherPressedPad == OTHER_STEP_WRONG, do nothing
				if (otherPressedStep == OTHER_STEP_NONE) { //no other pad pressed: this should not happen
					padMode = PAD_MODE_STEP_TOGGLE;
					
				} else if (otherPressedStep >= 0) { //second pad pressed (end point)
					
					if (otherPressedStep < stepPadIndex) {
						
						setActiveSeqStartEnd(otherPressedStep,stepPadIndex);

						padMode = PAD_MODE_STEP_TOGGLE;
					}
				}
			}
		}		
		

	} else if (padMode == PAD_MODE_STEP_TOGGLE) {

		if (
			activeSeqIndex >= 0 && seqState[activeSeqIndex] != SEQ_STATE_OFF &&
			(padIndex == PAD_UP || padIndex == PAD_DOWN) //active seq switching
		) {
			uint8_t originalActiveSeqIndex = activeSeqIndex;
			
			if (padIndex == PAD_UP) {
				do {
					if (activeSeqIndex <= 0) {
						activeSeqIndex = SEQ_COUNT - 1;
					} else {
						activeSeqIndex--;
					}
				} while (seqState[activeSeqIndex] == SEQ_STATE_OFF && activeSeqIndex >= 0 && originalActiveSeqIndex != activeSeqIndex);
			} else if (padIndex == PAD_DOWN) {
				do {
					if (activeSeqIndex >= SEQ_COUNT - 1) {
						activeSeqIndex = 0;
					} else {
						activeSeqIndex++;
					}
				} while (seqState[activeSeqIndex] == SEQ_STATE_OFF && activeSeqIndex >= 0 && originalActiveSeqIndex != activeSeqIndex);
			}
			setLedGreen(padIndex, LED_ON);
			if (activeSeqIndex < 0 || activeSeqIndex >= SEQ_COUNT || seqState[activeSeqIndex] == SEQ_STATE_OFF) {
				activeSeqIndex = originalActiveSeqIndex;
			}

			if (activeSeqIndex >= 0 && seqState[activeSeqIndex] != SEQ_STATE_OFF) {
				if (activePage != ((seqPageSlot[activeSeqIndex] & SEQ_PAGE_MASK)>> SEQ_PAGE_SHIFT)) {
					activePage = ((seqPageSlot[activeSeqIndex] & SEQ_PAGE_MASK)>> SEQ_PAGE_SHIFT);
				}
				
				if (originalActiveSeqIndex >= 0) setSeqState(originalActiveSeqIndex, SEQ_STATE_HIGHLIGHTED, false);				
				
				highlightActiveSeq(); //TODO: test this order, the highlighting was originally after redraw
				redrawSeqPlayer();
				
			}
			
		} else if (padIndex == PAD_LEFT) { //active page switching
			
			if (activePage <= 0) {
				activePage = PAGE_COUNT-1;
			} else {
				activePage--;
			}
			redrawSeqPlayer();
					
		} else if (padIndex == PAD_RIGHT) { //active page switching
			
			if (activePage >= PAGE_COUNT-1) {
				activePage = 0;
			} else {
				activePage++;
			}
			redrawSeqPlayer();
			

		} else if (stepPadIndex >= 0 && stepPadIndex < STEP_COUNT) { //step pads
			
			int8_t otherPressedStep = OTHER_STEP_NONE;
			
			if ( //redefine start/end points while holding the top left "UP" pad
				getPad(PAD_UP) && 
				activePage == ((seqPageSlot[activeSeqIndex] & SEQ_PAGE_MASK)>> SEQ_PAGE_SHIFT) &&
				activeSeqIndex >= 0
			) {
				otherPressedStep = getOtherPressedStep(padIndex,PAD_UP);
				
				if (otherPressedStep >= 0 && otherPressedStep < stepPadIndex) {
						
					setActiveSeqStartEnd(otherPressedStep,stepPadIndex);
					highlightActiveSeq();
					redrawSeqPlayer();
				}

			} else if ( //add new seq with defining start/end points while holding the "DOWN" pad
				getPad(PAD_DOWN) && 
				activePage == ((seqPageSlot[activeSeqIndex] & SEQ_PAGE_MASK)>> SEQ_PAGE_SHIFT)
			) {
				otherPressedStep = getOtherPressedStep(padIndex,PAD_DOWN);
				
				if (otherPressedStep >= 0 && otherPressedStep < stepPadIndex) {
					
					addSeq(otherPressedStep, stepPadIndex, notePickerCurrentMeasure, otherPressedStep/8);
					redrawSeqPlayer();
					
				}
				
				
			} else {

				if ( //active sequence is defined without steps, attempt to define start step
					activeSeqIndex >= 0 && 
					activePage == ((seqPageSlot[activeSeqIndex] & SEQ_PAGE_MASK)>> SEQ_PAGE_SHIFT) &&
					getSeqState(activeSeqIndex,SEQ_STATE_DEFINED_WITHOUT_STEPS) 
				) {
					otherPressedStep = getOtherPressedStep(padIndex,-1);

					//TODO: we want to prevent step toggling when we define start/end points

					if (otherPressedStep == OTHER_STEP_NONE) { //first pad pressed (start point)

						padMode = PAD_MODE_SETTING_STEPS; //TODO: turn off PAD_MODE_SETTING_STEPS in padOffHandler

					} else if (otherPressedStep >= 0) { //two pad pressed: this should not happen, second pad (end point) should be handled in PAD_MODE_SETTING_STEPS handler

						padMode = PAD_MODE_STEP_TOGGLE;
					} else { //more than one pad pressed or incorrect pads pressed

						padMode = PAD_MODE_STEP_TOGGLE;
					}


				}

				if (otherPressedStep == OTHER_STEP_NONE && padMode != PAD_MODE_SETTING_STEPS) {
					//toggle step
					if (EMPTY_STEP == (steps[activePage][stepPadIndex] & STEPS_NOTE_MASK)) { //toggle steps
						steps[activePage][stepPadIndex] &= ~STEPS_NOTE_MASK;
						steps[activePage][stepPadIndex] |= BASE_STEP | STEPS_OFF_ONE;
						if (ledMode == LED_MODE_SEQ_RUN) setLedRed(padIndex, LED_ON);
					} else {
						steps[activePage][stepPadIndex] &= ~STEPS_NOTE_MASK;
						//steps[activePage][stepPadIndex] |= EMPTY_STEP; //not needed if EMPTY_STEP = 0
						if (ledMode == LED_MODE_SEQ_RUN) {
							setLedRed(padIndex, 0); 
						}
					}
				}
			}
			
		} else if (padIndex >= 0 && padIndex <= 7) { //right arrows
			
		
			if (
				slotSeqs[activePage][padIndex] == SLOT_SEQS_NO_SEQ || //specific new seq init
				(slotSeqs[activePage][padIndex] >= 0 && seqState[slotSeqs[activePage][padIndex]] == SEQ_STATE_OFF) //this should never happen
			) {
				//in this case (for right arrows) slotIndex = padIndex
				addSeq(padIndex*8, padIndex*8+7, notePickerCurrentMeasure, padIndex);
				redrawSeqPlayer();
				
			} else if (slotSeqs[activePage][padIndex] >= 0) {
				uint8_t si = slotSeqs[activePage][padIndex];
				
				if (getSeqState(si, SEQ_STATE_PLAYING)) { //stop playing seq run 
					
					setSeqState(si, SEQ_STATE_ARMED, false);
					setSeqState(si, SEQ_STATE_PLAYING, false);
					seqPosition[si] = seqEnd[si];
					seqBeatClocksSinceLastStep[si] = seqMeasure[si];
					setLed(padIndex,ARROW_SEQ_PAUSED);
					
				} else { //restart not-playing seq run
					
					setSeqState(si, SEQ_STATE_ARMED, true);
					setLed(padIndex,ARROW_SEQ_ARMED);
					
				}
				redrawSeqPlayer();
				
			}
			
		} else if (padIndex == PAD_SESSION) {  //seq edit (note while pressed, then start,end points)
			
			if (activeSeqIndex >= 0 && seqState[activeSeqIndex] != SEQ_STATE_OFF) {
				//edit existing seq
				//TODO: highlight active edited seq?
				if (activePage != ((seqPageSlot[activeSeqIndex] & SEQ_PAGE_MASK)>> SEQ_PAGE_SHIFT)) {
					activePage = ((seqPageSlot[activeSeqIndex] & SEQ_PAGE_MASK)>> SEQ_PAGE_SHIFT); 
				}
				notePickerCurrentNote = seqBaseNote[activeSeqIndex];
				notePickerState &= ~NOTE_PICKER_STATE_MASK_CHANNEL;
				notePickerState |= (NOTE_PICKER_STATE_MASK_CHANNEL & seqChannel[activeSeqIndex]);
				notePickerCurrentMeasure = seqMeasure[activeSeqIndex];
				applyNotePickerToSeqIndex = activeSeqIndex;
			} else {
				//add new seq if none is active
				applyNotePickerToSeqIndex = -1;
			}
			padMode = PAD_MODE_NOTE_PICKER;
			switchLedMode(LED_MODE_NOTE_PICKER);
			setLedRed(PAD_SESSION, LED_ON);
			setLedGreen(PAD_SESSION, 0);
			
		} else if (padIndex == PAD_USER_1) { //pick note for existing active seq while playing

			padMode = PAD_MODE_SELECT_SEQ;
			setLed(PAD_USER_1, LED_AMBER);
			
			if (activeSeqIndex >= 0 && activePage == ((seqPageSlot[activeSeqIndex] & SEQ_PAGE_MASK)>> SEQ_PAGE_SHIFT)) {
				highlightActiveSeq();
				redrawSeqPlayer();
			}

		} else if (padIndex == PAD_MIXER) {
			/* * /
			if (
				tapCount == 0 ||
				(tapCount>0 && timeDiff(currentTime, tapTime[0]) > LATEST_FIRST_TAP)
				//TODO: use MAX_TAP_DISCREPANCY_RATIO
			) {
				for (uint8_t i=0; i<MAX_TAP_QUEUE; i++) tapTime[i]=0;
				tapTime[0] = currentTime;
				tapCount = 1;
				
			} else if (tapCount < MAX_TAP_QUEUE) {
				
				tapTime[tapCount] = currentTime;
				tapCount++;
				
				//if (tapCount > 1) {
					//unsigned long newMicrosPerStepRatio = (unsigned long)(timeDiff(currentTime,firstTapTime) / (tapCount * METRONOME_LENGTH * 48));
					//if (newMicrosPerStepRatio >= MIN_MICROS_PER_STEP_RATIO && newMicrosPerStepRatio <= MAX_MICROS_PER_STEP_RATIO) {
					//	microsPerStepRatio = newMicrosPerStepRatio; //TODO: nejak inak to spravit, lebo to blbne
					//}
				//}
			} else {
				
				for (uint8_t i=0; i<MAX_TAP_QUEUE-2; i++) tapTime[i]=tapTime[i+1];
				tapTime[MAX_TAP_QUEUE-1] = currentTime;
				
			}
			/* */
			
		} else if (padIndex == PAD_USER_2) { //seq scale offsets editor on
			
			setLed(PAD_USER_2, LED_ORANGE);
			switchLedMode(LED_MODE_SCALE_RUN);
			padMode = PAD_MODE_SCALE_RUN;
			
		}

	} else if (padMode == PAD_MODE_SELECT_SEQ) {
		
		if (padIndex >= 0 && padIndex <= 7) { //right arrows
			
			if (
				slotSeqs[activePage][padIndex] == SLOT_SEQS_NO_SEQ || //no seq defined for this slot yet
				(slotSeqs[activePage][padIndex] >= 0 && seqState[slotSeqs[activePage][padIndex]] == SEQ_STATE_OFF) //this should never happen
			) {
				if (activeSeqIndex >= 0) setSeqState(activeSeqIndex, SEQ_STATE_HIGHLIGHTED, false);
				addSeq(0,0, notePickerCurrentMeasure, padIndex); //0,0 = define seq without start and end steps, seq will wait for start/end steps to be specified
				redrawSeqPlayer();
				
			} else if (
				slotSeqs[activePage][padIndex] >= 0 &&
				seqState[slotSeqs[activePage][padIndex]] != SEQ_STATE_OFF
			) {
				if (activeSeqIndex >= 0) setSeqState(activeSeqIndex, SEQ_STATE_HIGHLIGHTED, false);
				activeSeqIndex = slotSeqs[activePage][padIndex];
				highlightActiveSeq();
				redrawSeqPlayer();
			}
		
		}
		
		
	} else if (padMode == PAD_MODE_ADD_SEQ || padMode == PAD_MODE_EDIT_SEQ) {
		
		if (stepPadIndex >= 0) {
			if (addSeqHandlingStepStart == -1) { //seq adding/editing, select seq start
				
				addSeqHandlingStepStart = stepPadIndex;
				
			} else if (stepPadIndex > addSeqHandlingStepStart) { //seq start held, pressed seq end

				if (activeSeqIndex >= 0 && seqState[activeSeqIndex] != SEQ_STATE_OFF) { //seq editing start/end
					
					seqStart[activeSeqIndex] = addSeqHandlingStepStart;
					seqEnd[activeSeqIndex] = stepPadIndex;
					
					if (seqPosition[activeSeqIndex] < addSeqHandlingStepStart) { //current position is not inside new start/end, change position
						seqPosition[activeSeqIndex] = addSeqHandlingStepStart;
						//TODO: reset timing and rearm?
					} else if (seqPosition[activeSeqIndex] > stepPadIndex) {
						seqPosition[activeSeqIndex] = addSeqHandlingStepStart;
					}
					redrawSeqPlayer(); //redraw new seq highlight positions
					
				} else { //seq adding, select seq end and create
					addSeq(addSeqHandlingStepStart, stepPadIndex, notePickerCurrentMeasure, addSeqHandlingStepStart/8);
				
					addSeqHandlingStepStart = -1;
					applyNotePickerToSeqIndex = -1;
					padMode = PAD_MODE_STEP_TOGGLE;
					redrawSeqPlayer();
					setLed(PAD_SESSION, 0);
					setLed(PAD_USER_1, 0);
				}
			}
			
		} else if (padIndex == PAD_SESSION) { //escape from seq adding
			
			applyNotePickerToSeqIndex = -1;
			padMode = PAD_MODE_STEP_TOGGLE;
			if (activeSeqIndex >= 0) {
				setSeqState(activeSeqIndex, SEQ_STATE_HIGHLIGHTED, false);
				redrawSeqPlayer();
			}
			setLed(PAD_SESSION, 0);
			setLed(PAD_USER_1, 0);
			
		} /*else if (padIndex >= 0 && padIndex <= 7) { //row arrows
			switch (padIndex) {
				case 0:  break;
				case 1:  break;
			}
		}*/
		
	} else if (padMode == PAD_MODE_SCALE_RUN) {
		
		if (stepPadIndex >= 56 && stepPadIndex < 64) { //horizontal scrollbar
			scalePlayerPosition = (stepPadIndex-56)*8;
			redrawScalePlayer();
			
		} else if (stepPadIndex >= 0 && stepPadIndex < 56) { //toggle notes
			//TODO: depending on vertical scroll position (scalePlayerOffset) set other parameters (bits in steps array)
			
			int8_t newStepPos = seqStart[activeSeqIndex] + scalePlayerPosition + (stepPadIndex % 8);
			if (newStepPos <= seqEnd[activeSeqIndex]) {
				
				int8_t newNoteValue = scalePlayerOffset + 7 - (int)(stepPadIndex / 8) -1; //pad row position, -1 for horizontal scrollbar
				int8_t scaleStepIndex = seqPosToScaleStep(newStepPos);
				if (newNoteValue == STEPS_ACC_VALUE) { //accent
					steps[activePage][newStepPos] ^= STEPS_ACC;
					redrawScalePlayer();
							
				} else if (newNoteValue == STEPS_OFF_ONE_VALUE) { //one step note off
					steps[activePage][newStepPos] ^= STEPS_OFF_ONE;
					redrawScalePlayer();
							
				} else if (newNoteValue == STEPS_OFF_NEXT_VALUE) { //next note note off
					steps[activePage][newStepPos] ^= STEPS_OFF_NEXT;
					redrawScalePlayer();
							
				} else if ((steps[activePage][newStepPos] & STEPS_NOTE_MASK) - BASE_STEP == newNoteValue) { //turn off existing step
					
					setStepLed(scaleStepIndex & SCALE_STEP_VALUE, 0);
					steps[activePage][newStepPos] &= ~STEPS_NOTE_MASK;
					//steps[activePage][newStepPos] |= EMPTY_STEP; //not needed if EMPTY_STEP = 0
					
				} else if (newNoteValue >= 0 && newNoteValue <= STEPS_NOTE_MASK) { //set new note value for step
					
					setStepLed(scaleStepIndex & SCALE_STEP_VALUE, 0);
					if (steps[activePage][newStepPos] == EMPTY_STEP) {
						steps[activePage][newStepPos] |= STEPS_OFF_ONE;
					}
					steps[activePage][newStepPos] &= ~STEPS_NOTE_MASK;
					steps[activePage][newStepPos] |= (STEPS_NOTE_MASK & (newNoteValue + BASE_STEP));
					
					
				}
				drawScaleStep(newStepPos,false);
				
			}
			
		} else if (padIndex == PAD_USER_2) { //pressed again, seq scale offsets editor off
			
			switchLedMode(LED_MODE_SEQ_RUN);
			padMode = PAD_MODE_STEP_TOGGLE;
			setLed(PAD_USER_2, 0);
		} else if (padIndex == PAD_LEFT) {
			
			if (scalePlayerPosition > 0) {
				scalePlayerPosition--;
			} else setLed(padIndex,LED_RED);
			redrawScalePlayer();
		} else if (padIndex == PAD_RIGHT) {

			if (scalePlayerPosition < MAX_SCALE_PLAYER_POSITION) {
				scalePlayerPosition++;
			} else setLed(padIndex,LED_RED);
			redrawScalePlayer();
		} else if (padIndex == PAD_UP) {

			if (scalePlayerOffset < MAX_SCALE_PLAYER_OFFSET) {
				scalePlayerOffset++;
			} else setLed(padIndex,LED_RED);
			redrawScalePlayer();
		} else if (padIndex == PAD_DOWN) {

			if (scalePlayerOffset > 0) {
				scalePlayerOffset--;
			} else setLed(padIndex,LED_RED);
			redrawScalePlayer();
		} else if (padIndex >= 3 && padIndex <= 7) { //right arrows = vertical scrollbar
			
			scalePlayerOffset = (7-padIndex)*7;
			redrawScalePlayer();
			
		}
		
	}
}
//---------------------------------------------------------------------------------------------------------------
// **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **

void padOffHandler(uint8_t padIndex) {
	int8_t stepPadIndex = (int8_t) pgm_read_byte(&(padToStep[padIndex]));
	//if (padIndex == PAD_MIXER) {}
	
	if (padMode == PAD_MODE_SELECT_SEQ) {

		if (padIndex == PAD_USER_1) {
			if (activeSeqIndex >= 0 && seqState[activeSeqIndex] != SEQ_STATE_OFF) {

				setSeqState(activeSeqIndex, SEQ_STATE_HIGHLIGHTED, false);

				notePickerCurrentNote = seqBaseNote[activeSeqIndex];
				notePickerState &= ~NOTE_PICKER_STATE_MASK_CHANNEL;
				notePickerState |= (NOTE_PICKER_STATE_MASK_CHANNEL & seqChannel[activeSeqIndex]);
				notePickerCurrentMeasure = seqMeasure[activeSeqIndex];
				applyNotePickerToSeqIndex = activeSeqIndex;
			}
			padMode = PAD_MODE_NOTE_PICKER;
			switchLedMode(LED_MODE_NOTE_PICKER);
			setLed(PAD_USER_1, LED_GREEN);
		}
		
	} else if (padMode == PAD_MODE_NOTE_PICKER) {
		if (padIndex == PAD_SESSION) { //note picking off, seq adding or editing on
			
			if (activeSeqIndex >= 0 && seqState[activeSeqIndex] != SEQ_STATE_OFF) { //seq editing
				
				applyNotePickerToSeqIndex = activeSeqIndex;
				padMode = PAD_MODE_EDIT_SEQ;
				highlightActiveSeq();
				
			} else { //seq adding
				
				applyNotePickerToSeqIndex = -1;
				padMode = PAD_MODE_ADD_SEQ;
			}

			switchLedMode(LED_MODE_SEQ_RUN);
			setLedRed(PAD_SESSION, 0);
			setLedGreen(PAD_SESSION, LED_ON);
			setLedGreen(PAD_USER_1, 0);

		} else if (stepPadIndex >= 0 && notePickerCurrentNote >= 0) { //note picker played note off when no seq is active
			
			if (
				applyNotePickerToSeqIndex < 0 &&
				(activeSeqIndex < 0 || seqState[activeSeqIndex] == SEQ_STATE_OFF) 
			) {
				midiNoteOff(notePickerCurrentNote, (notePickerState & NOTE_PICKER_STATE_MASK_CHANNEL));
			}
		}
	} else if (padMode == PAD_MODE_STEP_TOGGLE) { 

		if (padIndex == PAD_UP || padIndex == PAD_DOWN) { //active seq switching highlighting off
			setLedGreen(PAD_UP, 0);
			setLedGreen(PAD_DOWN, 0);
			if (activeSeqIndex >= 0) setSeqState(activeSeqIndex, SEQ_STATE_HIGHLIGHTED, false);
			redrawSeqPlayer();
		}
	} else if (padMode == PAD_MODE_ADD_SEQ || padMode == PAD_MODE_EDIT_SEQ) { //seq start point held no more
		
		if (addSeqHandlingStepStart >= 0) {
			addSeqHandlingStepStart = -1;
		}
				
		
	} else if (padMode == PAD_MODE_SCALE_RUN) {
	
		if (padIndex == PAD_LEFT) {
			
			if (scalePlayerPosition == 0) {
				setLed(PAD_LEFT,0);
			} else {
				setLed(PAD_LEFT,LED_GREEN);
			}
			
		} else if (padIndex == PAD_RIGHT) {

			if (scalePlayerPosition > 0) {
				setLed(PAD_LEFT,LED_GREEN);
			} else {
				setLed(PAD_LEFT,0);
			}
			
		} else if (padIndex == PAD_UP) {
			setLed(PAD_UP,0);
			if (scalePlayerOffset > 0) {
				setLed(PAD_DOWN,LED_GREEN);
			} else {
				setLed(PAD_DOWN,0);
			}

		} else if (padIndex == PAD_DOWN) {
			if (scalePlayerOffset == 0) {
				setLed(PAD_DOWN,0);
			} else {
				setLed(PAD_DOWN,LED_GREEN);
			}
		}
		
	} else if (padMode == PAD_MODE_SETTING_STEPS) { //exit from start/end steps setting mode

		if (stepPadIndex >= 0 && stepPadIndex < STEP_COUNT) { //step pads		
			padMode = PAD_MODE_STEP_TOGGLE;
		}
		
	}
}
//---------------------------------------------------------------------------------------------------------------
