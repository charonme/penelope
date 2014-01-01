#define DEBUG_MODE 0
/* TODO:
 * - bugs:
 *		- zelene sipky left a down v user2 mode zostanu zasvietene aj po vypnuti user2 modu
 *		- highlight seq zostava aj po vypnuti session modu
 *		- indikacia prave hrajucej pozicie v user2 mode
 *		- treba poslat note off ked sa zmaze sekvencia (alebo asi aj v inych pripadoch ked sa nieco radikalne zmeni)
 * 
 * 
 * - ovladanie (stop/run) vsetkych sekvencii na jednej stranke (momentalne mame 8 stranok, kazda max.8 sekv.)
 * 
 * - indikacia na ktorej som stranke
 * - indikacia vycerpaneho poctu sekvencii pri vytvarani novej nad limit
 * 
 * - viacero moznosti pri prepinani medzi modami, hlavne mam na mysli scale picker
 * - vymysliet lepsie scrollovanie v scale runneri (napr. mozno s animaciou alebo scrollbarom)
 *		- prave sipky zatial nemaju vyuzitie v scale runneri, mozno by sa dali pouzit na vyskovy scrollbar aj s rychlym prepinanim
 * - pouvazovat nad dvojoktavovymi stupnicami (teraz su 2 bajty s 12 bitmi, staci pridat 1 bajt a budu 3 bajty s 2x12 bitmi)
 * 
 * - BPM nastavovanie
 *		- tapovanim
 *		- plus a minus alebo ciselna volba
 * - start a stop vsetkych sekvencii (aj metronomu) pre MIDI casovanie/synchronizaciu
 *		- v slave mode start/stop signaly z externeho midi zdroja
 *		- zapnut metronom az ked pride externy povel start?
 *			- a ked pride stop, treba aj sekvencie vsetky resetnut na zaciatok alebo staci ked sa pauznu tym ze neide metronom?
 *		- alebo: ak nic nehra, pridavat sekvencie zastavene a cakat na externy povel start?
 * - cez MIDI vstup:
 *		- vyber noty pre sekvenciu (learn)
 *		- nastavovanie offsetov v note-offsetovej sekvencii
 *		- nacasovane zadavanie not (live rec) do nahravanej sekvencie
 * 
 * - zoznam (alebo priam "dokumentacia" features co uz je urobene a ako sa to ovlada
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

volatile int8_t activeSeqIndex = -1;
int8_t applyNotePickerToSeqIndex = -1;
uint8_t metronomePosition = 0;
unsigned long metronomeLastStepTime;
unsigned long lastMetronomeBeatTime;

volatile uint8_t playBufferStart = 0;
volatile uint8_t playBufferEnd = 0;
volatile uint8_t playBufferData[PLAY_BUFFER_SIZE];
volatile uint8_t playBufferCommand[PLAY_BUFFER_SIZE];

int8_t activePage = 0;
#define SLOT_SEQS_NO_SEQ -1
int8_t slotSeqs[PAGE_COUNT][8]; //TODO: ale nedovolit viac ako SEQ_COUNT sekvencii aj s upozornenim

uint8_t seqPosition[SEQ_COUNT];
int8_t seqLastUnprocessedStep[SEQ_COUNT];
#define SEQ_SCALE_MASK 4095 //B0000 1111 1111 1111
#define MAX_SCALE_SIZE 24
uint16_t seqScale[SEQ_COUNT]; //12 lower bits for scale config: 0000bAaG gFfeDdCc
uint8_t seqScaleHi[SEQ_COUNT]; //8 more bits for two-octave scales
uint8_t seqPage[SEQ_COUNT];
int8_t seqSlot[SEQ_COUNT]; //-1=none, 0..7 (row indicated by right arrows) should be unique for every page
uint8_t seqStart[SEQ_COUNT];
uint8_t seqEnd[SEQ_COUNT];
uint8_t seqBaseNote[SEQ_COUNT]; //0..127
uint8_t seqChannel[SEQ_COUNT];
#define SEQ_STATE_OFF 0 //seq not defined
#define SEQ_STATE_SET 1 //false = seq not defined
#define SEQ_STATE_ARMED 2 //true = waiting for metronome beat clock 0 to start playing
#define SEQ_STATE_PLAYING 4 //false = paused/stopped (not playing)
#define SEQ_STATE_HIGHLIGHTED 128

uint8_t seqState[SEQ_COUNT];
int8_t seqPlayingNote[SEQ_COUNT]; //-1=no note is playing
int8_t seqMeasure[SEQ_COUNT]; //number of beat clocks per step
int8_t seqBeatClocksSinceLastStep[SEQ_COUNT]; //always lower than seqMeasure

uint16_t hbpm = 16000; //16000 hbpm = 160bpm; 65535 hbpm = 655.35 bpm
#define HBPM_STEP_MEASURE_RATIO 750000000
unsigned long microsPerMetronomeStep;
unsigned long currentTime;
//const unsigned long MAX_TIME = 4294967295; //+1 je overflow do nuly
unsigned long lastTenthSecondTime;
/*
#define MAX_TAP_QUEUE 3
#define LATEST_FIRST_TAP 6000000
#define MAX_TAP_DISCREPANCY_RATIO 3
unsigned long tapTime[MAX_TAP_QUEUE];
uint8_t tapCount = 0;
*/
#define EMPTY_STEP -1
#define BASE_STEP 0
int8_t steps[PAGE_COUNT][STEP_COUNT]; //-1 = empty step (no note), 0 = seq base note, 1..127 = offset values for note calculation according to seq's scales and base notes

int8_t notePickerBottomOctave = -1; //-2..6 (najvyssia oktava je 8, vykresluju sa 3 od spodnej)
uint8_t notePickerCurrentNote = 40; //0..127
uint8_t notePickerCurrentChannel = 0;
int8_t notePickerCurrentMeasure = 6;

uint8_t scalePlayerPosition = 0; //leftmost time position (displaying 8 steps in columns from position to pos.+7)
uint8_t scalePlayerOffset = 0; //bottom note/scale offset value (displaying 8 values in rows from offset to off.+7)


#define SCALE_STEP_FLAG 192 //B11000000
#define SCALE_STEP_ABOVE 128 //B10000000
#define SCALE_STEP_BELOW 64 //B01000000
#define SCALE_STEP_VALUE 63 //B00111111
#define SCALE_STEP_EMPTY 192 //B11000000

boolean sharpTone[13] = {0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0};
int8_t tonePickerOffset[13] = {0, -7, 1, -6, 2, 3, -4, 4, -3, 5, -2, 6, 7};

uint8_t ledMode = LED_MODE_SEQ_RUN;
uint8_t padMode = PAD_MODE_STEP_TOGGLE;

boolean sendMidi = !DEBUG_MODE;
boolean midiBeatMaster = true; //false = slave, receive beat clock, don't send our own
int8_t midiWaitingForIncoming = 0; //MIDI_WAITING_FOR_SYSEX = -1
int8_t midiPassThruFilterOut = 0;

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
	seqPage[si] = 0;
	seqMeasure[si] = 6;
	seqLastUnprocessedStep[si] = -1;
	seqChannel[si] = 0;
	seqStart[si] = 0;
	seqEnd[si] = 0;
	seqBaseNote[si] = si + 40;
	seqPlayingNote[si] = -1;
	seqScale[si] = SEQ_SCALE_MASK;
	seqScaleHi[si] = 255;
	seqSlot[si] = -1;
}


void setup() {
	if (DEBUG_MODE) {
		Serial.begin(9600); //debug serial output
		Serial.println("debug mode");
	} else {
		Serial.begin(31250); //MIDI
	}
	nlpCoreInit();
	
	analogWrite(14,255); //test
	analogWrite(15,255); //test
	analogWrite(16,255); //test
	
	for (uint8_t pageIndex = 0; pageIndex < PAGE_COUNT; pageIndex++) {
		for (uint8_t stepIndex = 0; stepIndex < STEP_COUNT; stepIndex++) {
			steps[pageIndex][stepIndex] = EMPTY_STEP;
		}
		for (uint8_t slotIndex = 0; slotIndex < 8; slotIndex++) {
			slotSeqs[pageIndex][slotIndex] = SLOT_SEQS_NO_SEQ;
		}
	}
	currentTime = micros();
	metronomeLastStepTime = currentTime;
	lastMetronomeBeatTime = currentTime;
	microsPerMetronomeStep = HBPM_STEP_MEASURE_RATIO / (hbpm * METRONOME_LENGTH);

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

/* */
void loop() {
	currentTime = micros();
	
	if (isItTime(lastTenthSecondTime + 100000)) {
		lastTenthSecondTime = currentTime;
		tenthSecondHandler();
	}
	
	if (midiBeatMaster) processMidiMasterMetronome(); //processes sequences too
	
	nlpCoreScanBank();
	midiProcessBufferedOutput();
	midiProcessInput();
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
					midiPassThruFilterOut++;
				} else {
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

int8_t padMixerPressedTenthSeconds = 0;
void tenthSecondHandler() {
	/* */
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
}

void processMidiMasterMetronome() {
	if (isItTime(metronomeLastStepTime + microsPerMetronomeStep)) { //metronome
		metronomeLastStepTime += microsPerMetronomeStep;
		processMetronomeStep();
	}
}
void processMetronomeStep() {
		
	if (0 == metronomePosition) {
		lastMetronomeBeatTime = metronomeLastStepTime;
		
		for (uint8_t i=0; i<SEQ_COUNT; i++) {
			if (getSeqState(i, SEQ_STATE_ARMED)) {
				setSeqState(i, SEQ_STATE_ARMED, false);
				setSeqState(i, SEQ_STATE_PLAYING, true);
				if (seqPage[i] == activePage && seqSlot[i]>=0 && seqSlot[i]<=7) {
					if (i == activeSeqIndex) {
						setLed(seqSlot[i],ARROW_SEQ_ACTIVE_PLAYING);
					} else {
						setLed(seqSlot[i],ARROW_SEQ_PLAYING);
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
	processSeqencesOnBeatClock();
}

void setSeqStep(uint8_t seqIndex, uint8_t stepIndex, boolean isCurrentlyPlaying) {
	if (seqPage[seqIndex] == activePage) {
		if (isCurrentlyPlaying) { //playing current step
			
			if (EMPTY_STEP != steps[activePage][stepIndex]) { //toggled step
				if (seqIndex == activeSeqIndex) {
					if (getSeqState(seqIndex, SEQ_STATE_HIGHLIGHTED)) {
						//highlighted toggled
						setStep(stepIndex,LED_ORANGE);
					} else { //active toggled
						setStep(stepIndex,LED_ORANGE);
					}
				} else { //regular toggled step
					setStep(stepIndex,LED_AMBER);
				}
			} else { //empty step (no note)
				if (seqIndex == activeSeqIndex) {
					//active empty (both highlighted or not)
					setStep(stepIndex,LED_AMBER);
				} else { //regular empty
					setStep(stepIndex,LED_GREEN);
				}
			}
			
		} else { //other non-playing steps (turn off step not playing any more)
			
			if (EMPTY_STEP != steps[activePage][stepIndex]) { //toggled step
				if (seqIndex == activeSeqIndex) {
					if (getSeqState(seqIndex, SEQ_STATE_HIGHLIGHTED)) {
						//highlighted toggled
						setStep(stepIndex,LED_AMBER);
					} else { //active toggled
						setStep(stepIndex,LED_RED);
					}
				} else { //regular toggled step
					setStep(stepIndex,LED_RED);
				}
			} else { //empty step (no note)
				if (seqIndex == activeSeqIndex) {
					if (getSeqState(seqIndex, SEQ_STATE_HIGHLIGHTED)) {
						//highlighted empty
						setStep(stepIndex,LED_GREEN);
					} else { //active empty
						setStep(stepIndex,0);
					}
				} else { //regular empty
					setStep(stepIndex, 0);
				}
			}
		}
	}
}

void processSeqencesOnBeatClock() {
	for (uint8_t si = 0; si < SEQ_COUNT; si++) { //prepare sequences for stepping
		
		if (getSeqState(si, SEQ_STATE_PLAYING)) { //TODO: clear previous led even when seq just stopped playing
			seqBeatClocksSinceLastStep[si]++;
			if (seqBeatClocksSinceLastStep[si] >= seqMeasure[si]) {
				seqBeatClocksSinceLastStep[si] = 0;

				//previous step note off
				if (seqPlayingNote[si] >= 0) {
					midiNoteOff(seqPlayingNote[si], seqChannel[si]);
					seqPlayingNote[si] = -1;
				}

				//clear previous step LED
				if (ledMode == LED_MODE_SEQ_RUN && seqPage[si] == activePage && seqPosition[si]>=0 && seqPosition[si] < STEP_COUNT) { 
					setSeqStep(si,seqPosition[si],false);
				} else if (si == activeSeqIndex && ledMode == LED_MODE_SCALE_RUN) {
					drawScaleStep(seqPosition[si],false);
				}

			
				seqPosition[si]++;
				if (seqPosition[si] > seqEnd[si]) {
					seqPosition[si] = seqStart[si];
				}
				if (midiBeatMaster && midiBeatState == MIDI_BEAT_ARMED && seqPosition[si] == seqStart[si]) {
					midiBeatState = MIDI_BEAT_PLAYING;
					midiBeatStart();
				}
				seqLastUnprocessedStep[si] = seqPosition[si];
			}
		}
	}
	
	for (uint8_t si = 0; si < SEQ_COUNT; si++) { //process prepared unprocessed sequence steps
		if (seqState[si] != SEQ_STATE_OFF && seqLastUnprocessedStep[si] == seqPosition[si]) {
			seqLastUnprocessedStep[si] = -1;
			processStep(si);
		}
	}
}

int8_t getSeqNote(int8_t seqIndex) {
	/* */
	uint8_t scaleSize = 0;
	uint8_t scaleValues[MAX_SCALE_SIZE];
	for (int8_t i=0; i<MAX_SCALE_SIZE; i++) {
		if (
			(i<16 && 0 != (seqScale[seqIndex] & (1<<i))) ||
			(i>=16 && 0 != (seqScaleHi[seqIndex] & (1<<(i-16))))
		) {
			scaleValues[scaleSize] = i;
			scaleSize++;
		}
	}
	uint8_t stepValue = steps[seqPage[seqIndex]][seqPosition[seqIndex]];
	return seqBaseNote[seqIndex] + scaleValues[stepValue % scaleSize] + ((int)(stepValue/scaleSize))*12;
	/* */
	return 0;
}

void processStep(int8_t seqIndex) {
	if (ledMode == LED_MODE_SEQ_RUN) {
		
		setSeqStep(seqIndex,seqPosition[seqIndex],true);
		
	} else if (seqIndex == activeSeqIndex && ledMode == LED_MODE_SCALE_RUN) {
		
		drawScaleStep(seqPosition[seqIndex],true);
		
	}
	if (EMPTY_STEP != steps[seqPage[seqIndex]][seqPosition[seqIndex]]) {
		int8_t newSeqNote = getSeqNote(seqIndex);
		midiPlayNote(newSeqNote, seqChannel[seqIndex]);
		seqPlayingNote[seqIndex] = newSeqNote;
	}
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
	while (si < SEQ_COUNT && seqState[si] != SEQ_STATE_OFF) si++;
	
	if (slotIndex < 0) {
		slotIndex = 0;
		while (slotIndex < 8 && slotSeqs[activePage][slotIndex] >= 0) slotIndex++;
	}
	
	if (
		si < SEQ_COUNT && seqState[si] == SEQ_STATE_OFF &&
		slotIndex < 8 &&
		slotSeqs[activePage][slotIndex] == SLOT_SEQS_NO_SEQ
	) {
		seqPage[si] = activePage;
		seqEnd[si] = seqEndIndex;
		seqStart[si] = seqStartIndex;
		slotSeqs[activePage][slotIndex] = si;
		seqSlot[si] = slotIndex;
		
		setLed(slotIndex,ARROW_SEQ_ARMED); //seq in slot armed
		
		seqBaseNote[si] = notePickerCurrentNote;
		seqChannel[si] = notePickerCurrentChannel;

		if (measure <= 0) {
			seqMeasure[si] = 6;
		} else {
			seqMeasure[si] = measure;
		}
		setSeqState(si, SEQ_STATE_SET, true);
		setSeqState(si, SEQ_STATE_ARMED, true);
		setSeqState(si, SEQ_STATE_PLAYING, false);
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
	seqState[si] = SEQ_STATE_OFF;
	seqBeatClocksSinceLastStep[si] = 0;
	slotSeqs[seqPage[si]][seqSlot[si]] = SLOT_SEQS_NO_SEQ;
	seqSlot[si] = -1;

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
			
			
		} else if (activePage != seqPage[activeSeqIndex]) {
			activePage = seqPage[activeSeqIndex]; 
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
void setStep(int8_t stepIndex, int8_t ledValue) {
	if (stepIndex>=0 && stepIndex < STEP_COUNT) {
		setLed((int8_t) pgm_read_byte(&(stepToLed[stepIndex])), ledValue);
	}
}

void setNotePicker(uint8_t stepIndex) {
	int8_t toneOffset = (int8_t) pgm_read_byte(&(stepToToneOffset[stepIndex]));
	if (toneOffset >= 0) {
		int8_t newNote = 12 * (notePickerBottomOctave + 2) + toneOffset;
		if (newNote < 128) {
			if (applyNotePickerToSeqIndex >= 0) {
				
				seqBaseNote[applyNotePickerToSeqIndex] = newNote;
				seqChannel[applyNotePickerToSeqIndex] = notePickerCurrentChannel;
				
				//TODO: when changing measure, reset and/or rearm timing?
				seqMeasure[applyNotePickerToSeqIndex] = notePickerCurrentMeasure;
				
			} else if (activeSeqIndex < 0 || seqState[activeSeqIndex] == SEQ_STATE_OFF) {
				midiNoteOff(notePickerCurrentNote, notePickerCurrentChannel);
				midiPlayNote(newNote, notePickerCurrentChannel);
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
		setStepRed(stepIndex, 0);
		setStepGreen(stepIndex, 0);
	}
}

void highlightActiveSeq() {
	setSeqState(activeSeqIndex, SEQ_STATE_HIGHLIGHTED, true);
	for (uint8_t stepSeqMarkIndex = seqStart[activeSeqIndex]; stepSeqMarkIndex <= seqEnd[activeSeqIndex]; stepSeqMarkIndex++) {
		setSeqStep(activeSeqIndex,stepSeqMarkIndex,stepSeqMarkIndex == seqPosition[activeSeqIndex]);
	}
}


void redrawNotePicker() {
	clearStepLeds();
	if (notePickerBottomOctave<-2 || notePickerBottomOctave > 6) notePickerBottomOctave = -1; //pre istotu
	uint8_t octaveStartStep = 56;
	int8_t currentNoteOctave = notePickerCurrentNote / 12 - 2; //-2 .. 8
	uint8_t currentNoteTone = notePickerCurrentNote % 12; //0..11
	for (uint8_t octaveDrawingIndex = 0; octaveDrawingIndex < 3; octaveDrawingIndex++) {
		setStepRed(octaveStartStep - 7, LED_ON);
		setStepRed(octaveStartStep - 6, LED_ON);
		setStepRed(octaveStartStep - 4, LED_ON);
		setStepRed(octaveStartStep - 3, LED_ON);
		setStepRed(octaveStartStep - 2, LED_ON);
		if (notePickerBottomOctave + octaveDrawingIndex > 0) {
			for (uint8_t octaveMarker = 0; octaveMarker < notePickerBottomOctave + octaveDrawingIndex; octaveMarker++) {
				setStepGreen(octaveStartStep + octaveMarker, LED_ON);
			}
		} else if (notePickerBottomOctave + octaveDrawingIndex == -1) { //negative octaves
			setStepGreen(octaveStartStep, LED_ON);
			setStepRed(octaveStartStep, LED_ON);
		} else if (notePickerBottomOctave + octaveDrawingIndex == -2) {
			setStepGreen(octaveStartStep, LED_ON);
			setStepRed(octaveStartStep, LED_ON);
			setStepGreen(octaveStartStep + 1, LED_ON);
			setStepRed(octaveStartStep + 1, LED_ON);
		}
		
		//currently selected note
		if (
			currentNoteOctave == notePickerBottomOctave + octaveDrawingIndex &&
			(activeSeqIndex < 0 || padMode != PAD_MODE_SCALE_PICKER)
		) { 
			//if (sharpTone[currentNoteTone]) {
				setStep(octaveStartStep + tonePickerOffset[currentNoteTone], LED_ORANGE);
				//setStepGreen(octaveStartStep + tonePickerOffset[currentNoteTone], LED_DIM);
				//setStepRed(octaveStartStep + tonePickerOffset[currentNoteTone], LED_ON);
			//} else {
				//setStepGreen(octaveStartStep + tonePickerOffset[currentNoteTone], LED_DIM);
				//setStepRed(octaveStartStep + tonePickerOffset[currentNoteTone], LED_ON);
			//}
		}

		octaveStartStep -= 16;
	}
	
	//scale picker
	if (activeSeqIndex >=0 && padMode == PAD_MODE_SCALE_PICKER) {
		
		if (0 != (seqScale[activeSeqIndex] & (1 << 0))) setStep(56, LED_ORANGE);
		if (0 != (seqScale[activeSeqIndex] & (1 << 1))) setStep(49, LED_ORANGE);
		if (0 != (seqScale[activeSeqIndex] & (1 << 2))) setStep(57, LED_ORANGE);
		if (0 != (seqScale[activeSeqIndex] & (1 << 3))) setStep(50, LED_ORANGE);
		if (0 != (seqScale[activeSeqIndex] & (1 << 4))) setStep(58, LED_ORANGE);
		if (0 != (seqScale[activeSeqIndex] & (1 << 5))) setStep(59, LED_ORANGE);
		if (0 != (seqScale[activeSeqIndex] & (1 << 6))) setStep(52, LED_ORANGE);
		if (0 != (seqScale[activeSeqIndex] & (1 << 7))) setStep(60, LED_ORANGE);
		if (0 != (seqScale[activeSeqIndex] & (1 << 8))) setStep(53, LED_ORANGE);
		if (0 != (seqScale[activeSeqIndex] & (1 << 9))) setStep(61, LED_ORANGE);
		if (0 != (seqScale[activeSeqIndex] & (1 <<10))) setStep(54, LED_ORANGE);
		if (0 != (seqScale[activeSeqIndex] & (1 <<11))) setStep(62, LED_ORANGE);
		
	}
	
	//measure picker
	for (uint8_t measureIndicator = 0; measureIndicator < 16; measureIndicator++) {
		if (measureIndicator < notePickerCurrentMeasure) {
			setStep(measureIndicator, LED_ORANGE);
		} else {
			setStep(measureIndicator, LED_YELLOW);
		}
	}
	//channel picker
	setLed(13, ((1 & notePickerCurrentChannel) > 0)?LED_AMBER:0);
	setLed(14, ((2 & notePickerCurrentChannel) > 0)?LED_AMBER:0);
	setLed(15, ((4 & notePickerCurrentChannel) > 0)?LED_AMBER:0);
	setLed(16, ((8 & notePickerCurrentChannel) > 0)?LED_AMBER:0);
}

//void lightScaleOffsetStep(uint8_t seqIndex, uint8_t) {
//}
int8_t seqPosToScaleStep(int8_t seqPos) {
	if (steps[activePage][seqPos] == EMPTY_STEP) {
		//no note defined on step, indicate bottom row position for seq runner
		return SCALE_STEP_EMPTY + 56 + (seqPos - seqStart[activeSeqIndex] - scalePlayerPosition);
		
	} else if (steps[activePage][seqPos] < scalePlayerOffset) { //toggled step is under viewing window

		return SCALE_STEP_BELOW + 56 + (seqPos - seqStart[activeSeqIndex] - scalePlayerPosition);

	} else if (steps[activePage][seqPos] > scalePlayerOffset + 7) { //toggled step is above viewing window

		return SCALE_STEP_ABOVE + (seqPos - seqStart[activeSeqIndex] - scalePlayerPosition);

	} else {
		return (seqPos - seqStart[activeSeqIndex] - scalePlayerPosition)
		+ 8 * (7 - (steps[activePage][seqPos] - scalePlayerOffset));
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
					setStep(scaleStepIndex & SCALE_STEP_VALUE, LED_AMBER);
				} else {
					setStep(scaleStepIndex & SCALE_STEP_VALUE, LED_GREEN);
				}
				break;
			case SCALE_STEP_ABOVE:
				setStep(scaleStepIndex & SCALE_STEP_VALUE, LED_GREEN);
				break;
			case SCALE_STEP_EMPTY:
				if (seqPosition[activeSeqIndex] == seqStepIndex && isCurrentlyPlaying) {
					setStep(scaleStepIndex & SCALE_STEP_VALUE, LED_AMBER);
				} else {
					setStep(scaleStepIndex & SCALE_STEP_VALUE, 0);
				}
				break;
			default:
				if (seqPosition[activeSeqIndex] == seqStepIndex && isCurrentlyPlaying) {
					setStep(scaleStepIndex, LED_RED);
				} else {
					setStep(scaleStepIndex, LED_ORANGE);
				}
		}
	}

}

void redrawScalePlayer() {
	clearStepLeds();
	if (activeSeqIndex >= 0 && seqState[activeSeqIndex] != SEQ_STATE_OFF) {

		for (uint8_t winPos=0; winPos<8 && seqStart[activeSeqIndex]+winPos+scalePlayerPosition <= seqEnd[activeSeqIndex]; winPos++) {
			drawScaleStep(seqStart[activeSeqIndex]+winPos+scalePlayerPosition, true);
			//int8_t scaleStepIndex = seqPosToScaleStep(seqStart[activeSeqIndex]+winPos+scalePlayerPosition);
			/*
			if (scaleStepIndex == SCALE_STEP_BELOW) { 
				setStep(winPos+56, LED_GREEN);
			} else if (scaleStepIndex == SCALE_STEP_ABOVE) {
				setStep(winPos, LED_GREEN);
			} else if (scaleStepIndex >= SCALE_STEP_EMPTY_THRESHOLD) { 
				if (seqPosition[activeSeqIndex] == seqStart[activeSeqIndex]+winPos+scalePlayerPosition) {
					setStep(scaleStepIndex-SCALE_STEP_EMPTY_THRESHOLD, LED_AMBER);
				}
			} else {
				if (seqPosition[activeSeqIndex] == seqStart[activeSeqIndex]+winPos+scalePlayerPosition) {
					setStep(scaleStepIndex, LED_RED);
				} else {
					setStep(scaleStepIndex, LED_ORANGE);
				}
			}
			*/ 
		}
		
	}
}


void redrawSeqPlayer() {
	clearStepLeds();
	for (uint8_t stepIndex = 0; stepIndex < STEP_COUNT; stepIndex++) {
		if (EMPTY_STEP != steps[activePage][stepIndex]) {
			setStep(stepIndex, LED_RED);
		}
	}
	
	for (uint8_t si = 0; si < SEQ_COUNT; si++) {
		if (seqPage[si] == activePage && seqState[si] != SEQ_STATE_OFF && seqPosition[si] >= 0) {
			
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
	} else if (newLedMode == LED_MODE_SCALE_RUN) {
		redrawScalePlayer();
		ledMode = LED_MODE_SCALE_RUN;
	}
}

int8_t addSeqHandlingStepStart = -1;
//---------------------------------------------------------------------------------------------------------------

void padOnHandler(uint8_t padIndex) {
	
	int8_t stepPadIndex = (int8_t) pgm_read_byte(&(padToStep[padIndex]));
	
	if (padMode == PAD_MODE_NOTE_PICKER) {
		
		if (padIndex == PAD_UP && notePickerBottomOctave < 6) { //note picker octave switching
			notePickerBottomOctave++;
			redrawNotePicker();
		} else if (padIndex == PAD_DOWN && notePickerBottomOctave > -2) { //midi octaves go from -2 (c-2=0) to 8 (g+8=127)
			notePickerBottomOctave--;
			redrawNotePicker();

		} else if (padIndex >= 13 && padIndex <= 16) { //select channel
			if (padIndex == 13) notePickerCurrentChannel = notePickerCurrentChannel ^ 1;
			if (padIndex == 14) notePickerCurrentChannel = notePickerCurrentChannel ^ 2;
			if (padIndex == 15) notePickerCurrentChannel = notePickerCurrentChannel ^ 4;
			if (padIndex == 16) notePickerCurrentChannel = notePickerCurrentChannel ^ 8;
			setNotePickerChannel(notePickerCurrentChannel);

		} else if (stepPadIndex >= 0 && stepPadIndex < 16) { //measure selector
			
			notePickerCurrentMeasure = stepPadIndex+1;
			
			if (applyNotePickerToSeqIndex >= 0) { //apply measure change
				//TODO: when changing measure, reset and/or rearm timing?
				seqMeasure[applyNotePickerToSeqIndex] = notePickerCurrentMeasure;
			}
			
			redrawNotePicker();
			
		} else if (stepPadIndex >= 16 && stepPadIndex < STEP_COUNT) {

			setNotePicker(stepPadIndex);

		} else if (padIndex == PAD_MIXER && getPad(PAD_SESSION) && activeSeqIndex >= 0 && seqState[activeSeqIndex] != SEQ_STATE_OFF) { 
			//delete seq
			
			deleteSeq(activeSeqIndex);
			applyNotePickerToSeqIndex = -1;
			switchLedMode(LED_MODE_SEQ_RUN);
			padMode = PAD_MODE_STEP_TOGGLE;
			setLedGreen(PAD_SESSION, 0);
			setLedGreen(PAD_USER_1, 0);
			setLedRed(PAD_SESSION, 0);
			setLedRed(PAD_USER_1, 0);

		} else if (padIndex == PAD_USER_1) { //pressed again, turn off interactive note picker for running sequences
			//midiNoteOff(notePickerCurrentNote, notePickerCurrentChannel);
			if (activeSeqIndex >= 0 && seqState[activeSeqIndex] != SEQ_STATE_OFF) {
				seqBaseNote[activeSeqIndex] = notePickerCurrentNote;
				seqChannel[activeSeqIndex] = notePickerCurrentChannel;
			}
			applyNotePickerToSeqIndex = -1;
			padMode = PAD_MODE_STEP_TOGGLE;
			switchLedMode(LED_MODE_SEQ_RUN);
			setLedGreen(PAD_SESSION, 0);
			setLedGreen(PAD_USER_1, 0);
			
		} else if (padIndex == PAD_USER_2) { //enable scale picker
			
			padMode = PAD_MODE_SCALE_PICKER;
			setLed(PAD_USER_2, LED_ORANGE);
			redrawNotePicker();
			
		}
		
	} else if (padMode == PAD_MODE_SCALE_PICKER) {
		
		if (padIndex == PAD_USER_2) { //pressed again, disable scale picker
			
			padMode = PAD_MODE_NOTE_PICKER;
			setLed(PAD_USER_2, 0);
			redrawNotePicker();
			
		} else if (((int8_t) pgm_read_byte(&(stepToScaleBit[stepPadIndex]))) >= 0) {
			
			seqScale[activeSeqIndex] ^= (1 << ((int8_t) pgm_read_byte(&(stepToScaleBit[stepPadIndex]))));
			redrawNotePicker();
			
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
				if (activePage != seqPage[activeSeqIndex]) {
					activePage = seqPage[activeSeqIndex]; 
				}
				redrawSeqPlayer();
				
				highlightActiveSeq();
			}
			//Serial.println(activeSeqIndex);
			
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
			

		} else if (stepPadIndex >= 0 && stepPadIndex < STEP_COUNT) {
			
			if (EMPTY_STEP == steps[activePage][stepPadIndex]) { //toggle steps
				steps[activePage][stepPadIndex] = BASE_STEP;
				if (ledMode == LED_MODE_SEQ_RUN) setLedRed(padIndex, LED_ON);
			} else {
				steps[activePage][stepPadIndex] = EMPTY_STEP;
				if (ledMode == LED_MODE_SEQ_RUN) {
					setLedRed(padIndex, 0); 
				}
				
			}
			
		} else if (padIndex >= 0 && padIndex <= 7) { //right arrows
			
		
			if (
				slotSeqs[activePage][padIndex] == SLOT_SEQS_NO_SEQ || //specific new seq init
				(slotSeqs[activePage][padIndex] >= 0 && seqState[slotSeqs[activePage][padIndex]] == SEQ_STATE_OFF) //this should never happen
			) {
				
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
				//TODO: highlight active edited seq
				if (activePage != seqPage[activeSeqIndex]) {
					activePage = seqPage[activeSeqIndex]; 
				}
				notePickerCurrentNote = seqBaseNote[activeSeqIndex];
				notePickerCurrentChannel = seqChannel[activeSeqIndex];
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
			
			if (activeSeqIndex >= 0 && seqState[activeSeqIndex] != SEQ_STATE_OFF) {
				notePickerCurrentNote = seqBaseNote[activeSeqIndex];
				notePickerCurrentChannel = seqChannel[activeSeqIndex];
				notePickerCurrentMeasure = seqMeasure[activeSeqIndex];
				applyNotePickerToSeqIndex = activeSeqIndex;
			}
			switchLedMode(LED_MODE_NOTE_PICKER);
			padMode = PAD_MODE_NOTE_PICKER;
			setLedGreen(PAD_USER_1, LED_ON);
			
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
					
				} else { //seq adding, select seq end and create
					addSeq(addSeqHandlingStepStart, stepPadIndex, notePickerCurrentMeasure, -1); //-1 = autoassign slot
				
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
			setLed(PAD_SESSION, 0);
			setLed(PAD_USER_1, 0);
			
		} /*else if (padIndex >= 0 && padIndex <= 7) { //row arrows
			switch (padIndex) {
				case 0:  break;
				case 1:  break;
			}
		}*/
		
	} else if (padMode == PAD_MODE_SCALE_RUN) {
		
		if (stepPadIndex >= 0 && stepPadIndex <= 64) { //toggle notes
			
			int8_t newStepPos = seqStart[activeSeqIndex] + scalePlayerPosition + (stepPadIndex % 8);
			if (newStepPos <= seqEnd[activeSeqIndex]) {
				
				int8_t newNoteValue = scalePlayerOffset + 7 - (int)(stepPadIndex / 8);
				int8_t scaleStepIndex = seqPosToScaleStep(newStepPos);
				if (steps[activePage][newStepPos] == newNoteValue) {
					
					setStep(scaleStepIndex & SCALE_STEP_VALUE, 0);
					steps[activePage][newStepPos] = EMPTY_STEP;
					
				} else if (newNoteValue >= 0 && newNoteValue <= 127) {
					
					setStep(scaleStepIndex & SCALE_STEP_VALUE, 0);
					steps[activePage][newStepPos] = newNoteValue;
					
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

			if (scalePlayerPosition < 119) {
				scalePlayerPosition++;
			} else setLed(padIndex,LED_RED);
			redrawScalePlayer();
		} else if (padIndex == PAD_UP) {

			if (scalePlayerOffset < 119) {
				scalePlayerOffset++;
			} else setLed(padIndex,LED_RED);
			redrawScalePlayer();
		} else if (padIndex == PAD_DOWN) {

			if (scalePlayerOffset > 0) {
				scalePlayerOffset--;
			} else setLed(padIndex,LED_RED);
			redrawScalePlayer();
		}
		
	}
}
//---------------------------------------------------------------------------------------------------------------
// **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **  **

void padOffHandler(uint8_t padIndex) {
	int8_t stepPadIndex = (int8_t) pgm_read_byte(&(padToStep[padIndex]));
	if (padIndex == PAD_MIXER) padMixerPressedTenthSeconds = 0;
	
	if (padMode == PAD_MODE_NOTE_PICKER) {
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
				midiNoteOff(notePickerCurrentNote, notePickerCurrentChannel);
			}
		}
	} else if (padMode == PAD_MODE_STEP_TOGGLE) { //active seq switching highlighting off

		if (padIndex == PAD_UP || padIndex == PAD_DOWN) {
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
				setLed(padIndex,0);
			} else {
				setLed(padIndex,LED_GREEN);
			}
			
		} else if (padIndex == PAD_RIGHT) {

			setLed(padIndex,0);
			if (scalePlayerPosition > 0) {
				setLed(PAD_LEFT,LED_GREEN);
			}
			
		} else if (padIndex == PAD_UP) {

			setLed(padIndex,0);
			if (scalePlayerOffset > 0) {
				setLed(PAD_DOWN,LED_GREEN);
			}

		} else if (padIndex == PAD_DOWN) {

			if (scalePlayerOffset == 0) {
				setLed(padIndex,0);
			} else {
				setLed(padIndex,LED_GREEN);
			}
		}
		
	}
}
//---------------------------------------------------------------------------------------------------------------
