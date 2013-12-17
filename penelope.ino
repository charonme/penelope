/* TODO:
 * - note off s nastavitelnou dlzkou
 * - indikacia na ktorej som stranke
 * - priradenie sekvencii k osmim sipkam napravo
 *		- 8 sekv. na stranku
 *		- vytvorenie novej konkretnej sekv stlacenim sipky namiesto session
 *		- selectovanie sekvencie sipkami
 * - editovanie start, end a casovania sekvencie
 * - vizualna indikacia casovania sekvencie pri vytvarani
 * - polovicny jas LED (hlavne pre oranzovu)
 * - moznost zapnut hrania noty v notepickeri (napr. ak neexistuje aktualna sekvencia - toto som zakomentoval) alebo ked aktualna sekvencia neobsahuje zapnute noty
 * 
 * - zlvast celostrankove sekvencie pre lahke hranie not 
 * 
 * - BPM nastavovanie (tapovanim mixer padu?)
 * - start a stop vsetkych sekvencii (aj metronomu) pre MIDI casovanie/synchronizaciu
 * - MIDI vstup pre synchronizaciu (nacitanie casovania z prichadzajuceho midi signalu)
 *		- MIDI merge zo vstupu do vystupu
 *		- zadavanie not cez MIDI vstup
 * 
 * 
 */

#include <Arduino.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "initconstants.h"

extern HardwareSerial Serial;

#define PAGE_COUNT 4 //4=1297+751, 5=1505+543, 6=1713+335
#define STEP_COUNT 64
#define SEQ_COUNT 8*PAGE_COUNT //8 per page
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

#define PAD_MODE_STEP_TOGGLE 0
#define PAD_MODE_ADD_SEQ 1
#define PAD_MODE_NOTE_PICKER 2

#define MIDI_BUFFERED_COMMAND_MASK 240 //B11110000
#define MIDI_BUFFERED_CHANNEL_MASK 31 //B00001111
#define MIDI_BUFFERED_BEAT_CLOCK 240	//B11110000
#define MIDI_BUFFERED_BEAT_START 224	//B11100000
#define MIDI_BUFFERED_NOTE_ON 144		//B10010000
#define MIDI_BUFFERED_NOTE_OFF 128		//B10000000
#define MIDI_NOTE_ON 144	//B10010000
#define MIDI_BEAT_CLOCK 248	//B11111000
#define MIDI_BEAT_START 250	//B11111010

#define METRONOME_LENGTH 48
#define METRONOME_HALF_LENGTH 24
#define METRONOME_STEP_TIME_RATIO 100

volatile char activeSeqIndex = -1;
char applyNotePickerToSeqIndex = -1;
unsigned char metronomePosition = 0; //0..23, ratio = 200, length = 24
unsigned long metronomeLastStepTime;
unsigned long lastMetronomeBeatTime;

unsigned char playBufferStart = 0;
unsigned char playBufferEnd = 0;
unsigned char playBufferData[PLAY_BUFFER_SIZE];
unsigned char playBufferCommand[PLAY_BUFFER_SIZE];

char activePage = 0;

unsigned char seqPosition[SEQ_COUNT];
//char seqPrevPosition[SEQ_COUNT]; //todo toto lepsie zdokumentovat
char seqLastUnprocessedStep[SEQ_COUNT];
unsigned char seqPage[SEQ_COUNT];
unsigned char seqStart[SEQ_COUNT];
unsigned char seqEnd[SEQ_COUNT];
unsigned char seqLength[SEQ_COUNT];
unsigned char seqNote[SEQ_COUNT];
unsigned char seqChannel[SEQ_COUNT];
unsigned long lastStepTime[SEQ_COUNT];
unsigned long seqStepRatio[SEQ_COUNT];

//todo
char seqNextNoteOffStepRatio[SEQ_COUNT]; //todo
//todo
//const long noteOffStepMultiplier = 40; // noteOffStepMultiplier/seqNoteOffStepRatio = 1 step 

//todo: vypocet z BPM
unsigned long microsPerStepRatio = 160; //microsPerStepRatio * seqStepRatio = one step
unsigned long currentTime;
//const unsigned long MAX_TIME = 4294967295; //+1 je overflow do nuly?

unsigned char steps[PAGE_COUNT][STEP_COUNT];

char notePickerBottomOctave = -1; //-2..5 (najvyssia oktava je 8, vykresluju sa 4 od spodnej, cize 5+3=8)
unsigned char notePickerCurrentNote = 40;
unsigned char notePickerCurrentChannel = 0;

boolean sharpTone[13] = {0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0};
char tonePickerOffset[13] = {0, -7, 1, -6, 2, 3, -4, 4, -3, 5, -2, 6, 7};

unsigned char ledMode = LED_MODE_SEQ_RUN;
unsigned char padMode = PAD_MODE_STEP_TOGGLE;


boolean sendMidi = true;
boolean sendMidiBeatClock = true;
boolean sendMidiBeatStart = true;

void setup() {
	//Serial.begin(9600); //31250); //9600);
	if (sendMidi) {
		Serial.begin(31250); //MIDI
	} else {
		Serial.begin(9600); //debug serial output
	}
	nlpCoreInit();
	for (unsigned char pageIndex = 0; pageIndex < PAGE_COUNT; pageIndex++) {
		for (unsigned char stepIndex = 0; stepIndex < STEP_COUNT; stepIndex++) {
			steps[pageIndex][stepIndex] = 0;
		}
	}
	currentTime = micros();
	metronomeLastStepTime = currentTime;
	lastMetronomeBeatTime = currentTime;

	for (unsigned char si = 0; si < SEQ_COUNT; si++) {
		seqPage[si] = 0;
		seqStepRatio[si] = 600;
		seqNextNoteOffStepRatio[si] = -1;
		seqLastUnprocessedStep[si] = -1;
		seqChannel[si] = 0;
		seqStart[si] = 0;
		seqEnd[si] = 0;
		seqLength[si] = 0;
		seqNote[si] = si + 40;
	}
	//addSeq(0,7,0);

}
//------------------------------------------------------------------------------

boolean isTimeToStep(long nextStepTime) {
	//TODO: overflow handling
	if (currentTime >= nextStepTime - 500) { //500 = latency compensation
		return true;
	}
	return false;
}

/* */
void loop() {
	currentTime = micros();
	processMetronome();
	processSeqences();
	nlpCoreScanBank();
	midiProcessBuffered();

}
void processMetronome() {
	if (isTimeToStep(metronomeLastStepTime + METRONOME_STEP_TIME_RATIO * microsPerStepRatio)) { //metronome
		metronomeLastStepTime += METRONOME_STEP_TIME_RATIO * microsPerStepRatio;
		if (0 == metronomePosition) {
			lastMetronomeBeatTime = metronomeLastStepTime;
			setLedGreen(PAD_MIXER, true);
			setLedRed(PAD_MIXER, true);
			if (sendMidiBeatStart) {
				midiBeatStart();
				sendMidiBeatStart = false;
			}
		} else if (METRONOME_HALF_LENGTH == metronomePosition) {
			setLedGreen(PAD_MIXER, true);
			setLedRed(PAD_MIXER, false);
		} else {
			setLedGreen(PAD_MIXER, false);
			setLedRed(PAD_MIXER, false);
		}
		metronomePosition = (metronomePosition + 1) % METRONOME_LENGTH;
		
		midiBeatClock();
	}
}

void processSeqences() {
	for (unsigned char si = 0; si < SEQ_COUNT; si++) { //prepare sequences for stepping
		if (seqLength[si] > 0 && isTimeToStep(lastStepTime[si] + seqStepRatio[si] * microsPerStepRatio)) {
			lastStepTime[si] += seqStepRatio[si] * microsPerStepRatio;

			//clear previous step LED
			if (ledMode == LED_MODE_SEQ_RUN && seqPage[si] == activePage && seqPosition[si]>=0 && seqPosition[si] < STEP_COUNT) { 
				if (si != activeSeqIndex || (!getPad(PAD_UP) && !getPad(PAD_DOWN))) {
					setStepGreen(seqPosition[si], false);
				}
				if (!steps[seqPage[si]][seqPosition[si]]) { //last active seq step was amber
					setStepRed(seqPosition[si], false);
				}
			} 

			//seqPrevPosition[si] = seqPosition[si];
			seqPosition[si]++;
			if (seqPosition[si] > seqEnd[si]) {
				seqPosition[si] = seqStart[si];
			}
			seqLastUnprocessedStep[si] = seqPosition[si];
		}
	}
	
	for (unsigned char si = 0; si < SEQ_COUNT; si++) { //process prepared unprocessed sequence steps
		if (seqLength[si] > 0 && seqLastUnprocessedStep[si] == seqPosition[si]) {
			seqLastUnprocessedStep[si] = -1;
			processStep(si);
		}
	}
}

void processStep(char seqIndex) {
	if (ledMode == LED_MODE_SEQ_RUN && activePage == seqPage[seqIndex]) {
		setStepGreen(seqPosition[seqIndex], true);
		if (seqIndex == activeSeqIndex) {
			setStepRed(seqPosition[seqIndex], true); //active seq is amber
		}
	}
	//seqPrevPosition[seqIndex] = -1;
	if (steps[seqPage[seqIndex]][seqPosition[seqIndex]]) {
		midiPlayNote(seqNote[seqIndex], seqChannel[seqIndex]);
		//TODO: schedule note off
	}
}

void midiProcessBuffered() {
	unsigned char bufferOffset = 0;
	while (playBufferStart != playBufferEnd && bufferOffset < 4) { //play max 4 notes at a time
		if (sendMidi) {
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
			}
		}
		bufferOffset++;
		playBufferStart = (playBufferStart + 1) % PLAY_BUFFER_SIZE;
	}
}

void midiBufferCommand(unsigned char command, unsigned char channel, unsigned char data) {
	if (sendMidi) {
		if ((playBufferEnd + 1) % PLAY_BUFFER_SIZE == playBufferStart) { //buffer full, try to empty
			midiProcessBuffered();
		}
		playBufferCommand[playBufferEnd] = command | channel;
		playBufferData[playBufferEnd] = data;
		playBufferEnd = (playBufferEnd + 1) % PLAY_BUFFER_SIZE;
	}
}

void midiBeatStart() {
	if (sendMidiBeatClock) {
		midiBufferCommand(MIDI_BUFFERED_BEAT_START, 0, 0);
	}
}
void midiBeatClock() {
	if (sendMidiBeatClock) {
		midiBufferCommand(MIDI_BUFFERED_BEAT_CLOCK, 0, 0);
	}
}

void midiPlayNote(char midiNote, unsigned char midiChannel) {
	if (sendMidi && midiNote < 128) {
		midiBufferCommand(MIDI_BUFFERED_NOTE_ON, midiChannel, midiNote);
	}

}

void midiNoteOff(char midiNote, unsigned char midiChannel) {
	if (sendMidi && midiNote < 128) {
		midiBufferCommand(MIDI_BUFFERED_NOTE_OFF, midiChannel, midiNote);
	}
}

void addSeq(char seqStartIndex, char seqEndIndex, char seqType) {
	char si = 0;
	while (si < SEQ_COUNT && seqLength[si] > 0) si++;
	if (si < SEQ_COUNT && seqLength[si] == 0) {
		seqPage[si] = activePage;
		seqEnd[si] = seqEndIndex;
		seqStart[si] = seqStartIndex;
		seqLength[si] = seqEnd[si] - seqStart[si] + 1;

		seqNote[si] = notePickerCurrentNote;
		seqChannel[si] = notePickerCurrentChannel;

		seqStepRatio[si] = 600;
		if (seqType == 0) {
			seqStepRatio[si] = 600;
		} else if (seqType == 1) {
			seqStepRatio[si] = 1200;
		} else if (seqType == 2) {
			seqStepRatio[si] = 2400;
		} else if (seqType == 3) {
			seqStepRatio[si] = 800;
		} else if (seqType == 4) {
			seqStepRatio[si] = 1600;
		} else if (seqType == 5) {
			seqStepRatio[si] = 3200;
		} else if (seqType == 6) {
			seqStepRatio[si] = 960;
		} else if (seqType == 7) {
			seqStepRatio[si] = 1920;
		}

		lastStepTime[si] = lastMetronomeBeatTime + 4800 * microsPerStepRatio; //24*200 (next metronome beat)
		//seqPrevPosition[si] = -1;
		seqPosition[si] = seqStart[si];
		activeSeqIndex = si; //Serial.println((int)activeSeqIndex);
	}
}

void deleteSeq(char si) {
	seqEnd[si] = 0;
	seqStart[si] = 0;
	seqLength[si] = 0;
	//seqPrevPosition[si] = seqPosition[si];

	if (si == activeSeqIndex) {
		do {
			if (activeSeqIndex==0) {
				activeSeqIndex = SEQ_COUNT - 1;
			} else {
				activeSeqIndex--;
			}
		} while (seqLength[activeSeqIndex] <= 0 && si != activeSeqIndex);
		if (si == activeSeqIndex) {
			activeSeqIndex = -1;
		} else if (activePage != seqPage[activeSeqIndex]) {
			activePage = seqPage[activeSeqIndex]; 
			redrawSeqPlayer();
		}
	}
}

void setStepRed(char stepIndex, boolean lightOn) {
	if (stepIndex>=0 && stepIndex < STEP_COUNT) {
		setLedRed((char) pgm_read_byte(&(stepToLed[stepIndex])), lightOn);
	}
}
void setStepGreen(char stepIndex, boolean lightOn) {
	if (stepIndex>=0 && stepIndex < STEP_COUNT) {
		setLedGreen((char) pgm_read_byte(&(stepToLed[stepIndex])), lightOn);
	}
}

void setNotePicker(unsigned char stepIndex) {
	char toneOffset = (char) pgm_read_byte(&(stepToToneOffset[stepIndex]));
	if (toneOffset >= 0) {
		char newNote = 12 * (notePickerBottomOctave + 2) + toneOffset;
		if (newNote < 128) {
			if (applyNotePickerToSeqIndex >= 0) {
				seqNote[applyNotePickerToSeqIndex] = newNote;
				seqChannel[applyNotePickerToSeqIndex] = notePickerCurrentChannel;
			} /*else {
				midiNoteOff(notePickerCurrentNote, notePickerCurrentChannel);
				midiPlayNote(newNote, notePickerCurrentChannel);
			}*/
			if (notePickerCurrentNote != newNote) {
				notePickerCurrentNote = newNote;
				redrawNotePicker();
			}
		}
	}
}

void setNotePickerChannel(unsigned char newChannel) {
	if (applyNotePickerToSeqIndex >= 0) {
		seqChannel[applyNotePickerToSeqIndex] = newChannel;
	}
	redrawNotePicker();
}

void clearStepLeds() {
	for (char stepIndex = 0; stepIndex < STEP_COUNT; stepIndex++) {
		setStepRed(stepIndex, false);
		setStepGreen(stepIndex, false);
	}
}

void redrawNotePicker() {
	clearStepLeds();
	if (notePickerBottomOctave<-2 || notePickerBottomOctave > 5) notePickerBottomOctave = -1; //pre istotu
	unsigned char octaveStartStep = 56;
	char currentNoteOctave = notePickerCurrentNote / 12 - 2; //-2 .. 8
	unsigned char currentNoteTone = notePickerCurrentNote % 12; //0..11
	for (unsigned char octaveDrawingIndex = 0; octaveDrawingIndex < 4; octaveDrawingIndex++) {
		setStepRed(octaveStartStep - 7, true);
		setStepRed(octaveStartStep - 6, true);
		setStepRed(octaveStartStep - 4, true);
		setStepRed(octaveStartStep - 3, true);
		setStepRed(octaveStartStep - 2, true);
		if (notePickerBottomOctave + octaveDrawingIndex > 0) {
			for (unsigned char octaveMarker = 0; octaveMarker < notePickerBottomOctave + octaveDrawingIndex; octaveMarker++) {
				setStepGreen(octaveStartStep + octaveMarker, true);
			}
		} else if (notePickerBottomOctave + octaveDrawingIndex == -1) { //negative octaves
			setStepGreen(octaveStartStep, true);
			setStepRed(octaveStartStep, true);
		} else if (notePickerBottomOctave + octaveDrawingIndex == -2) {
			setStepGreen(octaveStartStep, true);
			setStepRed(octaveStartStep, true);
			setStepGreen(octaveStartStep + 1, true);
			setStepRed(octaveStartStep + 1, true);
		}
		if (currentNoteOctave == notePickerBottomOctave + octaveDrawingIndex) {
			if (sharpTone[currentNoteTone]) {
				setStepGreen(octaveStartStep + tonePickerOffset[currentNoteTone], true);
			} else {
				setStepGreen(octaveStartStep + tonePickerOffset[currentNoteTone], false);
				setStepRed(octaveStartStep + tonePickerOffset[currentNoteTone], true);
			}
		}

		octaveStartStep -= 16;
	}
	setLedGreen(9, ((1 & notePickerCurrentChannel) > 0));
	setLedGreen(10, ((2 & notePickerCurrentChannel) > 0));
	setLedGreen(11, ((4 & notePickerCurrentChannel) > 0));
	setLedGreen(12, ((8 & notePickerCurrentChannel) > 0));
}

void redrawSeqPlayer() {
	clearStepLeds();
	for (unsigned char stepIndex = 0; stepIndex < STEP_COUNT; stepIndex++) {
		if (steps[activePage][stepIndex]) {
			setStepRed(stepIndex, true);
		}
	}
	for (unsigned char si = 0; si < SEQ_COUNT; si++) {
		if (seqPage[si] == activePage && seqLength[si] > 0 && seqPosition[si] >= 0) {
			setStepGreen(seqPosition[si], true);
			if (si == activeSeqIndex) {
				setStepRed(seqPosition[si], true); //active seq is amber
			}
		}
	}
}

void switchLedMode(unsigned char newLedMode) {
	if (newLedMode == LED_MODE_SEQ_RUN) {
		redrawSeqPlayer();
		ledMode = LED_MODE_SEQ_RUN;
	} else if (newLedMode == LED_MODE_NOTE_PICKER && ledMode != newLedMode) {
		redrawNotePicker();
		ledMode = LED_MODE_NOTE_PICKER;
	}
}

char addSeqHandlingStepStart = -1;
char addSeqHandlingSeqType = -1; //modified by row arrows when adding seq
//---------------------------------------------------------------------------------------------------------------

void padOnHandler(unsigned char padIndex) {
	
	char stepPadIndex = (char) pgm_read_byte(&(padToStep[padIndex]));
	
	if (padMode == PAD_MODE_NOTE_PICKER) {
		if (padIndex == PAD_UP && notePickerBottomOctave + 3 < 8) { //note picker octave switching
			notePickerBottomOctave++;
			redrawNotePicker();
		} else if (padIndex == PAD_DOWN && notePickerBottomOctave>-2) { //midi octaves go from -2 (c-2=0) to 8 (g+8=127)
			notePickerBottomOctave--;
			redrawNotePicker();

		} else if (padIndex >= 9 && padIndex <= 12) { //select channel
			if (padIndex == 9) notePickerCurrentChannel = notePickerCurrentChannel ^ 1;
			if (padIndex == 10) notePickerCurrentChannel = notePickerCurrentChannel ^ 2;
			if (padIndex == 11) notePickerCurrentChannel = notePickerCurrentChannel ^ 4;
			if (padIndex == 12) notePickerCurrentChannel = notePickerCurrentChannel ^ 8;
			setNotePickerChannel(notePickerCurrentChannel);

		} else if (stepPadIndex >= 0 && stepPadIndex < STEP_COUNT) { //handle channel first, if not channel, proceed to notes

			setNotePicker(stepPadIndex);

		} else if (padIndex == PAD_MIXER && getPad(PAD_SESSION) && activeSeqIndex >= 0 && seqLength[activeSeqIndex] > 0) { //delete seq
			deleteSeq(activeSeqIndex);
			applyNotePickerToSeqIndex = -1;
			switchLedMode(LED_MODE_SEQ_RUN);
			padMode = PAD_MODE_STEP_TOGGLE;
			setLedGreen(PAD_SESSION, false);
			setLedGreen(PAD_USER_1, false);
			setLedRed(PAD_SESSION, false);
			setLedRed(PAD_USER_1, false);

		} else if (padIndex == PAD_USER_1) { //interactive note picker for running sequences
			//midiNoteOff(notePickerCurrentNote, notePickerCurrentChannel);
			if (activeSeqIndex >= 0 && seqLength[activeSeqIndex] > 0) {
				seqNote[activeSeqIndex] = notePickerCurrentNote;
				seqChannel[activeSeqIndex] = notePickerCurrentChannel;
			}
			applyNotePickerToSeqIndex = -1;
			padMode = PAD_MODE_STEP_TOGGLE;
			switchLedMode(LED_MODE_SEQ_RUN);
			setLedGreen(PAD_SESSION, false);
			setLedGreen(PAD_USER_1, false);
		}

	} else if (padMode == PAD_MODE_STEP_TOGGLE) {

		if (
			activeSeqIndex >= 0 && seqLength[activeSeqIndex] > 0 &&
			(padIndex == PAD_UP || padIndex == PAD_DOWN) //active seq switching
		) {
			unsigned char originalActiveSeqIndex = activeSeqIndex;
			
			if (padIndex == PAD_UP) {
				do {
					if (activeSeqIndex <= 0) {
						activeSeqIndex = SEQ_COUNT - 1;
					} else {
						activeSeqIndex--;
					}
				} while (seqLength[activeSeqIndex] <= 0 && activeSeqIndex >= 0 && originalActiveSeqIndex != activeSeqIndex);
			} else if (padIndex == PAD_DOWN) {
				do {
					if (activeSeqIndex >= SEQ_COUNT - 1) {
						activeSeqIndex = 0;
					} else {
						activeSeqIndex++;
					}
				} while (seqLength[activeSeqIndex] <= 0 && activeSeqIndex >= 0 && originalActiveSeqIndex != activeSeqIndex);
			}
			setLedGreen(padIndex, true);
			if (activeSeqIndex < 0 || activeSeqIndex >= SEQ_COUNT || seqLength[activeSeqIndex] <= 0) {
				activeSeqIndex = originalActiveSeqIndex;
			}

			if (activeSeqIndex >= 0 && seqLength[activeSeqIndex] > 0) {
				if (activePage != seqPage[activeSeqIndex]) {
					activePage = seqPage[activeSeqIndex]; 
					redrawSeqPlayer();
				}
				
				for (unsigned char stepSeqMarkIndex = seqStart[activeSeqIndex]; stepSeqMarkIndex <= seqEnd[activeSeqIndex]; stepSeqMarkIndex++) {
					setStepGreen(stepSeqMarkIndex, true);
				}
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
			
			if (!steps[activePage][stepPadIndex]) { //toggle steps
				steps[activePage][stepPadIndex] = 1;
				if (ledMode == LED_MODE_SEQ_RUN) setLedRed(padIndex, true);
			} else {
				steps[activePage][stepPadIndex] = 0;
				if (ledMode == LED_MODE_SEQ_RUN) setLedRed(padIndex, false);
			}
			
		} else if (padIndex == PAD_SESSION) { //seq adding, pick note while pad is pressed
			
			applyNotePickerToSeqIndex = -1;
			padMode = PAD_MODE_NOTE_PICKER;
			switchLedMode(LED_MODE_NOTE_PICKER);
			setLedRed(PAD_SESSION, true);
			setLedGreen(PAD_SESSION, false);
			addSeqHandlingSeqType = -1;
			
		} else if (padIndex == PAD_USER_1) { //pick note for existing active seq while playing
			
			if (activeSeqIndex >= 0 && seqLength[activeSeqIndex] > 0) {
				notePickerCurrentNote = seqNote[activeSeqIndex];
				notePickerCurrentChannel = seqChannel[activeSeqIndex];
				applyNotePickerToSeqIndex = activeSeqIndex;
			}
			switchLedMode(LED_MODE_NOTE_PICKER);
			padMode = PAD_MODE_NOTE_PICKER;
			setLedGreen(PAD_USER_1, true);
			
		}

	} else if (padMode == PAD_MODE_ADD_SEQ) {
		if (stepPadIndex >= 0) {
			if (addSeqHandlingStepStart == -1) { //seq adding, select seq start
				addSeqHandlingStepStart = stepPadIndex;
			} else if (stepPadIndex > addSeqHandlingStepStart) { //seq adding, select seq end and create //TODO: len ak je este stale stlaceny addSeqHandlingStepStart
				addSeq(addSeqHandlingStepStart, stepPadIndex, addSeqHandlingSeqType);
				addSeqHandlingStepStart = -1;
				applyNotePickerToSeqIndex = -1;
				padMode = PAD_MODE_STEP_TOGGLE;
				setLedGreen(PAD_SESSION, false);
				setLedGreen(PAD_USER_1, false);
			}
		} else if (padIndex == PAD_SESSION) { //escape from seq adding
			applyNotePickerToSeqIndex = -1;
			padMode = PAD_MODE_STEP_TOGGLE;
			setLedGreen(PAD_SESSION, false);
			setLedGreen(PAD_USER_1, false);
		} else if (padIndex >= 0 && padIndex <= 7) { //row arrows - select time ratio for new seq
			addSeqHandlingSeqType = padIndex;
		}
	}
}
//---------------------------------------------------------------------------------------------------------------

void padOffHandler(unsigned char padIndex) {
	char stepPadIndex = (char) pgm_read_byte(&(padToStep[padIndex]));
	
	if (padMode == PAD_MODE_NOTE_PICKER) {
		if (padIndex == PAD_SESSION) { //note picking off, seq adding on
			applyNotePickerToSeqIndex = -1;
			padMode = PAD_MODE_ADD_SEQ;
			switchLedMode(LED_MODE_SEQ_RUN);
			setLedRed(PAD_SESSION, false);
			setLedGreen(PAD_SESSION, true);
			setLedGreen(PAD_USER_1, false);

		} else if (stepPadIndex >= 0 && notePickerCurrentNote >= 0) {
			//midiNoteOff(notePickerCurrentNote, notePickerCurrentChannel);

		}
	} else if (padMode == PAD_MODE_STEP_TOGGLE) {

		if (padIndex == PAD_UP || padIndex == PAD_DOWN) {
			setLedGreen(PAD_UP, false);
			setLedGreen(PAD_DOWN, false);
			redrawSeqPlayer();
		}
	} else if (padMode == PAD_MODE_ADD_SEQ) {
		if (addSeqHandlingStepStart >= 0) {
			addSeqHandlingStepStart = -1;
		}
				
		
	}
}
//---------------------------------------------------------------------------------------------------------------
