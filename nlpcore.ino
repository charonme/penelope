#define PAD_RED_VALUE 224 //3bits B11100000 (shift 5) (was 128)
#define PAD_GREEN_VALUE 28 //3bits B00011100 (shift 2)  (was 64)
#define PAD_VALUE 3 //2bits B11 no shifting //63
#define PAD_THRESHOLD 10
unsigned char pads[80];
unsigned char lastBank = 3;
unsigned char nextBank = 0;
unsigned long lastBankScanTime = 0;

//unsigned long scanBankCount = 0;
//unsigned long firstScanBankTime = 0;

void nlpCoreInit() {

	//D
	pinMode(2, INPUT); //sw data (value = 4)
	pinMode(3, OUTPUT); //bb clk (value = 8)
	pinMode(4, OUTPUT); //bank0 v=16
	pinMode(5, OUTPUT); //bank1 v=32
	pinMode(6, OUTPUT); //bank2 v=64
	pinMode(7, OUTPUT); //bank3 v=128
	//B
	pinMode(8, OUTPUT); //v=1 led data
	pinMode(9, OUTPUT); //sw sl (data load) v=2

	initLeds();

	for (unsigned char padi = 0; padi < 80; padi++) {
		pads[padi] = 0;
	}
}

void nlpCoreScanBank() {
	if (currentTime > lastBankScanTime + 400) {
		lastBankScanTime = currentTime;
		/* * /
		if (firstScanBankTime==0 && scanBankCount==0) firstScanBankTime = currentTime;
		if (currentTime - firstScanBankTime >= 10000000) {
			Serial.println(firstScanBankTime); //34660
			Serial.println(currentTime - firstScanBankTime); //10000344
			Serial.println(scanBankCount); //16944
		}
		scanBankCount++;
		/* */

		resetDataLoad();
		setClock();
		resetBank();
		delayMicroseconds(18);
		nextBank = (lastBank + 1) % 4;
		for (unsigned char biti = 0; biti < 40; biti++) {
			char ledGreenIndex = (char) pgm_read_byte(&(bankBitsGreenLed[nextBank][biti]));
			char ledRedIndex = (char) pgm_read_byte(&(bankBitsRedLed[nextBank][biti]));
			if (
					(ledGreenIndex >= 0 && getLedGreen(ledGreenIndex)) ||
					(ledRedIndex >= 0 && getLedRed(ledRedIndex))
					) {
				setLedBit(true);
			} else {
				setLedBit(false);
			}
			if (biti < 24) {
				char padIndex = (char) pgm_read_byte(&(bankPadBits[lastBank][biti]));
				if (padIndex >= 0) {
					unsigned char newPadState = (0 != (PIND & 4));
					if (newPadState && !getPad(padIndex)) {
						setPad(padIndex);
						padOnHandler(padIndex);
					} else if (!newPadState && getPad(padIndex)) {
						resetPad(padIndex);
						padOffHandler(padIndex);
					}
				}
			}
			clockTick();
		}
		resetClock();
		setBank(nextBank);
		lastBank = nextBank;
		setDataLoad();
	}
}

void resetPad(unsigned char padIndex) {
	pads[padIndex] &= ~PAD_VALUE;
}

void setPad(unsigned char padIndex) {
	pads[padIndex] |= PAD_VALUE;
}

void setPadValue(unsigned char padIndex, unsigned char newPadValue) {
	resetPad(padIndex);
	pads[padIndex] |= newPadValue;
}
/*
void incPad(unsigned char padIndex) {
	if (!getPad(padIndex)) {
		setPadValue(padIndex, 1 + getPadValue(padIndex));
	}
}

void decPad(unsigned char padIndex) {
	if (getPadValue(padIndex) > 0) {
		setPadValue(padIndex, getPadValue(padIndex) - 1);
	}
}

boolean getPad(unsigned char padIndex) {
	return (pads[padIndex] & PAD_VALUE) > PAD_THRESHOLD; //no shifting
}
unsigned char getPadValue(unsigned char padIndex) {
	return (pads[padIndex] & PAD_VALUE); //no shifting
}
*/
boolean getPad(unsigned char padIndex) {
	return (pads[padIndex] & PAD_VALUE) != 0; //no shifting
}


boolean getLedRed(unsigned char ledIndex) {
	return 0 != (pads[ledIndex] & PAD_RED_VALUE);
}

void setLedRed(unsigned char ledIndex, boolean ledOn) {
	if (ledOn) {
		pads[ledIndex] |= PAD_RED_VALUE;
	} else {
		pads[ledIndex] &= ~PAD_RED_VALUE;
	}
}

boolean getLedGreen(unsigned char ledIndex) {
	return 0 != (pads[ledIndex] & PAD_GREEN_VALUE);
}

void setLedGreen(unsigned char ledIndex, boolean ledOn) {
	if (ledOn) {
		pads[ledIndex] |= PAD_GREEN_VALUE;
	} else {
		pads[ledIndex] &= ~PAD_GREEN_VALUE;
	}
}

void initLeds() {
	resetDataLoad();
	setClock();
	resetBank();
	setLedBit(true);
	delayMicroseconds(8);
	setLedBit(false);
	delayMicroseconds(425);
	resetClock();
	delayMicroseconds(850);
	setClock();
	//delayMicroseconds(1);
	setDataLoad();
	setLedBit(true);
	delay(33);
	resetDataLoad();
	delayMicroseconds(13);
}

void setLedBit(boolean on) { //TTL
	if (on) { //true = 0
		PORTB &= ~1; //set LED data low
	} else { //false = 1
		PORTB |= 1; //set LED data high
	}
}

void resetDataLoad() { //TTL
	PORTB |= 2; //set data load high (not loading parallel data, reading serial data)
}

void setDataLoad() { //TTL
	PORTB &= ~2; //set data load low - for gathering (loading) parallel values (~560us) 
}

void resetClock() { //TTL
	PORTD |= 8; //set timer high
}

void setClock() { //TTL
	PORTD &= ~8; //set timer low
}

void clockTick() { //TTL
	PORTD &= ~8; //set timer low
	PORTD |= 8; //set timer high
}

void resetBank() {
	PORTD &= ~(240); //reset banks
}

void setBank(unsigned char bankIndex) {
	PORTD |= (banks[bankIndex]); //set bank high
}
