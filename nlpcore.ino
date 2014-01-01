#define PAD_RED_BIT_VALUE 224 //3bits B11100000 (shift 5) (was 128)
#define PAD_RED_SHIFT 5
#define PAD_GREEN_BIT_VALUE 28 //3bits B00011100 (shift 2)  (was 64)
#define PAD_GREEN_SHIFT 2
#define PAD_LED_BIT_VALUE 252 //B11111100

#define PAD_VALUE 3 //2bits B11 no shifting //63

uint8_t pads[80];
uint8_t lastBank = 3;
uint8_t nextBank = 0;
unsigned long lastBankScanTime = 0;
uint8_t multiplexCycle = 0;

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

	for (uint8_t padi = 0; padi < 80; padi++) {
		pads[padi] = 0;
	}
}

void nlpCoreScanBank() {
	if (currentTime > lastBankScanTime + 380) {
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
		if (0==nextBank) { multiplexCycle++; } //=multiplexCycle%8; } //s tymto modulom to nefungovalo
		for (uint8_t biti = 0; biti < 40; biti++) {
			uint8_t ledBitValue = 0;
			int8_t ledBitIndex = (int8_t) pgm_read_byte(&(bankBitsGreenLed[nextBank][biti]));
			if (ledBitIndex >= 0) {
				ledBitValue = getLedGreen(ledBitIndex);
			} else {
				ledBitIndex = (int8_t) pgm_read_byte(&(bankBitsRedLed[nextBank][biti]));
				if (ledBitIndex >= 0) {
					ledBitValue = getLedRed(ledBitIndex);
				}
			}
			if ((ledBitValue == LED_ON) || (ledBitValue!=0 && (multiplexCycle & ledBitValue)==ledBitValue)) { //111=LED_ON=full 11=slabo 1=silno
				setLedBit(true);
			} else {
				setLedBit(false);
			}
			if (biti < 24) {
				int8_t padIndex = (int8_t) pgm_read_byte(&(bankPadBits[lastBank][biti]));
				if (padIndex >= 0) {
					uint8_t newPadState = (0 != (PIND & 4));
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

void resetPad(uint8_t padIndex) {
	pads[padIndex] &= ~PAD_VALUE;
}

void setPad(uint8_t padIndex) {
	pads[padIndex] |= PAD_VALUE;
}

void setPadValue(uint8_t padIndex, uint8_t newPadValue) {
	resetPad(padIndex);
	pads[padIndex] |= newPadValue;
}

boolean getPad(uint8_t padIndex) {
	return (pads[padIndex] & PAD_VALUE) != 0; //no shifting
}


boolean getLedRed(uint8_t ledIndex) {
	return (pads[ledIndex] & PAD_RED_BIT_VALUE)>>PAD_RED_SHIFT;
}

void setLedRed(uint8_t ledIndex, uint8_t newLedBitValue) {
	if (newLedBitValue > 0) {
		pads[ledIndex] &= ~PAD_RED_BIT_VALUE;
		pads[ledIndex] |= (PAD_RED_BIT_VALUE & (newLedBitValue << PAD_RED_SHIFT));
	} else {
		pads[ledIndex] &= ~PAD_RED_BIT_VALUE;
	}
}

boolean getLedGreen(uint8_t ledIndex) {
	return (pads[ledIndex] & PAD_GREEN_BIT_VALUE)>>PAD_GREEN_SHIFT;
}

void setLedGreen(uint8_t ledIndex, uint8_t newLedBitValue) {
	if (newLedBitValue > 0) {
		pads[ledIndex] &= ~PAD_GREEN_BIT_VALUE;
		pads[ledIndex] |= (PAD_GREEN_BIT_VALUE & (newLedBitValue << PAD_GREEN_SHIFT));
	} else {
		pads[ledIndex] &= ~PAD_GREEN_BIT_VALUE;
	}
}

void setLed(uint8_t ledIndex, uint8_t newLedBitValue) {
	if (newLedBitValue > 0) {
		pads[ledIndex] &= ~PAD_LED_BIT_VALUE;
		pads[ledIndex] |= (PAD_LED_BIT_VALUE & newLedBitValue);
	} else {
		pads[ledIndex] &= ~PAD_LED_BIT_VALUE;
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

void setBank(uint8_t bankIndex) {
	PORTD |= (banks[bankIndex]); //set bank high
}
