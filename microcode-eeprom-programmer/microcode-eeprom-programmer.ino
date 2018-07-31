/**
 * This sketch programs the microcode EEPROMs for the 8-bit breadboard computer
 * See this video for more: https://youtu.be/JUVt_KYAp-I
 */
#define SHIFT_DATA 2
#define SHIFT_CLK 3
#define SHIFT_LATCH 4
#define EEPROM_D0 5
#define EEPROM_D7 12
#define WRITE_EN 13
#define LAST_ADDRESS_BIT 12
#define CHIP_OFFSET (1 << LAST_ADDRESS_BIT)

#define CE  0b1000000000000000  // Program counter enable
#define CO  0b0100000000000000  // Program counter out
#define IO  0b0010000000000000  // Instruction register out
#define II  0b0001000000000000  // Instruction register in
#define RO  0b0000100000000000  // RAM data out
#define RI  0b0000010000000000  // RAM data in
#define MI  0b0000001000000000  // Memory address register in
#define HLT 0b0000000100000000  // Halt clock

#define OI  0b0000000010000000  // Output register in
#define OP  0b0000000001000000  // ALU subtract
#define EO  0b0000000000100000  // ALU out
#define BO  0b0000000000010000  // B register out
#define BI  0b0000000000001000  // B register in
#define AO  0b0000000000000100  // A register out
#define AI  0b0000000000000010  // A register in
#define J   0b0000000000000001  // Jump (program counter in)
#define CI J

#define ADDR_HACK(a) (((a) & 0b111) | (((a) & 0b1111000) << 1))

uint16_t ACTIVE_LOWS = (CO|IO|II|RO|MI|EO|BO|BI|AO|AI|CI);

uint16_t data[] = {
  MI|CO,  RO|II|CE,  0,      0,      0,         0, 0,0,   // 0000 - NOP
  MI|CO,  RO|II|CE,  IO|MI,  RO|AI,  0,         0, 0,0,   // 0001 - LDA
  MI|CO,  RO|II|CE,  IO|MI,  RO|BI,  EO|AI,     0, 0,0,   // 0010 - ADD
  MI|CO,  RO|II|CE,  IO|MI,  RO|BI,  EO|AI|OP,  0, 0,0,   // 0011 - SUB
  MI|CO,  RO|II|CE,  IO|MI,  AO|RI,  0,         0, 0,0,   // 0100 - STA
  MI|CO,  RO|II|CE,  IO|AI,  0,      0,         0, 0,0,   // 0101 - LDI
  MI|CO,  RO|II|CE,  IO|J,   0,      0,         0, 0,0,   // 0110 - JMP
  MI|CO,  RO|II|CE,  0,      0,      0,         0, 0,0,   // 0111
  MI|CO,  RO|II|CE,  0,      0,      0,         0, 0,0,   // 1000
  MI|CO,  RO|II|CE,  0,      0,      0,         0, 0,0,   // 1001
  MI|CO,  RO|II|CE,  0,      0,      0,         0, 0,0,   // 1010
  MI|CO,  RO|II|CE,  0,      0,      0,         0, 0,0,   // 1011
  MI|CO,  RO|II|CE,  0,      0,      0,         0, 0,0,   // 1100
  MI|CO,  RO|II|CE,  0,      0,      0,         0, 0,0,   // 1101
  MI|CO,  RO|II|CE,  AO|OI,  0,      0,         0, 0,0,   // 1110 - OUT
  MI|CO,  RO|II|CE,  HLT,    0,      0,         0, 0,0,   // 1111 - HLT
};


/*
 * Output the address bits and outputEnable signal using shift registers.
 */
void setAddress(int address, bool outputEnable) {
  shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, (address >> 8) | (outputEnable ? 0x00 : 0x80));
  shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, address);

  digitalWrite(SHIFT_LATCH, LOW);
  digitalWrite(SHIFT_LATCH, HIGH);
  digitalWrite(SHIFT_LATCH, LOW);
}


/*
 * Read a byte from the EEPROM at the specified address.
 */
byte readEEPROM(int address) {
  for (int pin = EEPROM_D0; pin <= EEPROM_D7; pin += 1) {
    pinMode(pin, INPUT);
  }
  setAddress(address, /*outputEnable*/ true);

  byte data = 0;
  for (int pin = EEPROM_D7; pin >= EEPROM_D0; pin -= 1) {
    data = (data << 1) + digitalRead(pin);
  }
  return data;
}


/*
 * Write a byte to the EEPROM at the specified address.
 */
void writeEEPROM(int address, byte data) {
  setAddress(address, /*outputEnable*/ false);
  for (int pin = EEPROM_D0; pin <= EEPROM_D7; pin += 1) {
    pinMode(pin, OUTPUT);
  }

  for (int pin = EEPROM_D0; pin <= EEPROM_D7; pin += 1) {
    digitalWrite(pin, data & 1);
    data = data >> 1;
  }
  digitalWrite(WRITE_EN, LOW);
  delayMicroseconds(1);
  digitalWrite(WRITE_EN, HIGH);
  delay(10);
}


/*
 * Read the contents of the EEPROM and print them to the serial monitor.
 */
void printContents() {
  for (int base = 0; base <= 255; base += 16) {
    byte data[16];
    for (int offset = 0; offset <= 15; offset += 1) {
      data[offset] = readEEPROM(base + offset);
    }

    char buf[80];
    sprintf(buf, "%03x:  %02x %02x %02x %02x %02x %02x %02x %02x   %02x %02x %02x %02x %02x %02x %02x %02x",
            base, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
            data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);

    Serial.println(buf);
  }
}


void setup() {
  // put your setup code here, to run once:
  pinMode(SHIFT_DATA, OUTPUT);
  pinMode(SHIFT_CLK, OUTPUT);
  pinMode(SHIFT_LATCH, OUTPUT);
  digitalWrite(WRITE_EN, HIGH);
  pinMode(WRITE_EN, OUTPUT);
  Serial.begin(57600);

  // Program data bytes
  Serial.println("Programming EEPROM");

  for (int address = 0; address < sizeof(data)/sizeof(data[0]); address += 1) {
    // High order bits
    writeEEPROM(ADDR_HACK(address) + CHIP_OFFSET, (ACTIVE_LOWS ^ data[address]) >> 8);
    Serial.print(ADDR_HACK(address) + CHIP_OFFSET);
    Serial.print(":");
    Serial.print((ACTIVE_LOWS ^ data[address]) >> 8);
    Serial.print(" ");

    writeEEPROM(ADDR_HACK(address), (ACTIVE_LOWS ^ data[address]) & 0xFF);
    Serial.print(ADDR_HACK(address));
    Serial.print(":");
    Serial.println((ACTIVE_LOWS ^ data[address]) & 0xFF);

    if (address % 32 == 0) {
      Serial.print(".");
    }
  }

  // Program the 8 low-order bits of microcode into the second half of EEPROM
  /*
  for (uint16_t address = 0; address < sizeof(data)/sizeof(data[0]); address += 1) {
    //writeEEPROM(ADDR_HACK(address + (1 << LAST_ADDRESS_BIT)), (ACTIVE_LOWS & 0xFF) ^ (data[address] & 0xFF));
    if (address % 64 == 0) {
      Serial.print(".");
    }
    
  }
  */
  Serial.println(" done");


  // Read and print out the contents of the EERPROM
  Serial.println("Reading EEPROM");
  printContents();
}


void loop() {
  // put your main code here, to run repeatedly:

}

