
#include <avr/io.h>
#include <avr/interrupt.h>


#define CPU_clock 16000000

#define SEND_START_CONDITION 100  // 01100100
#define SEND_START_CONDITION_AND_SET_TWINT 228  // 11100100
#define TWCR_INITIALISE 68  // 01000100
#define SET_TWINT 196 // 11000100
#define CLEAR_TWEA_FOR_NACK_AND_SET_TWINT 132 // 10000100
#define SEND_STOP_CONDITION 212
#define SLAVE_ADDRESS 80  // 1010000

// myRegister Pins
#define DRC 7    // Data read complete
#define STN 6    // Stop Now
#define GVN 5    // Gyro value negative
#define CWMPU 4  //  Communicating with gyro
#define LoadData 3

// SPI pins
#define TFT_CS   10  // Chip select
#define TFT_DC    9  // Data/Command
#define TFT_RST   8  // Reset


uint8_t IsrExitFlow;
uint8_t isrFunction;
uint8_t registerByte;
uint8_t myRegister;

uint8_t dataStreamStatus = 0;
uint16_t index = 0;
uint16_t page = 0;
uint8_t dataToSend[30];
uint16_t REG = 0;
uint8_t REG_H = 0;
uint8_t REG_L = 0;
bool setReadAddress = 0;
bool currentReadBool = 0;
bool byteWrite = 0;
bool readCycle = 0;
uint16_t ROW, COL = 0;
uint8_t byteBuffer = 0;
uint8_t idx = 0;
uint8_t incomingByte = 0;
uint8_t rowArray[160];

float counter = 0;
float time_1, time_2, time;

int pos = 1;
int pos_target = 70;



//  Initialise the TWI peripheral (I2C)
void twiInitialise(uint8_t bitRateGenerator) {

  // Activate internal pullups for twi
  digitalWrite(SDA, 1);
  digitalWrite(SCL, 1);

  TWCR = TWCR_INITIALISE;  // Setting control register bits

  TWBR = bitRateGenerator;  // Setting TWBR to 72 for a SCL frequency of 100kHz, if CPU f = 16MHz Set to 12 for 400kHz

  TWSR &= !(1 << TWPS1) & !(1 << TWPS0);  // Setting pre scaler bits to zero (Pre scaler = 1)

  //Serial.println("Initialised");
}


void writeMPU(uint8_t registerToWrite_H, uint8_t registerToWrite_L, uint8_t valueToWrite) {

  while (1) {

    // While TWINT is 0 wait in this loop
    while (!(TWCR & (1 << TWINT))) {
      ;
    }

    TWCR = TWCR_INITIALISE;  // Set TWINT to clear interrupt

    switch (TWSR) {

      case 8:
        //  Start condition has been transmitted
        //Serial.println("TWSR reads 8");
        TWCR = TWCR_INITIALISE;

        TWDR = (SLAVE_ADDRESS << 1);  // Load SLA + W

        break;

      case 16:
        // A repeated start condition has been transmitted
        TWCR = TWCR_INITIALISE;

        TWDR = (SLAVE_ADDRESS << 1);  // Load SLA + W

        break;

      case 24:
        // SLA+W has been transmitted; ACK has been received

        //Serial.println("SLA+W has been transmitted; ACK has been received, sending RA");

        TWDR = registerToWrite_H;  // Load register address
        dataStreamStatus = 0;

        break;

      case 32:
        // SLA+W has been transmitted; NOT ACK has been received
        //Retry operation
        IsrExitFlow = 1; // Send Repeated start

        break;

      case 40:
        // Address or data byte has been transmitted; ACK has been received


        switch (dataStreamStatus) {

          case 0:

            TWDR = registerToWrite_L;
            dataStreamStatus = 1;
            break;

          case 1:

            if (setReadAddress == 1) {

              dataStreamStatus = 0;
              IsrExitFlow = 3;

            } else if (byteWrite == 1) {

              TWDR = valueToWrite;
              dataStreamStatus = 2;

            } else {

              if (index < 64) {
                TWDR = dataToSend[index + (page * 64)];
                index++;
                dataStreamStatus = 2;
              } else {
                // Send stop condition
                IsrExitFlow = 3;
                dataStreamStatus = 0;
              }

            }
            break;

          case 2:

            if (byteWrite == 1) {

              // Send stop condition
              IsrExitFlow = 3;
              dataStreamStatus = 0;

            } else if (index < 64) {

              TWDR = dataToSend[index + (page * 64)];
              // Serial.print(dataToSend[index + (page * 64)]);
              // Serial.print(". ");
              index++;
              dataStreamStatus = 1;

            } else {

              // Send stop condition
              IsrExitFlow = 3;
              dataStreamStatus = 0;

            }
            break;

          default:
            break;
        }


        break;

      case 48:
        // Data byte has been transmitted; NOT ACK has been received

        break;

      case 56:
        // Arbitration lost in SLA+W or data bytes

        break;

      case 64:
        // SLA+R has been transmitted; ACK has been received, data byte will be received and ACK will be returned

        break;

      case 80:
        // Data byte has been received; ACK has been returned, data byte will be stored and NACK will be returned

        break;

      case 88:
        // Data byte has been received; NOT ACK has been returned, data byte will be store and STOP condition will be sent to end transmission

        break;

      default:
        break;
    }

    switch (IsrExitFlow) {

      case 0:
        TWCR = SET_TWINT;  // 0b11000101
        break;

      case 1:
        //Serial.println("Repeated start");
        TWCR = SEND_START_CONDITION_AND_SET_TWINT;
        break;

      case 2:
        //Serial.println("Return NACK");
        TWCR = CLEAR_TWEA_FOR_NACK_AND_SET_TWINT;

        registerByte = (TWDR);

        //Serial.print("High byte stored in registerByte: ");
        //Serial.println(registerByte);
        break;

      case 3:
        //Serial.println("STOP condition will be sent");
        IsrExitFlow = 0;
        
        TWCR = SEND_STOP_CONDITION;


        return;
        break;

      default:
        break;
    }
  }
}


uint8_t readMPU(uint8_t registerToRead_H, uint8_t registerToRead_L) {

  uint8_t readValue;

  // While communication with memory device bit is set
  while (1) {

    // While TWINT is 0 wait in this loop
    while (!(TWCR & (1 << TWINT))) {
      ;
    }

    IsrExitFlow = 0;

    TWCR = TWCR_INITIALISE;  // 01000100


    switch (TWSR) {

      case 8:
        //  Start condition has been transmitted
        TWCR = TWCR_INITIALISE;

        if (currentReadBool = 1) {
          TWDR = ((SLAVE_ADDRESS << 1) + 1);  // Load SLA + R
        } else {
          TWDR = (SLAVE_ADDRESS << 1);  // Load SLA + W
        }

        break;

      case 16:
        // A repeated start condition has been transmitted

        TWDR = ((SLAVE_ADDRESS << 1) + 1);  // Load SLA + R
        //Serial.println(TWDR);
        break;

      case 24:
        // SLA+W has been transmitted; ACK has been received
        TWDR = registerToRead_H;  // Load the high byte of the register address into TWDR
        dataStreamStatus = 0;
        break;

      case 32:
        // SLA+W has been transmitted; NOT ACK has been received
        //Serial.println("Random Access Read NACK");
        break;

      case 40:
        // Data byte has been transmitted; ACK has been received
        switch (dataStreamStatus) {

          case 0:
            TWDR = registerToRead_L;
            dataStreamStatus = 1;
            break;

          case 1:
            IsrExitFlow = 1;  // Repeated START
            break;

          default:
            break;
        }
        
        break;

      case 48:
        // Data byte has been transmitted; NOT ACK has been received
        break;

      case 56:
        // Arbitration lost in SLA+W or data bytes
        break;

      case 64:
        // SLA+R has been transmitted; ACK has been received, data byte will be received and we want to NACK
        
        IsrExitFlow = 2;

        break;

      case 80:
        // Data byte has been received; ACK has been returned, data byte will be stored and ACK will be returned to just keep reading

        //Serial.print("TWDR value at supposed data receival: ");
        // Serial.println(TWDR);
        readValue = TWDR;
        IsrExitFlow = 0;

        break;

      case 88:
        // Data byte has been received; NACK has been returned, data byte will be stored and STOP condition will be sent to end transmission

        readValue = TWDR;

        IsrExitFlow = 3;

        break;

      default:
        break;
    }


    switch (IsrExitFlow) {

      case 0:
        TWCR = SET_TWINT;  // 0b11000101
        break;

      case 1:
        //Serial.println("Repeated start");
        TWCR = SEND_START_CONDITION_AND_SET_TWINT;
        break;

      case 2:
        //Serial.println("Return NACK");
        TWCR = CLEAR_TWEA_FOR_NACK_AND_SET_TWINT;

        readValue = (TWDR);

        //Serial.print("High byte stored in registerByte: ");
        //Serial.println(registerByte);
        break;

      case 3:
        //Serial.println("STOP condition will be sent");
        TWCR = SEND_STOP_CONDITION;

        return readValue;

        break;

      default:
        break;
    }
  }

}


void sequentialRead(int number, uint8_t array[], uint8_t registerToRead_H, uint8_t registerToRead_L) { // alternatively uint8_t* array

  int sequentialCounter = 0;

  // While communication with memory device bit is set
  while (1) {

    // While TWINT is 0 wait in this loop
    while (!(TWCR & (1 << TWINT))) {
      ;
    }

    IsrExitFlow = 0;

    TWCR = TWCR_INITIALISE;  // 01000100


    switch (TWSR) {

      case 8:
        //  Start condition has been transmitted
        TWCR = TWCR_INITIALISE;

        TWDR = (SLAVE_ADDRESS << 1);  // Load SLA + W

        break;

      case 16:
        // A repeated start condition has been transmitted

        TWDR = ((SLAVE_ADDRESS << 1) + 1);  // Load SLA + R

        break;

      case 24:
        // SLA+W has been transmitted; ACK has been received
        TWDR = registerToRead_H;  // Load the high byte of the register address into TWDR
        dataStreamStatus = 0;
        break;

      case 32:
        // SLA+W has been transmitted; NOT ACK has been received
        //Serial.println("Random Access Read NACK");
        break;

      case 40:
        // Data byte has been transmitted; ACK has been received
        switch (dataStreamStatus) {

          case 0:
            TWDR = registerToRead_L;
            dataStreamStatus = 1;
            break;

          case 1:
            IsrExitFlow = 1;  // Repeated START
            break;

          default:
            break;
        }
        
        break;

      case 48:
        // Data byte has been transmitted; NOT ACK has been received
        break;

      case 56:
        // Arbitration lost in SLA+W or data bytes
        break;

      case 64:
        // SLA+R has been transmitted; ACK has been received, data byte will be received
        
        IsrExitFlow = 0;

        break;

      case 80:
        // Data byte has been received; ACK has been returned, data byte will be stored and ACK will be returned to just keep reading

        if (sequentialCounter < number) {
          array[sequentialCounter] = TWDR;
          IsrExitFlow = 0;
          sequentialCounter++;
        } else {
          IsrExitFlow = 2;  // NACK
        }
        
        break;

      case 88:
        // Data byte has been received; NACK has been returned, data byte will be stored and STOP condition will be sent to end transmission

        //readValue = TWDR;

        IsrExitFlow = 3;  // STOP

        break;

      default:
        break;
    }


    switch (IsrExitFlow) {

      case 0:
        TWCR = SET_TWINT;  // 0b11000101
        break;

      case 1:
        //Serial.println("Repeated start");
        TWCR = SEND_START_CONDITION_AND_SET_TWINT;
        break;

      case 2:
        //Serial.println("Return NACK");
        TWCR = CLEAR_TWEA_FOR_NACK_AND_SET_TWINT;

        //Serial.print("High byte stored in registerByte: ");
        //Serial.println(registerByte);
        break;

      case 3:
        //Serial.println("STOP condition will be sent");


        TWCR = SEND_STOP_CONDITION;

        return;

        break;

      default:
        break;
    }
  }
}

void spi_init() {
  // Set MOSI (PB2) and SCK (PB1) as output
  DDRB |= (1 << PB2) | (1 << PB1);

  SPSR = (1 << SPI2X); 

  // Enable SPI, Master mode, set clock rate fosc/4
  SPCR = (1 << SPE) | (1 << MSTR);


}

uint8_t spi_transfer(uint8_t data) {
  SPDR = data;                      // Start transmission
  while (!(SPSR & (1 << SPIF)));    // Wait for transmission complete
  return SPDR;                      // Return received data
}


void tft_write_command(uint8_t cmd) {
  digitalWrite(TFT_DC, LOW);     // Command mode
  digitalWrite(TFT_CS, LOW);
  spi_transfer(cmd);
  digitalWrite(TFT_CS, HIGH);
}

void tft_write_data(uint8_t data) {
  digitalWrite(TFT_DC, HIGH);    // Data mode
  digitalWrite(TFT_CS, LOW);
  spi_transfer(data);
  digitalWrite(TFT_CS, HIGH);
}


void tft_reset() {
  digitalWrite(TFT_RST, LOW);
  delay(50);
  digitalWrite(TFT_RST, HIGH);
  delay(50);
}

void tft_init() {
  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_DC, OUTPUT);
  pinMode(TFT_RST, OUTPUT);

  spi_init();

  tft_reset();

  // ST7735S Initialization sequence (minimal)
  tft_write_command(0x01); // Software reset
  delay(150);

  tft_write_command(0x11); // Sleep out
  delay(500);

  tft_write_command(0x3A); // Interface pixel format
  tft_write_data(0x03);    // 12-bit/pixel

  tft_write_command(0x36);   // MADCTL
  tft_write_data(0xA0);      // 0b10100000 = MY | MV | BGR

  tft_write_command(0x38);  // Idle mode OFF


  tft_write_command(0x29); // Display ON
  delay(100);
}

void tft_set_addr_window(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
  tft_write_command(0x2A); // Column addr set
  tft_write_data(0x00);
  tft_write_data(x0);
  tft_write_data(0x00);
  tft_write_data(x1);

  tft_write_command(0x2B); // Row addr set
  tft_write_data(0x00);
  tft_write_data(y0);
  tft_write_data(0x00);
  tft_write_data(y1);

  tft_write_command(0x2C); // Memory write
}

void tft_draw_next_8_pixels(uint8_t byte) {

  digitalWrite(TFT_DC, HIGH); // Data mode
  digitalWrite(TFT_CS, LOW);

  for (int k = 7; k >= 0; k--) {
    if ((byte >> k) & 1) {
      spi_transfer(0xF0);
      spi_transfer(0x0F);
    } else {
      spi_transfer(0);
      spi_transfer(0);      
    }
  }

  digitalWrite(TFT_CS, HIGH);
  
}


void setCurrentReadAddress(uint16_t registerAddress) {
  setReadAddress = 1;
  TWCR = SEND_START_CONDITION;
  writeMPU(registerAddress >> 8, registerAddress, 0);
  setReadAddress = 0;
}

uint8_t currentRead() {
  currentReadBool = 1;
  TWCR = SEND_START_CONDITION;
  uint8_t byte = readMPU(0,0);
  currentReadBool = 0;
  return byte;
}

void registerWrite(uint8_t registerToWrite_H, uint8_t registerToWrite_L, uint8_t valueToWrite) {

  byteWrite = 1;
  TWCR = SEND_START_CONDITION;
  writeMPU(registerToWrite_H, registerToWrite_L, valueToWrite);
  byteWrite = 0;

}


void drawRow(uint8_t* rowArray) {

  digitalWrite(TFT_DC, HIGH); // Data mode
  digitalWrite(TFT_CS, LOW);

  for (int j = 0; j < 160; j++) {
    for (int k = 7; k >= 0; k--) {
      if ((rowArray[j] >> k) & 1) {
        spi_transfer(0xFF);
        spi_transfer(0xF0);
      } else {
        spi_transfer(0);
        spi_transfer(0);      
      }
    }
  }

  digitalWrite(TFT_CS, HIGH);

}

void drawImageDataDoubleSize(uint8_t* imageData) {

  digitalWrite(TFT_DC, HIGH); // Data mode
  digitalWrite(TFT_CS, LOW);

  bool repeat = 1;

  for (int j = 0; j < 160; j++) {
    for (int k = 7; k >= 0; k--) {
      if ((rowArray[j] >> k) & 1) {
        spi_transfer(0xFF);
        spi_transfer(0xF0);
        spi_transfer(0xFF);
        // spi_transfer(0xF0);
      } else {
        spi_transfer(0);
        spi_transfer(0);
        spi_transfer(0);
        // spi_transfer(0);
      }
    }

    if ((j + 1) % 10 == 0) {
      if (repeat == 1) {
        j -= 10;
        repeat = 0;
      } else {
        repeat = 1;
      }
    }

  }

  digitalWrite(TFT_CS, HIGH);

}


void setup() {
  // put your setup code here, to run once:

  sei();

  Serial.begin(9600);
  while(!Serial);
  Serial.println("Serial begun");

  registerByte = 0;
  //stop_now = 0;
  //dataReadComplete = 1;
  //GVN = 0;
  myRegister = 128;
  REG = 16640;

  twiInitialise(12);  // 12 = 400kHz
  tft_init();

  setCurrentReadAddress(0); // CANNOT READ FROM CHIP BEFORE SETTING A READ ADDRESS FIRST (can also be done by writing data)

}


// small_face stored at 16640 to 16799

void loop() {

  TWCR = SEND_START_CONDITION;
  sequentialRead(160, rowArray, (REG >> 8), REG); // Read whole image
  

  tft_set_addr_window(0 + pos, 10, 79 + pos, 73);
  drawImageDataDoubleSize(rowArray);  // Draw whole face  // ~31Hz


  pos += (pos_target - pos) * 0.1;

  if (pos > 60) {
    pos_target = 0;

  } else if (pos < 1) {
    pos_target = 70;

  }


  // REG++;
  // REG_L = REG;
  // REG_H = REG >> 8;


  // if ((REG % 20 == 0) && (REG != 0)) { // Bitwise AND: &, Logical AND: &&. Modulo operator calculates remainder
  //   ROW++;
  //   COL = 0;
  // } else {
  //   COL += 0;
  // }





  // if (Serial.available() > 0) {

  //   incomingByte = Serial.read();

  //   int letterRegisterAddress = (incomingByte - 65) * 640;

  //   ROW = 0;

  //   if ((incomingByte > 90) || (incomingByte < 65)) {
  //     Serial.println("Please input a letter of the alphabet in upper case");
  //   } else {

  //     while (ROW < 128) {

  //       // time_1 = micros();
        
  //       TWCR = SEND_START_CONDITION;
  //       sequentialRead(160, rowArray, (((ROW * 5) + letterRegisterAddress) >> 8), ((ROW * 5) + letterRegisterAddress)); // Length of time for sequentialRead of 160 registers: 15.47 mS at 100kHz I2C. 400kHz: 4.7mS
        
  //       // time_2 = micros();
  //       // time = time_2 - time_1;
  //       // Serial.print("Read: "); // 4728 * 4 = 18,912uS per frame (13%)
  //       // Serial.println(time);




  //       // time_1 = micros();

  //       tft_set_addr_window(COL, ROW, COL+159, ROW+31);

  //       // time_2 = micros();
  //       // time = time_2 - time_1;
  //       // Serial.print("Set addr: "); // 224uS * 4 = 896uS per frame (0.6%)
  //       // Serial.println(time);




  //       // time_1 = micros();

  //       drawImageDataDoubleSize(rowArray);  // Time to set window addr and draw 1280 pixels: 8.75 mS

  //       // time_2 = micros();
  //       // time = time_2 - time_1;
  //       // Serial.print("Draw: "); // Draw time: 32432uS * 4 = 129,728uS per frame (86%)
  //       // Serial.println(time);
        
  //       ROW += 32;



  //     }

  //   }
  // }


  // if (Serial.available() > 0) {

  //   incomingByte = Serial.read();

    
  //   if (incomingByte == 70) { // ASCII: F

  //     readCycle = 1;
  //     REG = 0;
  //     setCurrentReadAddress(0);
  //     incomingByte = 0;

  //     delay(10);

  //   }

  //   if (incomingByte == 82) { // ASCII: R

  //     readCycle = 0;
  //     REG = 0;
  //     setCurrentReadAddress(0);
  //     Serial.println("Returned");
  //     incomingByte = 0;

  //   }


  //   if (readCycle == 0) {
      
  //     if (incomingByte == 49) { // ASCII: 1
  //       byteBuffer = byteBuffer | (1 << (7 - idx));  // 0000001
  //       idx++;
  //       incomingByte = 0;
  //     } else if (incomingByte == 48) {  // ASCII: 0
  //       byteBuffer = byteBuffer & ~(1 << (7 - idx));  // Bitwise NOT: ~     Logical NOT: !
  //       idx++;
  //       incomingByte = 0;
  //     }

  //     if (idx == 8) {
  //       idx = 0;
  //       delay(10);
  //       registerWrite(REG_H, REG_L, byteBuffer);
  //       Serial.print(" Written to register ");
  //       Serial.println(REG);
  //       REG++;
  //       REG_H = REG >> 8;
  //       REG_L = REG;
  //       byteBuffer = 0;
  //     }

  //   }


  // }


}

