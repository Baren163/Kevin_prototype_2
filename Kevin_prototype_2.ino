
#include <avr/io.h>
#include <avr/interrupt.h>


#define CPU_clock 16000000

#define SEND_START_CONDITION 100  // 01100100
#define SEND_START_CONDITION_AND_SET_TWINT 228  // 11100100
#define TWCR_INITIALISE 68  // 01000100
#define SET_TWINT 196 // 11000100
#define CLEAR_TWEA_FOR_NACK_AND_SET_TWINT 132 // 10000100
#define SEND_STOP_CONDITION 212
#define ST7735S_ADDRESS 80  // 1010000
#define MPU_ADDRESS 104
#define GYRO_CONFIG 27
#define FS_SEL0 3
#define FS_SEL1 4
#define GYRO_X_H 67
#define GYRO_X_L 68
#define ACCEL_Z_H 63
#define ACCEL_Y_H 61
#define ACCEL_X_H 59
#define SMPRT_DIV 25
#define CONFIG 26
#define OFFSET -530
#define alpha 0.95

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
bool reverseBool = 0;
bool stop = 0;
uint16_t ROW, COL = 0;
uint8_t byteBuffer = 0;
uint8_t idx = 0;
uint8_t incomingByte = 0;
uint8_t rowArray[160];
int drawImageY = 30;

float counter = 0;
float time_1, time_2;


int pos = 1;
int pos_target = 1;
int countDown = 4;
int pos_unmapped = 1;

float gyroAccelY;
float gyroAngle = 0;
float AccelZ;
float AccelY;
float accAngle;
int16_t gyroValue;  // data type 'short', signed 16 bit variable
unsigned long tempTime;
unsigned long time = 0;
float angle;


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

void writeMPU(uint8_t registerToWrite, uint8_t valueToWrite) {


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

        TWDR = (MPU_ADDRESS << 1);  // Load SLA + W

        break;

      case 16:
        // A repeated start condition has been transmitted

        break;

      case 24:
        // SLA+W has been transmitted; ACK has been received

        //Serial.println("SLA+W has been transmitted; ACK has been received, sending RA");

        TWDR = registerToWrite;  // Load register address (107)

        break;

      case 32:
        // SLA+W has been transmitted; NOT ACK has been received

        break;

      case 40:
        // Data byte has been transmitted; ACK has been received

        if (stop == 1) {
          stop = 0;
          IsrExitFlow = 3;
          break;
        }

        //Serial.println("Data byte has been transmitted; ACK has been received, sending '9'");

        TWDR = valueToWrite;  // Load decimal 9 into register 107 to clear sleep bit, disable temperature sensor and select Gyro X clock

        stop = 1;

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

        // gyroValue = (TWDR);

        //Serial.print("High byte stored in gyroValue: ");
        //Serial.println(gyroValue);
        break;

      case 3:
        TWCR = SEND_STOP_CONDITION;


        return;
        break;

      default:
        break;
    }
  }
}

int16_t readMPU(uint8_t registerToRead) {

  int16_t readValue;

  // While communication with gyro device bit is set
  while (1) {

    // While TWINT is 0 wait in this loop
    while (!(TWCR & (1 << TWINT))) {
    }

    IsrExitFlow = 0;

    TWCR = TWCR_INITIALISE;  // Set TWINT to clear interrupt


    // Read from GYRO_X
    switch (TWSR) {

      case 8:
        //  Start condition has been transmitted
        // Serial.println("TWSR reads 8");
        TWCR = TWCR_INITIALISE;

        TWDR = (MPU_ADDRESS << 1);  // Load SLA + W
        break;

      case 16:
        // A repeated start condition has been transmitted

        TWDR = ((MPU_ADDRESS << 1) + 1);  // Load SLA + R
        //Serial.println(TWDR);
        break;

      case 24:
        // SLA+W has been transmitted; ACK has been received
        TWDR = registerToRead;  // Write the gyro data register address to the slave
        break;

      case 32:
        // SLA+W has been transmitted; NOT ACK has been received
        break;

      case 40:
        // Data byte has been transmitted; ACK has been received
        IsrExitFlow = 1;  // Exit ISR with start condition (Repeated START)
        break;

      case 48:
        // Data byte has been transmitted; NOT ACK has been received
        break;

      case 56:
        // Arbitration lost in SLA+W or data bytes
        break;

      case 64:
        // SLA+R has been transmitted; ACK has been received, data byte will be received and ACK will be returned

        // IsrExitFlow = 0;

        break;

      case 80:
        // Data byte has been received; ACK has been returned, data byte will be stored and NACK will be returned

        //Serial.print("TWDR value at supposed data receival: ");
        //Serial.println(TWDR);

        IsrExitFlow = 2;  // Return NACK

        break;

      case 88:
        // Data byte has been received; NOT ACK has been returned, data byte will be store and STOP condition will be sent to end transmission

        //gyroValue += ((uint16_t) (TWDR << 8));

        readValue = readValue << 8;

        readValue += TWDR;

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

        //Serial.print("High byte stored in gyroValue: ");
        //Serial.println(gyroValue);
        break;

      case 3:
        //Serial.println("STOP condition will be sent");
        TWCR = SEND_STOP_CONDITION;

        // readValue /= 10;
        readValue -= OFFSET;

        readValue = ((float)readValue / 32767) * 250;


        return readValue;

        break;

      default:
        break;
    }
  }

}


void writeI2C(uint8_t registerToWrite_H, uint8_t registerToWrite_L, uint8_t valueToWrite) {

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

        TWDR = (ST7735S_ADDRESS << 1);  // Load SLA + W

        break;

      case 16:
        // A repeated start condition has been transmitted
        TWCR = TWCR_INITIALISE;

        TWDR = (ST7735S_ADDRESS << 1);  // Load SLA + W

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


uint16_t readI2C(uint8_t registerToRead_H, uint8_t registerToRead_L) {

  uint16_t readValue;

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
          TWDR = ((ST7735S_ADDRESS << 1) + 1);  // Load SLA + R
        } else {
          TWDR = (ST7735S_ADDRESS << 1);  // Load SLA + W
        }

        break;

      case 16:
        // A repeated start condition has been transmitted

        TWDR = ((ST7735S_ADDRESS << 1) + 1);  // Load SLA + R
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

        TWDR = (ST7735S_ADDRESS << 1);  // Load SLA + W

        break;

      case 16:
        // A repeated start condition has been transmitted

        TWDR = ((ST7735S_ADDRESS << 1) + 1);  // Load SLA + R

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
  tft_write_data(0x05);    // 16-bit/pixel

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
  writeI2C(registerAddress >> 8, registerAddress, 0);
  setReadAddress = 0;
}

uint8_t currentRead() {
  currentReadBool = 1;
  TWCR = SEND_START_CONDITION;
  uint8_t byte = readI2C(0,0);
  currentReadBool = 0;
  return byte;
}

void registerWrite(uint8_t registerToWrite_H, uint8_t registerToWrite_L, uint8_t valueToWrite) {

  byteWrite = 1;
  TWCR = SEND_START_CONDITION;
  writeI2C(registerToWrite_H, registerToWrite_L, valueToWrite);
  byteWrite = 0;

}


void drawRow(uint8_t* rowArray) {

  digitalWrite(TFT_DC, HIGH); // Data mode
  digitalWrite(TFT_CS, LOW);

  if (reverseBool == 0) {
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
  } else {
    for (int row = 0; row < 16; row++) {
      for (int reg = 10; reg > 0; reg--) {
        for (int pixel = 0; pixel < 8; pixel++) {
          if ((rowArray[(row*10) + (reg)] >> pixel) & 1) {
            spi_transfer(0xFF);
            spi_transfer(0xF0);
          } else {
            spi_transfer(0);
            spi_transfer(0);     
          }
        }
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
        spi_transfer(0xF0);
      } else {
        spi_transfer(0);
        spi_transfer(0);
        spi_transfer(0);
        spi_transfer(0);
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

void drawBackground() {
  tft_set_addr_window(0, 0, 161, 131);

  digitalWrite(TFT_DC, HIGH); // Data mode
  digitalWrite(TFT_CS, LOW);

  for (int i = 0; i < 21384; i++) {
    spi_transfer(0x00);
    spi_transfer(0x00);
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
  REG = 17280;

  twiInitialise(12);  // 12 = 400kHz
  tft_init();


  TWCR = SEND_START_CONDITION;
  writeMPU(107, 9); // Initialise the MPU


  TWCR = SEND_START_CONDITION;
  gyroAccelY = readMPU(ACCEL_Y_H);



  // Set start angle as balance point
  TWCR = SEND_START_CONDITION;
  AccelZ = readMPU(ACCEL_Z_H);


  TWCR = SEND_START_CONDITION;
  AccelY = readMPU(ACCEL_Y_H);

  // Calculate accAngle
  accAngle = atan2(AccelY, AccelZ);
  accAngle *= RAD_TO_DEG;

  //setCurrentReadAddress(0); // CANNOT READ FROM CHIP BEFORE SETTING A READ ADDRESS FIRST (can also be done by writing data)

  drawBackground();

  ROW = 0;

}

// alphabet stored from 0 to 16639
// small_face stored at 16640 to 17279  (80x64)
// 4 planes stores from 17280 to 19199 (4 * (80x48))

void loop() {

  // Take Readings
  TWCR = SEND_START_CONDITION;
  gyroValue = readMPU(GYRO_X_H);

  TWCR = SEND_START_CONDITION;
  AccelZ = readMPU(ACCEL_Z_H);

  TWCR = SEND_START_CONDITION;
  AccelY = readMPU(ACCEL_Y_H);

  // Calculate accAngle
  accAngle = atan2(AccelY, AccelZ);
  accAngle *= RAD_TO_DEG;

  // Calculate gyroAngle
  tempTime = millis();
  time = (tempTime - time);
  gyroAngle = (((float)time/1000) * gyroValue);




  // Complementary Filter
  angle = (alpha * (angle + gyroAngle)) + ((1 - alpha) * accAngle);


  TWCR = SEND_START_CONDITION;
  gyroAccelY = readMPU(ACCEL_Y_H);

  // pos_unmapped = gyroAccelY;
  // pos_target = (int16_t)((pos_unmapped + 50) * (128.0 / 100.0));
  // pos += (pos_target - pos) * 0.7 - 5;

  pos = angle;
  

  if (pos < 40) {
    reverseBool = 1;
  } else {
    reverseBool = 0;
  }


  TWCR = SEND_START_CONDITION;
  sequentialRead(160, rowArray, ((REG + (ROW*10)) >> 8), (REG + (ROW*10))); // Read 1/3 of plane image
  

  tft_set_addr_window(0+pos, (drawImageY + ROW), 79+pos, (drawImageY + ROW+15));
  drawRow(rowArray);


  ROW += 16;

  if (ROW == 48) {
    ROW = 0;
    if (abs(pos - 40) < 8) {
      REG = 17280;
    } else if (abs(pos - 40) < 15) {
      REG = 17760;
    } else if (abs(pos - 40) < 30) {
      REG = 18240;
    } else {
      REG = 18720;
    }


    countDown--;
    if (countDown == 0) {

      tft_set_addr_window(0, drawImageY, pos, drawImageY + 48);

      digitalWrite(TFT_DC, HIGH); // Data mode
      digitalWrite(TFT_CS, LOW);
      for (int i = 0; i < (pos*48); i++) {
        spi_transfer(0);
        spi_transfer(0);
      }
      digitalWrite(TFT_CS, HIGH);


      tft_set_addr_window(pos+80, drawImageY, 160, drawImageY + 48);

      digitalWrite(TFT_DC, HIGH); // Data mode
      digitalWrite(TFT_CS, LOW);
      for (int i = 0; i < ((80-pos)*48); i++) {
        spi_transfer(0);
        spi_transfer(0);  
      }
      digitalWrite(TFT_CS, HIGH);


      countDown = 4;
    }

    // pos += pos_target * 3;

    // if (pos > 78) {
    //   pos_target = -1;
    // } else if (pos < 0) {
    //   pos_target = 1;

    // }

  }



  time = millis();
  


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

    
  //   // if (incomingByte == 70) { // ASCII: F

  //   //   readCycle = 1;
  //   //   REG = 0;
  //   //   setCurrentReadAddress(0);
  //   //   incomingByte = 0;

  //   //   delay(10);

  //   // }

  //   // if (incomingByte == 82) { // ASCII: R

  //   //   readCycle = 0;
  //   //   REG = 0;
  //   //   setCurrentReadAddress(0);
  //   //   Serial.println("Returned");
  //   //   incomingByte = 0;

  //   // }


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

