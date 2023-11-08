#include <SoftwareSerial.h>
#include "SparkFun_UHF_RFID_Reader.h"

// data collection parameters
#define WINDOW_SIZE 50
#define SAMPLE_NUM 15

// buzzer pins
#define BUZZER1 9
#define BUZZER2 10

// create rfid shield paramters
#define NANO_BAUDRATE 38400
#define NANO_POWER 2700 // 27.0 dBm (max)
RFID nano; 

// program navigation
enum progState{WAIT, COLLECT, PRINT_DATA};

// RX, TX
SoftwareSerial rfidSerial(2, 3); 

// data collection arrays
float rssiBuf[WINDOW_SIZE];
unsigned long timeBuf[WINDOW_SIZE];
float rssiResultsBuf[SAMPLE_NUM];
float timeResultsBuf[SAMPLE_NUM];

int sampleCount = 0;
enum progState state = WAIT;

void setup() {
  Serial.begin(115200);

  // buzzer setup
  pinMode(BUZZER1, OUTPUT);
  pinMode(BUZZER2, OUTPUT);
  digitalWrite(BUZZER2, LOW);

  // rfid shield configuration
  if (setupNano(NANO_BAUDRATE) == false)
  {
    Serial.println("RFID shield failed to respond");
    while (1);
  }
  nano.setRegion(REGION_NORTHAMERICA);
  nano.setReadPower(NANO_POWER); 
  Serial.println("Successfully Iitialized");
}

void loop() {

  // stop tag reading to start
  nano.stopReading();

  Serial.read(); // burn extra byte
  Serial.println("Enter \" \" to begin data collection");

  // wait for user to send a character
  while (!Serial.available()); 
  // move on to data collection if space was sent
  if (Serial.read() != ' ') return;
  
  // get data
  for (sampleCount = 0; sampleCount < SAMPLE_NUM; sampleCount++) {
    if (sampleCount == 0) Serial.println("collecting data");

    Serial.print("SAMPLE NUM: ");
    Serial.println(sampleCount);
    Serial.print("READ NUM: ");

    // begin scanning for tags
    nano.startReading(); 

    // start measurement timer
    unsigned long measurementStart = millis();

    for (int i = 0; i < WINDOW_SIZE; i++) {
      Serial.print(i); Serial.print(" ");

      byte responseType = 0;
      while (responseType != RESPONSE_IS_TAGFOUND) {
        // check if any new data has come in from module
        if (nano.check() == true) {
          // break response into tag ID, RSSI, frequency, and timestamp
          responseType = nano.parseResponse(); 

          if (responseType == RESPONSE_IS_KEEPALIVE) {
            // Serial.println(F("Scanning"));
          }
        }
      }
      // stop timer
      unsigned long elapsedTime = millis() - measurementStart;

      // buzzer for user feedback
      tone(BUZZER1, 2093, 150); //C
      delay(25);
      digitalWrite(BUZZER1, LOW);

      int rssi = nano.getTagRSSI();
      rssiBuf[i] = (float) rssi;
      timeBuf[i] = elapsedTime;
      // reset measurement timer
      measurementStart = millis();
    }
    Serial.println();

    // find averages once buffers are full
    float rssiSum = 0;
    unsigned long timeSum = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
      rssiSum += rssiBuf[i];
      timeSum += timeBuf[i];
    }
    rssiResultsBuf[sampleCount] = rssiSum / WINDOW_SIZE;
    timeResultsBuf[sampleCount] = ((float) timeSum) / WINDOW_SIZE;

    
  }
  // output data after all samples have been collected
  printData();
}

// output data to serial monitor in csv format
void printData() {
  // stop tag reading
  nano.stopReading();
  
  Serial.println("Printing data");
  
  Serial.println("---------------------------\n");
  
  // print individual samples
  Serial.println("Sample,rssi,time");

  for (int i = 0; i < SAMPLE_NUM; i++) {
    
    Serial.print(i); Serial.print(","); 
    Serial.print(rssiResultsBuf[i]); Serial.print(",");
    Serial.println(timeResultsBuf[i]);
  }

  // calculate stats
  float rssiMean = calc_mean(rssiResultsBuf, SAMPLE_NUM);
  float timeMean = calc_mean(timeResultsBuf, SAMPLE_NUM);
  float rssiSTD = calc_std(rssiResultsBuf, SAMPLE_NUM, rssiMean);
  float timeSTD = calc_std(timeResultsBuf, SAMPLE_NUM, timeMean);
  float sqrtn = sqrt(SAMPLE_NUM);
  float rssiSE = rssiSTD / sqrtn;
  float timeSE = timeSTD / sqrtn;

  // print stats
  Serial.print("AVG,"); Serial.print(rssiMean); Serial.print(","); Serial.println(timeMean);
  Serial.print("STD,"); Serial.print(rssiSTD); Serial.print(","); Serial.println(timeSTD);
  Serial.print("SE,"); Serial.print(rssiSE); Serial.print(","); Serial.println(timeSE);

  Serial.println("\n---------------------------");
}

float calc_mean(float vals[], int n)
{
  float sum = 0;
  for (int i = 0; i < n; i++) sum += vals[i];
  return sum / n;
}

float calc_std(float vals[], int n, float avg)
{

  // find sum of squared differences from mean
  float sumSqDiff = 0;
  for (int i = 0; i < n; i++) sumSqDiff += pow((avg - vals[i]), 2);

  // find mean of sum of squared differences from the mean
  float meanSqDiff = sumSqDiff / n;

  // return sqrt of above
  return sqrt(meanSqDiff);
}

boolean setupNano(long baudRate)
{
  // initiate software serial communication
  nano.begin(rfidSerial); 

  // test to see if we are already connected to a module
  // this would be the case if the Arduino has been reprogrammed and the module has stayed powered
  rfidSerial.begin(baudRate); // assume module is already at our desired baud rate
  while(!rfidSerial); // wait for port to open

  //About 200ms from power on the module will send its firmware version at 115200. We need to ignore this.
  while(rfidSerial.available()) rfidSerial.read();
  
  nano.getVersion();

  // occurs if the baud rate is correct but the module is doing a continuous read
  if (nano.msg[0] == ERROR_WRONG_OPCODE_RESPONSE) {
    nano.stopReading();
    Serial.println(F("Module continuously reading. Asking it to stop..."));
    delay(1500);
  }
  else {
    // no response so assume shield has just been powered on and communicating at 115200bps
    rfidSerial.begin(115200); // start software serial at 115200
    nano.setBaud(baudRate); // tell module to use chosen baud rate. Ignore the response msg
    rfidSerial.begin(baudRate); // start software serial port, this time at chosen baud rate
  }

  // test connection
  nano.getVersion();
  if (nano.msg[0] != ALL_GOOD) return (false); // something is not right

  // the M6E has these settings no matter what
  nano.setTagProtocol(); // set protocol to GEN2

  nano.setAntennaPort(); // set TX/RX antenna ports to 1

  return (true);
}
