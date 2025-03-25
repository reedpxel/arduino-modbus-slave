#include <SoftwareSerial.h>

// Slave device id
unsigned char SLAVE_ID = 1;

// pins for communication with a master device
#define RX_M 3
#define TX_M 5
#define RE_DE 7

// pins for communication with BMS 
#define RX_BMS 9
#define TX_BMS 11

// time to wait for a BMS`s reply, ms
#define INTERVAL_TO_WAIT_FOR_BMS_MESSAGE 90

// time to wait for the entire BMS`s reply, ms
#define INTERVAL_TO_WAIT_FOR_ENTIRE_MESSAGE 300

// size of the BMS`s reply with main info
#define MAIN_MESSAGE_SIZE 36

// size of the BMS`s reply with info about lines` voltage
#define LINES_VOLTAGE_MESSAGE_SIZE 33

// size of the queries 0x03 and 0x04, other queries cause errors
#define MASTERS_QUERY_SIZE 8
#define BUFFER_SIZE 92
#define BMS_QUERY_SIZE 7

// functions` declarations
unsigned char* BMSMessageToArray(unsigned char mainInfoMessage[], unsigned char linesVoltageMessage[]);
unsigned short TwoBytesToShort(unsigned char high, unsigned char low);
float TwoBytesToFloat(unsigned char high, unsigned char low, int shiftToLeft);
int Pow(int base, int power);
unsigned char InputMessageChecking(unsigned char* query, unsigned char slave_id);
unsigned char* BufferToMessage_Success(unsigned char* buffer, unsigned char* query, unsigned char slave_id);
unsigned char* BufferToMessage_IllegalFunction(unsigned char* query, unsigned char slave_id);
unsigned char* BufferToMessage_IllegalDataAddress(unsigned char* query, unsigned char slave_id);
unsigned char* BufferToMessage_IllegalDataValue(unsigned char* query, unsigned char slave_id);
unsigned short Crc16Modbus(unsigned char* data, unsigned char length);

// buffers` definitions
unsigned char query[MASTERS_QUERY_SIZE];
unsigned char main_message_buffer[MAIN_MESSAGE_SIZE]{};
unsigned char lines_message_buffer[LINES_VOLTAGE_MESSAGE_SIZE]{};

// definitions of queries for BMS
unsigned char command_read_03[7] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};
unsigned char command_read_04[7] = {0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77};

// a timer to wait for a INTERVAL_TO_WAIT_FOR_BMS_MESSAGE ms, if there is no BMS`s reply
unsigned long t = 0;

// a timer to show program`s performance, used for debugging
// unsigned long t2 = 0;
int buffer_index = 0;
bool reading_process_goes_on = false;

//if data from BMS is got to the 1st time, both of the queries to BMS must be sent, otherwise data will not be got
bool data_read_1st_time = true;

// creating SoftwareSerial objects
SoftwareSerial S_Master(RX_M, TX_M);
SoftwareSerial S_BMS(RX_BMS, TX_BMS);

void setup() {

  // preparing pins for work
  pinMode(RX_M, INPUT);
  pinMode(TX_M, OUTPUT);
  pinMode(RX_BMS, INPUT);
  pinMode(TX_BMS, OUTPUT);
  pinMode(RE_DE, OUTPUT);

  // initially the microcontroller waits for a query from master, so voltage at DE_RE is LOW
  digitalWrite(RE_DE, LOW);

  // beginning of the Serial objects
  S_BMS.begin(9600);
  S_Master.begin(9600);
  Serial.begin(9600);

  // setting master device to be listened
  S_Master.listen();
}

void loop() {

  // the cycle waits for bytes from master device
  if (S_Master.available()>0){
    query[buffer_index] = S_Master.read();

    // not all queries are 8 bytes long, this check prevents buffer_index from not being equal to zero if a master sends such queries
    if (query[0]==SLAVE_ID)
      ++buffer_index;
    if (buffer_index>=MASTERS_QUERY_SIZE){

      //as the query is got, controller processes it
      switch(InputMessageChecking(query, SLAVE_ID)){
        case 0:{ // the query is viable
          
          // sending a command_read_03 query to BMS and writing the reply the main_message_buffer
          //t2 = millis();
          S_BMS.listen(); // setting softwareserial object to wait for BMS`s response
          // a query is sent only if it is requered to read the corresponding registers or if data is read to the first time
          if (query[3]<18 || data_read_1st_time){
            t=millis();
            for (int i=0; i<BMS_QUERY_SIZE; ++i)
              S_BMS.write(command_read_03[i]);
            buffer_index = 0;
            /* the reading process is interrupted if 
              1. the expected amount of bytes is got 
              2. there is no reply from BMS for INTERVAL_TO_WAIT_FOR_BMS_MESSAGE ms
              3. some bytes are got, but more than INTERVAL_TO_WAIT_FOR_ENTIRE_MESSAGE ms have passed 
                (added in case if a user will unplug the BMS connection during the reading process)
            */
            while(reading_process_goes_on || millis()-t<INTERVAL_TO_WAIT_FOR_BMS_MESSAGE){
              if (millis()-t>INTERVAL_TO_WAIT_FOR_ENTIRE_MESSAGE)
                break;
              if (S_BMS.available()>0){
                main_message_buffer[buffer_index] = S_BMS.read();
                if (main_message_buffer[buffer_index] == 0xDD)
                  reading_process_goes_on = true;
                if (reading_process_goes_on)
                  ++buffer_index;
                if (buffer_index>=MAIN_MESSAGE_SIZE){
                  reading_process_goes_on = false;
                  break;
                }
              }
            }
          }

          buffer_index = 0;

          // sending a command_read_04 query to BMS and writing the reply the main_message_buffer
          if (query[3]+query[5]>18 || data_read_1st_time){
            t=millis();
            for (int i=0; i<BMS_QUERY_SIZE; ++i)
              S_BMS.write(command_read_04[i]);
            /* the reading process is interrupted if 
              1. the expected amount of bytes is got 
              2. there is no reply from BMS for INTERVAL_TO_WAIT_FOR_BMS_MESSAGE ms
              3. some bytes are got, but more than INTERVAL_TO_WAIT_FOR_ENTIRE_MESSAGE ms have passed 
                (added in case if a user will unplug the BMS connection during the reading process)
            */
            while(reading_process_goes_on || millis()-t<INTERVAL_TO_WAIT_FOR_BMS_MESSAGE){
              if (millis()-t>INTERVAL_TO_WAIT_FOR_ENTIRE_MESSAGE)
                break;
              if (S_BMS.available()>0){
                lines_message_buffer[buffer_index] = S_BMS.read();
                if (lines_message_buffer[buffer_index] == 0xDD)
                  reading_process_goes_on = true;
                if (reading_process_goes_on)
                  ++buffer_index;
                if (buffer_index>=LINES_VOLTAGE_MESSAGE_SIZE){
                  reading_process_goes_on = false;
                  break;
                }
              }
            }
          }

          data_read_1st_time = false;
          buffer_index = 0;

          // the buffer simulates slave device`s memory
          unsigned char* buffer = BMSMessageToArray(main_message_buffer, lines_message_buffer);

          // sending the reply with data to the master device
          digitalWrite(RE_DE, HIGH);
          unsigned char* ans = BufferToMessage_Success(buffer, query, SLAVE_ID);
          for (int i=0; i<2*query[5]+5; ++i)
            S_Master.write(ans[i]);
          delete[] buffer; // memory must be freed to avoid memory leak
          delete[] ans; // memory must be freed to avoid memory leak
          digitalWrite(RE_DE, LOW);
          S_Master.listen();
          //Serial.print(millis()-t2);
          //Serial.println(" ms");
          break;

        }
        case 2:{ // the query`s code is not 0x03 or 0x04

          digitalWrite(RE_DE, HIGH);
          unsigned char* ans = BufferToMessage_IllegalFunction(query, SLAVE_ID);
          for (int i=0; i<5; ++i)
            S_Master.write(ans[i]);
          delete[] ans; // memory must be freed to avoid memory leak
          digitalWrite(RE_DE, LOW);
          break;

        }
        case 3:
        case 4:{ // the requested registed are not between 0 and 46

          digitalWrite(RE_DE, HIGH);
          unsigned char* ans = BufferToMessage_IllegalDataAddress(query, SLAVE_ID);
          for (int i=0; i<5; ++i)
            S_Master.write(ans[i]);
          delete[] ans; // memory must be freed to avoid memory leak
          digitalWrite(RE_DE, LOW);
          break;

        }
        case 5:{ // the CRC is incorrect

          digitalWrite(RE_DE, HIGH);
          unsigned char* ans = BufferToMessage_IllegalDataValue(query, SLAVE_ID);
          for (int i=0; i<5; ++i)
            S_Master.write(ans[i]);
          delete[] ans; // memory must be freed to avoid memory leak
          digitalWrite(RE_DE, LOW);
          break;

        }
      }
      buffer_index = 0;
    }
  }
}

// turns replies from BMS into a buffer with the processed data. The error of procession is the 1st byte, if it is non-zero,
// the entire buffer is set to zero
unsigned char* BMSMessageToArray(unsigned char mainInfoMessage[], unsigned char linesVoltageMessage[]){

    // memory allocation for the processed array
    unsigned char* ans = new unsigned char[92]();

    // check of the initial and last elements of the messages
    if (mainInfoMessage[0]!=0xdd ||
        mainInfoMessage[1]!=0x03 ||
        mainInfoMessage[35]!=0x77){ 
            ans[1] = 0x01; // discrepancy of the initial or/and last element of the main data message 
            return ans;
    }
    if (linesVoltageMessage[0]!=0xdd ||
        linesVoltageMessage[1]!=0x04 ||
        linesVoltageMessage[32]!=0x77){
            ans[1] = 0x02; // discrepancy of the initial or/and last element of the lines` voltage data message
            return ans;
    }

    // check of equality to zero of error codes of the initial values
    if (mainInfoMessage[2]){
        ans[1] = 0x03; // non-zero error code in the main data message
        return ans;
    }
    if (linesVoltageMessage[2]){
        ans[1] = 0x04; // non-zero error code in the lines` voltage data message 
        return ans;
    }

    // check of the length of the messages
    if (mainInfoMessage[3]!=0x1d){
        ans[1] = 0x05; // incorrect main data message`s length
        return ans;
    }
    if (linesVoltageMessage[3]!=0x1a){
        ans[1] = 0x06; // incorrect lines` voltage message`s length
        return ans;
    }

    //control sum check
    int sum = 0;
    for (int i=2; i<=32; ++i)
        sum+=mainInfoMessage[i];
    --sum;
    if (sum!=0xffff-TwoBytesToShort(mainInfoMessage[33], mainInfoMessage[34])){
        ans[1] = 0x07; // incorrect control sum in the main data message
        return ans;
    }
    sum=0;
    for (int i=2; i<=29; ++i)
        sum+=linesVoltageMessage[i];
    --sum;
    if (sum!=0xffff-TwoBytesToShort(linesVoltageMessage[30], linesVoltageMessage[31])){
        ans[1] = 0x08; //incorrect control sum in the lines` voltage message
        return ans;
    }

    //total voltage
    float temp_f = TwoBytesToFloat(mainInfoMessage[4], mainInfoMessage[5], 2);
    unsigned char* p_ch = reinterpret_cast<unsigned char*>(&temp_f);
    ans[2] = *(p_ch+3);
    ans[3] = *(p_ch+2);
    ans[4] = *(p_ch+1);
    ans[5] = *p_ch;

    //current
    if (TwoBytesToShort(mainInfoMessage[6], mainInfoMessage[7])<0x8000)
        temp_f = TwoBytesToFloat(mainInfoMessage[6], mainInfoMessage[7], 2);
    else
        temp_f = (TwoBytesToFloat(mainInfoMessage[6], mainInfoMessage[7], 0)-0xffff)/100;
    p_ch = reinterpret_cast<unsigned char*>(&temp_f);
    ans[6] = *(p_ch+3);
    ans[7] = *(p_ch+2);
    ans[8] = *(p_ch+1);
    ans[9] = *p_ch;

    //remaining charge
    temp_f = TwoBytesToFloat(mainInfoMessage[8], mainInfoMessage[9], 2);
    p_ch = reinterpret_cast<unsigned char*>(&temp_f);
    ans[10] = *(p_ch+3);
    ans[11] = *(p_ch+2);
    ans[12] = *(p_ch+1);
    ans[13] = *p_ch;

    //amount of cycles
    short temp_i = TwoBytesToShort(mainInfoMessage[12], mainInfoMessage[13]);
    ans[14] = temp_i>>8;
    ans[15] = temp_i;

    //lines are balanced
    temp_i = TwoBytesToShort(mainInfoMessage[16], mainInfoMessage[17]);
    ans[16] = temp_i>>8;
    ans[17] = temp_i;

    //lines` protection state
    temp_i = TwoBytesToShort(mainInfoMessage[20], mainInfoMessage[21]);
    ans[18] = temp_i>>8;
    ans[19] = temp_i;

    //remaining charge in percent
    ans[20] = 0x00;
    ans[21] = mainInfoMessage[23];

    //FET protection status on charge and discharge
    switch(mainInfoMessage[24]){
        case 0:
            ans[22] = 0; // status on CHARGE, 0 means the BMS is locked, 1 is unlocked 
            ans[23] = 0; // status on DISCHARGE
        break;
        case 1:
            ans[22] = 1;
            ans[23] = 0;
        break;
        case 2:
            ans[22] = 0;
            ans[23] = 1;
        break;
        case 3:
            ans[22] = 1;
            ans[23] = 1;
        break;
    }

    // 3 temperatures
    for (int i=0; i<3; ++i){
      temp_f = (TwoBytesToFloat(mainInfoMessage[2*i+27], mainInfoMessage[2*i+28], 0)-2731)/10;
      p_ch = reinterpret_cast<unsigned char*>(&temp_f);
      ans[24+4*i] = *(p_ch+3);
      ans[25+4*i] = *(p_ch+2);
      ans[26+4*i] = *(p_ch+1);
      ans[27+4*i] = *p_ch;
    }

    // voltage of 13 lines
    float voltage_max = -1e9, voltage_min = 1e9; 
    for (int i=0; i<13; ++i){
      temp_f = TwoBytesToFloat(linesVoltageMessage[2*i+4], linesVoltageMessage[2*i+5], 3);
      if (temp_f>voltage_max)
          voltage_max = temp_f;
      if (temp_f<voltage_min)
          voltage_min = temp_f;
      p_ch = reinterpret_cast<unsigned char*>(&temp_f);
      ans[36+4*i] = *(p_ch+3);
      ans[37+4*i] = *(p_ch+2);
      ans[38+4*i] = *(p_ch+1);
      ans[39+4*i] = *p_ch;
    }

    //difference between man and min voltage
    temp_f = round((voltage_max - voltage_min)*1000)/1000.;
    p_ch = reinterpret_cast<unsigned char*>(&temp_f);
    ans[88] = *(p_ch+3);
    ans[89] = *(p_ch+2);
    ans[90] = *(p_ch+1);
    ans[91] = *p_ch;

    return ans;
}

unsigned short TwoBytesToShort(unsigned char high, unsigned char low) { return 256*high+low; }

float TwoBytesToFloat(unsigned char high, unsigned char low, int shiftToLeft){
    float ans = high*256+low;
    return ans/(Pow(10, shiftToLeft));
}

int Pow(int base, int power){
    int ans = 1;
    for (int i=0; i<power; ++i)
        ans*=base;
    return ans;
}

// check of the master`s query
unsigned char InputMessageChecking(unsigned char* query, unsigned char slave_id){
  if (query[0]!=slave_id)
      return 1; // wrong slave ID
  if (query[1]!=0x03 && query[1]!=0x04)
      return 2; // wrong command, error code in message 1 (Illegal function)
  if (query[2] || query[3]>0x2e)
      return 3; // the first word to read is out of range, error code in message 2 (Illegal Data Address)
  if (query[4] || query[5]-query[3]>0x2e)
      return 4; // the reading query is out of range, error code in message 2 (Illegal Data Address)
  if (query[6]*0x100+query[7]!=Crc16Modbus(query, 6))
      return 5; // incorrect CRC, error code in message 3 (Illegal Data Value)
  return 0;
}

// forming a response with data
unsigned char* BufferToMessage_Success(unsigned char* buffer, unsigned char* query, unsigned char slave_id){
  unsigned char* ans = new unsigned char[2*query[5]+5];
  ans[0] = slave_id;
  ans[1] = query[1];
  ans[2] = 2*query[5];
  for (int i=3; i<2*query[5]+3; ++i)
      ans[i] = buffer[2*query[3]+i-3];
  unsigned short crc = Crc16Modbus(ans, 2*query[5]+3);
  ans[2*query[5]+3] = crc/0x100;
  ans[2*query[5]+4] = crc%0x100;
  return ans;
}

// forming responses, if the master`s query contains errors
unsigned char* BufferToMessage_IllegalFunction(unsigned char* query, unsigned char slave_id){
  unsigned char* ans = new unsigned char[5];
  ans[0] = slave_id;
  ans[1] = query[1]+0x80;
  ans[2] = 1;
  unsigned short crc = Crc16Modbus(ans, 3);
  ans[3] = crc/0x100;
  ans[4] = crc%0x100;
  return ans;
}

unsigned char* BufferToMessage_IllegalDataAddress(unsigned char* query, unsigned char slave_id){
  unsigned char* ans = new unsigned char[5];
  ans[0] = slave_id;
  ans[1] = query[1]+0x80;
  ans[2] = 2;
  unsigned short crc = Crc16Modbus(ans, 3);
  ans[3] = crc/0x100;
  ans[4] = crc%0x100;
  return ans;
}

unsigned char* BufferToMessage_IllegalDataValue(unsigned char* query, unsigned char slave_id){
  unsigned char* ans = new unsigned char[5];
  ans[0] = slave_id;
  ans[1] = query[1]+0x80;
  ans[2] = 3;
  unsigned short crc = Crc16Modbus(ans, 3);
  ans[3] = crc/0x100;
  ans[4] = crc%0x100;
  return ans;
}

// standard Modbus CRC
unsigned short Crc16Modbus(unsigned char* data, unsigned char length){
  unsigned short temp = 0xffff;
  for (int i=0; i<length; ++i){
      temp = temp ^ data[i];
      for (int j=0; j<8; ++j){
          int flag = temp & 0x0001;
          temp>>=1;
          if (flag)
              temp^=0xA001;
      }
  }
  unsigned short temp2 = temp>>8;
  temp = (temp<<8) | temp2;
  return temp;
}