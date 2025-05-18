#include <conio.h>
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>

#include "../dynamixel_sdk/include/dynamixel_sdk/dynamixel_sdk.h"                               

#define ADDR_TORQUE_ENABLE_XM          64       
#define ADDR_GOAL_POSITION_XM          116
#define ADDR_PROFILE_VELOCITY_XM       112
#define ADDR_PRESENT_POSITION_XM       132

#define ADDR_TORQUE_ENABLE_XL          24       
#define ADDR_GOAL_POSITION_XL          30
#define ADDR_MOVING_SPEED_XL           32
#define ADDR_PRESENT_POSITION_XL       37

// Data Byte Length
#define LEN_GOAL_POSITION_XL           2
#define LEN_PRESENT_POSITION_XL        2
#define LEN_GOAL_POSITION_XM           4
#define LEN_PRESENT_POSITION_XM        4

// Protocol version
#define PROTOCOL_VERSION                2.0          

// Default setting
#define DXL1_ID                         1      
#define DXL2_ID                         2          
#define DXL_ID_ALL                      {10,12,13}  
#define DXL_ID_XL                       {1,2,3,4,5,6,7,8,9,10,11,12} 
#define DXL_ID_XM                       {13,14,15,16,17,18,19,20,21,22,23,24,25,26}   



#define BAUDRATE                        1000000
#define DEVICENAME                      "COM6"    


#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold


#define ESC_ASCII_VALUE                 0x1b

using namespace std;

bool checkRange(const vector<vector<int>>& data, int minValue, int maxValue);
vector<vector<string>> readCSV(const string& filename);
vector<vector<int>> timeDxl(vector<vector<string>> data);
vector<vector<int>> degreeDxl(vector<vector<string>> data);
int mapDegree(int value, int minValue, int maxValue, int minByte, int maxByte);
vector<int> subtractVectors(const vector<int>& vec1, const vector<int>& vec2);
bool checkThreshold(const vector<int>& result, int threshold);


int main(){
  string filename = "../program/olegoleg.csv";
  vector<vector<string>> data = readCSV(filename);

  vector<vector<int>>  dataTime = timeDxl(data);

  vector<vector<int>>  dataDegree = degreeDxl(data);

  // Menampilkan seluruh data
  cout << "\nSeluruh Data Time CSV:\n";
  for (const auto& row : dataTime) {
      for (const auto& val : row) {
          cout << val << " ";
      }
      cout << endl;
  }
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Initialize GroupSyncWrite instance
  dynamixel::GroupSyncWrite groupSyncWriteXl(portHandler, packetHandler, ADDR_GOAL_POSITION_XL, 4);
  dynamixel::GroupSyncWrite groupSyncWriteXm(portHandler, packetHandler, ADDR_PROFILE_VELOCITY_XM, 8);

  // Initialize Groupsyncread instance for Present Position
  dynamixel::GroupSyncRead groupSyncReadXl(portHandler, packetHandler, ADDR_PRESENT_POSITION_XL, LEN_PRESENT_POSITION_XL);
  dynamixel::GroupSyncRead groupSyncReadXm(portHandler, packetHandler, ADDR_PRESENT_POSITION_XM, LEN_PRESENT_POSITION_XM);

  int dxl_comm_result = COMM_TX_FAIL;
  bool dxl_addparam_result = false;
  bool dxl_getdata_result = false;

  uint8_t dxl_error = 0;
  uint8_t param_goal_position_xl[4];
  uint8_t param_goal_position_xm[8];
  int32_t dxl1_present_position = 0, dxl2_present_position = 0; 

  // Open port
  if (portHandler->openPort()){
    printf("Succeeded to open the port!\n");
  }
  else{
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE)){
    printf("Succeeded to change the baudrate!\n");
  }
  else{
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    return 0;
  }

  // Enable Dynamixel#1 Torque
  for (int val : DXL_ID_ALL) {
    if (val < 13){
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, val, ADDR_TORQUE_ENABLE_XL, TORQUE_ENABLE, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS){
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      }
      else if (dxl_error != 0){
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
      }
      else{
        printf("Dynamixel#%d has been successfully connected \n", val);
      }

      dxl_addparam_result = groupSyncReadXl.addParam(val);
      if (dxl_addparam_result != true){
        fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", val);
        return 0;
      }
    } else {
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, val, ADDR_TORQUE_ENABLE_XM, TORQUE_ENABLE, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS){
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      }
      else if (dxl_error != 0){
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
      }
      else{
        printf("Dynamixel#%d has been successfully connected \n", val);
      }

      dxl_addparam_result = groupSyncReadXm.addParam(val);
      if (dxl_addparam_result != true){
        fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", val);
        return 0;
      }
    }
   }

  for(int i = 0; i < (data.size() - 1); i++){
    printf("Press any key to continue! (or press ESC to quit!)\n");
    if (getch() == ESC_ASCII_VALUE)
      break;

    // Allocate goal position value into byte array
    int indexes = 0;
    for (int val : DXL_ID_ALL){
      if (val < 13){
        int goal_pos = mapDegree(dataDegree[i][indexes], 0, 300, 0, 1023);
        cout << goal_pos << endl;
        param_goal_position_xl[0] = DXL_LOBYTE(DXL_LOWORD(goal_pos));
        param_goal_position_xl[1] = DXL_HIBYTE(DXL_LOWORD(goal_pos));
        param_goal_position_xl[2] = DXL_LOBYTE(DXL_LOWORD(dataTime[i][indexes]));
        param_goal_position_xl[3] = DXL_HIBYTE(DXL_LOWORD(dataTime[i][indexes])); 

         // Add Dynamixel#1 goal position value to the Syncwrite storage
        dxl_addparam_result = groupSyncWriteXl.addParam(val, param_goal_position_xl);
        if (dxl_addparam_result != true){
          fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", val);
          return 0;
        }

      } else {
        int goal_pos = mapDegree(dataDegree[i][indexes], 0, 360, 0, 4094);
        param_goal_position_xm[0] = DXL_LOBYTE(DXL_LOWORD(dataDegree[i][indexes]));
        param_goal_position_xm[1] = DXL_HIBYTE(DXL_LOWORD(dataDegree[i][indexes]));
        param_goal_position_xm[2] = DXL_LOBYTE(DXL_HIWORD(dataDegree[i][indexes]));
        param_goal_position_xm[3] = DXL_HIBYTE(DXL_HIWORD(dataDegree[i][indexes]));
        param_goal_position_xm[4] = DXL_LOBYTE(DXL_LOWORD(goal_pos));
        param_goal_position_xm[5] = DXL_HIBYTE(DXL_LOWORD(goal_pos));
        param_goal_position_xm[6] = DXL_LOBYTE(DXL_HIWORD(goal_pos));
        param_goal_position_xm[7] = DXL_HIBYTE(DXL_HIWORD(goal_pos));

        // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWriteXm.addParam(val, param_goal_position_xm);
        if (dxl_addparam_result != true){
          fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", val);
          return 0;
        }
      }
      indexes++;
    }

    // Syncwrite goal position
    dxl_comm_result = groupSyncWriteXl.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

    dxl_comm_result = groupSyncWriteXm.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

    // Clear syncwrite parameter storage
    groupSyncWriteXl.clearParam();
    groupSyncWriteXm.clearParam();

    do{
      // Syncread present position
      dxl_comm_result = groupSyncReadXl.txRxPacket();
      if (dxl_comm_result != COMM_SUCCESS){
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      }


      dxl_comm_result = groupSyncReadXm.txRxPacket();
      if (dxl_comm_result != COMM_SUCCESS){
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      }

      vector<int> prepos;
      prepos.clear();
      // Check if groupsyncread data of Dynamixel#1 is available
      int indexes = 0;
      for (int val : DXL_ID_ALL) {
        if (val < 13){
          dxl_getdata_result = groupSyncReadXl.isAvailable(val, ADDR_PRESENT_POSITION_XL, LEN_PRESENT_POSITION_XL);
          if (dxl_getdata_result != true)
          {
            fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", val);
            return 0;
          }
          dxl1_present_position = groupSyncReadXl.getData(val, ADDR_PRESENT_POSITION_XL, LEN_PRESENT_POSITION_XL);
          int dataPrepos = mapDegree(dxl1_present_position, 0, 1024, 0, 300);
          prepos.push_back(dataPrepos);
          printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", val, dataDegree[i][indexes], dataPrepos);
        } else {
          dxl_getdata_result = groupSyncReadXm.isAvailable(val, ADDR_PRESENT_POSITION_XM, LEN_PRESENT_POSITION_XM);
          if (dxl_getdata_result != true)
          {
            fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", val);
            return 0;
          }
          dxl2_present_position = groupSyncReadXm.getData(val, ADDR_PRESENT_POSITION_XM, LEN_PRESENT_POSITION_XM);
          int dataPrepos = mapDegree(dxl2_present_position, 0, 4095, 0, 360);
          prepos.push_back(dataPrepos);
          printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", val, dataDegree[i][indexes], dataPrepos);
        }
        indexes++;
      }

      vector<int> result = subtractVectors(dataDegree[i], prepos);
      int treshold = 10;

      if (checkThreshold(result, treshold)){
        break;
      } 

    }while(1);
  }

  // Close port
  portHandler->closePort();

  return 0;
}


bool checkRange(const vector<vector<int>>& data, int minValue, int maxValue) {
    for (const auto& row : data) {
        for (int value : row) {
            if (value < minValue || value > maxValue) {
                return false;
            }
        }
    }
    return true;
}


vector<vector<string>> readCSV(const string& filename) {
    ifstream file(filename);
    
    vector<vector<string>> data;
    
    if (!file.is_open()) {
        cerr << "Tidak dapat membuka file!" << endl;
        return data;
    }

    string line;
    while (getline(file, line)) {
        stringstream ss(line);
        string cell;
        vector<string> row; 

        while (getline(ss, cell, ',')) {
            row.push_back(cell);
        }

        data.push_back(row);
    }

    file.close();
    return data;
}

vector<vector<int>> timeDxl(vector<vector<string>> data) {
    vector<vector<int>> dataTime;
    int arrId[] = {10,12,13};
    for (size_t row = 1; row < data.size(); row++) {
        vector<int> dataRow;
        // for (size_t col = 1; col < 28; col++)
        // {
        //     dataRow.push_back(stoi(data[row][col])); 
        // }
        for (int val : arrId)
        {

          dataRow.push_back(stoi(data[row][(val+1)])); 
        }

        dataTime.push_back(dataRow);
    }
    
    return dataTime;
}

vector<vector<int>> degreeDxl(vector<vector<string>> data) {
    vector<vector<int>> dataDegree;
    int arrId[] = {10,12,13};
    for (size_t row = 1; row < data.size(); row++) {
        vector<int> dataRow;
        // for (size_t col = 28; col < 55; col++)
        // {
        //     dataRow.push_back(stoi(data[row][col])); 
        // }

        for (int val : arrId)
        {
            dataRow.push_back(stoi(data[row][(val+28)])); 
        }

        dataDegree.push_back(dataRow);
    }

    return dataDegree;
}

int mapDegree(int value, int minValue, int maxValue, int minByte, int maxByte) {
    int byteValue = (value - minValue) * (maxByte - minByte) / (maxValue - minValue) + minByte;

    if (byteValue < minByte) {
        byteValue = minByte;
    } else if (byteValue > maxByte) {
        byteValue = maxByte;
    }

    return byteValue;
}


vector<int> subtractVectors(const vector<int>& vec1, const vector<int>& vec2) {
    if (vec1.size() != vec2.size()) {
        throw invalid_argument("Vektor harus memiliki ukuran yang sama untuk pengurangan.");
    }

    vector<int> result(vec1.size());

    for (size_t i = 0; i < vec1.size(); ++i) {
        result[i] = abs(vec1[i] - vec2[i]); // Mengambil nilai absolut
    }

    return result;
}

bool checkThreshold(const vector<int>& result, int threshold) {
    for (int val : result) {
        if (val > threshold) {
            return false;
        }
    }
    return true;
}