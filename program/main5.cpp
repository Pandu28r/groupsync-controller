#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

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


vector<vector<string>> readCSV(const string& filename, int arr[]) {
    ifstream file(filename);
    
    vector<vector<string>> dataAll;
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

        dataAll.push_back(row);
    }

    for(vector<string> val : dataAll){
        vector<string> row; 
        for (int valArr : arr){
            row.push_back(val[valArr]);
        }
        
    }

    file.close();
    return data;
}

vector<vector<int>> timeDxl(vector<vector<string>> data) {
    vector<vector<int>> dataTime;

    for (size_t row = 1; row < data.size(); row++) {
        vector<int> dataRow;
        for (size_t col = 1; col < 28; col++)
        {
            dataRow.push_back(stoi(data[row][col])); 
        }

        dataTime.push_back(dataRow);
    }


    return dataTime;
}

vector<vector<int>> degreeDxl(vector<vector<string>> data) {
    vector<vector<int>> dataDegree;

    for (size_t row = 1; row < data.size(); row++) {
        vector<int> dataRow;
        for (size_t col = 28; col < 55; col++)
        {
            dataRow.push_back(stoi(data[row][col])); 
        }

        dataDegree.push_back(dataRow);
    }

    return dataDegree;
}



int main() {
    string filename = "olegoleg.csv";
    int id[] = {1,2,3};
    vector<vector<string>> data = readCSV(filename, id);

    vector<vector<int>>  dataTime = timeDxl(data);

    vector<vector<int>>  dataDegree = degreeDxl(data);

    // Menampilkan seluruh data
    cout << "\nSeluruh Data CSV:\n";
    for (const auto& row : dataTime) {
        for (const auto& val : row) {
            cout << val << " ";
        }
        cout << endl;
    }

    return 0;
}
