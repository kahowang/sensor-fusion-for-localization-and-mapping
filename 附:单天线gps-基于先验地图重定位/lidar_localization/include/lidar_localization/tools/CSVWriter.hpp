#ifndef CSVWRITER_H
#define CSVWRITER_H
#include <fstream>
#include <iostream>
#include <sstream>
#include <typeinfo>

using namespace std;
class CSVWriter
{
    public:
        CSVWriter(){
            this->firstRow = true;
            this->seperator = ";";
            this->columnNum = -1;
            this->valueCount = 0;
        }

        CSVWriter(int numberOfColums){
            this->firstRow = true;
            this->seperator = ";";
            this->columnNum = numberOfColums;
            this->valueCount = 0;
        }

        CSVWriter(string seperator){
            this->firstRow = true;
            this->seperator = seperator;
            this->columnNum = -1;
            this->valueCount = 0;
        }

        CSVWriter(string seperator, int numberOfColums){
            this->firstRow = true;
            this->seperator = seperator;
            this->columnNum = numberOfColums;
            this->valueCount = 0;
            cout << this->seperator << endl;
        }

        CSVWriter& add(const char *str){
            return this->add(string(str));
        }

        CSVWriter& add(char *str){
            return this->add(string(str));
        }

        CSVWriter& add(string str){
            //if " character was found, escape it
            size_t position = str.find("\"",0);
            bool foundQuotationMarks = position != string::npos;
            while(position != string::npos){
                str.insert(position,"\"");
                position = str.find("\"",position + 2);
            }
            if(foundQuotationMarks){
                str = "\"" + str + "\"";
            }else if(str.find(this->seperator) != string::npos){
                //if seperator was found and string was not escapted before, surround string with "
                str = "\"" + str + "\"";
            }
            return this->add<string>(str);
        }

        template<typename T>
        CSVWriter& add(T str){
            if(this->columnNum > -1){
                //if autoNewRow is enabled, check if we need a line break
                if(this->valueCount == this->columnNum ){
                    this->newRow();
                }
            }
            if(valueCount > 0)
                this->ss << this->seperator;
            this->ss << str;
            this->valueCount++;

            return *this;
        }

        template<typename T>
        CSVWriter& operator<<(const T& t){
            return this->add(t);
        }

        void operator+=(CSVWriter &csv){
            this->ss << endl << csv;
        }

        string toString(){
            return ss.str();
        }

        friend ostream& operator<<(std::ostream& os, CSVWriter & csv){
            return os << csv.toString();
        }

        CSVWriter& newRow(){
            if(!this->firstRow || this->columnNum > -1){
                ss << endl;
            }else{
                //if the row is the first row, do not insert a new line
                this->firstRow = false;
            }
            valueCount = 0;
            return *this;
        }

        bool writeToFile(string filename){
            return writeToFile(filename,false);
        }

        bool writeToFile(string filename, bool append){
            ofstream file;
            if(append)
                file.open(filename.c_str(),ios::out | ios::app);
            else
                file.open(filename.c_str(),ios::out | ios::trunc);
            if(!file.is_open())
                return false;
            if(append)
                file << endl;
            file << this->toString();
            file.close();
            return file.good();
        }

        void enableAutoNewRow(int numberOfColumns){
            this->columnNum = numberOfColumns;
        }

        void disableAutoNewRow(){
            this->columnNum = -1;
        }
    protected:
        bool firstRow;
        string seperator;
        int columnNum;
        int valueCount;
        stringstream ss;

};

#endif // CSVWRITER_H