#ifndef FILEHANDDLE_H
#define FILEHANDDLE_H

#include <jsoncpp/json/json.h>
#include "myparams.h"
#include <fstream>

class FileHanddle
{
public:
    FileHanddle();

    int writeDataLocal(TotalParams &total_params);
    void readDataLocal(TotalParams &total_params);
    void init(TotalParams &total_params);

private:
    std::string read_file_path;
};

#endif // FILEHANDDLE_H
