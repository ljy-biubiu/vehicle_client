#pragma once
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <string>
#include <vector>
#include <unistd.h>
#include <sstream>

using namespace std;

class Util {
public:
	static int mkdir_r(const char *path);// 创建多级目录

	static int scan_one_dir( const char * path, vector<string> &vecStr);// 列出一个目录下所有文件

	static void copyFile(const char *sourceFile, const char *destFile);// 复制文件

	static void moveFile(const char *sourceFile, const char *destFile);// 移动或剪切文件

	static std::vector<std::string> split(const std::string &str, const std::string &pattern);// 字符串分割

	static std::string &trim(std::string &s);// 去除字符串首尾空格、Tab

	static std::string formatValue(double val, int fixed);// 将double类型数值转为保留指定小数位数的字符串

	static std::string formatValue(float val, int fixed);// 将float类型数值转为保留指定小数位数的字符串

	static bool isNum(const string& str);// 使用C++提供的stringstream对象,判断字符串是否全为数字

    std::string intToString(int i,unsigned int zeroNum);// 将整数转为字符串，并在前面加0,使字符串为相同长度，（123， 6） -> "000123"

    string timeString(const string& spilt);// 获取当前时间的字符串，精确到毫秒，毫秒处用字符串spilt分割开

    static double timeMs();

    static unsigned long long timeUs();
};

