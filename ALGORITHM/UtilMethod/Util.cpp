
#include "Util.h"

// 创建多级目录
int Util::mkdir_r(const char *path) {
	if (path == nullptr) {
		return -1;
	}
	if (access(path, 0) != -1){
		return -1;
	}
	char *temp = strdup(path);
	char *pos = temp;

	/* 去掉开头的 './' 或 '/' */
	if (strncmp(temp, "/", 1) == 0) {
		pos += 1;
	} else if (strncmp(temp, "./", 2) == 0) {
		pos += 2;
	}

	/* 循环创建目录 */
	for (; *pos != '\0'; ++pos) {
		if (*pos == '/') {
			*pos = '\0';
			mkdir(temp, 0755);
			//printf("for %s\n", temp);
			*pos = '/';
		}
	}

	/* 如果最后一级目录不是以'/'结尾，
	遇到'\0'就中止循环，
	不会创建最后一级目录 */
	if (*(pos - 1) != '/') {
		//printf("if %s\n", temp);
		mkdir(temp, 0755);
	}
	free(temp);
	return 0;
}

/*
 * 扫描一个目录下所有文件，不扫描下一级目录
 * */
int Util::scan_one_dir(const char *path,vector<string> &vecStr) {
	if (NULL == path) {
		cout << " path is null ! " << endl;
		return -1;
	}

	struct stat s{};
	lstat(path, &s);
	if (!S_ISDIR(s.st_mode)) {
		return -2;
	}

	struct dirent *filename;
	DIR *dir;
	dir = opendir(path);
	if (NULL == dir) {
		return -3;
	}


	while ((filename = readdir(dir)) != NULL) {
		if (strcmp(filename->d_name, ".") == 0 ||
		    strcmp(filename->d_name, "..") == 0) {
			continue;
		}

		char wholePath[1024] = {0};
		sprintf(wholePath, "%s/%s", path, filename->d_name);
		string tempStr = filename->d_name;
		vecStr.push_back(tempStr);
		//cout << "wholePath= " << wholePath << endl;
	}

	for (int i = 0; i < vecStr.size() - 1; ++i) {
		for (int j = i + 1; j < vecStr.size(); ++j) {
			if (vecStr.at(i) > vecStr.at(j)) {
				string tempStr = vecStr.at(i);
				vecStr.at(i) = vecStr.at(j);
				vecStr.at(j) = tempStr;
			}
		}
	}

	free(filename);
	free(dir);
	return 1;
}

/*
 * 拷贝、复制文件
 * */
void Util::copyFile(const char *sourceFile, const char *destFile) {
	std::string command = "cp ";
	command += sourceFile;
	command += " ";
	command += destFile;//cp /home/file1 /root/file2
	system((char *) command.c_str());//
}

/*
 * 移动、剪切文件
 * */
void Util::moveFile(const char *sourceFile, const char *destFile) {
	std::string command = "mv ";
	command += sourceFile;
	command += " ";
	command += destFile;//mv /home/file1 /root/file2
	system((char *) command.c_str());//
}

/*
 * 字符串分割
 * 参数：str-待分割的字符串; pattern-分割符号
 * 返回：容器数组
 * */
std::vector<std::string> Util::split(const std::string &str, const std::string &pattern) {
	//const char* convert to char*
	char *strc = new char[strlen(str.c_str()) + 1];
	strcpy(strc, str.c_str());
	std::vector<std::string> resultVec;
	char *tmpStr = strtok(strc, pattern.c_str());
	while (tmpStr != NULL) {
		resultVec.push_back(std::string(tmpStr));
		tmpStr = strtok(NULL, pattern.c_str());
	}
	delete[] strc;
	return resultVec;
}

/*
 * 去除字符串首尾空格、Tab
 * */
std::string &Util::trim(std::string &s) {
	if (!s.empty()) {
		const string &delim = " \t";
		s.erase(0, s.find_first_not_of(delim));
		s.erase(s.find_last_not_of(delim) + 1);
	}
	return s;
}

/* 将double类型数值转为保留指定小数位数的字符串
 * val 数值
 * fixed 保留的小数位数
 * */
std::string Util::formatValue(double val, int fixed) {
	auto str = std::to_string(val);
	return str.substr(0, str.find(".") + fixed + 1);
}

/* 将float类型数值转为保留指定小数位数的字符串
 * val 数值
 * fixed 保留的小数位数
 * */
std::string Util::formatValue(float val, int fixed) {
	auto str = std::to_string(val);
	return str.substr(0, str.find(".") + fixed + 1);
}

bool Util::isNum(const string& str) {
	stringstream sin(str);
	double d;
	char c;
	if (!(sin >> d)) {
		return false;
	}
	if (sin >> c) {
		return false;
	}
	return true;
}

std::string Util::intToString(int i, unsigned int zeroNum) {
    std::string si = std::to_string(i);
    int siSize = si.size();
    if (siSize < zeroNum) {
        for (int j = 0; j < zeroNum - siSize; ++j) {
            si = "0" + si;
        }
    }
    return si;
}

string Util::timeString(const string& spilt) {
    struct timeval tv{};
    struct timezone tz{};
    struct tm *t;
    gettimeofday(&tv, &tz);
    t = localtime(&tv.tv_sec);
    string year = intToString(1900 + t->tm_year, 4);
    string month = intToString(1 + t->tm_mon, 2);
    string day = intToString(t->tm_mday, 2);
    string hour = intToString(t->tm_hour, 2);
    string mint = intToString(t->tm_min, 2);
    string sec = intToString(t->tm_sec, 2);
    string uSec = intToString(tv.tv_usec / 1000, 3);
    string strTime = year + month + day + hour + mint + sec + spilt + uSec;
    return strTime;
}

double Util::timeMs() {
    timespec ts{};
    timespec_get(&ts, TIME_UTC);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / (1000.0 * 1000.0);
}

unsigned long long Util::timeUs() {
    timespec ts{};
    timespec_get(&ts, TIME_UTC);
    return (unsigned long long) (ts.tv_sec * 1000.0 * 1000.0 + ts.tv_nsec / (1000.0));
}
