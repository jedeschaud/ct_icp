#pragma once

#include <iostream>
#include <string>
#include <fstream>



using namespace std;

enum openMode{ fileOpenMode_OUT = 0, fileOpenMode_IN = 1 };

class File
{
public:
	File(string path, openMode flag);
	~File();


protected:
	const openMode _mode;
	string     _path;
	fstream    _file;
};