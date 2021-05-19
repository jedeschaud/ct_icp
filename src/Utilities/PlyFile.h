// Ver 27.02.15
/*
	Dev notes : 
	- Ecrire fonction getSize from types qui remplit _propertySize une fois que les autres tableaux sont remplis
*/

#pragma once

#include <sstream>
#include <list>

#include "File.h"



using namespace std;


enum plyFormat{ binary_little_endian = 0, binary_big_endian = 1, ascii = 2 };
enum plyTypes{ float32 = 0, float64 = 1, uchar = 2, int32 = 3, otherxx = -1 };


class PlyFile :
	public File
{
public:
	PlyFile(string path, openMode flag);
	~PlyFile();
	
	void readFile(char*& points, int& pointSize, int& numPoints);
	void writeFile(char* points, int numPoints, list<string> properties, list<plyTypes> types);

	void displayInfos();
	
	static const int READ_SIZE = 16000;
	static const int WRITE_SIZE = 16000;

private:
	void readHeader();
	void writeHeader();


private:
	string    _header;
	plyFormat _format;

	int       _propertyNum;
	string*   _propertyName;
	plyTypes* _propertyType;
	int*	  _propertySize;

	int   _numPoints;
	int   _pointSize;
};