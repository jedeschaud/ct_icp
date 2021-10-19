#pragma once

#include <iostream>
#include <string>
#include <time.h>
#include <ctime>


using namespace std;

class PersoTimer
{
public:
	PersoTimer();
	~PersoTimer();

	void   tic();
	double toc();
	double toc(string str);

private:
	clock_t _t;
	double  _dt;
};