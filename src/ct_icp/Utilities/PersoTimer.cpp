#include "../../../include/ct_icp/Utilities/PersoTimer.h"


PersoTimer::PersoTimer() : _t(), _dt(0.0)
{
}


PersoTimer::~PersoTimer()
{
}

void PersoTimer::tic()
{
	_t = clock();

}

double PersoTimer::toc()
{
	_dt = (float) (clock() - _t) / ((float) CLOCKS_PER_SEC);
	cout << "Elapsed time : " << _dt << endl;

	return _dt;
}

double PersoTimer::toc(string str)
{
	_dt = (float)(clock() - _t) / ((float)CLOCKS_PER_SEC);
	cout << "Elapsed time (" << str << ") : " << _dt << endl;

	return _dt;
}