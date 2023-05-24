
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "database.h"

#pragma once
using namespace std;

class CoeffDataManager
{
	private:
		ifstream coeffDataFile;
		int coeffs[3];
	public:
		CoeffDataManager(){};
		int* getCoeffs();
};

