
#include "CoeffDataManager.hh"

int* CoeffDataManager::getCoeffs()
{
	this->coeffDataFile.open(COEFF_PATH);
	if (this->coeffDataFile.is_open()){
		string line;
		while(getline(this->coeffDataFile, line)){
			char* cstr = const_cast<char*>(line.c_str());
			char* p = strtok(cstr, ",");

			for(int i = 0; i < 3 && p != nullptr; i++){
				this->coeffs[i] = atoi(p);
				//cout << p;
				p = strtok(NULL, ",");
			}
		}
	}
	this->coeffDataFile.close();
	return this->coeffs;
}

