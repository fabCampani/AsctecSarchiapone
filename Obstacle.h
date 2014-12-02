
#include <iostream>
using namespace std;

class Obstacle{
public:
	double x;
	double y;
	double z;
	double time;
	double size;

	Obstacle(){}
	Obstacle(Obstacle &_obs){
		x = _obs.x;
		y = _obs.y;
		z = _obs.z;
		time = _obs.time;
		size = _obs.size;

	}
	Obstacle(double xa, double ya, double za, double timea, double sizea){
		x = xa;
		y = ya;
		z = za;
		time = timea;
		size = sizea;
	}

	~Obstacle(){}
};