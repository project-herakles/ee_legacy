#include "sineWave.h"
float sineWave(float amp,float frequency,float time){
	return amp*sin(2*PI*frequency*time);
}
