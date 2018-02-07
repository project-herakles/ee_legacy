#include "sineWave.h"
#include "stdio.h"
int main(){
	for(int i=0;i<1000;i++){
		printf("Sin:%f\n",sineWave(60,50,i/1000.0));
	}
	return 0;
}
