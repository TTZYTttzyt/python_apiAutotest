#ifndef _EMA_H_
#define _EMA_H_
#include "stdint.h"

typedef struct coordinate
{
  double x;
	double y;
	double z;
	double sqr_rho;
	double rho;
	double phi;
	double theta;
}Coordinate;

typedef struct mchArmAngle
{
	double gamma;
	double alpha;
	double phi;
}MchArmAngle;

void CordTF(Coordinate* cord , double x , double y , double z);
void AngleCalc(MchArmAngle* angle,Coordinate* cord);
#endif
