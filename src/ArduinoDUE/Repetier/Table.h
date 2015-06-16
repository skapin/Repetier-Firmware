#ifndef TABLE_H_INCLUDED
#define TABLE_H_INCLUDED

#include <inttypes.h>

#include "DOF.h"

#define TOLERANCE_ANGLE_ERROR 0.0001

class Table
{
	public:
	Table()
	{
		offset.x = offset.y = offset.z = 0.0;
	};
	~Table(){};

	/** Angle offset for pitch and roll for being perfectly leveled.  
	 * x for pitch
	 * x for roll
	 * z for yawl
	 * **/	
	Point offset;
	
	DOF captor;

	void rotateTable( Point &p );
	void rotateToOrigin();
	void checkAndRotate();
	void setOffset( Point origin);
	bool isLeveled();
	Point getCurrentAngle();
	
	
};











#endif // TABLE_H_INCLUDED

