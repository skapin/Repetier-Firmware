#include "Table.h"

#include "Repetier.h"


void Table::rotateToOrigin()
{
	this->rotateTable( this->offset );
}
void Table::initCaptor()
{
    uint32_t status = captor.begin();
 
    Serial.print("LSM9DS0 WHO_AM_I's returned: 0x");
    Serial.println(status, HEX);
    Serial.println("Should be 0x49D4");
    Serial.println();
}

void Table::setOffset( Point p )
{
	this->offset.x = p.x;
	this->offset.y = p.y;
	this->offset.z = p.z;
}
void Table::checkAndRotate()
{
	if ( ! this->isLeveled() )
	{
		rotateToOrigin();
	}
	
}
void Table::rotateTable( Point &p )
{
	
}
bool Table::isLeveled()
{
	Point p = this->getCurrentAngle();
	return ( DOF::angleEquals( p.x, 0.0 , TOLERANCE_ANGLE_ERROR ) 
		 && DOF::angleEquals( p.y, 0.0, TOLERANCE_ANGLE_ERROR ) );
	
}

Point Table::getCurrentAngle()
{
	Point p = captor.getCurrentAngle();
	p.x += this->offset.x;
	p.y += this->offset.y;
	p.z += this->offset.z;
	
	return p;	
}
