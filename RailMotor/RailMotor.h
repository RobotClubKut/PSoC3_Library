/* ========================================
 *
 * Copyright ROBOTCLUB SOFTWARE.
 * 
 * Vel = Velocity
 * Pos = Position
 * ========================================
*/

#include <project.h>

#define PI 3.14159

// Mode 列挙体
enum RailMotor_Mode
{
	NoneMode    = 0,
	VelPidMode  = 1,
	PosRampMode = 2,
	FBBrakeMode = 4
};

typedef struct
{
	double targetVel;
	double defVel[2];
	double defDefVel[2];
	double opeVel;
	double defOpeVel;
	double axel;
} VelPidParameter;

typedef struct
{
	int16 targetPos;
	int16 posInit;
	int16 defPosInit;
	int16 defPos;
	int16 vMax;
	int16 vMin;
	int16 axelPos;
	int16 dir;
	int16 axelMode;
	double axel;
	double outVel;
} PosRampParameter;

typedef struct
{
	double p;
	double i;
	double d;
} PidParameter;

// RailMotor 構造体
typedef struct
{
	uint8 mode;
	int16 resolution;
	int16 n;
	int16 dn;
	int16 ddn;	
	double x;
	double dx;
	double v;
	double a;
	double r;
	double dt;
	double opeMax;
	int16 railMin;
	int16 railMax;
	PidParameter Pid;
	VelPidParameter VelPid;
	PosRampParameter PosRamp;
} RailMotor;

void RailMotor_init( RailMotor* ,double,double,double,int16,int16,double,double,double);
double RailMotor_upDate( RailMotor* ,const int16 ,const int16 ,const int16 );
void RailMotor_velPidControl( RailMotor* ,const int16 );
void RailMotor_posRampControl( RailMotor* ,const int16 ,const int16 ,const int16 ,const int16 );
void RailMotor_feedBackBrakeEnable( RailMotor* );
void RailMotor_feedBackBrakeDisable( RailMotor* );
/* [] END OF FILE */
