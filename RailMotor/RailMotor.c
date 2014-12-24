/* ========================================
 *
 * Copyright ROBOTCLUB SOFTWARE.
 * 
 * ========================================
*/

#include <RailMotor.h>


// プロトタイプ宣言
static void RailMotor_mmToCount( RailMotor* );
static void RailMotor_getDefVel( RailMotor* );

// RailMotor 初期化関数
void RailMotor_init( RailMotor* this ,double dt,double r,double opeMax,int16 resolution ,int16 railMax ,double p,double i,double d)
{
	this->mode = NoneMode;
	this->Pid.p = (double)p;
	this->Pid.i = (double)i;
	this->Pid.d = (double)d;
	this->dt    = (double)dt;
	this->r     = (double)r;
	this->resolution = resolution;
	this->VelPid.defVel[0] = 0;
	this->VelPid.defVel[1] = 0;
	this->VelPid.defDefVel[0] = 0;
	this->VelPid.defDefVel[1] = 0;
	this->opeMax = (double)opeMax;
	this->railMax = railMax;
	this->railMin = 0;
}

// RailMotor 更新関数
double RailMotor_upDate( RailMotor* this ,const int16 n,const int16 dn,const int16 ddn)
{
	this->n   = (int16)n;
	this->dn  = (int16)dn;
	this->ddn = (int16)ddn;
	if(this->mode == NoneMode)
	{
		return this->VelPid.opeVel = 0;
	}
	
	RailMotor_mmToCount(this);
	// 台形位置制御
	if(this->mode & PosRampMode)
	{
		this->PosRamp.defPos = (int16)this->x - this->PosRamp.posInit;
		if(this->PosRamp.axelMode == 0)
		{
			if((this->PosRamp.defPos * this->PosRamp.dir) < this->PosRamp.axelPos)
			{
				this->PosRamp.outVel = this->PosRamp.defPos * this->PosRamp.axel + this->PosRamp.vMin * this->PosRamp.dir;
			}
			else
			{
				this->PosRamp.axelMode = 1;
			}
		}
		else if(this->PosRamp.axelMode == 1)
		{
			this->PosRamp.outVel = this->PosRamp.vMax * this->PosRamp.dir;
			if((this->PosRamp.defPos * this->PosRamp.dir) > ((this->PosRamp.dir*this->PosRamp.defPosInit) - this->PosRamp.axelPos))
			{
				this->PosRamp.axelMode = 3;
			}
		}
		// 最高速度に到達しない場合
		else if(this->PosRamp.axelMode == 2)
		{
			if((this->PosRamp.defPos * this->PosRamp.dir) < (this->PosRamp.defPosInit/2*this->PosRamp.dir))
			{
				this->PosRamp.outVel = this->PosRamp.defPos * this->PosRamp.axel + this->PosRamp.vMin * this->PosRamp.dir;
			}
			else
			{
				this->PosRamp.axelMode = 3;
			}
		}
		else if(this->PosRamp.axelMode == 3)
		{
			if((this->PosRamp.defPos * this->PosRamp.dir) < (this->PosRamp.defPosInit * this->PosRamp.dir))
			{
				this->PosRamp.outVel = (this->PosRamp.defPosInit - this->PosRamp.defPos) * this->PosRamp.axel;
			}
			else
			{
				this->PosRamp.axelMode = 4;
			}
		}
		
		else if(this->PosRamp.axelMode == 4)
		{
			if((this->v < 10) && (this->v > -10) 
				&& (this->PosRamp.targetPos-10 < this->x) && (this->PosRamp.targetPos+10 > this->x))
			{
				this->PosRamp.outVel = 0;
				this->PosRamp.axelMode = 0;
				this->mode &= ~PosRampMode;
			}
			else
			{
				this->PosRamp.outVel = (this->PosRamp.targetPos - this->x) * this->PosRamp.axel;
			}
		}
		this->VelPid.targetVel = this->PosRamp.outVel;
	}
	// フィードバックブレーキ(位置制御を一回以上したあとでないとバグるかも)
	if((this->mode & FBBrakeMode) && !(this->mode & PosRampMode))
	{
		if(((this->PosRamp.targetPos - this->x) < 2) && ((this->PosRamp.targetPos - this->x) > -2))
		{
			this->VelPid.targetVel = this->PosRamp.outVel = 0;
		}
		else
		{
			this->PosRamp.outVel = (this->PosRamp.targetPos - this->x) * this->PosRamp.axel;
			this->VelPid.targetVel = this->PosRamp.outVel;
		}
	}
	// PID速度制御
	if(this->mode & VelPidMode)
	{
		RailMotor_getDefVel(this);
		this->VelPid.defOpeVel = 
			(double)this->VelPid.defDefVel[0] * (double)this->Pid.p
			+ (this->VelPid.defDefVel[0] - this->VelPid.defDefVel[1]) * this->Pid.d
			+ this->VelPid.defVel[0] * this->Pid.i;
			
		this->VelPid.opeVel += (double)this->VelPid.defOpeVel;
		this->VelPid.defDefVel[1] = (double)this->VelPid.defDefVel[0];
		this->VelPid.defVel[1]    = (double)this->VelPid.defVel[0];
		if(this->VelPid.opeVel > this->opeMax)
		{
			this->VelPid.opeVel = this->opeMax;
		}
		else if(this->VelPid.opeVel < -this->opeMax)
		{
			this->VelPid.opeVel = -this->opeMax;
		}
		return this->VelPid.opeVel;
	}
	return 0;
}

// RailMotor 速度PID補償関数
void RailMotor_velPidControl( RailMotor* this,const int16 vel)
{
	if(!(this->mode & PosRampMode))
	{
		this->mode = VelPidMode;
		this->VelPid.targetVel = vel;
	}
}

// RailMotor 台形位置制御関数
void RailMotor_posRampControl( RailMotor* this,const int16 pos,const int16 axelPos,const int16 vMax,const int16 vMin)
{
	if(!(this->mode & PosRampMode))
	{
		this->mode |= VelPidMode;
		this->mode |= PosRampMode;
		if(pos > this->railMax)
		{
			this->PosRamp.targetPos = this->railMax;
		}
		else if(pos < this->railMin)
		{
			this->PosRamp.targetPos = this->railMin;
		}
		else
		{
			this->PosRamp.targetPos = pos;
		}
		this->PosRamp.defPosInit = this->PosRamp.targetPos-(int16)this->x;
		if(this->PosRamp.defPosInit > 0)
		{
			this->PosRamp.dir = 1;
		}
		else if(this->PosRamp.defPosInit > 0)
		{
			this->PosRamp.dir = -1;
		}
		this->PosRamp.posInit = (int16)this->x;
		this->PosRamp.axel = vMax / axelPos;
		this->PosRamp.vMax = vMax;
		this->PosRamp.vMin = vMin;
		this->PosRamp.axelPos = axelPos;
		if((this->PosRamp.defPosInit - axelPos * 2) > 0)
		{
			// 定速度状態が存在する
			this->PosRamp.axelMode = 0;
		}
		else
		{
			// 定速度状態が存在しない
			this->PosRamp.axelMode = 2;
		}
		this->PosRamp.outVel = 0;
		this->VelPid.targetVel = 0;
	}
}
void RailMotor_feedBackBrakeEnable( RailMotor* this)
{
	this->mode |= FBBrakeMode;
}
void RailMotor_feedBackBrakeDisable( RailMotor* this)
{
	this->mode &= ~FBBrakeMode;
}
// RailMotor mm/s <- Count 変換
static void RailMotor_mmToCount( RailMotor* this)
{
	this->x = ( (double)this->n   * 2 * PI * (double)this->r ) / (double)this->resolution;
	this->v = (( (double)this->dn  / (double)this->dt ) * 2 * PI * (double)this->r ) / (double)this->resolution;
	this->a = (( (double)this->ddn / (double)this->dt ) * 2 * PI * (double)this->r ) / (double)this->resolution;
}
// RailMotor 目標値との偏差と偏差の偏差を取得
static void RailMotor_getDefVel( RailMotor* this )
{
	this->VelPid.defVel[0] = (double)this->VelPid.targetVel - (double)this->v;
	this->VelPid.defDefVel[0] = (double)this->VelPid.defVel[0] - (double)this->VelPid.defVel[1];
}


/* [] END OF FILE */
