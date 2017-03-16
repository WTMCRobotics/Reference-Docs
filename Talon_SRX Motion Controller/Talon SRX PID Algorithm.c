/*
* 1ms process for PIDF closed-loop.
*	@param pid ptr to pid object
* 	@param pos signed integral position (or velocity when in velocity mode).
* 		The target pos/velocity is ramped into the target member from caller's 'in'.
* 		If the CloseLoopRamp in the selected Motor Controller Profile is zero then
* 			there is no ramping applied. (throttle units per ms)
* 		PIDF is traditional, unsigned coefficients for P,I,D, signed for F.
*		Izone gives the abilty to autoclear the integral sum if error is wound up.
*	@param revMotDuringCloseLoopEn nonzero to reverse PID output direction.
*	@param oneDirOnly when using positive only sensor, keep the closed-loop from outputing negative throttle.
*
* Use IGain to make up for P only error (PGain only provides a value if there is an error)
* Use DGain iAccumf you want to react to sharp changes in error (smooth things out).
*/

// PID structure
//	bool	  notFirst;		// '1' if not first scan
//	double 	  target;		// setpoint
//	int32_t	  err;			// current ever
//	double	  iAccum;		// accumulated integral value
//	double	  dErr;			// derivative error
//	double	  outBeforeRmp;	// out value before applying CloseLoopRampRate
//	double	  out;			// output value after ramp rate
//	double 	  prevErr; 		// previous error, used to compute derivative

// Motor Controller Profile structure
//	double 	  P;			// Proportional Gain Factor
//	double 	  I; 			// Integral Gain Factor
//	double 	  D; 			// Derivitive Gain Factor
//	double 	  F; 			// Feedforard Gain Factor
//	uint32_t  iZone;		// Integral Error Zone - iAccum will be reset if it ever exceeds iZone
//	double 	  CloseLoopRampRate;

// PID_Mux_Unsigned()		// multiply err value by unsigned gain value
// BDC_GetThrot()

void PID_Calc1Ms(pid_t * pid,						// pointer to pid structure 
				 int32_t pos,						// PV - process variable, could be position or velocity
				 uint8_t revMotDuringCloseLoopEn, 	// 
				 uint8_t oneDirOnly)
{
	// grab selected slot
	MotorControlProfile_t * slot = MotControlProf_GetSlot();

	// calculate curent error (err = target - pos) 
	int32_t err = pid->target - pos;
	pid->err = err;

	// get absolute error (used by IZone logic)
	int32_t absErr = err;
	if(err < 0)
		absErr = -absErr;
	
	// calculate integral error
	if (0 == pid->notFirst)
	{
		// first scan
		pid->iAccum = 0;
		// also tare the before ramp throt
		pid->out = BDC_GetThrot(); // then save the current ramp
	}
	else 
	{
		// not the first scan
		//  ( slot->IZone == 0) || (absErr < slot->IZone) )
		if( (!slot->IZone     ) || (absErr < slot->IZone) )
		{
			// update iAccum if IZone is not used OR absErr is within IZone
			pid->iAccum += err;
		}
		else
		{
			// clear iAccum if IZone used and absErr is > IZone
			pid->iAccum = 0;
		}
	}

	// calculate derivative error
	if (pid->notFirst)
	{
		// not the first scan, dErr = current error = previous error
		// calc dErr
		pid->dErr = (err - pid->prevErr);
	}
	else
	{
		// first scan
		// clear dErr
		pid->dErr = 0;
	}
	
	// calculate new pre-ramp output
	//
	
	// output =  proportional component
	//	P-Component = P-Gain * currentError
	pid->outBeforRmp = PID_Mux_Unsigned(err, slot->P);

	// output += I-component
	// 	I-Component = I-Gain * iAccum
	//	skip if there I-Gain or iAccum == 0
	if (pid->iAccum && slot->I)
	{
		pid->outBeforRmp += PID_Mux_Unsigned(pid->iAccum, slot->I);
	}
	
	// output += D-component
	// 	D-Component = D-Gain * dErr
	pid->outBeforRmp += PID_Mux_Unsigned(pid->dErr, slot->D);
	
	// output += F-component
	//	F-Component = F-Gain * target (SetPoint)
	pid->outBeforRmp += PID_Mux_Signed(pid->target, slot->F);


	// arm for next pass
	{
		pid->prevErr = err; // save currenet error for next scan's D calculation
		pid->notFirst = 1; 	// already serviced first pass
	}
	

	// adjust outpu based on one direction and reverse output options
	//

	// only allow positive output if oneDirection is true
	if(oneDirOnly)
	{
		if(pid->outBeforRmp < 0)
			pid->outBeforRmp = 0;
	}
	
	// invert the output if revMotDuringCloseLoopEn is true
	if(revMotDuringCloseLoopEn)
		pid->outBeforRmp = -pid->outBeforRmp;
	

	// limit output based on CloseLoopRampRate
	//
	if (0 != slot->CloseLoopRampRate)
	{
		if (pid->outBeforRmp >= pid->out)
		{
			// limit increase to CloseLoopRampRate (throttle units per mSec.)
			int32_t deltaUp = pid->outBeforRmp - pid->out;	// newOut - prevOut
			if (deltaUp > slot->CloseLoopRampRate)
				deltaUp = slot->CloseLoopRampRate;
			pid->out += deltaUp;
		}
		else
		{
// DOESN'T THIS CODE POTENTIALLY VIOLATE OneDirectionOnly !!!!!!!!
			// limit decrease to CloseLoopRampRate (throttle units per mSec.)
			int32_t deltaDn = pid->out - pid->outBeforRmp;	// prevOut - newOut
			if (deltaDn > slot->CloseLoopRampRate)
				deltaDn = slot->CloseLoopRampRate;
			pid->out -= deltaDn;
		}
	}
	else
	{
		pid->out = pid->outBeforRmp;
	}
}