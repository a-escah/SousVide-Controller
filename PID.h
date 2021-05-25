double P, I, D;
double maxError, errorSum, maxOutput, minOutput, maxIOutput, lastActual, lastOutput, outputFilter;
int  setPoint;
bool firstRun;

void init(){
	maxError = 0;
	errorSum = 0;
	maxOutput = 0;
	minOutput = 0;
	maxIOutput = 0;
	setPoint = 0;
	lastActual = 0;
	lastOutput = 0;
	firstRun = true;
	outputFilter = 0;
}

void checkSigns(){
	if(P<0) {P = P*-1;}
	if(I<0) {I = I*-1;}
	if(D<0) {D = D*-1;}
}

void startPID(double p, double i, double d){
	P = p; I = i; D = d;
	init();
	checkSigns();
}

void setP(double p){
	P = p;
	checkSigns();
}

void setI(double i){
	if(I!=0){
			errorSum=errorSum*I/i;
	}
	I = i;

}

void setD(double d){
	D = d;
}

void setPID(double p, double i, double d){
	P = p; I = i; D = d;
	checkSigns();
}

void setMaxIOutput(double maximum){
	maxIOutput=maximum;
	if(I!=0){
		maxError=maxIOutput/I;
	}
}

double clamp(double value, double min, double max){
	if(value > max){ return max;}
	if(value < min){ return min;}
	return value;
}

bool bounded(double value, double min, double max){
		return (min<value) && (value<max);
}

void setOutputLimits(double minimum,double maximum){
	if(maximum<minimum)return;
	maxOutput=maximum;
	minOutput=minimum;
	// Ensure the bounds of the I term are within the bounds of the allowable output swing
	if(maxIOutput==0 || maxIOutput>(maximum-minimum) ){
		setMaxIOutput(maximum-minimum);
	}
}

void setSetpoint(double setpoint){
	setPoint=setpoint;
}

void setOutputFilter(double strength){
	if(strength==0 || bounded(strength,0,1)){
		outputFilter=strength;
	}
}

double getOutput(float actual, int setpoint){
	double output;
	double Poutput;
	double Ioutput;
	double Doutput;
	setPoint = setpoint;
	double error = setpoint - actual;

	Poutput = P*error;

	if(firstRun){
			lastActual=actual;
			lastOutput=Poutput;
			firstRun=false;
	}

	Doutput= -D*(actual- lastActual);
	lastActual = actual;

	Ioutput=I*errorSum;
	if(maxIOutput!=0){
		Ioutput=clamp(Ioutput,-maxIOutput,maxIOutput);
	}
	output = Poutput + Ioutput + Doutput;

	if(minOutput!=maxOutput && !bounded(output, minOutput,maxOutput) ){
		errorSum=error;
	}
	else if(maxIOutput!=0){
		errorSum=clamp(errorSum+error,-maxError,maxError);
			// In addition to output limiting directly, we also want to prevent I term
			// buildup, so restrict the error directly
	}
	else{errorSum+=error; }

	if(minOutput!=maxOutput){
		output=clamp(output, minOutput,maxOutput);
	}
	if(outputFilter!=0){
		output=lastOutput*outputFilter+output*(1-outputFilter);
	}

	lastOutput=output;
	return output;
}

void resetPID(){
	firstRun=true;
	errorSum=0;
}
