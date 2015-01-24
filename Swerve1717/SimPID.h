#ifndef SIMPID_H_
#define SIMPID_H_

typedef struct PIDValues_struct{
	PIDValues_struct(float p = 0.f, float i = 0.f, float d = 0.f){ this->p = p; this->i = i; this->d = d; }
	float p;
	float i;
	float d;
}PIDValues;

class SimPID
{
public:
	SimPID(PIDValues values, int epsilon = 0, bool angleWrap = false);
	
	void setConstants(PIDValues values);
	void setErrorEpsilon(float epsilon);
	void setErrorIncrement(float inc);
	void setDesiredValue(float val);
	void setMaxOutput(float max);
	void resetErrorSum(void);
		
	float calcPID(float current);
	
	bool isDone(void);
	void setMinDoneCycles(int n);
	
private:
	float m_p;   // P coefficient
	float m_i;   // I coefficient
	float m_d;   // D coefficient

	float m_desiredValue; // Desired value
	float m_previousValue; // Value at last call
	float m_errorSum; // Sum of previous errors (for I calculation)
	float m_errorIncrement; // Max increment to error sum each call
	float m_errorEpsilon; // Allowable error in determining when done
	
	bool m_firstCycle; // Flag for first cycle
	float m_maxOutput; // Ceiling on calculation output
	
	bool m_angleWrap;

	int m_minCycleCount; // Minimum number of cycles in epsilon range to be done
	int m_cycleCount; // Current number of cycles in epsilon range
};

#endif // SIMPID_H_
