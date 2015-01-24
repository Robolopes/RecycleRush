#ifndef MULTI_SWITCH_H_
#define MULTI_SWITCH_H_

#include "WPILib.h"
#include <vector>

typedef std::vector<DigitalInput*> InputVector;

class MultiSwitch
{
	public:
		MultiSwitch(int numInputs, ...);
		UINT32 Get();
	private:
		InputVector mInputs;
};

#endif /* MULTI_SWITCH_H_ */
