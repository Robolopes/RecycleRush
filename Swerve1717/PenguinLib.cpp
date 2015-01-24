/*
---------------------------------------------------------------------------------------
This source file is part of the DP Engineering Academy Penguin Bot!


Copyright (c) 2008 Dos Pueblos Engineering Academy

---------------------------------------------------------------------------------------
*/

#include "PenguinLib.h"

void PenguinLib::Sleep(UINT64 ms)
{
	taskDelay((UINT64)(ms * sysClkRateGet() / 1000.0));
}

UINT32 PenguinLib::VirtualSlot(UINT32 slotChannel)
{
	return (slotChannel >> 16) & 0xFFFF;
}

UINT32 PenguinLib::VirtualChannel(UINT32 slotChannel)
{
	return slotChannel & 0xFFFF;
}
