#include "PCH.h"
#include "SensorNode.h"

namespace WSN
{
	std::string WorkingStateToString(const WorkingState& ws)
	{
		switch (ws)
		{
		case WorkingState::Transfer: 
			return "Transfer";
		case WorkingState::Collection: 
			return "Collection";
		case WorkingState::Recovery: 
			return "Recovery";
		}

		throw std::runtime_error("Unknown Working State in WorkingStateToString!");
		return "";
	}
}