#include "WSNPCH.h"

#include "DistributionParameters.h"
#include "Simulation.h"


int main()
{
	for (int redo = 2; redo < 3; redo++)
	{
		for (int meanMultiplier = 1; meanMultiplier <= 8; meanMultiplier *= 8)
		{
			for (int stddev = 900; stddev <= 7200; stddev += 900)
			{
				std::cout << "Starting :\t Redo : " << redo << ",\t Standard Deviation : " << stddev * meanMultiplier << ",\t Mean : " << 3600 * meanMultiplier << '\n';

				// totalDuration, transferTime, recoveryTime, mean, stddev, redo
				WSN::Simulation* Si = new WSN::Simulation(86400 * 90, 60, 30, 3600 * meanMultiplier, stddev * meanMultiplier, redo);


				Si->SimulateAll();

				// start delta, end delta, delta step
				Si->BruteForceAll(1, 100000, 1);

				Si->Summarize();

				delete Si;
			}
		}
	}

	WSN::Simulation::LogSummary();


	return 0;
}