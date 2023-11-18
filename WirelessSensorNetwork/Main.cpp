#include "WSNPCH.h"

#include "DistributionParameters.h"
#include "Simulation.h"


static constexpr int s_TotalDurationToBeTransferred = 3600 * 24 * 90;
static constexpr int s_TransferTime = 60;
static constexpr int s_RecoveryTime = 30;

int main()
{

	for (int redo = 100000; redo < 100001; redo++)
	{
		for (int meanMultiplier = 1; meanMultiplier <= 8; meanMultiplier *= 8)
		{
			for (int stddev = 900; stddev <= 7200; stddev += 900)
			{
				std::cout << "Starting :\t Redo : " << redo << ",\t Standard Deviation : " << stddev * meanMultiplier << ",\t Mean : " << 3600 * meanMultiplier << '\n';
				int currentMean = 3600 * meanMultiplier;
				// totalDurationToBeTransferred, transferTime, recoveryTime, mean, stddev, redo
				WSN::Simulation* Si = new WSN::Simulation(s_TotalDurationToBeTransferred, s_TransferTime, s_RecoveryTime, currentMean, stddev * meanMultiplier, redo);


				Si->SimulateAll();

				// start delta, end delta, delta step
				Si->BruteForceAll(1, 3 * Si->GetDeltaOpt(), 1);

				Si->Summarize();
				Si->LogCDF();

				delete Si;
				WSN::Simulation::LogSummary();
			}
		}
	}

	WSN::Simulation::AverageAllRedos(100000, 100000);
	//throw std::runtime_error("DONE !");

	return 0;
}