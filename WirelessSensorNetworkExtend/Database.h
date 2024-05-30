#if not _DEBUG
#pragma once
#include "Simulation.h"

namespace WSN
{
	class Database
	{
	public:
		Database(const Database&) = delete;

		inline static Database* GetDatabase() { return s_DatabaseInstance; }

		/// <summary>
		/// Saves the simulation hyperparameters
		/// </summary>
		/// <param name="simulationID">Simulation ID</param>
		/// <param name="simulationParameters">Simulation Hyperparameters</param>
		void Insert(uint64_t simulationID, const SimulationParameters& simulationParameters, const SimulationResults& sr, const SimulationType& st);

		/// <summary>
		/// Saves the initial sensor nodes in a simulation
		/// </summary>
		/// <param name="simulationID">Simulation ID</param>
		/// <param name="sensorNodes">Sensor Nodes within the simulation</param>
		void Insert(uint64_t simulationID, const std::vector<SensorNode>& sensorNodes, const const SimulationType& st);

		uint64_t GetLatestSimulationID();

	private:
		Database();

		static Database* s_DatabaseInstance;
	};
}
#endif