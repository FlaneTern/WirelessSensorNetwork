#pragma once
#include "DistributionParameters.h"

namespace WSN
{
	enum class State
	{
		Collection,
		Transfer,
		Recovery
	};

	struct SimulationInterval
	{
		State State;
		long long StartTime;
		long long EndTime;
	};

	struct BruteForceData
	{
		long long Delta = 0;
		long long CollectionTime = 0;
		long long WastedTime = 0;
		long long ActualTotalDuration = 0;
		long long FinalFailureIndex = 0;
	};

	struct SimulationData
	{
		std::vector<SimulationInterval> SimulationIntervals;
		BruteForceData BruteForceData;
	};


	struct SimulationSummaryData
	{
		long long Mean = -1;
		long long StdDev = -1;
		long long TotalDurationToBeTransferred = -1;
		long long TransferTime = -1;
		long long RecoveryTime = -1;

		long long DeltaOpt = -1;


		long long CollectionTimeWeibull = -1;
		long long CollectionTimeGamma = -1;
		long long CollectionTimeLognormal = -1;

		long long WastedTimeWeibull = -1;
		long long WastedTimeGamma = -1;
		long long WastedTimeLognormal = -1;

		long long ActualTotalDurationWeibull = -1;
		long long ActualTotalDurationGamma = -1;
		long long ActualTotalDurationLognormal = -1;

		long long FinalFailureIndexWeibull = -1;
		long long FinalFailureIndexGamma = -1;
		long long FinalFailureIndexLognormal = -1;


		long long DeltaStarWeibull = -1;
		long long DeltaStarGamma = -1;
		long long DeltaStarLognormal = -1;

		long long CollectionTimeStarWeibull = -1;
		long long CollectionTimeStarGamma = -1;
		long long CollectionTimeStarLognormal = -1;

		long long WastedTimeStarWeibull = -1;
		long long WastedTimeStarGamma = -1;
		long long WastedTimeStarLognormal = -1;

		long long ActualTotalDurationStarWeibull = -1;
		long long ActualTotalDurationStarGamma = -1;
		long long ActualTotalDurationStarLognormal = -1;

		long long FinalFailureIndexStarWeibull = -1;
		long long FinalFailureIndexStarGamma = -1;
		long long FinalFailureIndexStarLognormal = -1;

		long long RedoCount = -1;

	};

	template<typename T>
	class Distribution
	{
	public:
		Distribution() {}
		Distribution(double a, double b)
		{
			m_Distribution = T(a, b);
		}

		void GenerateFailures(std::mt19937_64& random, long long totalDurationToBeTransferred, std::string distname);

		T m_Distribution;
		std::vector<long long> m_FailurePoints;
		std::vector<long long> m_Intervals;

		std::string m_Name;

		void LogCDF(long long finalFailure, bool star);
	};

	class Simulation
	{
	public:
		Simulation(long long TotalDurationToBeTransferred, long long transferTime, long long recoveryTime, long long mean, long long stddev, long long redo);

		void SimulateAll();

		template<typename T>
		SimulationData SimulateSpecific(T& distribution);


		void BruteForceAll(long long start, long long end, long long step);

		template<typename T>
		std::vector<BruteForceData> BruteForceSpecific(T& distribution, long long start, long long end, long long step);

		void Summarize();
		static void LogSummary();
		static void AverageAllRedos(int redoStart, int redoEnd);

		long long GetDeltaOpt();

		void LogCDF();
	private:

		static std::vector<SimulationSummaryData> s_Summary;

		Distribution<std::weibull_distribution<double>> m_Weibull;
		Distribution<std::gamma_distribution<double>> m_Gamma;
		Distribution<std::lognormal_distribution<double>> m_Lognormal;

		std::mt19937_64 m_Random;

		SimulationSummaryData m_SummaryData;


		WeibullParameters m_WeibullParams;
		LognormalParameters m_LognormalParams;
		GammaParameters m_GammaParams;

	};
}