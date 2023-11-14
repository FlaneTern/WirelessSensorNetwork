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
		long long Delta;
		long long CollectionTime;
		long long WastedTime;
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
		long long TotalDuration = -1;
		long long TransferTime = -1;
		long long RecoveryTime = -1;

		long long DeltaOpt = -1;

		long long CollectionTimeWeibull = -1;
		long long CollectionTimeGamma = -1;
		long long CollectionTimeLognormal = -1;

		long long WastedTimeWeibull = -1;
		long long WastedTimeGamma = -1;
		long long WastedTimeLognormal = -1;


		long long DeltaStarOriWeibull = -1;
		long long DeltaStarOriGamma = -1;
		long long DeltaStarOriLognormal = -1;

		long long CollectionTimeStarOriWeibull = -1;
		long long CollectionTimeStarOriGamma = -1;
		long long CollectionTimeStarOriLognormal = -1;

		long long WastedTimeStarOriWeibull = -1;
		long long WastedTimeStarOriGamma = -1;
		long long WastedTimeStarOriLognormal = -1;


		long long DeltaStarMAWeibull = -1;
		long long DeltaStarMAGamma = -1;
		long long DeltaStarMALognormal = -1;

		long long CollectionTimeStarMAWeibull = -1;
		long long CollectionTimeStarMAGamma = -1;
		long long CollectionTimeStarMALognormal = -1;

		long long WastedTimeStarMAWeibull = -1;
		long long WastedTimeStarMAGamma = -1;
		long long WastedTimeStarMALognormal = -1;

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

		void GenerateFailures(std::mt19937& random, long long totalDuration);

		T m_Distribution;
		std::vector<long long> m_FailurePoints;
	};

	class Simulation
	{
	public:
		Simulation(long long totalDuration, long long transferTime, long long recoveryTime, long long mean, long long stddev, long long redo);

		void SimulateAll();

		template<typename T>
		SimulationData SimulateSpecific(T& distribution);


		void BruteForceAll(long long start, long long end, long long step);

		template<typename T>
		std::vector<BruteForceData> BruteForceSpecific(T& distribution, long long start, long long end, long long step);

		void Summarize();
		static void LogSummary();

	private:

		static std::vector<SimulationSummaryData> s_Summary;

		Distribution<std::weibull_distribution<double>> m_Weibull;
		Distribution<std::gamma_distribution<double>> m_Gamma;
		Distribution<std::lognormal_distribution<double>> m_Lognormal;

		std::mt19937 m_Random;

		SimulationSummaryData m_SummaryData;


		WeibullParameters m_WeibullParams;
		LognormalParameters m_LognormalParams;
		GammaParameters m_GammaParams;

	};
}