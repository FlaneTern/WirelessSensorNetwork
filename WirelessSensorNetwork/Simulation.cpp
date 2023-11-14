#include "WSNPCH.h"
#include "Simulation.h"


namespace WSN
{
	static std::string SimulationIntervalToJsonString(SimulationInterval si, long long pid)
	{
		std::string name;
		switch (si.State)
		{
		case State::Collection: name = "Collection"; break;
		case State::Transfer: name = "Transfer"; break;
		case State::Recovery: name = "Recovery"; break;
		}

		return "{ \"cat\" : \"Node\", \"pid\" :" + std::to_string(pid) + ",\"tid\" :" + std::to_string(pid) 
			+ ",\"dur\" :" + std::to_string((si.EndTime - si.StartTime) * 1000000) + ",\"ts\" :" + std::to_string(si.StartTime * 1000000)
			+ ",\"ph\" : \"X\",\"name\" :\"" + name + "\"}";
	}

	static std::string GetModeString(State s)
	{
		switch (s)
		{
		case State::Transfer: return "Transfer";
		case State::Collection: return "Collection";
		case State::Recovery: return "Recovery";
		}

		return "";
	}

	static std::vector<BruteForceData> SimpleMovingAverage(const std::vector<BruteForceData>& bfData,
		long long deltaStep, long long halfRange)
	{
		std::vector<BruteForceData> maData;
		maData.reserve(bfData.size());

		double CTSum = 0.0;
		long long count = 0;

		for (int i = 0; i < halfRange - 1; i++)
		{
			CTSum += bfData[i].CollectionTime;
			count++;
		}



		for (int i = 0; i < bfData.size(); i++)
		{
			if (i - halfRange >= 0)
			{
				CTSum -= bfData[i - halfRange].CollectionTime;
				count--;
			}

			if (i + halfRange < bfData.size())
			{
				CTSum += bfData[i + halfRange].CollectionTime;
				count++;
			}

			maData.push_back({ bfData[i].Delta, (long long)(CTSum / count) });
		}

		return maData;

	}

	static std::vector<BruteForceData> WeightedMovingAverage(const std::vector<BruteForceData>& bfData,
		long long deltaStep, long long halfRange)
	{
		std::vector<BruteForceData> maData;
		maData.reserve(bfData.size());

		for (int i = 0; i < bfData.size(); i++)
		{
			double CTSum = 0.0;
			double WTSum = 0.0;
			long long count = 0;

			for (int j = -halfRange, k = 0; j <= halfRange; j++)
			{
				if (j <= 0)
					k++;
				else
					k--;

				if (i + j >= 0 && i + j < bfData.size())
				{
					CTSum += bfData[i + j].CollectionTime * k;
					WTSum += bfData[i + j].WastedTime * k;
					count += k;
				}
			}

			maData.push_back({ bfData[i].Delta, (long long)(CTSum / count), (long long)(WTSum / count) });
		}

		return maData;
	}

	static std::tuple<long long, long long, long long> FindDeltaStar(const std::vector<BruteForceData>& maData)
	{
		long long bestDelta = maData[0].Delta;
		long long bestCT = maData[0].CollectionTime;
		long long bestWT = maData[0].WastedTime;

		for (int i = 1; i < maData.size(); i++)
		{
			if (maData[i].WastedTime < bestWT)
			{
				bestDelta = maData[i].Delta;
				bestCT = maData[i].CollectionTime;
				bestWT = maData[i].WastedTime;
			}
		}

		return { bestDelta, bestCT, bestWT };
	}

	std::vector<SimulationSummaryData> Simulation::s_Summary;

	Simulation::Simulation(long long totalDuration, long long transferTime, long long recoveryTime, long long mean, long long stddev, long long redo)
		: m_WeibullParams(mean, stddev), m_LognormalParams(mean, stddev), m_GammaParams(mean, stddev), 
		  m_Random(std::chrono::system_clock::now().time_since_epoch().count())
	{
		m_SummaryData =
		{
			mean,
			stddev,
			totalDuration,
			transferTime,
			recoveryTime,
			(long long)std::sqrt(2 * mean * transferTime)
		};
		m_SummaryData.RedoCount = redo;

		m_Weibull = Distribution<std::weibull_distribution<double>>(m_WeibullParams.Shape, m_WeibullParams.Scale);
		m_Gamma = Distribution<std::gamma_distribution<double>>(m_GammaParams.Shape, m_GammaParams.Scale);
		m_Lognormal = Distribution<std::lognormal_distribution<double>>(m_LognormalParams.M, m_LognormalParams.S);

		if (!std::filesystem::is_directory("Results") || !std::filesystem::exists("Results"))
			std::filesystem::create_directory("Results");


		if (!std::filesystem::is_directory("Results/Redo" + std::to_string(m_SummaryData.RedoCount)) || !std::filesystem::exists("Results/Redo" + std::to_string(m_SummaryData.RedoCount)))
			std::filesystem::create_directory("Results/Redo" + std::to_string(m_SummaryData.RedoCount));

		m_Weibull.GenerateFailures(m_Random, totalDuration);
		m_Gamma.GenerateFailures(m_Random, totalDuration);
		m_Lognormal.GenerateFailures(m_Random, totalDuration);
	}



	template<typename T>
	SimulationData Simulation::SimulateSpecific(T& distribution)
	{
		SimulationData simulationData;
		simulationData.BruteForceData.Delta = m_SummaryData.DeltaOpt;
		simulationData.BruteForceData.CollectionTime = 0;
		long long currentTime = 0;
		long long nextFailureTime = distribution.m_FailurePoints[0];
		long long failureIterator = 0;
		bool failed = false;
		bool finishFailures = false;

		State currentState = State::Collection;

		while (currentTime < m_SummaryData.TotalDuration)
		{
			if (failed)
			{
				failureIterator++;
				if (failureIterator >= distribution.m_FailurePoints.size())
					finishFailures = true;
				else
				{
					nextFailureTime = distribution.m_FailurePoints[failureIterator];
				}
				failed = false;
			}

			if (currentState == State::Collection)
			{
				long long nextTime = currentTime + m_SummaryData.DeltaOpt;
				if (nextTime > nextFailureTime && !finishFailures)
				{
					failed = true;
					simulationData.SimulationIntervals.push_back({ State::Collection, currentTime, nextFailureTime });
					currentState = State::Recovery;
					currentTime = nextFailureTime;
				}
				else
				{
					simulationData.SimulationIntervals.push_back({ State::Collection, currentTime, nextTime });
					currentState = State::Transfer;
					currentTime = nextTime;
					simulationData.BruteForceData.CollectionTime += m_SummaryData.DeltaOpt;
				}
			}
			else if (currentState == State::Transfer)
			{
				long long nextTime = currentTime + m_SummaryData.TransferTime;
				if (nextTime > nextFailureTime && !finishFailures)
				{
					failed = true;
					simulationData.SimulationIntervals.push_back({ State::Transfer, currentTime, nextFailureTime });
					currentState = State::Recovery;
					currentTime = nextFailureTime;
					simulationData.BruteForceData.CollectionTime -= m_SummaryData.DeltaOpt;
				}
				else
				{
					simulationData.SimulationIntervals.push_back({ State::Transfer, currentTime, nextTime });
					currentState = State::Collection;
					currentTime = nextTime;
				}
			}
			else
			{
				long long nextTime = currentTime + m_SummaryData.RecoveryTime;
				if (nextTime > nextFailureTime && !finishFailures)
				{
					failed = true;
					simulationData.SimulationIntervals.push_back({ State::Recovery, currentTime, nextFailureTime });
					currentState = State::Recovery;
					currentTime = nextFailureTime;
				}
				else
				{
					simulationData.SimulationIntervals.push_back({ State::Recovery, currentTime, nextTime });
					currentState = State::Collection;
					currentTime = nextTime;
				}
			}

		}

		simulationData.BruteForceData.WastedTime = simulationData.SimulationIntervals.back().EndTime - simulationData.BruteForceData.CollectionTime;

		return simulationData;
	}



	void Simulation::SimulateAll()
	{

		SimulationData DataWeibull = SimulateSpecific(m_Weibull);
		SimulationData DataGamma = SimulateSpecific(m_Gamma);
		SimulationData DataLognormal = SimulateSpecific(m_Lognormal);

		m_SummaryData.CollectionTimeWeibull = DataWeibull.BruteForceData.CollectionTime;
		m_SummaryData.CollectionTimeGamma = DataGamma.BruteForceData.CollectionTime;
		m_SummaryData.CollectionTimeLognormal = DataLognormal.BruteForceData.CollectionTime;
		m_SummaryData.WastedTimeWeibull = DataWeibull.BruteForceData.WastedTime;
		m_SummaryData.WastedTimeGamma = DataGamma.BruteForceData.WastedTime;
		m_SummaryData.WastedTimeLognormal = DataLognormal.BruteForceData.WastedTime;


		std::string baseFilePath = "Results/Redo" + std::to_string(m_SummaryData.RedoCount) + "/SimulationM" + std::to_string(m_SummaryData.Mean) + 'S' + std::to_string(m_SummaryData.StdDev)
			+ "DUR" + std::to_string(m_SummaryData.TotalDuration) + 'T' + std::to_string(m_SummaryData.TransferTime) + 'R' + std::to_string(m_SummaryData.RecoveryTime) + ".csv";
		std::ofstream OStream(baseFilePath);

		std::ofstream StreamJson("Results/Redo" + std::to_string(m_SummaryData.RedoCount) + "/SimulationTraceM" + std::to_string(m_SummaryData.Mean) + 'S' + std::to_string(m_SummaryData.StdDev)
			+ "DUR" + std::to_string(m_SummaryData.TotalDuration) + 'T' + std::to_string(m_SummaryData.TransferTime) + 'R' + std::to_string(m_SummaryData.RecoveryTime) + ".json");
		StreamJson << '[';

		OStream << "Delta,Collection Time,Wasted Time,,,Delta,Collection Time,Wasted Time,,,Delta,Collection Time,Wasted Time\n" 
			<< DataWeibull.BruteForceData.Delta << ',' << DataWeibull.BruteForceData.CollectionTime << ',' << DataWeibull.BruteForceData.WastedTime << ",,,"
			<< DataLognormal.BruteForceData.Delta << ',' << DataLognormal.BruteForceData.CollectionTime << ',' << DataLognormal.BruteForceData.WastedTime << ",,,"
			<< DataGamma.BruteForceData.Delta << ',' << DataGamma.BruteForceData.CollectionTime << ',' << DataGamma.BruteForceData.WastedTime << "\n\n\n"
			<< "State,Start Time,End Time,,,State,Start Time,End Time,,,State,Start Time,End Time\n";


		int maxSize = std::max(std::max(DataWeibull.SimulationIntervals.size(), DataLognormal.SimulationIntervals.size()), DataGamma.SimulationIntervals.size());

		for (int i = 0; i < maxSize; i++)
		{
			if (i < DataWeibull.SimulationIntervals.size())
				OStream << GetModeString(DataWeibull.SimulationIntervals[i].State) << ',' << DataWeibull.SimulationIntervals[i].StartTime << ',' << DataWeibull.SimulationIntervals[i].EndTime << ",,,";
			else
				OStream << ",,,,,";

			if (i < DataGamma.SimulationIntervals.size())
				OStream << GetModeString(DataGamma.SimulationIntervals[i].State) << ',' << DataGamma.SimulationIntervals[i].StartTime << ',' << DataGamma.SimulationIntervals[i].EndTime << ",,,";
			else
				OStream << ",,,,,";

			if (i < DataLognormal.SimulationIntervals.size())
				OStream << GetModeString(DataLognormal.SimulationIntervals[i].State) << ',' << DataLognormal.SimulationIntervals[i].StartTime << ',' << DataLognormal.SimulationIntervals[i].EndTime;
			OStream << '\n';
		}


		for (int i = 0; i < DataWeibull.SimulationIntervals.size(); i++)
			StreamJson << SimulationIntervalToJsonString(DataWeibull.SimulationIntervals[i], 1) << ",\n";


		for (int i = 0; i < DataGamma.SimulationIntervals.size(); i++)
			StreamJson << SimulationIntervalToJsonString(DataGamma.SimulationIntervals[i], 2) << ",\n";


		for (int i = 0; i < DataLognormal.SimulationIntervals.size(); i++)
		{
			StreamJson << SimulationIntervalToJsonString(DataLognormal.SimulationIntervals[i], 3);
			if (i != DataLognormal.SimulationIntervals.size() - 1)
				StreamJson << ',';
			StreamJson << '\n';
		}


		StreamJson << ']';

	}


	template<typename T>
	std::vector<BruteForceData> Simulation::BruteForceSpecific(T& distribution, long long start, long long end, long long step)
	{
		std::vector<BruteForceData> BFDatas;

		for (int i = start; i < end; i += step)
		{
			BruteForceData bfData = { i, 0 };

			bool done = false;
			long long currentTime = 0;
			bool failed = false;
			long long failureIterator = 0;
			long long nextFailureTime = distribution.m_FailurePoints[0];
			bool finishFailures = false;

			State currentState = State::Collection;

			while (currentTime < m_SummaryData.TotalDuration)
			{
				if (failed)
				{
					failureIterator++;
					if (failureIterator >= distribution.m_FailurePoints.size())
						finishFailures = true;
					else
					{
						nextFailureTime = distribution.m_FailurePoints[failureIterator];
					}
					failed = false;
				}

				if (currentState == State::Collection)
				{
					long long nextTime = currentTime + i;
					if (nextTime > nextFailureTime && !finishFailures)
					{
						failed = true;
						currentState = State::Recovery;
						currentTime = nextFailureTime;
					}
					else
					{
						currentState = State::Transfer;
						currentTime = nextTime;
						bfData.CollectionTime += i;
					}
				}
				else if (currentState == State::Transfer)
				{
					long long nextTime = currentTime + m_SummaryData.TransferTime;
					if (nextTime > nextFailureTime && !finishFailures)
					{
						failed = true;
						currentState = State::Recovery;
						currentTime = nextFailureTime;
						bfData.CollectionTime -= i;
					}
					else
					{
						currentState = State::Collection;
						currentTime = nextTime;
					}
				}
				else
				{
					long long nextTime = currentTime + m_SummaryData.RecoveryTime;
					if (nextTime > nextFailureTime && !finishFailures)
					{
						failed = true;
						currentState = State::Recovery;
						currentTime = nextFailureTime;
					}
					else
					{
						currentState = State::Collection;
						currentTime = nextTime;
					}
				}

			}
			bfData.WastedTime = currentTime - bfData.CollectionTime;

			BFDatas.push_back(bfData);
		}

		return BFDatas;
	}


	void Simulation::BruteForceAll(long long start, long long end, long long step)
	{
		std::vector<BruteForceData> bfDataWeibull = BruteForceSpecific(m_Weibull, start, end, step);
		std::vector<BruteForceData> bfDataGamma = BruteForceSpecific(m_Gamma, start, end, step);
		std::vector<BruteForceData> bfDataLognormal = BruteForceSpecific(m_Lognormal, start, end, step);

		//std::vector<BruteForceData> maDataWeibull = SimpleMovingAverage(bfDataWeibull, step, 100);
		//std::vector<BruteForceData> maDataGamma = SimpleMovingAverage(bfDataGamma, step, 100);
		//std::vector<BruteForceData> maDataLognormal = SimpleMovingAverage(bfDataLognormal, step, 100);

		std::vector<BruteForceData> maDataWeibull = WeightedMovingAverage(bfDataWeibull, step, 100);
		std::vector<BruteForceData> maDataGamma = WeightedMovingAverage(bfDataGamma, step, 100);
		std::vector<BruteForceData> maDataLognormal = WeightedMovingAverage(bfDataLognormal, step, 100);

		auto maWeibullResults = FindDeltaStar(maDataWeibull);
		auto maGammaResults = FindDeltaStar(maDataGamma);
		auto maLognormalResults = FindDeltaStar(maDataLognormal);

		auto bfWeibullResults = FindDeltaStar(bfDataWeibull);
		auto bfGammaResults = FindDeltaStar(bfDataGamma);
		auto bfLognormalResults = FindDeltaStar(bfDataLognormal);
		
		m_SummaryData.DeltaStarMAWeibull = std::get<0>(maWeibullResults);
		m_SummaryData.DeltaStarMAGamma = std::get<0>(maGammaResults);
		m_SummaryData.DeltaStarMALognormal = std::get<0>(maLognormalResults);

		m_SummaryData.CollectionTimeStarMAWeibull = std::get<1>(maWeibullResults);
		m_SummaryData.CollectionTimeStarMAGamma = std::get<1>(maGammaResults);
		m_SummaryData.CollectionTimeStarMALognormal = std::get<1>(maLognormalResults);

		m_SummaryData.WastedTimeStarMAWeibull = std::get<2>(maWeibullResults);
		m_SummaryData.WastedTimeStarMAGamma = std::get<2>(maGammaResults);
		m_SummaryData.WastedTimeStarMALognormal = std::get<2>(maLognormalResults);

		m_SummaryData.DeltaStarOriWeibull = std::get<0>(bfWeibullResults);
		m_SummaryData.DeltaStarOriGamma = std::get<0>(bfGammaResults);
		m_SummaryData.DeltaStarOriLognormal = std::get<0>(bfLognormalResults);

		m_SummaryData.CollectionTimeStarOriWeibull = std::get<1>(bfWeibullResults);
		m_SummaryData.CollectionTimeStarOriGamma = std::get<1>(bfGammaResults);
		m_SummaryData.CollectionTimeStarOriLognormal = std::get<1>(bfLognormalResults);

		m_SummaryData.WastedTimeStarOriWeibull = std::get<2>(bfWeibullResults);
		m_SummaryData.WastedTimeStarOriGamma = std::get<2>(bfGammaResults);
		m_SummaryData.WastedTimeStarOriLognormal = std::get<2>(bfLognormalResults);



		std::string baseFilePath = "Results/Redo" + std::to_string(m_SummaryData.RedoCount) + "/BruteForceM" + std::to_string(m_SummaryData.Mean) + 'S' + std::to_string(m_SummaryData.StdDev)
			+ "DUR" + std::to_string(m_SummaryData.TotalDuration) + 'T' + std::to_string(m_SummaryData.TransferTime) + 'R' + std::to_string(m_SummaryData.RecoveryTime) + ".csv";

		std::ofstream OStream(baseFilePath);

		OStream << "Weibull Distribution,,,,,Gamma Distribution,,,,,Lognormal Distribution,,,,,Weibull Distribution,,,,,Gamma Distribution,,,,,Lognormal Distribution\n"
			<< m_SummaryData.DeltaStarOriWeibull << ",,,,," << m_SummaryData.DeltaStarOriGamma << ",,,,," << m_SummaryData.DeltaStarOriLognormal << ",,,,," << m_SummaryData.DeltaStarMAWeibull << ",,,,," << m_SummaryData.DeltaStarMAGamma << ",,,,," << m_SummaryData.DeltaStarMALognormal << ",,,,," << "\n\n\n"
			<< "Delta,Collection Time,Wasted Time,,,Delta,Collection Time,Wasted Time,,,Delta,Collection Time,Wasted Time,,,Delta,Collection Time,Wasted Time,,,Delta,Collection Time,Wasted Time,,,Delta,Collection Time,Wasted Time\n";

		int maxSize = std::max(std::max(bfDataWeibull.size(), bfDataGamma.size()), bfDataLognormal.size());

		for (int i = 0; i < maxSize; i++)
		{
			if (i < bfDataWeibull.size())
				OStream << bfDataWeibull[i].Delta << ',' << bfDataWeibull[i].CollectionTime << ',' << bfDataWeibull[i].WastedTime << ",,,";
			else
				OStream << ",,,,,";

			if (i < bfDataGamma.size())
				OStream << bfDataGamma[i].Delta << ',' << bfDataGamma[i].CollectionTime << ',' << bfDataGamma[i].WastedTime << ",,,";
			else
				OStream << ",,,,,";

			if (i < bfDataLognormal.size())
				OStream << bfDataLognormal[i].Delta << ',' << bfDataLognormal[i].CollectionTime << ',' << bfDataLognormal[i].WastedTime << ",,,";
			else
				OStream << ",,,,,";


			if (i < maDataWeibull.size())
				OStream << maDataWeibull[i].Delta << ',' << maDataWeibull[i].CollectionTime  << ',' << maDataWeibull[i].WastedTime << ",,,";
			else
				OStream << ",,,,,";

			if (i < maDataGamma.size())
				OStream << maDataGamma[i].Delta << ',' << maDataGamma[i].CollectionTime << ',' << maDataGamma[i].WastedTime << ",,,";
			else
				OStream << ",,,,,";

			if (i < maDataLognormal.size())
				OStream << maDataLognormal[i].Delta << ',' << maDataLognormal[i].CollectionTime << ',' << maDataLognormal[i].WastedTime;
			OStream << '\n';
		}

	}

	void Simulation::Summarize()
	{
		s_Summary.push_back(m_SummaryData);
	}


	void Simulation::LogSummary()
	{
		std::vector<long long> redoLogged;

		for (int i = 0; i < s_Summary.size(); i++)
		{
			auto& current = s_Summary[i];
			std::string path = "Results/Redo" + std::to_string(current.RedoCount) + "/Summary.csv";

			if (std::find(redoLogged.begin(), redoLogged.end(), current.RedoCount) == redoLogged.end())
			{
				redoLogged.push_back(current.RedoCount);
				std::ofstream OStream(path);
				OStream << "Mean,Standard Deviation,Total Duration,Transfer Time,Recovery Time,Delta Optimal,Delta Star Ori Weibull,Delta Star Ori Gamma,Delta Star Ori Lognormal,"
					"Delta Star MA Weibull,Delta Star MA Gamma,Delta Star MA Lognormal,Collection Time Weibull,Collection Time Gamma,Collection Time Lognormal,"
					"Collection Time Star Ori Weibull,Collection Time Star Ori Gamma,Collection Time Star Ori Lognormal,Collection Time Star MA Weibull,Collection Time Star MA Gamma,Collection Time Star MA Lognormal,"
					"Wasted Time Weibull,Wasted Time Gamma,Wasted Time Lognormal," //
					"Wasted Time Star Ori Weibull,Wasted Time Star Ori Gamma,Wasted Time Star Ori Lognormal,Wasted Time Star MA Weibull,Wasted Time Star MA Gamma,Wasted Time Star MA Lognormal," //
					"Redo,Diff Delta Ori Weibull,Diff Delta Ori Gamma,Diff Delta Ori Lognormal,Diff Delta MA Weibull,Diff Delta MA Gamma,Diff Delta MA Lognormal,"
					"Diff CT Ori Weibull,Diff CT Ori Gamma,Diff CT Ori Lognormal,Diff CT MA Weibull,Diff CT MA Gamma,Diff CT MA Lognormal,"
					"Diff WT Ori Weibull,Diff WT Ori Gamma,Diff WT Ori Lognormal,Diff WT MA Weibull,Diff WT MA Gamma,Diff WT MA Lognormal,\n"; //;
			}

			double diffDeltaMAWeibull = (current.DeltaStarMAWeibull - current.DeltaOpt) / (double)current.DeltaOpt;
			double diffDeltaMAGamma = (current.DeltaStarMAGamma - current.DeltaOpt) / (double)current.DeltaOpt;
			double diffDeltaMALognormal = (current.DeltaStarMALognormal - current.DeltaOpt) / (double)current.DeltaOpt;

			// TO DO
			// (opt - star) / star
			double diffDeltaOriWeibull = (current.DeltaStarOriWeibull - current.DeltaOpt) / (double)current.DeltaOpt;
			double diffDeltaOriGamma = (current.DeltaStarOriGamma - current.DeltaOpt) / (double)current.DeltaOpt;
			double diffDeltaOriLognormal = (current.DeltaStarOriLognormal - current.DeltaOpt) / (double)current.DeltaOpt;

			double diffCTMAWeibull = (current.CollectionTimeStarMAWeibull - current.CollectionTimeWeibull) / (double)current.CollectionTimeWeibull;
			double diffCTMAGamma = (current.CollectionTimeStarMAGamma - current.CollectionTimeGamma) / (double)current.CollectionTimeGamma;
			double diffCTMALognormal = (current.CollectionTimeStarMALognormal - current.CollectionTimeLognormal) / (double)current.CollectionTimeLognormal;

			double diffCTOriWeibull = (current.CollectionTimeStarOriWeibull - current.CollectionTimeWeibull) / (double)current.CollectionTimeWeibull;
			double diffCTOriGamma = (current.CollectionTimeStarOriGamma - current.CollectionTimeGamma) / (double)current.CollectionTimeGamma;
			double diffCTOriLognormal = (current.CollectionTimeStarOriLognormal - current.CollectionTimeLognormal) / (double)current.CollectionTimeLognormal;


			double diffWTMAWeibull = (current.WastedTimeWeibull - current.WastedTimeStarMAWeibull) / (double)current.WastedTimeWeibull;
			double diffWTMAGamma = (current.WastedTimeGamma - current.WastedTimeStarMAGamma) / (double)current.WastedTimeGamma;
			double diffWTMALognormal = (current.WastedTimeLognormal - current.WastedTimeStarMALognormal) / (double)current.WastedTimeLognormal;

			double diffWTOriWeibull = (current.WastedTimeWeibull - current.WastedTimeStarOriWeibull) / (double)current.WastedTimeWeibull;
			double diffWTOriGamma = (current.WastedTimeGamma - current.WastedTimeStarOriGamma) / (double)current.WastedTimeGamma;
			double diffWTOriLognormal = (current.WastedTimeLognormal - current.WastedTimeStarOriLognormal) / (double)current.WastedTimeLognormal;



			std::ofstream OStream(path, std::fstream::app);
			OStream << current.Mean << ',' << current.StdDev << ',' << current.TotalDuration << ',' << current.TransferTime << ',' << current.RecoveryTime << ',' << current.DeltaOpt << ','
				<< current.DeltaStarOriWeibull << ',' << current.DeltaStarOriGamma << ',' << current.DeltaStarOriLognormal << ',' << current.DeltaStarMAWeibull << ',' << current.DeltaStarMAGamma << ',' << current.DeltaStarMALognormal << ','
				<< current.CollectionTimeWeibull << ',' << current.CollectionTimeGamma << ',' << current.CollectionTimeLognormal << ',' 
				<< current.CollectionTimeStarOriWeibull << ',' << current.CollectionTimeStarOriGamma << ',' << current.CollectionTimeStarOriLognormal << ',' 
				<< current.CollectionTimeStarMAWeibull << ',' << current.CollectionTimeStarMAGamma << ',' << current.CollectionTimeStarMALognormal << ','
				<< current.WastedTimeWeibull << ',' << current.WastedTimeGamma << ',' << current.WastedTimeLognormal << ','
				<< current.WastedTimeStarOriWeibull << ',' << current.WastedTimeStarOriGamma << ',' << current.WastedTimeStarOriLognormal << ','
				<< current.WastedTimeStarMAWeibull << ',' << current.WastedTimeStarMAGamma << ',' << current.WastedTimeStarMALognormal << ','
				<< current.RedoCount << ','
				<< diffDeltaOriWeibull << ',' << diffDeltaOriGamma << ',' << diffDeltaOriLognormal << ',' << diffDeltaMAWeibull << ',' << diffDeltaMAGamma << ',' << diffDeltaMALognormal << ','
				<< diffCTOriWeibull << ',' << diffCTOriGamma << ',' << diffCTOriLognormal << ',' << diffCTMAWeibull << ',' << diffCTMAGamma << ',' << diffCTMALognormal << ','
				<< diffWTOriWeibull << ',' << diffWTOriGamma << ',' << diffWTOriLognormal << ',' << diffWTMAWeibull << ',' << diffWTMAGamma << ',' << diffWTMALognormal
				<< '\n';

		}


	}

	template<typename T>
	void Distribution<T>::GenerateFailures(std::mt19937& random, long long totalDuration)
	{
		long long currentFailure = 0;
		
		while (currentFailure < totalDuration)
		{
			currentFailure += m_Distribution(random);
			m_FailurePoints.push_back(currentFailure);
		}
	}
}