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

	static std::tuple<long long, long long, long long, long long, long long> FindDeltaStar(const std::vector<BruteForceData>& maData)
	{
		long long bestDelta = maData[0].Delta;
		long long bestCT = maData[0].CollectionTime;
		long long bestWT = maData[0].WastedTime;
		long long bestTotalDuration = maData[0].WastedTime;
		long long bestFinalFailureIndex = maData[0].FinalFailureIndex;

		for (int i = 1; i < maData.size(); i++)
		{
			if (maData[i].WastedTime < bestWT)
			{
				bestDelta = maData[i].Delta;
				bestCT = maData[i].CollectionTime;
				bestWT = maData[i].WastedTime;
				bestTotalDuration = maData[i].ActualTotalDuration;
				bestFinalFailureIndex = maData[i].FinalFailureIndex;
			}
		}

		return { bestDelta, bestCT, bestWT, bestTotalDuration, bestFinalFailureIndex };
	}

	std::vector<SimulationSummaryData> Simulation::s_Summary;

	Simulation::Simulation(long long totalDurationToBeTransferred, long long transferTime, long long recoveryTime, long long mean, long long stddev, long long redo)
		: m_WeibullParams(mean, stddev), m_LognormalParams(mean, stddev), m_GammaParams(mean, stddev)
	{
		m_SummaryData =
		{
			mean,
			stddev,
			totalDurationToBeTransferred,
			transferTime,
			recoveryTime,
			(long long)std::sqrt(2 * mean * transferTime)
		};
		m_SummaryData.RedoCount = redo;

		std::cout << "Weibull K = " << m_WeibullParams.Shape << ", Weibull Lambda = " << m_WeibullParams.Scale
			<< ", Gamma K = " << m_GammaParams.Shape << ", Gamma Theta = " << m_GammaParams.Scale
			<< ", Lognormal Mu = " << m_LognormalParams.M << ", Lognormal Sigma = " << m_LognormalParams.S
			<< "\n\n";
		m_Weibull = Distribution<std::weibull_distribution<double>>(m_WeibullParams.Shape, m_WeibullParams.Scale);
		m_Gamma = Distribution<std::gamma_distribution<double>>(m_GammaParams.Shape, m_GammaParams.Scale);
		m_Lognormal = Distribution<std::lognormal_distribution<double>>(m_LognormalParams.M, m_LognormalParams.S);

		if (!std::filesystem::is_directory("Results") || !std::filesystem::exists("Results"))
			std::filesystem::create_directory("Results");


		if (!std::filesystem::is_directory("Results/Redo" + std::to_string(m_SummaryData.RedoCount)) || !std::filesystem::exists("Results/Redo" + std::to_string(m_SummaryData.RedoCount)))
			std::filesystem::create_directory("Results/Redo" + std::to_string(m_SummaryData.RedoCount));

		m_Random = std::mt19937_64(std::chrono::high_resolution_clock::now().time_since_epoch().count());

		"Results/Redo" + std::to_string(m_SummaryData.RedoCount) + "/SimulationM" + std::to_string(m_SummaryData.Mean) + 'S' + std::to_string(m_SummaryData.StdDev)
			+ "DUR" + std::to_string(m_SummaryData.TotalDurationToBeTransferred) + 'T' + std::to_string(m_SummaryData.TransferTime) + 'R' + std::to_string(m_SummaryData.RecoveryTime) + ".csv";


		static constexpr double failGenerationDurationMultiplier = 100.0;

		// std::cout << "Weibull : ";
		m_Weibull.GenerateFailures(m_Random, failGenerationDurationMultiplier * totalDurationToBeTransferred);
		// std::cout << "Gamma : ";
		m_Gamma.GenerateFailures(m_Random, failGenerationDurationMultiplier * totalDurationToBeTransferred);
		// std::cout << "Lognormal : ";
		m_Lognormal.GenerateFailures(m_Random, failGenerationDurationMultiplier * totalDurationToBeTransferred);
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
		long long transferredTotalDuration = 0;

		State currentState = State::Collection;

		while (transferredTotalDuration < m_SummaryData.TotalDurationToBeTransferred)
		{
			if (failed)
			{
				failureIterator++;
				if (failureIterator < distribution.m_FailurePoints.size())
					nextFailureTime = distribution.m_FailurePoints[failureIterator];
				else
					throw std::runtime_error("Exceeded the last failure point!");;

				failed = false;
			}

			if (currentState == State::Collection)
			{
				long long nextTime = currentTime + m_SummaryData.DeltaOpt;
				if (nextTime > nextFailureTime)
				{
					failed = true;
					simulationData.SimulationIntervals.push_back({ State::Collection, currentTime, nextFailureTime });
					currentState = State::Recovery;
					simulationData.BruteForceData.WastedTime += nextFailureTime - currentTime;
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
				if (nextTime > nextFailureTime)
				{
					failed = true;
					simulationData.SimulationIntervals.push_back({ State::Transfer, currentTime, nextFailureTime });
					currentState = State::Recovery;

					double proportionSent = (double)(nextFailureTime - currentTime) / m_SummaryData.TransferTime;
					simulationData.BruteForceData.WastedTime += m_SummaryData.DeltaOpt + nextFailureTime - currentTime;
					currentTime = nextFailureTime;
					simulationData.BruteForceData.CollectionTime -= m_SummaryData.DeltaOpt;
				}
				else
				{
					simulationData.SimulationIntervals.push_back({ State::Transfer, currentTime, nextTime });
					currentState = State::Collection;
					currentTime = nextTime;
					simulationData.BruteForceData.WastedTime += m_SummaryData.TransferTime;
					transferredTotalDuration += m_SummaryData.DeltaOpt;
				}
			}
			else
			{
				long long nextTime = currentTime + m_SummaryData.RecoveryTime;
				if (nextTime > nextFailureTime)
				{
					failed = true;
					simulationData.SimulationIntervals.push_back({ State::Recovery, currentTime, nextFailureTime });
					currentState = State::Recovery;
					simulationData.BruteForceData.WastedTime += nextFailureTime - currentTime;
					currentTime = nextFailureTime;
				}
				else
				{
					simulationData.SimulationIntervals.push_back({ State::Recovery, currentTime, nextTime });
					currentState = State::Collection;
					currentTime = nextTime;
					simulationData.BruteForceData.WastedTime += m_SummaryData.RecoveryTime;
				}
			}

		}
		simulationData.BruteForceData.ActualTotalDuration = currentTime;
		simulationData.BruteForceData.FinalFailureIndex = failureIterator - 1;

		//std::cout << "Last Failure Index = " << failureIterator << '\n';
		//simulationData.BruteForceData.WastedTime = simulationData.SimulationIntervals.back().EndTime - simulationData.BruteForceData.CollectionTime;

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

		m_SummaryData.ActualTotalDurationWeibull = DataWeibull.BruteForceData.ActualTotalDuration;
		m_SummaryData.ActualTotalDurationGamma = DataGamma.BruteForceData.ActualTotalDuration;
		m_SummaryData.ActualTotalDurationLognormal = DataLognormal.BruteForceData.ActualTotalDuration;

		m_SummaryData.FinalFailureIndexWeibull = DataWeibull.BruteForceData.FinalFailureIndex;
		m_SummaryData.FinalFailureIndexGamma = DataGamma.BruteForceData.FinalFailureIndex;
		m_SummaryData.FinalFailureIndexLognormal = DataLognormal.BruteForceData.FinalFailureIndex;


		std::string baseFilePath = "Results/Redo" + std::to_string(m_SummaryData.RedoCount) + "/SimulationM" + std::to_string(m_SummaryData.Mean) + 'S' + std::to_string(m_SummaryData.StdDev)
			+ "DUR" + std::to_string(m_SummaryData.TotalDurationToBeTransferred) + 'T' + std::to_string(m_SummaryData.TransferTime) + 'R' + std::to_string(m_SummaryData.RecoveryTime) + ".csv";
		std::ofstream OStream(baseFilePath);

		std::ofstream StreamJson("Results/Redo" + std::to_string(m_SummaryData.RedoCount) + "/SimulationTraceM" + std::to_string(m_SummaryData.Mean) + 'S' + std::to_string(m_SummaryData.StdDev)
			+ "DUR" + std::to_string(m_SummaryData.TotalDurationToBeTransferred) + 'T' + std::to_string(m_SummaryData.TransferTime) + 'R' + std::to_string(m_SummaryData.RecoveryTime) + ".json");
		StreamJson << '[';

		OStream << "Delta,Collection Time,Wasted Time,Total Actual Duration,,Delta,Collection Time,Wasted Time,Total Actual Duration,,Delta,Collection Time,Wasted Time,Total Actual Duration\n" 
			<< DataWeibull.BruteForceData.Delta << ',' << DataWeibull.BruteForceData.CollectionTime << ',' << DataWeibull.BruteForceData.WastedTime << ',' << DataWeibull.BruteForceData.ActualTotalDuration << ",,"
			<< DataLognormal.BruteForceData.Delta << ',' << DataLognormal.BruteForceData.CollectionTime << ',' << DataLognormal.BruteForceData.WastedTime << ',' << DataLognormal.BruteForceData.ActualTotalDuration << ",,"
			<< DataGamma.BruteForceData.Delta << ',' << DataGamma.BruteForceData.CollectionTime << ',' << DataGamma.BruteForceData.WastedTime << ',' << DataGamma.BruteForceData.ActualTotalDuration << "\n\n\n"
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
			long long transferredTotalDuration = 0;

			State currentState = State::Collection;

			while (transferredTotalDuration < m_SummaryData.TotalDurationToBeTransferred)
			{
				if (failed)
				{
					failureIterator++;
					if (failureIterator < distribution.m_FailurePoints.size())
						nextFailureTime = distribution.m_FailurePoints[failureIterator];
					else
						throw std::runtime_error("Exceeded the last failure point!");


					
					failed = false;
				}

				if (currentState == State::Collection)
				{
					long long nextTime = currentTime + i;
					if (nextTime > nextFailureTime)
					{
						failed = true;
						currentState = State::Recovery;
						bfData.WastedTime += nextFailureTime - currentTime;
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
					if (nextTime > nextFailureTime)
					{
						failed = true;
						currentState = State::Recovery;
						double proportionSent = (double)(nextFailureTime - currentTime) / m_SummaryData.TransferTime;
						bfData.WastedTime += i + nextFailureTime - currentTime;
						currentTime = nextFailureTime;
						bfData.CollectionTime -= i;
					}
					else
					{
						currentState = State::Collection;
						currentTime = nextTime;
						bfData.WastedTime += m_SummaryData.TransferTime;
						transferredTotalDuration += i;
					}
				}
				else
				{
					long long nextTime = currentTime + m_SummaryData.RecoveryTime;
					if (nextTime > nextFailureTime)
					{
						failed = true;
						currentState = State::Recovery;
						bfData.WastedTime += nextFailureTime - currentTime;
						currentTime = nextFailureTime;
					}
					else
					{
						currentState = State::Collection;
						currentTime = nextTime;
						bfData.WastedTime += m_SummaryData.RecoveryTime;
					}
				}

			}
			// bfData.WastedTime = currentTime - bfData.CollectionTime;
			// std::cout << "Last Failure Index = " << failureIterator << '\n';
			bfData.ActualTotalDuration = currentTime;
			bfData.FinalFailureIndex = failureIterator - 1;
			BFDatas.push_back(bfData);
		}

		return BFDatas;
	}


	void Simulation::BruteForceAll(long long start, long long end, long long step)
	{
		std::vector<BruteForceData> bfDataWeibull = BruteForceSpecific(m_Weibull, start, end, step);
		std::vector<BruteForceData> bfDataGamma = BruteForceSpecific(m_Gamma, start, end, step);
		std::vector<BruteForceData> bfDataLognormal = BruteForceSpecific(m_Lognormal, start, end, step);

		auto bfWeibullResults = FindDeltaStar(bfDataWeibull);
		auto bfGammaResults = FindDeltaStar(bfDataGamma);
		auto bfLognormalResults = FindDeltaStar(bfDataLognormal);
		
		m_SummaryData.DeltaStarWeibull = std::get<0>(bfWeibullResults);
		m_SummaryData.DeltaStarGamma = std::get<0>(bfGammaResults);
		m_SummaryData.DeltaStarLognormal = std::get<0>(bfLognormalResults);

		m_SummaryData.CollectionTimeStarWeibull = std::get<1>(bfWeibullResults);
		m_SummaryData.CollectionTimeStarGamma = std::get<1>(bfGammaResults);
		m_SummaryData.CollectionTimeStarLognormal = std::get<1>(bfLognormalResults);

		m_SummaryData.WastedTimeStarWeibull = std::get<2>(bfWeibullResults);
		m_SummaryData.WastedTimeStarGamma = std::get<2>(bfGammaResults);
		m_SummaryData.WastedTimeStarLognormal = std::get<2>(bfLognormalResults);

		m_SummaryData.ActualTotalDurationStarWeibull = std::get<3>(bfWeibullResults);
		m_SummaryData.ActualTotalDurationStarGamma = std::get<3>(bfGammaResults);
		m_SummaryData.ActualTotalDurationStarLognormal = std::get<3>(bfLognormalResults);


		m_SummaryData.FinalFailureIndexStarWeibull = std::get<4>(bfWeibullResults);
		m_SummaryData.FinalFailureIndexStarGamma = std::get<4>(bfGammaResults);
		m_SummaryData.FinalFailureIndexStarLognormal = std::get<4>(bfLognormalResults);


		std::string baseFilePath = "Results/Redo" + std::to_string(m_SummaryData.RedoCount) + "/BruteForceM" + std::to_string(m_SummaryData.Mean) + 'S' + std::to_string(m_SummaryData.StdDev)
			+ "DUR" + std::to_string(m_SummaryData.TotalDurationToBeTransferred) + 'T' + std::to_string(m_SummaryData.TransferTime) + 'R' + std::to_string(m_SummaryData.RecoveryTime) + ".csv";

		std::ofstream OStream(baseFilePath);

		OStream << "Weibull Distribution,,,,,Gamma Distribution,,,,,Lognormal Distribution\n"
			<< m_SummaryData.DeltaStarWeibull << ",,,,," << m_SummaryData.DeltaStarGamma << ",,,,," << m_SummaryData.DeltaStarLognormal << "\n\n\n"
			<< "Delta,Collection Time,Wasted Time,Total Actual Duration,,Delta,Collection Time,Wasted Time,Total Actual Duration,,Delta,Collection Time,Wasted Time,Total Actual Duration\n";

		int maxSize = std::max(std::max(bfDataWeibull.size(), bfDataGamma.size()), bfDataLognormal.size());

		for (int i = 0; i < maxSize; i++)
		{
			if (i < bfDataWeibull.size())
				OStream << bfDataWeibull[i].Delta << ',' << bfDataWeibull[i].CollectionTime << ',' << bfDataWeibull[i].WastedTime << ',' << bfDataWeibull[i].ActualTotalDuration << ",,";
			else
				OStream << ",,,,,";

			if (i < bfDataGamma.size())
				OStream << bfDataGamma[i].Delta << ',' << bfDataGamma[i].CollectionTime << ',' << bfDataGamma[i].WastedTime << ',' << bfDataGamma[i].ActualTotalDuration << ",,";
			else
				OStream << ",,,,,";

			if (i < bfDataLognormal.size())
				OStream << bfDataLognormal[i].Delta << ',' << bfDataLognormal[i].CollectionTime << ',' << bfDataLognormal[i].WastedTime << ',' << bfDataLognormal[i].ActualTotalDuration << ",,";
			else
				OStream << ",,,,,";


			OStream << '\n';
		}

	}

	void Simulation::Summarize()
	{
		s_Summary.push_back(m_SummaryData);
	}


	void Simulation::LogSummary()
	{
		
		static std::vector<long long> redoLogged;

		for (int i = 0; i < s_Summary.size(); i++)
		{
			auto& current = s_Summary[i];
			std::string path = "Results/Redo" + std::to_string(current.RedoCount) + "/Summary.csv";

			if (std::find(redoLogged.begin(), redoLogged.end(), current.RedoCount) == redoLogged.end())
			{
				redoLogged.push_back(current.RedoCount);
				std::ofstream OStream(path);
				OStream << "Mean,Standard Deviation,Total Duration To Be Transferred,Transfer Time,Recovery Time,Delta Optimal,Delta Star Weibull,Delta Star Gamma,Delta Star Lognormal,"
					"Collection Time Weibull,Collection Time Gamma,Collection Time Lognormal,"
					"Collection Time Star Weibull,Collection Time Star Gamma,Collection Time Star Lognormal,"
					"Wasted Time Weibull,Wasted Time Gamma,Wasted Time Lognormal," //
					"Wasted Time Star Weibull,Wasted Time Star Gamma,Wasted Time Star Lognormal," //
					"Actual Total Duration Weibull,Actual Total Duration Gamma,Actual Total Duration Lognormal,"
					"Actual Total Duration Star Weibull,Actual Total Duration Star Gamma,Actual Total Duration Star Lognormal,"
					"Total Number Of Failures Weibull,Total Number Of Failures Gamma, Total Number Of Failures Lognormal,"
					"Total Number Of Failures Star Weibull,Total Number Of Failures Star Gamma, Total Number Of Failures Star Lognormal," //;
					"Redo,Diff Delta Weibull,Diff Delta Gamma,Diff Delta Lognormal,"
					"Diff CT Weibull,Diff CT Gamma,Diff CT Lognormal,"
					"Diff WT Weibull,Diff WT Gamma,Diff WT Lognormal\n";
			}


			// TO DO
			// (opt - star) / star
			double diffDeltaWeibull = (current.DeltaOpt - current.DeltaStarWeibull) / (double)current.DeltaStarWeibull;
			double diffDeltaGamma = (current.DeltaOpt - current.DeltaStarGamma) / (double)current.DeltaStarGamma;
			double diffDeltaLognormal = (current.DeltaOpt - current.DeltaStarLognormal) / (double)current.DeltaStarLognormal;

			double diffCTWeibull = (current.CollectionTimeStarWeibull - current.CollectionTimeWeibull) / (double)current.CollectionTimeStarWeibull;
			double diffCTGamma = (current.CollectionTimeStarGamma - current.CollectionTimeGamma) / (double)current.CollectionTimeStarGamma;
			double diffCTLognormal = (current.CollectionTimeStarLognormal - current.CollectionTimeLognormal) / (double)current.CollectionTimeStarLognormal;


			double diffWTWeibull = (current.WastedTimeWeibull - current.WastedTimeStarWeibull) / (double)current.WastedTimeStarWeibull;
			double diffWTGamma = (current.WastedTimeGamma - current.WastedTimeStarGamma) / (double)current.WastedTimeStarGamma;
			double diffWTLognormal = (current.WastedTimeLognormal - current.WastedTimeStarLognormal) / (double)current.WastedTimeStarLognormal;



			std::ofstream OStream(path, std::fstream::app);
			OStream << current.Mean << ',' << current.StdDev << ',' << current.TotalDurationToBeTransferred << ',' << current.TransferTime << ',' << current.RecoveryTime << ',' << current.DeltaOpt << ','
				<< current.DeltaStarWeibull << ',' << current.DeltaStarGamma << ',' << current.DeltaStarLognormal << ','
				<< current.CollectionTimeWeibull << ',' << current.CollectionTimeGamma << ',' << current.CollectionTimeLognormal << ',' 
				<< current.CollectionTimeStarWeibull << ',' << current.CollectionTimeStarGamma << ',' << current.CollectionTimeStarLognormal << ',' 
				<< current.WastedTimeWeibull << ',' << current.WastedTimeGamma << ',' << current.WastedTimeLognormal << ','
				<< current.WastedTimeStarWeibull << ',' << current.WastedTimeStarGamma << ',' << current.WastedTimeStarLognormal << ','
				<< current.ActualTotalDurationWeibull << ',' << current.ActualTotalDurationGamma << ',' << current.ActualTotalDurationLognormal << ','
				<< current.ActualTotalDurationStarWeibull << ',' << current.ActualTotalDurationStarGamma << ',' << current.ActualTotalDurationStarLognormal << ','
				<< current.FinalFailureIndexWeibull << ',' << current.FinalFailureIndexGamma << ',' << current.FinalFailureIndexLognormal << ','
				<< current.FinalFailureIndexStarWeibull << ',' << current.FinalFailureIndexStarGamma << ',' << current.FinalFailureIndexStarLognormal << ','
				<< current.RedoCount << ','
				<< diffDeltaWeibull << ',' << diffDeltaGamma << ',' << diffDeltaLognormal << ',' 
				<< diffCTWeibull << ',' << diffCTGamma << ',' << diffCTLognormal << ',' 
				<< diffWTWeibull << ',' << diffWTGamma << ',' << diffWTLognormal << ',' 
				<< '\n';

		}

		s_Summary.clear();

	}

	void Simulation::AverageAllRedos(int redoStart, int redoEnd)
	{
		std::vector<std::vector<std::vector<double>>> datas;
		for (int r = redoStart; r <= redoEnd; r++)
		{
			datas.push_back({});
			std::string path = "Results/Redo" + std::to_string(r) + "/Summary.csv";
			std::ifstream summaryStream(path);
			std::string temp;
			std::getline(summaryStream, temp);

			while (std::getline(summaryStream, temp))
			{
				datas.back().push_back({});
				
				int prevStart = 0;
				for (int s = 0; s < temp.size(); s++)
				{
					if (temp[s] == ',' || s == temp.size() - 1)
					{
						double t = std::stod(temp.substr(prevStart, s - prevStart));
						datas.back().back().push_back(t);
						prevStart = s + 1;
					}
				}
			}
		}


		// DANGER ZONE !
		// 33rd index column = redo
		std::vector<std::vector<std::string>> averages(datas[0].size(), std::vector<std::string>(datas[0][0].size()));

		for (int i = 0; i < datas[0].size(); i++)
		{
			for (int j = 0; j < datas[0][0].size(); j++)
			{
				if (j < 33)
				{
					double average = 0.0;
					for (int k = 0; k < datas.size(); k++)
						average += datas[k][i][j] / datas.size();

					averages[i][j] = std::to_string(average);
				}
				else if (j == 33)
					averages[i][j] = std::to_string(redoStart) + "-" + std::to_string(redoEnd);
				else if (j < 37)
					averages[i][j] = std::to_string((std::stod(averages[i][5]) - std::stod(averages[i][j - 28])) / std::stod(averages[i][j - 28]));
				else if (j < 40)
					averages[i][j] = std::to_string((std::stod(averages[i][j - 28]) - std::stod(averages[i][j - 25])) / std::stod(averages[i][j - 25]));
				else if (j < 43)
					averages[i][j] = std::to_string((std::stod(averages[i][j - 25]) - std::stod(averages[i][j - 22])) / std::stod(averages[i][j - 22]));

			}
		}

		std::ofstream ostream("Results/AverageRedo" + std::to_string(redoStart) + '-' + std::to_string(redoEnd) + ".csv");
		ostream << std::fixed << std::setprecision(5);
		ostream << "Mean,Standard Deviation,Total Duration To Be Transferred,Transfer Time,Recovery Time,Delta Optimal,Delta Star Weibull,Delta Star Gamma,Delta Star Lognormal,"
			"Collection Time Weibull,Collection Time Gamma,Collection Time Lognormal,"
			"Collection Time Star Weibull,Collection Time Star Gamma,Collection Time Star Lognormal,"
			"Wasted Time Weibull,Wasted Time Gamma,Wasted Time Lognormal," 
			"Wasted Time Star Weibull,Wasted Time Star Gamma,Wasted Time Star Lognormal," 
			"Actual Total Duration Weibull,Actual Total Duration Gamma,Actual Total Duration Lognormal,"
			"Actual Total Duration Star Weibull,Actual Total Duration Star Gamma,Actual Total Duration Star Lognormal,"
			"Total Number Of Failures Weibull,Total Number Of Failures Gamma, Total Number Of Failures Lognormal,"
			"Total Number Of Failures Star Weibull,Total Number Of Failures Star Gamma, Total Number Of Failures Star Lognormal,"
			"Redo,Diff Delta Weibull,Diff Delta Gamma,Diff Delta Lognormal,"
			"Diff CT Weibull,Diff CT Gamma,Diff CT Lognormal,"
			"Diff WT Weibull,Diff WT Gamma,Diff WT Lognormal\n";

		for (int i = 0; i < averages.size(); i++)
		{
			for (int j = 0; j < averages[0].size(); j++)
			{
				ostream << averages[i][j];
				if (j != averages[0].size() - 1)
					ostream << ',';
			}
			ostream << '\n';
		}
	}

	long long Simulation::GetDeltaOpt()
	{
		return m_SummaryData.DeltaOpt;
	}


	template<typename T>
	void Distribution<T>::GenerateFailures(std::mt19937_64& random, long long totalDurationToBeTransferred)
	{
		long long currentFailure = 0;
		
		while (currentFailure < totalDurationToBeTransferred)
		{
			long long currentFailureInterval = m_Distribution(random);
			while (currentFailureInterval <= 0)
				currentFailureInterval = m_Distribution(random);

			m_Intervals.push_back(currentFailureInterval);
			currentFailure += currentFailureInterval;
			m_FailurePoints.push_back(currentFailure);
		}


#if 0
		for (int j = 2000; j < 3501; j += 100)
		{
			double average = 0.0;
			for (int i = 0; i < j; i++)
				average += (double)intervals[i] / j;

			double stddev = 0.0;
			for (int i = 0; i < j; i++)
				stddev += std::pow(intervals[i] - average, 2) / (j - 1);
			stddev = std::sqrt(stddev);

			std::cout << "LastFailure = " << j << ", Mean = " << average << ", StdDev = " << stddev << '\n';
		}

		for (int j = 250; j < 351; j += 2)
		{
			double average = 0.0;
			for (int i = 0; i < j; i++)
				average += (double)intervals[i] / j;

			double stddev = 0.0;
			for (int i = 0; i < j; i++)
				stddev += std::pow(intervals[i] - average, 2) / (j - 1);
			stddev = std::sqrt(stddev);

			std::cout << "LastFailure = " << j << ", Mean = " << average << ", StdDev = " << stddev << '\n';
		}
#endif
	}

	template<typename T>
	std::map<long long, long long> Distribution<T>::GetCDF(long long finalFailure)
	{
		//std::cout << "FinalFail = " << finalFailure << ", Size = " << m_Intervals.size() << '\n';
		std::vector<long long> tempIntervals(m_Intervals.begin(), m_Intervals.begin() + finalFailure);

		std::sort(tempIntervals.begin(), tempIntervals.end());

		std::map<long long, long long> hist;
		long long prevCount = 0;
		for (int i = 0; i < tempIntervals.size(); i++)
		{

			long long currentInterval = tempIntervals[i];

			int count = 1;
			while (i + 1 < tempIntervals.size() && tempIntervals[i + 1] == currentInterval)
			{
				count++;
				i++;
			}
			hist[currentInterval] = count + prevCount;
			prevCount = count + prevCount;

		}

		return hist;
	}

	void Simulation::LogCDF()
	{
		std::string temp = "Results/Redo" + std::to_string(m_SummaryData.RedoCount) + "/";
		std::string temp2 = "M" + std::to_string(m_SummaryData.Mean) + 'S' + std::to_string(m_SummaryData.StdDev)
			+ "DUR" + std::to_string(m_SummaryData.TotalDurationToBeTransferred) + 'T' + std::to_string(m_SummaryData.TransferTime) + 'R' + std::to_string(m_SummaryData.RecoveryTime) + "CDF.csv";

		{

			std::map<long long, long long> weibull = m_Weibull.GetCDF(m_SummaryData.FinalFailureIndexWeibull);
			std::map<long long, long long> gamma = m_Gamma.GetCDF(m_SummaryData.FinalFailureIndexGamma);
			std::map<long long, long long> lognormal = m_Lognormal.GetCDF(m_SummaryData.FinalFailureIndexLognormal);

			long long biggest = std::max(std::max((--weibull.end())->first, (--gamma.end())->first), (--lognormal.end())->first);
			long long prevInterval = -1;
			double prevCDWeibull = 0;
			double prevCDGamma = 0;
			double prevCDLognormal = 0;

			std::ofstream CDFstream(temp + "Simulation" + temp2);

			CDFstream << "Weibull,,,,Gamma,,,,Lognormal\n\n";
			CDFstream << "Failure Count," << m_SummaryData.FinalFailureIndexWeibull << ",,,Failure Count," << m_SummaryData.FinalFailureIndexGamma << ",,,Failure Count," << m_SummaryData.FinalFailureIndexLognormal << "\n\n";

			for (int i = 0; i <= biggest; i++)
			{
				if (weibull.find(i) != weibull.end())
					prevCDWeibull = weibull[i] / (double)m_SummaryData.FinalFailureIndexWeibull;
				if (gamma.find(i) != gamma.end())
					prevCDGamma = gamma[i] / (double)m_SummaryData.FinalFailureIndexGamma;
				if (lognormal.find(i) != lognormal.end())
					prevCDLognormal = lognormal[i] / (double)m_SummaryData.FinalFailureIndexLognormal;

				CDFstream << i << ',' << prevCDWeibull << ",,," << i << ',' << prevCDGamma << ",,," << i << ',' << prevCDLognormal << '\n';

			}
		}

		{

			std::map<long long, long long> weibullStar = m_Weibull.GetCDF(m_SummaryData.FinalFailureIndexStarWeibull);
			std::map<long long, long long> gammaStar = m_Gamma.GetCDF(m_SummaryData.FinalFailureIndexStarGamma);
			std::map<long long, long long> lognormalStar = m_Lognormal.GetCDF(m_SummaryData.FinalFailureIndexStarLognormal);

			long long biggest = std::max(std::max((--weibullStar.end())->first, (--gammaStar.end())->first), (--lognormalStar.end())->first);
			long long prevInterval = -1;
			double prevCDWeibull = 0;
			double prevCDGamma = 0;
			double prevCDLognormal = 0;

			std::ofstream CDFstream(temp + "BruteForce" + temp2);

			CDFstream << "Weibull,,,,Gamma,,,,Lognormal\n\n";
			CDFstream << "Failure Count," << m_SummaryData.FinalFailureIndexStarWeibull << ",,,Failure Count," << m_SummaryData.FinalFailureIndexStarGamma << ",,,Failure Count," << m_SummaryData.FinalFailureIndexStarLognormal << "\n\n";

			for (int i = 0; i <= biggest; i++)
			{
				if (weibullStar.find(i) != weibullStar.end())
					prevCDWeibull = weibullStar[i] / (double)m_SummaryData.FinalFailureIndexStarWeibull;
				if (gammaStar.find(i) != gammaStar.end())
					prevCDGamma = gammaStar[i] / (double)m_SummaryData.FinalFailureIndexStarGamma;
				if (lognormalStar.find(i) != lognormalStar.end())
					prevCDLognormal = lognormalStar[i] / (double)m_SummaryData.FinalFailureIndexStarLognormal;

				CDFstream << i << ',' << prevCDWeibull << ",,," << i << ',' << prevCDGamma << ",,," << i << ',' << prevCDLognormal << '\n';

			}
		}


	}

}