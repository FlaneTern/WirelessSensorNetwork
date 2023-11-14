#include "WSNPCH.h"

#include "DistributionParameters.h"


namespace WSN
{
	static double FunctionWeibull(double xCurrent, double target)
	{
		return std::tgammal(1 + 2 / xCurrent) / (std::tgammal(1 + 1 / xCurrent) * std::tgammal(1 + 1 / xCurrent)) - target;
	}
		
	static double NewtonsMethodWeibull(double mean, double stddev)
	{
		static constexpr double x0 = 0.5;
		static constexpr double step = 0.1;
		static constexpr double maximumError = 0.00001;
		static std::mt19937 Random(std::chrono::steady_clock::now().time_since_epoch().count());

		double target = (mean * mean + stddev * stddev) / (mean * mean);

		double xCurrent = x0;
		double yCurrent = FunctionWeibull(xCurrent, target);
		double yCurrent2 = FunctionWeibull(xCurrent + step, target);

		do
		{
			xCurrent -= yCurrent / ((yCurrent2 - yCurrent) / step);
			yCurrent = FunctionWeibull(xCurrent, target);
			yCurrent2 = FunctionWeibull(xCurrent + step, target);

		} while (std::abs(yCurrent) > maximumError);

		return xCurrent;
	}

	GammaParameters::GammaParameters(double mean, double stddev)
	{
		Shape = mean * mean / (stddev * stddev);
		Scale = stddev * stddev / mean;
	}

	LognormalParameters::LognormalParameters(double mean, double stddev)
	{
		S = std::sqrt(std::log(stddev * stddev / (mean * mean) + 1));
		M = std::log(mean) - std::log(stddev * stddev / (mean * mean) + 1) / 2;
	}

	WeibullParameters::WeibullParameters(double mean, double stddev)
	{
		Shape = NewtonsMethodWeibull(mean, stddev);
		Scale = mean / tgammal(1 + 1 / Shape);
	}
}