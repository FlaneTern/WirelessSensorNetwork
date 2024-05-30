#include "PCH.h"

#include "Distribution.h"


namespace WSN
{
	std::mt19937_64 s_RNG(std::chrono::high_resolution_clock::now().time_since_epoch().count());

	static double FunctionWeibull(double xCurrent, double target)
	{
		return std::tgammal(1 + 2 / xCurrent) / (std::tgammal(1 + 1 / xCurrent) * std::tgammal(1 + 1 / xCurrent)) - target;
	}
		
	static double NewtonsMethodWeibull(double mean, double stddev)
	{
		static constexpr double x0 = 0.5;
		static constexpr double step = 0.1;
		static constexpr double maximumError = 0.00001;

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

	ExponentialParameters::ExponentialParameters(double mean, double stddev)
	{
		Rate = 1.0 / mean;
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

	NormalParameters::NormalParameters(double mean, double stddev)
	{
		Mean = mean;
		Stddev = stddev;
	}

	UniformParameters::UniformParameters(double mean, double stddev)
	{
		A = mean - std::sqrt(3) * stddev;
		B = mean + std::sqrt(3) * stddev;
	}


	Distribution::Distribution(DistributionType distributionType, double mean, double stddev)
	{
		m_DistributionType = distributionType;
		m_Mean = mean;
		m_Stddev = stddev;

		switch (distributionType)
		{
		case DistributionType::Exponential: 
		{
			if (mean != stddev)
				throw std::runtime_error("Exponential distribution must have the same mean and stddev!");

			ExponentialParameters params(mean, stddev);
			m_Parameter1 = params.Rate;
			m_Parameter2 = 0;
			m_Distribution = new std::exponential_distribution<double>(params.Rate);
			break;
		}
		case DistributionType::Gamma:
		{
			GammaParameters params(mean, stddev);
			m_Parameter1 = params.Shape;
			m_Parameter2 = params.Scale;
			m_Distribution = new std::gamma_distribution<double>(params.Shape, params.Scale);
			break;
		}
		case DistributionType::Lognormal:
		{
			LognormalParameters params(mean, stddev);
			m_Parameter1 = params.M;
			m_Parameter2 = params.S;
			m_Distribution = new std::lognormal_distribution<double>(params.M, params.S);
			break;
		}
		case DistributionType::Weibull:
		{
			WeibullParameters params(mean, stddev);
			m_Parameter1 = params.Shape;
			m_Parameter2 = params.Scale;
			m_Distribution = new std::weibull_distribution<double>(params.Shape, params.Scale);
			break;
		}
		case DistributionType::Normal:
		{
			NormalParameters params(mean, stddev);
			m_Parameter1 = params.Mean;
			m_Parameter2 = params.Stddev;
			m_Distribution = new std::normal_distribution<double>(params.Mean, params.Stddev);
			break;
		}
		case DistributionType::Uniform:
		{
			UniformParameters params(mean, stddev);
			m_Parameter1 = params.A;
			m_Parameter2 = params.B;
			m_Distribution = new std::uniform_real_distribution<double>(params.A, params.B);
			break;
		}
		default:
		{
			throw std::runtime_error("Unknown Distribution Type in Distribution Constructor");
		}
		}
	}

	Distribution::Distribution(const Distribution& other)
		: Distribution(other.m_DistributionType, other.m_Mean, other.m_Stddev) {}

	Distribution::~Distribution()
	{
		switch (m_DistributionType)
		{
		case DistributionType::Exponential:
			delete (std::exponential_distribution<double>*)m_Distribution;
			break;
		case DistributionType::Gamma:
			delete (std::gamma_distribution<double>*)m_Distribution;
			break;
		case DistributionType::Lognormal:
			delete (std::lognormal_distribution<double>*)m_Distribution;
			break;
		case DistributionType::Weibull:
			delete (std::weibull_distribution<double>*)m_Distribution;
			break;
		case DistributionType::Normal:
			delete (std::normal_distribution<double>*)m_Distribution;
			break;
		case DistributionType::Uniform:
			delete (std::uniform_real_distribution<double>*)m_Distribution;
			break;
		default:
		{
			throw std::runtime_error("Unknown Distribution Type in Distribution Destructor");
		}
		}

	}

	double Distribution::GenerateRandomNumber()
	{
		switch (m_DistributionType)
		{
		case DistributionType::Exponential:
			return (*(std::exponential_distribution<double>*)m_Distribution)(s_RNG);
		case DistributionType::Gamma:
			return (*(std::gamma_distribution<double>*)m_Distribution)(s_RNG);
		case DistributionType::Lognormal:
			return (*(std::lognormal_distribution<double>*)m_Distribution)(s_RNG);
		case DistributionType::Weibull:
			return (*(std::weibull_distribution<double>*)m_Distribution)(s_RNG);
		case DistributionType::Normal:
			return (*(std::normal_distribution<double>*)m_Distribution)(s_RNG);
		case DistributionType::Uniform:
			return (*(std::uniform_real_distribution<double>*)m_Distribution)(s_RNG);
		}

		throw std::runtime_error("Unknown Distribution Type in Distribution::GenerateRandomNumber");
		return 0;
	}

	double Distribution::GenerateRandomNumber(std::mt19937_64& rng)
	{
		switch (m_DistributionType)
		{
		case DistributionType::Exponential:
			return (*(std::exponential_distribution<double>*)m_Distribution)(rng);
		case DistributionType::Gamma:
			return (*(std::gamma_distribution<double>*)m_Distribution)(rng);
		case DistributionType::Lognormal:
			return (*(std::lognormal_distribution<double>*)m_Distribution)(rng);
		case DistributionType::Weibull:
			return (*(std::weibull_distribution<double>*)m_Distribution)(rng);
		case DistributionType::Normal:
			return (*(std::normal_distribution<double>*)m_Distribution)(rng);
		case DistributionType::Uniform:
			return (*(std::uniform_real_distribution<double>*)m_Distribution)(rng);
		}

		throw std::runtime_error("Unknown Distribution Type in Distribution::GenerateRandomNumber");
		return 0;
	}


	std::string DistributionTypeToString(const DistributionType& dt)
	{
		switch (dt)
		{
		case DistributionType::Exponential:
			return "Exponential";
		case DistributionType::Gamma:
			return "Gamma";
		case DistributionType::Lognormal:
			return "Lognormal";
		case DistributionType::Weibull:
			return "Weibull";
		case DistributionType::Normal:
			return "Normal";
		case DistributionType::Uniform:
			return "Uniform";
		}

		throw std::runtime_error("Unknown Distribution Type in DistributionTypeToString");
		return "";
	}
}