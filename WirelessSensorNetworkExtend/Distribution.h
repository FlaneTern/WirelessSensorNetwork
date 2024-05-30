#pragma once

namespace WSN
{
	extern std::mt19937_64 s_RNG;

	enum class DistributionType
	{
		Exponential = 0,
		Gamma,
		Lognormal,
		Weibull,
		Normal,
		Uniform
	};

	std::string DistributionTypeToString(const DistributionType& dt);

	class Distribution
	{
	public:
		Distribution(DistributionType distributionType, double mean, double stddev);
		Distribution(const Distribution& other);
		~Distribution();

		double GenerateRandomNumber();
		double GenerateRandomNumber(std::mt19937_64& rng);
		std::map<long long, long long> GetCDF();

		void* m_Distribution;
		DistributionType m_DistributionType;

		double m_Mean;
		double m_Stddev;

		double m_Parameter1;
		double m_Parameter2;
	};

	struct ExponentialParameters
	{
		ExponentialParameters(double mean, double stddev);
		double Rate; // lambda
	};

	struct GammaParameters
	{
		GammaParameters(double mean, double stddev);
		double Shape; // k
		double Scale; // theta
	};

	struct LognormalParameters
	{
		LognormalParameters(double mean, double stddev);
		double M; // mu
		double S; // sigma
	};

	struct WeibullParameters
	{
		WeibullParameters(double mean, double stddev);
		double Shape; // k
		double Scale; // lambda
	};

	struct NormalParameters
	{
		NormalParameters(double mean, double stddev);
		double Mean; // mu
		double Stddev; // sigma
	};

	struct UniformParameters
	{
		UniformParameters(double mean, double stddev);
		double A; // a, lower limit
		double B; // b, upper limit
	};
}