#pragma once

namespace WSN
{

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
}