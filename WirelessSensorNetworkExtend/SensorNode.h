#pragma once

namespace WSN
{
	struct Position
	{
		double X;
		double Y;
	};

	// Sensor Node states
	enum class WorkingState
	{
		Collection,
		Transfer,
		Recovery
	};

	std::string WorkingStateToString(const WorkingState& ws);

	struct Packet
	{
		uint64_t InitialSNID;
		double InitialTimestamp;
		double Size;
	};

	class SensorNode
	{
	public:
		Position m_Position;
		int64_t m_Parent;
		uint64_t m_Level;

		double m_DeltaOpt;

		double m_CurrentData = 0;

		double m_CollectionTime = 0;
		double m_WastedTime = 0;

		double m_EnergyConsumed = 0;

		double m_SentPacketTotalDelay = 0;
		uint64_t m_SentPacketCount = 0;

		uint64_t m_Color;

		double m_TotalDataSent = 0;

		std::vector<Packet> m_Packets;

		int m_CurrentPacketIterator = -1;
	};
}