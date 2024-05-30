#include "PCH.h"

#if not _DEBUG
#include "Database.h"

#include "mysql_connection.h"
#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/prepared_statement.h>

namespace WSN
{
    // CONSTANTS
    static constexpr char c_Server[] = "tcp://127.0.0.1:3306\0";
    static constexpr char c_Username[] = "WSN\0";
    static constexpr char c_Password[] = "wsn123\0";
    static constexpr char c_DatabaseName[] = "WSN17\0";


	// A LOT OF UNFREED STUFF such as driver, connection, and statements. Maybe free these?
    // Doesn't matter tbh

	Database* Database::s_DatabaseInstance = new Database();

	static sql::Driver* s_Driver;
	static sql::Connection* s_Connection;

    static constexpr uint32_t c_BatchRowCount = 50000 / 14;

    static std::string CreateInsertString(uint32_t argumentCount, uint32_t rowCount)
    {
        std::stringstream ss;
        ss << " values";

        for (int i = 0; i < rowCount; i++)
        {
            ss << "(?";
            for (int j = 1; j < argumentCount; j++)
                ss << ",?";
            ss << ")";
            if (i != rowCount - 1)
                ss << ',';
        }

        ss << ';';

        return ss.str();
    }


 	Database::Database()
	{
        try
        {
            s_Driver = get_driver_instance();
            s_Connection = s_Driver->connect(c_Server, c_Username, c_Password);
            s_Connection->setSchema(c_DatabaseName);
        }
        catch (sql::SQLException e)
        {
            std::cout << "Could not connect to server. Error message: " + std::string(e.what()) << '\n';
            throw std::runtime_error("Could not connect to server. Error message: " + std::string(e.what()));
        }


	}

    void Database::Insert(uint64_t simulationID, const SimulationParameters& simulationParameters, const SimulationResults& sr, const SimulationType& st)
    {
        std::cout << "Inserting Simulation " << simulationID << '\n';
        try
        {
            static sql::PreparedStatement* statement1FTTDMA = s_Connection->prepareStatement(
                "Insert into "
                "SimulationFTTDMA(SimulationID, TotalDurationToBeTransferred, TransferTime, RecoveryTime, FailureDistributionType, FailureMean, FailureStddev, FailureParameter1, FailureParameter2, ActualTotalDuration, FinalFailureIndex, CWSNEfficiency, EnergyRateWorking, EnergyRateTransfer)"
                " values(?,?,?,?,?,?,?,?,?,?,?,?,?,?)");

            static sql::PreparedStatement* statement1RRTDMA = s_Connection->prepareStatement(
                "Insert into "
                "SimulationRRTDMA(SimulationID, TotalDurationToBeTransferred, TransferTime, RecoveryTime, FailureDistributionType, FailureMean, FailureStddev, FailureParameter1, FailureParameter2, ActualTotalDuration, FinalFailureIndex, CWSNEfficiency, EnergyRateWorking, EnergyRateTransfer)"
                " values(?,?,?,?,?,?,?,?,?,?,?,?,?,?)");

            sql::PreparedStatement* statement1 = nullptr;
            if (st == SimulationType::FT_TDMA)
                statement1 = statement1FTTDMA;
            else if(st == SimulationType::RR_TDMA)
                statement1 = statement1RRTDMA;

            std::cout << "Inserting Simulation " << simulationID << '\n';

            statement1->setUInt64(1, simulationID);
            statement1->setDouble(2, simulationParameters.TotalDurationToBeTransferred);
            statement1->setDouble(3, simulationParameters.TransferTime);
            statement1->setDouble(4, simulationParameters.RecoveryTime);
            statement1->setString(5, DistributionTypeToString(simulationParameters.FailureDistribution.m_DistributionType));
            statement1->setDouble(6, simulationParameters.FailureDistribution.m_Mean);
            statement1->setDouble(7, simulationParameters.FailureDistribution.m_Stddev);
            statement1->setDouble(8, simulationParameters.FailureDistribution.m_Parameter1);
            statement1->setDouble(9, simulationParameters.FailureDistribution.m_Parameter2);
            statement1->setDouble(10, sr.ActualTotalDuration);
            statement1->setUInt64(11, sr.FinalFailureIndex);
            statement1->setDouble(12, sr.CWSNEfficiency);
            statement1->setDouble(13, simulationParameters.EnergyRateWorking);
            statement1->setDouble(14, simulationParameters.EnergyRateTransfer);
            statement1->execute();
            s_Connection->commit();

        }
        catch (sql::SQLException& e)
        {
            std::cout << "SQL Error in Insert SP SR. Error message: " + std::string(e.what()) << '\n';
            throw std::runtime_error("SQL Error in Insert SP SR. Error message: " + std::string(e.what()));
        }
    }

    void Database::Insert(uint64_t simulationID, const std::vector<SensorNode>& sensorNodes, const const SimulationType& st)
    {
        try
        {
            static sql::PreparedStatement* preparedStatementFTTDMA = s_Connection->prepareStatement(
                "Insert ignore into "
                "SensorNodeFTTDMA(SimulationID, SensorNodeID, PosX, PosY, Parent, Level_, DeltaOpt, CollectionTime, WastedTime, EnergyConsumed, SentPacketTotalDelay, SentPacketCount, Color, TotalDataSent)" +
                CreateInsertString(14, c_BatchRowCount));

            static sql::PreparedStatement* preparedStatementRRTDMA = s_Connection->prepareStatement(
                "Insert ignore into "
                "SensorNodeRRTDMA(SimulationID, SensorNodeID, PosX, PosY, Parent, Level_, DeltaOpt, CollectionTime, WastedTime, EnergyConsumed, SentPacketTotalDelay, SentPacketCount, Color, TotalDataSent)" +
                CreateInsertString(14, c_BatchRowCount));

            sql::PreparedStatement* preparedStatement = nullptr;
            if (st == SimulationType::FT_TDMA)
                preparedStatement = preparedStatementFTTDMA;
            else if (st == SimulationType::RR_TDMA)
                preparedStatement = preparedStatementRRTDMA;

            bool done = false;
            int SNIterator = 0;
            for (int batchStartingRow = 0; !done; batchStartingRow += c_BatchRowCount)
            {
                std::cout << "Inserting SensorNode " << simulationID << ", " << SNIterator << '\n';
                std::cout << "Out of " << sensorNodes.size() << '\n';
                for (int currentBatchRow = 0; !done && currentBatchRow < c_BatchRowCount; currentBatchRow++)
                {
                    preparedStatement->setUInt64(currentBatchRow * 14 + 1, simulationID);
                    preparedStatement->setUInt64(currentBatchRow * 14 + 2, SNIterator);
                    preparedStatement->setDouble(currentBatchRow * 14 + 3, sensorNodes[SNIterator].m_Position.X);
                    preparedStatement->setDouble(currentBatchRow * 14 + 4, sensorNodes[SNIterator].m_Position.Y);
                    preparedStatement->setInt64(currentBatchRow  * 14 + 5, sensorNodes[SNIterator].m_Parent);
                    preparedStatement->setUInt64(currentBatchRow * 14 + 6, sensorNodes[SNIterator].m_Level);
                    preparedStatement->setDouble(currentBatchRow * 14 + 7, sensorNodes[SNIterator].m_DeltaOpt);
                    preparedStatement->setDouble(currentBatchRow * 14 + 8, sensorNodes[SNIterator].m_CollectionTime);
                    preparedStatement->setDouble(currentBatchRow * 14 + 9, sensorNodes[SNIterator].m_WastedTime);
                    preparedStatement->setDouble(currentBatchRow * 14 + 10, sensorNodes[SNIterator].m_EnergyConsumed);
                    preparedStatement->setDouble(currentBatchRow * 14 + 11, sensorNodes[SNIterator].m_SentPacketTotalDelay);
                    preparedStatement->setUInt64(currentBatchRow * 14 + 12, sensorNodes[SNIterator].m_SentPacketCount);
                    preparedStatement->setUInt64(currentBatchRow * 14 + 13, sensorNodes[SNIterator].m_Color);
                    preparedStatement->setDouble(currentBatchRow * 14 + 14, sensorNodes[SNIterator].m_TotalDataSent);

                    //std::cout << "here = " << currentBatchRow * 10 + 1 << '\n';

                    SNIterator++;
                    if (SNIterator == sensorNodes.size())
                        done = true;

                    if (done)
                    {
                        for (int i = currentBatchRow + 1; i < c_BatchRowCount; i++)
                        {
                            //std::cout << "here in i = " << i * 10 + 1 << '\n';
                            preparedStatement->setUInt64(i * 14 + 1, 0);
                            preparedStatement->setUInt64(i * 14 + 2, 0);
                            preparedStatement->setNull(i * 14 + 3, sql::DataType::DOUBLE);
                            preparedStatement->setNull(i * 14 + 4, sql::DataType::DOUBLE);
                            preparedStatement->setNull(i * 14 + 5, sql::DataType::BIGINT);
                            preparedStatement->setNull(i * 14 + 6, sql::DataType::BIGINT);
                            preparedStatement->setNull(i * 14 + 7, sql::DataType::DOUBLE);
                            preparedStatement->setNull(i * 14 + 8, sql::DataType::DOUBLE);
                            preparedStatement->setNull(i * 14 + 9, sql::DataType::DOUBLE);
                            preparedStatement->setNull(i * 14 + 10, sql::DataType::DOUBLE);
                            preparedStatement->setNull(i * 14 + 11, sql::DataType::DOUBLE);
                            preparedStatement->setNull(i * 14 + 12, sql::DataType::BIGINT);
                            preparedStatement->setNull(i * 14 + 13, sql::DataType::BIGINT);
                            preparedStatement->setNull(i * 14 + 14, sql::DataType::DOUBLE);

                        }
                    }
                }

                preparedStatement->execute();
                preparedStatement->clearAttributes();
                preparedStatement->clearParameters();
                s_Connection->commit();
            }

        }
        catch (sql::SQLException& e)
        {
            std::cout << "SQL Error in Insert SensorNode. Error message: " + std::string(e.what());
            throw std::runtime_error("SQL Error in Insert SensorNode. Error message: " + std::string(e.what()));
        }
    }



    uint64_t Database::GetLatestSimulationID()
    {
        static sql::PreparedStatement* statement = s_Connection->prepareStatement("select max(SimulationID) from SimulationFTTDMA");
        sql::ResultSet* result = statement->executeQuery();

        result->next();
        return result->getUInt64(1);
    }
}

#endif