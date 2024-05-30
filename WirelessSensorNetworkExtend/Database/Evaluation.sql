use WSN17;

-- total collection time
select SimulationFTTDMA.FailureMean,
sum(SimulationFTTDMA.ActualTotalDuration), sum(SimulationRRTDMA.ActualTotalDuration), sum(SimulationRRTDMA.ActualTotalDuration) / sum(SimulationFTTDMA.ActualTotalDuration) from SimulationFTTDMA 
inner join SimulationRRTDMA on SimulationRRTDMA.SimulationID = SimulationFTTDMA.SimulationID 
group by SimulationFTTDMA.FailureMean;


-- total energy consumed
select SimulationFTTDMA.EnergyRateWorking, SimulationFTTDMA.EnergyRateTransfer, SimulationFTTDMA.EnergyRateTransfer / SimulationFTTDMA.EnergyRateWorking as EnergyRatio,
sum(SensorNodeFTTDMA.EnergyConsumed), sum(SensorNodeRRTDMA.EnergyConsumed), sum(SensorNodeRRTDMA.EnergyConsumed) / sum(SensorNodeFTTDMA.EnergyConsumed)
from SimulationFTTDMA
inner join SensorNodeFTTDMA
inner join SensorNodeRRTDMA
on SimulationFTTDMA.SimulationID = SensorNodeFTTDMA.SimulationID and SimulationFTTDMA.SimulationID = SensorNodeRRTDMA.SimulationID
group by SimulationFTTDMA.SimulationID;