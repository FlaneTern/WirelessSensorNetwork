drop database if exists WSN17;

create database WSN17;
use WSN17;

create table SimulationFTTDMA(
	SimulationID bigint unsigned not null,
    TotalDurationToBeTransferred double,
    TransferTime double,
    RecoveryTime double, 
    FailureDistributionType enum('Exponential', 'Gamma', 'Lognormal', 'Weibull', 'Normal', 'Uniform'),
    FailureMean double,
    FailureStddev double,
    FailureParameter1 double,
    FailureParameter2 double,
    ActualTotalDuration double,
    FinalFailureIndex bigint unsigned,
    CWSNEfficiency double,
    EnergyRateWorking double,
    EnergyRateTransfer double,
    primary key(SimulationID)
);

create table SensorNodeFTTDMA(
	SimulationID bigint unsigned not null,
    SensorNodeID bigint unsigned not null,
    PosX double,
    PosY double,
    Parent bigint,
    Level_ bigint unsigned,
    DeltaOpt double,
    CollectionTime double,
    WastedTime double,
    EnergyConsumed double,
    SentPacketTotalDelay double,
    SentPacketCount bigint unsigned,
    Color bigint unsigned,
    TotalDataSent double,
    primary key(SimulationID, SensorNodeID),
    foreign key(SimulationID) references SimulationFTTDMA(SimulationID)
);

create table SimulationRRTDMA(
	SimulationID bigint unsigned not null,
    TotalDurationToBeTransferred double,
    TransferTime double,
    RecoveryTime double, 
    FailureDistributionType enum('Exponential', 'Gamma', 'Lognormal', 'Weibull', 'Normal', 'Uniform'),
    FailureMean double,
    FailureStddev double,
    FailureParameter1 double,
    FailureParameter2 double,
    ActualTotalDuration double,
    FinalFailureIndex bigint unsigned,
    CWSNEfficiency double,
	EnergyRateWorking double,
    EnergyRateTransfer double,
    primary key(SimulationID)
);

create table SensorNodeRRTDMA(
	SimulationID bigint unsigned not null,
    SensorNodeID bigint unsigned not null,
    PosX double,
    PosY double,
    Parent bigint,
    Level_ bigint unsigned,
    DeltaOpt double,
    CollectionTime double,
    WastedTime double,
    EnergyConsumed double,
    SentPacketTotalDelay double,
    SentPacketCount bigint unsigned,
    Color bigint unsigned,
    TotalDataSent double,
    primary key(SimulationID, SensorNodeID),
    foreign key(SimulationID) references SimulationRRTDMA(SimulationID)
);

-- placeholder
insert into SimulationFTTDMA values(0, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
insert into SimulationRRTDMA values(0, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);

select * from SimulationFTTDMA;
select * from SimulationRRTDMA;
select * from SensorNodeFTTDMA; -- where SimulationID > 54;

