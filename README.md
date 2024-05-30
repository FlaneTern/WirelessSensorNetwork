# WirelessSensorNetwork

Source code for **Investigating the Impact of Optimal Data Transfer Intervals on Failure-prone Wireless Sensor Networks**, Amrizal et al.

## 
The source code is written in C++, made using Microsoft Visual Studio 2022 IDE and compiled with the MSVC C++20 compiler.

The source code is separated into 2 parts. First, the **WirelessSensorNetwork** folder contains the source code for generating the data of *Increase in the Period of Data Loss* and *Differences in Data Transfer Interval*, used in Figure 3. Secondly, the **WirelessSensorNetworkExtend** folder contains the source code for generating the data of *Normalized Data Collection Time* and *Normalized Energy Consumption*, used in Figure 5.

## Generating Data in Figure 3
In the **WirelessSensorNetwork** folder, edit the parameters in the **Main.cpp** file according to the desired parameters. Compiling and running the source code will generate the data and save it as *.csv* files. The final result of the generation that is used in Figure 3 will be contained in **AverageRedo#.csv**.

## Generating Data in Figure 5
The data in this figure are saved into a MySQL 8.0 database. First, setup a MySQL server and edit the database server address, username, password, and database name in the **Database.cpp** source file in the **WirelessSensorNetworkExtend** folder to match your database. Next, open the **Database** folder and run the **RecreateDatabaseWSN.sql** script to generate the database and its tables.

In the **WirelessSensorNetworkExtend** folder, edit the parameters in the **Main.cpp** file according to the desired parameters. Compiling and running the source code will generate the data and save it in the MySQL database server specified beforehand.

To retrieve the data, open the **Evaluation.sql** file in the **Database** folder. It contains 2 queries, one for the *Normalized Data Collection Time* data and the other for the *Normalized Energy Consumption* data.

