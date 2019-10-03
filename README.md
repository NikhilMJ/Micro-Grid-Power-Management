# Micro Grid Power Management
This project investigates an optimal control strategy that efficiently manages an electric grid comprising of various renewable and non-renewable energy sources for a medium sized community. The grid consists of a solar farm, a conventional fossil fuel energy plant and a grid level energy storage. A community comprising of residential and commercial consumers is modeled using an expected demands. Power flow and battery storage is managed using expected power demands and expected solar production capacities. Goal is meet demand, minimize use of fossil fuels and ensure the energy storage is always maintained around a nominal point and ensure it isn't over-depleted. 

This project was conceived and developed in Fall of 2014, and the motivation was to allow for a rural community to independently and efficiently manage power sources.

The project features a behavior model developed from first principles and modeled after power flows and losses is formulated. The model is hybrid in nature, i.e. consists of switched states. A hybrid model predictive control scheme is implemented for choosing an optimal mode and set of inputs for the system for tracking both a constant and load-varying power demand profile. The entire compiled report is included in the pdf included, [Micro Grid using MPC](Micro-Grid-Power-Management/Micro%20Grid%20Energy%20Optimization%20using%20MPC.pdf)


## Grid Model
The below diagram shows the overall system layout of the micro-grid. Three power plant types(explained later) are included. The power transmission lines transmit this to residential and commercial plants. The equations of the model are attached in the attached pddf.
![alt text](Images/System%20Model.png?raw=true "Micro Grid Layout")


## Power Plant
The grid model can be expanded to include behavioral models of multiple energy producer and storage types. The main characteristics required at the grid level are time constant to meet reference targets, efficiency(or inversely cost) of production. Once a power demand is set for each power plant, it is assumed that specialized control schemes built for each power plant would perform control actions required to meet the requested power levels. In this micro-grid project, the three types of power plants are considered in order to build the proof of concept:
*Traditional: diesel generator(included in model). Other examples: hydel, nuclear power plants, thermal plants
*Renewable: solar farm(included in model). Other include wind energy, tidal energy, etc. 
*Storage: battery(included in mode). Others examples for storage plants are pumped hydro, molten salt, pressurized gas, etc.

The key here is to maximize the use of renewable sources and minimizing the use of the traditional sources. In a real world, constraints are added to ensure basic operation of these plants such as providing for spinning reserves, ensuring dam water levels are within maximum levels. These can all be included into the behavioral model as state bounds and cost per energy unit for each power plant can be adjusted for the control scheme to handle.

 
 
## Power Demand Curves
The diagram below is the reference power demand curve that would need to be met by the energy management unit. The power demand profile is known 24hrs in advance.
![alt text](Images/Load%20power%20curves.JPG?raw=true "Load Power Demand Profile")

Having a reference power profile to meet 24hrs in advance is a standard model that is currently in use by energy exchanges where 15min intervals that are bid, reserved and sold 24hrs in advance. While this allows for an efficient overall strategy, near term load demands are mostly met by the surplus that is passed to the grid as a factor of safety. The proposed model should be able to efficiently manage power demands that are made known few minutes to hours in advance. Meeting such immediate  demands is limited to the response rate of the fastest energy producer which in this case is the energy storage. 

While the Tesla powerwall concept was not known at the time of conceiving this project, it is to be noted that the response rate of energy storage solutions based on battery technologies are in the 100msec range [See link](https://reneweconomy.com.au/speed-of-tesla-big-battery-leaves-rule-makers-struggling-to-catch-up-36135/). This allows the control scheme to optimize for and meet power demands that are not known well ahead of time and allows handling of high frequency arbitrages. A behavioral model of this type allows operation and management of a system of systems and captures enough information to allow management of the grid. Additional constraints of resilience to failures, robustness and factor of safety can be easily added to the behavioral model. Furthermore, daily weather prediction data can be fed to this behavioral model to build better long term optimization goals whereas hourly prediction data can be fed and used to provide efficient control command generation.



## Results
Kindly refer to the [pdf](Micro%20Grid%20Energy%20Optimization%20using%20MPC.pdf) above for a detailed discussion on the motivation, problem formulation, MPC setup and results. Also results of several weight balances, constraints and other limits in [results](Code/Results). Below is a sample state trajectory from solving the MPC
![alt text](Code/Results/Results_08_Dec_2014_02_55_22/State_Trajectory.png?raw=true "Sample State Trajectory Solution")


## Keywords: Energy, Model, Predictive, Control, MPC, Powerwall, grid, storage
