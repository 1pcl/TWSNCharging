# RAWCS: Road-Aware Wireless Charging Scheduling for Traffic Wireless Sensor Networks

This repository contains the official implementation of the paper:

> **RAWCS: Road-Aware Wireless Charging Scheduling for Traffic Wireless Sensor Networks**  

## 📌 Abstract

Traffic Wireless Sensor Networks (TWSNs) are an important sensing infrastructure for intelligent transportation systems. As the nodes in TWSNs are typically powered by batteries, the lifetime of the network is limited by their energy capacity.
Mobile wireless charging is an effective technology for energy replenishment in large-scale TWSNs.
However, most existing works assume free-space mobility for charger path planning and treat network nodes as homogeneous, overlooking road constraints and node heterogeneity.
This paper proposes RAWCS, a Road-Aware Wireless Charging Scheduling algorithm for TWSNs. 
The proposed method first constructs a graph to model the spatial relationships between nodes deployed along the urban road network.
Then, the original large-scale network graph is compressed into a task-centric subgraph by retaining only nodes with charging demands and their associated edges.
An attention-based Deep Reinforcement Learning (DRL) scheduler is designed over the compressed graph to jointly optimize mobile charger (MC) routing and adaptive charging duration allocation.
Additionally, a practical software system for the scheduling of mobile wireless charging in TWSNs has been developed. 

## Directory Structure Description
```sh
|-- README.md     // help
|-- Algorithm  // Core RAWCS algorithm
|-- System  // Complete system code
