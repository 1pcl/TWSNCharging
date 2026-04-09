
# Table of Contents
- [Usage](#usage)
- [Environment Configuration](#environment-configuration)
- [Directory Structure Description](#directory-structure-description)



# Usage

This project implements a comprehensive **Algorithm Simulation and Verification Platform** with the following primary functionalities:

1.  **Scenario Simulation**: Customize simulation environment parameters (e.g., number of users, channel model, arrival rate) via configuration files located in the `Scenario` directory.
2.  **Algorithm Execution**:
    - **Main Algorithm**: Run the entry point located in the `PPO_MLP` directory to initiate training or testing processes for the RAWCS algorithm based on PPO reinforcement learning.
    - **Baseline/Auxiliary Algorithms**: Execute traditional or baseline algorithms stored in the `other` directory for performance comparison.
3.  **API Interface**: The `FlaskApi` module provides an HTTP service interface, allowing users to trigger simulations or retrieve real-time status data via API requests.
4.  **Result Storage**: All generated logs, trained model weights, and performance metric plots will be saved in the corresponding subdirectories under `data`.

**Quick Start Example**:
```sh
# Start the Flask API service (if required)
cd FlaskApi
python app.py

# Run the main algorithm training script
cd ../PPO_MLP
python train_limit.py or python train_limit_2.py
```

# Environment Configuration
Operating System: Ubuntu 20.04+ / macOS Monterey+ / Windows 11 (WSL2 recommended)

Python Version: 3.10


# Directory Structure Description
```sh
|-- README.md      // help
|-- data           // dataset
|-- FlaskApi       // python api
|-- other          // other algorithm
|-- PPO_MLP        // RAWCS algorithm
|-- Scenario       // scenario configuration
```








