---
title: Hardware Tracks
description: Hardware requirements and setup options
---

# Hardware Tracks: On-Premise vs Cloud "Ether Lab"

## Overview

This textbook supports two hardware tracks:

1. **On-Premise**: Physical hardware in your lab
2. **Cloud "Ether Lab"**: Remote cloud-based hardware access

## On-Premise Setup

### Minimum Requirements

- **Computer**: Ubuntu 22.04, 16GB RAM, dedicated GPU (recommended)
- **Robot Hardware**: Varies by chapter (see specific requirements)
- **Network**: Local network for robot communication

### Recommended Hardware

- **Development Machine**: 
  - CPU: Intel i7 or AMD Ryzen 7
  - RAM: 32GB
  - GPU: NVIDIA RTX 3060 or better
  - Storage: 500GB SSD

- **Robot Platform**: 
  - ROS 2 compatible robot
  - Sensors: Camera, LiDAR (as needed)
  - Actuators: Servos, motors

## Cloud "Ether Lab" Setup

### Benefits

- No physical hardware required
- Access to advanced robots
- Scalable resources
- Remote collaboration

### Setup Steps

1. Register for Ether Lab access
2. Configure VPN connection
3. Reserve robot time slots
4. Connect via web interface or SSH

### Requirements

- Stable internet connection (10+ Mbps)
- VPN client software
- Account credentials

## Lab Setup Instructions

### Ubuntu 22.04 Installation

1. Download Ubuntu 22.04 LTS
2. Create bootable USB
3. Install with dual-boot or as primary OS
4. Update system:
   ```bash
   sudo apt update && sudo apt upgrade -y
   ```

### ROS 2 Humble Installation

See [Week 1 Lab](/docs/foundations/week1) for detailed ROS 2 installation steps.

### Gazebo Installation

```bash
sudo apt install gazebo11 -y
```

### Isaac Sim Prerequisites

- NVIDIA GPU with CUDA support
- NVIDIA Driver 470+
- CUDA 11.4+
- Docker (for containerized deployment)

## Troubleshooting

### Common Issues

1. **ROS 2 not found**: Source the setup file
2. **Gazebo won't start**: Check GPU drivers
3. **Network issues**: Verify firewall settings

## Next Steps

- Complete [Lab Setup](/docs/foundations/week1)
- Review [Safety Guidelines](/docs/preface/intro#safety--ethics)
- Set up your chosen hardware track

## Diagrams

```mermaid
graph TD
    OnPremise[On-Premise Setup] --> LocalPC[Local Development PC]
    OnPremise --> PhysicalRobot[Physical Robot Hardware]

    EtherLab[Cloud "Ether Lab" Setup] --> RemoteAccess[Remote Access (VPN/SSH)]
    RemoteAccess --> CloudRobot[Cloud-Based Robot/Simulation]

    LocalPC -- Connects To --> PhysicalRobot
    LocalPC -- Connects To --> RemoteAccess

    subgraph Hardware Options
        OnPremise & EtherLab
    end
```
