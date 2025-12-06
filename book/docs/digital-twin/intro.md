---
title: Digital Twin - Introduction
---

# Digital Twin (Weeks 6–7)

## Overview

Learn to create and use digital twins with Gazebo and Unity.

## Learning Path

- [Week 6: Gazebo Simulation](/docs/digital-twin/week6)
- [Week 7: Unity Integration](/docs/digital-twin/week7)

## Diagrams

```mermaid
graph TD
    PhysicalAsset[Physical Robot/System] -- Sensor Data --> DigitalTwin[Digital Twin (Simulation)]
    DigitalTwin -- Control Commands --> PhysicalAsset
    Operator[Operator/AI] -- Interact with --> DigitalTwin
    DigitalTwin -- Insights --> Operator
```
