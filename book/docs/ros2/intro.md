---
title: ROS 2 Nervous System - Introduction
description: Introduction to ROS 2
---

# ROS 2 Nervous System (Weeks 3–5)

## Overview

ROS 2 serves as the "nervous system" for robots, providing communication, control, and coordination.

## Learning Path

- [Week 3: ROS 2 Basics](/docs/ros2/week3)
- [Week 4: Topics, Services, and Actions](/docs/ros2/week4)
- [Week 5: Advanced ROS 2 Concepts](/docs/ros2/week5)

## Diagrams

```mermaid
graph TD
    Node1[Node A] -- Topic --> Node2[Node B]
    Node2 -- Service Request --> Node3[Node C]
    Node3 -- Service Response --> Node2
    Node4[Node D] -- Action Goal --> Node5[Node E]
    Node5 -- Action Feedback --> Node4
    Node5 -- Action Result --> Node4
```
