---
title: Conversational Robotics - Introduction
---

# Conversational Robotics (Week 13)

## Overview

Enable robots to understand and respond to natural language.

## Learning Path

- [Week 13: Voice and Language Interaction](/docs/conversational-robotics/week13)

## Diagrams

```mermaid
graph TD
    Voice[Voice Input] --> ASR[ASR (Speech to Text)]
    ASR --> NLU[NLU (Understand Intent)]
    NLU --> DM[Dialogue Manager]
    Vision[Visual Perception] --> VLM[Vision-Language Model]
    VLM -- Context --> DM
    DM --> NLG[NLG (Generate Response)]
    NLG --> TTS[TTS (Text to Speech)]
    TTS --> RobotOutput[Robot Voice Output/Action]
```
