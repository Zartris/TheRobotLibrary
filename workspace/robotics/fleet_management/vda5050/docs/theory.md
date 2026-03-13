# VDA 5050 Protocol — Theory

## Overview

VDA 5050 is a published industry standard for communication between autonomous mobile robots (AMRs) and a fleet management system (FMS). Version 2.0 is the current stable release. The protocol is transport-agnostic (designed for MQTT but applicable to any message bus or REST) and defines five canonical JSON message types.

## Message Types

| Type | Direction | Description |
|------|-----------|-------------|
| `Order` | FMS → Robot | Planned path with nodes (waypoints), edges (transitions), and actions |
| `InstantAction` | FMS → Robot | Immediate command: pause, resume, cancel, custom |
| `State` | Robot → FMS | Full robot state: position, battery, order status, safety state |
| `Connection` | Robot → FMS | Heartbeat / connection lifecycle event |
| `Visualization` | Robot → FMS | Optional high-frequency position stream for live display |

## Order Structure

An `Order` contains:
- **Header** — `headerId`, `timestamp`, `version`, `manufacturer`, `serialNumber`
- **Nodes** — ordered list of waypoints, each with `nodeId`, `sequenceId`, `x`, `y`, `theta`, `released`, `nodeActions[]`
- **Edges** — transitions between consecutive nodes, with `edgeId`, `startNodeId`, `endNodeId`, `maxSpeed`, `edgeActions[]`

## State Message

Key fields returned by the robot:
- `orderId`, `orderUpdateId`, `lastNodeId`, `lastNodeSequenceId`
- `driving` — whether the robot is moving
- `actionStates[]` — execution status of each action (waiting / initialising / running / finished / failed)
- `batteryState` — `batteryCharge`, `batteryVoltage`, `charging`
- `safetyState` — e-stop / field violation flags
- `errors[]`, `informations[]` — typed error / info messages with references

## Serialization Notes

All message types are serialized with **nlohmann/json** using to/from JSON ADL functions. Field names use `camelCase` per the VDA 5050 spec. Optional fields use `std::optional<T>` and serialize as absent (not null) when unset.

## References

- [VDA 5050 v2.0 Specification PDF](https://www.vda.de/dam/jcr:ed36b5b3-1b14-4e3a-b00e-fd76a98c9b89/vda5050_EN_2020.pdf)
- [VDA 5050 GitHub reference](https://github.com/VDA5050/VDA5050)
- nlohmann/json — `NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE` macro for ADL pairs
