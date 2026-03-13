# VDA 5050 Types

C++ structs and nlohmann/json serialization for the VDA 5050 v2.0 AGV/AMR interface
specification. Provides the data model for all FMS ↔ robot communication.

## Message Types

| Type | Direction | File |
|------|-----------|------|
| `Order` | FMS → Robot | `include/vda5050/order.hpp` |
| `InstantAction` | FMS → Robot | `include/vda5050/instant_action.hpp` |
| `State` | Robot → FMS | `include/vda5050/state.hpp` |
| `Connection` | Robot → FMS | `include/vda5050/connection.hpp` |
| `Visualization` | Robot → FMS | `include/vda5050/visualization.hpp` |

## Usage

```cpp
#include <vda5050/order.hpp>
#include <nlohmann/json.hpp>

// Parse incoming order JSON
auto json = nlohmann::json::parse(raw_payload);
auto order = json.get<vda5050::Order>();

// Serialize robot state to JSON for WS broadcast
vda5050::State state{...};
auto payload = nlohmann::json(state).dump();
```
