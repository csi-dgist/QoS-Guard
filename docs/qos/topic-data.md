# TOPIC_DATA

Attaches application-specific metadata to the topic entity.

## What it controls

Opaque application-defined bytes on the Topic entity, propagated during topic discovery.

## Key settings

| Field | Default | Mutability |
|:---|:---|:---|
| `value` | empty sequence | Can be changed at runtime |

## Compatibility role

Discovery — topic-level metadata is visible before subscription. Does not participate in RxO matching; applications use it for schema or configuration checks.

## When this conflicts

No QoS Guard rule is triggered by Topic Data on its own. It still carries application metadata that other tools may read.

## Related policies

- [USER_DATA](user-data.md)
- [GROUP_DATA](group-data.md)

### Example

The TOPDATA QoS can be used by applications to verify schema compatibility in advance. For example, each robot can embed value such as schema=2.1 and frame=lidar in the TOPDATA of the scan cloud topic. During topic discovery, an inventory management application can read this information, and if the schema is incompatible, it can prevent subscription and avoid data parsing errors.
