# GROUP_DATA

Attaches application-specific metadata to the Publisher and Subscriber entities.

## What it controls

Opaque application-defined bytes on Publisher or Subscriber entities, visible to remote endpoints during discovery.

## Key settings

| Field | Default | Mutability |
|:---|:---|:---|
| `value` | empty sequence | Can be changed at runtime |

## Compatibility role

Discovery — metadata is exchanged at the Publisher/Subscriber level but does not enforce DDS matching. Applications read values in discovery callbacks to segment or filter flows.

## When this conflicts

No QoS Guard rule is triggered by Group Data on its own. It still carries application metadata that other tools may read.

## Related policies

- [PARTITION](partition.md)
- [USER_DATA](user-data.md)
- [TOPIC_DATA](topic-data.md)

### Example

The GRPDATA QoS can be used to logically segment data flows in a manner similar to the PART QoS. For example, if delivery and inventory robots share topics within the same domain, assigning value=delivery or value=inventory to each Publisher or Subscriber allows the central management server to read these values during discovery callbacks. Although similar to PART, the key distinction is that PART enforces matching at the DDS level, whereas GRPDATA leaves the interpretation of the field entirely to the application.
