# USER_DATA

Allows application-specific metadata to be attached to DDS entities such as Domain Participant, Publisher, and Subscriber.

## What it controls

Opaque application-defined bytes carried on Participant, Publisher, or Subscriber entities and exchanged during discovery.

## Key settings

| Field | Default | Mutability |
|:---|:---|:---|
| `value` | empty sequence | Can be changed at runtime |

## Compatibility role

Discovery — metadata is visible during SPDP/SEDP but is not used for DDS-level matching. Interpretation is entirely application-defined.

## When this conflicts

No QoS Guard rule is triggered by User Data on its own. It still carries application metadata that other tools may read.

## Related policies

- [GROUP_DATA](group-data.md)
- [TOPIC_DATA](topic-data.md)

### Example

The USRDATA QoS can be used to flexibly deliver identity, authentication, and configuration information without requiring additional topics or separate domains. For example, each robot may embed value such as robot id=R12 and token=ABCD123 in its participant, allowing the server to inspect the token during the SPDP phase and admit only authorized robots while blocking others. Similarly, a Publisher for a LiDAR topic may include value such as sensor=LiDAR and fov=270 in its USRDATA, enabling the subscribing application to determine sensor configuration and immediately select an appropriate filtering strategy before receiving any samples.
