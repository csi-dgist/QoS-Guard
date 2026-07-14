# ENTITY_FACTORY

Controls whether newly created DDS entities automatically start participating in discovery.

## What it controls

Whether child entities created under a DomainParticipant are immediately enabled for discovery, or require an explicit `enable()` call before they can participate.

* **TRUE**: Newly created child entities are immediately enabled and begin participating in discovery.
* **FALSE**: The application must explicitly call enable() before the entity can participate in discovery.

## Key settings

| Field | Default | Mutability |
|:---|:---|:---|
| `autoenable_created_entities` | TRUE | Can be changed at runtime |

## Compatibility role

Discovery timing — sets when entities begin PDP/EDP participation. Does not participate in Writer–Reader RxO matching.

## When this conflicts

Entity Factory takes part in the following QoS Guard rules. Each links to a full explanation with an example and a fix.

<div class="conflict-list">
<a class="conflict-item" href="../../evidence/rule-19/">
<span class="conflict-title">Manual enable with volatile durability drops pre-enable data</span>
<span class="conflict-meta"><span class="conflict-rel">with Durability</span><span class="rule-consequence rule-consequence-resource">Wastes resources</span></span>
</a>
</div>

## Related policies

- [DURABILITY](durability.md)

### Example

The ENTFAC QoS can be used to conserve resources and allow multiple robots to initiate discovery simultaneously under synchronized conditions. For instance, the Publishers and Subscribers of a navigation module may be activated only after completing local sensor calibration or localization. By setting autoenable created entities=false, the system delays communication until the robot is ready by explicitly calling enable() at the appropriate time.
