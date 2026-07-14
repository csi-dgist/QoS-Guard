# Dependency Map

QoS policies do not act alone. The value you choose for one policy can quietly change what another policy does, and that interaction is where many surprising QoS bugs come from. This map shows which policies pull on each other, so you can spot the risky pairs before you tune them.

## How to use the map

- Each **box** is a QoS policy. Each **arrow** is a rule, a specific combination QoS Guard flags.
- **Click an arrow** to isolate that one rule. The map hides everything else and shows only the two policies involved, and a panel explains the rule and links to its page.
- **Click a policy box** to see just that policy's conflicts.
- Use **Back to map** (or click an empty area) to return to the full picture.

Arrow colour shows what the conflict does.

<div class="dependency-graph-wrap" id="dependency-graph"></div>

<div class="dependency-graph-legend">
  <span class="dependency-graph-legend-item dependency-graph-legend-s">Won't connect</span>
  <span class="dependency-graph-legend-item dependency-graph-legend-f">Breaks a guarantee</span>
  <span class="dependency-graph-legend-item dependency-graph-legend-o">Wastes resources</span>
  <span class="dependency-graph-legend-hint">Click an arrow to isolate a rule. Click a policy box to see its conflicts.</span>
</div>

Prefer a searchable list? The [Rules](rules/index.md) page lists all 40 rules as filterable cards. To read what a single policy does and every rule it takes part in, open its page in the [QoS Encyclopedia](qos/index.md).
