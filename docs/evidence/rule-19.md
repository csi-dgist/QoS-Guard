# Manual enable with volatile durability drops pre-enable data

<p class="rule-ref-line">Rule 19 &middot; applies to publishers and subscribers &middot; <a href="../../rules/">Back to all rules</a></p>

Wastes effort and loses early data. Samples written before the entity is manually enabled are not retained under volatile durability.

<div class="rule-conflict-callout rule-conflict-resource">
<div class="rule-conflict-settings">If you set <b>Entity Factory autoenable_created_entities = false</b> together with <b>Durability = VOLATILE</b></div>
<div class="rule-consequence rule-consequence-resource">Wastes resources</div>
</div>

- Settings involved: <a href="../../qos/entity-factory/">Entity Factory</a> and <a href="../../qos/durability/">Durability</a>
- What QoS Guard checks: `[DURABL = VOLATILE] ∧ [autoenable = FALSE]`

## Example

You create a writer with autoenable off and publish before calling enable(). With volatile durability those early samples are gone.

## How to fix it

Enable the entity before publishing, or use TRANSIENT_LOCAL durability if pre-enable samples must survive.

## Why this rule is flagged

#### What the DDS specification says

| OMG DDS § | QoS policy | Standard statement |
|:---|:---|:---|
| §2.2.3.20 | ENTITY_FACTORY | The `autoenable_created_entities` flag controls whether entities created by a factory are automatically enabled. If it is false, created entities must be explicitly enabled before they participate in communication. |

<hr class="evidence-subsection-divider">

#### What the engine source code shows

Fast DDS gates newly created readers through `ENTITY_FACTORY.autoenable_created_entities`; if it is false, the reader remains disabled until explicitly enabled.

!!! note "Fast DDS implementation evidence"
    ```cpp
    // SubscriberImpl.cpp: entity factory auto-enable gate
    if (user_subscriber_->is_enabled() &&
            qos_.entity_factory().autoenable_created_entities)
    {
        if (ReturnCode_t::RETCODE_OK != reader->enable())
        {
            delete_datareader(reader);
            return nullptr;
        }
    }
    ```

Cyclone DDS does not implement this dynamic entity-enabling gate and forces entities into the enabled state.

| Item | Value |
|:---|:---|
| Dataset | [Download CSV](../data/evidence/rule-19/rule-19-data.csv) |
| Fixed QoS setting | None |
| Tested variable | `ENTFAC.autoenable_created_entities`, `DURABL.kind` |
| Tested values | `ENTFAC.autoenable ∈ {true, false}`, `DURABL ∈ {VOLATILE, TRANSIENT_LOCAL}` |
| Rule-relevant case | `ENTFAC.autoenable = false`, `DURABL = VOLATILE` |
| Tested engines / versions | Fast DDS 2.6.11 (Humble), Fast DDS 2.14.6 (Jazzy), Cyclone DDS 0.10.5 |
| Network setting | `RTT = 1 ms`, `loss = 0%`, `PP = 50 ms`, `message size = 256 B` |

| Engine | Tested setting | Observed behavior |
|:---|:---|:---|
| Fast DDS 2.6.11 (Humble) | `ENTFAC.autoenable ∈ {true, false}`, `DURABL ∈ {VOLATILE, TRANSIENT_LOCAL}` | Profile accepted, matched, and delivered; `late_join_recovered = 1` |
| Fast DDS 2.14.6 (Jazzy) | `ENTFAC.autoenable ∈ {true, false}`, `DURABL ∈ {VOLATILE, TRANSIENT_LOCAL}` | Profile accepted, matched, and delivered; `late_join_recovered = 1` |
| Cyclone DDS 0.10.5 | `ENTFAC.autoenable ∈ {true, false}`, `DURABL ∈ {VOLATILE, TRANSIENT_LOCAL}` | Profile accepted, matched, and delivered; `late_join_recovered = 1` |

<hr class="evidence-subsection-divider">

#### What you'll see when you run it

QoS Guard flags this as an operational conflict:

```text
[OPERATIONAL] [tf_sub] [SUB] DURABL=VOLATILE <-> autoenable=FALSE
```

At runtime the entity stays disabled until the application explicitly enables it, and anything written before that moment is not stored under volatile durability. Fast DDS gates the reader through the entity-factory flag and leaves it disabled until enable() is called, so early samples are simply lost. The pre-enable publishing effort is wasted because volatile durability keeps nothing for a late enable to recover.
