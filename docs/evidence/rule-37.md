# Unbounded durable writer cache under keep-all

<p class="rule-ref-line">Rule 37 &middot; applies to the publisher &middot; <a href="../../rules/">Back to all rules</a></p>

Wastes resources. The durable writer cache keeps every sample to serve future late joiners, so memory grows without bound and late-join recovery gets slower. Observed on Cyclone DDS.

<div class="rule-conflict-callout rule-conflict-resource">
<div class="rule-conflict-settings">If you set <b>Durability = TRANSIENT_LOCAL with History = KEEP_ALL</b> together with <b>large or default resource limits</b></div>
<div class="rule-consequence rule-consequence-resource">Wastes resources</div>
</div>

- Settings involved: <a href="../../qos/history/">History</a> and <a href="../../qos/durability/">Durability</a>
- What QoS Guard checks: `[DURABL ≥ TRAN_LOCAL] ∧ [KEEP_ALL] ∧ [RESLIM.mpi ≥ default]`

## Example

A high-rate KEEP_ALL transient-local writer runs for hours and its history cache grows until memory is exhausted.

## How to fix it

Bound the writer with finite resource limits or a Lifespan, or use KEEP_LAST when you do not need full history.

## Why this rule is flagged

#### What the DDS specification says

The DDS specification does not settle this case on its own, so the rule rests on the engine's implementation and direct measurement.

<hr class="evidence-subsection-divider">

#### What the engine source code shows

Cyclone DDS treats a `TRANSIENT_LOCAL` writer with unbounded transient-local depth as a cache that must not delete historical samples.

!!! note "Cyclone DDS implementation evidence"
    ```c
    // dds_whc.c: transient_local + tldepth == 0 prevents deletion.
    if (whc->wrinfo.is_transient_local && whc->wrinfo.tldepth == 0)
    {
        /* If transient local data is not allowed to be deleted
           we can never ever delete anything. */
        return 0;
    }
    ```

<hr class="evidence-subsection-divider">

#### What the measurements show

Measurement of the durable late-join recovery shows the recovered backlog rising with the retained-sample budget (RESLIM.mpi) under keep-all history, because every sample is kept to serve a future late joiner. When that budget is large or left at its default, nothing bounds the writer cache, so it keeps growing and sends a late joiner a large backlog during recovery. In the measurement the backlog was bounded only by whichever of the resource limit or the sample lifespan was reached first.

| Measurement | Finding |
|---|---|
| Setup | keep_all history with transient_local durability on Cyclone DDS |
| Result | The writer cache never deletes a sample, so the recovered late-join backlog rises with RESLIM.mpi |
| Bound | Whichever of the resource limit or the sample lifespan is reached first |
