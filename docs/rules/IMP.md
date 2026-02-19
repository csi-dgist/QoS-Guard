# IMP Rules

This page describes the QoS dependency rules derived from the specific implementation behaviors of ROS 2 Middlewares (RMWs) such as eProsima Fast DDS and Eclipse Cyclone DDS. These dependencies are not explicitly mandated by the DDS standard but are critical for functional consistency in practice.

---

## Stage 1
*In..*

| No. | Identifier | QoS Conflict Condition (Violation) | Dependency | Entity | Basis |
|:---:|:---|:---|:---:|:---:|:---:|
| [3](#rule-3) | RELIAB → DURABL | $[DURABL \ge TRAN\_LOCAL] \wedge [RELIAB = BEST\_EFFORT]$ | Functional | Pub, Sub | IMP |
| [4](#rule-4) | RELIAB → OWNST | $[OWNST = EXCLUSIVE] \wedge [RELIAB = BEST\_EFFORT]$ | Functional | Pub, Sub | IMP |
| [5](#rule-5) | RELIAB → LIVENS | $[LIVENS = MANUAL] \wedge [RELIAB = BEST\_EFFORT]$ | Functional | Pub, Sub | IMP |
| [7](#rule-7) | LFSPAN → DEADLN | $LFSPAN.duration < DEADLN.period$ | Functional | Sub | IMP |
| [8](#rule-8) | HIST → DESTORD | $[DESTORD = BY\_SOURCE] \wedge [HIST.kind = KEEP\_LAST] \wedge [depth = 1]$ | Functional | Sub | IMP |
| [9](#rule-9) | RESLIM → DESTORD | $[DESTORD = BY\_SOURCE] \wedge [KEEP\_ALL] \wedge [mpi = 1]$ | Functional | Sub | IMP |
| [10](#rule-10) | DEADLN → OWNST | $[OWNST = EXCLUSIVE] \wedge [DEADLN.period = \infty]$ | Functional | Sub | IMP |
| [11](#rule-11) | LIVENS → OWNST | $[OWNST = EXCLUSIVE] \wedge [LIVENS.lease = \infty]$ | Functional | Sub | IMP |
| [12](#rule-12) | LIVENS → RDLIFE | $[autopurge\_nowriter > 0] \wedge [LIVENS.lease = \infty]$ | Functional | Sub | IMP |
| [13](#rule-13) | RDLIFE → DURABL | $[DURABL \ge TRANSIENT] \wedge [autopurge\_disposed \neq \infty]$ | Functional | Sub | IMP |
| [14](#rule-14) | PART → DEADLN | $[DEADLN.period > 0] \wedge [PART.names \neq \emptyset]$ | Functional | Sub | IMP |
| [15](#rule-15) | PART → LIVENS | $[LIVENS = MANUAL] \wedge [PART.names \neq \emptyset]$ | Functional | Sub | IMP |
| [16](#rule-16) | OWNST → WDLIFE | $[autodispose = TRUE] \wedge [OWNST = EXCLUSIVE]$ | Functional | Sub | IMP |
| [17](#rule-17) | HIST → LFSPAN | $[HIST.KEEP\_LAST] \wedge [LFSPAN.duration > HIST.depth \times PP]$ | Operational | Pub, Sub | IMP |
| [18](#rule-18) | RESLIM → LFSPAN | $[KEEP\_ALL] \wedge [LFSPAN.duration > mpi \times PP]$ | Operational | Pub, Sub | IMP |
| [19](#rule-19) | ENTFAC → DURABL | $[DURABL \neq VOLATILE] \wedge [autoenable = FALSE]$ | Operational | Pub, Sub | IMP |
| [20](#rule-20) | PART → DURABL | $[DURABL \ge TRAN\_LOCAL] \wedge [PART.names \neq \emptyset]$ | Operational | Pub, Sub | IMP |

---

## Stage 2
*Dec.*

| No. | Identifier | QoS Conflict Condition (Violation) | Dependency | Entity | Basis |
|:---:|:---|:---|:---:|:---:|:---:|
| [28](#rule-28) | WDLIFE → RDLIFE | $[W.autodispose = FALSE] \wedge [R.autopurge\_nowriter = 0]$ | Functional | Pub ↔ Sub | IMP |
| [29](#rule-29) | WDLIFE → RDLIFE | $[W.autodispose = FALSE] \wedge [R.autopurge\_disposed > 0]$ | Operational | Pub ↔ Sub | IMP |
| [30](#rule-30) | WDLIFE → RDLIFE | $[W.autodispose = FALSE] \wedge [R.autopurge\_nowriter = \infty]$ | Operational | Pub ↔ Sub | IMP |

---

## Implementation Evidence Details
*Below are the code-level justifications and source references for each IMP rule.*


### Rule 3
- **RMW/Implementation:** - **Source File:** - **Code Snippet:**
```cpp
// TODO: Insert relevant code from FastDDS or CycloneDDS
```
### Rule 4
- **RMW/Implementation:** - **Source File:** - **Code Snippet:**
```cpp
// TODO: Insert relevant code from FastDDS or CycloneDDS
```
### Rule 5
- **RMW/Implementation:** - **Source File:** - **Code Snippet:**
```cpp
// TODO: Insert relevant code from FastDDS or CycloneDDS
```
### Rule 7
- **RMW/Implementation: FastDDS**
```cpp
// The Reader's Lifespan and Deadline timers share the same History 
!History 
detail::DataReaderHistory history_;
```
- **RMW/Implementation: CycloneDDS**
```cpp
// Both timers operate independently

// Deadline
ddsrt_mtime_t deadline_next_missed_locked (struct deadline_adm *deadline_adm, ddsrt_mtime_t tnow, void **instance)
{
struct deadline_elem *elem = NULL;
if (!ddsrt_circlist_isempty (&deadline_adm->list))
{
struct ddsrt_circlist_elem *list_elem = ddsrt_circlist_oldest (&deadline_adm->list);
elem = DDSRT_FROM_CIRCLIST (struct deadline_elem, e, list_elem);
if (elem->t_deadline.v <= tnow.v)
{
ddsrt_circlist_remove (&deadline_adm->list, &elem->e);
if (instance != NULL)
*instance = (char *)elem - deadline_adm->elem_offset;
return (ddsrt_mtime_t) { 0 };
}
}
if (instance != NULL)
*instance = NULL;
return (elem != NULL) ? elem->t_deadline : DDSRT_MTIME_NEVER;
}

// Lifespan
ddsrt_mtime_t lifespan_next_expired_locked (const struct lifespan_adm *lifespan_adm, ddsrt_mtime_t tnow, void **sample)
{
struct lifespan_fhnode *node;
if ((node = ddsrt_fibheap_min(&lifespan_fhdef, &lifespan_adm->ls_exp_heap)) != NULL && node->t_expire.v <= tnow.v)
{
*sample = (char *)node - lifespan_adm->fhn_offset;
return (ddsrt_mtime_t) { 0 };
}
*sample = NULL;
return (node != NULL) ? node->t_expire : DDSRT_MTIME_NEVER;
}
```
### Rule 8
- **RMW/Implementation:** - **Source File:** - **Code Snippet:**
```cpp
// TODO: Insert relevant code from FastDDS or CycloneDDS
```
### Rule 9
- **RMW/Implementation:** - **Source File:** - **Code Snippet:**
```cpp
// TODO: Insert relevant code from FastDDS or CycloneDDS
```
### Rule `0
- **RMW/Implementation:** - **Source File:** - **Code Snippet:**
```cpp
// TODO: Insert relevant code from FastDDS or CycloneDDS
```
### Rule 11
- **RMW/Implementation:** - **Source File:** - **Code Snippet:**
```cpp
// TODO: Insert relevant code from FastDDS or CycloneDDS
```
### Rule 12
- **RMW/Implementation:** - **Source File:** - **Code Snippet:**
```cpp
// TODO: Insert relevant code from FastDDS or CycloneDDS
```
### Rule 13
- **RMW/Implementation:** - **Source File:** - **Code Snippet:**
```cpp
// TODO: Insert relevant code from FastDDS or CycloneDDS
```
### Rule 14
- **RMW/Implementation:** - **Source File:** - **Code Snippet:**
```cpp
// TODO: Insert relevant code from FastDDS or CycloneDDS
```
### Rule 15
- **RMW/Implementation:** - **Source File:** - **Code Snippet:**
```cpp
// TODO: Insert relevant code from FastDDS or CycloneDDS
```
### Rule 16
- **RMW/Implementation:** - **Source File:** - **Code Snippet:**
```cpp
// TODO: Insert relevant code from FastDDS or CycloneDDS
```
### Rule 17
- **RMW/Implementation:** - **Source File:** - **Code Snippet:**
```cpp
// TODO: Insert relevant code from FastDDS or CycloneDDS
```
### Rule 18
- **RMW/Implementation:** - **Source File:** - **Code Snippet:**
```cpp
// TODO: Insert relevant code from FastDDS or CycloneDDS
```
### Rule 19
- **RMW/Implementation:** - **Source File:** - **Code Snippet:**
```cpp
// TODO: Insert relevant code from FastDDS or CycloneDDS
```
### Rule 20
- **RMW/Implementation:** - **Source File:** - **Code Snippet:**
```cpp
// TODO: Insert relevant code from FastDDS or CycloneDDS
```
### Rule 28
- **RMW/Implementation:** - **Source File:** - **Code Snippet:**
```cpp
// TODO: Insert relevant code from FastDDS or CycloneDDS
```
### Rule 29
- **RMW/Implementation:** - **Source File:** - **Code Snippet:**
```cpp
// TODO: Insert relevant code from FastDDS or CycloneDDS
```
### Rule 30
- **RMW/Implementation:** - **Source File:** - **Code Snippet:**
```cpp
// TODO: Insert relevant code from FastDDS or CycloneDDS
```
