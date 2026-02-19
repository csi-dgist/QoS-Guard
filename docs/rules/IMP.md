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
- **RMW/Implementation: FastDDS** 
```cpp
// Removal is performed by comparing the sourceTimestamp; if depth=1, existing older data is simply deleted.
// Therefore, as there is only one piece of archived data, there is nothing to compare it with, rendering the process meaningless.
// Try to substitute the oldest sample.
CacheChange_t* first_change = instance_changes.at(0);
if (a_change->sourceTimestamp >= first_change->sourceTimestamp)
{
// As the instance is ordered by source timestamp, we can always remove the first one.
ret_value = remove_change_sub(first_change);
}
else
{
// Received change is older than oldest, and should be discarded
return true;
}
```
- **RMW/Implementation: CycloneDDS** 
```cpp
// In BY_SOURCE_TIMESTAMP mode, samples that are "reversed relative to the source timestamp (i.e., past-time samples arriving late)" are discarded unconditionally.
// This behaviour results in retaining only the single most recent sample, rather than performing any sorting function.
static int inst_accepts_sample (const struct dds_rhc_default *rhc, const struct rhc_instance *inst, const struct ddsi_writer_info *wrinfo, const struct ddsi_serdata sample, const bool has_data)
{ if (rhc->by_source_ordering) {
  if (sample->timestamp.v > inst->tstamp.v)
{/ ok /}
else if (sample->timestamp.v < inst->tstamp.v)
{return 0;}
else if (inst_accepts_sample_by_writer_guid (inst, wrinfo))
{/ ok /}
else
{return 0;}}
if (rhc->exclusive_ownership && inst->wr_iid_islive && inst->wr_iid != wrinfo->iid)
{int32_t strength = wrinfo->ownership_strength;
if (strength > inst->strength) {
/ ok /
} else if (strength < inst->strength) {
return 0;
} else if (inst_accepts_sample_by_writer_guid (inst, wrinfo)) {
/ ok /
} else {return 0;}}
if (has_data && !content_filter_accepts (rhc->reader, sample, inst, wrinfo->iid, inst->iid))
{return 0;}
return 1;}
```
### Rule 9
- **RMW/Implementation: FastDDS** 
```cpp
// The KeepAll policy is capped at 1 by resourcelimits, meaning no new samples are added once the buffer is full.
// Consequently, with only one piece of data being retained, there is nothing to compare it with, rendering it meaningless.
(DataReaderHistory.cpp)
InstanceCollection::iterator vit;
if (find_key(a_change->instanceHandle, vit)) 
{ DataReaderInstance::ChangeCollection& instance_changes = vit->second->cache_changes; 
size_t total_size = instance_changes.size() + unknown_missing_changes_up_to; 
if (total_size < static_cast<size_t>(resource_limited_qos_.max_samples_per_instance)) 
{ 
return add_received_change_with_key(a_change, *vit->second); 
} 
logInfo(SUBSCRIBER, "Change not added due to maximum number of samples per instance"); }
```
- **RMW/Implementation: CycloneDDS** 
```cpp
// Set to Keep_all, but limited to 1 due to max_samples_per_instance
/* Check if resource max_samples QoS exceeded */ 
if (rhc->reader && rhc->max_samples != DDS_LENGTH_UNLIMITED && rhc->n_vsamples >= (uint32_t) rhc->max_samples) 
{ 
cb_data->raw_status_id = (int) DDS_SAMPLE_REJECTED_STATUS_ID; 
cb_data->extra = DDS_REJECTED_BY_SAMPLES_LIMIT; 
cb_data->handle = inst->iid; 
cb_data->add = true; return false;  
} 
/* Check if resource max_samples_per_instance QoS exceeded */ 
if (rhc->reader && rhc->max_samples_per_instance != DDS_LENGTH_UNLIMITED && inst->nvsamples >= (uint32_t) rhc->max_samples_per_instance) 
{ 
cb_data->raw_status_id = (int) DDS_SAMPLE_REJECTED_STATUS_ID; 
cb_data->extra = DDS_REJECTED_BY_SAMPLES_PER_INSTANCE_LIMIT; 
cb_data->handle = inst->iid; 
cb_data->add = true; return false; 
}
```
### Rule 10
- **RMW/Implementation: FastDDS**
```cpp
// Changing the owner in deadline_missed()
void deadline_missed() 
{ if (fastdds::rtps::c_Guid_Unknown != current_owner.first) 
{ if (alive_writers.remove_if([&](const WriterOwnership& item) 
{ return item.first == current_owner.first; })) 
{ 
current_owner.second = 0;
current_owner.first = fastdds::rtps::c_Guid_Unknown; 
if (alive_writers.empty() && (InstanceStateKind::ALIVE_INSTANCE_STATE == instance_state)) 
{ instance_state = InstanceStateKind::NOT_ALIVE_NO_WRITERS_INSTANCE_STATE; } 
if (ALIVE_INSTANCE_STATE == instance_state) 
{ update_owner(); } } } }

// However, assume that the `deadline_missed()` function is not infinite!
bool DataReaderImpl::deadline_timer_reschedule()
{
assert(qos_.deadline().period != dds::c_TimeInfinite);
```
- **RMW/Implementation: CycloneDDS** 
```cpp
// When a deadline is missed, inst->wr_iid_islive is reset to 0, thereby changing ownership. 
// However, if the deadline is set to infinity, inst->wr_iid_islive will never be reset to 0.
#endif /* DDS_HAS_LIFESPAN */
#ifdef DDS_HAS_DEADLINE_MISSED
ddsrt_mtime_t dds_rhc_default_deadline_missed_cb(void *hc, ddsrt_mtime_t tnow)
{ struct dds_rhc_default *rhc = hc; 
void *vinst; 
ddsrt_mtime_t tnext; 
ddsrt_mutex_lock (&rhc->lock); 
while ((tnext = deadline_next_missed_locked (&rhc->deadline, tnow, &vinst)).v == 0) 
{ struct rhc_instance *inst = vinst; deadline_reregister_instance_locked (&rhc->deadline, &inst->deadline, tnow); 
inst->wr_iid_islive = 0; 
ddsi_status_cb_data_t cb_data; 
cb_data.raw_status_id = (int) DDS_REQUESTED_DEADLINE_MISSED_STATUS_ID; 
cb_data.extra = 0; 
cb_data.handle = inst->iid; 
cb_data.add = true; 
ddsrt_mutex_unlock (&rhc->lock); 
dds_reader_status_cb (&rhc->reader->m_entity, &cb_data); ddsrt_mutex_lock (&rhc->lock); 
tnow = ddsrt_time_monotonic (); } 
ddsrt_mutex_unlock (&rhc->lock); 
return tnext;}
#endif /* DDS_HAS_DEADLINE_MISSED */
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
- **RMW/Implementation:FastDDS**
```cpp
// Keep_last removes the oldest sample when the depth is full.
 {// Try to substitute the oldest sample.
CacheChange_t* first_change = instance_changes.at(0);
if (a_change->sourceTimestamp >= first_change->sourceTimestamp){
// As the instance is ordered by source timestamp, we can always remove the first one.
ret_value = remove_change_sub(first_change);}
else{
// Received change is older than oldest, and should be discarded
return true;}}

// Looking at the lifespan timer code, it states that it "may already have been removed from the history".
CacheChange_t* earliest_change; 
while (history_.get_earliest_change(&earliest_change)) 
{ fastdds::rtps::Time_t expiration_ts = earliest_change->sourceTimestamp + qos_.lifespan().duration; 
// Check that the earliest change has expired (the change which started the timer could have been removed from the history) 
if (current_ts < expiration_ts) 
{ fastdds::rtps::Time_t interval = expiration_ts - current_ts; lifespan_timer_->update_interval_millisec(interval.to_ns() * 1e-6); return true; } 
// The earliest change has expired 
history_.remove_change_sub(earliest_change); 
try_notify_read_conditions(); } 
return false;
```
- **RMW/Implementation:CycloneDDS** 
```cpp
// A situation where a sample is overwritten and disappears due to history depth before it "expires" based on its lifespan.
// In the code, it deletes data using keep_last and then deletes it again based on lifespan.
if (inst->nvsamples == rhc->history_depth) { 
/* replace oldest sample; latest points to the latest one, the list is circular from old -> new, so latest->next is the oldest */ 
inst_clear_invsample_if_exists (rhc, inst, trig_qc); 
assert (inst->latest != NULL); s = inst->latest->next; 
assert (trig_qc->dec_conds_sample == 0); 
ddsi_serdata_unref (s->sample);
#ifdef DDS_HAS_LIFESPAN 
lifespan_unregister_sample_locked (&rhc->lifespan, &s->lifespan);
#endif 
trig_qc->dec_sample_read = s->isread; 
trig_qc->dec_conds_sample = s->conds; 
if (s->isread) 
{ inst->nvread--; rhc->n_vread--; } }
```
### Rule 18
- **RMW/Implementation: FastDDS** 
```cpp
// Keep_All waits until an ACK is received confirming that the oldest data has been successfully delivered to the other party before deleting the data.
else if (history_qos_.kind == KEEP_ALL_HISTORY_QOS) 
{ if (vit->second.cache_changes.size() < static_cast<size_t>(resource_limited_qos_.max_samples_per_instance)) 
{ add = true; } 
else { 
SequenceNumber_t seq_to_remove = vit->second.cache_changes.front()->sequenceNumber; 
if (!mp_writer->wait_for_acknowledgement(seq_to_remove, max_blocking_time, lock)) { 
// Timeout waiting. Will not add change to history. 
break; } 
// vit may have been invalidated 
if (!find_or_add_key(change->instanceHandle, change->serializedPayload, &vit)) 
{ break; } 
// If the change we were trying to remove was already removed, try again 
if (vit->second.cache_changes.empty() || vit->second.cache_changes.front()->sequenceNumber != seq_to_remove) 
{ continue; } 
// Remove change if still present 
add = remove_change_pub(vit->second.cache_changes.front()); } }

// Looking at the lifespan timer code, it states that it "may already have been removed from the history".
CacheChange_t* earliest_change; 
while (history_.get_earliest_change(&earliest_change)) 
{ fastdds::rtps::Time_t expiration_ts = earliest_change->sourceTimestamp + qos_.lifespan().duration; 
// Check that the earliest change has expired (the change which started the timer could have been removed from the history) 
if (current_ts < expiration_ts) 
{ fastdds::rtps::Time_t interval = expiration_ts - current_ts; lifespan_timer_->update_interval_millisec(interval.to_ns() * 1e-6); return true; } 
// The earliest change has expired 
history_.remove_change_sub(earliest_change); 
try_notify_read_conditions(); } 
return false;
```
- **RMW/Implementation: CycloneDDS** 
```cpp
// Although set to Keep_all, due to max_samples_per_instance, samples are not saved at all before their lifespan expires, meaning no samples are deleted by lifespan.
/* Check if resource max_samples QoS exceeded */ 
if (rhc->reader && rhc->max_samples != DDS_LENGTH_UNLIMITED && rhc->n_vsamples >= (uint32_t) rhc->max_samples) 
{ 
cb_data->raw_status_id = (int) DDS_SAMPLE_REJECTED_STATUS_ID; 
cb_data->extra = DDS_REJECTED_BY_SAMPLES_LIMIT; 
cb_data->handle = inst->iid; 
cb_data->add = true; return false; 
} 
/* Check if resource max_samples_per_instance QoS exceeded */ 
if (rhc->reader && rhc->max_samples_per_instance != DDS_LENGTH_UNLIMITED && inst->nvsamples >= (uint32_t) rhc->max_samples_per_instance) 
{ 
cb_data->raw_status_id = (int) DDS_SAMPLE_REJECTED_STATUS_ID; 
cb_data->extra = DDS_REJECTED_BY_SAMPLES_PER_INSTANCE_LIMIT; 
cb_data->handle = inst->iid; 
cb_data->add = true; return false; 
}
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
