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
- **RMW/Implementation: FastDDS** 
```cpp
// [TRANSIENT_LOCAL late-joiner logic resides within if (is_reliable)]
// → When best-effort, it returns false, preventing the later-joiner logic from executing → Transient behaviour does not occur
bool is_reliable = rp->is_reliable(); 
if (is_reliable) 
{ SequenceNumber_t min_seq = get_seq_num_min(); 
SequenceNumber_t last_seq = get_seq_num_max(); 
RTPSMessageGroup group(mp_RTPSParticipant, this, rp->message_sender()); 
// History not empty 
if (min_seq != SequenceNumber_t::unknown()) 
{ (void)last_seq; assert(last_seq != SequenceNumber_t::unknown()); assert(min_seq <= last_seq);
 try { 
// Late-joiner 
if (TRANSIENT_LOCAL <= rp->durability_kind() && TRANSIENT_LOCAL <= m_att.durabilityKind)
```
- **RMW/Implementation: CycloneDDS** 
```cpp
// Providing historical data to late joiners is only performed when it is more efficient than best-effort processing.
  /* Store available data into the late joining reader when it is reliable (we don't do
historical data for best-effort data over the wire, so also not locally). */
if (rd->xqos->reliability.kind > DDS_RELIABILITY_BEST_EFFORT && rd->xqos->durability.kind > DDS_DURABILITY_VOLATILE)
ddsi_deliver_historical_data (wr, rd);
```
### Rule 4
- **RMW/Implementation: FastDDS** 
```cpp
// Only the FastDDS code contains code that checks this rule.
if (m_reliability.kind == BEST_EFFORT_RELIABILITY_QOS && m_ownership.kind == EXCLUSIVE_OWNERSHIP_QOS) 
{
logError(RTPS_QOS_CHECK, "BEST_EFFORT incompatible with EXCLUSIVE ownership"); 
return false; 
} 
return true;
```
### Rule 5
- **RMW/Implementation: FastDDS** 
```cpp
// When using best effort, a StatelessWriter is created.
// When using Manual_by_topic, a heartbeat is sent only if it is a StatefulWriter ⇒ No heartbeat is sent.
if (qos_.liveliness().kind == MANUAL_BY_TOPIC_LIVELINESS_QOS) 
{ 
// As described in the RTPS specification, if liveliness kind is manual a heartbeat must be sent // This only applies to stateful writers, as stateless writers do not send heartbeats 
StatefulWriter* stateful_writer = dynamic_cast<StatefulWriter*>(writer_); 
if (stateful_writer != nullptr) 
{ stateful_writer->send_periodic_heartbeat(true, true); } } 
return RETCODE_OK;
```
- **RMW/Implementation: CycloneDDS** 
```cpp
// Only when reliable → HB transmission
// When best-effort, loss is possible, so live link status determination may be incorrect
/* heartbeat event will be deleted when the handler can't find a
writer for it in the hash table. NEVER => won't ever be
scheduled, and this can only change by writing data, which won't
happen until after it becomes visible. */
if (wr->reliable)
wr->heartbeat_xevent = qxev_heartbeat (wr->evq, DDSRT_MTIME_NEVER, &wr->e.guid);
else
wr->heartbeat_xevent = NULL;
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
- **RMW/Implementation: FastDDS**
```cpp
// If the lease duration is infinite, the writer is not registered. Consequently, lease expiry → not alive does not occur → owner change fails.
if (liveliness_lease_duration_ < c_TimeInfinite)
{
auto wlp = this->mp_RTPSParticipant->wlp();
if ( wlp != nullptr)
{ 
wlp->sub_liveliness_manager_->add_writer(
wdata.guid(),
liveliness_kind_,
liveliness_lease_duration_);
}
```
### Rule 12
- **RMW/Implementation: FastDDS**
```cpp
// livelinss lost switched to NOT_ALIVE
// However, if set to infinite, the transition does not occur, so the sample cannot be automatically removed.
bool LivelinessManager::timer_expired()
{ std::unique_lock<std::mutex> lock(mutex_); 
if (timer_owner_ == nullptr) 
{ EPROSIMA_LOG_ERROR(RTPS_WRITER, "Liveliness timer expired but there is no writer"); 
return false; } 
else { timer_owner_->status = LivelinessData::WriterStatus::NOT_ALIVE; }
```
- **RMW/Implementation: CycloneDDS**
```cpp
// Upon expiry of the lease term,
case DDSI_EK_PROXY_WRITER:
ddsi_proxy_writer_set_notalive ((struct ddsi_proxy_writer *) l->entity, true);
break;
case DDSI_EK_WRITER:
ddsi_writer_set_notalive ((struct ddsi_writer *) l->entity, true);
break;
// However, if set to infinite, it will not expire, so the data deletion function will not operate.
```
### Rule 13
- **RMW/Implementation: FastDDS** 
```cpp
// Stored in permanent storage
void StatefulPersistentReader::set_last_notified(
const GUID_t& writer_guid,
const SequenceNumber_t& seq)
{
history_state_->history_record[writer_guid] = seq;
persistence_->update_writer_seq_on_storage(persistence_guid_, writer_guid, seq);
}
// But the data vanished in an instant.
Duration_t autopurge_no_writer_samples_delay;
/**
* @brief Indicates the duration the DataReader must retain information regarding instances that have the
* instance_state NOT_ALIVE_DISPOSED. <br>
* By default, c_TimeInfinite.
*/
```
### Rule 14
- **RMW/Implementation: FastDDS** 
```cpp
// Partition change → Unmatching → Call ddsi_rhc_unregister_wr (rd->rhc, &wrinfo);
// ⇒ Instance is immediately unregistered from the deadline. → Deadline timer is not updated.
if (inst->deadline_reg)
{
inst->deadline_reg = 0;
deadline_unregister_instance_locked (&rhc->deadline, &inst->deadline);
}
// During the process of dynamically changing partitions, instances are removed from the deadline list due to writer unregistration.
// Since the timer is not reset to the "next expiry time" upon unregistration, the deadline timer ceases to function once the list becomes empty.
```
### Rule 15
- **RMW/Implementation: FastDDS**
```cpp
// The writer signals liveliness, but if a reader becomes involved in the partition dynamic change process, it no longer receives updates to the writer's liveliness
// → Subsequently, when the lease expires, the notification is sent only to readers that are "still matched".
// However, if there are no readers, the liveliness notification is disabled even during the partition dynamic change process.
static void proxy_writer_notify_liveliness_change_may_unlock (struct ddsi_proxy_writer *pwr)
{ 
struct ddsi_alive_state alive_state; 
proxy_writer_get_alive_state_locked (pwr, &alive_state); 
struct ddsi_guid rdguid; 
struct ddsi_pwr_rd_match *m; memset (&rdguid, 0, sizeof (rdguid)); 
while (pwr->alive_vclock == alive_state.vclock && (m = ddsrt_avl_lookup_succ (&ddsi_pwr_readers_treedef, &pwr->readers, &rdguid)) != NULL) 
{ rdguid = m->rd_guid; ddsrt_mutex_unlock (&pwr->e.lock); 
/* unlocking pwr means alive state may have changed already; we break out of the loop once we detect this but there for the reader in the current iteration, anything is possible */ 
ddsi_reader_update_notify_pwr_alive_state_guid (&rdguid, pwr, &alive_state); 
ddsrt_mutex_lock (&pwr->e.lock); }}
// → The recipients of the notification are only the readers listed in pwr→readers.
```
### Rule 16
- **RMW/Implementation: FastDDS** 
```cpp
// The system is designed so that not just anyone can delete data;
// only the sender with the most powerful permissions can alter the status.
bool writer_dispose
( const fastrtps::rtps::GUID_t& writer_guid, const uint32_t ownership_strength) 
{ bool ret_val = false; writer_set(writer_guid, ownership_strength); 
if (ownership_strength >= current_owner.second) 
{ current_owner.first = writer_guid; current_owner.second = ownership_strength; 
if (InstanceStateKind::ALIVE_INSTANCE_STATE == instance_state) 
{ ret_val = true; instance_state = InstanceStateKind::NOT_ALIVE_DISPOSED_INSTANCE_STATE; } } 
return ret_val; }
```
- **RMW/Implementation: CycloneDDS** 
```cpp
// Only writers with a stronger strength can now modify the instance's state.
static int inst_accepts_sample (const struct dds_rhc_default *rhc, const struct rhc_instance *inst, const struct ddsi_writer_info *wrinfo, const struct ddsi_serdata sample, const bool has_data)
{if (rhc->by_source_ordering){
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
/ ok */
} else {
return 0;}}
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
- **RMW/Implementation: FastDDS**
```cpp
// autoenable_created_entities = FALSE means entities are not automatically enabled.
publisher_->rtps_participant()->register_writer(writer_, topic_desc, wqos);
// ⇒ Only within enable() are register_writer() / register_reader() called to register with discovery.
// As VOLATILE entities do not become existing until enabled, any previously sent data is discarded.
// When set to Volatile, upon enabling, all previous data will not be received. 
```
- **RMW/Implementation: CycloneDDS**
```cpp
// When set to VOLATILE, previous data is not fully retrieved.
// The implementation where autoenable_created_entities=FALSE is not present.
if(qos->durability.kind == DDS_DURABILITY_VOLATILE) 
{ opts.historyCapacity = 0; } 
else 
{ 
// Transient Local and stronger  
if (qos->durability_service.history.kind == DDS_HISTORY_KEEP_LAST) 
{ 
opts.historyCapacity = (uint64_t)qos->durability_service.history.depth;
 } 
else { opts.historyCapacity = 0; } }
```
### Rule 20
- **RMW/Implementation: FastDDS**
```cpp
// Upon successful matching, it may be treated as a later-join, potentially resulting in duplicate data being received.
if (!matched) //Different partitions
{
EPROSIMA_LOG_WARNING(RTPS_EDP, "INCOMPATIBLE QOS (topic: " << rdata->topic_name << "): Different Partitions");
reason.set(MatchingFailureMask::partitions);
}
// transient_local always enters PRMSS_SYNC upon (re)matching and does not verify whether it has already received the entire data
// → Consequently, even if the partition is rematched, it is treated as a new match, leading to redundant data reception.
if (rd->handle_as_transient_local)
m->in_sync = PRMSS_OUT_OF_SYNC;
else if (vendor_is_eclipse (pwr->c.vendor))
m->in_sync = PRMSS_OUT_OF_SYNC;
else
m->in_sync = PRMSS_SYNC;
m->u.not_in_sync.end_of_tl_seq = MAX_SEQ_NUMBER;
```
### Rule 28 / Rule 29 / Rule 30
- **RMW/Implementation: FastDDS** 
```cpp
// If autodispose_unregistered_instances=false and the application does not explicitly call dispose(), the DataWriter will not send a disposal notification (DISPOSE), meaning the NOT_ALIVE_DISPOSED state is not passed to the DataReader.
// Consequently, the autopurge_disposed_samples_delay setting in READER_DATA_LIFECYCLE becomes irrelevant for that instance, rendering it meaningless.
/**
* @brief Indicates the duration the DataReader must retain information regarding instances that have the
* instance_state NOT_ALIVE_DISPOSED. <br>
* By default, dds::c_TimeInfinite.
*/
dds::Duration_t autopurge_disposed_samples_delay;
```
- **RMW/Implementation: CycloneDDS** 
```cpp
// As the instance is not in the DISPOSED state, autopurge_disposed_samples_delay does not apply to it in the first place.
void ddsi_reader_update_notify_pwr_alive_state (struct ddsi_reader *rd, const struct ddsi_proxy_writer *pwr, const struct ddsi_alive_state *alive_state)
{ struct ddsi_rd_pwr_match *m; bool notify = false; 
int delta = 0; 
/* -1: alive -> not_alive; 0: unchanged; 1: not_alive -> alive */ 
ddsrt_mutex_lock (&rd->e.lock); 
if ((m = ddsrt_avl_lookup (&ddsi_rd_writers_treedef, &rd->writers, &pwr->e.guid)) != NULL) 
{ if ((int32_t) (alive_state->vclock - m->pwr_alive_vclock) > 0) 
{ delta = (int) alive_state->alive - (int) m->pwr_alive; notify = true; 
m->pwr_alive = alive_state->alive; 
m->pwr_alive_vclock = alive_state->vclock; } } 
ddsrt_mutex_unlock (&rd->e.lock); 
if (delta < 0 && rd->rhc) { struct ddsi_writer_info wrinfo; 
ddsi_make_writer_info (&wrinfo, &pwr->e, pwr->c.xqos, NN_STATUSINFO_UNREGISTER); ddsi_rhc_unregister_wr (rd->rhc, &wrinfo); }
```
