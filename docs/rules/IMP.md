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
<hr class="hr-dashed">
<span id="rule-19"></span>
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
<hr class="hr-dashed">
<span id="rule-20"></span>
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
<hr class="hr-dashed">
<span id="rule-28"></span><span id="rule-29"></span><span id="rule-30"></span>
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
//If RDLIFE.autopurge_nowriter == 0: Triggers Rule #28 (Immediate purge of data).
//If RDLIFE.autopurge_disposed > 0: Triggers Rule #29 (never receives the disposal signal required to trigger the purge timer=Inapplicable).
//If RDLIFE.autopurge_nowriter == INF: Triggers Rule #30 (causes memory leak).
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
<hr class="hr-dashed">
<span id="rule-34"></span>
### Rule 34
- **RMW/Implementation: FastDDS** 
```cpp
// Set to best-effort mode, so NOT_ALIVE_DISPOSED_UNREGISTERED messages may be lost.
f !defined(NDEBUG) 
if (history_.is_key_registered(ih)) 
{ WriteParams wparams; ChangeKind_t change_kind = NOT_ALIVE_DISPOSED; 
if (!dispose) 
{ change_kind = qos_.writer_data_lifecycle().autodispose_unregistered_instances ? NOT_ALIVE_DISPOSED_UNREGISTERED : 
NOT_ALIVE_UNREGISTERED; }
```
- **RMW/Implementation: CycloneDDS** 
```cpp
// There is absolutely no retransmission/ACK-based guarantee for the transmitted sample (data + unregister/dispose control message).
static int rhc_unregister_updateinst ( ….
 if (!inst->isdisposed)
{
if (inst->latest == NULL || inst->latest->isread)
{
inst_set_invsample (rhc, inst, trig_qc, nda);
update_inst_no_wr_iid (inst, wrinfo, tstamp);
}
if (!inst->autodispose)
rhc->n_not_alive_no_writers++;
// When the Writer sends an unregister request, the Reader's RHC receives it and, if auto_dispose == 1, transitions the instance to the DISPOSED state.
```
<hr class="hr-dashed">
<span id="rule-40"></span>
### Rule 40
- **RMW/Implementation: FastDDS** 
```cpp
// 1. Writer previously issued multiple samples (e.g., one hour ago)
// 2. Reader joins late and receives past samples as TRANSIENT_LOCAL
// 3. Each retransmitted sample calls on_new_cache_change_added
// 4. Deadline timer is reset with each retransmission
bool DataReaderImpl::on_new_cache_change_added( 
const CacheChange_t* const change)
{ std::lock_guard<RecursiveTimedMutex> guard(reader_->getMutex()); 
if (qos_.deadline().period != c_TimeInfinite) 
{ if (!history_.set_next_deadline( change->instanceHandle, steady_clock::now() + duration_cast<system_clock::duration>(deadline_duration_us_))) 
{ logError(SUBSCRIBER, "Could not set next deadline in the history"); } 
else if (timer_owner_ == change->instanceHandle || timer_owner_ == InstanceHandle_t()) 
{ if (deadline_timer_reschedule()) 
{ deadline_timer_->cancel_timer(); deadline_timer_->restart_timer(); } } }
```
- **RMW/Implementation: CycloneDDS** 
```cpp
The deadline is set based on the sample reception time
// → With the transient setting, previous data arrives, causing the sample reception time to reset the deadline.
static void postprocess_instance_update (struct dds_rhc_default * __restrict rhc, struct rhc_instance * __restrict * __restrict instptr, const struct trigger_info_pre *pre, const struct trigger_info_post *post, struct trigger_info_qcond *trig_qc)
{ { struct rhc_instance *inst = *instptr;
#ifdef DDS_HAS_DEADLINE_MISSED 
if (inst->isdisposed) 
{ if (inst->deadline_reg) 
{ inst->deadline_reg = 0; 
deadline_unregister_instance_locked (&rhc->deadline, &inst->deadline); } } 
else 
{ if (inst->deadline_reg) 
deadline_renew_instance_locked (&rhc->deadline, &inst->deadline); 
else { deadline_register_instance_locked (&rhc->deadline, &inst->deadline, ddsrt_time_monotonic ()); 
inst->deadline_reg = 1; } }
#endif
```
<hr class="hr-double">
