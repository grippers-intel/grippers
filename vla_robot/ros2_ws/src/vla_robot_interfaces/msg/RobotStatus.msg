# pi_mission_node 가 /vla_robot/status 로 발행한다. 디버깅·모니터링용이며
# Host 로 가는 UDP 상태 패킷(vla_common.protocol.PiStatus)과 같은 내용이다.
builtin_interfaces/Time stamp
string state
bool busy
uint32 job_id
string last_action
bool last_ok
string last_detail
bool base_ok
bool watchdog
bool estop_latched
string host_address
