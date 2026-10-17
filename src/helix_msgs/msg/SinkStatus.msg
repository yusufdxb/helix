# SinkStatus: what the GO2 sport sink sent and what the robot reported back.
#
# Every stamp is the sink host's monotonic clock in seconds. The supervisor
# runs on the same host, so it compares these stamps with its own steady
# clock directly.
#
# stop_acked is true only when the most recent request the sink sent was a
# StopMove (1003) and the robot answered it on /api/sport/response with
# status code 0. dry_run sends nothing, so it can never ack.
# stopped is measured: |v_xy| and |yaw_speed| from SportModeState stayed
# below the sink's thresholds for still_s.

string mode                  # dry_run | stop_only | armed
bool feedback_available      # response + state topics are being received
int32 last_api_id            # api id of the most recent request sent (0 if none)
int64 last_request_id
string last_reason
float64 last_sent_at
int64 last_ack_request_id
int32 last_ack_code          # status.code of that response
float64 last_ack_at
bool stop_acked
float64 stop_ack_sent_at     # when the acknowledged StopMove request was sent
float64 stop_acked_at        # receipt time of the ack that made stop_acked true
float64 speed_xy             # m/s, measured
float64 yaw_speed            # rad/s, measured
float64 state_at             # receipt time of the newest SportModeState
uint8 robot_mode             # SportModeState.mode
bool stopped
float64 stopped_since
uint64 seq
