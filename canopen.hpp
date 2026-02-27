#ifndef CANOPEN_HPP
#define CANOPEN_HPP

#include <string>
#include <vector>
#include <cstdint>
#include <linux/can.h>
#include <map>
#include <chrono>
#include <functional>

class CANInterface {
public:
    CANInterface(const std::string &ifname, bool dry=false);
    ~CANInterface();
    bool open();
    void close();
    bool send_frame(const struct can_frame &frame);
    bool receive_frame(struct can_frame &frame, int timeout_ms);
    bool is_open() const { return sockfd_ >= 0; }
    bool dry_run() const { return dry_; }
private:
    std::string ifname_;
    int sockfd_ = -1;
    bool dry_ = false;
};

class CANopenMaster {
public:
    CANopenMaster(CANInterface &can);
    // NMT
    void nmt_start(uint8_t nodeid);
    void nmt_stop(uint8_t nodeid);
    // Heartbeat: send one byte state via COB-ID 0x700 + node
    void send_heartbeat(uint8_t nodeid, uint8_t state);
    // SDO expedited write (1/2/4 bytes)
    // SDO expedited write (1/2/4 bytes). Returns true on success.
    bool sdo_write_exp(uint8_t nodeid, uint16_t index, uint8_t sub, uint32_t value, size_t size,
                       int timeout_ms = 500, int retries = 3);
    // SDO read (expedited upload). Fills 'out' with the returned bytes on success.
    // Returns true if a valid expedited response was received. Supports simple expedited responses only.
    bool sdo_read_exp(uint8_t nodeid, uint16_t index, uint8_t sub, std::vector<uint8_t> &out,
                      int timeout_ms=500, int retries=3);
    // PDO send (raw 8-byte payload)
    bool send_rpdo(uint32_t cob_id, const uint8_t payload[8]);
    // Poll for incoming CAN frames and handle SDO/TPDO/heartbeat parsing.
    // Returns true if a frame was received and handled.
    bool poll_and_handle(int timeout_ms);

    enum MachineState { UNKNOWN=0, PRE_OPERATIONAL, OPERATIONAL, STOPPED, FAULT };

    struct MachineStatus {
        uint8_t nodeid = 0;
        MachineState state = UNKNOWN;
        uint16_t statusword = 0;
        std::chrono::steady_clock::time_point last_heartbeat{};
        bool connected = false;
        bool alarm = false;
    };

    // Check heartbeats for all known nodes; if a node hasn't sent heartbeat within
    // `threshold_ms`, trigger alarm callback. Returns number of alarms triggered.
    int check_heartbeats(int threshold_ms);

    // Get status for a node; returns nullptr if unknown
    const MachineStatus* get_status(uint8_t nodeid) const;

    using AlarmCallback = std::function<void(uint8_t,const std::string&)>;
    void register_alarm_callback(AlarmCallback cb) { alarm_cb_ = cb; }

private:
    CANInterface &can_;
    std::map<uint8_t, MachineStatus> statuses_;
    AlarmCallback alarm_cb_ = nullptr;
};

#endif // CANOPEN_HPP
