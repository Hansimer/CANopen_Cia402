#include "canopen.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <map>
#include <chrono>


CANInterface::CANInterface(const std::string &ifname, bool dry): ifname_(ifname), dry_(dry) {}

CANInterface::~CANInterface() { close(); }

bool CANInterface::open()
{
    if (dry_) return true;
    sockfd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd_ < 0) {
        perror("socket");
        sockfd_ = -1;
        return false;
    }
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, ifname_.c_str(), IFNAMSIZ-1);
    if (ioctl(sockfd_, SIOCGIFINDEX, &ifr) < 0) {
        perror("SIOCGIFINDEX");
        ::close(sockfd_);
        sockfd_ = -1;
        return false;
    }
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sockfd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        ::close(sockfd_);
        sockfd_ = -1;
        return false;
    }
    return true;
}

void CANInterface::close()
{
    if (sockfd_ >= 0) {
        ::close(sockfd_);
        sockfd_ = -1;
    }
}

bool CANInterface::send_frame(const struct can_frame &frame)
{
    if (dry_) {
        std::cout << "DRY SEND can_id=0x" << std::hex << frame.can_id << std::dec << " dlc=" << int(frame.can_dlc) << " data=";
        for (int i=0;i<frame.can_dlc;i++) printf("%02X", frame.data[i]);
        std::cout << std::endl;
        return true;
    }
    if (sockfd_ < 0) return false;
    ssize_t n = write(sockfd_, &frame, sizeof(frame));
    return n == sizeof(frame);
}

bool CANInterface::receive_frame(struct can_frame &frame, int timeout_ms)
{
    if (dry_) {
        usleep(timeout_ms * 1000);
        return false;
    }
    if (sockfd_ < 0) return false;
    fd_set rset;
    FD_ZERO(&rset);
    FD_SET(sockfd_, &rset);
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    int rv = select(sockfd_ + 1, &rset, NULL, NULL, &tv);
    if (rv <= 0) return false;
    ssize_t n = read(sockfd_, &frame, sizeof(frame));
    return n == sizeof(frame);
}

CANopenMaster::CANopenMaster(CANInterface &can): can_(can) {}

void CANopenMaster::nmt_start(uint8_t nodeid)
{
    struct can_frame frame;
    frame.can_id = 0x000;
    frame.can_dlc = 2;
    frame.data[0] = 0x01; // start remote node
    frame.data[1] = nodeid;
    can_.send_frame(frame);
}

void CANopenMaster::nmt_stop(uint8_t nodeid)
{
    struct can_frame frame;
    frame.can_id = 0x000;
    frame.can_dlc = 2;
    frame.data[0] = 0x02; // stop remote node
    frame.data[1] = nodeid;
    can_.send_frame(frame);
}

void CANopenMaster::send_heartbeat(uint8_t nodeid, uint8_t state)
{
    struct can_frame frame;
    frame.can_id = 0x700 + nodeid;
    frame.can_dlc = 1;
    frame.data[0] = state;
    can_.send_frame(frame);
}

bool CANopenMaster::sdo_write_exp(uint8_t nodeid, uint16_t index, uint8_t sub, uint32_t value, size_t size,
                                  int timeout_ms, int retries)
{
    if (size != 1 && size != 2 && size !=4) return false;
    uint8_t cs = 0x23; // default 4-bytes
    if (size == 1) cs = 0x2F;
    else if (size == 2) cs = 0x2B;
    else if (size == 4) cs = 0x23;

    struct can_frame frame;
    frame.can_id = 0x600 + nodeid; // SDO request
    frame.can_dlc = 8;
    frame.data[0] = cs;
    frame.data[1] = index & 0xFF;
    frame.data[2] = (index >> 8) & 0xFF;
    frame.data[3] = sub;
    // little endian payload
    for (size_t i=0;i<4;i++) frame.data[4+i] = (value >> (8*i)) & 0xFF;

    // send and wait for response (0x580 + nodeid) with retries
    for (int attempt=0; attempt<retries; ++attempt) {
        if (!can_.send_frame(frame)) continue;
        struct can_frame resp;
        int wait = timeout_ms;
        while (wait > 0) {
            int step = std::min(wait, 200);
            if (can_.receive_frame(resp, step)) {
                if ((resp.can_id & CAN_EFF_FLAG)==0 && resp.can_id == (0x580 + nodeid)) {
                    uint8_t csr = resp.data[0];
                    if (csr == 0x60) return true; // successful expedited write response
                    if (csr == 0x80) return false; // abort
                }
            }
            wait -= step;
        }
    }
    return false;
}

bool CANopenMaster::sdo_read_exp(uint8_t nodeid, uint16_t index, uint8_t sub, std::vector<uint8_t> &out,
                                 int timeout_ms, int retries)
{
    struct can_frame frame;
    frame.can_id = 0x600 + nodeid;
    frame.can_dlc = 8;
    frame.data[0] = 0x40; // initiate upload
    frame.data[1] = index & 0xFF;
    frame.data[2] = (index >> 8) & 0xFF;
    frame.data[3] = sub;
    for (int i=4;i<8;i++) frame.data[i]=0x00;

    for (int attempt=0; attempt<retries; ++attempt) {
        if (!can_.send_frame(frame)) continue;
        struct can_frame resp;
        int wait = timeout_ms;
        while (wait > 0) {
            int step = std::min(wait, 200);
            if (can_.receive_frame(resp, step)) {
                if ((resp.can_id & CAN_EFF_FLAG)==0 && resp.can_id == (0x580 + nodeid)) {
                    uint8_t cs = resp.data[0];
                    out.clear();
                    if (cs == 0x43) { // 4 byte expedited
                        for (int i=4;i<8;i++) out.push_back(resp.data[i]);
                        return true;
                    } else if (cs == 0x4B) { // 2 byte expedited
                        out.push_back(resp.data[4]);
                        out.push_back(resp.data[5]);
                        return true;
                    } else if (cs == 0x4F) { // 1 byte expedited
                        out.push_back(resp.data[4]);
                        return true;
                    } else if (cs == 0x80) { // abort
                        return false;
                    } else {
                        // unsupported (e.g., segmented) - bail out for now
                        return false;
                    }
                }
            }
            wait -= step;
        }
    }
    return false;
}

bool CANopenMaster::poll_and_handle(int timeout_ms)
{
    struct can_frame frame;
    if (!can_.receive_frame(frame, timeout_ms)) return false;

    uint32_t id = frame.can_id & CAN_SFF_MASK;
    using clk = std::chrono::steady_clock;
    auto now = clk::now();

    // Heartbeat: COB-ID 0x700 + node
    if (id >= 0x700 && id <= 0x77F) {
        uint8_t node = id - 0x700;
        uint8_t hb = frame.data[0];
        auto &st = statuses_[node];
        st.nodeid = node;
        st.last_heartbeat = now;
        st.connected = true;
        st.alarm = false;
        if (hb == 0x05) st.state = OPERATIONAL;
        else if (hb == 0x04) st.state = PRE_OPERATIONAL;
        else if (hb == 0x7F) st.state = STOPPED;
        else st.state = UNKNOWN;
        return true;
    }

    // SDO response (0x580 + node)
    if (id >= 0x580 && id <= 0x5FF) {
        uint8_t node = id - 0x580;
        uint8_t cs = frame.data[0];
        // 0x80 = abort
        if (cs == 0x80) {
            auto &st = statuses_[node];
            st.alarm = true;
            if (alarm_cb_) alarm_cb_(node, "SDO Abort received");
            return true;
        }
        // expedited read responses handled elsewhere; mark last heartbeat time as activity
        statuses_[node].last_heartbeat = now;
        statuses_[node].connected = true;
        return true;
    }

    // TPDO range typical 0x180-0x1FF (node = id - 0x180)
    if (id >= 0x180 && id <= 0x1FF) {
        uint8_t node = id - 0x180;
        // If TPDO contains statusword at first two bytes, capture it
        if (frame.can_dlc >= 2) {
            uint16_t status = frame.data[0] | (frame.data[1] << 8);
            auto &st = statuses_[node];
            st.nodeid = node;
            st.statusword = status;
            // derive simple state from statusword bits (Operation Enabled bit7)
            if (status & 0x0040) st.state = OPERATIONAL; // example: Operation enabled bit
            // record activity
            st.last_heartbeat = now;
            st.connected = true;
        }
        return true;
    }

    // Other frames ignored for now
    return true;
}

int CANopenMaster::check_heartbeats(int threshold_ms)
{
    using clk = std::chrono::steady_clock;
    auto now = clk::now();
    int alarms = 0;
    for (auto &kv : statuses_) {
        auto node = kv.first;
        auto &st = kv.second;
        if (!st.connected) continue;
        if (st.last_heartbeat.time_since_epoch().count() == 0) continue;
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_heartbeat).count();
        if (elapsed > threshold_ms) {
            if (st.connected) {
                st.connected = false;
                st.alarm = true;
                alarms++;
                if (alarm_cb_) alarm_cb_(node, "Heartbeat lost / timeout");
            }
        }
    }
    return alarms;
}

const CANopenMaster::MachineStatus* CANopenMaster::get_status(uint8_t nodeid) const
{
    auto it = statuses_.find(nodeid);
    if (it == statuses_.end()) return nullptr;
    return &it->second;
}

bool CANopenMaster::send_rpdo(uint32_t cob_id, const uint8_t payload[8])
{
    struct can_frame frame;
    frame.can_id = cob_id;
    frame.can_dlc = 8;
    for (int i=0;i<8;i++) frame.data[i]=payload[i];
    return can_.send_frame(frame);
}
