#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>
#include <atomic>
#include <cmath>
#include "pugixml.hpp"
#include "canopen.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    std::string ifname = "can0";
    std::string xmlfile = "product.xml";
    bool dry = false;
    if (argc > 1) ifname = argv[1];
    if (argc > 2) xmlfile = argv[2];
    if (argc > 3 && std::strcmp(argv[3], "--dry") == 0) dry = true;

    // load XML to get node id and PDO IDs
    pugi::xml_document doc;
    if (!doc.load_file(xmlfile.c_str())) {
        std::cerr << "Failed to open " << xmlfile << std::endl;
        return 1;
    }

    auto servo = doc.child("Servos").child("Servo");
    if (!servo) {
        std::cerr << "No Servo node in " << xmlfile << std::endl;
        return 1;
    }
    uint8_t nodeid = (uint8_t)servo.attribute("node_id").as_uint();

    // read RPDO/TPDO ids (simple reading first mapping)
    uint32_t rpdo_id = 0x200 + nodeid; // default
    uint32_t tpdo_id = 0x180 + nodeid; // default
    auto rpdo = servo.child("PDOs").child("RPDO");
    if (rpdo) rpdo_id = std::stoul(rpdo.attribute("id").as_string(), nullptr, 0);
    auto tpdo = servo.child("PDOs").child("TPDO");
    if (tpdo) tpdo_id = std::stoul(tpdo.attribute("id").as_string(), nullptr, 0);

    CANInterface can(ifname, dry);
    if (!can.open()) {
        std::cerr << "Failed to open CAN interface " << ifname << ", switching to dry-run." << std::endl;
        can = CANInterface(ifname, true);
    }

    CANopenMaster master(can);

    // register alarm callback
    master.register_alarm_callback([&](uint8_t node, const std::string &msg){
        std::cerr << "ALARM node " << int(node) << ": " << msg << std::endl;
    });

    // start receiver thread to poll incoming frames and update status
    std::atomic<bool> recv_running{true};
    std::thread recv_thread([&]{
        while (recv_running) {
            master.poll_and_handle(100);
            // small sleep to avoid busy loop
            std::this_thread::sleep_for(10ms);
        }
    });

    // Start node
    master.nmt_start(nodeid);

    // Set mode of operation to velocity (example mode 3) via SDO (0x6060)
    std::cout << "Setting mode of operation via SDO (0x6060) -> 3" << std::endl;
    master.sdo_write_exp(nodeid, 0x6060, 0x00, 3, 1);

    // Build RPDO payload template: controlword(2) + target velocity (4) + padding
    auto send_velocity = [&](int32_t vel){
        uint8_t payload[8];
        // controlword: enable operation + new setpoint (example minimal control)
        uint16_t controlword = 0x000F; // may need state machine steps in practice
        payload[0] = controlword & 0xFF;
        payload[1] = (controlword >> 8) & 0xFF;
        // target velocity as int32 little endian
        int32_t tv = vel;
        payload[2] = tv & 0xFF;
        payload[3] = (tv>>8) & 0xFF;
        payload[4] = (tv>>16) & 0xFF;
        payload[5] = (tv>>24) & 0xFF;
        payload[6] = 0;
        payload[7] = 0;
        master.send_rpdo(rpdo_id, payload);
    };

    // heartbeat sender thread (for master monitoring purpose)
    std::atomic<bool> running{true};
    std::thread hb_thread([&]{
        while (running) {
            master.send_heartbeat(nodeid, 0x05); // Operational
            std::this_thread::sleep_for(1000ms);
        }
    });

    // cyclic control loop 8ms
    std::cout << "Entering control loop (8ms cycle). Press Ctrl+C to exit." << std::endl;
    for (int i=0;i<1000;i++) {
        // simple sinusoidal velocity test
        int32_t vel = 1000 * (int)std::sin(i * 0.1);
        send_velocity(vel);
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
        // periodically check heartbeats (threshold 3000ms)
        if (i % 125 == 0) {
            master.check_heartbeats(3000);
        }
    }

    running = false;
    hb_thread.join();
    recv_running = false;
    recv_thread.join();

    return 0;
}
