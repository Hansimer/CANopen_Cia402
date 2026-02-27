// Simple XML -> JSON converter and example data retrieval
#include <iostream>
#include <fstream>
#include <string>
#include "pugixml.hpp"
#include "json.hpp"

using json = nlohmann::json;

// Convert a pugi::xml_node to nlohmann::json recursively
json node_to_json(const pugi::xml_node &node)
{
    // If node has no children and has text, return text
    bool has_element_child = false;
    for (auto &ch : node.children()) {
        if (ch.type() == pugi::node_element) { has_element_child = true; break; }
    }

    json obj;

    // Attributes
    for (auto &attr : node.attributes()) {
        obj["@" + std::string(attr.name())] = std::string(attr.value());
    }

    // If no element children, and text exists, return text (or attributes+text)
    if (!has_element_child) {
        std::string text = node.text().get();
        if (!text.empty()) {
            if (obj.empty()) return text;
            obj["#text"] = text;
            return obj;
        }
        // only attributes
        if (!obj.empty()) return obj;
        return json();
    }

    // Process element children: group repeated names into arrays
    std::map<std::string, std::vector<json>> groups;
    for (auto &ch : node.children()) {
        if (ch.type() != pugi::node_element) continue;
        groups[ch.name()].push_back(node_to_json(ch));
    }

    for (auto &kv : groups) {
        if (kv.second.size() == 1) obj[kv.first] = kv.second[0];
        else obj[kv.first] = kv.second;
    }

    // merge attributes already collected
    for (auto &attr : node.attributes()) {
        obj["@" + std::string(attr.name())] = std::string(attr.value());
    }

    return obj;
}

// Find servo by node_id in converted JSON
json find_servo_by_nodeid(const json &root, const std::string &nodeid)
{
    if (!root.is_object()) return json();
    // support both single Servo or array
    if (root.contains("Servos")) {
        auto servos = root["Servos"]; // could be object with Servo
        if (servos.contains("Servo")) {
            auto s = servos["Servo"];
            if (s.is_array()) {
                for (auto &item : s) {
                    if (item.contains("@node_id") && item["@node_id"] == nodeid) return item;
                }
            } else if (s.is_object()) {
                if (s.contains("@node_id") && s["@node_id"] == nodeid) return s;
            }
        }
    }
    return json();
}

int main(int argc, char **argv)
{
    std::string filename = "product.xml";
    if (argc > 1) filename = argv[1];

    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(filename.c_str());
    if (!result) {
        std::cerr << "Failed to load XML file: " << filename << " (" << result.description() << ")\n";
        return 1;
    }

    json out;
    // Convert root children
    for (auto &child : doc.children()) {
        if (child.type() != pugi::node_element) continue;
        out[child.name()] = node_to_json(child);
    }

    // Print pretty JSON
    std::cout << out.dump(2) << std::endl;

    // Demo: find servo node_id 2
    json servo = find_servo_by_nodeid(out, "2");
    if (!servo.is_null()) {
        std::cout << "\nFound Servo node_id=2, MotionDefaults: \n";
        if (servo.contains("MotionDefaults")) std::cout << servo["MotionDefaults"].dump(2) << std::endl;
        if (servo.contains("Limits")) std::cout << "Limits: " << servo["Limits"].dump() << std::endl;
    } else {
        std::cout << "\nServo node_id=2 not found." << std::endl;
    }

    return 0;
}
