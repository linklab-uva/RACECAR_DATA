// STD
#include <cstdio>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>



// ROSBAG
#include "rcpputils/filesystem_helper.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"

void readAndWrite(rosbag2_cpp::Reader& reader, rosbag2_cpp::Writer& writer,
                  bool rename, std::string topic_namespace,
                  std::vector<std::string> topics_filter) {
    auto topics = reader.get_all_topics_and_types();
    for (auto& topic : topics) {
        if (std::find(topics_filter.begin(), topics_filter.end(), topic.name) !=
            topics_filter.end()) {
            // only add topics in filter
            if (rename) {
                // add topic namespace
                std::string name = topic_namespace + topic.name;
                topic.name = name;
            }
            writer.create_topic(topic);
        }
    }

    while (reader.has_next()) {
        auto bag_message = reader.read_next();
        if (std::find(topics_filter.begin(), topics_filter.end(),
                      bag_message->topic_name) != topics_filter.end()) {
            // only keep topics in the topic filter
            if (rename) {
                // add topic namespace
                bag_message->topic_name =
                    topic_namespace + bag_message->topic_name;
            }
            writer.write(bag_message);
        }
    }
}

int main(int argc, char** argv) {
    std::vector<std::string> input_bag_files;
    std::string output_bag;
    std::vector<std::string> topics;
    std::vector<std::string> topic_namespaces;

    bool remap = false;

    for (int i = 0; i < argc; ++i) {
        std::string argn = std::string(argv[i]);
        if (argn == "-h" || argn == "--help") {
            std::cout << "rosbag_merger -i <input bagfile> [-i <input bagfile "
                         "2>] -o <output_bagfile> -n <bag1_namespace> [-t "
                         "<topic_a> <topic_b> <topic_c>...]"
                      << std::endl;
            std::cout << "Mergest input bags into a single output bag."
                      << std::endl;
            return 0;
        } else if (argn == "-i") {
            if (i + 1 >= argc) {
                std::cout << "bag name must be after -i flag" << std::endl;
                continue;
            }
            if (argv[i + 1][0] == '-') {
                std::cout
                    << "start of argument after -i flag is a `-`... do better."
                    << std::endl;
                continue;
            }
            input_bag_files.push_back(std::string(argv[i + 1]));
            ++i;
        } else if (argn == "-o") {
            if (i + 1 >= argc) {
                std::cerr << "bag name must be after -o flag" << std::endl;
                continue;
            }
            if (argv[i + 1][0] == '-') {
                std::cerr
                    << "start of argument after -o flag is a `-`... do better."
                    << std::endl;
                continue;
            }
            output_bag = std::string(argv[i + 1]);
            ++i;
        } else if (argn == "-t") {
            if (i + 1 >= argc) {
                std::cerr << "topic name must be after -t flag" << std::endl;
                continue;
            }
            if (argv[i + 1][0] == '-') {
                std::cerr
                    << "start of argument after -t flag is a `-`... do better."
                    << std::endl;
                continue;
            }
            for (int j = i + 1; j < argc; j++) {
                topics.push_back(std::string(argv[j]));
                ++i;
            }
        } else if (argn == "-n") {
            if (i + 1 >= argc) {
                std::cerr << "namespace must be after -n flag" << std::endl;
                continue;
            }
            if (argv[i + 1][0] == '-') {
                std::cerr
                    << "start of argument after -n flag is a `-`... do better."
                    << std::endl;
                continue;
            }

            topic_namespaces.push_back(std::string(argv[i + 1]));
            ++i;
        }
    }
    if (input_bag_files.size() == 0) {
        std::cerr << "No input bags :(" << std::endl;
        return 1;
    }
    if (output_bag == "") {
        std::cerr << "Output bag not set" << std::endl;
        return 1;
    }

    if (topics.size() > 0) {
        std::cout << "\nReading only " << topics.size() << " topics."
                  << std::endl;
        for (const auto& topic : topics) {
            std::cout << "  - " << topic << std::endl;
        }
    }

    if (topic_namespaces.size() > 0) {
        std::cout << "\nThere are " << topic_namespaces.size() << " namespaces."
                  << std::endl;
        for (const auto& topic : topic_namespaces) {
            std::cout << "  - " << topic << std::endl;
        }

        if (topic_namespaces.size() != input_bag_files.size()) {
            std::cerr
                << "Number of namespaces does not match number of input bags."
                << std::endl;
            return 1;
        }
        remap = true;
    }

    rosbag2_cpp::Writer writer;
    rosbag2_cpp::Reader reader;

    std::cout << "\nWriting to " << output_bag << std::endl;
    writer.open(output_bag);

    int i = 0;
    std::cout << "\nReading from " << input_bag_files.size()
              << " bags:" << std::endl;
    for (const auto& bag : input_bag_files) {
        std::cout << "  - " << bag << std::endl;
        try {
            reader.open(bag);
            readAndWrite(reader, writer, remap, topic_namespaces[i], topics);
            reader.close();
            i = i + 1;
        } catch (const std::exception& err) {
            std::cout << "Unable to open " << bag << ": " << err.what()
                      << std::endl;
            continue;
        }
    }

    return 0;
}
