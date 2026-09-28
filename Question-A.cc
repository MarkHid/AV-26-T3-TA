// Part A: This is an extension task that requires you to decode sensor data from CAN log files.
// CAN (Controller Area Network) is a communication standard used in automotive applications (including Redback cars)
// to allow communication between sensors and controllers.
//
// Your Task: Using the signal definitions in SteeringBench.dbc, read each CAN capture in data/
// and turn it into a CSV with one row per decoded frame:
// t,u_commanded,y_measured
// eg:
// 0,15.0,0.0
// 0.005,15.0,0.0
// ...
// where t is the frame timestamp minus the first kept frame's timestamp (s), u_commanded is
// the decoded CmdAngularRate (deg/s), and y_measured is the decoded MeasuredAngle (deg).
// The above values are not real numbers; they are only there to show the expected data output format.
// Do this for all three captures:
// data/step_test.log       ->  data/step_test.csv
// data/reversal_test.log   ->  data/reversal_test.csv
// data/deadband_test.log   ->  data/deadband_test.csv
//
// The Row type, writeCsv(), and main() below are provided -- they loop the three logs, call your
// decodeLog(), and write the CSV in exactly the format above. You just need to implement decodeLog().
//
// You do not need to use any external libraries. Use the resources below to understand how to
// extract sensor data.
// Hint: Think about manual bit masking and shifting, data types required,
// what formats are used to represent values, etc.
// Resources:
// https://www.csselectronics.com/pages/can-bus-simple-intro-tutorial
// https://www.csselectronics.com/pages/can-dbc-file-database-intro
//
// Sanity check: plot your CSVs (python3 plot_data.py) and compare against the pre-plotted
// data/*.png files -- they should match.
//
// Build & run (from the TA/ folder):
//     c++ -std=c++17 Question-A.cc -o decode
//     ./decode

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

// One output row.
struct Row {
    double t;            // seconds since the first kept frame
    double u_commanded;  // deg/s
    double y_measured;   // deg
};

// Convert two hexadecimal characters from the CAN data into one byte.
int readHexByte(const std::string& data, std::size_t position) {
    return std::stoi(data.substr(position, 2), nullptr, 16);
}

// Read the candump log at `path` and return one Row per STEER_ActuatorLog frame, in order.
// Push one Row{t, u_commanded, y_measured} per kept frame.
std::vector<Row> decodeLog(const std::string& path) {
    std::vector<Row> rows;

    std::ifstream file(path);
    std::string line;
    double firstTimestamp = 0.0;

    while (std::getline(file, line)) {
        // CAN ID 0x200 is STEER_ActuatorLog in SteeringBench.dbc.
        const std::size_t framePosition = line.find(" 200#");
        if (framePosition == std::string::npos)
            continue;

        const std::size_t dataPosition = framePosition + 5;
        if (line.size() < dataPosition + 8)
            continue;

        const std::string data = line.substr(dataPosition);
        const int byte0 = readHexByte(data, 0);
        const int byte1 = readHexByte(data, 2);
        const int byte2 = readHexByte(data, 4);
        const int byte3 = readHexByte(data, 6);

        // The DBC stores both signals as signed, little-endian 16-bit values.
        int measured = byte0 | (byte1 << 8);
        int commanded = byte2 | (byte3 << 8);

        if (measured & 0x8000)
            measured -= 0x10000;
        if (commanded & 0x8000)
            commanded -= 0x10000;

        const std::size_t timestampEnd = line.find(')');
        const double timestamp = std::stod(line.substr(1, timestampEnd - 1));

        if (rows.empty())
            firstTimestamp = timestamp;
        rows.push_back({timestamp - firstTimestamp, commanded * 0.1, measured * 0.1});
    }

    return rows;
}

// Provided -- writes the rows to a CSV in the required format. Do not change.
void writeCsv(const std::string& path, const std::vector<Row>& rows) {
    std::ofstream f(path);
    f << "t,u_commanded,y_measured\n";
    for (const Row& r : rows)
        f << r.t << "," << r.u_commanded << "," << r.y_measured << "\n";
}

// Provided -- runs decodeLog() + writeCsv() for each of the three captures.
int main() {
    const char* names[] = {"step_test", "reversal_test", "deadband_test"};
    for (const char* n : names) {
        const std::string in  = std::string("data/") + n + ".log";
        const std::string out = std::string("data/") + n + ".csv";
        const std::vector<Row> rows = decodeLog(in);
        writeCsv(out, rows);
        std::printf("%-14s %6zu frames -> %s\n", n, rows.size(), out.c_str());
    }
    return 0;
}
