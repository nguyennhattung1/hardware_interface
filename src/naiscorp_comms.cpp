#include "naiscorp_hardware_interface/naiscorp_comms.hpp"
#include "naiscorp_hardware_interface/constants.h"
#include <iostream>
#include <iomanip>
#include <sstream>

namespace naiscorp
{

    LibSerial::BaudRate convert_baud_rate(int baud_rate)
    {
        // Just handle some common baud rates
        switch (baud_rate)
        {
        case 1200:
            return LibSerial::BaudRate::BAUD_1200;
        case 1800:
            return LibSerial::BaudRate::BAUD_1800;
        case 2400:
            return LibSerial::BaudRate::BAUD_2400;
        case 4800:
            return LibSerial::BaudRate::BAUD_4800;
        case 9600:
            return LibSerial::BaudRate::BAUD_9600;
        case 19200:
            return LibSerial::BaudRate::BAUD_19200;
        case 38400:
            return LibSerial::BaudRate::BAUD_38400;
        case 57600:
            return LibSerial::BaudRate::BAUD_57600;
        case 115200:
            return LibSerial::BaudRate::BAUD_115200;
        case 230400:
            return LibSerial::BaudRate::BAUD_230400;
        default:
            std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to BAUD_115200" << std::endl;
            return LibSerial::BaudRate::BAUD_115200;
        }
    }

    MCUComms::MCUComms() = default;

    void MCUComms::connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
    {
        timeout_ms_ = timeout_ms;
        std::cerr << "[MCUComms::connect] Attempting to open device: " << serial_device << std::endl;
        try
        {
            serial_conn_.Open(serial_device);
            std::cerr << "[MCUComms::connect] serial_conn_.Open() called." << std::endl;
            serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
            std::cerr << "[MCUComms::connect] Baud rate set." << std::endl;
        }
        catch (const LibSerial::OpenFailed &e)
        {
            std::cerr << "[MCUComms::connect] LibSerial::OpenFailed: " << e.what() << std::endl;
            std::cerr << "[MCUComms::connect] Make sure the device '" << serial_device << "' exists and you have permissions." << std::endl;
            throw;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[MCUComms::connect] std::exception: " << e.what() << std::endl;
            throw;
        }
        std::cerr << "[MCUComms::connect] Connection attempt finished." << std::endl;
    }

    void MCUComms::disconnect()
    {
        if (serial_conn_.IsOpen())
        {
            serial_conn_.Close();
        }
    }

    bool MCUComms::connected() const
    {
        return serial_conn_.IsOpen();
    }

    std::string MCUComms::send_msg(const std::vector<char> &msg_to_send, bool print_output)
    {
        // Helper function to convert binary data to hex representation
        auto to_hex = [](const std::vector<char> &input)
        {
            std::stringstream ss;
            for (unsigned char c : input)
            {
                ss << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                   << static_cast<unsigned int>(static_cast<unsigned char>(c)) << " ";
            }
            return ss.str();
        };

        // Print raw data as hex
        std::cout << "Raw data: " << to_hex(msg_to_send) << std::endl;

        serial_conn_.FlushIOBuffers(); // Clear buffers

        // Send raw binary data
        std::string data_str(msg_to_send.begin(), msg_to_send.end());
        serial_conn_.Write(data_str);

        std::string response = "";
        if (print_output)
        {
            try
            {
                // Responses end with \r\n so we will read up to (and including) the \n.
                serial_conn_.ReadLine(response, '\n', timeout_ms_);
            }
            catch (const LibSerial::ReadTimeout &)
            {
                std::cerr << "The ReadByte() call has timed out." << std::endl;
            }
            std::cout << "Sent: " << to_hex(msg_to_send) << " Recv: " << response << std::endl;
        }

        return response;
    }

    std::vector<MCUComms::MotorState> MCUComms::read_state_values()
    {
        // Store all motor states
        std::vector<MotorState> motor_states;

        // Send data to ask for values
        std::vector<char> packet;
        char checksum = 0;

        // Add all bytes to the packet
        packet.push_back(START_BYTE);
        packet.push_back(PACKET_TYPE_STATE_COMMAND);
        packet.push_back(PAYLOAD_LENGTH_STATE_COMMAND);
        packet.push_back(checksum);

        // Send request
        send_msg(packet, false);

        // Read responses for all motors (expecting 7 motors)
        try
        {
            for (int i = 0; i < 2; i++) // MODIFIED: Was 7, now 2, assuming only LEFT and RIGHT wheels provide state
            {
                // Buffer to hold the complete packet for one motor
                std::vector<unsigned char> buffer;
                std::string byte = "";

                // Read until we find the start byte
                std::cout << "Reading motor state..." << std::endl;
                try
                {
                    serial_conn_.ReadLine(byte, 0x02, timeout_ms_);
                    std::cout << "Read byte: " << std::endl;
                    for (size_t j = 0; j < byte.size(); j++)
                    {
                        printf("%02X ", byte[j]);
                    }
                    printf("\n");
                }
                catch (const LibSerial::ReadTimeout &e)
                {
                    std::cerr << "Timeout while reading motor " << (i + 1) << ": " << e.what() << std::endl; // MODIFIED: Log (i+1) for 1-based motor ID
                    // Continue to the next motor or break out if no data is expected
                    continue;
                }

                // Rest of the code for parsing motor state packet
                // This is commented out in the original code
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Exception in read_state_values: " << e.what() << std::endl;
        }

        return motor_states;
    }

    void MCUComms::set_motor_values(char motor_id, char command_type, char value)
    {
        std::vector<char> packet;
        
        // Convert char value to numeric type for proper comparison and rescaling
        int numeric_value = static_cast<int>(value) & 0xFF;
        
        // Rescale the value to the range [0, 30]
        numeric_value = numeric_value * 30 / 100;
        
        // Convert back to char after rescaling
        value = static_cast<char>(numeric_value);
        
        // Log the rescaled value
        std::cout << "Setting motor " << static_cast<int>(motor_id) 
                  << " with command type " << static_cast<int>(command_type)
                  << " to value " << numeric_value << " (rescaled to range 0-30)" << std::endl;

        // Add all bytes to the packet (without checksum initially)
        packet.push_back(START_BYTE);
        packet.push_back(PACKET_TYPE_MOTOR_COMMAND);
        packet.push_back(PAYLOAD_LENGTH_MOTOR_COMMAND);
        packet.push_back(motor_id);
        packet.push_back(command_type);
        packet.push_back(value);
        
        // Direct calculation of checksum (for verification)
        char direct_checksum = motor_id ^ command_type ^ value;
        
        // Calculate checksum by XORing all bytes after payload length
        char checksum = 0;
        for (size_t i = 3; i < packet.size(); i++) {
            checksum ^= packet[i];
        }
        
        // Verify checksums match
        if (direct_checksum != checksum) {
            std::cerr << "Warning: Checksum mismatch. Direct: 0x" 
                      << std::hex << static_cast<int>(direct_checksum & 0xFF)
                      << ", Calculated: 0x" << static_cast<int>(checksum & 0xFF) << std::endl;
        }
        
        // Add the calculated checksum
        packet.push_back(checksum);
        
        // Pass the vector directly to send_msg
        send_msg(packet, true);
    }

    void MCUComms::set_pid_values(char motor_id, char kp, char ki, char kd)
    {
        std::vector<char> packet;
        
        // Add all bytes to the packet (without checksum initially)
        packet.push_back(START_BYTE);
        packet.push_back(PACKET_TYPE_PID_COMMAND);
        packet.push_back(PAYLOAD_LENGTH_PID_COMMAND);
        packet.push_back(motor_id);
        packet.push_back(kp);
        packet.push_back(ki);
        packet.push_back(kd);
        
        // Direct calculation of checksum (for verification)
        char direct_checksum = motor_id ^ kp ^ ki ^ kd;
        
        // Calculate checksum by XORing all bytes after payload length
        char checksum = 0;
        for (size_t i = 3; i < packet.size(); i++) {
            checksum ^= packet[i];
        }
        
        // Verify checksums match
        if (direct_checksum != checksum) {
            std::cerr << "Warning: Checksum mismatch. Direct: 0x" 
                      << std::hex << static_cast<int>(direct_checksum & 0xFF)
                      << ", Calculated: 0x" << static_cast<int>(checksum & 0xFF) << std::endl;
        }
        
        // Add the calculated checksum
        packet.push_back(checksum);
        
        // Pass the vector directly to send_msg
        send_msg(packet, true);
    }

} // namespace naiscorp