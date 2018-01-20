
#include <assert.h>
#include <cstdio>
#include <cstdlib>

#include "../include/serial_port.h"

using communication::Serial;

bool ShowLog(Serial* serial)
{
    if (serial == nullptr)
    {
        assert(false);
        return false;
    }

    std::vector<std::string> logs;
    if (!serial->GetLogs(&logs))
        return false;

    printf("Log details:\n");
    for (std::vector<std::string>::iterator it = logs.begin(); it != logs.end(); ++it)
    {
        printf("- %s\n", it->c_str());
    }

    return true;
}

int main(int argc, char const* argv[])
{
    printf("\nSerial Port Initialization\n");

    Serial serial;
    unsigned int port_number = 4;
    unsigned int baud_rate = 9600;
    char parity = 'N';
    unsigned int data_bits = 8;
    unsigned int stop_bits = 1;
    unsigned long communication_events = EV_RXCHAR | EV_CTS;
    unsigned int buffer_size = 512;
    if (!serial.Init(port_number, baud_rate, parity, data_bits, stop_bits, communication_events, buffer_size))
    {
        printf("Error: Could not Initialize Port\n");
        ShowLog(&serial);
        std::system("pause");
        return 0;
    }

    if (!serial.Start())
    {
        printf("Error: Could not Start communication\n");
        ShowLog(&serial);
        std::system("pause");
        return 0;
    }

    ShowLog(&serial);

    printf("\nWriting Data\n");
    std::vector<char> buffer;
    std::string value("Value to write\n");
    std::copy(value.begin(), value.end(), std::back_inserter(buffer));
    if (!serial.Write(10 * 1000, buffer))
    {
        printf("Error: Could not Write data\n");
        ShowLog(&serial);
        std::system("pause");
        return 0;
    }

    ShowLog(&serial);

    printf("\nReading Data\n");
    buffer.clear();
    if (!serial.Read(60 * 1000, '\n', &buffer))
    {
        printf("Error: Could not Read data\n");
        ShowLog(&serial);
        std::system("pause");
        return 0;
    }

    printf("Data Read: %s\n", std::string(buffer.begin(), buffer.end()).c_str());
    ShowLog(&serial);

    std::system("pause");
    return 0;
}
