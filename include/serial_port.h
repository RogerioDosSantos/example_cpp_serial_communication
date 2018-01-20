
#ifndef _CPP_SERIAL_COMMUNICATION_INCLUDE_SERIAL_PORT_H_
#define _CPP_SERIAL_COMMUNICATION_INCLUDE_SERIAL_PORT_H_

#include <Windows.h>
#include <atomic>
#include <deque>
#include <memory>
#include <thread>
#include <vector>

namespace communication
{
    class CriticalSessionGuard
    {
    public:
        CriticalSessionGuard(CRITICAL_SECTION* critical_session)
            : _critical_session(critical_session)
        {
            EnterCriticalSection(_critical_session);
        }
        ~CriticalSessionGuard()
        {
            LeaveCriticalSection(_critical_session);
        }

    private:
        CRITICAL_SECTION* _critical_session;
    };

    class Serial
    {
    public:
        Serial();
        ~Serial();

        bool Init(unsigned int port_number = 1, unsigned int baud_rate = 19200, char parity = 'N', unsigned int data_bits = 8,
            unsigned int stop_bits = 1, unsigned long communication_events = EV_RXCHAR | EV_CTS, unsigned int buffer_size = 512);

        bool Start();
        bool Restart();
        bool Stop();

        bool Read(int time_out, const char end_of_message, std::vector<char>* buffer);
        bool Write(int time_out, const std::vector<char>& buffer);

        bool GetLogs(std::vector<std::string>* logs);

    protected:
        void Log(const char* message, bool show_error);
        static void Thread(Serial* port);
        static void Receive(Serial* port);
        static void Send(Serial* port);

    protected:
        std::vector<HANDLE> _events_handle;
        OVERLAPPED _async_io_data;
        COMMTIMEOUTS _communication_timeout_data;
        DCB _control_settings;
        unsigned int _port_number;
        unsigned long _communication_events;
        unsigned long _write_buffer_size;

    private:
        std::vector<std::string> _logs;
        std::vector<char> _read_buffer;
        std::vector<char> _write_buffer;
        std::vector<char> _consumer_buffer;
        std::shared_ptr<std::thread> _thread;
        CRITICAL_SECTION _sync_communication;

    protected:
        std::atomic<bool> _thread_alive;
        HANDLE _port_handler;
        HANDLE _shutdown_event_handler;
        HANDLE _write_event_handler;
        HANDLE _write_completed_event_handler;
    };
}

#endif  // _CPP_SERIAL_COMMUNICATION_INCLUDE_SERIAL_PORT_H_
