#include "../include/serial_port.h"
#include <assert.h>
#include <ctime>
#include <sstream>

using communication::Serial;
using communication::CriticalSessionGuard;

Serial::Serial()
    : _port_handler(nullptr)
    , _write_event_handler(nullptr)
    , _write_completed_event_handler(nullptr)
    , _shutdown_event_handler(nullptr)
    , _thread_alive(false)
{
    _async_io_data.Offset = 0;
    _async_io_data.OffsetHigh = 0;
    _async_io_data.hEvent = nullptr;
}

Serial::~Serial()
{
    Stop();
}

bool Serial::Init(unsigned int port_number /*= 1*/, unsigned int baud_rate /*= 19200*/, char parity /*= 'N'*/, unsigned int data_bits /*= 8*/,
    unsigned int stop_bits /*= 1*/, unsigned long communication_events /*= EV_RXCHAR | EV_CTS*/, unsigned int buffer_size /*= 512*/)
{
    if (!Stop())
        return false;

    if (_async_io_data.hEvent != nullptr)
        ResetEvent(_async_io_data.hEvent);

    _async_io_data.hEvent = CreateEvent(nullptr, true, false, nullptr);
    if (_write_event_handler != nullptr)
        ResetEvent(_write_event_handler);

    _write_event_handler = CreateEvent(nullptr, true, false, nullptr);
    if (_write_completed_event_handler != nullptr)
        ResetEvent(_write_completed_event_handler);

    _write_completed_event_handler = CreateEvent(nullptr, true, false, nullptr);
    if (_shutdown_event_handler != nullptr)
        ResetEvent(_shutdown_event_handler);

    _shutdown_event_handler = CreateEvent(nullptr, true, false, nullptr);
    InitializeCriticalSection(&_sync_communication);
    if (!_write_buffer.empty())
        _write_buffer.clear();

    _write_buffer.reserve(buffer_size + 1);
    _port_number = port_number;
    _write_buffer_size = buffer_size;
    _communication_events = communication_events;

    CriticalSessionGuard session_guard(&_sync_communication);
    if (_port_handler != nullptr)
    {
        CloseHandle(_port_handler);
        _port_handler = nullptr;
    }

    std::ostringstream port_config;
    port_config << "\\\\.\\COM" << port_number;
    _port_handler = CreateFileA(port_config.str().c_str(),  // communication port string (COMX)
        GENERIC_READ | GENERIC_WRITE,                      // read/write types
        0,                                                 // comm devices must be opened with exclusive access
        NULL,                                              // no security attributes
        OPEN_EXISTING,                                     // comm devices must use OPEN_EXISTING
        FILE_FLAG_OVERLAPPED,                              // Async I/O
        0);                                                // template must be 0 for comm devices

    if (_port_handler == INVALID_HANDLE_VALUE)
    {
        Log("Init - Invalid Port.", true);
        return false;
    }

    _communication_timeout_data.ReadIntervalTimeout = 1000;
    _communication_timeout_data.ReadTotalTimeoutMultiplier = 1000;
    _communication_timeout_data.ReadTotalTimeoutConstant = 1000;
    _communication_timeout_data.WriteTotalTimeoutMultiplier = 1000;
    _communication_timeout_data.WriteTotalTimeoutConstant = 1000;
    if (!SetCommTimeouts(_port_handler, &_communication_timeout_data))
    {
        Log("Init - Could not set communication timeout.", true);
        return false;
    }

    if (!SetCommMask(_port_handler, communication_events))
    {
        Log("Init - Could not set communication mask.", true);
        return false;
    }

    if (!GetCommState(_port_handler, &_control_settings))
    {
        Log("Init - Could not set communication state.", true);
        return false;
    }

    std::ostringstream control_settings;
    control_settings << "baud=" << baud_rate << " parity=" << parity << " data=" << data_bits << " stop=" << stop_bits;
    _control_settings.fRtsControl = RTS_CONTROL_ENABLE;
    if (!BuildCommDCBA(control_settings.str().c_str(), &_control_settings))
    {
        Log("Init - Could not get the current device control settings.", true);
        return false;
    }

    if (!SetCommState(_port_handler, &_control_settings))
    {
        Log("Init - Could not set the communication state.", true);
        PurgeComm(_port_handler, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
        return false;
    }

    PurgeComm(_port_handler, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
    Log("Init - Initialization completed with success.", false);
    return true;
}

void Serial::Thread(Serial* port)
{
    port->_thread_alive = true;
    if (port->_port_handler)
        PurgeComm(port->_port_handler, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

    while (true)
    {
        // WaitCommEvent will return imediatelly due FILE_FLAG_OVERLAPPED
        // and initilize the _async_io_data.hEvent to be signed in case something arrives.
        unsigned long event_type = 0;
        if (WaitCommEvent(port->_port_handler, &event_type, &port->_async_io_data))
        {
            unsigned long error_type = 0;
            COMSTAT device_status;
            if (!ClearCommError(port->_port_handler, &error_type, &device_status))
                port->Log("Thread - Failed to clean the communication error.", true);

            if (device_status.cbInQue == 0)
            {
                // We recevied a signal but didi not
                if (WaitForSingleObject(port->_shutdown_event_handler, 100) == WAIT_OBJECT_0)
                {
                    // Shutdown Event
                    port->_thread_alive = false;
                    return;
                }

                continue;
            }
        }
        else
        {
            switch (GetLastError())
            {
            // case 87:
            case ERROR_IO_PENDING:
                break;

            default:
                port->Log("Thread - Failed to get event from the communication device.", true);
                break;
            }
        }

        HANDLE events_handle[3];
        events_handle[0] = port->_shutdown_event_handler;
        events_handle[1] = port->_async_io_data.hEvent;
        events_handle[2] = port->_write_event_handler;
        event_type = 0;
        switch (WaitForMultipleObjects(3, events_handle, FALSE, INFINITE))
        {
        case 0:
            // Shutdown Event
            port->_thread_alive = false;
            return;

        case 1:
            // Read Event
            GetCommMask(port->_port_handler, &event_type);
            if (event_type & EV_BREAK)
                ;  // port->Log("Thread - BREAK: A break was detected on input.", false);

            if (event_type & EV_CTS)
                ;  // port->Log("Thread - CTS: Clear-to-send signal changed state.", false);

            if (event_type & EV_DSR)
                ;  // port->Log("Thread - DSR: Data-set-ready signal changed state.", false);

            if (event_type & EV_ERR)
                ;  // port->Log("Thread - ERR: A line-status error occurred.", false);

            if (event_type & EV_RING)
                ;  // port->Log("Thread - RING: A ring indicator was detected.", false);

            if (event_type & EV_RLSD)
                ;  // port->Log("Thread - RLSD: Receive-line-signal-detect signal changed state.", false);

            if (event_type & EV_RXCHAR)
            {
                Receive(port);
            }

            if (event_type & EV_RXFLAG)
                ;  // port->Log("Thread - RXFLAG: Event character was received and placed in the input buffer.", false);

            if (event_type & EV_TXEMPTY)
                ;  // port->Log("Thread - TXEMPTY: The last character in the output buffer was sent.", false);

            break;

        case 2:
            // Write Event
            Send(port);
            break;
        }
    }

    port->_thread_alive = false;
}

bool Serial::Start()
{
    if (_thread_alive)
        return true;

    _thread = std::shared_ptr<std::thread>(new std::thread(Serial::Thread, this));
    if (_thread == nullptr)
        return false;

    return true;
}

bool Serial::Stop()
{
    if (!_thread_alive)
        return true;

    SetEvent(_shutdown_event_handler);
    _thread->join();
    if(_port_handler != nullptr)
    {
      CloseHandler(_port_handler);
      _port_handler = nullptr;
    }

    return true;
}

bool Serial::Restart()
{
    Stop();
    Start();
    return true;
}

void Serial::Log(const char* message, bool show_error)
{
    unsigned int error = 0;
    std::string error_message;
    if (show_error)
    {
        error = GetLastError();
        char* error_message_buffer = nullptr;
        unsigned int buffer_size = FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL,
            error, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (char*)&error_message_buffer, 0, NULL);
        error_message = std::string(error_message_buffer, buffer_size);
        LocalFree(error_message_buffer);
    }

    std::time_t date_time = std::time(nullptr);
    std::ostringstream full_message;
    std::string time_message = std::asctime(std::localtime(&date_time));
    time_message.pop_back();
    full_message << time_message << ",COM" << _port_number << "," << message << "," << error << "," << error_message;

    CriticalSessionGuard session_guard(&_sync_communication);
    _logs.push_back(full_message.str());
}

void Serial::Send(Serial* port)
{
    unsigned long message_size = port->_write_buffer.size();
    unsigned long bytes_sent = 0;
    ResetEvent(port->_write_event_handler);

    CriticalSessionGuard(&port->_sync_communication);
    port->_async_io_data.Offset = 0;
    port->_async_io_data.OffsetHigh = 0;
    PurgeComm(port->_port_handler, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
    if (!WriteFile(port->_port_handler, &port->_write_buffer[0], message_size, &bytes_sent, &port->_async_io_data))
    {
        switch (GetLastError())
        {
        case ERROR_IO_PENDING:
        {
            bytes_sent = 0;
            if (!GetOverlappedResult(port->_port_handler, &port->_async_io_data, &bytes_sent, true))
                port->Log("Send - Could not get result", true);

            break;
        }
        default:
            port->Log("Send - Could not Write", true);
            break;
        }
    }

    if (bytes_sent != message_size)
    {
        std::ostringstream message;
        message << "Send - Error writing: Message Size:" << message_size << " ; Chars Sent: " << bytes_sent;
        port->Log(message.str().c_str(), false);
        return;
    }

    SetEvent(port->_write_completed_event_handler);
}

void Serial::Receive(Serial* port)
{
    for (;;)
    {
        CriticalSessionGuard(&port->_sync_communication);
        unsigned long error_type = 0;
        COMSTAT device_status;
        if (!ClearCommError(port->_port_handler, &error_type, &device_status))
            port->Log("Receive - Could not clear error", true);

        if (device_status.cbInQue == 0)
            break;

        unsigned char read_byte;
        unsigned long quantity_of_bytes_read = 0;
        if (!ReadFile(port->_port_handler, &read_byte, 1, &quantity_of_bytes_read, &port->_async_io_data))
        {
            switch (GetLastError())
            {
            case ERROR_IO_PENDING:
                if (!GetOverlappedResult(port->_port_handler, &port->_async_io_data, &quantity_of_bytes_read, true))
                    port->Log("Receive - Could not clear result of the assynchronous (overlapped) operation", true);

                break;

            default:
                port->Log("Receive - Error to read", true);
                break;
            }
        }

        port->_read_buffer.push_back(read_byte);
    }
}

bool Serial::Read(int time_out, const char end_of_message, std::vector<char>* buffer)
{
    if (buffer == nullptr)
    {
        assert(false);
        return false;
    }

    buffer->clear();
    for (unsigned long end_time = GetTickCount() + time_out; GetTickCount() <= end_time;)
    {
        {
            CriticalSessionGuard session_guard(&_sync_communication);
            _consumer_buffer.insert(_consumer_buffer.end(), _read_buffer.begin(), _read_buffer.end());
            _read_buffer.clear();
        }

        while (!_consumer_buffer.empty())
        {
            char value = _consumer_buffer[0];
            buffer->push_back(value);
            _consumer_buffer.erase(_consumer_buffer.begin());
            if (value == end_of_message)
            {
                Log("Read - Executed with success!", false);
                return true;
            }
        }

        Sleep(100);
    }

    Log("Read - Timeout", false);
    return false;
}

bool Serial::Write(int time_out, const std::vector<char>& buffer)
{
    if (buffer.empty())
        return true;

    if (_write_completed_event_handler != nullptr)
        ResetEvent(_write_completed_event_handler);

    {
        CriticalSessionGuard session_guard(&_sync_communication);
        _write_buffer.insert(_write_buffer.end(), buffer.begin(), buffer.end());
        SetEvent(_write_event_handler);
    }

    DWORD test = WaitForSingleObject(_write_completed_event_handler, time_out);
    if (test != WAIT_OBJECT_0)
    {
        Log("Write - Timeout", false);
        return false;
    }

    Log("Write - Executed with success!", false);
    return true;
}

bool Serial::GetLogs(std::vector<std::string>* logs)
{
    if (logs == nullptr)
    {
        assert(false);
        return false;
    }

    CriticalSessionGuard session_guard(&_sync_communication);
    *logs = _logs;
    _logs.clear();

    return true;
}
