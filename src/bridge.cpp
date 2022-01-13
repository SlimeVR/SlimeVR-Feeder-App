#include <fstream>
#include <vector>
#include <fmt/core.h>
#include <fmt/ostream.h>
#include "bridge.hpp"

#if defined(_WIN32)
#include <windows.h>

class NamedPipeBridge final: public SlimeVRBridge {
    private:
        static constexpr char* pipe_name = "\\\\.\\pipe\\SlimeVRInput";
        HANDLE pipe = INVALID_HANDLE_VALUE;
        // our buffer "size" will probably always be 0, we're just making use of the capacity.
        std::vector<uint8_t> buffer;

        void pipe_error() {
            status = BRIDGE_ERROR;
            fmt::print("Bridge error: 0x{:x}", GetLastError());
        }
    public:
        bool getNextMessage(messages::ProtobufMessage &msg) final override {
            if (status != BRIDGE_CONNECTED) {
                return false;
            }

            DWORD read_bytes = 0;
            uint8_t size_bytes[4];
            if (!PeekNamedPipe(pipe, size_bytes, 4, &read_bytes, NULL, NULL)) {
                pipe_error();
                return false;
            } else if (read_bytes != 4) {
                return false;
            }

            DWORD size = size_bytes[0] | size_bytes[1] << 8 | size_bytes[2] << 16 | size_bytes[3] << 24;
            buffer.reserve(size);
            if (!ReadFile(pipe, buffer.data(), size, &read_bytes, NULL)) {
                pipe_error();
                return false;
            }

            return msg.ParseFromArray(buffer.data() + 4, size - 4);
        }

        bool sendMessage(messages::ProtobufMessage &msg) final override {
            if (status != BRIDGE_CONNECTED) {
                return false;
            }

            DWORD size = msg.ByteSizeLong() + 4; // wire size includes 4 bytes for size
            buffer.reserve(size);

            buffer[0] = size & 0xFF;
            buffer[1] = (size >> 8) & 0xFF;
            buffer[2] = (size >> 16) & 0xFF;
            buffer[3] = (size >> 24) & 0xFF;
            if (!msg.SerializeToArray(buffer.data() + 4, size - 4)) {
                return false;
            }
            
            DWORD _written = 0;
            if (!WriteFile(pipe, buffer.data(), size, &_written, NULL)) {
                pipe_error();
                return false;
            }

            return true;
        }
    private:
        void connect() final override {
            pipe = CreateFileA(pipe_name, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
            if (pipe != INVALID_HANDLE_VALUE) {
                status = BRIDGE_CONNECTED;
                fmt::print("Pipe was connected!");
            }
        }
        virtual void reset() final override {
            if (pipe != INVALID_HANDLE_VALUE) {
                CloseHandle(pipe);
                pipe = INVALID_HANDLE_VALUE;
                status = BRIDGE_DISCONNECTED;
                fmt::print("Pipe was reset.");
            }
        }
        virtual void update() final override {}
};

#endif

bool SlimeVRBridge::runFrame() {
    switch (status) {
        case BRIDGE_DISCONNECTED:
            connect();
            return status == BRIDGE_CONNECTED;
        case BRIDGE_ERROR:
            reset();
            return false;
        case BRIDGE_CONNECTED:
            update();
            return false;
    }
}

// TODO: take some kind of configuration input for switching between named pipes, unix sockets, websockets?
std::unique_ptr<SlimeVRBridge> SlimeVRBridge::factory() {
#if defined(_WIN32)
    return std::make_unique<NamedPipeBridge>();
#else
    #error Unsupported platform
#endif
}

