#include <fstream>
#include <vector>
#include <fmt/core.h>
#include <fmt/ostream.h>
#include <optional>
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
            fmt::print("Bridge error: 0x{:x}\n", GetLastError());
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
                fmt::print("Pipe was connected!\n");
            }
        }
        virtual void reset() final override {
            if (pipe != INVALID_HANDLE_VALUE) {
                CloseHandle(pipe);
                pipe = INVALID_HANDLE_VALUE;
                status = BRIDGE_DISCONNECTED;
                fmt::print("Pipe was reset.\n");
            }
        }
        virtual void update() final override {}
};

#else
#include "unix_sockets.hpp"

class UnixSocketBridge final : public SlimeVRBridge {
private:
    static constexpr std::string_view SOCKET_NAME = "/tmp/SlimeVRInput"
    inline constexpr int HEADER_SIZE = 4;
    inline constexpr int BUFFER_SIZE = 1024;
    using ByteBuffer = std::array<uint8_t, BUFFER_SIZE>;

    ByteBuffer byteBuffer;
    BasicLocalClient client;

    /// @return iterator after header
    template <typename TBufIt>
    std::optional<TBufIt> WriteHeader(TBufIt bufBegin, int bufSize, int msgSize) {
        const int totalSize = msgSize + HEADER_SIZE; // include header bytes in total size
        if (bufSize < totalSize) return std::nullopt; // header won't fit

        const auto size = static_cast<uint32_t>(totalSize);
        TBufIt it = bufBegin;
        *(it++) = static_cast<uint8_t>(size);
        *(it++) = static_cast<uint8_t>(size >> 8U);
        *(it++) = static_cast<uint8_t>(size >> 16U);
        *(it++) = static_cast<uint8_t>(size >> 24U);
        return it;
    }
    /// @return iterator after header
    template <typename TBufIt>
    std::optional<TBufIt> ReadHeader(TBufIt bufBegin, int numBytesRecv, int& outMsgSize) {
        if (numBytesRecv < HEADER_SIZE) return std::nullopt; // header won't fit

        uint32_t size = 0;
        TBufIt it = bufBegin;
        size = static_cast<uint32_t>(*(it++));
        size |= static_cast<uint32_t>(*(it++)) << 8U;
        size |= static_cast<uint32_t>(*(it++)) << 16U;
        size |= static_cast<uint32_t>(*(it++)) << 24U;

        const auto totalSize = static_cast<int>(size);
        if (totalSize < HEADER_SIZE) return std::nullopt;
        outMsgSize = totalSize - HEADER_SIZE;
        return it;
    }

    void connect() final {
        if (!client.IsOpen()) {
            client.Open(SOCKET_NAME);
        }
    }
    void reset() final {
        client.Close();
    }
    void update() final {
        client.UpdateOnce();
    }

public:
    bool getNextMessage(messages::ProtobufMessage &msg) final {
        if (!client.IsOpen()) return false;

        int bytesRecv = 0;
        try {
            bytesRecv = client.RecvOnce(byteBuffer.begin(), HEADER_SIZE);
        } catch (const std::exception& e) {
            client.Close();
            fmt::print("bridge send error: {}\n", e.what());
            return false;
        }
        if (bytesRecv == 0) return false; // no message waiting

        int msgSize = 0;
        const std::optional msgBeginIt = ReadHeader(byteBuffer.begin(), bytesRecv, msgSize);
        if (!msgBeginIt) {
            fmt::print("bridge recv error: invalid message header or size\n");
            return false;
        }
        if (msgSize <= 0) {
            fmt::print("bridge recv error: empty message\n");
            return false;
        }
        try {
            if (!client.RecvAll(*msgBeginIt, msgSize)) {
                fmt::print("bridge recv error: client closed\n");
                return false;
            }
        } catch (const std::exception& e) {
            client.Close();
            fmt::print("bridge send error: {}\n", e.what());
            return false;
        }
        if (!message.ParseFromArray(&(**msgBeginIt), msgSize)) {
            fmt::print("bridge recv error: failed to parse\n");
            return false;
        }

        return true;
    }
    bool sendMessage(messages::ProtobufMessage &msg) final {
        if (!client.IsOpen()) return false;
        const auto bufBegin = byteBuffer.begin();
        const auto bufferSize = static_cast<int>(std::distance(bufBegin, byteBuffer.end()));
        const auto msgSize = static_cast<int>(message.ByteSizeLong());
        const std::optional msgBeginIt = WriteHeader(bufBegin, bufferSize, msgSize);
        if (!msgBeginIt) {
            fmt::print("bridge send error: failed to write header\n");
            return false;
        }
        if (!message.SerializeToArray(&(**msgBeginIt), msgSize)) {
            fmt::print("bridge send error: failed to serialize\n");
            return false;
        }
        int bytesToSend = static_cast<int>(std::distance(bufBegin, *msgBeginIt + msgSize));
        if (bytesToSend <= 0) {
            fmt::print("bridge send error: empty message\n");
            return false;
        }
        if (bytesToSend > bufferSize) {
            fmt::print("bridge send error: message too big\n");
            return false;
        }
        try {
            return client.Send(bufBegin, bytesToSend);
        } catch (const std::exception& e) {
            client.Close();
            fmt::print("bridge send error: {}\n" + e.what());
            return false;
        }
    }
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
        default:
            // uhhh, what?
            reset();
            status = BRIDGE_DISCONNECTED;
            return false;
    }
}

// TODO: take some kind of configuration input for switching between named pipes, unix sockets, websockets?
std::unique_ptr<SlimeVRBridge> SlimeVRBridge::factory() {
#if defined(_WIN32)
    return std::make_unique<NamedPipeBridge>();
#elif defined(__linux__)
    return std::make_unique<UnixSocketBridge>();
#else
    #error Unsupported platform
#endif
}

