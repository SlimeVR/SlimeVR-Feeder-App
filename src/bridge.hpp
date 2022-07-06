#pragma once
#include <ProtobufMessages.pb.h>

enum BridgeStatus {
    BRIDGE_DISCONNECTED = 0,
    BRIDGE_CONNECTED = 1,
    BRIDGE_ERROR = 2,
};

class SlimeVRBridge {
    public:
        SlimeVRBridge() {}

        virtual ~SlimeVRBridge() {};

        BridgeStatus status;

        // returns true if the pipe has *just* (re-)connected
        bool runFrame();

        virtual bool getNextMessage(messages::ProtobufMessage &msg) = 0;
        virtual bool sendMessage(messages::ProtobufMessage &msg) = 0;

        static std::unique_ptr<SlimeVRBridge> factory();

    private:
        virtual void connect() = 0;
        virtual void reset() = 0;
        virtual void update() = 0;
};