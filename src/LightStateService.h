#ifndef LightStateService_h
#define LightStateService_h

/**
 *   ESP32 SvelteKit
 *
 *   A simple, secure and extensible framework for IoT projects for ESP32 platforms
 *   with responsive Sveltekit front-end built with TailwindCSS and DaisyUI.
 *   https://github.com/theelims/ESP32-sveltekit
 *
 *   Copyright (C) 2018 - 2023 rjwats
 *   Copyright (C) 2023 - 2025 theelims
 *
 *   All Rights Reserved. This software may be modified and distributed under
 *   the terms of the LGPL v3 license. See the LICENSE file for details.
 **/

#include <LightMqttSettingsService.h>
#include <FSPersistence.h>

#include <EventSocket.h>
#include <HttpEndpoint.h>
#include <MqttEndpoint.h>
#include <EventEndpoint.h>
#include <WebSocketServer.h>
#include <ESP32SvelteKit.h>

#define DEFAULT_LED_STATE false
#define DEFAULT_RETECT_SECONDS 30
#define DEFAULT_FEED_SECONDS 30
#define DEFAULT_TARGET_DISTANCE_CM 30
#define DEFAULT_RETURN_DISTANCE_CM 30
#define OFF_STATE "OFF"
#define ON_STATE "ON"

#define LIGHT_SETTINGS_ENDPOINT_PATH "/rest/lightState"
#define LIGHT_SETTINGS_SOCKET_PATH "/ws/lightState"
#define LIGHT_SETTINGS_EVENT "led"

class LightState
{
public:
    bool ledOn;
    int retect_seconds; // seconds to retract
    int feed_seconds;   // seconds to feed
    int target_distance_cm; // lower position target during approach (cm)
    int return_distance_cm; // absolute distance target to return to
    int auto_interval_min; // minutes between automatic runs (0=off)

    static void read(LightState &settings, JsonObject &root)
    {
        root["led_on"] = settings.ledOn;
    root["retect_seconds"] = settings.retect_seconds;
    root["feed_seconds"] = settings.feed_seconds;
    root["target_distance_cm"] = settings.target_distance_cm;
    root["return_distance_cm"] = settings.return_distance_cm;
    root["auto_interval_min"] = settings.auto_interval_min;
    }

    static StateUpdateResult update(JsonObject &root, LightState &lightState, const String& originID)
    {
        StateUpdateResult result = StateUpdateResult::UNCHANGED;
        bool newLed = root["led_on"] | DEFAULT_LED_STATE;
        int newRetract = root["retect_seconds"] | DEFAULT_RETECT_SECONDS;
        int newFeed = root["feed_seconds"] | DEFAULT_FEED_SECONDS;
    int newTarget = root["target_distance_cm"] | DEFAULT_TARGET_DISTANCE_CM;
    int newReturn = root["return_distance_cm"] | DEFAULT_RETURN_DISTANCE_CM;
    int newAutoInterval = root["auto_interval_min"] | 0;

        if (lightState.ledOn != newLed)
        {
            lightState.ledOn = newLed;
            result = StateUpdateResult::CHANGED;
        }
        if (lightState.retect_seconds != newRetract)
        {
            lightState.retect_seconds = newRetract;
            result = StateUpdateResult::CHANGED;
        }
        if (lightState.feed_seconds != newFeed)
        {
            lightState.feed_seconds = newFeed;
            result = StateUpdateResult::CHANGED;
        }
        if (lightState.target_distance_cm != newTarget)
        {
            // clamp to sane range (10..200 cm)
            if (newTarget < 10) newTarget = 10; if (newTarget > 200) newTarget = 200;
            lightState.target_distance_cm = newTarget;
            result = StateUpdateResult::CHANGED;
        }
        if (lightState.return_distance_cm != newReturn)
        {
            if (newReturn < 10) newReturn = 10; if (newReturn > 200) newReturn = 200;
            lightState.return_distance_cm = newReturn;
            result = StateUpdateResult::CHANGED;
        }
        if (newAutoInterval < 0) newAutoInterval = 0; if (newAutoInterval > 60) newAutoInterval = 60;
        if (lightState.auto_interval_min != newAutoInterval)
        {
            lightState.auto_interval_min = newAutoInterval;
            result = StateUpdateResult::CHANGED;
        }
        return result;
    }

    static void homeAssistRead(LightState &settings, JsonObject &root)
    {
        root["state"] = settings.ledOn ? ON_STATE : OFF_STATE;
    }

    static StateUpdateResult homeAssistUpdate(JsonObject &root, LightState &lightState, const String& originID)
    {
        String state = root["state"];
        // parse new led state
        boolean newState = false;
        if (state.equals(ON_STATE))
        {
            newState = true;
        }
        else if (!state.equals(OFF_STATE))
        {
            return StateUpdateResult::ERROR;
        }
        // change the new state, if required
        if (lightState.ledOn != newState)
        {
            lightState.ledOn = newState;
            return StateUpdateResult::CHANGED;
        }
        return StateUpdateResult::UNCHANGED;
    }
};

class LightStateService : public StatefulService<LightState>
{
public:
    LightStateService(PsychicHttpServer *server,
                      ESP32SvelteKit *sveltekit,
                      LightMqttSettingsService *lightMqttSettingsService);

    void begin();
    // Getter for current persisted state
    const LightState& getCurrentState() const { return _state; }

private:
    HttpEndpoint<LightState> _httpEndpoint;
    EventEndpoint<LightState> _eventEndpoint;
    MqttEndpoint<LightState> _mqttEndpoint;
    WebSocketServer<LightState> _webSocketServer;
    FSPersistence<LightState> _fsPersistence;
    PsychicMqttClient *_mqttClient;
    LightMqttSettingsService *_lightMqttSettingsService;

    void registerConfig();
    void onConfigUpdated();
};

#endif
