---@diagnostic disable: undefined-global

-- ACNextGen module adapter for the bounded CSP Physics Worker bridge.
-- Copy to ACNextGen/modules/physics_worker_bridge.lua and apply the small
-- ACNextGen.lua registration/UI patch described in the integration notes.

local Bridge = require("modules.ACNextGenPhysicsBridge")
local M = {}

local state = {
    workerMessage = "Worker stopped",
    safetyMessage = "Waiting for offline, parked, neutral vehicle",
    lastPublish = "none",
}

local function number(v, fallback)
    local n = tonumber(v)
    if not n or n ~= n or n == math.huge or n == -math.huge then return fallback end
    return n
end

local function field(obj, key, fallback)
    if not obj then return fallback end
    local ok, value = pcall(function() return obj[key] end)
    if not ok or value == nil then return fallback end
    return value
end

local function physicsAllowed()
    if not physics or type(physics.allowed) ~= "function" then return false end
    local ok, allowed = pcall(physics.allowed)
    return ok and allowed == true
end

local function safetySnapshot(car, runtime)
    local sim = nil
    if ac and type(ac.getSim) == "function" then
        local ok, value = pcall(ac.getSim)
        if ok then sim = value end
    end

    local offline = sim ~= nil and field(sim, "isOnlineRace", nil) == false
    local apiAllowed = physicsAllowed()
    local speedKmh = math.abs(number(field(car, "speedKmh", math.huge), math.huge))
    local gear = math.floor(number(field(car, "gear", 999), 999))
    local gas = number(field(car, "gas", 1), 1)
    local brake = number(field(car, "brake", 1), 1)
    local handbrake = number(field(car, "handbrake", 1), 1)
    local vehicleReady = runtime
        and runtime.carOK == true
        and runtime.wheelsOK == true
        and speedKmh <= 0.10
        and gear == 0
        and gas <= 0.01
        and brake <= 0.01
        and handbrake <= 0.01

    return {
        offline = offline,
        extendedPhysics = apiAllowed,
        vehicleReady = vehicleReady == true,
    }, speedKmh, gear, gas, brake, handbrake
end

function M.init()
    Bridge.init({ maxForceN = 80.0, publishInterval = 1 / 60 })
end

function M.update(dt, car, runtime)
    local safety, speedKmh, gear, gas, brake, handbrake = safetySnapshot(car, runtime)
    Bridge.setSafetyState(safety)
    Bridge.update(dt)
    local status = Bridge.getStatus()

    if not safety.offline then
        state.safetyMessage = "Blocked: offline session required"
    elseif not safety.extendedPhysics then
        state.safetyMessage = "Blocked: physics.allowed() is false"
    elseif not safety.vehicleReady then
        state.safetyMessage = string.format(
            "Park/neutral/release controls (%.2f km/h, gear %d, pedals %.2f/%.2f/%.2f)",
            speedKmh, gear, gas, brake, handbrake
        )
    elseif status.safetyReady then
        state.safetyMessage = "READY: offline / physics allowed / parked"
    end
end

-- Public hook for a future ACNextGen module that produces a validated force
-- residual. Raw tire_force estimates are intentionally rejected by the bridge.
function M.publishResidual(output)
    local ok, result = Bridge.publishModuleOutput(output)
    state.lastPublish = tostring(output and output.source or "unknown") .. ": " .. tostring(result)
    return ok, result
end

function M.drawUI()
    if not ui then return end
    local status = Bridge.getStatus()

    ui.separator()
    ui.text("CSP Physics Worker bridge (diagnostic; disabled by default)")
    ui.text("Safety: " .. state.safetyMessage)
    ui.text("Worker: " .. tostring(status.workerStatus))
    ui.text("Bridge: " .. tostring(status.status) .. " / enabled=" .. tostring(status.enabled))

    if ui.button("Start physics worker") then
        if status.safetyReady then
            local ok, message = Bridge.startWorker()
            state.workerMessage = tostring(message)
            if not ok then state.safetyMessage = tostring(message) end
        else
            state.workerMessage = "Start refused: safety interlock not ready"
        end
    end

    if not status.enabled then
        if ui.button("Enable bridge (manual test only)") then
            local ok, message = Bridge.setEnabled(true)
            state.workerMessage = ok and "Bridge enabled" or tostring(message)
        end
    else
        if ui.button("Disable bridge") then
            Bridge.clear("USER_DISABLED")
            Bridge.setEnabled(false)
            state.workerMessage = "Bridge disabled"
        end
        if ui.button("Send +local-Z 80 N / max 8 frames") then
            local ok, message = Bridge.publishForce(0, 0, 0, 80)
            state.lastPublish = "+local-Z: " .. tostring(message)
            if not ok then state.workerMessage = tostring(message) end
        end
        if ui.button("Send -local-Z 80 N / max 8 frames") then
            local ok, message = Bridge.publishForce(0, 0, 0, -80)
            state.lastPublish = "-local-Z: " .. tostring(message)
            if not ok then state.workerMessage = tostring(message) end
        end
    end

    if ui.button("Stop physics worker") then
        Bridge.stopWorker()
        state.workerMessage = "Stop requested"
    end

    ui.text("Consumed/applied seq: " .. tostring(status.workerConsumedSequence)
        .. "/" .. tostring(status.workerAppliedSequence))
    ui.text("Applied physics frames: " .. tostring(status.workerAppliedFrames))
    ui.text("Accepted module: " .. tostring(status.workerAcceptedSource))
    ui.text("Worker error: " .. tostring(status.workerError))
    ui.text("Last publish: " .. state.lastPublish)
    ui.text("Control: " .. state.workerMessage)
    ui.text("Model residual connection: not installed")
end

return M
