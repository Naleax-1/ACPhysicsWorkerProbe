---@diagnostic disable: undefined-global

-- ACNextGen -> CSP Physics Worker bounded command bridge.
-- This module is intended to be copied into ACNextGen's normal Lua app.
-- It publishes sequenced force requests. The worker independently checks the
-- enable lease, heartbeat, sequence, age and magnitude.

local Bridge = {}

local PREFIX = "ngp_bridge_v1_"
local K = {
    enable = PREFIX .. "enable",
    stop = PREFIX .. "stop",
    heartbeat = PREFIX .. "heartbeat",
    sequence = PREFIX .. "sequence",
    consumed = PREFIX .. "consumed",
    target = PREFIX .. "target_car",
    x = PREFIX .. "force_x",
    y = PREFIX .. "force_y",
    z = PREFIX .. "force_z",
    maxMagnitude = PREFIX .. "max_magnitude",
    status = PREFIX .. "status",
    applied = PREFIX .. "applied_sequence",
    appliedFrames = PREFIX .. "applied_frames",
    source = PREFIX .. "source",
    acceptedSource = PREFIX .. "accepted_source",
    error = PREFIX .. "error",
}

local DEFAULTS = {
    maxForceN = 80.0,
    publishInterval = 1 / 60,
    safetyTimeout = 0.10,
}

local state = {
    enabled = false,
    sequence = 0,
    heartbeat = 0,
    timer = 0,
    initialized = false,
    status = "DISABLED",
    workerRequested = false,
    workerStatus = "NOT_STARTED",
    safety = { offline = false, extendedPhysics = false, vehicleReady = false },
    safetyAge = math.huge,
}

local function validNumber(v)
    return type(v) == "number" and v == v and v > -math.huge and v < math.huge
end

local function safeStore(key, value)
    if not ac or type(ac.store) ~= "function" then return false end
    return pcall(ac.store, key, value)
end

local function safeLoad(key, fallback)
    if not ac or type(ac.load) ~= "function" then return fallback end
    local ok, value = pcall(ac.load, key)
    if not ok or value == nil then return fallback end
    return value
end

local function magnitude(x, y, z)
    return math.sqrt(x * x + y * y + z * z)
end

local function publishZero(status)
    state.sequence = state.sequence + 1
    safeStore(K.x, 0)
    safeStore(K.y, 0)
    safeStore(K.z, 0)
    safeStore(K.target, 0)
    safeStore(K.sequence, state.sequence)
    state.status = status or "ZERO_PUBLISHED"
    safeStore(K.status, state.status)
end

function Bridge.init(options)
    options = options or {}
    state.maxForceN = math.min(math.max(tonumber(options.maxForceN) or DEFAULTS.maxForceN, 0), DEFAULTS.maxForceN)
    state.publishInterval = math.min(math.max(tonumber(options.publishInterval) or DEFAULTS.publishInterval, 1 / 120), 0.10)
    state.enabled = false
    state.initialized = true
    safeStore(K.maxMagnitude, state.maxForceN)
    safeStore(K.enable, 0)
    safeStore(K.stop, 0)
    safeStore(K.heartbeat, 0)
    publishZero("DISABLED")
    return true
end

-- Start the dedicated bridge worker through CSP. Keep it inert until the
-- application has explicitly passed its own offline/session safety checks.
function Bridge.startWorker(options, callback)
    options = options or {}
    if not state.initialized then Bridge.init() end
    if state.workerRequested then return true, "ALREADY_REQUESTED" end
    if not physics or type(physics.startPhysicsWorker) ~= "function" then
        state.workerStatus = "START_API_UNAVAILABLE"
        return false, state.workerStatus
    end
    if not ac or type(ac.store) ~= "function" or type(ac.load) ~= "function" then
        state.workerStatus = "SHARED_DATA_API_UNAVAILABLE"
        return false, state.workerStatus
    end
    if type(physics.allowed) ~= "function" then
        state.workerStatus = "PHYSICS_ALLOWED_API_UNAVAILABLE"
        return false, state.workerStatus
    end
    local allowedOK, allowed = pcall(physics.allowed)
    if not allowedOK or allowed ~= true then
        state.workerStatus = "PHYSICS_NOT_ALLOWED"
        return false, state.workerStatus
    end
    safeStore(K.enable, 0)
    safeStore(K.stop, 0)
    local ok, err = pcall(function()
        physics.startPhysicsWorker(
            "physics_worker_bridge_v1",
            options,
            function(workerError)
                state.workerRequested = false
                state.workerStatus = workerError and ("STOPPED_ERROR: " .. tostring(workerError)) or "STOPPED"
                state.enabled = false
                safeStore(K.enable, 0)
                publishZero("WORKER_STOPPED")
                if type(callback) == "function" then pcall(callback, workerError) end
            end
        )
    end)
    if not ok then
        state.workerStatus = "START_ERROR: " .. tostring(err)
        return false, state.workerStatus
    end
    state.workerRequested = true
    state.workerStatus = "START_REQUESTED"
    return true, state.workerStatus
end

function Bridge.stopWorker()
    state.enabled = false
    safeStore(K.enable, 0)
    publishZero("STOP_REQUESTED")
    safeStore(K.stop, 1)
    state.workerStatus = "STOP_REQUESTED"
end

-- Explicit activation is required. Call only after validating offline mode,
-- Extended Physics and the selected car for the current session.
function Bridge.setEnabled(enabled)
    if not state.initialized then Bridge.init() end
    local requested = enabled == true
    local safety = state.safety
    if requested and not state.workerRequested then
        state.enabled = false
        safeStore(K.enable, 0)
        publishZero("REJECTED_WORKER_NOT_RUNNING")
        return false, "WORKER_NOT_RUNNING"
    end
    if requested and not (safety.offline and safety.extendedPhysics and safety.vehicleReady and state.safetyAge <= DEFAULTS.safetyTimeout) then
        state.enabled = false
        safeStore(K.enable, 0)
        publishZero("REJECTED_SAFETY_INTERLOCK")
        return false, "SAFETY_INTERLOCK"
    end
    state.enabled = requested
    safeStore(K.enable, state.enabled and 1 or 0)
    if state.enabled then
        state.status = "ENABLED_WAITING_FOR_COMMAND"
    else
        publishZero("DISABLED")
    end
    safeStore(K.status, state.status)
    return state.enabled
end

-- The ACNextGen host must refresh these live conditions from its own trusted
-- session/car APIs. Loss of any condition immediately disables output.
function Bridge.setSafetyState(safety)
    safety = safety or {}
    state.safety = {
        offline = safety.offline == true,
        extendedPhysics = safety.extendedPhysics == true,
        vehicleReady = safety.vehicleReady == true,
    }
    state.safetyAge = 0
    if state.enabled and not (state.safety.offline and state.safety.extendedPhysics and state.safety.vehicleReady) then
        state.enabled = false
        safeStore(K.enable, 0)
        publishZero("SAFETY_INTERLOCK_LOST")
    end
end

-- Force components must already be expressed in the CSP worker's documented
-- world/local coordinate convention. Target car index is explicit.
local function publishVector(carIndex, x, y, z, source)
    if not state.initialized then Bridge.init() end
    if not state.enabled then
        state.status = "REJECTED_DISABLED"
        safeStore(K.status, state.status)
        return false, state.status
    end
    if not validNumber(x) or not validNumber(y) or not validNumber(z) then
        publishZero("REJECTED_NONFINITE")
        return false, state.status
    end
    carIndex = math.floor(tonumber(carIndex) or -1)
    if carIndex ~= 0 then
        publishZero("REJECTED_UNSUPPORTED_CAR")
        return false, state.status
    end
    local norm = magnitude(x, y, z)
    if norm > state.maxForceN then
        local scale = state.maxForceN / norm
        x, y, z = x * scale, y * scale, z * scale
        state.status = "CLAMPED"
    else
        state.status = "PUBLISHED"
    end

    -- Publish payload before sequence: the worker treats sequence as commit.
    safeStore(K.x, x)
    safeStore(K.y, y)
    safeStore(K.z, z)
    safeStore(K.target, carIndex)
    safeStore(K.source, source or "manual")
    state.sequence = state.sequence + 1
    safeStore(K.sequence, state.sequence)
    safeStore(K.status, state.status)
    return true, state.status
end

-- Module outputs must explicitly describe a force delta in Newtons and the
-- local vehicle frame expected by forceLocal=true. Estimated total tire force
-- signals must not be passed here without a validated residual calculation.
function Bridge.publishModuleOutput(output)
    if type(output) ~= "table"
    or type(output.source) ~= "string"
    or output.source == ""
    or #output.source > 48
    or not output.source:match("^[%w_%-]+$")
    or output.semantic ~= "residual_force"
    or output.units ~= "N"
    or output.frame ~= "car_local" then
        if state.enabled then publishZero("REJECTED_MODULE_CONTRACT") end
        return false, "MODULE_CONTRACT_REQUIRED"
    end
    return publishVector(
        output.carIndex or 0,
        output.x,
        output.y,
        output.z,
        output.source
    )
end

-- Low-level path reserved for manual calibration/bridge diagnostics.
function Bridge.publishForce(carIndex, x, y, z)
    return publishVector(carIndex, x, y, z, "manual_probe")
end

function Bridge.update(dt)
    if not state.initialized then Bridge.init() end
    dt = validNumber(dt) and math.max(dt, 0) or 0
    state.timer = state.timer + dt
    state.safetyAge = state.safetyAge + dt
    if state.timer >= state.publishInterval then
        state.timer = 0
        state.heartbeat = state.heartbeat + 1
        safeStore(K.heartbeat, state.heartbeat)
    end
    if state.enabled and not (state.safety.offline and state.safety.extendedPhysics and state.safety.vehicleReady and state.safetyAge <= DEFAULTS.safetyTimeout) then
        state.enabled = false
        safeStore(K.enable, 0)
        publishZero("SAFETY_INTERLOCK_LOST")
    end
    if not state.enabled then return end
    -- The host must call publishForce() with each fresh actuator result.
    -- The worker bounds each result to eight physics frames and expires stale
    -- sequences, even while this application heartbeat continues.
end

function Bridge.clear(status)
    if state.initialized then publishZero(status or "CLEARED") end
    state.enabled = false
    safeStore(K.enable, 0)
end

function Bridge.getStatus()
    return {
        enabled = state.enabled,
        sequence = state.sequence,
        status = state.status,
        workerStatus = state.workerStatus,
        safetyReady = state.safety.offline and state.safety.extendedPhysics and state.safety.vehicleReady and state.safetyAge <= DEFAULTS.safetyTimeout,
        maxForceN = state.maxForceN or DEFAULTS.maxForceN,
        workerConsumedSequence = tonumber(safeLoad(K.consumed, 0)) or 0,
        workerAppliedSequence = tonumber(safeLoad(K.applied, 0)) or 0,
        workerAppliedFrames = tonumber(safeLoad(K.appliedFrames, 0)) or 0,
        workerAcceptedSource = tostring(safeLoad(K.acceptedSource, "")),
        workerError = tonumber(safeLoad(K.error, 0)) or 0,
    }
end

return Bridge
