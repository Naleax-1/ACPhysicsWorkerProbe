---@diagnostic disable: undefined-global

-- Bounded ACNextGen force bridge worker. Independent from the calibration
-- worker; it is deliberately inert until the parent explicitly enables it.

local K = {
    enable = "ngp_bridge_v1_enable",
    stop = "ngp_bridge_v1_stop",
    heartbeat = "ngp_bridge_v1_heartbeat",
    sequence = "ngp_bridge_v1_sequence",
    consumed = "ngp_bridge_v1_consumed",
    target = "ngp_bridge_v1_target_car",
    x = "ngp_bridge_v1_force_x",
    y = "ngp_bridge_v1_force_y",
    z = "ngp_bridge_v1_force_z",
    maxMagnitude = "ngp_bridge_v1_max_magnitude",
    status = "ngp_bridge_v1_status",
    applied = "ngp_bridge_v1_applied_sequence",
    appliedFrames = "ngp_bridge_v1_applied_frames",
    source = "ngp_bridge_v1_source",
    acceptedSource = "ngp_bridge_v1_accepted_source",
    error = "ngp_bridge_v1_error",
}

local MAX_FORCE_N = 80.0
local MAX_HOLD_FRAMES = 8
local ENABLE_TIMEOUT_SEC = 0.25
local HEARTBEAT_TIMEOUT_SEC = 0.20

local zeroPosition = vec3(0, 0, 0)
local force = vec3(0, 0, 0)
local elapsed = 0
local lastHeartbeat = -1
local heartbeatAge = math.huge
local lastSequence = 0
local commandAge = math.huge
local holdFramesRemaining = 0
local appliedFrames = 0

local function load(key, fallback)
    if not ac or type(ac.load) ~= "function" then return fallback end
    local ok, value = pcall(ac.load, key)
    if not ok or value == nil then return fallback end
    return value
end

local function store(key, value)
    if ac and type(ac.store) == "function" then pcall(ac.store, key, value) end
end

local function number(key, fallback)
    local value = tonumber(load(key, fallback))
    if not value or value ~= value or value == math.huge or value == -math.huge then return fallback end
    return value
end

local function stopWith(code)
    force.x, force.y, force.z = 0, 0, 0
    holdFramesRemaining = 0
    store(K.error, code)
    store(K.status, "INHIBITED_" .. tostring(code))
end

local function consumeCommand()
    local sequence = math.floor(number(K.sequence, 0))
    if sequence <= lastSequence then return false end

    local target = math.floor(number(K.target, -1))
    local source = tostring(load(K.source, "unknown"))
    local x, y, z = number(K.x, 0), number(K.y, 0), number(K.z, 0)
    local maxMagnitude = math.min(math.max(number(K.maxMagnitude, MAX_FORCE_N), 0), MAX_FORCE_N)
    local norm = math.sqrt(x * x + y * y + z * z)
    if target ~= 0 or norm > maxMagnitude + 1e-6 then
        lastSequence = sequence
        commandAge = math.huge
        stopWith(4)
        return false
    end
    lastSequence = sequence
    commandAge = 0
    force.x, force.y, force.z = x, y, z
    holdFramesRemaining = norm > 1e-9 and MAX_HOLD_FRAMES or 0
    store(K.consumed, sequence)
    store(K.acceptedSource, source)
    store(K.error, 0)
    store(K.status, "COMMAND_ACCEPTED")
    return true
end

local function apply()
    local norm = math.sqrt(force.x * force.x + force.y * force.y + force.z * force.z)
    if norm <= 1e-9 or holdFramesRemaining <= 0 then return end
    local ok = pcall(function()
        physics.addForce(0, zeroPosition, true, force, true, -1)
    end)
    if not ok then stopWith(5); return end
    store(K.applied, lastSequence)
    appliedFrames = appliedFrames + 1
    store(K.appliedFrames, appliedFrames)
    holdFramesRemaining = holdFramesRemaining - 1
    store(K.status, "FORCE_APPLIED_FRAME")
    if holdFramesRemaining == 0 then
        force.x, force.y, force.z = 0, 0, 0
        store(K.status, "HOLD_EXPIRED")
    end
end

function script.update(dt)
    dt = math.min(math.max(tonumber(dt) or 0.003, 0.0001), 0.010)
    elapsed = elapsed + dt

    if number(K.stop, 0) == 1 then
        force.x, force.y, force.z = 0, 0, 0
        store(K.enable, 0)
        store(K.status, "STOPPED")
        if worker and type(worker.terminate) == "function" then pcall(worker.terminate) end
        return
    end
    commandAge = commandAge + dt

    local heartbeat = math.floor(number(K.heartbeat, 0))
    if heartbeat ~= lastHeartbeat then
        lastHeartbeat = heartbeat
        heartbeatAge = 0
    else
        heartbeatAge = heartbeatAge + dt
    end

    if number(K.enable, 0) ~= 1 then
        force.x, force.y, force.z = 0, 0, 0
        store(K.status, "DISABLED")
        return
    end

    if heartbeatAge > HEARTBEAT_TIMEOUT_SEC then stopWith(2); return end
    consumeCommand()
    if commandAge > ENABLE_TIMEOUT_SEC then stopWith(3); return end
    apply()
end
