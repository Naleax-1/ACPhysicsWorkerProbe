---@diagnostic disable: undefined-global

--============================================================
-- ACPhysicsWorkerProbe.lua
-- Physics Worker Probe v1.5.1
--
-- FINAL PROBE:
--   local Z force, raw magnitude 80
--   durations 1 / 2 / 4 / 8 physics frames
--   five positive/negative pairs per duration
--
-- Confirms:
--   * response grows with application duration
--   * Newton-like raw value is passed without manual dt scaling
--   * conservative validated envelope for actuator_core bring-up
--============================================================

local VERSION = "1.5"

local TEST = {
    rawForce = 80.0,
    durations = { 1, 2, 4, 8 },
    repeats = 5,

    initialSettleSec = 0.800,
    prepSec = 0.250,
    betweenSignsSec = 0.350,
    betweenPairsSec = 0.450,

    armSec = 10.0,

    appMaxSpeedKmh = 0.50,
    workerMaxSpeedMps = 0.60,

    maxGas = 0.03,
    maxBrake = 0.08,
    maxHandbrake = 0.05,
}

local K = {
    command = "ngp_pw_v151_command",
    commandSeq = "ngp_pw_v151_command_seq",
    appHeartbeat = "ngp_pw_v151_app_heartbeat",

    workerFrame = "ngp_pw_v151_worker_frame",
    workerHz = "ngp_pw_v151_worker_hz",

    phase = "ngp_pw_v151_phase",
    phaseTime = "ngp_pw_v151_phase_time",
    durationIndex = "ngp_pw_v151_duration_index",
    repeatIndex = "ngp_pw_v151_repeat_index",
    pairIndex = "ngp_pw_v151_pair_index",
    durationFrames = "ngp_pw_v151_duration_frames",
    positiveFirst = "ngp_pw_v151_positive_first",
    activeApplyFrame = "ngp_pw_v151_active_apply_frame",

    forceCallOK = "ngp_pw_v151_force_call_ok",
    forceError = "ngp_pw_v151_force_error",
    lastStepDt = "ngp_pw_v151_last_step_dt",

    latestSymF1 = "ngp_pw_v151_latest_sym_f1",
    latestSymF3 = "ngp_pw_v151_latest_sym_f3",
    latestSymF10 = "ngp_pw_v151_latest_sym_f10",
    latestCommonF1 = "ngp_pw_v151_latest_common_f1",
    latestSideLeakF1 = "ngp_pw_v151_latest_side_leak_f1",
    latestUpLeakF1 = "ngp_pw_v151_latest_up_leak_f1",
    latestPolarityPass = "ngp_pw_v151_latest_polarity_pass",

    completed = "ngp_pw_v151_completed",
    aborted = "ngp_pw_v151_aborted",
    abortReason = "ngp_pw_v151_abort_reason",

    calibrated = "ngp_pw_v151_calibrated",
    classificationCode = "ngp_pw_v151_classification_code",
    monotonic = "ngp_pw_v151_monotonic",
    polarityPassCount = "ngp_pw_v151_polarity_pass_count",
    totalPairs = "ngp_pw_v151_total_pairs",

    averageDt = "ngp_pw_v151_average_dt",
    effectiveDt = "ngp_pw_v151_effective_dt",
    durationModelError = "ngp_pw_v151_duration_model_error",
    linearityError = "ngp_pw_v151_linearity_error",
    leakageRatio = "ngp_pw_v151_leakage_ratio",
    orderBias = "ngp_pw_v151_order_bias",
    ratio2 = "ngp_pw_v151_ratio_2",
    ratio4 = "ngp_pw_v151_ratio_4",
    ratio8 = "ngp_pw_v151_ratio_8",
    peakObservedSpeed = "ngp_pw_v151_peak_observed_speed",

    mean1 = "ngp_pw_v151_mean_1",
    mean2 = "ngp_pw_v151_mean_2",
    mean4 = "ngp_pw_v151_mean_4",
    mean8 = "ngp_pw_v151_mean_8",
}

local PHASE = {
    IDLE = 0,
    INITIAL_SETTLE = 1,
    FIRST_PREP = 2,
    FIRST_APPLY = 3,
    FIRST_CAPTURE = 4,
    BETWEEN_SIGNS = 5,
    SECOND_PREP = 6,
    SECOND_APPLY = 7,
    SECOND_CAPTURE = 8,
    PAIR_EVAL = 9,
    BETWEEN_PAIRS = 10,
    COMPLETE = 11,
    ABORTED = 12,
}

local PHASE_NAME = {
    [0] = "IDLE",
    [1] = "INITIAL SETTLE",
    [2] = "FIRST-SIGN PREP",
    [3] = "FIRST-SIGN FORCE ACTIVE",
    [4] = "FIRST-SIGN CAPTURE",
    [5] = "SIGN SETTLE",
    [6] = "SECOND-SIGN PREP",
    [7] = "SECOND-SIGN FORCE ACTIVE",
    [8] = "SECOND-SIGN CAPTURE",
    [9] = "PAIR EVALUATION",
    [10] = "NEXT-PAIR SETTLE",
    [11] = "COMPLETE",
    [12] = "ABORTED",
}

local ABORT_NAME = {
    [0] = "NONE",
    [1] = "APP HEARTBEAT LOST",
    [2] = "BASELINE SPEED TOO HIGH",
    [3] = "TEST SPEED LIMIT",
    [4] = "PHYSICS FORCE ERROR",
    [5] = "WORKER STOPPED",
    [6] = "INVALID COMMAND",
    [7] = "INVALID MASS",
}

local CLASS_NAME = {
    [0] = "PENDING",
    [1] = "DURATION LINEAR / DIRECT FORCE CONFIRMED",
    [2] = "DURATION RESPONSE PRESENT / NONLINEAR",
    [3] = "DURATION RESPONSE BELOW DETECTION",
}

local state = {
    appFrame = 0,
    dtSum = 0.0,
    appHz = 0.0,
    heartbeatTimer = 0.0,
    appHeartbeat = 0,

    carOK = false,
    wheelsOK = false,
    online = false,
    carMass = 0.0,
    speedKmh = 0.0,
    gear = 0,
    gas = 0.0,
    brake = 0.0,
    handbrake = 0.0,

    startCallable = false,
    physicsAllowed = false,

    workerRequested = false,
    workerCallback = "NOT STARTED",
    workerAlive = false,
    workerEverAlive = false,
    workerFrame = 0,
    previousWorkerFrame = 0,
    workerHz = 0.0,
    heartbeatAge = math.huge,

    armed = false,
    armTimeLeft = 0.0,
    safetyReady = false,
    safetyReason = "NOT READY",
    commandSeq = 0,

    phase = PHASE.IDLE,
    phaseTime = 0.0,
    durationIndex = 0,
    repeatIndex = 0,
    pairIndex = 0,
    durationFrames = 0,
    positiveFirst = false,
    activeApplyFrame = 0,

    forceCallOK = false,
    forceError = 0,
    lastStepDt = 0.0,

    latestSymF1 = 0.0,
    latestSymF3 = 0.0,
    latestSymF10 = 0.0,
    latestCommonF1 = 0.0,
    latestSideLeakF1 = 0.0,
    latestUpLeakF1 = 0.0,
    latestPolarityPass = false,

    completed = false,
    completedEver = false,
    aborted = false,
    abortReason = 0,

    calibrated = false,
    calibratedEver = false,
    classificationCode = 0,
    classificationCodeEver = 0,
    monotonic = false,
    polarityPassCount = 0,
    totalPairs = 0,

    averageDt = 0.0,
    effectiveDt = 0.0,
    durationModelError = 0.0,
    linearityError = 0.0,
    leakageRatio = 0.0,
    orderBias = 0.0,
    ratio2 = 0.0,
    ratio4 = 0.0,
    ratio8 = 0.0,
    peakObservedSpeed = 0.0,

    means = { 0.0, 0.0, 0.0, 0.0 },
    pairs = {},

    verdictCode = 0,
    verdict = "START WORKER",

    reportStatus = "NOT EXPORTED",
    reportPath = "",
    lastError = "",
}

local function finite(v)
    local n = tonumber(v)
    if n == nil or n ~= n
    or n == math.huge
    or n == -math.huge then
        return nil
    end
    return n
end

local function num(v, fallback)
    return finite(v) or fallback or 0.0
end

local function getSymbol(namespace, name)
    if type(namespace) ~= "table" then
        return nil
    end
    local ok, value = pcall(function()
        return namespace[name]
    end)
    return ok and value or nil
end

local function isCallable(value)
    local valueType = type(value)
    if valueType == "function"
    or valueType == "cdata" then
        return true
    end
    if valueType == "table"
    or valueType == "userdata" then
        local ok, mt = pcall(getmetatable, value)
        return ok
            and type(mt) == "table"
            and mt.__call ~= nil
    end
    return false
end

local function safeStore(key, value)
    if not ac or type(ac.store) ~= "function" then
        return false
    end
    return pcall(function()
        ac.store(key, value)
    end)
end

local function safeLoad(key, fallback)
    if not ac or type(ac.load) ~= "function" then
        return fallback
    end
    local ok, value = pcall(function()
        return ac.load(key)
    end)
    if not ok or value == nil then
        return fallback
    end
    return value
end

local function safeGetCar()
    local fn = getSymbol(ac, "getCar")
    if not isCallable(fn) then
        return nil
    end
    local ok, car = pcall(function()
        return fn(0)
    end)
    return ok and car or nil
end

local function safeGetSim()
    local fn = getSymbol(ac, "getSim")
    if not isCallable(fn) then
        return nil
    end
    local ok, sim = pcall(function()
        return fn()
    end)
    return ok and sim or nil
end

local function safeField(object, key, fallback)
    if object == nil then
        return fallback
    end
    local ok, value = pcall(function()
        return object[key]
    end)
    if ok and value ~= nil then
        return value
    end
    return fallback
end

local function detectOnline(sim)
    if not sim then
        return false
    end
    for _, key in ipairs({
        "isOnlineRace",
        "isOnline",
        "online",
    }) do
        local value = safeField(sim, key, nil)
        if value ~= nil then
            return value == true
        end
    end
    return false
end

local function pairKey(pair, field)
    return string.format(
        "ngp_pw_v151_p%02d_%s",
        pair,
        field
    )
end

local PAIR_FIELDS = {
    "duration",
    "positive_first",
    "dt_mean",
    "plus_f1",
    "plus_f3",
    "plus_f10",
    "minus_f1",
    "minus_f3",
    "minus_f10",
    "sym_f1",
    "sym_f3",
    "sym_f10",
    "common_f1",
    "side_leak_f1",
    "up_leak_f1",
    "polarity_pass",
}

local function clearShared()
    for _, key in pairs(K) do
        safeStore(key, 0)
    end

    local pairCount =
        #TEST.durations * TEST.repeats

    for pair = 1, pairCount do
        for _, field in ipairs(PAIR_FIELDS) do
            safeStore(
                pairKey(pair, field),
                0
            )
        end
    end
end

local function scanParentAPI()
    local startFn =
        getSymbol(
            physics,
            "startPhysicsWorker"
        )

    local allowedFn =
        getSymbol(
            physics,
            "allowed"
        )

    state.startCallable =
        isCallable(startFn)

    state.physicsAllowed = false

    if isCallable(allowedFn) then
        local ok, result = pcall(function()
            return allowedFn()
        end)

        state.physicsAllowed =
            ok and result == true

        if not ok then
            state.lastError =
                "physics.allowed(): " ..
                tostring(result)
        end
    end
end

local function readCarState()
    local car = safeGetCar()
    local sim = safeGetSim()

    state.carOK = car ~= nil
    state.online = detectOnline(sim)
    state.wheelsOK = false

    if car then
        state.wheelsOK =
            safeField(
                car,
                "wheels",
                nil
            ) ~= nil

        state.carMass =
            num(
                safeField(
                    car,
                    "mass",
                    0
                ),
                0
            )

        state.speedKmh =
            math.abs(
                num(
                    safeField(
                        car,
                        "speedKmh",
                        0
                    ),
                    0
                )
            )

        state.gear =
            math.floor(
                num(
                    safeField(
                        car,
                        "gear",
                        0
                    ),
                    0
                )
            )

        state.gas =
            num(
                safeField(
                    car,
                    "gas",
                    0
                ),
                0
            )

        state.brake =
            num(
                safeField(
                    car,
                    "brake",
                    0
                ),
                0
            )

        state.handbrake =
            num(
                safeField(
                    car,
                    "handbrake",
                    0
                ),
                0
            )
    else
        state.carMass = 0
        state.speedKmh = 0
        state.gear = 0
        state.gas = 0
        state.brake = 0
        state.handbrake = 0
    end
end

local function updateSafety()
    state.safetyReady = false

    if state.online then
        state.safetyReason = "ONLINE BLOCKED"
        return
    end

    if not state.workerAlive then
        state.safetyReason = "WORKER NOT RUNNING"
        return
    end

    if not state.physicsAllowed then
        state.safetyReason = "PHYSICS NOT ALLOWED"
        return
    end

    if not state.carOK then
        state.safetyReason = "CAR STATE MISSING"
        return
    end

    if state.carMass < 100
    or state.carMass > 10000 then
        state.safetyReason = "INVALID CAR MASS"
        return
    end

    if state.phase ~= PHASE.IDLE
    and state.phase ~= PHASE.COMPLETE
    and state.phase ~= PHASE.ABORTED then
        state.safetyReason = "CALIBRATION ACTIVE"
        return
    end

    if state.speedKmh >
       TEST.appMaxSpeedKmh then
        state.safetyReason =
            string.format(
                "SPEED %.3f > %.3f KM/H",
                state.speedKmh,
                TEST.appMaxSpeedKmh
            )
        return
    end

    if state.gear ~= 0 then
        state.safetyReason = "GEAR MUST BE N"
        return
    end

    if state.gas > TEST.maxGas then
        state.safetyReason = "RELEASE THROTTLE"
        return
    end

    if state.brake > TEST.maxBrake then
        state.safetyReason = "RELEASE BRAKE"
        return
    end

    if state.handbrake >
       TEST.maxHandbrake then
        state.safetyReason = "RELEASE HANDBRAKE"
        return
    end

    state.safetyReady = true
    state.safetyReason = "READY"
end

local function startWorker()
    state.lastError = ""
    scanParentAPI()

    if state.online then
        state.workerCallback = "BLOCKED ONLINE"
        return
    end

    if not state.startCallable then
        state.workerCallback =
            "START API NOT AVAILABLE"
        return
    end

    if state.carMass < 100
    or state.carMass > 10000 then
        state.workerCallback = "INVALID CAR MASS"
        return
    end

    clearShared()

    state.appHeartbeat = 0
    state.commandSeq = 0
    state.workerRequested = true
    state.workerCallback = "START REQUESTED"

    local fn =
        getSymbol(
            physics,
            "startPhysicsWorker"
        )

    local ok, err = pcall(function()
        fn(
            "physics_worker_v1_5_1",
            {
                rawForce = TEST.rawForce,
                repeats = TEST.repeats,
                initialSettleSec =
                    TEST.initialSettleSec,
                prepSec = TEST.prepSec,
                betweenSignsSec =
                    TEST.betweenSignsSec,
                betweenPairsSec =
                    TEST.betweenPairsSec,
                maxSpeedMps =
                    TEST.workerMaxSpeedMps,
                carMass = state.carMass,
            },
            function(workerError)
                if workerError
                and tostring(workerError) ~= "" then
                    state.workerCallback =
                        "STOPPED/ERROR: " ..
                        tostring(workerError)
                    state.lastError =
                        tostring(workerError)
                else
                    state.workerCallback =
                        "STOPPED NORMALLY"
                end
            end
        )
    end)

    if not ok then
        state.workerRequested = false
        state.workerCallback =
            "START ERROR: " ..
            tostring(err)
        state.lastError = tostring(err)
    end
end

local function sendCommand(command)
    state.commandSeq =
        state.commandSeq + 1

    safeStore(K.command, command)
    safeStore(
        K.commandSeq,
        state.commandSeq
    )
end

local function armCalibration()
    updateSafety()

    if not state.safetyReady then
        state.lastError =
            "Cannot arm: " ..
            state.safetyReason
        return
    end

    state.armed = true
    state.armTimeLeft = TEST.armSec
    state.lastError = ""
end

local function runCalibration()
    updateSafety()

    if not state.armed then
        state.lastError =
            "Calibration is not armed"
        return
    end

    if not state.safetyReady then
        state.lastError =
            "Safety changed: " ..
            state.safetyReason
        state.armed = false
        state.armTimeLeft = 0
        return
    end

    sendCommand(2)
    state.armed = false
    state.armTimeLeft = 0
    state.lastError = ""
end

local function stopWorker()
    state.armed = false
    state.armTimeLeft = 0

    if state.workerRequested then
        sendCommand(9)
    end
end

local function readPairs()
    local pairCount =
        #TEST.durations * TEST.repeats

    for pair = 1, pairCount do
        local row = state.pairs[pair] or {}
        for _, field in ipairs(PAIR_FIELDS) do
            row[field] =
                num(
                    safeLoad(
                        pairKey(
                            pair,
                            field
                        ),
                        0
                    ),
                    0
                )
        end
        state.pairs[pair] = row
    end
end

local function readWorkerState(dt)
    local frame =
        math.floor(
            num(
                safeLoad(
                    K.workerFrame,
                    0
                ),
                0
            )
        )

    state.workerFrame = frame
    state.workerHz =
        num(
            safeLoad(
                K.workerHz,
                0
            ),
            0
        )

    if frame ~= state.previousWorkerFrame then
        state.heartbeatAge = 0
        state.previousWorkerFrame = frame
    else
        state.heartbeatAge =
            state.heartbeatAge + dt
    end

    state.workerAlive =
        frame > 0 and
        state.heartbeatAge < 0.5

    if state.workerAlive then
        state.workerEverAlive = true
    end

    local intKeys = {
        "phase",
        "durationIndex",
        "repeatIndex",
        "pairIndex",
        "durationFrames",
        "activeApplyFrame",
        "forceError",
        "abortReason",
        "classificationCode",
        "polarityPassCount",
        "totalPairs",
    }

    for _, name in ipairs(intKeys) do
        state[name] =
            math.floor(
                num(
                    safeLoad(
                        K[name],
                        0
                    ),
                    0
                )
            )
    end

    local numberKeys = {
        "phaseTime",
        "lastStepDt",
        "latestSymF1",
        "latestSymF3",
        "latestSymF10",
        "latestCommonF1",
        "latestSideLeakF1",
        "latestUpLeakF1",
        "averageDt",
        "effectiveDt",
        "durationModelError",
        "linearityError",
        "leakageRatio",
        "orderBias",
        "ratio2",
        "ratio4",
        "ratio8",
        "peakObservedSpeed",
    }

    for _, name in ipairs(numberKeys) do
        state[name] =
            num(
                safeLoad(
                    K[name],
                    0
                ),
                0
            )
    end

    state.positiveFirst =
        num(
            safeLoad(
                K.positiveFirst,
                0
            ),
            0
        ) > 0

    state.forceCallOK =
        num(
            safeLoad(
                K.forceCallOK,
                0
            ),
            0
        ) > 0

    state.latestPolarityPass =
        num(
            safeLoad(
                K.latestPolarityPass,
                0
            ),
            0
        ) > 0

    state.completed =
        num(
            safeLoad(
                K.completed,
                0
            ),
            0
        ) > 0

    state.aborted =
        num(
            safeLoad(
                K.aborted,
                0
            ),
            0
        ) > 0

    state.calibrated =
        num(
            safeLoad(
                K.calibrated,
                0
            ),
            0
        ) > 0

    state.monotonic =
        num(
            safeLoad(
                K.monotonic,
                0
            ),
            0
        ) > 0

    state.means[1] =
        num(safeLoad(K.mean1, 0), 0)
    state.means[2] =
        num(safeLoad(K.mean2, 0), 0)
    state.means[3] =
        num(safeLoad(K.mean4, 0), 0)
    state.means[4] =
        num(safeLoad(K.mean8, 0), 0)

    if state.completed then
        state.completedEver = true
        if state.classificationCode ~= 0 then
            state.classificationCodeEver =
                state.classificationCode
        end
        if state.calibrated then
            state.calibratedEver = true
        end
    end

    readPairs()
end

local function updateVerdict()
    if not state.startCallable then
        state.verdictCode = 0
        state.verdict =
            "NO PHYSICS WORKER ROUTE"
        return
    end

    if not state.workerEverAlive then
        state.verdictCode = 1
        state.verdict =
            "START PHYSICS WORKER"
        return
    end

    if state.aborted then
        state.verdictCode = 2
        state.verdict =
            "DURATION CALIBRATION ABORTED: " ..
            (
                ABORT_NAME[
                    state.abortReason
                ] or
                "UNKNOWN"
            )
        return
    end

    if state.completedEver then
        local code =
            state.classificationCodeEver
        state.verdictCode =
            state.calibratedEver and 5 or 4
        state.verdict =
            CLASS_NAME[code] or
            "DURATION CALIBRATION COMPLETE"
        return
    end

    if state.workerAlive then
        state.verdictCode = 3
        state.verdict =
            "WORKER READY / ARM FINAL TEST"
        return
    end

    state.verdictCode = 2
    state.verdict =
        "WORKER STOPPED / HISTORY SAVED"
end

local function reportDirectory()
    local fn = getSymbol(ac, "getFolder")

    if isCallable(fn)
    and ac.FolderID
    and ac.FolderID.Root then
        local ok, path = pcall(function()
            return fn(ac.FolderID.Root)
        end)
        if ok
        and type(path) == "string"
        and path ~= "" then
            return path ..
                "/apps/lua/ACPhysicsWorkerProbe"
        end
    end

    return "apps/lua/ACPhysicsWorkerProbe"
end

local function exportReport()
    local path =
        reportDirectory() ..
        "/physics_worker_probe_v1_5_1_report.txt"

    local file, err = io.open(path, "w")

    if not file then
        state.reportStatus = "WRITE ERROR"
        state.lastError = tostring(err)
        return
    end

    file:write(
        "AC Physics Worker Probe v1.5.1\n"
    )
    file:write(
        "============================\n\n"
    )
    file:write(
        "Version: " .. VERSION .. "\n"
    )
    file:write(
        "Verdict: " ..
        state.verdict .. "\n"
    )
    file:write(
        "Verdict code: " ..
        tostring(state.verdictCode) .. "\n"
    )
    file:write(
        "Classification code: " ..
        tostring(
            state.classificationCodeEver
        ) .. "\n"
    )
    file:write(
        "Classification: " ..
        tostring(
            CLASS_NAME[
                state.classificationCodeEver
            ] or
            "UNKNOWN"
        ) .. "\n"
    )
    file:write(
        "Calibrated: " ..
        tostring(
            state.calibratedEver
        ) .. "\n\n"
    )

    file:write("SESSION\n")
    file:write(
        "Online: " ..
        tostring(state.online) .. "\n"
    )
    file:write(
        "Car OK: " ..
        tostring(state.carOK) .. "\n"
    )
    file:write(
        "Wheels OK: " ..
        tostring(state.wheelsOK) .. "\n"
    )
    file:write(
        string.format(
            "Car mass: %.6f kg\n",
            state.carMass
        )
    )
    file:write(
        "Worker ever alive: " ..
        tostring(
            state.workerEverAlive
        ) .. "\n"
    )
    file:write(
        "Worker currently alive: " ..
        tostring(
            state.workerAlive
        ) .. "\n"
    )
    file:write(
        string.format(
            "Worker rate: %.6f Hz\n",
            state.workerHz
        )
    )
    file:write(
        "Physics allowed: " ..
        tostring(
            state.physicsAllowed
        ) .. "\n\n"
    )

    file:write("FINAL TEST DESIGN\n")
    file:write(
        "Raw local-Z force: 80\n"
    )
    file:write(
        "Durations: 1, 2, 4, 8 physics frames\n"
    )
    file:write(
        "Repeats per duration: 5\n"
    )
    file:write(
        "Positive/negative paired: true\n"
    )
    file:write(
        "Sign order alternates: true\n"
    )
    file:write(
        "Manual dt scaling applied: false\n"
    )
    file:write(
        "Exact pre-fire velocity snapshot: true\n"
    )
    file:write(
        "Car awakened before every sign preparation: true\n"
    )
    file:write(
        "Polarity judged from symmetric separation: true\n"
    )
    file:write(
        "Extended Physics expected: enabled\n\n"
    )

    file:write("AGGREGATE\n")
    file:write(
        "Completed: " ..
        tostring(
            state.completedEver
        ) .. "\n"
    )
    file:write(
        "Monotonic duration response: " ..
        tostring(
            state.monotonic
        ) .. "\n"
    )
    file:write(
        "Polarity pairs passed: " ..
        tostring(
            state.polarityPassCount
        ) ..
        " / " ..
        tostring(
            state.totalPairs
        ) .. "\n"
    )
    file:write(
        string.format(
            "Average physics dt: %.9f s\n",
            state.averageDt
        )
    )
    file:write(
        string.format(
            "Effective dt from duration slope: %.9f s\n",
            state.effectiveDt
        )
    )
    file:write(
        string.format(
            "Duration-model relative error: %.9f\n",
            state.durationModelError
        )
    )
    file:write(
        string.format(
            "Duration linearity error: %.9f\n",
            state.linearityError
        )
    )
    file:write(
        string.format(
            "Axis leakage ratio: %.9f\n",
            state.leakageRatio
        )
    )
    file:write(
        string.format(
            "Sign-order bias ratio: %.9f\n",
            state.orderBias
        )
    )
    file:write(
        string.format(
            "Mean symmetric response 1 frame: %.9f m/s\n",
            state.means[1]
        )
    )
    file:write(
        string.format(
            "Mean symmetric response 2 frames: %.9f m/s\n",
            state.means[2]
        )
    )
    file:write(
        string.format(
            "Mean symmetric response 4 frames: %.9f m/s\n",
            state.means[3]
        )
    )
    file:write(
        string.format(
            "Mean symmetric response 8 frames: %.9f m/s\n",
            state.means[4]
        )
    )
    file:write(
        string.format(
            "Ratio 2/1: %.9f\n",
            state.ratio2
        )
    )
    file:write(
        string.format(
            "Ratio 4/1: %.9f\n",
            state.ratio4
        )
    )
    file:write(
        string.format(
            "Ratio 8/1: %.9f\n",
            state.ratio8
        )
    )
    file:write(
        string.format(
            "Peak observed worker speed: %.9f m/s\n\n",
            state.peakObservedSpeed
        )
    )

    file:write("ACTUATOR DECISION\n")
    file:write(
        "Pass Newton-like force directly: " ..
        tostring(
            state.calibratedEver
        ) .. "\n"
    )
    file:write(
        "Manual force*dt scaling required: false\n"
    )
    file:write(
        "Tested probe force magnitude: 80 N equivalent\n"
    )
    file:write(
        "Tested maximum duration: 8 physics frames\n"
    )
    file:write(
        string.format(
            "Tested duration at measured dt: %.9f s\n",
            state.averageDt * 8
        )
    )
    file:write(
        "Validated continuous duration: " ..
        (state.calibratedEver and "8 physics frames" or "NOT VALIDATED") ..
        "\n\n"
    )

    file:write("PAIRS\n")
    local pairCount =
        #TEST.durations * TEST.repeats

    for pair = 1, pairCount do
        local r = state.pairs[pair] or {}
        file:write(
            string.format(
                "P%02d duration=%.0f positiveFirst=%s dtMean=%.9f " ..
                "plusF1=%.9f plusF3=%.9f plusF10=%.9f " ..
                "minusF1=%.9f minusF3=%.9f minusF10=%.9f " ..
                "symF1=%.9f symF3=%.9f symF10=%.9f " ..
                "commonF1=%.9f sideLeakF1=%.9f upLeakF1=%.9f " ..
                "polarityPass=%s\n",
                pair,
                num(r.duration, 0),
                tostring(
                    num(
                        r.positive_first,
                        0
                    ) > 0
                ),
                num(r.dt_mean, 0),
                num(r.plus_f1, 0),
                num(r.plus_f3, 0),
                num(r.plus_f10, 0),
                num(r.minus_f1, 0),
                num(r.minus_f3, 0),
                num(r.minus_f10, 0),
                num(r.sym_f1, 0),
                num(r.sym_f3, 0),
                num(r.sym_f10, 0),
                num(r.common_f1, 0),
                num(r.side_leak_f1, 0),
                num(r.up_leak_f1, 0),
                tostring(
                    num(
                        r.polarity_pass,
                        0
                    ) > 0
                )
            )
        )
    end

    file:write("\nSAFETY\n")
    file:write(
        "Online calibration permitted: false\n"
    )
    file:write(
        "Manual arm required: true\n"
    )
    file:write(
        "Automatic arm expiry: true\n"
    )
    file:write(
        "Maximum raw force magnitude: 80\n"
    )
    file:write(
        "Maximum consecutive non-zero frames: 8\n"
    )
    file:write(
        "Worker speed cutoff: true\n"
    )
    file:write(
        "App heartbeat watchdog: true\n"
    )
    file:write(
        "Wheel torque used: false\n"
    )
    file:write(
        "Engine RPM override used: false\n"
    )
    file:write(
        "Control override used: false\n"
    )
    file:write(
        "Tyre or mass modification used: false\n\n"
    )

    file:write("LAST ERROR\n")
    file:write(
        tostring(
            state.lastError or ""
        ) .. "\n"
    )

    file:close()
    state.reportStatus = "EXPORTED"
    state.reportPath = path
end

local function line(label, value)
    ui.text(
        tostring(label) ..
        ": " ..
        tostring(value)
    )
end

function update(dt)
    dt =
        math.max(
            num(dt, 0.001),
            0.001
        )

    state.appFrame =
        state.appFrame + 1
    state.dtSum =
        state.dtSum + dt

    if state.dtSum > 0 then
        state.appHz =
            state.appFrame /
            state.dtSum
    end

    readCarState()
    scanParentAPI()
    readWorkerState(dt)

    state.heartbeatTimer =
        state.heartbeatTimer + dt

    if state.heartbeatTimer >= 0.05 then
        state.heartbeatTimer = 0
        state.appHeartbeat =
            state.appHeartbeat + 1
        safeStore(
            K.appHeartbeat,
            state.appHeartbeat
        )
    end

    if state.armed then
        state.armTimeLeft =
            math.max(
                state.armTimeLeft - dt,
                0
            )
        if state.armTimeLeft <= 0 then
            state.armed = false
        end
    end

    updateSafety()
    updateVerdict()
end

function windowMain()
    ui.text(
        "AC Physics Worker Probe v1.5.1"
    )
    ui.text(
        "FINAL corrected force-duration calibration"
    )
    ui.separator()

    line("Verdict", state.verdict)
    line("Verdict code", state.verdictCode)
    line("Online", state.online)
    line(
        "Car / wheels",
        tostring(state.carOK) ..
        " / " ..
        tostring(state.wheelsOK)
    )
    line(
        "Car mass",
        string.format(
            "%.2f kg",
            state.carMass
        )
    )
    line(
        "App rate",
        string.format(
            "%.2f Hz",
            state.appHz
        )
    )

    ui.separator()
    ui.text("SAFETY INTERLOCK")
    line("Safety", state.safetyReason)
    line(
        "Speed",
        string.format(
            "%.3f km/h",
            state.speedKmh
        )
    )
    line("Gear", state.gear)
    line("Gas", string.format("%.3f", state.gas))
    line("Brake", string.format("%.3f", state.brake))
    line("Handbrake", string.format("%.3f", state.handbrake))
    line("Armed", state.armed)
    line(
        "Arm time left",
        string.format(
            "%.2f s",
            state.armTimeLeft
        )
    )

    ui.separator()
    ui.text("PHYSICS WORKER")
    line("Start API", state.startCallable)
    line("Physics allowed", state.physicsAllowed)
    line("Alive", state.workerAlive)
    line("Ever alive", state.workerEverAlive)
    line("Frame", state.workerFrame)
    line(
        "Rate",
        string.format(
            "%.2f Hz",
            state.workerHz
        )
    )
    line("Callback", state.workerCallback)

    ui.separator()
    ui.text("FINAL TEST PROGRESS")
    line(
        "Duration case",
        string.format(
            "%d / %d",
            state.durationIndex,
            #TEST.durations
        )
    )
    line(
        "Repeat",
        string.format(
            "%d / %d",
            state.repeatIndex,
            TEST.repeats
        )
    )
    line(
        "Pair",
        string.format(
            "%d / %d",
            state.pairIndex,
            #TEST.durations *
                TEST.repeats
        )
    )
    line(
        "Duration frames",
        state.durationFrames
    )
    line(
        "Active force frame",
        state.activeApplyFrame
    )
    line(
        "Phase",
        PHASE_NAME[state.phase] or
        state.phase
    )
    line(
        "Positive first",
        state.positiveFirst
    )
    line(
        "Force call OK",
        state.forceCallOK
    )
    line(
        "Force error",
        state.forceError
    )
    line(
        "Last physics dt",
        string.format(
            "%.9f s",
            state.lastStepDt
        )
    )

    ui.separator()
    ui.text("LATEST PAIR")
    line(
        "Symmetric response frame1",
        string.format(
            "%.9f m/s",
            state.latestSymF1
        )
    )
    line(
        "Symmetric response frame3",
        string.format(
            "%.9f m/s",
            state.latestSymF3
        )
    )
    line(
        "Symmetric response frame10",
        string.format(
            "%.9f m/s",
            state.latestSymF10
        )
    )
    line(
        "Common-mode frame1",
        string.format(
            "%.9f m/s",
            state.latestCommonF1
        )
    )
    line(
        "Side leakage frame1",
        string.format(
            "%.9f m/s",
            state.latestSideLeakF1
        )
    )
    line(
        "Up leakage frame1",
        string.format(
            "%.9f m/s",
            state.latestUpLeakF1
        )
    )
    line(
        "Polarity pass",
        state.latestPolarityPass
    )

    ui.separator()
    ui.text("AGGREGATE")
    for i, frames in ipairs(TEST.durations) do
        line(
            string.format(
                "Mean response %d frame",
                frames
            ),
            string.format(
                "%.9f m/s",
                state.means[i]
            )
        )
    end
    line("Monotonic", state.monotonic)
    line(
        "Polarity pairs",
        string.format(
            "%d / %d",
            state.polarityPassCount,
            state.totalPairs
        )
    )
    line(
        "Average physics dt",
        string.format(
            "%.9f s",
            state.averageDt
        )
    )
    line(
        "Effective dt",
        string.format(
            "%.9f s",
            state.effectiveDt
        )
    )
    line(
        "Duration-model error",
        string.format(
            "%.6f",
            state.durationModelError
        )
    )
    line(
        "Linearity error",
        string.format(
            "%.6f",
            state.linearityError
        )
    )
    line(
        "Leakage ratio",
        string.format(
            "%.6f",
            state.leakageRatio
        )
    )
    line(
        "Order bias",
        string.format(
            "%.6f",
            state.orderBias
        )
    )
    line(
        "Ratios 2/1, 4/1, 8/1",
        string.format(
            "%.3f / %.3f / %.3f",
            state.ratio2,
            state.ratio4,
            state.ratio8
        )
    )
    line(
        "Peak worker speed",
        string.format(
            "%.6f m/s",
            state.peakObservedSpeed
        )
    )
    line(
        "Classification",
        CLASS_NAME[
            state.classificationCode
        ] or
        "UNKNOWN"
    )
    line("Calibrated", state.calibrated)

    ui.separator()

    if state.online then
        ui.text(
            "All physical tests are blocked online."
        )
    else
        if ui.button(
            "1. Start physics worker"
        ) then
            startWorker()
        end
        if ui.button(
            "2. ARM final duration test"
        ) then
            armCalibration()
        end
        if ui.button(
            "3. RUN armed final test"
        ) then
            runCalibration()
        end
        if ui.button(
            "STOP physics worker"
        ) then
            stopWorker()
        end
    end

    if ui.button(
        "Export v1.5.1 report"
    ) then
        exportReport()
    end

    line("Report", state.reportStatus)
    if state.reportPath ~= "" then
        line("Path", state.reportPath)
    end
    if state.lastError ~= "" then
        line("Last error", state.lastError)
    end

    ui.separator()
    ui.text("Use Extended Physics ON.")
    ui.text(
        "Do not touch pedals during the automatic sequence."
    )
    ui.text(
        "The corrected sequence takes roughly 40 seconds."
    )
    ui.text(
        "Raw force is ±80 for at most 8 physics frames."
    )
end

clearShared()
scanParentAPI()
updateVerdict()
