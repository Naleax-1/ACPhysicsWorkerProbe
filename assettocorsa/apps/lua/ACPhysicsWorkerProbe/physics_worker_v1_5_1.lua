---@diagnostic disable: undefined-global

--============================================================
-- physics_worker_v1_5.lua
-- Corrected final force-duration proportionality calibration.
--============================================================

local input = __input or {}

local CONFIG = {
    rawForce =
        math.min(
            math.max(
                tonumber(
                    input.rawForce
                ) or 80.0,
                1.0
            ),
            80.0
        ),

    durations = { 1, 2, 4, 8 },

    repeats =
        math.min(
            math.max(
                math.floor(
                    tonumber(
                        input.repeats
                    ) or 5
                ),
                1
            ),
            5
        ),

    initialSettleSec =
        math.min(
            math.max(
                tonumber(
                    input.initialSettleSec
                ) or 0.800,
                0.400
            ),
            1.500
        ),

    prepSec =
        math.min(
            math.max(
                tonumber(
                    input.prepSec
                ) or 0.250,
                0.150
            ),
            0.600
        ),

    betweenSignsSec =
        math.min(
            math.max(
                tonumber(
                    input.betweenSignsSec
                ) or 0.350,
                0.200
            ),
            0.800
        ),

    betweenPairsSec =
        math.min(
            math.max(
                tonumber(
                    input.betweenPairsSec
                ) or 0.450,
                0.250
            ),
            1.000
        ),

    maxSpeedMps =
        math.min(
            math.max(
                tonumber(
                    input.maxSpeedMps
                ) or 0.60,
                0.20
            ),
            0.60
        ),

    carMass =
        tonumber(
            input.carMass
        ) or 0.0,

    heartbeatTimeoutSec = 0.20,

    polarityThresholdMps = 0.000020,
    monotonicToleranceMps = 0.000050,
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

local ZERO_POSITION =
    vec3(0, 0, 0)

local FORCE_VECTOR =
    vec3(0, 0, 0)

local frame = 0
local dtSum = 0.0

local phase = PHASE.IDLE
local phaseTime = 0.0

local durationIndex = 0
local repeatIndex = 0
local pairIndex = 0
local positiveFirst = true
local firstSign = 1
local secondSign = -1

local activeApplyFrame = 0
local activeSign = 0
local captureFrame = 0

local lastCommandSeq = -1
local lastAppHeartbeat = 0
local appHeartbeatAge = 0.0

local forceCallOK = false
local forceError = 0
local lastStepDt = 0.0

local forceDtSum = 0.0
local forceCallCount = 0
local pairDtSum = 0.0
local pairDtCount = 0

local preForward = 0.0
local preSide = 0.0
local preUp = 0.0

local currentForward = {
    [1] = 0.0,
    [3] = 0.0,
    [10] = 0.0,
}

local currentSide = {
    [1] = 0.0,
    [3] = 0.0,
    [10] = 0.0,
}

local currentUp = {
    [1] = 0.0,
    [3] = 0.0,
    [10] = 0.0,
}

local plus = {
    forward = {},
    side = {},
    up = {},
}

local minus = {
    forward = {},
    side = {},
    up = {},
}

local latestSymF1 = 0.0
local latestSymF3 = 0.0
local latestSymF10 = 0.0
local latestCommonF1 = 0.0
local latestSideLeakF1 = 0.0
local latestUpLeakF1 = 0.0
local latestPolarityPass = false

local completed = false
local aborted = false
local abortReason = 0

local calibrated = false
local classificationCode = 0
local monotonic = false
local polarityPassCount = 0
local totalPairs = 0

local averageDt = 0.0
local effectiveDt = 0.0
local durationModelError = 0.0
local linearityError = 0.0
local leakageRatio = 0.0
local orderBias = 0.0
local ratio2 = 0.0
local ratio4 = 0.0
local ratio8 = 0.0
local peakObservedSpeed = 0.0

local responseSum = {
    0.0,
    0.0,
    0.0,
    0.0,
}

local sideLeakSum = {
    0.0,
    0.0,
    0.0,
    0.0,
}

local upLeakSum = {
    0.0,
    0.0,
    0.0,
    0.0,
}

local durationCount = {
    0,
    0,
    0,
    0,
}

local meanResponse = {
    0.0,
    0.0,
    0.0,
    0.0,
}

local eightPositiveFirstSum = 0.0
local eightPositiveFirstCount = 0
local eightNegativeFirstSum = 0.0
local eightNegativeFirstCount = 0

local pairRows = {}

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

local function pairKey(pair, field)
    return string.format(
        "ngp_pw_v151_p%02d_%s",
        pair,
        field
    )
end

local function dot(a, b)
    if not a or not b then
        return 0.0
    end
    return
        num(a.x, 0) * num(b.x, 0) +
        num(a.y, 0) * num(b.y, 0) +
        num(a.z, 0) * num(b.z, 0)
end

local function vectorLength(v)
    if not v then
        return 0.0
    end
    local x = num(v.x, 0)
    local y = num(v.y, 0)
    local z = num(v.z, 0)
    return math.sqrt(
        x * x + y * y + z * z
    )
end

local function readLocalKinematics()
    local transform =
        physics.getBodyTransform(
            0,
            -1,
            false
        )

    local velocity =
        physics.getPointVelocity(
            0,
            ZERO_POSITION,
            true
        )

    local forward = 0.0
    local side = 0.0
    local up = 0.0

    if transform then
        if transform.look then
            forward =
                dot(
                    velocity,
                    transform.look
                )
        end
        if transform.side then
            side =
                dot(
                    velocity,
                    transform.side
                )
        end
        if transform.up then
            up =
                dot(
                    velocity,
                    transform.up
                )
        end
    end

    return
        vectorLength(velocity),
        forward,
        side,
        up
end


local function wakeCar()
    if physics
    and type(physics.awakeCar) == "function" then
        local ok = pcall(function()
            physics.awakeCar(0)
        end)

        return ok
    end

    return false
end

local function clearPairMeasurements()
    pairDtSum = 0.0
    pairDtCount = 0

    for _, sampleFrame in ipairs({
        1,
        3,
        10,
    }) do
        plus.forward[sampleFrame] = 0.0
        plus.side[sampleFrame] = 0.0
        plus.up[sampleFrame] = 0.0

        minus.forward[sampleFrame] = 0.0
        minus.side[sampleFrame] = 0.0
        minus.up[sampleFrame] = 0.0
    end
end

local function publishPair(pair, row)
    for field, value in pairs(row) do
        safeStore(
            pairKey(
                pair,
                field
            ),
            value
        )
    end
end

local function publish()
    local hz =
        dtSum > 0 and
        frame / dtSum or
        0.0

    local duration =
        CONFIG.durations[
            durationIndex
        ] or
        0

    safeStore(K.workerFrame, frame)
    safeStore(K.workerHz, hz)
    safeStore(K.phase, phase)
    safeStore(K.phaseTime, phaseTime)
    safeStore(K.durationIndex, durationIndex)
    safeStore(K.repeatIndex, repeatIndex)
    safeStore(K.pairIndex, pairIndex)
    safeStore(K.durationFrames, duration)
    safeStore(
        K.positiveFirst,
        positiveFirst and 1 or 0
    )
    safeStore(
        K.activeApplyFrame,
        activeApplyFrame
    )
    safeStore(
        K.forceCallOK,
        forceCallOK and 1 or 0
    )
    safeStore(K.forceError, forceError)
    safeStore(K.lastStepDt, lastStepDt)

    safeStore(K.latestSymF1, latestSymF1)
    safeStore(K.latestSymF3, latestSymF3)
    safeStore(K.latestSymF10, latestSymF10)
    safeStore(K.latestCommonF1, latestCommonF1)
    safeStore(K.latestSideLeakF1, latestSideLeakF1)
    safeStore(K.latestUpLeakF1, latestUpLeakF1)
    safeStore(
        K.latestPolarityPass,
        latestPolarityPass and 1 or 0
    )

    safeStore(
        K.completed,
        completed and 1 or 0
    )
    safeStore(
        K.aborted,
        aborted and 1 or 0
    )
    safeStore(K.abortReason, abortReason)
    safeStore(
        K.calibrated,
        calibrated and 1 or 0
    )
    safeStore(
        K.classificationCode,
        classificationCode
    )
    safeStore(
        K.monotonic,
        monotonic and 1 or 0
    )
    safeStore(
        K.polarityPassCount,
        polarityPassCount
    )
    safeStore(K.totalPairs, totalPairs)

    safeStore(K.averageDt, averageDt)
    safeStore(K.effectiveDt, effectiveDt)
    safeStore(
        K.durationModelError,
        durationModelError
    )
    safeStore(K.linearityError, linearityError)
    safeStore(K.leakageRatio, leakageRatio)
    safeStore(K.orderBias, orderBias)
    safeStore(K.ratio2, ratio2)
    safeStore(K.ratio4, ratio4)
    safeStore(K.ratio8, ratio8)
    safeStore(
        K.peakObservedSpeed,
        peakObservedSpeed
    )

    safeStore(K.mean1, meanResponse[1])
    safeStore(K.mean2, meanResponse[2])
    safeStore(K.mean4, meanResponse[3])
    safeStore(K.mean8, meanResponse[4])

    for pair = 1, #pairRows do
        publishPair(
            pair,
            pairRows[pair]
        )
    end
end

local function abortTest(reason)
    phase = PHASE.ABORTED
    phaseTime = 0.0
    aborted = true
    abortReason = reason
    completed = false
end

local function updateHeartbeat(dt)
    local value =
        math.floor(
            num(
                safeLoad(
                    K.appHeartbeat,
                    0
                ),
                0
            )
        )

    if value ~= lastAppHeartbeat then
        lastAppHeartbeat = value
        appHeartbeatAge = 0.0
    else
        appHeartbeatAge =
            appHeartbeatAge + dt
    end

    if phase ~= PHASE.IDLE
    and phase ~= PHASE.COMPLETE
    and phase ~= PHASE.ABORTED
    and appHeartbeatAge >
        CONFIG.heartbeatTimeoutSec then
        abortTest(1)
    end
end

local function resetCapture()
    captureFrame = 0
    for _, sampleFrame in ipairs({
        1,
        3,
        10,
    }) do
        currentForward[sampleFrame] = 0.0
        currentSide[sampleFrame] = 0.0
        currentUp[sampleFrame] = 0.0
    end
end

local function beginPrep(newPhase, wakeBefore)
    resetCapture()
    activeApplyFrame = 0

    if wakeBefore then
        wakeCar()
    end

    phase = newPhase
    phaseTime = 0.0
end

local function startSequence()
    if CONFIG.carMass < 100
    or CONFIG.carMass > 10000 then
        abortTest(7)
        return
    end

    durationIndex = 1
    repeatIndex = 1
    pairIndex = 1

    positiveFirst = true
    firstSign = 1
    secondSign = -1

    forceCallOK = false
    forceError = 0
    lastStepDt = 0.0

    forceDtSum = 0.0
    forceCallCount = 0
    pairDtSum = 0.0
    pairDtCount = 0

    latestSymF1 = 0.0
    latestSymF3 = 0.0
    latestSymF10 = 0.0
    latestCommonF1 = 0.0
    latestSideLeakF1 = 0.0
    latestUpLeakF1 = 0.0
    latestPolarityPass = false

    completed = false
    aborted = false
    abortReason = 0
    calibrated = false
    classificationCode = 0
    monotonic = false
    polarityPassCount = 0
    totalPairs = 0

    averageDt = 0.0
    effectiveDt = 0.0
    durationModelError = 0.0
    linearityError = 0.0
    leakageRatio = 0.0
    orderBias = 0.0
    ratio2 = 0.0
    ratio4 = 0.0
    ratio8 = 0.0
    peakObservedSpeed = 0.0

    eightPositiveFirstSum = 0.0
    eightPositiveFirstCount = 0
    eightNegativeFirstSum = 0.0
    eightNegativeFirstCount = 0

    pairRows = {}

    for i = 1, 4 do
        responseSum[i] = 0.0
        sideLeakSum[i] = 0.0
        upLeakSum[i] = 0.0
        durationCount[i] = 0
        meanResponse[i] = 0.0
    end

    clearPairMeasurements()
    wakeCar()

    phase = PHASE.INITIAL_SETTLE
    phaseTime = 0.0
end

local function updateSettle(
    dt,
    limit,
    nextPhase,
    wakeNext
)
    local speed =
        readLocalKinematics()

    peakObservedSpeed =
        math.max(
            peakObservedSpeed,
            speed
        )

    if speed >
       CONFIG.maxSpeedMps then
        abortTest(2)
        return
    end

    phaseTime = phaseTime + dt

    if phaseTime >= limit then
        beginPrep(
            nextPhase,
            wakeNext == true
        )
    end
end

local function snapshotPreFire()
    local speed,
        forward,
        side,
        up =
        readLocalKinematics()

    peakObservedSpeed =
        math.max(
            peakObservedSpeed,
            speed
        )

    if speed >
       CONFIG.maxSpeedMps then
        abortTest(2)
        return false
    end

    preForward = forward
    preSide = side
    preUp = up

    return true
end

local function callForce(sign, dt)
    FORCE_VECTOR.x = 0.0
    FORCE_VECTOR.y = 0.0
    FORCE_VECTOR.z =
        CONFIG.rawForce * sign

    local ok = pcall(function()
        physics.addForce(
            0,
            ZERO_POSITION,
            true,
            FORCE_VECTOR,
            true,
            -1
        )
    end)

    if not ok then
        forceCallOK = false
        forceError = 1
        abortTest(4)
        return false
    end

    forceCallOK = true
    lastStepDt = dt
    forceDtSum = forceDtSum + dt
    forceCallCount = forceCallCount + 1
    pairDtSum = pairDtSum + dt
    pairDtCount = pairDtCount + 1

    return true
end

local function beginApply(sign, newPhase)
    if not snapshotPreFire() then
        return
    end

    activeSign = sign
    activeApplyFrame = 0
    phase = newPhase
    phaseTime = 0.0
end

local function updateApply(dt, nextCapturePhase)
    local duration =
        CONFIG.durations[
            durationIndex
        ] or
        1

    activeApplyFrame =
        activeApplyFrame + 1

    if not callForce(
        activeSign,
        dt
    ) then
        return
    end

    if activeApplyFrame >= duration then
        captureFrame = 0
        phase = nextCapturePhase
        phaseTime = 0.0
    end
end

local function captureResponse(
    sign,
    isFirst
)
    local speed,
        forward,
        side,
        up =
        readLocalKinematics()

    peakObservedSpeed =
        math.max(
            peakObservedSpeed,
            speed
        )

    if speed >
       CONFIG.maxSpeedMps then
        abortTest(3)
        return
    end

    captureFrame =
        captureFrame + 1

    if captureFrame == 1
    or captureFrame == 3
    or captureFrame == 10 then
        currentForward[captureFrame] =
            forward - preForward
        currentSide[captureFrame] =
            side - preSide
        currentUp[captureFrame] =
            up - preUp
    end

    if captureFrame >= 10 then
        local target =
            sign > 0 and plus or minus

        for _, sampleFrame in ipairs({
            1,
            3,
            10,
        }) do
            target.forward[sampleFrame] =
                currentForward[
                    sampleFrame
                ]
            target.side[sampleFrame] =
                currentSide[
                    sampleFrame
                ]
            target.up[sampleFrame] =
                currentUp[
                    sampleFrame
                ]
        end

        if isFirst then
            phase = PHASE.BETWEEN_SIGNS
        else
            phase = PHASE.PAIR_EVAL
        end

        phaseTime = 0.0
    end
end

local function evaluatePair()
    local symF = {}
    local symSide = {}
    local symUp = {}
    local commonF = {}

    for _, sampleFrame in ipairs({
        1,
        3,
        10,
    }) do
        symF[sampleFrame] =
            (
                num(
                    plus.forward[
                        sampleFrame
                    ],
                    0
                ) -
                num(
                    minus.forward[
                        sampleFrame
                    ],
                    0
                )
            ) / 2.0

        symSide[sampleFrame] =
            (
                num(
                    plus.side[
                        sampleFrame
                    ],
                    0
                ) -
                num(
                    minus.side[
                        sampleFrame
                    ],
                    0
                )
            ) / 2.0

        symUp[sampleFrame] =
            (
                num(
                    plus.up[
                        sampleFrame
                    ],
                    0
                ) -
                num(
                    minus.up[
                        sampleFrame
                    ],
                    0
                )
            ) / 2.0

        commonF[sampleFrame] =
            (
                num(
                    plus.forward[
                        sampleFrame
                    ],
                    0
                ) +
                num(
                    minus.forward[
                        sampleFrame
                    ],
                    0
                )
            ) / 2.0
    end

    latestSymF1 = symF[1]
    latestSymF3 = symF[3]
    latestSymF10 = symF[10]
    latestCommonF1 = commonF[1]
    latestSideLeakF1 = symSide[1]
    latestUpLeakF1 = symUp[1]

    latestPolarityPass =
        latestSymF1 >
            CONFIG.polarityThresholdMps
        and
        (
            plus.forward[1] -
            minus.forward[1]
        ) >
            CONFIG.polarityThresholdMps * 2.0

    totalPairs = totalPairs + 1

    if latestPolarityPass then
        polarityPassCount =
            polarityPassCount + 1
    end

    responseSum[durationIndex] =
        responseSum[durationIndex] +
        latestSymF1

    sideLeakSum[durationIndex] =
        sideLeakSum[durationIndex] +
        math.abs(
            latestSideLeakF1
        )

    upLeakSum[durationIndex] =
        upLeakSum[durationIndex] +
        math.abs(
            latestUpLeakF1
        )

    durationCount[durationIndex] =
        durationCount[durationIndex] + 1

    if durationIndex == 4 then
        if positiveFirst then
            eightPositiveFirstSum =
                eightPositiveFirstSum +
                latestSymF1
            eightPositiveFirstCount =
                eightPositiveFirstCount + 1
        else
            eightNegativeFirstSum =
                eightNegativeFirstSum +
                latestSymF1
            eightNegativeFirstCount =
                eightNegativeFirstCount + 1
        end
    end

    local duration =
        CONFIG.durations[
            durationIndex
        ]

    local row = {
        duration = duration,
        positive_first =
            positiveFirst and 1 or 0,
        dt_mean =
            pairDtSum /
            math.max(
                pairDtCount,
                1
            ),

        plus_f1 =
            plus.forward[1] or 0,
        plus_f3 =
            plus.forward[3] or 0,
        plus_f10 =
            plus.forward[10] or 0,

        minus_f1 =
            minus.forward[1] or 0,
        minus_f3 =
            minus.forward[3] or 0,
        minus_f10 =
            minus.forward[10] or 0,

        sym_f1 = latestSymF1,
        sym_f3 = latestSymF3,
        sym_f10 = latestSymF10,
        common_f1 = latestCommonF1,
        side_leak_f1 =
            latestSideLeakF1,
        up_leak_f1 =
            latestUpLeakF1,
        polarity_pass =
            latestPolarityPass and
            1 or 0,
    }

    pairRows[pairIndex] = row

    pairDtSum = 0.0
    pairDtCount = 0

    phase = PHASE.BETWEEN_PAIRS
    phaseTime = 0.0
end

local function finalizeCalibration()
    local totalLeak = 0.0
    local totalResponse = 0.0

    for i = 1, 4 do
        local count =
            math.max(
                durationCount[i],
                1
            )

        meanResponse[i] =
            responseSum[i] /
            count

        totalLeak =
            totalLeak +
            sideLeakSum[i] /
                count +
            upLeakSum[i] /
                count

        totalResponse =
            totalResponse +
            math.abs(
                meanResponse[i]
            )
    end

    averageDt =
        forceDtSum /
        math.max(
            forceCallCount,
            1
        )

    monotonic =
        meanResponse[2] +
            CONFIG.monotonicToleranceMps >=
            meanResponse[1]
        and
        meanResponse[3] +
            CONFIG.monotonicToleranceMps >=
            meanResponse[2]
        and
        meanResponse[4] +
            CONFIG.monotonicToleranceMps >=
            meanResponse[3]

    local numerator = 0.0
    local denominator = 0.0

    for i = 1, 4 do
        local frames =
            CONFIG.durations[i]

        numerator =
            numerator +
            frames *
            meanResponse[i]

        denominator =
            denominator +
            frames * frames
    end

    local velocityPerFrame =
        numerator /
        math.max(
            denominator,
            0.000001
        )

    effectiveDt =
        CONFIG.carMass *
        velocityPerFrame /
        CONFIG.rawForce

    durationModelError =
        math.abs(
            effectiveDt -
            averageDt
        ) /
        math.max(
            averageDt,
            0.000001
        )

    local fittedError = 0.0
    local measuredScale = 0.0

    for i = 1, 4 do
        local predicted =
            velocityPerFrame *
            CONFIG.durations[i]

        fittedError =
            fittedError +
            math.abs(
                meanResponse[i] -
                predicted
            )

        measuredScale =
            measuredScale +
            math.abs(
                meanResponse[i]
            )
    end

    linearityError =
        fittedError /
        math.max(
            measuredScale,
            0.000001
        )

    leakageRatio =
        totalLeak /
        math.max(
            totalResponse,
            0.000001
        )

    ratio2 =
        meanResponse[2] /
        math.max(
            meanResponse[1],
            0.000001
        )

    ratio4 =
        meanResponse[3] /
        math.max(
            meanResponse[1],
            0.000001
        )

    ratio8 =
        meanResponse[4] /
        math.max(
            meanResponse[1],
            0.000001
        )

    local positiveFirstMean =
        eightPositiveFirstSum /
        math.max(
            eightPositiveFirstCount,
            1
        )

    local negativeFirstMean =
        eightNegativeFirstSum /
        math.max(
            eightNegativeFirstCount,
            1
        )

    orderBias =
        math.abs(
            positiveFirstMean -
            negativeFirstMean
        ) /
        math.max(
            math.abs(
                meanResponse[4]
            ),
            0.000001
        )

    local polarityRate =
        polarityPassCount /
        math.max(
            totalPairs,
            1
        )

    local responseDetected =
        meanResponse[4] >=
            0.000200
        and polarityRate >= 0.80

    if not responseDetected then
        classificationCode = 3
        calibrated = false
    elseif monotonic
    and durationModelError <= 0.45
    and linearityError <= 0.35 then
        classificationCode = 1
        calibrated = true
    else
        classificationCode = 2
        calibrated = false
    end

    completed = true
    phase = PHASE.COMPLETE
    phaseTime = 0.0
end

local function configurePairOrder()
    positiveFirst =
        repeatIndex % 2 == 1

    if positiveFirst then
        firstSign = 1
        secondSign = -1
    else
        firstSign = -1
        secondSign = 1
    end
end

local function advancePair()
    if repeatIndex <
       CONFIG.repeats then
        repeatIndex =
            repeatIndex + 1
        pairIndex =
            pairIndex + 1
        configurePairOrder()
        clearPairMeasurements()
        beginPrep(
            PHASE.FIRST_PREP,
            true
        )
        return
    end

    if durationIndex <
       #CONFIG.durations then
        durationIndex =
            durationIndex + 1
        repeatIndex = 1
        pairIndex =
            pairIndex + 1
        configurePairOrder()
        clearPairMeasurements()
        beginPrep(
            PHASE.FIRST_PREP,
            true
        )
        return
    end

    finalizeCalibration()
end

local function processCommand()
    local seq =
        math.floor(
            num(
                safeLoad(
                    K.commandSeq,
                    0
                ),
                0
            )
        )

    if seq == lastCommandSeq then
        return
    end

    lastCommandSeq = seq

    local command =
        math.floor(
            num(
                safeLoad(
                    K.command,
                    0
                ),
                0
            )
        )

    if command == 2 then
        if phase == PHASE.IDLE
        or phase == PHASE.COMPLETE
        or phase == PHASE.ABORTED then
            startSequence()
        else
            abortTest(6)
        end
    elseif command == 9 then
        if phase ~= PHASE.IDLE
        and phase ~= PHASE.COMPLETE
        and phase ~= PHASE.ABORTED then
            abortTest(5)
        end

        publish()

        if worker
        and type(
            worker.terminate
        ) == "function" then
            worker.terminate()
        end
    end
end

function script.update(dt)
    dt =
        math.min(
            math.max(
                num(dt, 0.003),
                0.0001
            ),
            0.010
        )

    frame = frame + 1
    dtSum = dtSum + dt

    updateHeartbeat(dt)
    processCommand()

    if phase == PHASE.INITIAL_SETTLE then
        updateSettle(
            dt,
            CONFIG.initialSettleSec,
            PHASE.FIRST_PREP,
            true
        )
    elseif phase == PHASE.FIRST_PREP then
        updateSettle(
            dt,
            CONFIG.prepSec,
            PHASE.FIRST_APPLY,
            false
        )
    elseif phase == PHASE.FIRST_APPLY then
        if activeApplyFrame == 0 then
            beginApply(
                firstSign,
                PHASE.FIRST_APPLY
            )
        end
        updateApply(
            dt,
            PHASE.FIRST_CAPTURE
        )
    elseif phase == PHASE.FIRST_CAPTURE then
        captureResponse(
            firstSign,
            true
        )
    elseif phase == PHASE.BETWEEN_SIGNS then
        updateSettle(
            dt,
            CONFIG.betweenSignsSec,
            PHASE.SECOND_PREP,
            true
        )
    elseif phase == PHASE.SECOND_PREP then
        updateSettle(
            dt,
            CONFIG.prepSec,
            PHASE.SECOND_APPLY,
            false
        )
    elseif phase == PHASE.SECOND_APPLY then
        if activeApplyFrame == 0 then
            beginApply(
                secondSign,
                PHASE.SECOND_APPLY
            )
        end
        updateApply(
            dt,
            PHASE.SECOND_CAPTURE
        )
    elseif phase == PHASE.SECOND_CAPTURE then
        captureResponse(
            secondSign,
            false
        )
    elseif phase == PHASE.PAIR_EVAL then
        evaluatePair()
    elseif phase == PHASE.BETWEEN_PAIRS then
        local speed =
            readLocalKinematics()

        peakObservedSpeed =
            math.max(
                peakObservedSpeed,
                speed
            )

        if speed >
           CONFIG.maxSpeedMps then
            abortTest(3)
        else
            phaseTime = phaseTime + dt

            if phaseTime >=
               CONFIG.betweenPairsSec then
                advancePair()
            end
        end
    end

    if frame % 4 == 0
    or phase == PHASE.COMPLETE
    or phase == PHASE.ABORTED then
        publish()
    end
end

publish()
