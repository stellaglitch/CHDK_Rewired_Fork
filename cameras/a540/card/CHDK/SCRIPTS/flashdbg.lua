--[[
@title Flash crash logger
@chdk_version 1.7
@param a Sample every (ms)
@default a 100
@range a 20 1000
@param b Flush to card every (ms)
@default b 2000
@range b 200 10000
@param c Leave ALT mode so you can shoot
@default c 1
@range c 0 1
]]

-- Logs camera state to A/CHDK/LOGS/FLASHDBG.LOG while you shoot by hand, so
-- the last lines on the card show what the camera was doing when it died.
--
-- How to use:
--   1. Enter <ALT>, Scripts, load flashdbg.lua, press the shutter to start.
--   2. With "Leave ALT mode" = 1 the script drops out of <ALT> and keeps
--      running in the background; shoot normally, with the flash on.
--   3. After the crash, pull the battery, put the card in a computer and read
--      A/CHDK/LOGS/FLASHDBG.LOG. Each session starts with "=== START"; a
--      session that has no "=== END" line is the one that crashed.
--   4. To stop it cleanly: press <ALT>, then the shutter.
--
-- What to look at:
--   lag   ms between two samples, minus the sample period. It is the script
--         task being starved. A big lag while full=1 means a higher priority
--         task is eating the CPU while the shutter is held.
--   rdy   flash ready (1 charged / 0 charging). Crashes while rdy=0 and
--         full=1 point at the strobe recharge.
--   vbat  battery mV. A drop toward ~2100 during charging is a brown-out.
--   mfree largest free heap block. Shrinking every shot is a leak.
--
-- Writing is buffered and flushed with open/append/close, never kept open, so
-- a crash loses at most the last flush period. Flushing is held off while a
-- shot is in progress, so the logger itself does not write to the card while
-- Canon is writing the JPEG.

local LOG = "A/CHDK/LOGS/FLASHDBG.LOG"
local SAMPLE_MS = a
local FLUSH_MS  = b

-- propset 1 (A540)
local PC_FLASH_MODE   = 16    -- 0 auto 1 on 2 off
local PC_FLASH_FIRE   = 79    -- 1 if this shot fires the flash
local PC_FLASH_ADJ    = 15
local PC_SHOOTING     = 205
local PC_FLASH_READY  = 221

-- CHDK config ids as get_config_value sees them (see bendshot.lua)
local CFG_BEND_ENABLE  = 1280
local CFG_BENDX_ENABLE = 1290
local CFG_BEND_SAVE    = 1293

local buf = {}
local nbuf = 0

local function add(line)
    nbuf = nbuf + 1
    buf[nbuf] = line
end

local function flush()
    if nbuf == 0 then return end
    local f = io.open(LOG, "a")
    if f then
        for i = 1, nbuf do f:write(buf[i], "\n") end
        f:close()
    end
    buf = {}
    nbuf = 0
end

local function b2i(v)
    if v == true then return 1 end
    if v == false or v == nil then return 0 end
    return v
end

local function cfg(id)
    local v = get_config_value(id)
    if type(v) == "number" then return v end
    return -1
end

local function mem()
    local m = get_meminfo()
    if m then return (m.free_block_max_size or -1), (m.free_size or -1) end
    return -1, -1
end

-- session header -----------------------------------------------------------
local bi = get_buildinfo() or {}
local mf, ms = mem()
add(string.format("=== START tick=%d build=%s %s %s fw=%s",
    get_tick_count(), tostring(bi.build_number), tostring(bi.build_date),
    tostring(bi.build_time), tostring(bi.platsub)))
add(string.format("cfg bend=%d bendx=%d bend_save=%d mode=%d exp=%d mfree=%d mtotal=%d",
    cfg(CFG_BEND_ENABLE), cfg(CFG_BENDX_ENABLE), cfg(CFG_BEND_SAVE),
    get_mode() and 1 or 0, get_exp_count(), mf, ms))
add("tick lag shoot half full fmode fire fadj rdy shot# vbat mfree mtot t_opt t_ccd t_bat tv96 av96 sv96 note")
flush()

if c == 1 then exit_alt() end

-- sampling loop ------------------------------------------------------------
local last_tick  = get_tick_count()
local last_flush = last_tick
local prev = {}
local worst_lag = 0

local function sample(note)
    local now   = get_tick_count()
    local lag   = now - last_tick - SAMPLE_MS
    if lag < 0 then lag = 0 end
    if lag > worst_lag then worst_lag = lag end
    last_tick = now

    local shoot = b2i(get_shooting())
    local half  = b2i(is_pressed("shoot_half"))
    local full  = b2i(is_pressed("shoot_full"))
    local fmode = get_prop(PC_FLASH_MODE)
    local fire  = get_prop(PC_FLASH_FIRE)
    local fadj  = get_prop(PC_FLASH_ADJ)
    local rdy   = b2i(get_flash_ready())
    local shot  = get_exp_count()
    local vbat  = get_vbatt()
    local mfree, mtot = mem()

    -- Only log every sample when something interesting is happening; idle
    -- samples are logged once a second so the file stays readable.
    local busy = (shoot == 1) or (half == 1) or (full == 1) or (rdy == 0) or (lag > 50)
    local changed = (shoot ~= prev.shoot) or (full ~= prev.full) or (half ~= prev.half)
                 or (rdy ~= prev.rdy) or (shot ~= prev.shot) or (fire ~= prev.fire)
    if busy or changed or note or (now - (prev.logged or 0)) >= 1000 then
        add(string.format("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %s",
            now, lag, shoot, half, full, fmode, fire, fadj, rdy, shot, vbat,
            mfree, mtot, get_temperature(0), get_temperature(1), get_temperature(2),
            get_tv96(), get_av96(), get_sv96(), note or ""))
        prev.logged = now
    end

    if shot ~= prev.shot and prev.shot ~= nil then
        add(string.format("# shot %d done, worst lag so far %d ms, flash fire=%d", shot, worst_lag, fire))
    end

    prev.shoot, prev.full, prev.half, prev.rdy, prev.shot, prev.fire =
        shoot, full, half, rdy, shot, fire

    -- Never touch the card while Canon may be exposing or writing a frame.
    if shoot == 0 and full == 0 and half == 0 and (now - last_flush) >= FLUSH_MS then
        flush()
        last_flush = now
    end
end

-- Runs until the user stops the script (<ALT> then shutter). restore() below
-- writes the END marker that tells a clean stop from a crash.
function restore()
    add(string.format("=== END tick=%d worst_lag=%d", get_tick_count(), worst_lag))
    flush()
end

while true do
    sample(nil)
    sleep(SAMPLE_MS)
end
