# Fly ArduPlane in SITL
from __future__ import print_function
import math
import os
import shutil

import pexpect
from pymavlink import mavutil

from common import *
from pysim import util



HOME_LOCATION = '-35.362938,149.165085,585,354'
WIND = "0,180,0.2"  # speed,direction,variance

homeloc = None

options = '--sitl=127.0.0.1:5501 --out=127.0.0.1:19550 --streamrate=10'
if viewerip:
options += " --out=%s:14550" % viewerip
if use_map:
options += ' --map'

sitl = util.start_SITL(binary, model='plane-elevrev', home=HOME_LOCATION, speedup=10,
                          valgrind=valgrind, gdb=gdb,
                          defaults_file=os.path.join(testdir, 'default_params/plane-jsbsim.parm'))
mavproxy = util.start_MAVProxy_SITL('ArduPlane', options=options)
mavproxy.expect('Telemetry log: (\S+)')
logfile = mavproxy.match.group(1)
print("LOGFILE %s" % logfile)

buildlog = util.reltopdir("../buildlogs/ArduPlane-test.tlog")
print("buildlog=%s" % buildlog)
if os.path.exists(buildlog):
os.unlink(buildlog)
try:
os.link(logfile, buildlog)
except Exception:
pass

util.expect_setup_callback(mavproxy, expect_callback)

mavproxy.expect('Received [0-9]+ parameters')

expect_list_clear()
expect_list_extend([sitl, mavproxy])

print("Started simulator")

# get a mavlink connection going
try:
mav = mavutil.mavlink_connection('127.0.0.1:19550', robust_parsing=True)
except Exception as msg:
print("Failed to start mavlink connection on 127.0.0.1:19550" % msg)
raise
mav.message_hooks.append(message_hook)
mav.idle_hooks.append(idle_hook)

failed = False
fail_list = []
e = 'None'
try:
print("Waiting for a heartbeat with mavlink protocol %s" % mav.WIRE_PROTOCOL_VERSION)
mav.wait_heartbeat()
print("Setting up RC parameters")
setup_rc(mavproxy)
print("Waiting for GPS fix")
mav.recv_match(condition='VFR_HUD.alt>10', blocking=True)
mav.wait_gps_fix()
while mav.location().alt < 10:
    mav.wait_gps_fix()
homeloc = mav.location()
print("Home location: %s" % homeloc)
if not takeoff(mavproxy, mav):
    print("Failed takeoff")
    failed = True
    fail_list.append("takeoff")
if not fly_left_circuit(mavproxy, mav):
    print("Failed left circuit")
    failed = True
    fail_list.append("left_circuit")
if not axial_left_roll(mavproxy, mav, 1):
    print("Failed left roll")
    failed = True
    fail_list.append("left_roll")
if not inside_loop(mavproxy, mav):
    print("Failed inside loop")
    failed = True
    fail_list.append("inside_loop")
if not test_stabilize(mavproxy, mav):
    print("Failed stabilize test")
    failed = True
    fail_list.append("stabilize")
if not test_acro(mavproxy, mav):
    print("Failed ACRO test")
    failed = True
    fail_list.append("acro")
if not test_FBWB(mavproxy, mav):
    print("Failed FBWB test")
    failed = True
    fail_list.append("fbwb")
if not test_FBWB(mavproxy, mav, mode='CRUISE'):
    print("Failed CRUISE test")
    failed = True
    fail_list.append("cruise")
if not fly_RTL(mavproxy, mav):
    print("Failed RTL")
    failed = True
    fail_list.append("RTL")
if not fly_LOITER(mavproxy, mav):
    print("Failed LOITER")
    failed = True
    fail_list.append("LOITER")
if not fly_CIRCLE(mavproxy, mav):
    print("Failed CIRCLE")
    failed = True
    fail_list.append("LOITER")
if not fly_mission(mavproxy, mav, os.path.join(testdir, "ap1.txt"), height_accuracy = 10,
                   target_altitude=homeloc.alt+100):
    print("Failed mission")
    failed = True
    fail_list.append("mission")
if not log_download(mavproxy, mav, util.reltopdir("../buildlogs/ArduPlane-log.bin")):
    print("Failed log download")
    failed = True
    fail_list.append("log_download")
except pexpect.TIMEOUT as e:
print("Failed with timeout")
failed = True
fail_list.append("timeout")

mav.close()
util.pexpect_close(mavproxy)
util.pexpect_close(sitl)

valgrind_log = util.valgrind_log_filepath(binary=binary, model='plane-elevrev')
if os.path.exists(valgrind_log):
os.chmod(valgrind_log, 0o644)
shutil.copy(valgrind_log, util.reltopdir("../buildlogs/ArduPlane-valgrind.log"))
