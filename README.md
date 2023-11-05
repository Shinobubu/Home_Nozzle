# ZOffsetFinder (This doesn't work yet)

To be renamed Nozzle Offset finder. 
This is a simple Klipper plugin for using an Endstop switch to find the tip of the nozzle relative to the Z-endstop of a 3D printer. 
inspired by the Z Calibration project. This is intended for people who still uses inductive bed leveling probes and not a microswitch.

https://www.printables.com/model/633611-optical-endstop-switch
![IMG_6104](https://github.com/Shinobubu/ZOffsetFinder/assets/14949931/46fd0f89-9997-4bda-bd13-b399e8629cc1)

Proof of concept. but times out and freezes near the end waiting for a lift event. 
https://www.youtube.com/watch?v=kTixtqg_B9k

Traceback (most recent call last):
  File "/home/xyriz/klipper/klippy/webhooks.py", line 256, in _process_request
    func(web_request)
  File "/home/xyriz/klipper/klippy/webhooks.py", line 436, in _handle_script
    self.gcode.run_script(web_request.get_str('script'))
  File "/home/xyriz/klipper/klippy/gcode.py", line 216, in run_script
    self._process_commands(script.split('\n'), need_ack=False)
  File "/home/xyriz/klipper/klippy/gcode.py", line 198, in _process_commands
    handler(gcmd)
  File "/home/xyriz/klipper/klippy/gcode.py", line 135, in <lambda>
    func = lambda params: origfunc(self._get_extended_params(params))
  File "/home/xyriz/klipper/klippy/extras/zoffsetfinder.py", line 206, in probeNozzleOffset
    kin.home(homing_state)
  File "/home/xyriz/klipper/klippy/kinematics/cartesian.py", line 91, in home
    self.home_axis(homing_state, axis, self.rails[axis])
  File "/home/xyriz/klipper/klippy/kinematics/cartesian.py", line 84, in home_axis
    homing_state.home_rails([rail], forcepos, homepos)
  File "/home/xyriz/klipper/klippy/extras/homing.py", line 185, in home_rails
    hmove.homing_move(homepos, hi.speed)
  File "/home/xyriz/klipper/klippy/extras/homing.py", line 101, in homing_move
    trigger_time = mcu_endstop.home_wait(move_end_print_time)
  File "/home/xyriz/klipper/klippy/mcu.py", line 305, in home_wait
    res = [trsync.stop() for trsync in self._trsyncs]
  File "/home/xyriz/klipper/klippy/mcu.py", line 305, in <listcomp>
    res = [trsync.stop() for trsync in self._trsyncs]
  File "/home/xyriz/klipper/klippy/mcu.py", line 210, in stop
    self._mcu.register_response(None, "trsync_state", self._oid)
  File "/home/xyriz/klipper/klippy/mcu.py", line 871, in register_response
    self._serial.register_response(cb, msg, oid)
  File "/home/xyriz/klipper/klippy/serialhdl.py", line 237, in register_response
    del self.handlers[name, oid]
KeyError: ('trsync_state', 1)
