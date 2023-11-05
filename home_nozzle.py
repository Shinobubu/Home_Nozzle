import logging
import threading
import pins
import struct

from . import manual_probe
from .homing import Homing
from mcu import MCU, MCU_trsync, MCU_endstop
from clocksync import SecondarySync
import chelper, serialhdl, msgproto

class home_nozzle:
	
	cmd_HELP = "Simply home it hommie."

	def __init__(self,config):
		self.nozzlehoming = False
		self.config = config
		self.printer = config.get_printer()		
		self.steppers = []
		
		self.configfile = self.printer.lookup_object('configfile')		
		self.gcode = self.printer.lookup_object('gcode')
		gcode_macro = self.printer.load_object(config, 'gcode_macro')
		self.pre_template = gcode_macro.load_template(config, 'pre_gcode')
		self.post_template = gcode_macro.load_template(config, 'post_gcode')
		self.endstop_pin = config.get('endstop_pin')	
		self.fast_feedrate = config.getfloat('fast_feedrate')
		self.slow_feedrate = config.getfloat('slow_feedrate')
		x_pos,y_pos,z_pos = config.getfloatlist('start_position',3)
		self.startZpos = z_pos
		self.startPos = [x_pos,y_pos,z_pos]
		self.nozzle_hop = config.getfloat('nozzle_hop')		
		self.homingspeed = self.fast_feedrate #homeconfig.getfloat('speed')
		self.probing_speed = None # 
		self.z_homing = None
		self.second_speed = None
		self.retract_dist = None
		self.position_min = None
		max_lower_distance = config.getfloat('max_lower_distance')
		self.gcode.register_command("HOME_NOZZLE",self.probeNozzleOffset,self.cmd_HELP)
		self.printer.register_event_handler("homing:home_rails_end",self.handle_home_rails_end)
		self.printer.register_event_handler("klippy:connect", self.handle_connect)
		self.printer.register_event_handler('klippy:ready',self.handle_ready)
		
		ppins = self.printer.lookup_object('pins')		
		pin_params = ppins.parse_pin(self.endstop_pin, True, True)		
		self.nozzleEndstopPin =  ppins.setup_pin('endstop', self.endstop_pin)		
		# Normalize pin name
		self.nozzle_pin_name = "%s:%s" % (pin_params['chip_name'], pin_params['pin'])
		self.register_endstop(self.nozzleEndstopPin,None,'N')		

		#add event for aborted homing.
		
	#Not Implimented
	def handle_ready(self):				
		pass

	def handle_connect(self):	
		#zconfig = config.getsection('stepper_z')
		#homeconfig = config.getsection('safe_z_home')
		# store original safe z home offset
		#self.original_home = homeconfig.getfloatlist('home_xy_position',2)					
		self.homing = self.printer.lookup_object("homing")
		self.toolhead = self.printer.lookup_object('toolhead')
		self.initialize_nozzle_pin_steppers()

	def completed_probing(self):				
		# resuming from probeNozzleOffset 
		kin = self.toolhead.get_kinematics()
		rails = kin.rails
		# return endstops
		for rail in rails:
			if rail.get_steppers()[0].get_name() == "stepper_z":
				for r in range(len(rail.get_steppers())):
					#remove endstops from nozzlepin											
					stepper = rail.get_steppers()[r]
					self.remove_stepper_trsync(stepper,self.nozzleEndstopPin)					
				
				# change it to original
				rail.endstops = self.original_rail_enstops
		self.register_endstop(self.nozzleEndstopPin,None,'N')
		
		
		self.nozzlehoming = False
		
	#Not Implimented
	def aborted_home_rails(self,homing_state,rails):		
		if self.nozzlehoming:
			self.completed_probing()
			self.gcode.respond_info("Nozzle Homing Aborted")


	
	def handle_home_rails_end(self, homing_state, rails):
		self.gcode.respond_info("end stop MCU status "+str(type(self.nozzleEndstopPin._mcu))+" and "+str(type(self.nozzleEndstopPin._trdispatch)))
		if self.nozzlehoming:
			self.completed_probing()
			self.gcode.respond_info("Nozzle Homing Completed")
		else:
			for rail in rails:
				if rail.get_steppers()[0].is_active_axis('z'):
					mcu_endstop = rail.get_endstops()[0][0]
					self.gcode.respond_info(" Stepper homing name " + str(rail.get_steppers()[0].get_name() +"   "+ mcu_endstop._pin ))
					# get homing settings from z rail
					self.z_homing = rail.position_endstop
					if self.probing_speed is None:
						self.probing_speed = rail.homing_speed
					if self.second_speed is None:
						self.second_speed = rail.second_homing_speed
					if self.retract_dist is None:
						self.retract_dist = rail.homing_retract_dist
					if self.position_min is None:
						self.position_min = rail.position_min
					self.position_z_endstop = rail.position_endstop

	def register_endstop(self,mcu_endstop,stepper=None,name=None):
		if name == None:
			name = stepper.get_name(short=True)				
		query_endstops = self.printer.load_object(self.config, 'query_endstops')
		query_endstops.register_endstop(mcu_endstop, name)
		


	def unregister_endstop(self,mcu_endstop,stepper=None,name=None):
		if name == None:
			name = stepper.get_name(short=True)				
		query_endstops = self.printer.load_object(self.config, 'query_endstops')
		for i in range(len(query_endstops.endstops)):
			if(query_endstops.endstops[i][0] == mcu_endtop):
				self.gcode.respond_info("Found old MCU in query_endstops removing temporarily")
				query_endstops.endstops.pop(i)
				return True
		return False
	
	def clear_stepper_trsync(self,mcu_endstop):
		mcu_endstop._trsyncs = [mcu_endstop._trsyncs[0]] 		

	def remove_stepper_trsync(self,stepper,mcu_endstop):
		#https://github.com/Klipper3d/klipper/blob/9e765daeedb2adf7641b96882326b80aeeb70c93/klippy/mcu.py#L223
		trsyncs = {trsync.get_mcu(): trsync for trsync in mcu_endstop._trsyncs}
		trsync = trsyncs.get(stepper.get_mcu())
		if stepper in trsync._steppers:			
			trsync._steppers.remove(stepper)

	def add_stepper_trsync(self,stepper,mcu_endstop):
		#https://github.com/Klipper3d/klipper/blob/9e765daeedb2adf7641b96882326b80aeeb70c93/klippy/mcu.py#L223
		mcu_endstop.add_stepper(stepper)
		trsyncs = {trsync.get_mcu(): trsync for trsync in mcu_endstop._trsyncs}
		trsync = trsyncs.get(stepper.get_mcu())
		if stepper in trsync._steppers:
			trysnc = trsync._steppers[ trsync._steppers.index(stepper) ]
			#debugdata = trysnc._mcu._serial.dump_debug()			
			# figure out TRSYNC issues
			#self.gcode.respond_info(str(trysnc._mcu._serial.handlers))			
			
	def initialize_nozzle_pin_steppers(self):	
		self.original_rail_enstops = []
		kin = self.toolhead.get_kinematics()
		rails = kin.rails
		self.railToHome = None
		# clear endstop_pin steppers
		self.clear_stepper_trsync(self.nozzleEndstopPin)

		for rail in rails:			
			if rail.get_steppers()[0].get_name() == "stepper_z":	
				self.railToHome = rail			
				#Register stepper motor to our Nozzle Endstop
				for r in range(len(rail.get_steppers())):					
					stepper = rail.get_steppers()[r]					
					self.nozzleEndstopPin.add_stepper(stepper)	

				self.original_rail_enstops = rail.endstops


	def probeNozzleOffset(self,gcmd):		
		# https://github.com/Klipper3d/klipper/blob/9e765daeedb2adf7641b96882326b80aeeb70c93/klippy/stepper.py#L296
		
		pre_context = self.pre_template.create_template_context()
		pre_context['params'] = gcmd.get_command_parameters()
		try:
			self.pre_template.run_gcode_from_command(pre_context)
		except:
			pass
		self.toolhead.wait_moves()

		kin = self.toolhead.get_kinematics()
		if self.z_homing is None:
			raise gcmd.error("Must home axes first")

		self.nozzlehoming = True
		
		# swap rail endstops
		self.railToHome.endstops = [(self.nozzleEndstopPin,self.nozzle_pin_name)]
		
		# Move nozzle to position.			
		self.gcode.respond_info("Beginning Nozzle Homing ")
		currentPos = self.toolhead.get_position()
		self.gcode.respond_info("Curpos is " + str(currentPos)+" probe speed "+str(self.homingspeed))
		self.toolhead.manual_move([currentPos[0],currentPos[1],self.startZpos,0], self.probing_speed) # height first
		self.toolhead.manual_move([self.startPos[0],self.startPos[1],self.startZpos,0], self.homingspeed) # position in place
		self.toolhead.wait_moves()
		self.gcode.respond_info("Moved in position ")
		axes = [2]
		homing_state = Homing(self.printer)
		homing_state.set_axes(axes)
		kin.home(homing_state)

		
		
		self.completed_probing()
		post_context = self.post_template.create_template_context()
		post_context['params'] = gcmd.get_command_parameters()
		try:
			self.post_template.run_gcode_from_command(post_context)
		except:
			pass				
	
def load_config(config):
	return home_nozzle(config)

