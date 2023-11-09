import logging
import threading
import pins
import struct

from . import manual_probe
from .homing import Homing
from mcu import MCU, MCU_trsync, MCU_endstop



class home_nozzle:
	
	cmd_HELP = "Home Z Axis and then run home_nozzle."

	def __init__(self,config):
		self.nozzlehoming = False
		self.config = config
		self.printer = config.get_printer()				
		self.configfile = self.printer.lookup_object('configfile')		
		self.gcode = self.printer.lookup_object('gcode')
		gcode_macro = self.printer.load_object(config, 'gcode_macro')
		self.pre_template = gcode_macro.load_template(config, 'pre_gcode')
		self.post_template = gcode_macro.load_template(config, 'post_gcode')
		self.endstop_pin = config.get('endstop_pin')	
		self.position_feedrate = config.getfloat('position_feedrate')
		x_pos,y_pos,z_pos = config.getfloatlist('start_position',3)
		self.startZpos = z_pos
		self.startPos = [x_pos,y_pos,z_pos]				
		self.safeHomePosition = [None,None]	
		self.safeHomeHop = None
		self.endstopName = "Nozzle"
		self.gcode.register_command("HOME_NOZZLE",self.probeNozzleOffset,self.cmd_HELP)
		self.printer.register_event_handler("homing:home_rails_end",self.handle_home_rails_end)
		self.printer.register_event_handler("klippy:connect", self.handle_connect)
		
		self.z_homing = None

		ppins = self.printer.lookup_object('pins')		
		pin_params = ppins.parse_pin(self.endstop_pin, True, True)		
		self.nozzleEndstopPin =  ppins.setup_pin('endstop', self.endstop_pin)		
		# Normalize pin name
		self.nozzle_pin_name = "%s:%s" % (pin_params['chip_name'], pin_params['pin'])
		self.register_endstop(self.nozzleEndstopPin,None,self.endstopName)		
		#add event for aborted homing.
	

	def handle_connect(self):			
		self.homing = self.printer.lookup_object("homing")
		self.toolhead = self.printer.lookup_object('toolhead')
		try:
			self.safezhome = self.printer.lookup_object("safe_z_home")
			self.safeHomePosition = [self.safezhome.home_x_pos, self.safezhome.home_y_pos]
			self.safeHomeHop = self.safezhome.z_hop
			print(" Safe Home Found %s %s" % (self.safezhome.home_x_pos, self.safezhome.home_y_pos))						
		except:
			self.gcode.respond_info("No Safe Zone Detected")

		self.initialize_nozzle_pin_steppers()

	def completed_probing(self):				
		# resuming from probeNozzleOffset 
		kin = self.toolhead.get_kinematics()
		rails = kin.rails
		# return endstops
		for rail in rails:
			if rail.get_steppers()[0].get_name() == "stepper_z":				
				rail.endstops = self.original_rail_enstops

		if 	self.safeHomeHop != None:
			self.safezhome.home_x_pos = self.safeHomePosition[0]
			self.safezhome.home_y_pos = self.safeHomePosition[1]
			self.safezhome.z_hop = self.safezhome.z_hop			

		self.nozzlehoming = False
		
	#Not Implimented
	def aborted_home_rails(self,homing_state,rails):		
		if self.nozzlehoming:
			self.completed_probing()
			self.gcode.respond_info("Nozzle Homing Aborted")


	
	def handle_home_rails_end(self, homing_state, rails):
		
		if self.nozzlehoming:
			self.completed_probing()
			self.gcode.respond_info("Nozzle Homing Completed")

			post_context = self.post_template.create_template_context()			
			try:
				self.post_template.run_gcode_from_command(post_context)
			except:
				pass				
			
		else:
			for rail in rails:
				if rail.get_steppers()[0].is_active_axis('z'):					
					# get homing settings from z rail
					self.z_homing = rail.position_endstop		
					# now perform nozzle homing after Z is homed
					if self.nozzlehoming == False:						
						self.probeNozzleOffset()			

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
			if(query_endstops.endstops[i][0] == mcu_endstop):
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


	def probeNozzleOffset(self,gcmd=None):		
		# https://github.com/Klipper3d/klipper/blob/9e765daeedb2adf7641b96882326b80aeeb70c93/klippy/stepper.py#L296
		#self.gcmd = gcmd
		pre_context = self.pre_template.create_template_context()
		#pre_context['params'] = gcmd.get_command_parameters()
		try:
			self.pre_template.run_gcode_from_command(pre_context)
		except:
			pass
		self.toolhead.wait_moves()

		kin = self.toolhead.get_kinematics()
		if self.z_homing is None:
			self.gcode.respond_info("Must home Z axes first")
			return False
			#raise gcmd.error("Must home Z axes first")

		self.nozzlehoming = True
		if self.safeHomeHop != None:
			self.safezhome.home_x_pos = self.startPos[0]
			self.safezhome.home_y_pos = self.startPos[1]
			self.safezhome.z_hop = self.startZpos		
		# swap rail endstops
		self.railToHome.endstops = [(self.nozzleEndstopPin,self.nozzle_pin_name)]
		
		# Move nozzle to position.			
		
		currentPos = self.toolhead.get_position()		
		self.toolhead.manual_move([currentPos[0],currentPos[1],self.startZpos,0], self.position_feedrate) # height first
		self.toolhead.manual_move([self.startPos[0],self.startPos[1],self.startZpos,0], self.position_feedrate) # position in place
		self.toolhead.wait_moves()
		self.gcode.respond_info("Beginning Nozzle Homing ")
		axes = [2]
		homing_state = Homing(self.printer)
		homing_state.set_axes(axes)
		kin.home(homing_state)
		return True

						
		
		
	
def load_config(config):
	return home_nozzle(config)

