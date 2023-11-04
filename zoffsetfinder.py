import logging
import threading
import pins
import struct

from . import manual_probe
from .homing import Homing
from mcu import MCU, MCU_trsync, MCU_endstop
from clocksync import SecondarySync
import chelper, serialhdl, msgproto

class zoffsetfinder:
	
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
		

	def handle_ready(self):				
		pass

	def handle_connect(self):	
		#zconfig = config.getsection('stepper_z')
		#homeconfig = config.getsection('safe_z_home')
		# store original safe z home offset
		#self.original_home = homeconfig.getfloatlist('home_xy_position',2)					

		self.homing = self.printer.lookup_object("homing")
		self.toolhead = self.printer.lookup_object('toolhead')

	def completed_probing(self):				
		# resuming from probeNozzleOffset 
		kin = self.toolhead.get_kinematics()
		rails = kin.rails
		# return endstops
		for rail in rails:
			if rail.get_steppers()[0].get_name() == "stepper_z":
				for r in range(len(rail.get_steppers())):
					# change it to original
					rail.endstops[r] = self.original_endstops[r]
					mcu_endstop = rail.endstops[r][0]
					stepper = rail.get_steppers()[r]
					self.remove_stepper_trsync(stepper,self.nozzleEndstopPin)
					self.add_stepper_trsync(stepper,mcu_endstop)
		self.register_endstop(self.nozzleEndstopPin,None,'N')
		
		self.nozzlehoming = False
		

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
			
			


	def probeNozzleOffset(self,gcmd):		
		# https://github.com/Klipper3d/klipper/blob/9e765daeedb2adf7641b96882326b80aeeb70c93/klippy/stepper.py#L296
		if self.z_homing is None:
			raise gcmd.error("Must home axes first")

		self.nozzlehoming = True
		self.original_endstops = []
		#modules = self.printer.lookup_objects(None) 
		#for m in modules:
		#	self.gcode.respond_info(" Modules : " + m[0])
		
		kin = self.toolhead.get_kinematics()
		rails = kin.rails
		railToHome = None
		for rail in rails:			
			if rail.get_steppers()[0].get_name() == "stepper_z":	
				railToHome = rail			
				for r in range(len(rail.get_steppers())):
					
					stepper = rail.get_steppers()[r]
					#self.gcode.respond_info("Adding steppers to endstop"+stepper.get_name())					
					self.add_stepper_trsync(stepper,self.nozzleEndstopPin)
					self.nozzleEndstopPin.add_stepper(stepper)
				#self.register_endstop(rail,stepper,nozzleEndstopPin)
				
				for r in range(len(rail.get_steppers())):
					mcu_endstop = rail.endstops[r][0] # rail.get_endstops()[r][0]	
					existingoid = mcu_endstop._oid		
					existingpin = rail.endstops[r][1]		
					stepper = rail.get_steppers()[r]
					self.original_endstops.append( rail.endstops[r] )
					# remove pin from stepper
					self.remove_stepper_trsync(stepper,mcu_endstop)
					
					# change it to our endstop
					rail.endstops[r] = (self.nozzleEndstopPin,self.nozzle_pin_name)	
					self.gcode.respond_info("Existing Endstop {"+str(existingoid )+"} "+existingpin)				
					self.gcode.respond_info("Nozzle Endstop {"+str(self.nozzleEndstopPin._oid )+"} "+self.nozzle_pin_name)				

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

		'''
		pre_context = self.pre_template.create_template_context()
		pre_context['params'] = gcmd.get_command_parameters()
		try:
			self.pre_template.run_gcode_from_command(pre_context)
		except:
			pass
		'''
		#self.completed_probing()
		

		

	

	def probeNozzleOffset2(self,gcmd):
		#check if homed
		if self.z_homing is None:
			raise gcmd.error("Must home axes first")

		
			
		# store original stepper motor endstops		
		self.toolhead = self.printer.lookup_object('toolhead')
		kin = self.toolhead.get_kinematics()
		original_endstops = []
		for stepper in kin.get_steppers():
			if stepper.is_active_axis('z'):
				stepperconfig = self.config.getsection(stepper.get_name())
				endstopP = stepperconfig.get("endstop_pin")
				self.gcode.respond_info(" steppers found %s endstop %s" % (stepper.get_name(),endstopP) )
				
				self.steppers.append(stepper)
				original_endstops.append(endstopP)

		
		# change safe z home offset to start_position
		
		#self.configfile.set('safe_z_home','home_xy_position', " , ".join(str(self.startPos)))

		# change endstops for each Z steppers
		for i in range(len(original_endstops)):
			self.configfile.set(self.steppers[i].get_name(),"endstop_pin", self.endstop_pin)


		# begin homing Z
		self.gcode.respond_info("Beginning Nozzle Homing ")
		currentPos = self.toolhead.get_position()
		self.gcode.respond_info("Curpos is " + str(currentPos)+" probe speed "+str(self.homingspeed))
		self.toolhead.move([currentPos[0],currentPos[1],self.startZpos,0], self.probing_speed) # height first
		self.toolhead.move([self.startPos[0],self.startPos[1],self.startZpos,0], self.homingspeed) # position in place

		pre_context = self.pre_template.create_template_context()
		pre_context['params'] = gcmd.get_command_parameters()
		try:
			self.pre_template.run_gcode_from_command(pre_context)
		except:
			pass
		

		# restore original stepper motor endstops
		for i in range(len(original_endstops)):
			self.configfile.set(self.steppers[i].get_name(),"endstop_pin", original_endstops[i])
		# restore original safe z home offset
		
		#self.configfile.set('safe_z_home','home_xy_position'," , ".join(str(self.original_home)))

		post_context = self.post_template.create_template_context()
		post_context['params'] = gcmd.get_command_parameters()
		try:
			self.post_template.run_gcode_from_command(post_context)
		except:
			pass
		self.gcode.respond_info("Nozzle Homing Completed ")

	
def load_config(config):
	return zoffsetfinder(config)

		
#configfile.set('stepper_z','position_endstop', "%.3f" % (offset,))		